"""Synthetic boundary tests; the full-sequence acceptance is a separate run."""
import importlib
import importlib.util
import math
import pytest


def module():
    name = 'rm27.perception.vision.localization.experiments.openvins_runtime'
    assert importlib.util.find_spec(name), 'runtime adapter missing'
    return importlib.import_module(name)


def test_velocity_frame_conversion():
    m = module()
    assert m.world_velocity([0,0,math.sqrt(.5),math.sqrt(.5)], [1,0,0]) == pytest.approx([0,1,0])


def test_wire_pose_convention_unknowns():
    p = module().wire_pose([1,2,3], [0,0,0,1], 1403715273262142976)
    assert p['orientation_wxyz'] == [1,0,0,0]
    assert p['source_time']['raw_value'] == 1403715273262142976
    assert p['child_frame'] == 'imu' and p['translation_unit'] == 'm'
    assert 'tracking_state' not in p and 'publish_time' not in p


def test_order_is_per_stream_and_strict():
    s = module().StreamOrder()
    s.push('camera', 10)
    s.push('imu', 5)
    s.push('camera', 11)
    with pytest.raises(ValueError):
        s.push('camera', 11)
    with pytest.raises(ValueError):
        s.push('imu', 4)


@pytest.mark.parametrize('t', [1.2, True, -1])
def test_invalid_stamp(t):
    with pytest.raises(ValueError):
        module().StreamOrder().push('imu', t)


def test_state_text_integer_precision_and_world_velocity():
    state = module().state_row('1403715273.26214 0 0 0 1 1 2 3 .1 .2 .3 0 0 0 0 0 0 .0001000 1')
    assert state['state_ns'] == 1403715273262140000
    assert state['image_ns'] == 1403715273262040000
    assert state['velocity_world'] == [.1,.2,.3]


@pytest.mark.parametrize('q,v', [([0,0,0,2],[0,0,0]), ([0,0,0,1],[float('nan'),0,0])])
def test_invalid_velocity_or_quaternion(q,v):
    with pytest.raises(ValueError):
        module().world_velocity(q,v)


def test_same_run_metric_parity_gate():
    m = module()
    baseline = dict(ate={'rmse_m':.07},rpe_1s={'translation_rmse_m':.045,'rotation_rmse_deg':.478},
                    scale_diagnostic={'path_length_ratio_est_over_ref':.98})
    assert m.metric_consistency(baseline,baseline)['pass']
    bad = {**baseline,'ate':{'rmse_m':.2}}
    assert not m.metric_consistency(bad,baseline)['pass']


def test_runtime_dispatch_rejects_fixture():
    from rm27.perception.vision.localization.experiments.runner import run
    # Must reach the dedicated VIO validation, not old monocular schema checks.
    result = run(dict(schema_version=1,backend='OpenVINS',run_id='test',purpose='TEST_FIXTURE_ONLY'))
    assert result['failure_code'] == 'VIO_REAL_RUN_REQUIRED'


def test_wrapper_preserves_ros_python_environment():
    from pathlib import Path
    script = (Path(__file__).resolve().parents[1]/'tools/openvins/run_adapter_ros2.sh').read_text()
    assert 'export PYTHONPATH=/repo:${PYTHONPATH' in script


def test_existing_output_is_never_modified(tmp_path):
    p = tmp_path/'old-run'
    p.mkdir()
    marker = p/'run.json'
    marker.write_text('original evidence')
    with pytest.raises(FileExistsError):
        module().create_output(p)
    assert marker.read_text() == 'original evidence'


def test_new_output_has_raw_directory(tmp_path):
    p = module().create_output(tmp_path/'new-run')
    assert (p/'raw').is_dir()


def test_transport_scope_rejects_other_sequences(tmp_path):
    import json
    bag=tmp_path/'bag'
    bag.mkdir()
    (bag/'source_verification.json').write_text(json.dumps(dict(
        readback_exact=True,truth_included=False,source_sha256='0'*64)))
    with pytest.raises(ValueError,match='VIO_SEQUENCE_UNSUPPORTED'):
        module().EurocVioDataset(tmp_path/'missing.zip',bag)


def test_coverage_cannot_pass_on_just_similar_errors():
    base = dict(raw_pose_count=2800,evaluated_pose_count=2780,evaluated_duration_s=138.95,
        association={'first_camera_index':112,'last_camera_index':2911},
        alignment={'type':'SE3','scale':1.0},gt_csv_sha256='reference')
    assert module().coverage_consistency(base,base)['pass']
    assert not module().coverage_consistency({**base,'evaluated_pose_count':2000},base)['pass']
    assert not module().coverage_consistency({**base,'alignment':{'type':'Sim3','scale':1.01}},base)['pass']


def test_observer_buffers_imu_bursts_and_requires_odom_readiness():
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]/'tools/openvins'
    assert 'QoSProfile(depth=10000' in (root/'observe_ros2.py').read_text()
    assert 'ros2 topic info /odomimu' in (root/'run_adapter_ros2.sh').read_text()


@pytest.mark.parametrize('mode', ['mono', 'stereo'])
def test_comparison_observer_matches_all_reliable_inputs_before_ready(tmp_path, monkeypatch, mode):
    """Exercise the real observer setup and spin path without ROS or sensor input."""
    import builtins
    import json
    from pathlib import Path
    import sys
    from types import SimpleNamespace
    from tools.openvins import observe_comparison_ros2 as observer

    class Subscription:
        def __init__(self, topic, callback, qos):
            self.topic, self.callback, self.qos, self.count = topic, callback, qos, 0
        def get_publisher_count(self):
            return self.count

    class Node:
        def __init__(self, unused):
            self.subscriptions, self.destroyed = {}, False
        def create_subscription(self, kind, topic, callback, qos):
            subscription = Subscription(topic, callback, qos)
            self.subscriptions[topic] = subscription
            return subscription
        def destroy_node(self):
            self.destroyed = True

    node = Node('test')
    spins = []
    def spin_once(actual_node, timeout_sec):
        assert actual_node is node
        assert not (tmp_path/'observer-inputs-matched.json').exists()
        spins.append(1)
        for topic, subscription in node.subscriptions.items():
            subscription.count = 0 if topic == '/imu0' and len(spins) == 1 else 1
        if len(spins) == 2:
            xyz = SimpleNamespace(x=.1, y=.2, z=.3)
            node.subscriptions['/imu0'].callback(SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=1403715273, nanosec=262142976), frame_id='imu0'),
                angular_velocity=xyz, linear_acceleration=xyz))

    reliability = SimpleNamespace(RELIABLE='reliable', BEST_EFFORT='best_effort')
    modules = {
        'rclpy': SimpleNamespace(init=lambda: None, ok=lambda: len(spins) < 2,
                                spin_once=spin_once, shutdown=lambda: None),
        'rclpy.node': SimpleNamespace(Node=lambda unused: node),
        'rclpy.qos': SimpleNamespace(QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs), ReliabilityPolicy=reliability),
        'sensor_msgs.msg': SimpleNamespace(Image=object, Imu=object),
        'geometry_msgs.msg': SimpleNamespace(PoseWithCovarianceStamped=object),
        'nav_msgs.msg': SimpleNamespace(Odometry=object),
    }
    for name, value in modules.items():
        monkeypatch.setitem(sys.modules, name, value)
    def mapped_path(path):
        path = Path(path)
        return tmp_path/path.name if str(path).startswith('/out/') else path
    monkeypatch.setattr(observer, 'Path', mapped_path)
    monkeypatch.setattr(observer, 'open', lambda path, *a, **kw: builtins.open(mapped_path(path), *a, **kw), raising=False)
    monkeypatch.setattr(observer.signal, 'signal', lambda *unused: None)
    monkeypatch.setattr(sys, 'argv', ['observer', '--mode', mode])
    observer.main()
    topics = {'/cam0/image_raw', '/imu0'} | ({'/cam1/image_raw'} if mode == 'stereo' else set())
    matched = json.loads((tmp_path/'observer-inputs-matched.json').read_text())
    assert matched['matched_publisher_counts'] == dict.fromkeys(topics, 1)
    assert all(node.subscriptions[topic].qos.reliability == 'reliable' for topic in topics)
    assert node.subscriptions['/imu0'].qos.depth == 10000
    assert json.loads((tmp_path/'imu0.jsonl').read_text())['state_ns'] == 1403715273262142976
    assert node.destroyed and len(spins) == 2


def test_comparison_match_evidence_requires_inputs_and_refuses_overwrite(tmp_path):
    from types import SimpleNamespace
    from tools.openvins.observe_comparison_ros2 import write_input_match_evidence
    path = tmp_path/'matched.json'
    assert not write_input_match_evidence({}, path)
    assert not write_input_match_evidence({'/imu0': SimpleNamespace(get_publisher_count=lambda: 0)}, path)
    assert not path.exists()
    matched = {'/imu0': SimpleNamespace(get_publisher_count=lambda: 1)}
    assert write_input_match_evidence(matched, path)
    original = path.read_bytes()
    with pytest.raises(FileExistsError):
        write_input_match_evidence(matched, path)
    assert path.read_bytes() == original


def native_input_match_record(mode='mono', pid=123):
    cameras = [0] if mode == 'mono' else [0, 1]
    return dict(schema_version=1, kind='native_input_publisher_matches',
        scope='required native subscriptions matched publishers at snapshot; not ingestion, writer completeness or VIO health',
        mode=mode, native_monotonic_ns=123456789, native_process_id=pid, camera_ids=cameras,
        subscriptions=[dict(sensor_kind='imu', sensor_id=0, topic='/imu0', matched_publisher_count=1)] +
            [dict(sensor_kind='camera', sensor_id=i, topic=f'/cam{i}/image_raw', matched_publisher_count=1)
             for i in cameras])


@pytest.mark.parametrize('mode', ['mono', 'stereo'])
def test_native_input_matches_preserve_snapshot_and_never_claim_ingestion(tmp_path, mode):
    import json
    from tools.openvins.native_observations import read_input_publisher_matches
    path = tmp_path/'matched.json'
    row = native_input_match_record(mode)
    path.write_text(json.dumps(row))
    before = path.read_bytes()
    assert read_input_publisher_matches(path, mode=mode, native_pid=123) == row
    assert path.read_bytes() == before


@pytest.mark.parametrize('defect', ['missing', 'pending', 'dangling_pending', 'symlink', 'overlong',
    'duplicate_key', 'nonfinite', 'extra_field', 'schema_bool', 'wrong_mode', 'wrong_pid', 'bool_pid',
    'zero_time', 'camera_ids_bool', 'camera_ids_missing', 'wrong_topic', 'wrong_kind', 'zero_match',
    'bool_match', 'duplicate_subscription', 'missing_subscription', 'extra_subscription'])
def test_native_input_match_invalid_snapshot_fails_closed(tmp_path, defect):
    import json
    from tools.openvins.native_observations import read_input_publisher_matches
    path = tmp_path/'matched.json'
    row = native_input_match_record('stereo')
    if defect == 'extra_field': row['ingestion'] = 'PASS'
    elif defect == 'schema_bool': row['schema_version'] = True
    elif defect == 'wrong_mode': row['mode'] = 'mono'
    elif defect == 'wrong_pid': row['native_process_id'] = 124
    elif defect == 'bool_pid': row['native_process_id'] = True
    elif defect == 'zero_time': row['native_monotonic_ns'] = 0
    elif defect == 'camera_ids_bool': row['camera_ids'] = [False, True]
    elif defect == 'camera_ids_missing': row['camera_ids'] = [0]
    elif defect == 'wrong_topic': row['subscriptions'][0]['topic'] = '/other_imu'
    elif defect == 'wrong_kind': row['subscriptions'][0]['sensor_kind'] = 'camera'
    elif defect == 'zero_match': row['subscriptions'][0]['matched_publisher_count'] = 0
    elif defect == 'bool_match': row['subscriptions'][0]['matched_publisher_count'] = True
    elif defect == 'duplicate_subscription': row['subscriptions'].append(row['subscriptions'][0])
    elif defect == 'missing_subscription': row['subscriptions'].pop()
    elif defect == 'extra_subscription': row['subscriptions'][0]['healthy'] = True
    path.write_text(json.dumps(row))
    if defect == 'missing': path.unlink()
    elif defect == 'pending': (tmp_path/'matched.json.pending').write_text('not committed')
    elif defect == 'dangling_pending': (tmp_path/'matched.json.pending').symlink_to('absent')
    elif defect == 'symlink':
        path.rename(tmp_path/'target.json'); path.symlink_to('target.json')
    elif defect == 'overlong': path.write_text(' '*4097)
    elif defect == 'duplicate_key': path.write_text(path.read_text().replace('"schema_version": 1', '"schema_version": 2,"schema_version": 1'))
    elif defect == 'nonfinite': path.write_text(path.read_text().replace('123456789', 'NaN'))
    with pytest.raises((ValueError, OSError)):
        read_input_publisher_matches(path, mode='stereo', native_pid=123)


@pytest.mark.parametrize('case', ['legacy', 'missing', 'pending', 'wrong_pid', 'ready'])
def test_wrapper_native_match_barrier_uses_real_decoder_before_release(tmp_path, case):
    """Execute the wrapper's production barrier, not a copied readiness rule."""
    import json
    import os
    from pathlib import Path
    import subprocess
    root = Path(__file__).resolve().parents[1]
    wrapper = (root/'tools/openvins/run_comparison_ros2.sh').read_text()
    start = wrapper.index('native_input_matches_ready() {')
    function = wrapper[start:wrapper.index('\n}', start)+2]
    path = tmp_path/'matched.json'
    row = native_input_match_record(pid=124 if case == 'wrong_pid' else 123)
    if case != 'missing': path.write_text(json.dumps(row))
    if case == 'pending': (tmp_path/'matched.json.pending').write_text('pending')
    env = dict(os.environ, PYTHONPATH=str(root), VIO_MODE='mono',
               RM27_OPENVINS_INPUT_MATCHES='' if case == 'legacy' else str(path))
    result = subprocess.run(['bash', '-c', function + '\nnative_pid=123\n'
        'if native_input_matches_ready; then printf released; else exit 3; fi'],
        cwd=root, env=env, text=True, capture_output=True, timeout=10)
    assert (result.returncode == 0) == (case in ('legacy', 'ready'))
    assert result.stdout == ('released' if case in ('legacy', 'ready') else '')


def input_match_installation(tmp_path):
    from rm27.perception.vision.localization.experiments.common import file_hash
    install = camera_observation_installation(tmp_path)
    spec = dict(schema_version=1, environment_variable='RM27_OPENVINS_INPUT_MATCHES',
        path_semantics='fresh exclusive atomic final JSON; sibling .pending reserved exclusively; one bounded attempt',
        max_bytes=4096, api='actual owned rclcpp subscription get_publisher_count -> rcl -> rmw matched publishers',
        handles='sub_imu; mono subs_cam; stereo sync_subs_cam.getSubscriber; mode and camera IDs explicit',
        timer='existing20ms independent observation timer; weak visualizer ownership; executor quiesces before destruction',
        scope='reader matched publishers at snapshot only; not writer completeness, input ingestion, tracking health or old failure cause',
        status='BUILD_AND_HELPER_VERIFIED_NATIVE_REPLAY_NOT_RUN', selected_by_runner=False,
        native_replay='NOT_RUN', old_failure_root_cause='UNKNOWN')
    for name in ('helper', 'closure'):
        path = tmp_path/(name+'.json'); path.write_text('synthetic unit-test receipt')
        spec[name+'_receipt'] = str(path)
        spec[name+'_receipt_sha256'] = file_hash(path)
    install['input_publisher_match_observation'] = spec
    return install


@pytest.mark.parametrize('defect', ['build_only', 'unknown_claim', 'altered_receipt', 'no_camera', 'max_bool'])
def test_input_match_capability_rejects_unqualified_or_changed_evidence(tmp_path, defect):
    from pathlib import Path
    from rm27.perception.vision.localization.experiments.openvins_comparison import input_publisher_match_spec
    install = input_match_installation(tmp_path)
    spec = install['input_publisher_match_observation']
    if defect == 'build_only': spec['status'] = 'BUILD_VERIFIED_HELPER_AND_NATIVE_REPLAY_NOT_RUN'
    elif defect == 'unknown_claim': spec['delivery_complete'] = True
    elif defect == 'altered_receipt': Path(spec['helper_receipt']).write_text('changed')
    elif defect == 'no_camera': install.pop('camera_consumption_observation')
    elif defect == 'max_bool': spec['max_bytes'] = True
    with pytest.raises(ValueError): input_publisher_match_spec(install)


def test_input_match_command_is_explicit_and_legacy_unchanged(tmp_path):
    from types import SimpleNamespace
    from rm27.perception.vision.localization.experiments.openvins_comparison import command_for
    installation = input_match_installation(tmp_path)
    dataset = SimpleNamespace(mode='mono', bag=tmp_path/'bag')
    command = command_for(tmp_path, dataset, installation, dict(rate=1.), 'mock')
    assert 'RM27_OPENVINS_INPUT_MATCHES=/out/native-inputs-matched.json' in command
    installation.pop('input_publisher_match_observation')
    legacy = command_for(tmp_path, dataset, installation, dict(rate=1.), 'mock')
    assert not any('RM27_OPENVINS_INPUT_MATCHES' in value for value in legacy)


def test_raw_stream_reconciliation_rejects_changed_velocity():
    row=dict(state_ns=10,position=[1,2,3],quaternion_xyzw=[0,0,0,1],velocity_local=[1,0,0])
    module().check_output_row(row,row,True)
    with pytest.raises(ValueError):
        module().check_output_row(row,{**row,'velocity_local':[0,1,0]},True)


def comparison_fixture(tmp_path, truth=False):
    import hashlib
    import json
    import sqlite3
    import zipfile
    from rm27.perception.vision.localization.experiments.openvins_comparison import ComparisonDataset
    archive = tmp_path/'source.zip'
    with zipfile.ZipFile(archive, 'w') as z:
        for sensor in ['cam0', 'cam1']:
            z.writestr('mav0/' + sensor + '/data.csv', '1403715273262142977,1403715273262142977.png\n')
        z.writestr('mav0/imu0/data.csv', '1403715273262142977,1,2,3,4,5,6\n')
    bag = tmp_path/'bag'
    bag.mkdir()
    with sqlite3.connect(bag/'bag.db3') as db:
        db.execute('CREATE TABLE topics (name TEXT, type TEXT)')
        for name, kind in [('/cam0/image_raw', 'Image'), ('/cam1/image_raw', 'Image'), ('/imu0', 'Imu')]:
            db.execute('INSERT INTO topics VALUES (?,?)', (name, 'sensor_msgs/msg/' + kind))
        if truth:
            db.execute('INSERT INTO topics VALUES (?,?)', ('/truth', 'nav_msgs/msg/Odometry'))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    (bag/'metadata.yaml').write_text('synthetic test metadata; not a playable bag')
    verification = dict(purpose='EUROC_PUBLIC_COMPARISON', sequence='V1_03_difficult', mode='stereo',
                        source_sha256=digest(archive), readback_exact=True, truth_included=False,
                        verified_counts={'cam0': 1, 'cam1': 1, 'imu0': 1},
                        artifacts_sha256={name: digest(bag/name) for name in ['bag.db3', 'metadata.yaml']})
    (bag/'source_verification.json').write_text(json.dumps(verification))
    return archive, bag, verification, ComparisonDataset


def test_independent_comparison_keeps_sequence_and_checks_actual_topics(tmp_path):
    archive, bag, verification, dataset_class = comparison_fixture(tmp_path)
    data = dataset_class(archive, bag, 'V1_03_difficult', 'stereo', verification['source_sha256'])
    assert data.camera_ns == [1403715273262142977]
    assert data.streams['cam1'][0][0] == data.camera_ns[0]
    with pytest.raises(ValueError, match='COMPARISON_INPUT_MISMATCH'):
        dataset_class(archive, bag, 'V1_01_easy', 'stereo', verification['source_sha256'])


def test_comparison_refuses_truth_even_if_manifest_claims_sensor_only(tmp_path):
    archive, bag, verification, dataset_class = comparison_fixture(tmp_path, truth=True)
    with pytest.raises(ValueError, match='TRUTH_ISOLATION_FAILED'):
        dataset_class(archive, bag, 'V1_03_difficult', 'stereo', verification['source_sha256'])


@pytest.mark.parametrize('extra', ['truth.csv', 'truth-directory'])
def test_comparison_refuses_extra_mounted_files_and_directories(tmp_path, extra):
    archive, bag, verification, dataset_class = comparison_fixture(tmp_path)
    if extra.endswith('csv'):
        (bag/extra).write_text('must remain outside estimator mount')
    else:
        (bag/extra).mkdir()
    with pytest.raises(ValueError, match='TRUTH_ISOLATION_FAILED'):
        dataset_class(archive, bag, 'V1_03_difficult', 'stereo', verification['source_sha256'])


def test_comparison_rechecks_bag_membership_and_manifest(tmp_path):
    archive, bag, verification, dataset_class = comparison_fixture(tmp_path)
    data = dataset_class(archive, bag, 'V1_03_difficult', 'stereo', verification['source_sha256'])
    (bag/'source_verification.json').write_text('{}')
    with pytest.raises(ValueError, match='HASH_MISMATCH'):
        data.verify_bag()


def test_comparison_requires_both_database_and_metadata_hashes(tmp_path):
    import json
    archive, bag, verification, dataset_class = comparison_fixture(tmp_path)
    del verification['artifacts_sha256']['metadata.yaml']
    (bag/'source_verification.json').write_text(json.dumps(verification))
    with pytest.raises(ValueError, match='BAG_ARTIFACT_INVALID'):
        dataset_class(archive, bag, 'V1_03_difficult', 'stereo', verification['source_sha256'])


def test_comparison_clock_projection_preserves_unknown_validity_and_integer_time():
    from rm27.perception.vision.localization.experiments.openvins import pose_observation
    from rm27.perception.vision.localization.experiments.openvins_comparison import set_sequence_clock
    ns = 1403715273262142977
    row = pose_observation(source_frame_id=1, image_ns=ns, state_ns=ns, recorder_ns=ns,
                           position=[1, 2, 3], quaternion_xyzw=[0, 0, 0, 1], run_id='test', epoch=0,
                           binary_sha256='0'*64, raw_reference={'topic': '/poseimu'})
    original = dict(row['source_timestamp'])
    result = set_sequence_clock(row, 'V1_03_difficult')
    assert result['source_timestamp'] == {**original, 'clock_domain': 'EuRoC:V1_03_difficult:sensor_time'}
    assert result['backend_timestamp']['raw_value'] == ns
    assert result['tracking_state'] == 'UNKNOWN' and result['validity'] is False
    assert result['localization_fields']['valid'] is False


def test_comparison_command_does_not_mount_source_truth(tmp_path):
    from types import SimpleNamespace
    from rm27.perception.vision.localization.experiments.openvins_comparison import command_for
    cmd = command_for(tmp_path/'out', SimpleNamespace(mode='stereo', bag=tmp_path/'bag'),
                      dict(source='/upstream-source', workspace='/workspace', image='sha256:'+'0'*64),
                      dict(rate=.5), 'container')
    assert 'VIO_MODE=stereo' in cmd and 'VIO_RATE=0.5' in cmd
    assert str(tmp_path/'source.zip') not in ' '.join(cmd)
    assert 'none' in cmd and '4' in cmd and '12g' in cmd


@pytest.mark.parametrize('sensor,change,code', [
    ('cam0', {'frame_id': 'wrong-camera'}, 'INPUT_FRAME_INVALID'),
    ('imu0', {'frame_id': 'wrong-imu'}, 'INPUT_FRAME_INVALID'),
    ('cam0', {'is_bigendian': 1}, 'CAMERA_ENCODING_INVALID'),
])
def test_comparison_checks_left_camera_and_imu_envelopes_before_projection(tmp_path, sensor, change, code):
    import json
    from rm27.perception.vision.localization.experiments.openvins_comparison import normalize_comparison
    (tmp_path/'raw').mkdir()
    for name in ['cam0', 'imu0']:
        row = dict(frame_id=name, is_bigendian=0)
        if name == sensor:
            row.update(change)
        (tmp_path/'raw'/(name+'.jsonl')).write_text(json.dumps(row)+'\n')
    with pytest.raises(ValueError, match=code):
        normalize_comparison(tmp_path, None, {}, 'TEST_FIXTURE_ONLY')


def test_comparison_same_run_score_retains_native_text_camera_rounding(tmp_path):
    from decimal import Decimal
    import json
    from types import SimpleNamespace
    import zipfile
    from rm27.perception.vision.localization.experiments.openvins import pose_observation
    from rm27.perception.vision.localization.experiments.openvins_comparison import evaluate_comparison
    ns = [1403715273262142976 + i*10_000_000 for i in range(12)]
    points = [[i*.1, (i % 3)*.1, (i % 2)*.05] for i in range(12)]
    archive = tmp_path/'SYNTHETIC_TEST.zip'
    with zipfile.ZipFile(archive, 'w') as z:
        z.writestr('mav0/cam0/data.csv', ''.join(f'{t},{t}.png\n' for t in ns))
        z.writestr('mav0/state_groundtruth_estimate0/data.csv', ''.join(
            ','.join(map(str, [t, *p, 1, 0, 0, 0]))+'\n' for t, p in zip(ns, points)))
    (tmp_path/'raw').mkdir()
    (tmp_path/'online').mkdir()
    (tmp_path/'online/trajectory.json').write_text('[]')
    poses = [dict(state_ns=t, position=p, quaternion_xyzw=[0, 0, 0, 1])
             for t, p in zip(ns, points)]
    (tmp_path/'raw/pose.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in poses))
    (tmp_path/'raw/state_estimate.txt').write_text(''.join(' '.join(map(str, [
        Decimal(t)/Decimal(10**9), 0, 0, 0, 1, *p, *([0]*9), '.000005', 1]))+'\n'
        for t, p in zip(ns, points)))
    records = [pose_observation(source_frame_id=i, image_ns=t, state_ns=t, recorder_ns=t,
                    position=p, quaternion_xyzw=[0, 0, 0, 1], run_id='TEST_FIXTURE_ONLY', epoch=0,
                    binary_sha256='0'*64, raw_reference={'topic': '/poseimu'})
               for i, (t, p) in enumerate(zip(ns, points))]
    score = evaluate_comparison(tmp_path, SimpleNamespace(archive=archive, camera_ns=ns), records)
    assert score['same_run_metrics_exact'] and score['scored_epoch_count'] == 1
    assert score['epochs'][0]['first_image_ns'] == ns[0]
    assert score['epochs'][0]['first_state_ns'] == ns[0]
    assert score['camera_association']['max_native_text_residual_ns'] == 5000
    assert score['epochs'][0]['metrics']['ate']['rmse_m'] < 1e-10


@pytest.mark.parametrize('interrupted,protected_output', [(False, None), (True, None),
                                                       ('nonzero', None), ('daemon-failure', None),
                                                       ('still-running', None), ('success-with-live-container', None),
                                                       (False, 'bag'), (False, 'workspace'), (False, 'source')])
def test_comparison_timeout_stops_daemon_owned_container(tmp_path, monkeypatch, interrupted, protected_output):
    import json
    import subprocess
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    from rm27.perception.vision.localization.experiments.openvins import PIN
    from rm27.perception.vision.localization.experiments.common import file_hash
    archive, bag, verification, _ = comparison_fixture(tmp_path)
    binary = tmp_path/'ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    binary.parent.mkdir(parents=True)
    binary.write_text('synthetic binary identity; not executable')
    installation = tmp_path/'installation.json'
    installation.write_text(json.dumps(dict(commit=PIN, image='sha256:'+'0'*64, source=str(tmp_path/'source'),
                                            workspace=str(tmp_path/'ws'), binary_sha256=file_hash(binary), files_sha256={})))
    calls = []
    def fake_run(command, **kwargs):
        calls.append(command)
        if command[1] == 'run':
            if interrupted == 'nonzero':
                return subprocess.CompletedProcess(command, 125)
            if interrupted == 'success-with-live-container':
                return subprocess.CompletedProcess(command, 0)
            if interrupted is True:
                raise KeyboardInterrupt()
            raise subprocess.TimeoutExpired(command, kwargs['timeout'])
        return subprocess.CompletedProcess(command, int(interrupted == 'daemon-failure'),
            stdout='remaining-id\n' if interrupted in ('still-running', 'success-with-live-container') else '')
    monkeypatch.setattr(comparison.subprocess, 'run', fake_run)
    config = dict(schema_version=1, purpose='EUROC_PUBLIC_COMPARISON', run_id='test', backend='OpenVINS',
                  archive=str(archive), archive_sha256=verification['source_sha256'], bag=str(bag),
                  installation=str(installation), output=str(tmp_path/'new-run'), sequence='V1_03_difficult', mode='stereo', rate=.5)
    if protected_output:
        protected = {'bag': bag, 'workspace': tmp_path/'ws', 'source': tmp_path/'source'}[protected_output]
        config['output'] = str(protected/'must-not-create')
        result = comparison.run(config)
        assert result['status'] == 'REFUSED' and result['failure_code'] == 'OUTPUT_INVALID'
        assert not (protected/'must-not-create').exists() and calls == []
        return
    if interrupted is True:
        with pytest.raises(KeyboardInterrupt):
            comparison.run(config)
        result = json.loads((tmp_path/'new-run/run.json').read_text())
    else:
        result = comparison.run(config)
    assert result['status'] == 'FAILED' and result['execution_started']
    expected = ('BACKEND_EXECUTION_FAILED' if interrupted == 'nonzero' else
                'CONTAINER_CLEANUP_UNVERIFIED' if interrupted == 'success-with-live-container' else
                'EXECUTION_INTERRUPTED')
    assert result['failure_code'] == expected
    assert result['transport_protocol'] == 'explicit-reliable-publisher-and-observer-paused-match-v1'
    hashes = result['runtime_sources_sha256']
    for suffix in ['comparison_qos.yaml', 'openvins_runtime.py', 'common.py', 'trajectory.py', 'schema.py']:
        assert any(path.endswith('/' + suffix) for path in hashes)
    clean = interrupted not in ('daemon-failure', 'still-running', 'success-with-live-container')
    assert result['container_cleanup']['absence_verified'] == clean
    assert ('output_sha256' in result) == clean
    if clean:
        assert 'docker.log' in result['output_sha256']
    else:
        assert result['artifact_finalization'] == 'NOT_FROZEN_CONTAINER_CLEANUP_UNVERIFIED'
    assert len(calls) == 3 and calls[1][:3] == ['docker', 'rm', '-f']
    owned = calls[0][calls[0].index('--name') + 1]
    assert calls[1][-1] == owned
    assert calls[2] == ['docker', 'ps', '--all', '--filter', 'name=^/'+owned+'$', '--format', '{{.ID}}']
    assert (tmp_path/'new-run/run.json').is_file()


def camera_observation_installation(tmp_path, yaml_value='false # pinned configuration'):
    """Mock installation identity only; no estimator or hardware executes."""
    from rm27.perception.vision.localization.experiments.openvins import PIN
    from rm27.perception.vision.localization.experiments.common import file_hash
    source=tmp_path/'camera-source'
    configuration=source/'config/euroc_mav/estimator_config.yaml'
    configuration.parent.mkdir(parents=True)
    configuration.write_text('%YAML:1.0\ndownsample_cameras: '+yaml_value+'\n')
    binary=tmp_path/'camera-workspace/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    binary.parent.mkdir(parents=True)
    binary.write_text('TEST_FIXTURE_ONLY; not executable')
    return dict(commit=PIN,image='sha256:'+'0'*64,source=str(source),workspace=str(tmp_path/'camera-workspace'),
        binary_sha256=file_hash(binary),files_sha256={str(configuration):file_hash(configuration)},
        native_observation=dict(schema_version=1,environment_variable='RM27_OPENVINS_NATIVE_EVENTS',
                                max_bytes=104857600,max_events=200000),
        camera_consumption_observation=dict(status='BUILD_AND_HELPER_VERIFIED_NATIVE_REPLAY_NOT_RUN',
            environment_variable='RM27_OPENVINS_NATIVE_EVENTS',payload_key='camera_consumption',schema_version=1,
            pending_receipt_limit=10000,hash='OpenSSL EVP SHA-256',selected_by_runner=False,native_replay='NOT_RUN',
            adapter_integration='NOT_RUN',tracking_health='UNKNOWN',raw_subscription_before_stereo_synchronizer=True,
            stereo_synchronizer='upstream ApproximateTime(10), unchanged',
            pixel_digest='decoded CV_8UC1 row bytes only; excludes row padding; feed and frontend boundaries independent',
            shutdown='executor returned, camera worker joined, unresolved subscriptions/queue exposed before shared footer',
            ownership_fix='callback ImuData captured by value, estimator input values unchanged'))


def openvins_health_capability():
    """Declared native wire capability; no build or replay is claimed by this fixture."""
    return dict(schema_version=2, success_kind='imu_fast_prediction_completed',
        unavailable_kind='imu_fast_prediction_unavailable',
        success_values=['integration_start_seconds', 'integration_end_seconds',
            'main_filter_camera_state_seconds', 'cache_state_time_seconds', 'cache_camera_imu_offset_seconds',
            'mean_all_finite', 'covariance_all_finite'],
        unavailable_values=['integration_start_seconds', 'integration_end_seconds', 'main_filter_camera_state_seconds',
            'cache_state_time_seconds', 'cache_camera_imu_offset_seconds'],
        sensor_time_semantic='REQUESTED_IMU_CALLBACK_DOUBLE', tracking_health='UNKNOWN',
        native_reset='UNSUPPORTED', covariance='FINITE_WITNESS_ONLY_UNQUALIFIED')


def openvins_health_policy():
    return dict(schema_version=1, policy=dict(max_visual_age_ns=300_000_000, max_imu_gap_ns=20_000_000,
        max_receive_age_ns=100_000_000, max_visual_receive_age_ns=600_000_000, max_buffer_samples=100))


def openvins_quality_capability(tmp_path):
    from rm27.perception.vision.localization.experiments.common import file_hash
    schema=tmp_path/'TEST_FIXTURE_ONLY-quality-schema.json'
    schema.write_text('{"fixture":"not native evidence"}')
    return dict(schema_version=1,environment_variable='RM27_OPENVINS_NATIVE_QUALITY',default_enabled=False,
        max_bytes=104857600,max_rows=200000,max_gate_entries_per_batch=2048,
        schema=str(schema),schema_sha256=file_hash(schema),
        tracking_health='UNQUALIFIED; accepted gate support is not overall tracking health',
        covariance='NATIVE_15X15_ERROR_CHART_UNQUALIFIED',native_reset='UNSUPPORTED',
        selected_by_runner=False,native_replay='NOT_RUN')


@pytest.mark.parametrize('failure',[None,'enabled','tracking','covariance','reset','rows','schema_hash','extra_config','bool_version'])
def test_openvins_quality_default_off_and_unsupported_capability_refused(tmp_path,failure):
    from rm27.perception.vision.localization.experiments.openvins_comparison import tracking_quality_spec
    installation=camera_observation_installation(tmp_path)
    spec=installation['native_quality_observation']=openvins_quality_capability(tmp_path)
    configuration=dict(schema_version=1)
    assert tracking_quality_spec(installation,None) is None
    if failure is None:
        assert tracking_quality_spec(installation,configuration) is spec
        return
    if failure=='enabled':spec['default_enabled']=True
    elif failure=='tracking':spec['tracking_health']='VALID'
    elif failure=='covariance':spec['covariance']='CALIBRATED'
    elif failure=='reset':spec['native_reset']='SUPPORTED'
    elif failure=='rows':spec['max_rows']+=1
    elif failure=='schema_hash':spec['schema_sha256']='0'*64
    elif failure=='extra_config':configuration['trust_tracking']=True
    elif failure=='bool_version':configuration['schema_version']=True
    with pytest.raises(ValueError):tracking_quality_spec(installation,configuration)


def test_openvins_quality_runner_refuses_opt_in_without_original_native_health(tmp_path,monkeypatch):
    import json
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    archive,bag,verification,_=comparison_fixture(tmp_path)
    installation=camera_observation_installation(tmp_path)
    installation['native_quality_observation']=openvins_quality_capability(tmp_path)
    path=tmp_path/'installation.json';path.write_text(json.dumps(installation))
    config=dict(schema_version=1,purpose='EUROC_PUBLIC_COMPARISON',run_id='TEST_FIXTURE_ONLY',backend='OpenVINS',
        archive=str(archive),archive_sha256=verification['source_sha256'],bag=str(bag),installation=str(path),
        output=str(tmp_path/'run'),sequence='V1_03_difficult',mode='stereo',rate=.5,native_quality=dict(schema_version=1))
    def refuse(*args,**kwargs):raise AssertionError('must not execute any estimator')
    monkeypatch.setattr(comparison.subprocess,'run',refuse)
    result=comparison.run(config)
    assert result['status']=='REFUSED' and result['failure_code']=='OPENVINS_QUALITY_CONFIG_INVALID'
    assert result['execution_started'] is False and not (tmp_path/'run').exists()


def test_openvins_health_opt_in_requires_explicit_current_native_capability(tmp_path):
    from rm27.perception.vision.localization.experiments.openvins_comparison import state_health_spec
    installation = camera_observation_installation(tmp_path)
    assert state_health_spec(installation, None) is None
    with pytest.raises(ValueError, match='OPENVINS_HEALTH_CONFIG_INVALID'):
        state_health_spec(installation, openvins_health_policy())
    installation['native_health_observation'] = openvins_health_capability()
    policy = openvins_health_policy()
    assert state_health_spec(installation, policy) is policy
    assert state_health_spec(installation, None) is None


@pytest.mark.parametrize('failure', ['missing_native_observation', 'claimed_tracking', 'claimed_reset',
    'unknown_covariance', 'wrong_wire_shape', 'missing_policy', 'extra_config', 'bool_version',
    'native_bool_version', 'legacy_native_version', 'zero_threshold'])
def test_openvins_health_rejects_unsupported_claims_and_unbounded_policy(tmp_path, failure):
    from rm27.perception.vision.localization.experiments.openvins_comparison import state_health_spec
    installation = camera_observation_installation(tmp_path)
    capability = installation['native_health_observation'] = openvins_health_capability()
    policy = openvins_health_policy()
    if failure == 'missing_native_observation': installation.pop('native_observation')
    elif failure == 'claimed_tracking': capability['tracking_health'] = 'PASS'
    elif failure == 'claimed_reset': capability['native_reset'] = 'SUPPORTED'
    elif failure == 'unknown_covariance': capability['covariance'] = 'FLIGHT_VALID'
    elif failure == 'wrong_wire_shape': capability['success_values'].pop()
    elif failure == 'missing_policy': policy.pop('policy')
    elif failure == 'extra_config': policy['epoch'] = 0
    elif failure == 'bool_version': policy['schema_version'] = True
    elif failure == 'native_bool_version': capability['schema_version'] = True
    elif failure == 'legacy_native_version': capability['schema_version'] = 1
    elif failure == 'zero_threshold': policy['policy']['max_visual_age_ns'] = 0
    with pytest.raises((ValueError, TypeError)):
        state_health_spec(installation, policy)


@pytest.mark.parametrize('enabled,quality_enabled', [(False,False),(True,False),(True,True)])
def test_openvins_health_runner_adds_opt_in_artifact_without_changing_trajectory_or_score(tmp_path, monkeypatch, enabled, quality_enabled):
    """Real runner/decoder integration with constructed transport, no estimator run."""
    import json
    import subprocess
    from pathlib import Path
    from test_native_events import ov_health_rows,ov_quality_rows,write_ov_quality
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    from rm27.perception.vision.localization.experiments.openvins import pose_observation
    archive, bag, verification, _ = comparison_fixture(tmp_path)
    installation = camera_observation_installation(tmp_path)
    installation['native_health_observation'] = openvins_health_capability()
    installation['native_quality_observation'] = openvins_quality_capability(tmp_path)
    install_path = tmp_path/'installation.json'
    install_path.write_text(json.dumps(installation))
    config = dict(schema_version=1, purpose='EUROC_PUBLIC_COMPARISON', run_id='TEST_FIXTURE_ONLY',
        backend='OpenVINS', archive=str(archive), archive_sha256=verification['source_sha256'], bag=str(bag),
        installation=str(install_path), output=str(tmp_path/'run'), sequence='V1_03_difficult', mode='stereo', rate=.5)
    if enabled:
        config['state_health'] = openvins_health_policy()
    if quality_enabled:
        config['native_quality'] = dict(schema_version=1)
    def execute(command, **kwargs):
        assert ('RM27_OPENVINS_NATIVE_QUALITY=/out/native-quality.jsonl' in command) is quality_enabled
        raw = tmp_path/'run/raw'
        rows = ov_health_rows()
        for index, row in enumerate(rows):
            row['sequence'] = index
        footer = dict(schema_version=1, kind='observation_closed', event_count=len(rows),
            monotonic_ns=rows[-1]['monotonic_ns']+1, dropped_events=0, io_failed=False)
        (raw/'native-events.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows+[footer]))
        if quality_enabled:
            write_ov_quality(raw/'native-quality.jsonl',ov_quality_rows())
        (raw/'imu0.jsonl').write_text(''.join(json.dumps(dict(state_ns=row['raw_sec']*10**9+row['raw_nsec'],
            frame_id='imu0', gyro=[0]*3, accel=[0]*3))+'\n' for row in rows if row['kind']=='imu_ros_callback'))
        return subprocess.CompletedProcess(command, 0)
    record = pose_observation(source_frame_id=0, image_ns=1403715273262142977, state_ns=1403715273262142977,
        recorder_ns=1, position=[1, 2, 3], quaternion_xyzw=[0, 0, 0, 1], run_id='TEST_FIXTURE_ONLY', epoch=0,
        binary_sha256=installation['binary_sha256'], raw_reference=dict(topic='/poseimu'))
    def normalize(root, *args):
        (root/'online').mkdir()
        (root/'online/trajectory.json').write_text('UNCHANGED PARTIAL TRAJECTORY')
        return [record], dict(scope='TEST_FIXTURE_ONLY')
    def evaluate(root, *args):
        (root/'evaluation.json').write_text('{"fixed_scale_score":"UNCHANGED"}')
    monkeypatch.setattr(comparison.subprocess, 'run', execute)
    monkeypatch.setattr(comparison, 'cleanup_container', lambda *args: dict(absence_verified=True))
    monkeypatch.setattr(comparison, 'observe_camera_consumption', lambda *args: dict(status='PASS', scope='TEST_FIXTURE_ONLY'))
    monkeypatch.setattr(comparison, 'normalize_comparison', normalize)
    monkeypatch.setattr(comparison, 'evaluate_comparison', evaluate)
    result = comparison.run(config)
    assert result['status'] == 'EXECUTION_COMPLETED', result
    assert result['stage_result'] == 'COMPARISON_RUNTIME_AND_METRIC_PASS'
    root = tmp_path/'run'
    assert (root/'online/trajectory.json').read_text() == 'UNCHANGED PARTIAL TRAJECTORY'
    assert (root/'evaluation.json').read_text() == '{"fixed_scale_score":"UNCHANGED"}'
    assert record['tracking_state'] == 'UNKNOWN' and record['validity'] is False
    assert ('native_state_health' in result) is enabled
    assert ('native_tracking_quality' in result) is quality_enabled
    assert (root/'native-health.json').exists() is enabled
    assert (root/'online/native-health.jsonl').exists() is enabled
    state_source = str(Path(comparison.__file__).parent.parent/'state_output.py')
    assert (state_source in result['runtime_sources_sha256']) is enabled
    if enabled:
        health = json.loads((root/'native-health.json').read_text())
        assert health['status'] == 'PASS' and health['operational_health'] == 'UNKNOWN'
        assert health['operational_valid'] is False
        assert result['native_state_health']['status'] == 'PASS'
        assert 'native-health.json' in result['output_sha256']
        assert 'online/native-health.jsonl' in result['output_sha256']
    if quality_enabled:
        quality=json.loads((root/'native-quality.json').read_text())
        assert quality['measurement_health']['counts']['PREDICTION_ONLY']>0
        assert quality['operational_tracking_health']=='UNKNOWN' and not quality['output_eligible']
        assert 'online/measurement-health.jsonl' in result['output_sha256']


def test_camera_capability_accepts_owned_initializer_shutdown_without_health_upgrade(tmp_path):
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    installation=camera_observation_installation(tmp_path)
    capability=installation['camera_consumption_observation']
    capability['shutdown']='executor returned; all three owned worker levels joined; then queue/receipt footer; independent of observation enabled'
    result=comparison.camera_observation_spec(installation)
    assert result is capability
    assert result['tracking_health']=='UNKNOWN' and result['native_replay']=='NOT_RUN'
    assert result['adapter_integration']=='NOT_RUN' and result['selected_by_runner'] is False
    assert comparison.camera_observation_configuration(installation)['downsample_cameras'] is False


@pytest.mark.parametrize('value,wanted', [('true # preserved comment',True),('false',False),('1',True),('0',False),
                                         ('"True"',True),('FALSE',False)])
def test_camera_downsample_resolves_actual_locked_opencv_yaml(tmp_path,value,wanted):
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    from rm27.perception.vision.localization.experiments.common import file_hash
    installation=camera_observation_installation(tmp_path,value)
    result=comparison.camera_observation_configuration(installation)
    assert result['downsample_cameras'] is wanted and result['parameter_override'] is False
    assert installation['files_sha256'][result['estimator_config']]==result['estimator_config_sha256']
    assert result['launcher_sha256']==file_hash(result['launcher'])


@pytest.mark.parametrize('value', ['2','0.0','null','[false]','false\ndownsample_cameras: true'])
def test_camera_downsample_unknown_or_duplicate_is_refused(tmp_path,value):
    from rm27.perception.vision.localization.experiments.openvins_comparison import camera_observation_configuration
    with pytest.raises(ValueError,match='OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID'):
        camera_observation_configuration(camera_observation_installation(tmp_path,value))


@pytest.mark.parametrize('defect', ['unlocked','missing','modified'])
def test_camera_downsample_requires_exact_locked_configuration(tmp_path,defect):
    from pathlib import Path
    from rm27.perception.vision.localization.experiments.openvins_comparison import camera_observation_configuration
    installation=camera_observation_installation(tmp_path)
    path=Path(next(iter(installation['files_sha256'])))
    if defect=='unlocked':installation['files_sha256']={}
    elif defect=='missing':path.write_text('%YAML:1.0\nother_parameter: false\n');installation['files_sha256'][str(path)]=module().file_hash(path)
    else:path.write_text('%YAML:1.0\ndownsample_cameras: true\n')
    with pytest.raises(ValueError):camera_observation_configuration(installation)


@pytest.mark.parametrize('key,value', [('schema_version',2),('schema_version',True),('payload_key','other'),
    ('environment_variable','OTHER'),('pending_receipt_limit',True),('pending_receipt_limit',9999),
    ('raw_subscription_before_stereo_synchronizer',False),('hash','custom hash'),('tracking_health','PASS'),
    ('shutdown','worker may still run'),('unknown_stage',True),('status','PASS'),
    ('selected_by_runner',True),('selected_by_runner',0),('native_replay','PASS'),('adapter_integration','PASS')])
def test_camera_unknown_capability_is_refused_before_output_or_execution(tmp_path,monkeypatch,key,value):
    import json
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    archive,bag,verification,_=comparison_fixture(tmp_path)
    installation=camera_observation_installation(tmp_path)
    installation['camera_consumption_observation'][key]=value
    path=tmp_path/'installation.json';path.write_text(json.dumps(installation))
    def forbidden(*args,**kwargs):raise AssertionError('invalid capability must not execute')
    monkeypatch.setattr(comparison.subprocess,'run',forbidden)
    config=dict(schema_version=1,purpose='EUROC_PUBLIC_COMPARISON',run_id='TEST_FIXTURE_ONLY',backend='OpenVINS',
        archive=str(archive),archive_sha256=verification['source_sha256'],bag=str(bag),installation=str(path),
        output=str(tmp_path/'must-not-create'),sequence='V1_03_difficult',mode='stereo',rate=.5)
    result=comparison.run(config)
    assert result['status']=='REFUSED' and not result['execution_started']
    assert result['failure_code']=='OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID'
    assert not (tmp_path/'must-not-create').exists()


def test_camera_capability_requires_native_observation_and_old_install_is_unchanged(tmp_path):
    from types import SimpleNamespace
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    installation=camera_observation_installation(tmp_path)
    installation.pop('native_observation')
    with pytest.raises(ValueError,match='OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID'):
        comparison.camera_observation_spec(installation)
    installation.pop('camera_consumption_observation')
    assert comparison.camera_observation_spec(installation) is None
    command=comparison.command_for(tmp_path,SimpleNamespace(mode='mono',bag=tmp_path/'bag'),installation,dict(rate=.5),'mock')
    assert not any('RM27_OPENVINS_NATIVE_EVENTS' in token for token in command)


def test_camera_evidence_uses_complete_canonical_dataset_and_keeps_fail_separate(tmp_path,monkeypatch):
    import json
    from types import SimpleNamespace
    from tools.openvins import native_observations
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    dataset=SimpleNamespace(mode='mono')
    config=dict(downsample_cameras=False,estimator_config_sha256='0'*64)
    reference=dict(records={0:[]},camera_ids=[0],complete=True,source_archive_sha256='1'*64)
    calls=[]
    def build(actual,raw,**kwargs):
        assert actual is dataset and raw==tmp_path/'raw'
        assert kwargs==dict(downsample_cameras=False,sample_limit=None)
        calls.append('reference');return reference
    def analyze(path,**kwargs):
        assert path==tmp_path/'raw/native-events.jsonl' and kwargs['reference'] is reference
        assert kwargs['camera_ids']==[0] and kwargs['downsample_cameras'] is False
        calls.append('native')
        return dict(observation_integrity='PASS',input_coverage=dict(status='PASS'),full_source_consumption='FAIL')
    monkeypatch.setattr(native_observations,'build_camera_reference',build)
    monkeypatch.setattr(native_observations,'analyze_camera_consumption',analyze)
    gate=comparison.observe_camera_consumption(tmp_path,dataset,config)
    assert calls==['reference','native'] and gate['status']=='FAIL'
    assert gate['input_coverage']['status']=='PASS' and gate['full_source_consumption']=='FAIL'
    assert gate['numerical_trajectory_status']=='INDEPENDENT' and gate['ekf_acceptance']=='NOT_INFERRED'
    assert gate['health_qualification']=='NOT_RUN'
    output=json.loads((tmp_path/'native-camera-observations.json').read_text())
    assert output['configuration']==config and 'records' not in output['input_reference']


def test_camera_source_mismatch_is_retained_as_evidence_failure(tmp_path,monkeypatch):
    import json
    from tools.openvins import native_observations
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    from rm27.perception.vision.localization.experiments.common import ExperimentError
    def mismatch(*args,**kwargs):
        raise ExperimentError('OPENVINS_NATIVE_CAMERA_INPUT_MISMATCH','original CDR and pixels differ')
    monkeypatch.setattr(native_observations,'build_camera_reference',mismatch)
    result=comparison.observe_camera_consumption(tmp_path,None,dict(downsample_cameras=False))
    report=json.loads((tmp_path/'native-camera-observations.json').read_text())
    assert result['status']=='FAIL' and result['input_coverage']['status']=='FAIL'
    assert report['failure_code']=='OPENVINS_NATIVE_CAMERA_INPUT_MISMATCH' and not report['observation_complete']


def test_camera_runner_routes_evidence_without_promoting_unknown_state(tmp_path,monkeypatch):
    import json
    import subprocess
    from tools.openvins import native_observations
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    from rm27.perception.vision.localization.experiments.openvins import pose_observation
    archive,bag,verification,_=comparison_fixture(tmp_path)
    installation=camera_observation_installation(tmp_path)
    path=tmp_path/'installation.json';path.write_text(json.dumps(installation))
    config=dict(schema_version=1,purpose='EUROC_PUBLIC_COMPARISON',run_id='TEST_FIXTURE_ONLY',backend='OpenVINS',
        archive=str(archive),archive_sha256=verification['source_sha256'],bag=str(bag),installation=str(path),
        output=str(tmp_path/'run'),sequence='V1_03_difficult',mode='stereo',rate=.5)
    calls=[]
    def execute(command,**kwargs):
        assert 'RM27_OPENVINS_NATIVE_EVENTS=/out/native-events.jsonl' in command
        (tmp_path/'run/raw/native-events.jsonl').write_text('mock wire; decoder tested separately')
        return subprocess.CompletedProcess(command,0)
    def observe(root,data,configuration):
        assert isinstance(data,comparison.ComparisonDataset) and data.mode=='stereo'
        assert configuration['downsample_cameras'] is False
        calls.append('camera')
        return dict(status='FAIL',scope='MOCK_ROUTING_ONLY')
    ns=1403715273262142977
    row=pose_observation(source_frame_id=0,image_ns=ns,state_ns=ns,recorder_ns=ns,
        position=[0,0,0],quaternion_xyzw=[0,0,0,1],run_id='TEST_FIXTURE_ONLY',epoch=0,
        binary_sha256=installation['binary_sha256'],raw_reference=dict(topic='/poseimu'))
    def normalize(root,*args):
        calls.append('normalize')
        (root/'online').mkdir()
        (root/'online/trajectory.json').write_text('mock trajectory must remain unchanged')
        return [row],dict(scope='MOCK_ONLY')
    def evaluate(root,data,records):
        assert records[0]['tracking_state']=='UNKNOWN' and records[0]['validity'] is False
        (root/'evaluation.json').write_text('{"mock_score":"must remain unchanged"}')
        calls.append('score')
    monkeypatch.setattr(comparison.subprocess,'run',execute)
    monkeypatch.setattr(comparison,'cleanup_container',lambda *args:dict(absence_verified=True))
    monkeypatch.setattr(comparison,'observe_camera_consumption',observe)
    monkeypatch.setattr(comparison,'normalize_comparison',normalize)
    monkeypatch.setattr(comparison,'evaluate_comparison',evaluate)
    def native_projection(*args,**kwargs):
        assert kwargs['include_camera_transactions'] is True
        return dict(status='PASS',observation_integrity='PASS',accepted_visual_update_count=1,
            native_imu_callback_coverage=dict(status='PASS'),
            camera_transactions=dict(status='NOT_RUN',transaction_count=0,records=[]))
    monkeypatch.setattr(native_observations,'analyze_openvins_observations',native_projection)
    result=comparison.run(config)
    assert result['status']=='EXECUTION_COMPLETED',result
    assert result['stage_result']=='COMPARISON_RUNTIME_AND_METRIC_PASS'
    assert result['camera_consumption_evidence_gate']['status']=='FAIL'
    assert calls==['camera','normalize','score']
    assert result['native_camera_observation_configuration']['downsample_cameras'] is False
    assert result['container_cleanup']['absence_verified'] is True
    pointer=result['evidence']['native_camera_transactions']
    assert pointer['path']=='native-observations.json' and pointer['section']=='camera_transactions'
    assert pointer['status']=='NOT_RUN' and pointer['transaction_count']==0
    assert json.loads((tmp_path/'run/online/evidence.json').read_text())==result['evidence']
    assert (tmp_path/'run/online/trajectory.json').read_text()=='mock trajectory must remain unchanged'
    assert (tmp_path/'run/evaluation.json').read_text()=='{"mock_score":"must remain unchanged"}'
    assert row['tracking_state']=='UNKNOWN' and row['validity'] is False


def camera_reference_fixture(tmp_path,mode):
    """Actual serialized CDR and PNG bytes, entirely synthetic test input."""
    import hashlib
    import json
    import zipfile
    pytest.importorskip('rosbag2_py')
    from test_schurvins_transport import source_zip
    from tools.openvins.comparison_transport import build_bag,image_pixels
    from rm27.perception.vision.localization.experiments.openvins_comparison import ComparisonDataset
    archive=tmp_path/'TEST_FIXTURE_ONLY.zip';source_zip(archive)
    bag=tmp_path/'bag'
    verification=build_bag(archive,bag,mode,'V1_01_easy',reserve_bytes=0)
    dataset=ComparisonDataset(archive,bag,'V1_01_easy',mode,verification['source_sha256'])
    raw=tmp_path/'raw';raw.mkdir()
    with zipfile.ZipFile(archive) as source:
        for sensor,rows in dataset.streams.items():
            if sensor=='imu0':continue
            observed=[]
            for ns,name in rows:
                image=image_pixels(source,sensor,name)
                observed.append(dict(state_ns=ns,frame_id=sensor,encoding='mono8',is_bigendian=0,
                    height=image.shape[0],width=image.shape[1],step=image.shape[1],
                    pixels_sha256=hashlib.sha256(image.tobytes()).hexdigest()))
            (raw/(sensor+'.jsonl')).write_text(''.join(json.dumps(row)+'\n' for row in observed))
    return dataset,raw


@pytest.mark.parametrize('mode',['mono','stereo'])
def test_camera_reference_reconciles_all_source_cdr_observer_bytes_and_integer_times(tmp_path,mode):
    from tools.openvins.native_observations import build_camera_reference
    dataset,raw=camera_reference_fixture(tmp_path,mode)
    complete=build_camera_reference(dataset,raw,downsample_cameras=False,sample_limit=None)
    assert complete['complete'] is True and complete['truth_used'] is False
    assert complete['camera_ids']==([0,1] if mode=='stereo' else [0])
    for camera,records in complete['records'].items():
        assert len(records)==3
        assert [record['raw']['raw_sensor_timestamp_ns'] for record in records]==[ns for ns,_ in dataset.streams['cam'+str(camera)]]
        assert all(record['raw']['decoded']==record['frontend'] for record in records)
    sampled=build_camera_reference(dataset,raw,downsample_cameras=False,sample_limit=1)
    assert sampled['complete'] is False and len(sampled['records'][0])==1
    assert sampled['native_consumption']=='NOT_RUN'


@pytest.mark.parametrize('defect',['observer_pixels','observer_frame','observer_time','observer_order',
                                    'cdr_pixels','cdr_time','cdr_frame','odd_downsample'])
def test_camera_reference_source_equality_cannot_be_replaced_by_consistent_counts(tmp_path,defect):
    import json
    import sqlite3
    from tools.openvins.native_observations import build_camera_reference
    from rm27.perception.vision.localization.experiments.common import file_hash
    from rm27.perception.vision.localization.experiments.openvins_comparison import ComparisonDataset
    dataset,raw=camera_reference_fixture(tmp_path,'mono')
    path=raw/'cam0.jsonl'
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    if defect.startswith('observer'):
        if defect=='observer_pixels':rows[0]['pixels_sha256']='0'*64
        elif defect=='observer_frame':rows[0]['frame_id']='wrong-camera'
        elif defect=='observer_time':rows[0]['state_ns']+=1
        else:rows[0],rows[1]=rows[1],rows[0]
        path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    elif defect.startswith('cdr'):
        from rclpy.serialization import deserialize_message,serialize_message
        from sensor_msgs.msg import Image
        database=next(dataset.bag.glob('*.db3'))
        with sqlite3.connect(database) as db:
            ordinal,data=db.execute('SELECT m.id,m.data FROM messages m JOIN topics t ON m.topic_id=t.id '
                "WHERE t.name='/cam0/image_raw' ORDER BY m.id LIMIT 1").fetchone()
            message=deserialize_message(data,Image)
            if defect=='cdr_pixels':message.data[0]^=1
            elif defect=='cdr_time':message.header.stamp.nanosec+=1
            else:message.header.frame_id='wrong-camera'
            db.execute('UPDATE messages SET data=? WHERE id=?',(serialize_message(message),ordinal))
        # Deliberately freeze the altered synthetic CDR so equality must be
        # checked against the source and observer rather than only file hashes.
        manifest=json.loads((dataset.bag/'source_verification.json').read_text())
        manifest['artifacts_sha256'][database.name]=file_hash(database)
        (dataset.bag/'source_verification.json').write_text(json.dumps(manifest))
        dataset=ComparisonDataset(dataset.archive,dataset.bag,dataset.sequence,dataset.mode,manifest['source_sha256'])
    with pytest.raises(ValueError):
        build_camera_reference(dataset,raw,downsample_cameras=defect=='odd_downsample',sample_limit=None)


@pytest.mark.parametrize('suffix',['\n-p downsample_cameras:=true\n','\n-p config_path:=/other.yaml\n',
                                   '\n--params-file /other.yaml\n'])
def test_camera_configuration_refuses_launcher_overrides(tmp_path,monkeypatch,suffix):
    from pathlib import Path
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    installation=camera_observation_installation(tmp_path)
    original=Path.read_text
    def read(path,*args,**kwargs):
        value=original(path,*args,**kwargs)
        return value+suffix if path.name=='run_comparison_ros2.sh' else value
    monkeypatch.setattr(Path,'read_text',read)
    with pytest.raises(ValueError,match='OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID'):
        comparison.camera_observation_configuration(installation)
