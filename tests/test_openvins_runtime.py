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
