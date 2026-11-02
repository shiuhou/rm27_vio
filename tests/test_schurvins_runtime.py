"""Synthetic file/transport boundaries; these do not execute a native estimator."""
import copy
from decimal import Decimal
import errno
import json
from pathlib import Path
import subprocess
import signal
import zipfile

import cv2
import numpy as np
import pytest

from rm27.perception.vision.localization.experiments.common import file_hash, write_json
from rm27.perception.vision.localization.experiments.schurvins import PIN
from rm27.perception.vision.localization.experiments import schurvins_runtime as runtime
from tools.schurvins.build import snapshot
from tools.schurvins.dataset import prepare
from tools.schurvins.evaluate import evaluate_rows, read_reference
from tools.schurvins import run_inner


def jsonl(path, rows):
    path.write_text(''.join(json.dumps(r)+'\n' for r in rows))


@pytest.fixture
def fixture(tmp_path):
    ns = [1403715273262142976+i*10_000_000 for i in range(12)]
    points = [[i*.1, (i % 3)*.1, (i % 2)*.05] for i in range(12)]
    archive=tmp_path/'TEST_FIXTURE_ONLY.zip'
    ok, png=cv2.imencode('.png',np.zeros((8,10),np.uint8))
    assert ok
    with zipfile.ZipFile(archive,'w') as z:
        z.writestr('mav0/cam0/data.csv',''.join(f'{t},{t}.png\n' for t in ns))
        z.writestr('mav0/imu0/data.csv',''.join(f'{t},1,2,3,4,5,6\n' for t in ns))
        for sensor in ['cam0','imu0']:
            z.writestr('mav0/'+sensor+'/sensor.yaml','TEST_FIXTURE_ONLY: true\n')
        for t in ns:
            z.writestr(f'mav0/cam0/data/{t}.png',png.tobytes())
        z.writestr('mav0/state_groundtruth_estimate0/data.csv',''.join(
            ','.join(map(str,[t,*p,1,0,0,0]))+'\n' for t,p in zip(ns,points)))
    inputs=tmp_path/'input'
    data=prepare(archive,inputs,'mono')
    ws=tmp_path/'workspace'
    source=ws/'src/SchurVINS'
    (source/'svo/include/svo').mkdir(parents=True)
    (source/'svo/include/svo/global.h').write_text('#define USE_SCHUR_VINS\n')
    cmake=source/'svo_cmake/cmake/Modules/SvoSetup.cmake'
    cmake.parent.mkdir(parents=True)
    cmake.write_text('SET(USE_LOOP_CLOSING FALSE)\nSET(USE_GLOBAL_MAP FALSE)\n')
    binary=ws/'devel/lib/svo_ros/svo_node'
    binary.parent.mkdir(parents=True)
    binary.write_bytes(b'TEST_FIXTURE_ONLY: not a runnable native binary')
    install=dict(schema_version=1,backend='SchurVINS',commit=PIN,image_id='sha256:'+'a'*64,
        workspace=str(ws),binary='devel/lib/svo_ros/svo_node',binary_sha256=file_hash(binary),
        source_files_sha256=snapshot(ws/'src'),devel_files_sha256=snapshot(ws/'devel'))
    install.update(source_symlinks={}, devel_symlinks={})
    installation=tmp_path/'installation.json'
    write_json(installation,install)
    config=dict(schema_version=1,backend='SchurVINS',purpose='EUROC_PUBLIC_BASELINE',
        run_id='TEST_FIXTURE_ONLY',sequence='V1_01_easy',mode='mono',rate=.5,
        input=str(inputs),installation=str(installation),reference_archive=str(archive),
        output=str(tmp_path/'run'),timeout_s=30)
    return dict(config=config,inputs=inputs,data=data,install=install,ws=ws,
                archive=archive,ns=ns,points=points,root=tmp_path/'run')


def raw_run(fixture):
    root=fixture['root']
    (root/'raw').mkdir(parents=True)
    (root/'logs').mkdir()
    expected=runtime.jsonl(fixture['inputs']/'expected.jsonl')
    camera=[r for r in expected if r['stream']=='cam0']
    jsonl(root/'raw/cam0.jsonl',camera)
    jsonl(root/'raw/imu0.jsonl',[r for r in expected if r['stream']=='imu0'])
    # The real publisher logs every selected stream, with IMU first at a tie.
    # Share each source timestamp's synthetic deadline so camera-lag checks
    # retain the same timing while reconciliation also exercises IMU delivery.
    published=[dict(r,publish_call_monotonic_ns=1_000_000_000+r['state_ns']-fixture['ns'][0],
                    publish_return_monotonic_ns=1_000_000_001+r['state_ns']-fixture['ns'][0])
               for r in sorted(expected,key=lambda r:(r['state_ns'],r['stream']!='imu0'))]
    jsonl(root/'raw/published.jsonl',published)
    poses=[dict(state_ns=t,native_sequence=i+1,frame_id='world',position=p,
                quaternion_xyzw=[0,0,0,1],observed_monotonic_ns=1_005_000_000+i*10_000_000)
           for i,(t,p) in enumerate(zip(fixture['ns'],fixture['points']))]
    info=[dict(state_ns=t,native_sequence=i+1,stage=2,tracking_quality=2,
               processing_time_s=.003,observed_monotonic_ns=1_004_000_000+i*10_000_000)
          for i,t in enumerate(fixture['ns'])]
    jsonl(root/'raw/pose.jsonl',poses)
    jsonl(root/'raw/info.jsonl',info)
    (root/'logs/imu_traj.txt').write_text(''.join(
        str(Decimal(r['state_ns'])/Decimal(10**9))+' '+
        ' '.join(map(str,r['position']+r['quaternion_xyzw']))+'\n' for r in poses))
    return poses,info


def test_preflight_verifies_real_files_without_output_creation(fixture):
    value=runtime.preflight(fixture['config'])
    assert value[0] == fixture['inputs']
    assert value[1]['original_archive_sha256'] == file_hash(fixture['archive'])
    assert not fixture['root'].exists()


def test_transport_snapshot_is_immutable_when_development_source_changes(tmp_path):
    source, frozen = tmp_path/'source', tmp_path/'snapshot'
    source.mkdir()
    for name in ('run_inner.py', 'ros_transport.py'):
        (source/name).write_text('original transport\n')
    hashes = runtime.freeze_transport(source, frozen)
    (source/'ros_transport.py').write_text('later development\n')
    assert file_hash(frozen/'ros_transport.py') == hashes['ros_transport.py']
    assert (frozen/'ros_transport.py').read_text() == 'original transport\n'


@pytest.mark.parametrize('failure', [None, 'marker_missing', 'wrong_clock', 'unjoined',
                                    'native', 'player', 'observer', 'master', 'cleanup'])
def test_watchdog_completeness_requires_join_and_all_process_exits(tmp_path, failure):
    (tmp_path/'raw').mkdir()
    tick = dict(schema_version=1, event='WATCHDOG_TICK', trigger='INDEPENDENT_SCHEDULE',
                clock_source='OBSERVER_CLOCK_MONOTONIC', observed_monotonic_ns=123,
                sensor_now=None)
    jsonl(tmp_path/'raw/watchdog.jsonl', [tick])
    marker = dict(schema_version=1, trigger='INDEPENDENT_SCHEDULE', thread_joined=True,
                  clock_source='OBSERVER_CLOCK_MONOTONIC')
    if failure == 'wrong_clock':
        marker['clock_source'] = 'NATIVE_STEADY'
    if failure == 'unjoined':
        marker['thread_joined'] = False
    if failure != 'marker_missing':
        write_json(tmp_path/'raw/watchdog-complete.json', marker)
    execution = dict(failure=None, cleanup_errors={},
                     exits=dict.fromkeys(['native', 'player', 'observer', 'master'], 0))
    if failure in execution['exits']:
        execution['exits'][failure] = 1
    if failure == 'cleanup':
        execution['cleanup_errors']['observer'] = 'not reaped'
    ticks, complete = runtime.watchdog_evidence(tmp_path, execution)
    assert ticks == [tick]
    assert bool(complete) == (failure is None)


def test_native_reset_segments_are_scored_as_separate_epochs(fixture):
    poses, _ = raw_run(fixture)
    events = []
    for i, pose in enumerate(poses):
        if i == 6:
            events.append(dict(schema_version=1, kind='reset', sensor_ns=-1,
                               bundle_id=-1, monotonic_ns=4*i, values=[]))
        for delta, kind, values in [(1, 'frontend_begin', []), (2, 'frontend_end', [2, 0, 2])]:
            events.append(dict(schema_version=1, kind=kind, sensor_ns=pose['state_ns'],
                               bundle_id=i, monotonic_ns=4*i+delta, values=values))
    jsonl(fixture['root']/'native-events.jsonl', events)
    rows, projected, evidence = runtime.normalize(fixture['root'], fixture['data'],
                                                   fixture['install'], fixture['config'])
    assert [r['epoch'] for r in rows] == [0]*6+[1]*6
    assert [r['localization_epoch'] for r in projected] == [0]*6+[1]*6
    assert evidence['adapter_epoch_count'] == 2 and evidence['native_reset_count'] == 1
    assert all(not r['validity'] for r in projected)


def test_preflight_refuses_unfrozen_container_absolute_link(fixture):
    (fixture['ws']/'devel/setup.bash').symlink_to('/ws/devel/.private/setup.bash')
    result = runtime.run(fixture['config'])
    assert result['failure_code'] == 'INSTALLATION_SYMLINK_MISMATCH'
    assert not fixture['root'].exists()


def test_preflight_refuses_new_unfrozen_source_file(fixture):
    (fixture['ws']/'src/unfrozen.py').write_text('unexpected runtime source')
    result = runtime.run(fixture['config'])
    assert result['failure_code'] == 'INSTALLATION_TREE_CHANGED'


@pytest.mark.parametrize('changed',['ledger','binary','source'])
def test_preflight_refuses_changed_frozen_files(fixture,changed):
    path={'ledger':fixture['inputs']/'expected.jsonl',
          'binary':fixture['ws']/'devel/lib/svo_ros/svo_node',
          'source':fixture['ws']/'src/SchurVINS/svo/include/svo/global.h'}[changed]
    path.write_bytes(path.read_bytes()+b'changed')
    result=runtime.run(fixture['config'])
    assert result['status']=='REFUSED' and result['failure_code']=='HASH_MISMATCH'
    assert not fixture['root'].exists()


def test_preflight_inspects_truth_isolation_beyond_manifest_flag(fixture):
    with zipfile.ZipFile(fixture['inputs']/'sensors.zip','a') as z:
        z.writestr('mav0/state_groundtruth_estimate0/data.csv','forbidden truth')
    path=fixture['inputs']/'input.json'
    data=json.loads(path.read_text())
    data['artifacts_sha256']['sensors.zip']=file_hash(fixture['inputs']/'sensors.zip')
    write_json(path,data)
    result=runtime.run(fixture['config'])
    assert result['failure_code']=='INPUT_TRUTH_OR_TOPIC_FORBIDDEN'
    assert not fixture['root'].exists()


def test_preflight_refuses_unlisted_files_in_mounted_input_directory(fixture):
    (fixture['inputs']/'state_groundtruth_estimate0.csv').write_text('forbidden truth')
    result=runtime.run(fixture['config'])
    assert result['failure_code']=='INPUT_TRUTH_OR_TOPIC_FORBIDDEN'
    assert not fixture['root'].exists()


def test_existing_run_is_never_overwritten_even_on_refusal(fixture):
    fixture['root'].mkdir()
    saved=fixture['root']/'run.json'
    saved.write_text('original failed experiment\n')
    result=runtime.run(fixture['config'])
    assert result['status']=='REFUSED'
    assert saved.read_text()=='original failed experiment\n'


def test_competing_output_created_after_preflight_is_never_overwritten(fixture, monkeypatch):
    preflight = runtime.preflight
    saved = fixture['root']/'run.json'

    def competing_preflight(config):
        prepared = preflight(config)
        fixture['root'].mkdir()
        saved.write_text('competing experiment evidence\n')
        return prepared

    monkeypatch.setattr(runtime, 'preflight', competing_preflight)
    result = runtime.run(fixture['config'])
    assert result['status'] == 'REFUSED'
    assert not result['execution_started']
    assert saved.read_text() == 'competing experiment evidence\n'


def test_normalization_preserves_integer_time_and_same_run_metrics(fixture):
    poses,_=raw_run(fixture)
    rows,projected,evidence=runtime.normalize(fixture['root'],fixture['data'],
                                             fixture['install'],fixture['config'])
    assert rows[0]['state_ns']==1403715273262142976
    assert projected[0]['source_timestamp']['raw_value']==poses[0]['state_ns']
    assert projected[0]['tracking_state']=='UNKNOWN' and not projected[0]['validity']
    assert evidence['native_reset_count'] is None and evidence['adapter_epoch_count']==1
    assert evidence['native_tail_seen']
    assert evidence['native_camera_callbacks']['missing_count'] == 0
    assert evidence['native_text_parity']['max_stamp_residual_ns']==0
    ref=read_reference(fixture['archive'])
    assert evaluate_rows(rows,ref,tolerance_ns=0)==evaluate_rows(projected,ref,tolerance_ns=0)
    assert poses==runtime.jsonl(fixture['root']/'raw/pose.jsonl')


def test_generic_euroc_format_label_preserves_existing_clock_and_projection(fixture):
    raw_run(fixture)
    upper = dict(fixture['data'], dataset='EUROC')
    rows, projected, _ = runtime.normalize(fixture['root'], upper, fixture['install'], fixture['config'])
    assert runtime.sensor_dataset(upper) == 'EuRoC'
    assert projected[0]['source_timestamp']['clock_domain'] == 'EuRoC:V1_01_easy:sensor_time'
    assert rows[0]['state_ns'] == fixture['ns'][0]
    assert upper['dataset'] == 'EUROC'
    assert evaluate_rows(projected, read_reference(fixture['archive']), tolerance_ns=0)['scored_epoch_count'] == 1
    for invalid in [dict(upper, purpose='UZH_FPV_PUBLIC_REPLAY'),
                    dict(upper, dataset='UNQUALIFIED'), dict(upper, purpose='SEALED_SENSOR_RECORDING_REPLAY')]:
        with pytest.raises(ValueError, match='purpose and sensor dataset identity'):
            runtime.sensor_dataset(invalid)


def test_normalization_preserves_delivery_order_and_source_high_water_mark(fixture):
    poses, info = raw_run(fixture)
    path = fixture['root']/'raw/published.jsonl'
    published = runtime.jsonl(path)
    camera_slots = [i for i, row in enumerate(published) if row['stream'] == 'cam0']
    # Source camera 1 arrives after camera 2. Publication calls stay ordered;
    # the native guard rejects the late old camera instead of retiming it.
    first, second = camera_slots[1:3]
    published[first]['state_ns'], published[second]['state_ns'] = (
        published[second]['state_ns'], published[first]['state_ns'])
    jsonl(path, published)
    camera = runtime.jsonl(fixture['root']/'raw/cam0.jsonl')
    camera[1], camera[2] = camera[2], camera[1]
    jsonl(fixture['root']/'raw/cam0.jsonl', camera)
    poses, info = poses[:1]+poses[2:], info[:1]+info[2:]
    for i, row in enumerate(info):
        row['observed_monotonic_ns'] = 1_025_000_000+i*10_000_000
    jsonl(fixture['root']/'raw/pose.jsonl', poses)
    jsonl(fixture['root']/'raw/info.jsonl', info)
    (fixture['root']/'logs/imu_traj.txt').write_text(''.join(
        str(Decimal(row['state_ns'])/Decimal(10**9))+' '+
        ' '.join(map(str,row['position']+row['quaternion_xyzw']))+'\n' for row in poses))
    rows, projected, evidence = runtime.normalize(fixture['root'], fixture['data'],
                                                  fixture['install'], fixture['config'])
    assert [row['source_frame_id'] for row in rows] == [0, *range(2, 12)]
    assert [row['state_ns'] for row in rows] == [row['state_ns'] for row in poses]
    assert evidence['native_camera_callbacks']['missing_count'] == 1
    lag = evidence['camera_lag_at_info_receipt']
    assert lag['max'] == 2  # prefix maximum is camera 2, not late camera 1
    assert lag['source_time_inversions'] == 1
    assert lag['estimator_queue_latency_s'] is None
    assert runtime.jsonl(path) == published
    assert evaluate_rows(rows, read_reference(fixture['archive']), tolerance_ns=0) == evaluate_rows(
        projected, read_reference(fixture['archive']), tolerance_ns=0)


@pytest.mark.parametrize('change', ['unknown_time', 'duplicate_time', 'float_time'])
def test_normalization_refuses_unbound_or_duplicate_camera_publications(fixture, change):
    raw_run(fixture)
    path = fixture['root']/'raw/published.jsonl'
    published = runtime.jsonl(path)
    cameras = [row for row in published if row['stream'] == 'cam0']
    if change == 'float_time':
        cameras[0]['state_ns'] = float(cameras[0]['state_ns'])
    else:
        cameras[1]['state_ns'] = cameras[0]['state_ns'] if change == 'duplicate_time' else cameras[1]['state_ns']+1
    jsonl(path, published)
    with pytest.raises(ValueError):
        runtime.normalize(fixture['root'], fixture['data'], fixture['install'], fixture['config'])


def test_normalization_reports_explicit_native_reinitialization_configuration(fixture):
    raw_run(fixture)
    config = dict(fixture['config'], native_parameters=dict(automatic_reinitialization=True))
    _, _, observations = runtime.normalize(fixture['root'], fixture['data'], fixture['install'], config)
    assert observations['automatic_reinitialization'] is True
    assert observations['native_reset_count'] is None  # configuration is not an observed reset


def test_explicit_fault_scores_epochs_separately_without_baseline_continuity(fixture):
    poses, _ = raw_run(fixture)
    rows, _, observations = runtime.normalize(fixture['root'], fixture['data'], fixture['install'], fixture['config'])
    for row in rows[6:]:
        row['epoch'] = 1
    score = evaluate_rows(rows, read_reference(fixture['archive']), tolerance_ns=0)
    assert score['epoch_count'] == score['scored_epoch_count'] == 2
    observations['tracking_loss_observations'] = [dict(state_ns=poses[5]['state_ns'], stage=0, quality=0)]
    data = dict(fixture['data'], derivation=dict(experiment_kind='ARTIFICIAL_DISTURBANCE', validation=dict(status='PASS')))
    scope = runtime.check_trajectory_scope(data, score, observations)
    assert scope['numerical_scope'] == 'PER_EPOCH_FAULT_OBSERVATION'
    assert scope['baseline_continuity_status'] == 'NOT_APPLICABLE'
    assert scope['tracking_loss_observation_count'] == 1
    assert scope['combined_epoch_score'] is None
    assert scope['recovery_status'] == 'NOT_RUN_BY_NUMERICAL_SCORING'
    with pytest.raises(ValueError, match='one complete scored baseline epoch'):
        runtime.check_trajectory_scope(fixture['data'], score, observations)
    # A partially scored extra epoch must not disappear from baseline gates.
    score['scored_epoch_count'] = 1
    with pytest.raises(ValueError, match='one complete scored baseline epoch'):
        runtime.check_trajectory_scope(fixture['data'], score, observations)


def test_original_baseline_keeps_tracking_loss_gate_and_faults_need_validated_input():
    score = dict(epoch_count=1, scored_epoch_count=1)
    observations = dict(tracking_loss_observations=[dict(stage=0)])
    with pytest.raises(ValueError, match='native left kTracking'):
        runtime.check_trajectory_scope({}, score, observations)
    data = dict(derivation=dict(experiment_kind='ARTIFICIAL_DISTURBANCE', validation=dict(status='FAIL')))
    with pytest.raises(ValueError, match='validated explicit artificial disturbance'):
        runtime.check_trajectory_scope(data, score, observations)
    data['derivation']['validation']['status'] = 'PASS'
    with pytest.raises(ValueError, match='at least one separately scored epoch'):
        runtime.check_trajectory_scope(data, dict(epoch_count=2, scored_epoch_count=0), observations)


def test_info_and_pose_have_independent_ros_publication_sequences(fixture):
    poses, info = raw_run(fixture)
    # roscpp supplies one counter per topic. Info is published during
    # initialization too, so its count is ahead of the tracking-only poses.
    for row in info:
        row['native_sequence'] += 7
    jsonl(fixture['root']/'raw/info.jsonl', info)
    rows, _, evidence = runtime.normalize(fixture['root'], fixture['data'],
                                          fixture['install'], fixture['config'])
    assert len(rows) == len(poses)
    assert evidence['native_info_sequence_gaps'] == []
    assert evidence['pose_topic_sequence_gaps'] == []
    assert evidence['pose_info_association'].startswith('exact integer')


def test_pose_publication_sequence_gap_is_retained(fixture):
    poses, _ = raw_run(fixture)
    for row in poses[4:]:
        row['native_sequence'] += 1
    jsonl(fixture['root']/'raw/pose.jsonl', poses)
    _, _, evidence = runtime.normalize(fixture['root'], fixture['data'],
                                        fixture['install'], fixture['config'])
    assert evidence['pose_topic_sequence_gaps'] == [[4, 6]]


@pytest.mark.parametrize('change',['frame','info_missing','text_corrupt','duplicate_info'])
def test_normalization_refuses_inconsistent_native_evidence(fixture,change):
    poses,info=raw_run(fixture)
    if change=='frame':
        poses[0]['frame_id']='camera'
        jsonl(fixture['root']/'raw/pose.jsonl',poses)
    elif change=='info_missing':
        jsonl(fixture['root']/'raw/info.jsonl',info[1:])
    elif change=='duplicate_info':
        jsonl(fixture['root']/'raw/info.jsonl',[info[0],*info])
    else:
        p=fixture['root']/'logs/imu_traj.txt'
        p.write_text(p.read_text().replace('0.0 0.0 0.0','99.0 0.0 0.0',1))
    with pytest.raises(ValueError):
        runtime.normalize(fixture['root'],fixture['data'],fixture['install'],fixture['config'])


def test_tracking_loss_is_recorded_without_inventing_native_reset(fixture):
    poses,info=raw_run(fixture)
    # A final nontracking native Info is not emitted as a valid pose.
    info[-1]['stage']=3
    info[-1]['processing_time_s']=900
    jsonl(fixture['root']/'raw/info.jsonl',info)
    jsonl(fixture['root']/'raw/pose.jsonl',poses[:-1])
    text=fixture['root']/'logs/imu_traj.txt'
    text.write_text('\n'.join(text.read_text().splitlines()[:-1])+'\n')
    _,projected,evidence=runtime.normalize(fixture['root'],fixture['data'],
                                           fixture['install'],fixture['config'])
    assert evidence['tracking_loss_observations'][0]['stage']==3
    assert evidence['frontend_processing_s']['samples']==len(info)-1
    assert evidence['frontend_processing_s']['p95'] < 1
    assert all(row['localization_epoch']==0 and row['reset_event'] is None for row in projected)


def test_observer_loss_does_not_renumber_source_camera_indices(fixture):
    poses, info = raw_run(fixture)
    # An independent observer can lose one camera input while receiving native
    # output. That cannot change the original frame ID attached to each pose.
    camera = runtime.jsonl(fixture['root']/'raw/cam0.jsonl')
    jsonl(fixture['root']/'raw/cam0.jsonl', camera[:4]+camera[5:])
    rows, _, evidence = runtime.normalize(fixture['root'],fixture['data'],
                                          fixture['install'],fixture['config'])
    assert [row['source_frame_id'] for row in rows] == list(range(len(poses)))
    assert evidence['native_camera_callbacks']['input_count'] == len(poses)


@pytest.mark.parametrize('partial', [False, True])
def test_failed_native_keeps_primary_failure_and_audited_scores(fixture,monkeypatch,partial):
    def fake_run(command, **kwargs):
        if command[:2] != ['docker', 'run']:
            return subprocess.CompletedProcess(command, 0, stdout='')
        assert command[:2] == ['docker','run']
        raw_run(fixture)
        if partial:
            # Retain a real prefix of every output, as an early native crash can
            # leave incomplete publication/receipt logs and still valid poses.
            for path in (fixture['root']/'raw').glob('*.jsonl'):
                path.write_text('\n'.join(path.read_text().splitlines()[:7])+'\n')
            path = fixture['root']/'logs/imu_traj.txt'
            path.write_text('\n'.join(path.read_text().splitlines()[:7])+'\n')
        write_json(fixture['root']/'execution.json',dict(
            failure='native exited during playback: 139',
            exits=dict(native=139, player=-2, observer=0)))
        return subprocess.CompletedProcess(command,1)
    monkeypatch.setattr(runtime.subprocess,'run',fake_run)
    result = runtime.run(fixture['config'])
    assert result['status'] == 'FAILED'
    assert result['failure_code'] == 'BACKEND_EXECUTION_FAILED'
    assert result['stage_result'] == 'FAILED_RUN_WITH_AUDITED_NUMERICAL_OUTPUT'
    assert result['same_run_metrics_exact'] is True
    score = json.loads((fixture['root']/'evaluation.json').read_text())
    assert score['scored_epoch_count'] == 1
    assert score['raw_pose_count'] == (7 if partial else 12)
    if partial:
        assert result['input_reconciliation']['status'] == 'FAILED'
        assert result['analysis_failure']['failure_code'] == 'NATIVE_TAIL_MISSING'
        assert score['missing_after_last'] == 5
    assert result['online_trajectory'] == 'online/trajectory.json'


@pytest.mark.parametrize('failed_process', [None, 'native', 'player', 'observer', 'master'])
def test_every_started_process_requires_a_clean_exit(fixture, monkeypatch, failed_process):
    def fake_run(command, **kwargs):
        if command[:2] != ['docker', 'run']:
            return subprocess.CompletedProcess(command, 0, stdout='')
        raw_run(fixture)
        exits=dict.fromkeys(['native', 'player', 'observer', 'master'], 0)
        if failed_process is not None:
            exits[failed_process]=1
        write_json(fixture['root']/'execution.json', dict(failure=None, exits=exits))
        return subprocess.CompletedProcess(command, 0)
    monkeypatch.setattr(runtime.subprocess, 'run', fake_run)
    result=runtime.run(fixture['config'])
    if failed_process is None:
        assert result['status'] == 'PASS'
        assert result['container_cleanup']['absence_verified']
        assert 'evaluation.json' in result['output_sha256']
        assert any(p.endswith('/native_accuracy.py') for p in result['runtime_sources_sha256'])
        sensor_source=Path(runtime.__file__).with_name('sensors.py')
        assert result['runtime_sources_sha256'][str(sensor_source)] == file_hash(sensor_source)
        return
    assert result['status']=='FAILED'
    assert result['failure_code']=='PROCESS_EXIT_FAILED'


@pytest.mark.parametrize('ledger_mode', ['legacy', 'valid', 'missing', 'no_footer', 'missing_sample', 'bad_value'])
def test_native_imu_ledger_is_required_for_instrumented_sources_and_failure_retains_scores(
        fixture, monkeypatch, ledger_mode):
    from test_native_events import imu_ledger_fixture, write_imu_ledger
    import struct
    if ledger_mode != 'legacy':
        marker = fixture['ws']/'src/SchurVINS/svo/include/svo/rm27_imu_observation.h'
        marker.write_text('TEST_FIXTURE_ONLY: instrumented source identity\n')
        fixture['install']['source_files_sha256'] = snapshot(fixture['ws']/'src')
        write_json(fixture['config']['installation'], fixture['install'])

    def fake_run(command, **kwargs):
        if command[:2] != ['docker', 'run']:
            return subprocess.CompletedProcess(command, 0, stdout='')
        raw_run(fixture)
        if ledger_mode not in ('legacy', 'missing'):
            _, observer, rows = imu_ledger_fixture(fixture['root'], times=fixture['ns'])
            for index, row in enumerate(rows[:-1]):
                values = [1., 2., 3., 4., 5., 6.]
                row['values'] = values
                row['value_bits'] = [struct.unpack('=Q', struct.pack('=d', value))[0] for value in values]
                observer[index].update(gyro=values[:3], accel=values[3:])
            jsonl(fixture['root']/'raw/imu0.jsonl', observer)
            if ledger_mode == 'no_footer':
                rows = rows[:-1]
            elif ledger_mode == 'missing_sample':
                rows.pop(4)
                for index, row in enumerate(rows[:-1]):
                    row['receipt_sequence'] = index
                for name in ('attempted_callbacks', 'retained_records', 'written_records'):
                    rows[-1][name] -= 1
            elif ledger_mode == 'bad_value':
                rows[0]['value_bits'][0] = 0
            write_imu_ledger(fixture['root']/'native-imu-ledger.jsonl', rows)
        write_json(fixture['root']/'execution.json', dict(failure=None,
            exits=dict.fromkeys(['native', 'player', 'observer', 'master'], 0)))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(runtime.subprocess, 'run', fake_run)
    result = runtime.run(fixture['config'])
    expected = 'NOT_RUN' if ledger_mode == 'legacy' else 'PASS' if ledger_mode == 'valid' else 'FAIL'
    assert result['native_imu_callback_ledger_status'] == expected
    assert result['status'] == ('PASS' if expected != 'FAIL' else 'FAILED')
    if expected == 'FAIL':
        assert result['failure_code'] == 'NATIVE_IMU_CALLBACK_INCOMPLETE'
    assert result['same_run_metrics_exact'] is True
    score = json.loads((fixture['root']/'evaluation.json').read_text())
    assert score['raw_pose_count'] == 12 and score['scored_epoch_count'] == 1
    native = json.loads((fixture['root']/'native-observations.json').read_text())
    assert native['imu']['callback_ledger']['status'] == expected
    assert native['imu']['callback_ledger']['downstream_integration'] == 'UNOBSERVED'


@pytest.mark.parametrize('interruption', [subprocess.TimeoutExpired, KeyboardInterrupt])
def test_timeout_removes_only_the_unique_owned_container(fixture,monkeypatch,interruption):
    calls=[]
    def fake_run(command,**kwargs):
        calls.append(command)
        if command[:2]==['docker','run']:
            if interruption is KeyboardInterrupt:
                raise KeyboardInterrupt()
            raise subprocess.TimeoutExpired(command,kwargs['timeout'])
        return subprocess.CompletedProcess(command,0,stdout='')
    monkeypatch.setattr(runtime.subprocess,'run',fake_run)
    if interruption is KeyboardInterrupt:
        with pytest.raises(KeyboardInterrupt):
            runtime.run(fixture['config'])
        result=json.loads((fixture['root']/'run.json').read_text())
        assert result['failure_code']=='EXECUTION_INTERRUPTED'
    else:
        result=runtime.run(fixture['config'])
    assert result['status']=='FAILED'
    owned=calls[0][calls[0].index('--name')+1]
    assert owned.startswith('rm27-schur-') and len(owned)==len('rm27-schur-')+32
    assert calls[1]==['docker','rm','-f',owned]
    assert calls[2]==['docker','ps','--all','--filter','name=^/'+owned+'$', '--format','{{.ID}}']
    assert result['container_cleanup']['absence_verified']
    assert (fixture['root']/'run.json').exists()
    assert '--network' in calls[0] and 'none' in calls[0]
    assert not any(str(fixture['archive']) in item for item in calls[0])


@pytest.mark.parametrize('inspect_exit, remaining', [(0, 'still-running-id\n'), (1, '')])
def test_failed_timeout_cleanup_does_not_finalize_mutable_outputs(fixture, monkeypatch, inspect_exit, remaining):
    def fake_run(command, **kwargs):
        if command[:2] == ['docker', 'run']:
            raise subprocess.TimeoutExpired(command, kwargs['timeout'])
        if command[:2] == ['docker', 'rm']:
            return subprocess.CompletedProcess(command, 1)
        return subprocess.CompletedProcess(command, inspect_exit, stdout=remaining)
    monkeypatch.setattr(runtime.subprocess, 'run', fake_run)
    result = runtime.run(fixture['config'])
    assert result['status'] == 'FAILED'
    assert result['failure_code'] == 'EXECUTION_INTERRUPTED'
    assert not result['container_cleanup']['absence_verified']
    assert result['artifact_finalization'] == 'NOT_FROZEN_CONTAINER_CLEANUP_UNVERIFIED'
    assert 'output_sha256' not in result
    saved = json.loads((fixture['root']/'run.json').read_text())
    assert saved == result


@pytest.mark.parametrize('remaining', ['', 'still-running-id\n'])
def test_docker_nonzero_without_execution_record_preserves_failure_and_checks_absence(fixture, monkeypatch, remaining):
    calls = []
    def fake_run(command, **kwargs):
        calls.append(command)
        if command[:2] == ['docker', 'run']:
            return subprocess.CompletedProcess(command, 125)
        return subprocess.CompletedProcess(command, 0, stdout=remaining)
    monkeypatch.setattr(runtime.subprocess, 'run', fake_run)
    result = runtime.run(fixture['config'])
    assert result['status'] == 'FAILED'
    assert result['failure_code'] == 'BACKEND_EXECUTION_FAILED'
    assert result['exit_code'] == 125
    assert len(calls) == 3
    assert result['container_cleanup']['absence_verified'] == (not remaining)
    assert ('output_sha256' in result) == (not remaining)
    assert not (fixture['root']/'evaluation.json').exists()


def test_inner_shutdown_reaps_child_that_exited_between_poll_and_signal(monkeypatch):
    class Child:
        pid, code = 123, None
        def poll(self):
            return self.code
        def wait(self):
            assert self.code is not None
            return self.code
    child = Child()
    def exited(pid, signum):
        assert pid == child.pid and signum == signal.SIGINT
        child.code = 0
        raise OSError(errno.ESRCH, 'already exited')
    monkeypatch.setattr(run_inner.os, 'killpg', exited)
    assert run_inner.stop_process_group(child) == 0


def test_inner_shutdown_escalates_with_a_bound_and_never_waits_on_live_child(monkeypatch):
    class Child:
        pid = 123
        def poll(self):
            return None
        def wait(self):
            pytest.fail('wait on a live process would be unbounded on Python 2')
    clock = iter(range(0, 100*10**9, 5*10**9))
    signals = []
    monkeypatch.setattr(run_inner, 'monotonic_ns', lambda: next(clock))
    monkeypatch.setattr(run_inner.time, 'sleep', lambda _: None)
    monkeypatch.setattr(run_inner.os, 'killpg', lambda pid, sig: signals.append(sig))
    with pytest.raises(RuntimeError, match='did not exit'):
        run_inner.stop_process_group(Child())
    assert signals == [signal.SIGINT, signal.SIGKILL]


@pytest.fixture
def propagation_binding(fixture):
    import yaml
    source = fixture['ws']/'src/SchurVINS'
    calibration = source/'svo_ros/param/calib/euroc_mono.yaml'
    calibration.parent.mkdir(parents=True)
    identity = dict(data=np.eye(4).reshape(-1).tolist())
    calibration.write_text(yaml.safe_dump(dict(imu_params=dict(delay_imu_cam=0.),
        cameras=[dict(T_B_C=identity)])))
    interface = source/'svo_ros/src/svo_interface.cpp'
    interface.parent.mkdir(parents=True)
    interface.write_text('msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z\n'
        'msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z\n'
        'const ImuMeasurement m(msg->header.stamp.toSec(), omega_imu, lin_acc_imu)\n')
    archive = fixture['inputs']/'sensors.zip'
    # Replace the fixture's intentionally invalid placeholder calibrations.
    with zipfile.ZipFile(archive) as z:
        payloads={name:z.read(name) for name in z.namelist()}
    payloads['mav0/imu0/sensor.yaml']=yaml.safe_dump(dict(sensor_type='imu', T_BS=identity)).encode()
    payloads['mav0/cam0/sensor.yaml']=yaml.safe_dump(dict(T_BS=identity)).encode()
    with zipfile.ZipFile(archive, 'w') as z:
        for name, content in payloads.items():z.writestr(name,content)
    import hashlib
    data=copy.deepcopy(fixture['data'])
    data['calibration_sha256']={sensor:hashlib.sha256(payloads['mav0/'+sensor+'/sensor.yaml']).hexdigest()
                                for sensor in ('imu0','cam0')}
    return fixture, data, calibration, interface


def test_propagation_binding_checks_sealed_source_axes_clock_and_full_ledger(propagation_binding):
    fixture,data,_,_=propagation_binding
    binding=runtime.propagation_input_contract(fixture['inputs'],data,fixture['ws'],fixture['config'],
                                             dict(imu0=dict(content_exact=True)))
    assert binding['status']=='PASS' and binding['timestamp_offset_ns']==0
    assert binding['measurement_kind']=='RAW_SPECIFIC_FORCE' and binding['native_imu_frame']=='imu'
    assert binding['source_imu_id']=='EuRoC:'+data['original_archive_sha256']+':imu0'
    assert [row['state_ns'] for row in binding['source_rows']]==fixture['ns']
    assert len(binding['evidence_sha256'])==4


@pytest.mark.parametrize('change', ['delayed','extrinsic','callback','hash','unreconciled','uzh'])
def test_propagation_binding_keeps_unknown_axes_delay_or_inputs_unqualified(propagation_binding, change):
    import yaml
    fixture,data,calibration,interface=propagation_binding
    reconciliation=dict(imu0=dict(content_exact=True))
    if change in ('delayed','extrinsic'):
        settings=yaml.safe_load(calibration.read_text())
        if change=='delayed':settings['imu_params']['delay_imu_cam']=.01
        else:settings['cameras'][0]['T_B_C']['data'][3]=.1
        calibration.write_text(yaml.safe_dump(settings))
    elif change=='callback':interface.write_text('unqualified converted callback')
    elif change=='hash':data['calibration_sha256']['imu0']='a'*64
    elif change=='unreconciled':reconciliation['imu0']['content_exact']=False
    else:data['dataset']='UZH_FPV'
    binding=runtime.propagation_input_contract(fixture['inputs'],data,fixture['ws'],fixture['config'],reconciliation)
    assert binding['status']=='NOT_RUN' and binding['reason']
    assert 'source_rows' not in binding


from test_schurvins_transport import sealed_native_recording


@pytest.fixture
def sensor_recording_fixture(fixture,sealed_native_recording,tmp_path):
    from tools.schurvins.dataset import prepare_recording
    source,source_config,events,parameters=sealed_native_recording
    native_input=tmp_path/'sensor-native-input'
    data=prepare_recording(source,native_input,parameters)
    value=dict(fixture)
    value['inputs']=native_input;value['data']=data
    value['config']=dict(fixture['config'],purpose='SEALED_SENSOR_RECORDING_REPLAY',
        sequence='EXPLICIT_DOWN_MOCK',input=str(native_input),reference_archive=None)
    value['ns']=[event['timestamp']['raw_value'] for event in events if event['kind']=='CAMERA']
    value['points']=[[i*.1,0,0] for i in range(len(value['ns']))]
    return value


def test_recording_preflight_uses_sealed_source_calibration_and_has_no_implicit_truth(sensor_recording_fixture):
    value=sensor_recording_fixture
    checked=runtime.preflight(value['config'])
    assert checked[-1] is None and checked[1]['dataset']=='SENSOR_RECORDING'
    assert checked[1]['source_camera_id']=='down' and not value['root'].exists()
    binding=runtime.propagation_input_contract(value['inputs'],value['data'],value['ws'],value['config'],
        dict(imu0=dict(content_exact=True)))
    assert binding['status']=='NOT_RUN' and 'SOURCE_RECORDING' in binding['reason']


def test_recording_preflight_refuses_implicit_public_truth_or_mode(sensor_recording_fixture):
    value=sensor_recording_fixture
    for change in [dict(reference_archive=str(value['archive'])),dict(mode='stereo')]:
        with pytest.raises(ValueError):runtime.preflight(dict(value['config'],**change))
    assert not value['root'].exists()


def test_recording_replay_without_native_metric_evidence_cannot_claim_native_acceptance(sensor_recording_fixture,monkeypatch):
    value=sensor_recording_fixture
    def fake_run(command,**kwargs):
        if command[:2]==['docker','run']:
            raw_run(value)
            write_json(value['root']/'execution.json',dict(failure=None,
                exits=dict.fromkeys(['native','player','observer','master'],0)))
        return subprocess.CompletedProcess(command,0,stdout='')
    monkeypatch.setattr(runtime.subprocess,'run',fake_run)
    value['config']['state_output_policy']=dict(max_visual_age_ns=100_000_000,max_imu_gap_ns=20_000_000,
        max_receive_age_ns=100_000_000,max_visual_receive_age_ns=100_000_000,max_buffer_samples=100)
    result=runtime.run(value['config'])
    assert result['status']=='FAILED' and result['failure_code']=='NATIVE_INITIALIZATION_EVIDENCE_UNAVAILABLE'
    assert result['recording_transport_status']=='PASS'
    assert result['recording_interface_status']=='NOT_RUN'
    assert result['same_run_metrics_exact'] is None
    evaluation=json.loads((value['root']/'evaluation.json').read_text())
    assert evaluation['status']=='NOT_RUN' and evaluation['reason']=='NO_REFERENCE_SUPPLIED'
    assert 'ate' not in evaluation and result['container_cleanup']['absence_verified']
    states=json.loads((value['root']/'native-state-output.json').read_text())
    assert states['status']=='NOT_RUN' and states['corrections']==states['outputs']==[]


@pytest.mark.parametrize('failure,code', [
    (None, 'NATIVE_OUTPUT_EMPTY'), ('native', 'PROCESS_EXIT_FAILED'),
    ('execution', 'BACKEND_EXECUTION_FAILED'), ('input', 'INPUT_RECONCILIATION_FAILED'),
    ('cleanup', 'CONTAINER_CLEANUP_UNVERIFIED'), ('source', 'HASH_MISMATCH'),
    ('validator_source', 'HASH_MISMATCH'),
    ('cleanup_errors', 'NATIVE_OUTPUT_EMPTY')])
def test_recording_transport_is_separate_from_empty_native_output(
        sensor_recording_fixture, monkeypatch, failure, code):
    value=sensor_recording_fixture
    from rm27.perception.vision.localization.experiments import common
    original_hash=common.file_hash
    executed=False
    def observed_hash(path):
        if failure=='validator_source' and executed and Path(path)==Path(runtime.__file__).with_name('sensors.py'):
            return '0'*64
        return original_hash(path)
    monkeypatch.setattr(common,'file_hash',observed_hash)
    def fake_run(command, **kwargs):
        nonlocal executed
        if command[:2]==['docker','run']:
            executed=True
            raw_run(value)
            (value['root']/'raw/pose.jsonl').write_text('')
            (value['root']/'logs/imu_traj.txt').write_text('')
            exits=dict.fromkeys(['native','player','observer','master'],0)
            if failure=='native':exits['native']=1
            execution=dict(failure='native failure' if failure=='execution' else None,
                           exits=exits, cleanup_errors={})
            if failure=='cleanup_errors':execution['cleanup_errors']['observer']='not reaped'
            write_json(value['root']/'execution.json',execution)
            if failure=='input':
                path=value['root']/'raw/imu0.jsonl'
                path.write_text('\n'.join(path.read_text().splitlines()[1:])+'\n')
            if failure=='source':
                (value['ws']/'devel/lib/svo_ros/svo_node').write_bytes(b'changed after execution')
        remaining='still-running\n' if failure=='cleanup' and command[:2]==['docker','ps'] else ''
        return subprocess.CompletedProcess(command,0,stdout=remaining)
    monkeypatch.setattr(runtime.subprocess,'run',fake_run)
    result=runtime.run(value['config'])
    assert result['status']=='FAILED' and result['failure_code']==code
    assert result['recording_transport_status']==('PASS' if failure is None else 'FAIL')
    assert result['recording_interface_status']=='NOT_RUN'
    assert result['estimator_reproduction']=='NOT_RUN_NO_REFERENCE'
    assert result['accuracy_qualification']=='NOT_RUN'
    assert result['physical_camera_imu_qualification'] is False
    assert not (value['root']/'evaluation.json').exists()
    if failure is None:
        assert all(stream['content_exact'] for stream in result['input_reconciliation'].values())
        assert all(not stream['native_imu_ingestion_verified']
                   for stream in result['input_reconciliation'].values())


@pytest.mark.parametrize('cleanup,first_status',[(True,'PASS'),(True,'FAILED'),(False,'FAILED')])
def test_recording_session_uses_existing_native_lifecycle_and_blocks_after_unverified_cleanup(
        sensor_recording_fixture,tmp_path,monkeypatch,cleanup,first_status):
    value=sensor_recording_fixture
    session=tmp_path/'session'
    segments=[dict(value['config'],run_id='MOCK_ONLY_'+str(i),output=str(session/str(i))) for i in range(2)]
    calls=[]
    def fake_run(config):
        calls.append(config['run_id'])
        return dict(status=first_status if len(calls)==1 else 'PASS',execution_started=True,
                    container_cleanup=dict(absence_verified=cleanup if len(calls)==1 else True))
    monkeypatch.setattr(runtime,'run',fake_run)
    report=runtime.run_recordings(dict(schema_version=1,backend='SchurVINS',purpose='SEALED_SENSOR_RECORDING_SESSION',
        run_id='MOCK_SESSION_LIFECYCLE_ONLY',output=str(session),segments=segments))
    assert report['trajectory_stitching'] is False
    if not cleanup:
        assert len(calls)==1 and report['status']=='FAIL'
        assert report['remaining_segments']=='NOT_RUN_CLEANUP_BARRIER'
    else:
        assert len(calls)==2 and [s['session_epoch'] for s in report['segments']]==[0,1]
        assert report['status']==('PASS' if first_status=='PASS' else 'PARTIAL')
    assert json.loads((session/'session.json').read_text())==report
    with pytest.raises(ValueError,match='OUTPUT_EXISTS'):
        runtime.run_recordings(dict(schema_version=1,backend='SchurVINS',purpose='SEALED_SENSOR_RECORDING_SESSION',
            run_id='MOCK_SESSION_LIFECYCLE_ONLY',output=str(session),segments=segments))


def test_native_session_switch_binds_front_and_down_to_distinct_separate_process_epochs(
        sensor_recording_fixture,sealed_native_recording,tmp_path,monkeypatch):
    from rm27.perception.vision.localization.experiments import sensors
    from tools.schurvins.dataset import prepare_recording
    value=sensor_recording_fixture
    source,config,events,parameters=sealed_native_recording
    config=copy.deepcopy(config);events=copy.deepcopy(events)
    config['selected_camera_ids']=['front'];config['config_version']='FRONT_EXPLICIT_CONFIG'
    config['cameras'][0]['calibration']=copy.deepcopy(config['cameras'][1]['calibration'])
    config['cameras'][0]['calibration']['version']='FRONT_EXPLICIT_CALIBRATION'
    config['cameras'][0]['calibration']['T_imu_camera']['matrix'][0][3]=.15
    for event in events:
        event['config_version']=config['config_version']
        if event['kind']=='CAMERA':event.update(sensor_id='front',frame_id='camera0')
    front=tmp_path/'front-recording';front_native=tmp_path/'front-native'
    sensors.write_recording(front,config,events,source)
    prepare_recording(front,front_native,parameters)
    session=tmp_path/'switch-session'
    segments=[dict(value['config'],input=str(front_native),run_id='MOCK_FRONT',output=str(session/'front')),
              dict(value['config'],run_id='MOCK_DOWN',output=str(session/'down'))]
    calls=[]
    def fake_run(config):
        calls.append(config['run_id'])
        return dict(status='PASS',execution_started=True,container_cleanup=dict(absence_verified=True))
    monkeypatch.setattr(runtime,'run',fake_run)
    report=runtime.run_recordings(dict(schema_version=1,backend='SchurVINS',purpose='SEALED_SENSOR_RECORDING_SESSION',
        run_id='MOCK_SWITCH_ONLY',output=str(session),segments=segments))
    assert calls==['MOCK_FRONT','MOCK_DOWN']
    assert [r['camera_id'] for r in report['segments']]==['front','down']
    assert [r['session_epoch'] for r in report['segments']]==[0,1]
    assert len({r['calibration_sha256'] for r in report['segments']})==2
    assert all(r['reset_boundary']=='NATIVE_SESSION_RESTART' for r in report['segments'])
    assert report['trajectory_stitching'] is False


def test_native_recording_session_rechecks_next_input_and_keeps_first_failure_evidence(
        sensor_recording_fixture,tmp_path,monkeypatch):
    value=sensor_recording_fixture
    session=tmp_path/'changed-next-session'
    segments=[dict(value['config'],run_id='MOCK_'+str(i),output=str(session/str(i))) for i in range(2)]
    calls=[]
    def fake_run(config):
        calls.append(config['run_id'])
        manifest=value['inputs']/'input.json'
        manifest.write_text(manifest.read_text()+'\n')
        return dict(status='FAILED',execution_started=True,container_cleanup=dict(absence_verified=True))
    monkeypatch.setattr(runtime,'run',fake_run)
    with pytest.raises(ValueError,match='manifest changed'):
        runtime.run_recordings(dict(schema_version=1,backend='SchurVINS',purpose='SEALED_SENSOR_RECORDING_SESSION',
            run_id='MOCK_RECHECK_ONLY',output=str(session),segments=segments))
    report=json.loads((session/'session.json').read_text())
    assert calls==['MOCK_0'] and report['status']=='FAIL'
    assert len(report['segments'])==1 and report['segments'][0]['result']['status']=='FAILED'
    assert report['remaining_segments']=='NOT_RUN'


def test_native_recording_session_preserves_interrupted_current_member_manifest(sensor_recording_fixture,tmp_path,monkeypatch):
    value=sensor_recording_fixture
    session=tmp_path/'interrupted-session'
    segments=[dict(value['config'],run_id='MOCK_'+str(i),output=str(session/str(i))) for i in range(2)]
    def interrupted(config):
        target=Path(config['output']);target.mkdir(parents=True)
        write_json(target/'run.json',dict(status='FAILED',execution_started=True,
            failure_code='EXECUTION_INTERRUPTED',container_cleanup=dict(absence_verified=True)))
        raise KeyboardInterrupt()
    monkeypatch.setattr(runtime,'run',interrupted)
    with pytest.raises(KeyboardInterrupt):
        runtime.run_recordings(dict(schema_version=1,backend='SchurVINS',purpose='SEALED_SENSOR_RECORDING_SESSION',
            run_id='MOCK_INTERRUPTION_ONLY',output=str(session),segments=segments))
    report=json.loads((session/'session.json').read_text())
    assert len(report['segments'])==1 and report['segments'][0]['result']['failure_code']=='EXECUTION_INTERRUPTED'
    assert report['segments'][0]['owned_processes_absent'] is True
    assert report['status']=='FAIL' and report['remaining_segments']=='NOT_RUN'


@pytest.fixture
def derived_propagation_binding(propagation_binding, tmp_path):
    import shutil
    fixture, data, calibration, interface = propagation_binding
    parent=fixture['inputs']
    data['artifacts_sha256']['sensors.zip']=file_hash(parent/'sensors.zip')
    write_json(parent/'input.json',data)
    child=tmp_path/'derived-binding'; child.mkdir()
    shutil.copyfile(parent/'sensors.zip', child/'sensors.zip')
    shutil.copyfile(parent/'expected.jsonl', child/'source-expected.jsonl')
    delivered=runtime.jsonl(parent/'expected.jsonl')
    next(row for row in delivered if row['stream']=='imu0')['gyro'][0] += .1
    jsonl(child/'expected.jsonl',delivered)
    declaration=dict(experiment_kind='ARTIFICIAL_DISTURBANCE', validation=dict(status='PASS'),
        profile=dict(kind='imu_bias',sensor_ids=['imu0']),parent_input_path=str(parent),
        parent_input_sha256=file_hash(parent/'input.json'),parent_artifacts_sha256=data['artifacts_sha256'])
    write_json(child/'derivation.json',declaration)
    derived=dict(data,derivation=declaration,source_counts=data['counts'],
        artifacts_sha256={path.name:file_hash(path) for path in child.iterdir()})
    write_json(child/'input.json',derived)
    return fixture, child, derived


def test_propagation_binding_keeps_original_and_qualified_delivered_ledgers(derived_propagation_binding):
    fixture,child,data=derived_propagation_binding
    binding=runtime.propagation_input_contract(child,data,fixture['ws'],fixture['config'],
                                             dict(imu0=dict(content_exact=True)))
    assert binding['status']=='PASS'
    assert binding['source_rows'][0]['gyro'][0] == 1.
    assert binding['delivered_rows'][0]['gyro'][0] == 1.1
    assert [row['state_ns'] for row in binding['source_rows']] == fixture['ns']
    assert binding['delivery_evidence']['prepared_input_sha256'] == file_hash(child/'input.json')
    assert binding['delivery_evidence']['experiment_kind'] == 'ARTIFICIAL_DISTURBANCE'


@pytest.mark.parametrize('change',['source','delivered','derivation','parent','data','missing'])
def test_propagation_binding_fails_closed_for_invalid_derived_proof(derived_propagation_binding,change):
    fixture,child,data=derived_propagation_binding
    if change=='data':data['derivation']['profile']['kind']='forged'
    else:
        path={'source':child/'source-expected.jsonl','delivered':child/'expected.jsonl',
              'derivation':child/'derivation.json','parent':fixture['inputs']/'input.json',
              'missing':child/'expected.jsonl'}[change]
        if change=='missing':path.unlink()
        else:path.write_text(path.read_text()+'\n')
    binding=runtime.propagation_input_contract(child,data,fixture['ws'],fixture['config'],
                                             dict(imu0=dict(content_exact=True)))
    assert binding['status']=='FAIL' and binding['reason'].startswith('PROPAGATION_DERIVATION_INVALID:')
    assert 'source_rows' not in binding and 'delivered_rows' not in binding


@pytest.mark.parametrize('prior_failure',[False,True])
def test_failed_state_projection_preserves_primary_failure_and_existing_scores(fixture,monkeypatch,prior_failure):
    def fake_run(command,**kwargs):
        if command[:2]==['docker','run']:
            raw_run(fixture)
            exits=dict.fromkeys(['native','player','observer','master'],0)
            if prior_failure:exits['native']=139
            write_json(fixture['root']/'execution.json',dict(failure=None,exits=exits))
            return subprocess.CompletedProcess(command,1 if prior_failure else 0)
        return subprocess.CompletedProcess(command,0,stdout='')
    def refused(*args,**kwargs):
        runtime.require(False,'PROPAGATION_INPUT_INVALID','TEST_FIXTURE_ONLY: invalid delivery proof')
    monkeypatch.setattr(runtime.subprocess,'run',fake_run)
    monkeypatch.setattr(runtime,'project_native_corrections',refused)
    fixture['config']['state_output_policy']=dict(max_visual_age_ns=100_000_000,max_imu_gap_ns=20_000_000,
        max_receive_age_ns=100_000_000,max_visual_receive_age_ns=100_000_000,max_buffer_samples=100)
    result=runtime.run(fixture['config'])
    assert result['status']=='FAILED'
    assert result['failure_code']==('BACKEND_EXECUTION_FAILED' if prior_failure else 'PROPAGATION_INPUT_INVALID')
    assert result['native_state_projection_failure']['failure_code']=='PROPAGATION_INPUT_INVALID'
    assert result['same_run_metrics_exact'] is True
    assert result['stage_result']=='FAILED_RUN_WITH_AUDITED_NUMERICAL_OUTPUT'
    assert json.loads((fixture['root']/'evaluation.json').read_text())['raw_pose_count']==12
    states=json.loads((fixture['root']/'native-state-output.json').read_text())
    assert states['status']=='FAIL' and states['outputs']==[]
    assert states['propagation']['output_complete'] is False
