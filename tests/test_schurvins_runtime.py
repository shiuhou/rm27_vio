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
        return
    assert result['status']=='FAILED'
    assert result['failure_code']=='PROCESS_EXIT_FAILED'


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
