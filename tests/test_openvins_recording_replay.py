"""Mock-process boundary tests; none of these fixtures executes OpenVINS.

The native CSV/JSON fixtures exercise reconciliation and failure reporting. The
mock preparer isolates runner ownership from the separately tested sensor
adapter; a successful test is not estimator or physical Camera+IMU evidence.
"""
from __future__ import annotations

import copy
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time

import pytest

from tools.openvins import recording_replay as replay


CAMERA_NS = [1_403_715_273_262_142_976 + i * 50_000_000 for i in range(5)]
MOCK_PROVENANCE = dict(kind='MOCK', description='Runner test fixture; not estimator output')


def _json(path, value):
    Path(path).write_text(json.dumps(value, allow_nan=False) + '\n')


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _csv(path, rows, fieldnames):
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _fixture_payload():
    """Five mock callbacks: initialize at index 1; visual updates at 1 and 3."""
    frames = [dict(timestamp_ns=str(stamp), initialized=str(int(i >= 1)),
        state_matches_camera='1', finite_posterior='1', pose_emitted=str(int(i >= 1)),
        msckf_updates=str(int(i in (1, 3))), slam_updates='0', imu_delivered=str(2 + 2*i),
        queue_pending_frames='0',
        scheduled_ns=str(2_000_000_000 + i*50_000_000),
        start_ns=str(2_000_000_000 + i*50_000_000),
        end_ns=str(2_002_000_000 + i*50_000_000)) for i, stamp in enumerate(CAMERA_NS)]
    poses = [dict(timestamp_ns=str(stamp), tx='1', ty='2', tz='3',
        qw='1', qx='0', qy='0', qz='0', vx='0', vy='0', vz='0',
        camera_to_imu_offset_s='0.001',
        timestamp_imu_seconds=f'{stamp*1e-9 + .001:.9f}',
        visual_update=str(int(i in (1, 3))))
        for i, stamp in enumerate(CAMERA_NS) if i >= 1]
    summary = dict(upstream_commit=replay.PIN, backend='OpenVINS',
        status='EXECUTION_COMPLETED', mode='mono_imu', rate=1,
        quaternion_convention='native_OpenVINS_JPL_q_GtoI_reordered_wxyz',
        position_convention='p_IinG_m', camera_input=5, selected_frames=5,
        imu_input=12, imu_delivered=12, poses=4, accepted_visual_update_frames=2,
        msckf_updates=2, slam_updates=0, first_initialized_timestamp_ns=CAMERA_NS[1],
        last_visual_update_timestamp_ns=CAMERA_NS[3], maximum_visual_update_gap_ns=100_000_000,
        nonfinite_posteriors=0, state_timestamp_failures=0)
    prepared = dict(truth_included=False, counts=dict(cam0=5, imu0=12),
        camera_timestamps_ns=CAMERA_NS.copy(), dataset_path='dataset',
        config_path='config/estimator_config.yaml', source_provenance=MOCK_PROVENANCE.copy(),
        source_epoch=7, camera_id='mock-camera', imu_id='mock-imu', clock_domain='mock-clock')
    return dict(summary=summary, frames=frames, poses=poses, prepared=prepared)


def _save_native(path, payload):
    path.mkdir(parents=True, exist_ok=True)
    _json(path/'summary.json', payload['summary'])
    _csv(path/'frames.csv', payload['frames'], _fixture_payload()['frames'][0].keys())
    _csv(path/'poses.csv', payload['poses'], _fixture_payload()['poses'][0].keys())


@pytest.fixture
def native_case(tmp_path):
    payload = _fixture_payload()
    payload['path'] = tmp_path/'mock-native'
    _save_native(payload['path'], payload)
    return payload


def test_native_reconciliation_preserves_mock_scope_and_separate_visual_support(native_case):
    report = replay.diagnose(native_case['path'], native_case['prepared'], rate=1)
    assert report['reconciliation'] == 'PASS'
    assert report['poses'] == 4
    assert report['trajectory_status'] == 'OBSERVED_POSE_CONTINUITY'
    assert report['accepted_visual_update_frames'] == 2
    assert report['maximum_visual_support_gap_ns'] == 100_000_000
    assert report['initialization_prefix_frames'] == 1
    assert report['callback_service_ms']['p95'] == pytest.approx(2)
    assert report['nominal_replay_deadline_ms'] == pytest.approx(50)
    assert report['source_provenance']['kind'] == 'MOCK'
    assert report['source_epoch'] == 7
    assert report['accuracy'] == 'NOT_EVALUATED_NO_REFERENCE'
    assert report['tracking_health'] == 'UNKNOWN'
    assert report['native_reset_count'] is None
    assert not report['operational_validity']
    assert not report['physical_camera_imu_qualification']


def test_uninitialized_exit_zero_remains_no_trajectory(native_case):
    for row in native_case['frames']:
        row.update(initialized='0', pose_emitted='0', msckf_updates='0')
    native_case['poses'] = []
    native_case['summary'].update(poses=0, accepted_visual_update_frames=0, msckf_updates=0,
        first_initialized_timestamp_ns=0, last_visual_update_timestamp_ns=0,
        maximum_visual_update_gap_ns=0)
    _save_native(native_case['path'], native_case)
    report = replay.diagnose(native_case['path'], native_case['prepared'], rate=1)
    assert report['trajectory_status'] == 'NOT_INITIALIZED'
    assert not report['initialized'] and report['poses'] == 0
    assert report['maximum_visual_support_gap_ns'] is None
    assert report['initialization_prefix_frames'] is None
    assert not report['operational_validity']


def test_initialized_without_poses_retains_entire_missing_tail(native_case):
    for row in native_case['frames'][1:]:
        row.update(state_matches_camera='0', pose_emitted='0', msckf_updates='0')
    native_case['poses'] = []
    native_case['summary'].update(poses=0, accepted_visual_update_frames=0, msckf_updates=0,
        last_visual_update_timestamp_ns=0, maximum_visual_update_gap_ns=0,
        state_timestamp_failures=4)
    _save_native(native_case['path'], native_case)
    report = replay.diagnose(native_case['path'], native_case['prepared'], rate=1)
    assert report['trajectory_status'] == 'NO_POSES'
    assert report['missing_pose_timestamps_after_initialization'] == CAMERA_NS[1:]
    assert report['maximum_visual_support_gap_ns'] == 150_000_000


def test_sparse_native_poses_do_not_hide_recovered_gap(native_case):
    native_case['frames'][2].update(state_matches_camera='0', pose_emitted='0')
    del native_case['poses'][1]
    native_case['summary'].update(poses=3, state_timestamp_failures=1)
    _save_native(native_case['path'], native_case)
    report = replay.diagnose(native_case['path'], native_case['prepared'], rate=1)
    assert report['trajectory_status'] == 'GAPS_AFTER_INITIALIZATION'
    assert report['missing_pose_timestamps_after_initialization'] == [CAMERA_NS[2]]
    assert report['poses'] == 3 and report['accepted_visual_update_frames'] == 2


def test_visual_update_loss_includes_final_interval_even_with_continuous_poses(native_case):
    native_case['frames'][3]['msckf_updates'] = '0'
    native_case['poses'][2]['visual_update'] = '0'
    native_case['summary'].update(msckf_updates=1, accepted_visual_update_frames=1,
        last_visual_update_timestamp_ns=CAMERA_NS[1], maximum_visual_update_gap_ns=0)
    _save_native(native_case['path'], native_case)
    report = replay.diagnose(native_case['path'], native_case['prepared'], rate=1)
    assert report['trajectory_status'] == 'OBSERVED_POSE_CONTINUITY'
    assert report['accepted_visual_update_frames'] == 1
    assert report['maximum_visual_support_gap_ns'] == 150_000_000


def test_unpaced_replay_does_not_invent_a_schedule_deadline(native_case):
    native_case['summary']['rate'] = 0
    for row in native_case['frames']:
        row['scheduled_ns'] = '2000000000'
    _save_native(native_case['path'], native_case)
    report = replay.diagnose(native_case['path'], native_case['prepared'], rate=0)
    assert report['callback_service_ms']['count'] == 5
    assert report['scheduled_completion_lateness_ms'] is None
    assert report['nominal_replay_deadline_ms'] is None
    assert report['nominal_replay_deadline_misses'] is None


def test_half_speed_replay_uses_source_spacing_for_deadline(native_case):
    native_case['summary']['rate'] = .5
    for i, row in enumerate(native_case['frames']):
        row.update(scheduled_ns=str(2_000_000_000 + i*100_000_000),
                   start_ns=str(2_000_000_000 + i*100_000_000),
                   end_ns=str(2_002_000_000 + i*100_000_000))
    _save_native(native_case['path'], native_case)
    report = replay.diagnose(native_case['path'], native_case['prepared'], rate=.5)
    assert report['nominal_replay_deadline_ms'] == pytest.approx(100)
    assert report['nominal_replay_deadline_misses'] == 0


@pytest.mark.parametrize('field,value', [
    ('upstream_commit', '0'*40), ('rate', .5), ('mode', 'stereo_imu'),
    ('poses', 5), ('accepted_visual_update_frames', 3), ('imu_delivered', 11),
    ('first_initialized_timestamp_ns', CAMERA_NS[0]),
    ('last_visual_update_timestamp_ns', CAMERA_NS[4]),
    ('maximum_visual_update_gap_ns', 0), ('state_timestamp_failures', 1),
    ('nonfinite_posteriors', 1), ('msckf_updates', 3),
])
def test_native_summary_cannot_override_callback_witnesses(native_case, field, value):
    native_case['summary'][field] = value
    _save_native(native_case['path'], native_case)
    with pytest.raises(ValueError, match='REPLAY_(OUTPUT_INVALID|INPUT_MISMATCH)'):
        replay.diagnose(native_case['path'], native_case['prepared'], rate=1)


@pytest.mark.parametrize('target,field,value', [
    ('frames', 'timestamp_ns', str(CAMERA_NS[0])),
    ('frames', 'imu_delivered', '1'),
    ('frames', 'end_ns', '0'),
    ('frames', 'scheduled_ns', '0'),
    ('frames', 'queue_pending_frames', '1'),
    ('frames', 'finite_posterior', '0'),
    ('frames', 'initialized', '2'),
    ('poses', 'timestamp_ns', str(CAMERA_NS[0])),
    ('poses', 'qw', '2'),
    ('poses', 'tx', 'nan'),
    ('poses', 'timestamp_imu_seconds', '0'),
    ('poses', 'visual_update', '1'),
])
def test_native_ledger_tampering_is_rejected(native_case, target, field, value):
    native_case[target][1][field] = value
    _save_native(native_case['path'], native_case)
    with pytest.raises(ValueError, match='REPLAY_'):
        replay.diagnose(native_case['path'], native_case['prepared'], rate=1)


def test_missing_native_camera_cannot_pass_via_summary_count(native_case):
    del native_case['frames'][2]
    _save_native(native_case['path'], native_case)
    with pytest.raises(ValueError, match='REPLAY_INPUT_MISMATCH'):
        replay.diagnose(native_case['path'], native_case['prepared'], rate=1)


def _runtime(tmp_path, body):
    """Pin a real executable mock process through the production runtime loader."""
    binary = tmp_path/'mock-openvins-process'
    binary.write_text('#!' + sys.executable + '\n# MOCK PROCESS, NOT OPENVINS\n' + body)
    binary.chmod(0o755)
    build = tmp_path/'mock-build.json'
    _json(build, dict(upstream_commit=replay.PIN, ros_free=True,
                      binary=dict(sha256=_sha(binary)), fixture='MOCK_NOT_ESTIMATOR'))
    runtime = tmp_path/'mock-runtime.json'
    _json(runtime, dict(schema_version=1, mode='local', binary=binary.name,
                        build_manifest=build.name, build_manifest_sha256=_sha(build)))
    return runtime, binary, build


@pytest.mark.parametrize('tamper', ['binary', 'build_receipt', 'image_in_local_mode'])
def test_runtime_identity_rejects_changed_binary_build_or_mode(tmp_path, tamper):
    runtime, binary, build = _runtime(tmp_path, 'raise SystemExit(0)\n')
    descriptor = replay.runtime_descriptor(runtime)
    assert descriptor['binary_sha256'] == _sha(binary)
    if tamper == 'binary':
        binary.write_text(binary.read_text() + '# changed after build\n')
    elif tamper == 'build_receipt':
        build.write_text(build.read_text() + '\n')
    else:
        value = json.loads(runtime.read_text())
        value['image'] = 'sha256:' + 'a'*64
        _json(runtime, value)
    with pytest.raises(ValueError, match='HASH_MISMATCH|REPLAY_RUNTIME_INVALID'):
        replay.runtime_descriptor(runtime)


def _mock_preparer(monkeypatch):
    """Boundary fixture: sensor adapter validation is tested in its own module."""
    from tools.openvins import recording

    def prepare(recording_directory, output, parameters, estimator_config, *, source_epoch=None):
        root = Path(output)
        (root/'dataset').mkdir(parents=True)
        (root/'config').mkdir()
        (root/'dataset/mock-image.png').write_bytes(b'MOCK IMAGE; not a sensor image')
        (root/'config/estimator_config.yaml').write_text('# MOCK tuning; no estimator executes\n')
        payload = copy.deepcopy(_fixture_payload()['prepared'])
        payload['artifacts_sha256'] = {str(p.relative_to(root)): _sha(p)
                                     for p in root.rglob('*') if p.is_file()}
        _json(root/'input.json', payload)
        return payload

    monkeypatch.setattr(recording, 'prepare_recording', prepare)
    # This fixture intentionally lacks physical/source calibration sidecars.
    # Exercise the runner's independent before/after identity binding here;
    # the real adapter checker has separate source-reconstruction tests.
    monkeypatch.setattr(recording, 'check_recording_input',
                        lambda directory: dict(status='MOCK_ADAPTER_BOUNDARY_ONLY'))


def _native_writer_body(*, before='', after=''):
    payload = _fixture_payload()
    payload.pop('prepared')
    return ('import argparse, csv, json, pathlib\n'
            'parser=argparse.ArgumentParser()\n'
            'for key in ("dataset", "config", "output", "rate"): parser.add_argument("--"+key)\n'
            'args=parser.parse_args()\n' + before + '\n'
            'out=pathlib.Path(args.output); out.mkdir()\n'
            'payload=' + repr(payload) + '\n'
            '(out/"summary.json").write_text(json.dumps(payload["summary"]))\n'
            'for name in ("frames", "poses"):\n'
            ' with (out/(name+".csv")).open("w", newline="") as stream:\n'
            '  writer=csv.DictWriter(stream, fieldnames=payload[name][0].keys())\n'
            '  writer.writeheader(); writer.writerows(payload[name])\n' + after + '\n')


def _execute_mock(tmp_path, monkeypatch, body):
    _mock_preparer(monkeypatch)
    runtime, binary, build = _runtime(tmp_path, body)
    output = tmp_path/'result'
    arguments = (tmp_path/'mock-recording', output, {}, tmp_path/'mock-tuning.yaml', runtime)
    return arguments, output, binary, build


def test_mock_process_end_to_end_preserves_report_and_artifact_hashes(tmp_path, monkeypatch):
    args, output, binary, _ = _execute_mock(tmp_path, monkeypatch, _native_writer_body())
    report = replay.replay_recording(*args, rate=1)
    assert report['reconciliation'] == 'PASS' and report['source_provenance']['kind'] == 'MOCK'
    assert not report['operational_validity']
    request = json.loads((output/'request.json').read_text())
    assert request['runtime']['binary_sha256'] == _sha(binary)
    assert request['prepared_manifest_sha256'] == _sha(output/'prepared/input.json')
    assert 'MOCK' in (output/'report.md').read_text()
    artifacts = json.loads((output/'artifacts.json').read_text())['artifacts_sha256']
    assert {'native/summary.json', 'native/frames.csv', 'native/poses.csv',
            'execution.json', 'request.json', 'report.json', 'native.log'} <= artifacts.keys()
    assert all(_sha(output/name) == digest for name, digest in artifacts.items())


def test_nonzero_mock_exit_keeps_partial_outputs_log_and_failure_seals(tmp_path, monkeypatch):
    body = _native_writer_body(after='print("MOCK native failure", flush=True)\nraise SystemExit(7)')
    args, output, _, _ = _execute_mock(tmp_path, monkeypatch, body)
    with pytest.raises(ValueError, match='REPLAY_NATIVE_FAILED'):
        replay.replay_recording(*args, rate=1)
    report = json.loads((output/'report.json').read_text())
    execution = json.loads((output/'execution.json').read_text())
    assert report['status'] == 'FAIL' and not report['operational_validity']
    assert execution['exit_code'] == 7 and not execution['timed_out']
    assert 'MOCK native failure' in (output/'native.log').read_text()
    assert (output/'native/poses.csv').is_file()
    assert 'native/poses.csv' in json.loads((output/'artifacts.json').read_text())['artifacts_sha256']


def test_mock_timeout_preserves_partial_native_files_and_explicit_execution_failure(tmp_path, monkeypatch):
    body = _native_writer_body(after='import time\nprint("MOCK partial outputs", flush=True)\ntime.sleep(30)')
    args, output, _, _ = _execute_mock(tmp_path, monkeypatch, body)
    with pytest.raises(ValueError, match='REPLAY_NATIVE_FAILED'):
        replay.replay_recording(*args, rate=1, timeout_s=.2)
    execution = json.loads((output/'execution.json').read_text())
    assert execution['timed_out'] and execution['exit_code'] != 0
    assert execution['elapsed_s'] < 3
    assert json.loads((output/'report.json').read_text())['status'] == 'FAIL'
    assert 'MOCK partial outputs' in (output/'native.log').read_text()
    assert (output/'native/poses.csv').is_file()
    assert 'native/poses.csv' in json.loads((output/'artifacts.json').read_text())['artifacts_sha256']


def test_output_reuse_never_overwrites_existing_evidence(tmp_path, monkeypatch):
    args, output, _, _ = _execute_mock(tmp_path, monkeypatch, _native_writer_body())
    output.mkdir()
    marker = output/'report.json'
    marker.write_text('original evidence')
    with pytest.raises(FileExistsError):
        replay.replay_recording(*args, rate=1)
    assert marker.read_text() == 'original evidence'


def test_native_input_resealing_cannot_hide_changed_image(tmp_path, monkeypatch):
    tamper = ('root=pathlib.Path(args.dataset).parent\n'
              'image=root/"dataset/mock-image.png"; image.write_bytes(b"CHANGED MOCK INPUT")\n'
              'manifest=root/"input.json"; value=json.loads(manifest.read_text())\n'
              'import hashlib\n'
              'value["artifacts_sha256"]["dataset/mock-image.png"]=hashlib.sha256(image.read_bytes()).hexdigest()\n'
              'manifest.write_text(json.dumps(value))')
    args, output, _, _ = _execute_mock(tmp_path, monkeypatch, _native_writer_body(before=tamper))
    with pytest.raises(ValueError, match='HASH_MISMATCH|REPLAY_.*INVALID|REPLAY_INPUT_'):
        replay.replay_recording(*args, rate=1)
    assert json.loads((output/'report.json').read_text())['status'] == 'FAIL'
    assert (output/'native/summary.json').is_file()


def test_native_runtime_self_modification_cannot_keep_original_provenance(tmp_path, monkeypatch):
    tamper = 'pathlib.Path(__file__).write_text(pathlib.Path(__file__).read_text()+"\\n# MODIFIED MOCK RUNTIME\\n")'
    args, output, _, _ = _execute_mock(tmp_path, monkeypatch, _native_writer_body(after=tamper))
    with pytest.raises(ValueError, match='HASH_MISMATCH|REPLAY_RUNTIME_'):
        replay.replay_recording(*args, rate=1)
    assert json.loads((output/'report.json').read_text())['status'] == 'FAIL'


def test_capture_drains_log_after_storage_cap_and_keeps_exit_code(tmp_path, monkeypatch):
    monkeypatch.setattr(replay, 'LOG_LIMIT', 1024)
    log = tmp_path/'mock-native.log'
    result = replay._capture([sys.executable, '-c',
        'import os; os.write(1,b"x"*500000); raise SystemExit(9)'], log, timeout=5)
    assert result['exit_code'] == 9 and not result['timed_out']
    assert result['log_bytes_seen'] == 500000
    assert result['log_bytes_saved'] == log.stat().st_size == 1024
    assert result['log_truncated']


def _live(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().split()[2] not in ('Z', 'X')
    except FileNotFoundError:
        return False


def _wait_dead(pid):
    deadline = time.monotonic() + .5
    while _live(pid) and time.monotonic() < deadline:
        time.sleep(.01)
    return not _live(pid)


@pytest.mark.parametrize('close_output', [False, True])
def test_capture_deadline_kills_owned_process_group_even_after_stdout_closes(tmp_path, close_output):
    pidfile = tmp_path/'mock-group.pid'
    childfile = tmp_path/'mock-child.pid'
    code = ('import os, pathlib, subprocess, sys, time\n'
        f'pathlib.Path({str(pidfile)!r}).write_text(str(os.getpid()))\n'
        'child=subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"],'
        'stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)\n'
        f'pathlib.Path({str(childfile)!r}).write_text(str(child.pid))\n' +
        ('os.close(1); os.close(2)\n' if close_output else '') + 'time.sleep(30)\n')
    started = time.monotonic()
    try:
        result = replay._capture([sys.executable, '-c', code], tmp_path/'mock-timeout.log', timeout=.2)
        assert result['timed_out'] and result['exit_code'] != 0
        assert time.monotonic()-started < 3
        assert _wait_dead(int(pidfile.read_text()))
        assert _wait_dead(int(childfile.read_text()))
    finally:
        if pidfile.exists() and _live(int(pidfile.read_text())):
            os.killpg(int(pidfile.read_text()), signal.SIGKILL)
        if childfile.exists() and _live(int(childfile.read_text())):
            os.kill(int(childfile.read_text()), signal.SIGKILL)


def test_capture_cleans_owned_child_when_mock_parent_exits_successfully(tmp_path):
    pidfile = tmp_path/'mock-parent.pid'
    childfile = tmp_path/'mock-orphan.pid'
    code = ('import os, pathlib, subprocess, sys\n'
        f'pathlib.Path({str(pidfile)!r}).write_text(str(os.getpid()))\n'
        'child=subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"],'
        'stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)\n'
        f'pathlib.Path({str(childfile)!r}).write_text(str(child.pid))\n')
    try:
        result = replay._capture([sys.executable, '-c', code], tmp_path/'mock-parent.log', timeout=2)
        assert result['exit_code'] == 0 and not result['timed_out']
        assert _wait_dead(int(childfile.read_text()))
    finally:
        if pidfile.exists() and _live(int(pidfile.read_text())):
            os.killpg(int(pidfile.read_text()), signal.SIGKILL)
        if childfile.exists() and _live(int(childfile.read_text())):
            os.kill(int(childfile.read_text()), signal.SIGKILL)
