"""Replay sealed Camera+IMU recordings with the existing ROS-free OpenVINS.

This entry opens no sensors and sends no flight-controller messages. Public,
mock and hardware provenance remain distinct. A successful replay is a software
transport result; physical synchronization, accuracy and flight validity require
their own evidence.
"""
from __future__ import annotations

import argparse
from bisect import bisect_right
import csv
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import time
import uuid

import numpy as np

from rm27.perception.vision.localization.experiments.common import (
    checked_file, cleanup_container, file_hash, read_json, require, write_json,
)

PIN = '69488123ed9362dd44b6f28e7f4680abbff1442b'
LOG_LIMIT = 16 * 1024 * 1024


def verify_prepared(directory):
    from tools.openvins.recording import check_recording_input

    root = Path(directory).resolve()
    check_recording_input(root)
    return read_json(root / 'input.json')


def runtime_descriptor(path):
    """Bind the executable and optional immutable container to a build receipt."""
    path = Path(path).resolve()
    runtime = read_json(path)
    required = {'schema_version', 'mode', 'binary', 'build_manifest', 'build_manifest_sha256'}
    require(isinstance(runtime, dict) and required <= set(runtime) <= required | {'image'},
            'REPLAY_RUNTIME_INVALID', 'runtime fields')
    require(type(runtime.get('schema_version')) is int and runtime['schema_version'] == 1
            and runtime.get('mode') in ('local', 'docker')
            and all(isinstance(runtime[k], str) and runtime[k] for k in
                    ('binary', 'build_manifest', 'build_manifest_sha256')),
            'REPLAY_RUNTIME_INVALID', 'schema/mode')
    binary, binary_hash = checked_file(path.parent / runtime['binary'])
    build_path, build_hash = checked_file(path.parent / runtime['build_manifest'],
                                         runtime['build_manifest_sha256'])
    build = read_json(build_path)
    require(build.get('upstream_commit') == PIN and build.get('ros_free') is True,
            'REPLAY_RUNTIME_INVALID', 'pinned ROS-free build required')
    require(build.get('binary', {}).get('sha256') == binary_hash,
            'HASH_MISMATCH', 'runner differs from build receipt')
    require(os.access(binary, os.X_OK), 'REPLAY_RUNTIME_INVALID', 'runner must be executable')
    if runtime['mode'] == 'docker':
        image = runtime.get('image')
        require(isinstance(image, str) and image.startswith('sha256:') and len(image) == 71
                and all(c in '0123456789abcdef' for c in image[7:])
                and image == build.get('build_image_id'),
                'REPLAY_RUNTIME_INVALID', 'immutable build image required')
    else:
        require('image' not in runtime, 'REPLAY_RUNTIME_INVALID', 'image is docker-only')
    return dict(runtime, binary=str(binary), build_manifest=str(build_path),
                binary_sha256=binary_hash, build_manifest_sha256=build_hash)


def _integer(text, name):
    require(isinstance(text, str) and text.isdecimal(), 'REPLAY_OUTPUT_INVALID', name)
    return int(text)


def _finite(text, name):
    try:
        value = float(text)
    except (ValueError, TypeError) as exc:
        raise ValueError('REPLAY_OUTPUT_INVALID: ' + name) from exc
    require(math.isfinite(value), 'REPLAY_OUTPUT_INVALID', name)
    return value


def _table(path):
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames is not None and len(set(reader.fieldnames)) == len(reader.fieldnames),
                'REPLAY_OUTPUT_INVALID', 'CSV header')
        rows = list(reader)
    require(all(None not in row and None not in row.values() for row in rows),
            'REPLAY_OUTPUT_INVALID', 'CSV width')
    return rows


def _statistics(values):
    if not values:
        return None
    return dict(count=len(values), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), maximum=float(max(values)))


def diagnose(native, prepared, *, rate):
    """Reconcile actual native outputs; no reference trajectory is opened."""
    native = Path(native)
    summary = read_json(native / 'summary.json')
    frames, poses = _table(native / 'frames.csv'), _table(native / 'poses.csv')
    camera_ns = prepared['camera_timestamps_ns']
    counts = prepared['counts']
    require(summary.get('upstream_commit') == PIN and summary.get('backend') == 'OpenVINS'
            and summary.get('status') == 'EXECUTION_COMPLETED' and summary.get('mode') == 'mono_imu'
            and summary.get('rate') == rate,
            'REPLAY_OUTPUT_INVALID', 'native identity/status/rate')
    require(summary.get('quaternion_convention') == 'native_OpenVINS_JPL_q_GtoI_reordered_wxyz'
            and summary.get('position_convention') == 'p_IinG_m',
            'REPLAY_OUTPUT_INVALID', 'native pose convention')
    require([_integer(r['timestamp_ns'], 'frame time') for r in frames] == camera_ns,
            'REPLAY_INPUT_MISMATCH', 'native camera sequence differs from sealed source')
    require(summary.get('camera_input') == summary.get('selected_frames') == len(camera_ns) == counts['cam0']
            and summary.get('imu_input') == summary.get('imu_delivered') == counts['imu0'],
            'REPLAY_INPUT_MISMATCH', 'full native input counts')
    emitted, initialized, accepted, service_ms, lateness_ms = [], [], [], [], []
    prior_imu, prior_end = 0, 0
    first_due = _integer(frames[0]['scheduled_ns'], 'first schedule')
    schedule = [first_due + (stamp-camera_ns[0]) * (2 if rate == .5 else 1)
                if rate else first_due for stamp in camera_ns]
    sums = {key: 0 for key in ('msckf_updates', 'slam_updates')}
    for row, stamp in zip(frames, camera_ns):
        flags = {key: _integer(row[key], key) for key in
                 ('initialized', 'state_matches_camera', 'finite_posterior', 'pose_emitted')}
        require(set(flags.values()) <= {0, 1}, 'REPLAY_OUTPUT_INVALID', 'boolean flags')
        require(flags['finite_posterior'] == 1, 'REPLAY_POSTERIOR_INVALID', str(stamp))
        should_emit = bool(flags['initialized'] and flags['state_matches_camera'] and flags['finite_posterior'])
        require(bool(flags['pose_emitted']) == should_emit, 'REPLAY_OUTPUT_INVALID', 'pose emission flags')
        if flags['initialized']:
            initialized.append(stamp)
        if should_emit:
            emitted.append(stamp)
        updates = 0
        for key in sums:
            value = _integer(row[key], key)
            sums[key] += value
            updates += value
        if updates and should_emit:
            accepted.append(stamp)
        imu = _integer(row['imu_delivered'], 'IMU cursor')
        require(prior_imu <= imu <= counts['imu0'], 'REPLAY_OUTPUT_INVALID', 'IMU cursor')
        prior_imu = imu
        start, end, due = (_integer(row[k], k) for k in ('start_ns', 'end_ns', 'scheduled_ns'))
        require(prior_end <= start <= end and due <= start,
                'REPLAY_OUTPUT_INVALID', 'callback monotonic times')
        require(due == schedule[len(service_ms)], 'REPLAY_OUTPUT_INVALID', 'replay schedule differs from source/rate')
        pending = max(0, bisect_right(schedule, end) - len(service_ms) - 1) if rate else 0
        require(_integer(row['queue_pending_frames'], 'pending replay frames') == pending,
                'REPLAY_OUTPUT_INVALID', 'scheduled replay pending frames')
        prior_end = end
        service_ms.append((end - start) / 1e6)
        if rate:
            lateness_ms.append((end - due) / 1e6)
    require([_integer(r['timestamp_ns'], 'pose time') for r in poses] == emitted
            and summary.get('poses') == len(poses), 'REPLAY_OUTPUT_INVALID', 'pose sequence/count')
    accepted_set = set(accepted)
    for row in poses:
        values = {k: _finite(row[k], k) for k in
                  ('tx', 'ty', 'tz', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz',
                   'camera_to_imu_offset_s', 'timestamp_imu_seconds')}
        norm = sum(values[k] ** 2 for k in ('qw', 'qx', 'qy', 'qz'))
        require(abs(norm - 1) < 1e-5, 'REPLAY_OUTPUT_INVALID', 'native quaternion norm')
        stamp = int(row['timestamp_ns'])
        require(_integer(row['visual_update'], 'visual update') == int(stamp in accepted_set),
                'REPLAY_OUTPUT_INVALID', 'pose correction witness')
        require(abs(values['timestamp_imu_seconds'] -
                    (stamp * 1e-9 + values['camera_to_imu_offset_s'])) <= 1e-6,
                'REPLAY_OUTPUT_INVALID', 'native IMU state-time convention')
    require(summary.get('accepted_visual_update_frames') == len(accepted)
            and all(summary.get(k) == v for k, v in sums.items())
            and summary.get('first_initialized_timestamp_ns') == (initialized[0] if initialized else 0)
            and summary.get('last_visual_update_timestamp_ns') == (accepted[-1] if accepted else 0)
            and summary.get('maximum_visual_update_gap_ns') == max(
                (b-a for a, b in zip(accepted, accepted[1:])), default=0)
            and summary.get('nonfinite_posteriors') == 0
            and summary.get('state_timestamp_failures') == sum(
                int(r['initialized']) and not int(r['state_matches_camera']) for r in frames),
            'REPLAY_OUTPUT_INVALID', 'native summary/ledger disagreement')
    first_pose_index = camera_ns.index(emitted[0]) if emitted else None
    missing_after_init = [stamp for stamp, row in zip(camera_ns, frames)
                          if initialized and stamp >= initialized[0] and not int(row['pose_emitted'])]
    # Include the start and tail of the initialized interval, rather than only
    # distances between successful updates, which would hide total update loss.
    support = ([initialized[0]] + accepted + [camera_ns[-1]]) if initialized else []
    deadline_ms = (float(np.median(np.diff(camera_ns))) / rate / 1e6) if rate and len(camera_ns) > 1 else None
    trajectory = ('NOT_INITIALIZED' if not initialized else 'NO_POSES' if not emitted else
                  'GAPS_AFTER_INITIALIZATION' if missing_after_init else 'OBSERVED_POSE_CONTINUITY')
    return dict(schema_version=1, status='EXECUTION_COMPLETED', reconciliation='PASS',
        backend='OpenVINS', replay_scope='PC_RECORDED_INPUT', source_provenance=prepared['source_provenance'],
        source_epoch=prepared['source_epoch'], camera_id=prepared['camera_id'], imu_id=prepared['imu_id'],
        clock_domain=prepared['clock_domain'], rate=rate, input_counts=counts,
        poses=len(poses), initialized=bool(initialized), trajectory_status=trajectory,
        first_initialized_timestamp_ns=initialized[0] if initialized else None,
        initialization_prefix_frames=first_pose_index,
        missing_pose_timestamps_after_initialization=missing_after_init,
        accepted_visual_update_frames=len(accepted),
        maximum_visual_support_gap_ns=max((b-a for a,b in zip(support,support[1:])), default=None),
        callback_service_ms=_statistics(service_ms), scheduled_completion_lateness_ms=_statistics(lateness_ms),
        nominal_replay_deadline_ms=deadline_ms,
        nominal_replay_deadline_misses=sum(v > deadline_ms for v in lateness_ms) if deadline_ms is not None else None,
        timing_scope='native decode/IMU admission/camera callback; paced lateness is not live sensor latency',
        orientation_convention=summary['quaternion_convention'], position_convention=summary['position_convention'],
        accuracy='NOT_EVALUATED_NO_REFERENCE', native_reset_count=None, tracking_health='UNKNOWN',
        imu_ingestion_evidence='native feed_measurement_imu callback count; filter integration samples are not independently observed',
        operational_validity=False, physical_camera_imu_qualification=False,
        limitations=['Observed pose continuity is not a tracking/accuracy/flight qualification.',
                     'Raw and mapped source times are preserved in prepared/source/events.jsonl.',
                     'Physical shutter, clock synchronization and body-frame mapping require separate qualification.'])


def _capture(command, log_path, timeout):
    """Drain a bounded native log and kill this process group on timeout."""
    started = time.monotonic()
    seen = saved = 0
    timed_out = False
    remaining_group_terminated = False
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            start_new_session=True)
    try:
        with selectors.DefaultSelector() as selector, Path(log_path).open('xb') as log:
            selector.register(proc.stdout, selectors.EVENT_READ)
            while selector.get_map() or proc.poll() is None:
                if not timed_out and time.monotonic() - started > timeout:
                    timed_out = True
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                for key, _ in selector.select(timeout=0.1):
                    data = os.read(key.fileobj.fileno(), 65536)
                    if not data:
                        selector.unregister(key.fileobj)
                        continue
                    seen += len(data)
                    chunk = data[:max(0, LOG_LIMIT-saved)]
                    log.write(chunk)
                    saved += len(chunk)
            code = proc.wait(timeout=10)
    finally:
        # A completed parent may have left children which closed their log
        # pipes. They still belong to this session and must not outlive replay.
        try:
            os.killpg(proc.pid, signal.SIGKILL)
            remaining_group_terminated = True
        except ProcessLookupError:
            pass
        if proc.poll() is None:
            proc.wait(timeout=10)
        proc.stdout.close()
    return dict(exit_code=code, timed_out=timed_out, elapsed_s=time.monotonic()-started,
                log_bytes_seen=seen, log_bytes_saved=saved, log_truncated=seen > saved,
                remaining_process_group_terminated=remaining_group_terminated)


def _markdown(report):
    if report['status'] != 'EXECUTION_COMPLETED':
        return '# OpenVINS offline replay\n\nStatus: **FAIL**\n\n' + report.get('reason', '') + '\n'
    return ('# OpenVINS offline replay\n\n'
            f"Source: {report['source_provenance']['kind']}; PC recorded-input replay.\n\n"
            f"Input: {report['input_counts']['cam0']} images, {report['input_counts']['imu0']} raw IMU samples.\n\n"
            f"Native input/output reconciliation: {report['reconciliation']}. "
            f"Poses: {report['poses']}; accepted visual-update frames: {report['accepted_visual_update_frames']}.\n\n"
            f"Initialization: {report['initialized']}; trajectory: {report['trajectory_status']}.\n\n"
            f"Missing poses after initialization: {len(report['missing_pose_timestamps_after_initialization'])}.\n\n"
            'Accuracy: NOT EVALUATED (no reference trajectory). Tracking health: UNKNOWN. '
            'Operational / flight validity: false.\n\n'
            'Timing measures offline callback service and optional scheduling lateness; '
            'it does not measure exposure-to-output latency. See report.json for numerical diagnostics.\n')


def replay_recording(recording, output, parameters, estimator_config, runtime, *,
                     source_epoch=None, rate=0, timeout_s=180, prepare_only=False):
    from tools.openvins.recording import prepare_recording

    require(type(rate) in (int, float) and rate in (0, 0.5, 1), 'REPLAY_RATE_INVALID', str(rate))
    require(type(timeout_s) in (int, float) and math.isfinite(timeout_s) and timeout_s > 0,
            'REPLAY_TIMEOUT_INVALID', 'positive timeout required')
    descriptor = runtime_descriptor(runtime) if not prepare_only else None
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    report, container = {}, None
    try:
        prepare_recording(recording, output / 'prepared', parameters, estimator_config,
                          source_epoch=source_epoch)
        prepared = verify_prepared(output / 'prepared')
        prepared_hash = file_hash(output/'prepared/input.json')
        write_json(output / 'request.json', dict(recording=str(Path(recording).resolve()),
            prepared_manifest_sha256=prepared_hash,
            rate=rate, timeout_s=timeout_s, prepare_only=prepare_only,
            orchestration_sha256=file_hash(__file__), runtime=descriptor))
        if prepare_only:
            report = dict(schema_version=1, status='PREPARED_NOT_RUN',
                          input_counts=prepared['counts'], source_provenance=prepared['source_provenance'],
                          execution_started=False, operational_validity=False)
        else:
            binary = Path(descriptor['binary'])
            dataset, config = prepared['dataset_path'], prepared['config_path']
            require((output/'prepared'/dataset).resolve().is_relative_to(output/'prepared')
                    and (output/'prepared'/config).resolve().is_relative_to(output/'prepared'),
                    'REPLAY_MANIFEST_INVALID', 'native input paths')
            if descriptor['mode'] == 'docker':
                container = 'rm27-openvins-recording-' + uuid.uuid4().hex
                command = ['docker', 'run', '--rm', '--name', container, '--network', 'none',
                    '--cpus', '4', '--memory', '4g', '--memory-swap', '4g', '--pids-limit', '256',
                    '--read-only', '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
                    '--user', f'{os.getuid()}:{os.getgid()}', '--tmpfs', '/tmp:rw,size=64m',
                    '--mount', f'type=bind,source={output / "prepared"},target=/input,readonly',
                    '--mount', f'type=bind,source={binary.parent},target=/runner,readonly',
                    '--mount', f'type=bind,source={output},target=/out', descriptor['image'],
                    '/runner/' + binary.name, '--dataset', '/input/' + dataset,
                    '--config', '/input/' + config, '--output', '/out/native', '--rate', str(rate)]
            else:
                command = [str(binary), '--dataset', str(output/'prepared'/dataset),
                    '--config', str(output/'prepared'/config), '--output', str(output/'native'), '--rate', str(rate)]
            write_json(output/'command.json', dict(argv=command))
            checked_file(binary, descriptor['binary_sha256'])
            checked_file(descriptor['build_manifest'], descriptor['build_manifest_sha256'])
            execution = _capture(command, output/'native.log', timeout_s)
            write_json(output/'execution.json', execution)
            if container:
                with (output/'cleanup.log').open('x') as log:
                    cleanup = cleanup_container(container, log)
                write_json(output/'cleanup.json', cleanup)
                require(cleanup['absence_verified'], 'REPLAY_CLEANUP_FAILED', container)
                container = None
            require(not execution['timed_out'] and execution['exit_code'] == 0,
                    'REPLAY_NATIVE_FAILED', 'see execution.json and native.log')
            # Recheck the source-derived input after the external process exits.
            checked_file(output/'prepared/input.json', prepared_hash)
            verify_prepared(output/'prepared')
            checked_file(binary, descriptor['binary_sha256'])
            checked_file(descriptor['build_manifest'], descriptor['build_manifest_sha256'])
            report = diagnose(output/'native', prepared, rate=rate)
            report['execution'] = execution
        write_json(output/'report.json', report)
        if not prepare_only:
            (output/'report.md').write_text(_markdown(report))
    except Exception as exc:
        report = dict(schema_version=1, status='FAIL', reason=str(exc),
                      operational_validity=False, accuracy='NOT_EVALUATED_NO_REFERENCE')
        write_json(output/'report.json', report)
        (output/'report.md').write_text(_markdown(report))
        raise
    finally:
        if container:
            with (output/'cleanup.log').open('a') as log:
                cleanup = cleanup_container(container, log)
            write_json(output/'cleanup.json', cleanup)
        write_json(output/'artifacts.json', dict(schema_version=1,
            artifacts_sha256={str(p.relative_to(output)): file_hash(p)
                              for p in sorted(output.rglob('*')) if p.is_file() and p != output/'artifacts.json'}))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--parameters', required=True, help='evidenced IMU/gravity parameter JSON')
    parser.add_argument('--estimator-config', required=True, help='explicit OpenVINS tuning YAML')
    parser.add_argument('--runtime', help='pinned binary/build receipt JSON; required for execution')
    parser.add_argument('--source-epoch', type=int)
    parser.add_argument('--rate', type=float, choices=(0, 0.5, 1), default=0)
    parser.add_argument('--timeout-s', type=float, default=180)
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args(argv)
    if not args.prepare_only and not args.runtime:
        parser.error('--runtime is required unless --prepare-only is selected')
    try:
        report = replay_recording(args.recording, args.output, read_json(args.parameters),
            args.estimator_config, args.runtime, source_epoch=args.source_epoch,
            rate=args.rate, timeout_s=args.timeout_s, prepare_only=args.prepare_only)
    except Exception as exc:
        parser.exit(1, str(exc)+'\n')
    print(report['status'])


if __name__ == '__main__':
    main()
