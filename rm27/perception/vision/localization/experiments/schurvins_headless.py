"""Recorded ROS-free poses, retaining unknown operational state and mono scale."""
import argparse
import csv
from decimal import Decimal
from pathlib import Path

from ..schema import TimestampEvidence
from .common import canonical, file_hash, number, read_json, require, write_json
from .online import OnlinePoseRecord
from .schurvins import PIN
from .trajectory import rotation_wxyz

POSE_COLUMNS = ['timestamp_ns', 'tx', 'ty', 'tz', 'qw', 'qx', 'qy', 'qz']
FRAME_COLUMNS = ['timestamp_ns', 'scheduled_ns', 'start_ns', 'end_ns', 'stage',
                 'quality', 'pose_emitted', 'imu_added', 'queue_pending_frames']


def _integer(value):
    require(isinstance(value, str) and value.isascii() and value.isdecimal(),
            'HEADLESS_INTEGER_INVALID', str(value))
    return int(value)


def _rows(path, columns):
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames == columns, 'HEADLESS_COLUMNS_INVALID', str(path))
        rows = list(reader)
    require(all(set(row) == set(columns) and None not in row.values() for row in rows),
            'HEADLESS_ROW_INVALID', str(path))
    return rows


def load_run(run, camera_csv=None):
    """Validate summary, frame/pose parity, clocks and optional full camera prefix."""
    run = Path(run)
    summary = read_json(run/'summary.json')
    require(type(summary) is dict and type(summary.get('schema_version')) is int
            and summary['schema_version'] == 1 and summary.get('runtime') == 'ROS_FREE_CPP'
            and summary.get('status') == 'EXECUTION_COMPLETED'
            and summary.get('mode') in ('mono', 'stereo'),
            'HEADLESS_SUMMARY_INVALID', 'completed versioned native execution required')
    for name in ('camera_input', 'camera_selected', 'imu_input', 'imu_added', 'pose_count', 'tracking_losses'):
        require(type(summary.get(name)) is int and summary[name] >= 0,
                'HEADLESS_SUMMARY_INVALID', name)
    require(0 < summary['camera_selected'] <= summary['camera_input']
            and 0 < summary['imu_input'] >= summary['imu_added'],
            'HEADLESS_SUMMARY_INVALID', 'input counts')
    frames = [{key: _integer(value) for key, value in row.items()}
              for row in _rows(run/'frames.csv', FRAME_COLUMNS)]
    poses = []
    for row in _rows(run/'poses.csv', POSE_COLUMNS):
        pose = dict(timestamp_ns=_integer(row['timestamp_ns']),
                    position=[number(float(row[key]), key) for key in ('tx', 'ty', 'tz')],
                    orientation_wxyz=[number(float(row[key]), key) for key in ('qw', 'qx', 'qy', 'qz')])
        rotation_wxyz(pose['orientation_wxyz'])
        poses.append(pose)
    require(len(frames) == summary['camera_selected'] and len(poses) == summary['pose_count'],
            'HEADLESS_COUNT_MISMATCH', 'summary/frame/pose counts')
    for index, frame in enumerate(frames):
        require(frame['timestamp_ns'] > 0 and frame['end_ns'] >= frame['start_ns']
                and frame['stage'] in (0, 1, 2, 3) and frame['quality'] in (0, 1, 2)
                and frame['pose_emitted'] in (0, 1) and frame['imu_added'] <= summary['imu_added']
                and (not frame['pose_emitted'] or frame['stage'] == 2),
                'HEADLESS_FRAME_INVALID', str(index))
        if index:
            previous = frames[index-1]
            require(frame['timestamp_ns'] > previous['timestamp_ns']
                    and frame['start_ns'] >= previous['end_ns']
                    and frame['scheduled_ns'] >= previous['scheduled_ns']
                    and frame['imu_added'] >= previous['imu_added'],
                    'HEADLESS_ORDER_INVALID', str(index))
    require([pose['timestamp_ns'] for pose in poses] ==
            [frame['timestamp_ns'] for frame in frames if frame['pose_emitted']],
            'HEADLESS_POSE_ASSOCIATION_INVALID', 'exact emitted-frame association; no retiming')
    losses = sum(a['stage'] == 2 and b['stage'] != 2 for a, b in zip(frames, frames[1:]))
    require(losses == summary['tracking_losses'], 'HEADLESS_COUNT_MISMATCH', 'tracking losses')
    if camera_csv is not None:
        with Path(camera_csv).open(newline='') as stream:
            source = list(csv.reader(line for line in stream if line.strip() and not line.startswith('#')))
        require(all(len(row) == 2 for row in source), 'HEADLESS_CAMERA_INVALID', 'camera rows')
        stamps = [_integer(row[0]) for row in source]
        require(all(a < b for a, b in zip(stamps, stamps[1:])),
                'HEADLESS_CAMERA_INVALID', 'strictly increasing camera stamps')
        require(len(stamps) == summary['camera_input'] and
                stamps[:len(frames)] == [frame['timestamp_ns'] for frame in frames],
                'HEADLESS_CAMERA_MISMATCH', 'complete original camera prefix')
    return summary, frames, poses


def project(run, camera_csv, *, binary_sha256, run_id, sequence):
    require(isinstance(binary_sha256, str) and len(binary_sha256) == 64
            and all(c in '0123456789abcdef' for c in binary_sha256),
            'BINARY_HASH_INVALID', 'SHA256')
    require(all(isinstance(value, str) and value for value in (run_id, sequence)),
            'RUN_ID_INVALID', 'run and EuRoC sequence identity required')
    summary, frames, poses = load_run(run, camera_csv)
    by_stamp = {pose['timestamp_ns']: pose for pose in poses}
    records = []
    for index, frame in enumerate(frames):
        pose = by_stamp.get(frame['timestamp_ns'])
        if pose is None:
            continue
        stamp = TimestampEvidence(frame['timestamp_ns'], 'ns', 'EuRoC:'+sequence+':sensor_time',
                                  'DATASET_CAMERA_TIMESTAMP', 'VERIFIED').to_dict()
        completion = TimestampEvidence(frame['end_ns'], 'ns', run_id+':native_steady',
                                       'NATIVE_CALLBACK_COMPLETION', 'OBSERVED').to_dict()
        stereo = summary['mode'] == 'stereo'
        fields = dict(parent_frame='backend_world', child_frame='imu', translation=pose['position'],
                      orientation_wxyz=pose['orientation_wxyz'], valid=False,
                      scale_state='METRIC' if stereo else 'UNKNOWN',
                      translation_unit='m' if stereo else 'UNKNOWN',
                      localization_epoch=0, source_time=stamp)
        raw = dict(stream='headless/poses.csv', pose_csv=str(Path(run)/'poses.csv'),
                   frame_csv=str(Path(run)/'frames.csv'), frame_row=index+2,
                   stage=frame['stage'], quality=frame['quality'],
                   imu_added=frame['imu_added'], epoch_owner='adapter_run',
                   metric_initialization_evidence=None, accepted_visual_correction=None,
                   scale_evidence=('CALIBRATED_STEREO_BASELINE_CONVENTION' if stereo else 'UNKNOWN'),
                   upstream_reset_event=None, covariance_semantics='UNKNOWN_NOT_EXPORTED')
        records.append(OnlinePoseRecord(source_frame_id=index, source_timestamp=stamp,
            backend='SchurVINS', backend_version=dict(commit=PIN, binary_sha256=binary_sha256),
            raw_backend_state=canonical(dict(stage=frame['stage'], quality=frame['quality'])),
            tracking_state='UNKNOWN', initialization_state='UNKNOWN', validity=False,
            reason='NUMERICAL_POSE_ONLY_OPERATIONAL_STATE_UNQUALIFIED',
            encoded_pts=str(Decimal(frame['timestamp_ns'])/Decimal(10**9)),
            localization_fields=fields, backend_timestamp=stamp,
            processing_completed_timestamp=completion,
            processing_time_s=(frame['end_ns']-frame['start_ns'])/1e9,
            localization_epoch=0, raw_output=raw,
            backend_pose_convention='Hamilton_T_world_imu_wxyz').to_dict())
    return records


def export(run, camera_csv, *, binary, run_id, sequence, output):
    output = Path(output)
    require(not output.exists() and not output.is_symlink(), 'OUTPUT_EXISTS', str(output))
    paths = [Path(run)/name for name in ('summary.json', 'frames.csv', 'poses.csv')]
    paths += [Path(camera_csv), Path(binary)]
    before = {str(path.resolve()): file_hash(path) for path in paths}
    records = project(run, camera_csv, binary_sha256=file_hash(binary), run_id=run_id, sequence=sequence)
    require(all(file_hash(path) == digest for path, digest in before.items()),
            'INPUT_MUTATED', 'recorded inputs changed during projection')
    output.mkdir(parents=True, exist_ok=False)
    with (output/'observations.jsonl').open('x') as stream:
        for record in records:
            stream.write(canonical(record)+'\n')
    result = dict(status='PASS', scope='OFFLINE_RECORDED_CONTRACT_ONLY', count=len(records),
                  operational_validity='NOT_RUN', mavlink_eligibility='NOT_RUN',
                  inputs_sha256=before, adapter_sha256=file_hash(__file__),
                  output_sha256=file_hash(output/'observations.jsonl'))
    write_json(output/'projection.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('run', 'camera-csv', 'binary', 'run-id', 'sequence', 'output'):
        parser.add_argument('--'+name, required=True)
    result = export(**vars(parser.parse_args()))
    print(canonical(result))


if __name__ == '__main__':
    main()
