"""Score recorded headless states after execution; never feed truth to VIO.

Numerical scores retain unknown monocular metric scale and operational state.
Execution, full-input reliability and numeric scoring are separate judgments.
"""
import argparse
import csv
import json
from pathlib import Path

from rm27.perception.vision.localization.experiments.common import file_hash, write_json
from rm27.perception.vision.localization.experiments.schurvins_headless import load_run
from tools.schurvins.evaluate import evaluate_rows, read_reference


def load_poses(path):
    rows = []
    with Path(path).open() as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ['timestamp_ns', 'tx', 'ty', 'tz', 'qw', 'qx', 'qy', 'qz']:
            raise ValueError('unexpected headless pose columns')
        for row in reader:
            ns = int(row['timestamp_ns'])
            rows.append(dict(state_ns=ns, image_ns=ns, epoch=0,
                position=[float(row[k]) for k in ('tx', 'ty', 'tz')],
                quaternion_xyzw=[float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')]))
    return rows


def assess(summary, numerical):
    """Do not promote a process exit or unqualified mono scale to VIO PASS."""
    failures = []
    counts = ('camera_input', 'camera_selected', 'imu_input', 'imu_added', 'tracking_losses')
    if (type(summary.get('schema_version')) is not int or summary['schema_version'] != 1
            or summary.get('runtime') != 'ROS_FREE_CPP' or summary.get('mode') not in ('mono', 'stereo')
            or any(type(summary.get(key)) is not int or summary[key] < 0 for key in counts)
            or not summary.get('camera_input') or not summary.get('imu_input')):
        failures.append('EXECUTION_SCHEMA_INVALID')
    if summary.get('status') != 'EXECUTION_COMPLETED':
        failures.append('EXECUTION_INCOMPLETE')
    if summary.get('camera_selected') != summary.get('camera_input'):
        failures.append('PARTIAL_SEQUENCE')
    if summary.get('imu_added') != summary.get('imu_input'):
        failures.append('IMU_NOT_FULLY_DELIVERED')
    if summary.get('tracking_losses') != 0:
        failures.append('TRACKING_LOSS')
    if not numerical or not numerical.get('scored_epoch_count'):
        failures.append('TRAJECTORY_NOT_EVALUABLE')
    elif any(type(numerical.get(key)) is not int or numerical[key] != 0
             for key in ('missing_after_last', 'missing_within_output_span')):
        failures.append('OUTPUT_TAIL_OR_GAPS')
    if summary.get('mode') == 'mono' and summary.get('metric_initialization_verified') is not True:
        failures.append('MONO_METRIC_INITIALIZATION_UNVERIFIED')
    return dict(status='FAIL' if failures else 'PASS', failure_codes=failures,
        scope='FULL_SEQUENCE_NUMERICAL_REPLAY_ONLY', operational_validity='NOT_RUN',
        input_ingestion='wrapper delivery counts; internal IMU use requires native event evidence',
        accuracy_gate='existing fixed-scale scoring; no new absolute accuracy threshold introduced')


def evaluate(run, archive, output):
    run, archive, output = Path(run).resolve(), Path(archive).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    files = [run/'summary.json', run/'poses.csv', run/'frames.csv', run/'effective-params.yaml']
    before = {str(p): file_hash(p) for p in files}
    summary = json.loads((run/'summary.json').read_text())
    reference = read_reference(archive)
    numerical, error = None, None
    integrity_error = None
    try:
        _, frames, _ = load_run(run)
        if (summary['camera_input'] != len(reference['camera_ns']) or
                [frame['timestamp_ns'] for frame in frames] != reference['camera_ns'][:len(frames)]):
            raise ValueError('frame timeline differs from original complete camera prefix')
    except ValueError as exc:
        integrity_error = str(exc)
    try:
        numerical = evaluate_rows(load_poses(run/'poses.csv'), reference, tolerance_ns=0)
    except ValueError as exc:
        error = str(exc)
    acceptance = assess(summary, numerical)
    if integrity_error is not None:
        acceptance['status'] = 'FAIL'
        acceptance['failure_codes'].append('RAW_RUN_INCONSISTENT')
    result = dict(schema_version=1, acceptance=acceptance, execution=summary,
        numerical=numerical, scoring_error=error, integrity_error=integrity_error, raw_artifacts_sha256=before,
        archive_sha256=reference['archive_sha256'], gt_csv_sha256=reference['gt_csv_sha256'],
        source_sha256={str(p):file_hash(p) for p in [Path(__file__),
            Path(__file__).with_name('evaluate.py'), Path(__file__).parents[1]/'openvins/native_accuracy.py',
            Path(__file__).parents[2]/'rm27/perception/vision/localization/experiments/schurvins_headless.py']},
        scale_qualification=('UNVERIFIED_MONO_SCALE; numeric score is diagnostic only'
            if summary.get('mode') == 'mono' and not summary.get('metric_initialization_verified')
            else 'stereo baseline from explicit calibration; operational state remains unqualified'),
        board_qualification='NOT_RUN; execution-machine evidence is separate')
    if any(file_hash(path) != digest for path, digest in before.items()):
        raise ValueError('run artifacts changed during evaluation')
    output.parent.mkdir(parents=True, exist_ok=True)
    write_json(output, result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--archive', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    result = evaluate(args.run, args.archive, args.output)
    print(json.dumps(result['acceptance']))
    return int(result['acceptance']['status'] != 'PASS')


if __name__ == '__main__':
    raise SystemExit(main())
