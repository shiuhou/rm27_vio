"""Numerical evaluation of SchurVINS/OpenVINS online rows using existing SE3 math.

Input is JSONL raw visual observations or OnlinePoseRecord projections. Epochs
are scored separately; their ATEs are never combined into a continuous result.
Common-interval results supplement, and never replace, each run's full coverage.
This module has no ROS dependency and never provides truth to an estimator.
"""
import argparse
import csv
from decimal import Decimal, ROUND_HALF_EVEN
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np

from tools.openvins.native_accuracy import (
    associate, digest, interpolate, metrics, ordered,
    prepare_reference_quaternions, rotations,
)


def load_rows(path):
    """Read raw rows or the existing online/trajectory JSON envelope."""
    text = Path(path).read_text()
    if text.lstrip().startswith('['):
        return json.loads(text)
    # Normalized trajectories may be a JSON object with a records array.
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        value = None
    if isinstance(value, dict) and isinstance(value.get('records'), list):
        return value['records']
    return [json.loads(line) for line in text.splitlines() if line.strip()]


def canonical_row(row):
    """Preserve state time and camera association, without using validity gates."""
    if row.get('contract') == 'LocalizationEstimate':
        fields = row['localization_fields']
        if not fields:
            raise ValueError('visual pose required')
        if fields['parent_frame'] != 'backend_world' or fields['child_frame'] != 'imu':
            raise ValueError('T_backend_world_imu required')
        if fields['scale_state'] != 'METRIC' or fields['translation_unit'] != 'm':
            raise ValueError('metric pose required; no fitted scale')
        state = row['backend_timestamp']
        image = row['source_timestamp']
        if state['unit'] != 'ns' or image['unit'] != 'ns':
            raise ValueError('integer nanosecond evidence required')
        if state['clock_domain'] != image['clock_domain']:
            raise ValueError('image/state clocks differ without mapping')
        q = fields['orientation_wxyz']
        return dict(state_ns=state['raw_value'], image_ns=image['raw_value'],
                    position=fields['translation'], quaternion_xyzw=[*q[1:], q[0]],
                    epoch=row['localization_epoch'], source_frame_id=row['source_frame_id'])
    return dict(row)


def read_reference(archive):
    """Read original EuRoC truth; normalize its rounding only, as OpenVINS does."""
    with zipfile.ZipFile(archive) as z:
        if len(z.namelist()) != len(set(z.namelist())):
            raise ValueError('duplicate ZIP members')
        camera = [int(r[0]) for r in csv.reader(
            line for line in z.read('mav0/cam0/data.csv').decode().splitlines()
            if line and not line.startswith('#'))]
        gt_bytes = z.read('mav0/state_groundtruth_estimate0/data.csv')
        gt = list(csv.reader(line for line in gt_bytes.decode().splitlines()
                            if line and not line.startswith('#')))
    ordered(camera)
    ns = [int(r[0]) for r in gt]
    ordered(ns)
    q, normalization = prepare_reference_quaternions([[float(v) for v in r[4:8]] for r in gt])
    return dict(camera_ns=camera, ns=ns,
                positions=np.asarray([[float(v) for v in r[1:4]] for r in gt]),
                quaternions=q, normalization=normalization,
                gt_csv_sha256=hashlib.sha256(gt_bytes).hexdigest(),
                archive_sha256=digest(archive))


def read_uzh_reference(archive, *, camera_id='cam0'):
    """Read published v3 UZH T_world_IMU truth with its original header times.

    The official render_gt_projection.py at f01b23d confirms the body/IMU
    frame. Camera time precompensation is not established by that code. Thus
    these scores are explicitly diagnostic, never a qualified time alignment.
    No offset or scale is fitted and the unscored source prefix is retained.
    """
    from tools.schurvins.ros_transport import uzh_rows
    if camera_id not in ('cam0', 'cam1'):
        raise ValueError('explicit source camera required')
    with zipfile.ZipFile(archive) as z:
        if len(z.namelist()) != len(set(z.namelist())):
            raise ValueError('duplicate ZIP members')
        camera = [row[0] for row in uzh_rows(z.read('left_images.txt' if camera_id == 'cam0'
                                                  else 'right_images.txt').decode(), 'cam0')]
        gt_bytes = z.read('groundtruth.txt')
    rows = [line.split() for line in gt_bytes.decode().splitlines()
            if line.strip() and not line.lstrip().startswith('#')]
    if not rows or any(len(row) != 8 for row in rows):
        raise ValueError('UZH groundtruth requires timestamp tx ty tz qx qy qz qw')
    ns = [int((Decimal(row[0])*10**9).to_integral_value(rounding=ROUND_HALF_EVEN)) for row in rows]
    ordered(ns)
    q, normalization = prepare_reference_quaternions([[float(row[7]), *map(float, row[4:7])] for row in rows])
    positions = np.asarray([[float(v) for v in row[1:4]] for row in rows])
    if not np.isfinite(positions).all():
        raise ValueError('nonfinite UZH reference position')
    return dict(camera_ns=camera, ns=ns, positions=positions, quaternions=q, normalization=normalization,
        gt_csv_sha256=hashlib.sha256(gt_bytes).hexdigest(), archive_sha256=digest(archive),
        reference_evidence=dict(dataset='UZH_FPV', version='v3', parent_frame='source_world', child_frame='imu',
            frame_source='uzh-rpg/uzh_fpv_open:f01b23d06724e5b8050b5820fae413614a486176:render_gt_projection.py',
            timestamp_policy='published GT and image headers; nearest-ns ROUND_HALF_EVEN; no fitted offset',
            camera_imu_precompensation='NOT_ESTABLISHED', scoring_qualification='DIAGNOSTIC_ONLY'))


def validate_rows(rows, reference, tolerance_ns=6000):
    """Validate all rows before scoring; never drop malformed/failing poses."""
    if not rows:
        raise ValueError('no visual poses')
    result = [canonical_row(r) for r in rows]
    previous_epoch = -1
    for r in result:
        epoch = r.get('epoch', 0)
        if type(epoch) is not int or epoch < 0 or epoch < previous_epoch:
            raise ValueError('epochs must be nonnegative and ordered')
        for key in ['state_ns', 'image_ns']:
            if type(r[key]) is not int or r[key] < 0:
                raise ValueError('integer nonnegative ' + key + ' required')
        previous_epoch = epoch
        for key, length in [('position', 3), ('quaternion_xyzw', 4)]:
            value = r.get(key)
            if (not isinstance(value, (list, tuple)) or len(value) != length
                    or any(type(x) not in (float, int) for x in value)):
                raise ValueError('invalid real-valued ' + key)
    # Global source ordering is required even across reset epochs. A dataset
    # rewind is a separate run, not a silently accepted localization reset.
    ids = associate(reference['camera_ns'], [r['image_ns'] for r in result], tolerance_ns)
    for r, index in zip(result, ids):
        if 'source_frame_id' in r and r['source_frame_id'] != index:
            raise ValueError('source_frame_id does not match image timestamp')
        r['source_frame_id'] = index
        r['epoch'] = r.get('epoch', 0)
    for epoch in dict.fromkeys(r['epoch'] for r in result):
        group = [r for r in result if r['epoch'] == epoch]
        ordered([r['state_ns'] for r in group])
        p = np.asarray([r['position'] for r in group], float)
        if p.shape != (len(group), 3) or not np.isfinite(p).all():
            raise ValueError('nonfinite or invalid position')
        rotations([[r['quaternion_xyzw'][3], *r['quaternion_xyzw'][:3]] for r in group])
    return result


def evaluate_rows(rows, reference, *, tolerance_ns=6000):
    """Return per-epoch score and complete coverage, retaining unevaluable epochs."""
    rows = validate_rows(rows, reference, tolerance_ns)
    all_camera = reference['camera_ns']
    epochs = []
    for epoch in dict.fromkeys(r['epoch'] for r in rows):
        group = [r for r in rows if r['epoch'] == epoch]
        ns = [r['state_ns'] for r in group]
        p = np.asarray([r['position'] for r in group], float)
        q = np.asarray([[r['quaternion_xyzw'][3], *r['quaternion_xyzw'][:3]] for r in group])
        report = dict(epoch=epoch, raw_pose_count=len(group),
                      first_camera_index=group[0]['source_frame_id'],
                      last_camera_index=group[-1]['source_frame_id'],
                      first_image_ns=group[0]['image_ns'], last_image_ns=group[-1]['image_ns'],
                      first_state_ns=ns[0], last_state_ns=ns[-1],
                      evaluated_pose_count=0, status='NOT_EVALUABLE')
        try:
            ref = interpolate(reference['ns'], reference['positions'], reference['quaternions'], ns, 10000000)
            ids = ref['indices']
            selected_ns = [ns[i] for i in ids]
            report.update(evaluated_pose_count=len(ids), excluded_outside_gt=len(group)-len(ids),
                          evaluated_duration_s=(selected_ns[-1]-selected_ns[0])/1e9,
                          associations=[dict(raw_pose_index=i, camera_index=group[i]['source_frame_id'],
                                             image_ns=group[i]['image_ns'], state_ns=ns[i], gt_bracket=b)
                                        for i, b in zip(ids, ref['brackets'])])
            score = metrics(selected_ns, p[ids], q[ids], ref['positions'], ref['quaternions'])
            aligned_rot = np.asarray(score['alignment']['rotation']) @ rotations(q[ids])
            err_rot = np.transpose(rotations(ref['quaternions']), (0, 2, 1)) @ aligned_rot
            angles = np.degrees(np.arccos(np.clip((np.trace(err_rot, axis1=1, axis2=2)-1)/2, -1, 1)))
            score['orientation_absolute'] = dict(rmse_deg=float(np.sqrt(np.mean(angles**2))),
                                                 p95_deg=float(np.percentile(angles, 95)))
            report.update(status='SCORED', metrics=score)
        except ValueError as exc:
            # An epoch with no truth overlap or degenerate motion is visible,
            # rather than disappearing from the reliability denominator.
            report['reason'] = str(exc)
        epochs.append(report)
    ids = [r['source_frame_id'] for r in rows]
    span = ids[-1]-ids[0]+1
    result = dict(schema_version=1, trajectory_variant='online',
                alignment_policy='SE3 scale=1; no scale/time-shift fitting; epochs separate',
                time_policy='state header ns; original GT interpolation <=10ms; no extrapolation',
                gt_csv_sha256=reference['gt_csv_sha256'], archive_sha256=reference['archive_sha256'],
                reference_quaternion_normalization=reference['normalization'],
                reference_orientation_caveat=('original V1_01_easy orientation truth is known inaccurate'
                    if reference['archive_sha256'] == 'a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa'
                    else 'original archive truth; no external accuracy qualification asserted'),
                raw_pose_count=len(rows), epoch_count=len(epochs),
                scored_epoch_count=sum(e['status'] == 'SCORED' for e in epochs),
                camera_input_count=len(all_camera), output_camera_fraction=len(ids)/len(all_camera),
                missing_before_first=ids[0], missing_after_last=len(all_camera)-ids[-1]-1,
                missing_within_output_span=span-len(ids),
                epochs=epochs, continuous_trajectory_qualified=False,
                operational_validity='UNKNOWN; numerical scoring does not establish availability')
    if 'reference_evidence' in reference:
        result['reference_evidence'] = reference['reference_evidence']
    return result


def compare_rows(left, right, reference, *, tolerance_ns=6000):
    """Compare overlapping epoch pairs, while retaining both full-run reports."""
    a, b = validate_rows(left, reference, tolerance_ns), validate_rows(right, reference, tolerance_ns)
    result = dict(left_full=evaluate_rows(a, reference, tolerance_ns=tolerance_ns),
                  right_full=evaluate_rows(b, reference, tolerance_ns=tolerance_ns),
                  common_intervals=[], interpretation='common intervals supplement full coverage; no winner gate')
    for ea in dict.fromkeys(r['epoch'] for r in a):
        ga = [r for r in a if r['epoch'] == ea]
        for eb in dict.fromkeys(r['epoch'] for r in b):
            gb = [r for r in b if r['epoch'] == eb]
            low_id = max(ga[0]['source_frame_id'], gb[0]['source_frame_id'])
            high_id = min(ga[-1]['source_frame_id'], gb[-1]['source_frame_id'])
            if low_id > high_id:
                continue
            sa = [r for r in ga if low_id <= r['source_frame_id'] <= high_id]
            sb = [r for r in gb if low_id <= r['source_frame_id'] <= high_id]
            if not sa or not sb:
                continue
            result['common_intervals'].append(dict(
                left_epoch=ea, right_epoch=eb,
                first_camera_index=low_id, last_camera_index=high_id,
                first_image_ns=reference['camera_ns'][low_id],
                last_image_ns=reference['camera_ns'][high_id],
                left=evaluate_rows(sa, reference, tolerance_ns=tolerance_ns),
                right=evaluate_rows(sb, reference, tolerance_ns=tolerance_ns)))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--compare-poses', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    reference = read_reference(args.archive)
    rows = load_rows(args.poses)
    if args.compare_poses:
        result = compare_rows(rows, load_rows(args.compare_poses), reference)
    else:
        result = evaluate_rows(rows, reference)
    math_source = Path(__file__).resolve().parents[1]/'openvins/native_accuracy.py'
    result['inputs_sha256'] = {str(p): digest(p)
                              for p in [args.archive, args.poses, Path(__file__), math_source]}
    if args.compare_poses:
        result['inputs_sha256'][str(args.compare_poses)] = digest(args.compare_poses)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    return result


if __name__ == '__main__':
    main()
