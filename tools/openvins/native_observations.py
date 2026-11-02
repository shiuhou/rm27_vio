"""Audit bounded OpenVINS native evidence without assigning tracking health.

Accepted feature times are associated by their original double bits to earlier
integer ROS callback stamps. A state timestamp, ZUPT or publication never
refreshes the visual measurement time. This is an observation decoder, not an
estimator, clock mapping, or operational validity policy.
"""

from collections import Counter, defaultdict, deque
from bisect import bisect_left
import hashlib
import itertools
import re
import sqlite3
import zipfile
import json
import math
import os
from pathlib import Path
import struct

from rm27.perception.vision.localization.experiments.common import file_hash, require

VISUAL_UPDATES = {
    "msckf_visual_ekf_completed",
    "landmark_visual_ekf_completed",
    "landmark_initialization_visual_ekf_completed",
}
MAX_EVENTS = 200_000
MAX_BYTES = 100 * 1024**2
MAX_LINE_BYTES = 8192
INPUT_MATCH_MAX_BYTES = 4096
INPUT_MATCH_SCOPE = ('required native subscriptions matched publishers at snapshot; '
                     'not ingestion, writer completeness or VIO health')


def inspect_openvins_imu_covariance(values):
    """Inspect the unmodified native 15-state error covariance, not its trust.

    Coordinates are [theta_I, p_G, v_G, bg_I, ba_I]. The native JPL left
    attitude error is a *local IMU* perturbation, not a fixed-world ROS pose
    rotation error. No covariance is sent to a consumer or repaired here.
    The symmetric part's spectrum is diagnostic even when the original
    matrix fails symmetry. Numerical PSD does not establish consistency.
    """
    import numpy as np

    result = dict(numerical_status='FAIL', reasons=[], dimensions=15,
        error_chart='JPL_LEFT_THETA_IMU_POSITION_GLOBAL_VELOCITY_GLOBAL_BIASES_IMU',
        statistical_consistency='UNQUALIFIED', flight_covariance='UNQUALIFIED',
        covariance_modified=False)
    if (not isinstance(values, list) or len(values) != 225
            or any(not _finite(x) for x in values)):
        result['reasons'].append('MARGINAL_REQUIRES_225_FINITE_NUMBERS')
        return result
    matrix = np.asarray(values, dtype=float).reshape(15, 15)
    # Relative floating-point diagnostic, never an eigenvalue/variance floor.
    scale = float(np.max(np.abs(matrix)))
    tolerance = 64 * 15 * np.finfo(float).eps * scale
    with np.errstate(over='ignore', invalid='ignore'):
        asymmetry = float(np.max(np.abs(matrix-matrix.T)))
    if not math.isfinite(asymmetry):
        result['reasons'].append('COVARIANCE_DIAGNOSTIC_NUMERIC_RANGE')
        return result
    diagonal = np.diag(matrix)
    if asymmetry > tolerance:
        result['reasons'].append('NATIVE_COVARIANCE_NOT_SYMMETRIC')
    if float(np.min(diagonal)) < 0:
        result['reasons'].append('NATIVE_COVARIANCE_NEGATIVE_DIAGONAL')
    # Divide before adding, so two finite large elements cannot overflow merely
    # because the diagnostic forms their average.
    symmetric = matrix/2 + matrix.T/2
    try:
        eigenvalues = np.linalg.eigvalsh(symmetric)
        if not np.isfinite(eigenvalues).all():
            raise ValueError('nonfinite eigenvalues')
    except (ValueError, np.linalg.LinAlgError):
        result['reasons'].append('COVARIANCE_SPECTRUM_UNAVAILABLE')
        return result
    if float(eigenvalues[0]) < -tolerance:
        result['reasons'].append('NATIVE_COVARIANCE_NOT_PSD')
    result.update(numerical_status='PASS' if not result['reasons'] else 'FAIL',
        max_abs_antisymmetric=asymmetry, roundoff_tolerance=tolerance,
        roundoff_definition='64 * dimension15 * float64_epsilon * max_abs_original_entry; no floor',
        minimum_diagonal=float(np.min(diagonal)), maximum_diagonal=float(np.max(diagonal)),
        minimum_symmetric_part_eigenvalue=float(eigenvalues[0]),
        maximum_symmetric_part_eigenvalue=float(eigenvalues[-1]),
        symmetric_part_rank=int(np.count_nonzero(eigenvalues > tolerance)),
        rank_scope='mixed-unit original matrix numerical diagnostic, not observability or information rank')
    return result


QUALITY_MAX_BYTES = 100 * 1024**2
QUALITY_MAX_ROWS = 200_000
QUALITY_MAX_LINE_BYTES = 1024**2
QUALITY_GATE_KINDS = {'msckf_gate_batch', 'landmark_gate_batch', 'landmark_initialization_gate'}


def _quality_records(path):
    """Require a complete, bounded, exclusive native quality stream."""
    code = 'OPENVINS_QUALITY_INVALID'
    path = Path(path)
    require(not path.is_symlink() and path.stat().st_size <= QUALITY_MAX_BYTES, code, 'quality byte bound/path')
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, code, 'duplicate quality key')
            result[key] = value
        return result
    def nonfinite(value):
        require(False, code, 'illegal JSON nonfinite constant')
    previous = -1
    total_bytes = count = 0
    closed = False
    with path.open('rb') as stream:
        while True:
            raw = stream.readline(QUALITY_MAX_LINE_BYTES+1)
            if not raw:
                break
            require(not closed and len(raw) <= QUALITY_MAX_LINE_BYTES and raw.endswith(b'\n'), code, 'quality line/footer')
            row = json.loads(raw, object_pairs_hook=unique, parse_constant=nonfinite)
            require(isinstance(row, dict) and type(row.get('schema_version')) is int and row['schema_version'] == 1
                    and type(row.get('monotonic_ns')) is int and row['monotonic_ns'] >= max(previous, 0),
                    code, 'quality schema or serialization clock')
            previous = row['monotonic_ns']
            if row.get('kind') == 'quality_closed':
                require(set(row) == {'schema_version','kind','row_count','dropped_rows','bytes_before_footer','io_failed','monotonic_ns'}
                        and type(row['row_count']) is int and row['row_count'] == count
                        and type(row['dropped_rows']) is int and row['dropped_rows'] == 0
                        and type(row['bytes_before_footer']) is int and row['bytes_before_footer'] == total_bytes
                        and row['io_failed'] is False, code, 'incomplete, truncated or failed quality stream')
                closed = True
                continue
            require(count < QUALITY_MAX_ROWS and type(row.get('sequence')) is int and row['sequence'] == count
                    and _finite(row.get('state_time_seconds')) and row['state_time_seconds'] >= 0
                    and type(row.get('state_time_double_bits')) is int
                    and row['state_time_double_bits'] == _bits(row['state_time_seconds']), code, 'quality sequence/time bits')
            total_bytes += len(raw)
            count += 1
            yield row
    require(closed, code, 'quality footer missing')


def analyze_openvins_tracking_quality(path, native_path, *, session_id):
    """Join actual innovation populations and 15-state covariances to cameras.

    A native gate pass is evidence of the estimator's measurement decision, not
    of an unbiased innovation, calibrated covariance or reliable trajectory.
    Do not use truth, numeric pose counts, nearest timestamps or publish time.
    """
    code = 'OPENVINS_QUALITY_INVALID'
    require(isinstance(session_id, str) and session_id.strip(), code, 'explicit session required')
    source_hashes = {str(Path(p)): file_hash(p) for p in (path, native_path)}
    observed = analyze_openvins_observations(native_path, include_camera_transactions=True)
    transactions = observed['camera_transactions']['records']
    require(observed['camera_transactions']['status'] == 'PASS', code, 'native camera transactions incomplete')
    by_bits = {r['camera_time_double_bits']: r for r in transactions}
    require(len(by_bits) == len(transactions), code, 'ambiguous camera double association')
    next_begin = {t['camera_time_double_bits']: transactions[i+1]['begin_native_monotonic_ns']
                  for i,t in enumerate(transactions[:-1])}
    native_events = {r['sequence']: r for r in _events(native_path)}
    batches = defaultdict(list)
    snapshots = []
    totals = Counter()
    nis_by_kind = defaultdict(list)
    common = {'schema_version','kind','sequence','monotonic_ns','state_time_seconds','state_time_double_bits'}
    for row in _quality_records(path):
        kind = row['kind']
        if kind in QUALITY_GATE_KINDS:
            require(set(row) == common | {'input_feature_count','gate_count','accepted_count','rejected_count',
                    'nonfinite_count','entries_dropped','completed_ekf','gates'}, code, 'quality gate fields')
            counts = ['input_feature_count','gate_count','accepted_count','rejected_count','nonfinite_count','entries_dropped']
            require(all(type(row[k]) is int and row[k] >= 0 for k in counts)
                    and row['entries_dropped'] == 0 and type(row['completed_ekf']) is bool
                    and isinstance(row['gates'], list) and len(row['gates']) == row['gate_count'] <= 2048
                    and row['gate_count'] <= row['input_feature_count']
                    and row['accepted_count']+row['rejected_count'] == row['gate_count'], code, 'quality gate population incomplete')
            accepted = rejected = nonfinite_count = invalid_count = 0
            features = set()
            for entry in row['gates']:
                require(isinstance(entry,list) and len(entry)==9, code, 'quality gate entry')
                feature, nis, dof, threshold_dof, threshold, multiplier, reject, res_finite, s_finite = entry
                require(type(feature) is int and feature >= 0 and feature not in features
                        and type(dof) is int and dof >= 0 and type(threshold_dof) is int and threshold_dof > 0
                        and all(type(x) is bool for x in (reject,res_finite,s_finite))
                        and all(x is None or _finite(x) for x in (nis,threshold,multiplier)), code, 'quality gate scalar types')
                features.add(feature)
                finite = all(_finite(x) for x in (nis,threshold,multiplier)) and res_finite and s_finite
                nonfinite_count += not finite
                effective_threshold = multiplier*threshold if finite else None
                invalid = (not finite or not _finite(effective_threshold)
                           or effective_threshold <= 0 or nis < 0 or threshold <= 0 or multiplier <= 0)
                ratio = nis/effective_threshold if not invalid else None
                invalid = invalid or not _finite(ratio)
                invalid_count += invalid
                if finite:
                    require(reject == (nis > multiplier*threshold), code, 'native chi-square branch differs')
                if kind != 'landmark_initialization_gate':
                    require(dof == threshold_dof and dof > 0, code, 'native updater DOF differs')
                # Init threshold DOF deliberately differs from projected DOF.
                # Record the actual implementation, not a standard chi2 test.
                accepted += not reject
                rejected += reject
                if not invalid:
                    nis_by_kind[kind].append((nis,dof,ratio))
            require((accepted,rejected,nonfinite_count) == (row['accepted_count'],row['rejected_count'],row['nonfinite_count']),
                    code, 'native aggregate differs from complete gates')
            if kind == 'landmark_initialization_gate':
                require(row['completed_ekf'] is False and row['gate_count'] == 1, code, 'init gate is precommit only')
            transaction = by_bits.get(row['state_time_double_bits'])
            require(transaction is not None
                    and transaction['begin_native_monotonic_ns'] <= row['monotonic_ns'] <= transaction['end_native_monotonic_ns'],
                    code, 'gate has no exact processing transaction')
            batches[row['state_time_double_bits']].append(dict(row, invalid_gate_count=invalid_count))
            totals[kind] += 1
            totals['gated_features'] += row['gate_count']
            totals['accepted_gates'] += accepted
            totals['rejected_gates'] += rejected
            totals['nonfinite_gates'] += nonfinite_count
            totals['invalid_gates'] += invalid_count
        elif kind == 'camera_state_snapshot':
            require(set(row) == common | {'camera_time_seconds','camera_time_double_bits','camera_ids',
                    'camera_imu_offset_seconds','mean','covariance_row_major','mean_all_finite','covariance_all_finite'},
                    code, 'quality snapshot fields')
            require(_finite(row['camera_time_seconds']) and type(row['camera_time_double_bits']) is int
                    and _bits(row['camera_time_seconds']) == row['camera_time_double_bits']
                    and _finite(row['camera_imu_offset_seconds'])
                    and isinstance(row['camera_ids'],list) and row['camera_ids']
                    and all(type(camera) is int and camera >= 0 for camera in row['camera_ids'])
                    and isinstance(row['mean'],list) and len(row['mean']) == 16
                    and isinstance(row['covariance_row_major'],list) and len(row['covariance_row_major']) == 225
                    and all(x is None or _finite(x) for x in row['mean']+row['covariance_row_major'])
                    and type(row['mean_all_finite']) is bool and type(row['covariance_all_finite']) is bool
                    and row['mean_all_finite'] == all(_finite(x) for x in row['mean'])
                    and row['covariance_all_finite'] == all(_finite(x) for x in row['covariance_row_major']),
                    code, 'quality snapshot shape/finite witnesses')
            transaction = by_bits.get(row['camera_time_double_bits'])
            require(transaction is not None and row['camera_ids'] == transaction['camera_ids']
                    and transaction['returned_state_event_sequence'] is not None, code, 'snapshot camera binding')
            returned = native_events[transaction['returned_state_event_sequence']]
            state = _state(returned)
            mean = (state['quaternion_jpl_G_to_I_xyzw']+state['position_world_m']+state['velocity_world_mps']
                    +state['gyro_bias_radps']+state['accel_bias_mps2'])
            require(row['state_time_double_bits'] == returned['sensor_time_double_bits']
                    and row['mean'] == mean and row['camera_imu_offset_seconds'] == state['camera_imu_offset_seconds']
                    and row['monotonic_ns'] >= returned['monotonic_ns']
                    and row['monotonic_ns'] <= next_begin.get(row['camera_time_double_bits'],float('inf')),
                    code, 'snapshot differs from actual returned state or camera lifecycle')
            snapshots.append((row,transaction))
            totals[kind] += 1
        else:
            require(False, code, 'unknown quality kind')
    # Completed update flags are checked against the original independent lane.
    records = []
    used_cameras = set()
    updates = {r['sequence']:r for r in observed['visual_updates']}
    for row,transaction in snapshots:
        bits = row['camera_time_double_bits']
        require(bits not in used_cameras, code, 'duplicate quality snapshot')
        used_cameras.add(bits)
        camera_batches = batches.get(bits, [])
        counts = Counter()
        for batch in camera_batches:
            for field in ('input_feature_count','gate_count','accepted_count','rejected_count','nonfinite_count','invalid_gate_count'):
                counts[field] += batch[field]
        native_kinds = Counter(updates[s]['kind'] for s in transaction['accepted_update_sequences'])
        for batch_kind,update_kind in [('msckf_gate_batch','msckf_visual_ekf_completed'),
                                      ('landmark_gate_batch','landmark_visual_ekf_completed')]:
            completed = [b for b in camera_batches if b['kind']==batch_kind and b['completed_ekf']]
            accepted_events = [native_events[s] for s in transaction['accepted_update_sequences']
                               if native_events[s]['kind']==update_kind]
            require(len(completed)==len(accepted_events), code, 'completed gate batch differs from native accepted update')
            for batch,event in zip(completed,accepted_events):
                require(event['monotonic_ns'] <= batch['monotonic_ns']
                        and batch['accepted_count']==event['measurement_support']['feature_count'],
                        code, 'accepted gate population or completion order differs from actual visual update')
        accepted_initializers = [(b,b['gates'][0]) for b in camera_batches
            if b['kind']=='landmark_initialization_gate' and not b['gates'][0][6] and b['gates'][0][2]>0]
        initialization_events = [native_events[s] for s in transaction['accepted_update_sequences']
            if native_events[s]['kind']=='landmark_initialization_visual_ekf_completed']
        require(len(accepted_initializers)==len(initialization_events), code,
                'precommit initialization gates differ from actual visual commits')
        for (batch,gate),event in zip(accepted_initializers,initialization_events):
            require(batch['monotonic_ns'] <= event['monotonic_ns']
                    and event['measurement_support']['feature_count']==1
                    and event['values'][0]==2 and gate[2]==event['values'][1], code,
                    'precommit gate projected DOF or order differs from actual initialization update')
        support = transaction['merged_measurement_support']
        covariance = inspect_openvins_imu_covariance(row['covariance_row_major'])
        records.append(dict(schema_version=1, qualification='VERIFIED_NATIVE_MEASUREMENT_DIAGNOSTIC_ONLY',
            truth_used=False, session_id=session_id, native_observation_ns=row['monotonic_ns'],
            camera_time_double_bits=bits, state_time_double_bits=row['state_time_double_bits'],
            camera_ids=row['camera_ids'], quality_sequence=row['sequence'],
            returned_state_event_sequence=transaction['returned_state_event_sequence'],
            initialization_commit_event_sequence=transaction['initialization_commit_event_sequence'],
            last_visual_sensor_ns=transaction['last_visual_measurement_ns'],
            visual_correction_observed=bool(transaction['accepted_update_count']),
            visual_measurement_time_advanced=transaction['advances_visual_measurement_time'],
            accepted_visual_update_count=transaction['accepted_update_count'],
            accepted_observation_uses=sum(s['observation_use_count'] for s in support),
            native_gate_counts={k:counts[k] for k in ('input_feature_count','gate_count','accepted_count',
                'rejected_count','nonfinite_count','invalid_gate_count')}, covariance=covariance,
            operational_tracking_health='UNKNOWN', output_eligible=False,
            native_epoch=None, native_reset_count=None))
    require({t['camera_time_double_bits'] for t in transactions if t['returned_state_event_sequence'] is not None}
            == used_cameras, code, 'quality snapshot coverage differs from initialized returned states')
    require(set(batches) <= used_cameras, code, 'quality gate batches without returned-state snapshot')
    require(all(file_hash(p)==digest for p,digest in source_hashes.items()), code, 'quality sources changed during audit')
    distributions = {}
    for kind, rows in nis_by_kind.items():
        ratios = sorted(r[2] for r in rows)
        try:
            nis_sum = math.fsum(r[0] for r in rows)
        except OverflowError:
            nis_sum = None
        require(_finite(nis_sum), code, 'innovation aggregate numeric range')
        distributions[kind] = dict(gates=len(rows), nis_sum=nis_sum,
            projected_dof_sum=sum(r[1] for r in rows), native_gate_ratio_p50=ratios[len(ratios)//2],
            native_gate_ratio_max=ratios[-1], statistical_consistency='UNQUALIFIED',
            selection_scope='all gated candidates including rejected; cleaning/triangulation exclusions not innovations')
    return dict(schema_version=1,status='PASS',scope='NATIVE_MEASUREMENT_AND_COVARIANCE_DIAGNOSTICS_ONLY',truth_used=False,
        session_id=session_id,source_sha256=source_hashes,counts=dict(totals),gate_diagnostics=distributions,
        records=records,operational_tracking_health='UNKNOWN',output_eligible=False,
        covariance_consistency='UNQUALIFIED',native_epoch=None,
        limitations=['Native gates are correlated/selected, not an independent chi-square calibration test',
                    'Initialization threshold uses original, not projected residual DOF',
                    'Numerical covariance is not statistical consistency or flight qualification',
                    'No source/publication/FC clock mapping or native reset is inferred'])


def qualify_openvins_tracking_health(path, native_path, health_path, health_summary, output, *, session_id):
    """Evaluate every native freshness row; persist a compact transition ledger.

    Snapshot association uses only prior native serialization events of the same
    process. Sensor times are never compared to receive/host/FC clocks. Neither
    prediction nor this assessment manufactures a new accepted visual event.
    """
    from rm27.perception.vision.localization.state_output import evaluate_openvins_measurement_health

    require(health_summary.get('session_id') == session_id and health_summary.get('status') == 'PASS'
            and file_hash(health_path) == health_summary['timeline']['sha256']
            and file_hash(native_path) == health_summary.get('native_event_source',{}).get('sha256'),
            'OPENVINS_QUALITY_INVALID', 'verified same-session native freshness timeline required')
    health_hash = file_hash(health_path)
    quality = analyze_openvins_tracking_quality(path, native_path, session_id=session_id)
    snapshots = iter(quality['records'])
    pending = next(snapshots, None)
    current = previous = None
    counts = Counter()
    total = transitions = eligible = 0
    covariance_counts = Counter(r['covariance']['numerical_status'] for r in quality['records'])
    with Path(output).open('x') as target, Path(health_path).open() as source:
        for line in source:
            row = json.loads(line)
            while pending is not None and pending['native_observation_ns'] <= row['native_observation_ns']:
                current, pending = pending, next(snapshots, None)
            result = evaluate_openvins_measurement_health(row, health_summary, current)
            total += 1
            eligible += result['output_eligible']
            counts[result['measurement_health']] += 1
            compact = (result['freshness'],result['measurement_health'],tuple(result['reasons']),
                       result['last_visual_sensor_ns'],result['quality_sequence'],result['state_source'])
            if compact != previous:
                target.write(json.dumps(dict(result, native_event_sequence=row['native_event_sequence'],
                    native_observation_ns=row['native_observation_ns'],native_epoch=None,native_reset_count=None),
                    separators=(',',':'),allow_nan=False)+'\n')
                transitions += 1
                previous = compact
    require(file_hash(health_path)==health_hash and not eligible, 'OPENVINS_QUALITY_INVALID',
            'source changed or diagnostic promoted output')
    quality.update(measurement_health=dict(evaluated_native_rows=total, counts=dict(counts),
        output_eligible_rows=eligible, scope='MEASUREMENT_DIAGNOSTIC_NOT_TRAJECTORY_OR_FLIGHT_HEALTH'),
        covariance_numerical_counts=dict(covariance_counts),
        health_timeline_source_sha256=health_hash,
        timeline=dict(path=Path(output).name,sha256=file_hash(output),transition_rows=transitions,
                      persistence='compact changes; every source row evaluated, original complete native health timeline preserved'),
        time_qualification=dict(sensor_clock=health_summary['final']['sensor_clock'],
            native_clock='PROCESS_LOCAL_STEADY_EVENT_SERIALIZATION_NOT_HARDWARE_RECEIPT',
            source_to_native_clock_mapping=None,publication_time=None,source_to_px4_boot_mapping=None,
            px4_timesync='NOT_RUN',native_reset='BLOCKED',ekf_accepted='BLOCKED_NOT_RUN',ekf_fused='BLOCKED_NOT_RUN'))
    return quality


def read_input_publisher_matches(path, *, mode, native_pid):
    """Read one native reader-match snapshot; this never certifies delivery.

    The PID is the actual child PID recorded inside the replay container. A
    sibling pending name means the native publication has not fully completed.
    This same decoder gates the paused player and validates the sealed output.
    """
    code = 'OPENVINS_INPUT_MATCH_INVALID'
    require(mode in ('mono', 'stereo') and type(native_pid) is int and native_pid > 0,
            code, 'explicit mode and actual native child PID required')
    path = Path(path)
    require(not path.is_symlink() and not os.path.lexists(str(path)+'.pending'),
            code, 'native match publication is pending or redirected')
    with path.open('rb') as stream:
        data = stream.read(INPUT_MATCH_MAX_BYTES+1)
    require(len(data) <= INPUT_MATCH_MAX_BYTES, code, 'native match snapshot byte bound')

    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, code, 'duplicate JSON key')
            result[key] = value
        return result

    def nonfinite(value):
        require(False, code, 'nonfinite JSON constant: '+value)

    row = json.loads(data, object_pairs_hook=unique_object, parse_constant=nonfinite)
    fields = {'schema_version', 'kind', 'scope', 'mode', 'native_monotonic_ns',
              'native_process_id', 'camera_ids', 'subscriptions'}
    cameras = [0] if mode == 'mono' else [0, 1]
    require(isinstance(row, dict) and set(row) == fields
            and type(row['schema_version']) is int and row['schema_version'] == 1
            and row['kind'] == 'native_input_publisher_matches' and row['scope'] == INPUT_MATCH_SCOPE
            and row['mode'] == mode and type(row['native_process_id']) is int
            and row['native_process_id'] == native_pid
            and type(row['native_monotonic_ns']) is int and row['native_monotonic_ns'] > 0
            and isinstance(row['camera_ids'], list) and row['camera_ids'] == cameras
            and all(type(camera) is int for camera in row['camera_ids']),
            code, 'native match schema, process or mode differs')
    wanted = {('imu', 0, '/imu0')} | {('camera', camera, f'/cam{camera}/image_raw') for camera in cameras}
    subscriptions = row['subscriptions']
    require(isinstance(subscriptions, list) and len(subscriptions) == len(wanted),
            code, 'all required native subscriptions must be represented once')
    seen = set()
    for item in subscriptions:
        require(isinstance(item, dict) and set(item) ==
                {'sensor_kind', 'sensor_id', 'topic', 'matched_publisher_count'}
                and type(item['sensor_id']) is int and isinstance(item['sensor_kind'], str)
                and isinstance(item['topic'], str) and type(item['matched_publisher_count']) is int
                and item['matched_publisher_count'] > 0,
                code, 'native subscription identity or positive match count required')
        identity = item['sensor_kind'], item['sensor_id'], item['topic']
        require(identity in wanted and identity not in seen, code, 'unexpected or duplicate native subscription')
        seen.add(identity)
    require(not os.path.lexists(str(path)+'.pending'), code, 'native publication remains pending')
    return row


def _integer(value, label, minimum=0):
    require(type(value) is int and value >= minimum,
            "OPENVINS_OBSERVATION_INVALID", label + " must be an integer")
    return value


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _bits(value):
    require(_finite(value), "OPENVINS_OBSERVATION_INVALID", "finite double required")
    return struct.unpack("=Q", struct.pack("=d", value))[0]


def _events(path):
    previous = -1
    closed = False
    count = 0
    total = 0

    def reject_constant(value):
        raise ValueError("nonfinite JSON constant: " + value)

    with Path(path).open("rb") as stream:
        while line := stream.readline(MAX_LINE_BYTES + 1):
            total += len(line)
            require(len(line) <= MAX_LINE_BYTES and total <= MAX_BYTES + MAX_LINE_BYTES,
                    "OPENVINS_OBSERVATION_LIMIT", "native observation byte bound exceeded")
            require(not closed, "OPENVINS_OBSERVATION_INVALID", "event after footer")
            row = json.loads(line, parse_constant=reject_constant)
            require(isinstance(row, dict) and type(row.get("schema_version")) is int
                    and row["schema_version"] == 1 and isinstance(row.get("kind"), str),
                    "OPENVINS_OBSERVATION_INVALID", "native observation schema")
            now = _integer(row.get("monotonic_ns"), "native monotonic time")
            require(now >= previous, "OPENVINS_OBSERVATION_INVALID", "native clock moved backwards")
            previous = now
            if row["kind"] == "observation_closed":
                require(type(row.get("event_count")) is int and row["event_count"] == count
                        and type(row.get("dropped_events")) is int and row["dropped_events"] == 0
                        and row.get("io_failed") is False,
                        "OPENVINS_OBSERVATION_INCOMPLETE", "footer reports missing or failed observations")
                closed = True
                continue
            require(total <= MAX_BYTES, "OPENVINS_OBSERVATION_LIMIT", "native event byte bound exceeded")
            require(type(row.get("sequence")) is int and row["sequence"] == count,
                    "OPENVINS_OBSERVATION_INVALID", "native sequence gap or duplicate")
            count += 1
            require(count <= MAX_EVENTS, "OPENVINS_OBSERVATION_LIMIT", "native event bound exceeded")
            require(row.get("sensor_time_double_bits") == _bits(row.get("sensor_time_seconds"))
                    and type(row.get("sensor_time_double_bits")) is int
                    and isinstance(row.get("values"), list)
                    and all(value is None or _finite(value) for value in row["values"])
                    and isinstance(row.get("sensor_ids"), list)
                    and all(type(value) is int and value >= 0 for value in row["sensor_ids"]),
                    "OPENVINS_OBSERVATION_INVALID", "native time bits, identities or values differ")
            yield row
    require(closed, "OPENVINS_OBSERVATION_INCOMPLETE", "native completion footer absent")


def _state(row):
    values = row["values"]
    require(values and _finite(values[0]) and values[0] >= 0 and int(values[0]) == values[0],
            "OPENVINS_OBSERVATION_INVALID", "state detail count")
    size = int(values[0])
    require(len(values) == 18 + size and all(_finite(v) for v in values),
            "OPENVINS_OBSERVATION_INVALID", "complete finite state evidence required")
    raw = values[1+size:]
    q = raw[1:5]
    require(abs(sum(v*v for v in q) - 1.0) <= 1e-5,
            "OPENVINS_OBSERVATION_INVALID", "native quaternion is not unit")
    return dict(camera_imu_offset_seconds=raw[0], quaternion_jpl_G_to_I_xyzw=q,
                position_world_m=raw[5:8], velocity_world_mps=raw[8:11],
                gyro_bias_radps=raw[11:14], accel_bias_mps2=raw[14:17])


def _support(row, callbacks):
    support = row.get("measurement_support")
    require(isinstance(support, dict) and type(support.get("schema_version")) is int
            and support["schema_version"] == 1,
            "OPENVINS_VISUAL_SUPPORT_MISSING", "accepted feature measurement support required")
    for key in ("feature_count", "measurement_count", "invalid_timestamps", "overflow_measurements"):
        _integer(support.get(key), key)
    entries = support.get("entries")
    require(support.get("complete") is True and support["invalid_timestamps"] == 0
            and support["overflow_measurements"] == 0 and support["feature_count"] > 0
            and isinstance(entries, list) and 0 < len(entries) <= 32,
            "OPENVINS_VISUAL_SUPPORT_INCOMPLETE", "accepted measurement support is incomplete")
    seen = set()
    associated = []
    for item in entries:
        require(isinstance(item, dict), "OPENVINS_OBSERVATION_INVALID", "support entry")
        camera = _integer(item.get("camera_id"), "camera ID")
        bits = _integer(item.get("sensor_time_double_bits"), "measurement double bits")
        count = _integer(item.get("observation_count"), "observation count", 1)
        key = camera, bits
        require(bits == _bits(item.get("sensor_time_seconds")) and key not in seen,
                "OPENVINS_OBSERVATION_INVALID", "duplicate support or inconsistent time bits")
        seen.add(key)
        require(key in callbacks, "OPENVINS_VISUAL_TIME_UNASSOCIATED",
                "accepted observation lacks an earlier exact native ROS callback")
        require(item["sensor_time_seconds"] <= row["sensor_time_seconds"],
                "OPENVINS_OBSERVATION_INVALID", "measurement is later than the corrected state")
        associated.append(dict(camera_id=camera, sensor_ns=callbacks[key], observation_count=count))
    require(sum(item["observation_count"] for item in associated) == support["measurement_count"],
            "OPENVINS_OBSERVATION_INVALID", "measurement support count differs")
    return associated


def _imu_callback_coverage(rows, input_directory):
    """Compare actual callback values to the independently reconciled observer."""
    from rm27.perception.vision.localization.experiments.common import file_hash
    source = Path(input_directory)/'imu0.jsonl'
    expected = []
    with source.open() as stream:
        for line in stream:
            row = json.loads(line)
            require(isinstance(row, dict), 'OPENVINS_OBSERVATION_INVALID', 'observer IMU object required')
            ns = _integer(row.get('state_ns'), 'observed IMU timestamp')
            gyro, accel = row.get('gyro'), row.get('accel')
            require(isinstance(gyro, list) and len(gyro) == 3
                    and isinstance(accel, list) and len(accel) == 3,
                    'OPENVINS_OBSERVATION_INVALID', 'observer IMU requires two three-axis vectors')
            values = gyro + accel
            require(row.get('frame_id') == 'imu0' and all(_finite(value) for value in values)
                    and (not expected or ns > expected[-1][0]),
                    'OPENVINS_OBSERVATION_INVALID', 'observer IMU identity, values or order')
            expected.append((ns, values))
            require(len(expected) <= MAX_EVENTS, 'OPENVINS_OBSERVATION_LIMIT', 'observer IMU bound')
    lookup = {ns: (index, values) for index, (ns, values) in enumerate(expected)}
    indices = []
    for ns, values in rows:
        require(ns in lookup and (not indices or lookup[ns][0] > indices[-1]),
                'OPENVINS_NATIVE_INPUT_MISMATCH', 'unexpected, duplicated or reordered IMU callback')
        index, reference = lookup[ns]
        require(values == reference, 'OPENVINS_NATIVE_INPUT_MISMATCH', 'actual callback IMU values differ')
        indices.append(index)
    received = set(indices)
    missing = []
    for index, (ns, _) in enumerate(expected):
        if index in received:
            continue
        if missing and missing[-1]['last_source_index'] + 1 == index:
            missing[-1].update(last_source_index=index, last_sensor_ns=ns, count=missing[-1]['count']+1)
        else:
            missing.append(dict(first_source_index=index, last_source_index=index,
                                first_sensor_ns=ns, last_sensor_ns=ns, count=1))
    return dict(status='PASS' if expected and not missing else 'FAIL',
        expected_observer_count=len(expected), native_callback_count=len(rows),
        present_values_exact=True, missing_count=len(expected)-len(rows), missing_intervals=missing,
        source='raw/imu0.jsonl', source_sha256=file_hash(source),
        scope='native ROS IMU callback only; downstream integration and image pixel equality not qualified')


def analyze_openvins_observations(path, *, input_directory=None, include_camera_transactions=False):
    """Require complete native evidence; never infer raw ingestion or health.

    The optional projection closes actual CameraData processing transactions.
    It reuses this decoder's accepted updates, retaining the historical summary
    shape by default. One stereo CameraData transaction is not two frames.
    """
    require(type(include_camera_transactions) is bool, "OPENVINS_OBSERVATION_INVALID",
            "camera transaction projection requires an explicit boolean")
    callbacks = {}
    counts = Counter()
    updates = []
    initializations = []
    last_visual_ns = None
    ticks = []
    imu_callbacks = []
    transactions = []
    active_transaction = None
    unbound_updates = 0
    unbound_returned_states = 0
    last_transaction_native_state = None
    for row in _events(path):
        kind = row["kind"]
        counts[kind] += 1
        if kind in {"camera_ros_callback", "imu_ros_callback"}:
            sec = _integer(row.get("raw_sec"), "raw ROS sec")
            nsec = _integer(row.get("raw_nsec"), "raw ROS nsec")
            require(nsec < 10**9 and len(row["sensor_ids"]) == 1
                    and _bits(sec + nsec*1e-9) == row["sensor_time_double_bits"],
                    "OPENVINS_OBSERVATION_INVALID", "raw ROS time and native double differ")
            if kind == 'imu_ros_callback':
                require(row['sensor_ids'] == [0] and len(row['values']) == 6
                        and all(_finite(value) for value in row['values']),
                        'OPENVINS_OBSERVATION_INVALID', 'native IMU callback identity and values')
                imu_callbacks.append((sec*10**9+nsec, row['values']))
                continue
            key = row["sensor_ids"][0], row["sensor_time_double_bits"]
            require(key not in callbacks, "OPENVINS_VISUAL_TIME_AMBIGUOUS",
                    "duplicate or colliding native camera timestamp")
            callbacks[key] = sec*10**9+nsec
        elif kind == "metric_initialization_committed":
            initializations.append(dict(sequence=row["sequence"], monotonic_ns=row["monotonic_ns"],
                                        state_time_seconds=row["sensor_time_seconds"], state=_state(row)))
        elif kind in VISUAL_UPDATES:
            state = _state(row)
            require(initializations, "OPENVINS_OBSERVATION_INVALID", "visual correction before metric initialization")
            support = _support(row, callbacks)
            latest = max(item["sensor_ns"] for item in support)
            advances = last_visual_ns is None or latest > last_visual_ns
            if advances:
                last_visual_ns = latest
            updates.append(dict(kind=kind, sequence=row["sequence"], monotonic_ns=row["monotonic_ns"],
                state_time_seconds=row["sensor_time_seconds"], state_time_double_bits=row["sensor_time_double_bits"],
                measurement_support=support, oldest_measurement_ns=min(item["sensor_ns"] for item in support),
                newest_measurement_ns=latest, advances_visual_measurement_time=advances,
                last_visual_measurement_ns=last_visual_ns, state=state))
        elif kind == "independent_observation_tick":
            ticks.append(row["monotonic_ns"])
        if include_camera_transactions:
            if kind == "camera_processing_begin":
                require(active_transaction is None and row["sensor_ids"]
                        and len(set(row["sensor_ids"])) == len(row["sensor_ids"])
                        and row["sensor_time_seconds"] >= 0,
                        "OPENVINS_TRANSACTION_BOUNDARY_INVALID", "nested or invalid camera transaction")
                active_transaction = dict(begin=row, update_start=len(updates),
                                          previous_visual_ns=last_visual_ns)
            elif kind in VISUAL_UPDATES:
                if active_transaction is None:
                    unbound_updates += 1
                else:
                    require(row["sensor_time_double_bits"] == active_transaction["begin"]["sensor_time_double_bits"],
                            "OPENVINS_TRANSACTION_BOUNDARY_INVALID", "accepted update state differs from camera transaction")
            elif kind == "camera_processing_returned":
                require(active_transaction is not None,
                        "OPENVINS_TRANSACTION_BOUNDARY_INVALID", "camera return without a processing begin")
                begin = active_transaction["begin"]
                require(row["sensor_time_double_bits"] == begin["sensor_time_double_bits"]
                        and row["sensor_ids"] == begin["sensor_ids"],
                        "OPENVINS_TRANSACTION_BOUNDARY_INVALID", "camera transaction time or IDs differ")
                accepted = updates[active_transaction["update_start"]:]
                support = Counter()
                for update in accepted:
                    for item in update["measurement_support"]:
                        support[item["camera_id"], item["sensor_ns"]] += item["observation_count"]
                last = accepted[-1] if accepted else None
                state = None
                if last is not None:
                    native = last["state"]
                    q = native["quaternion_jpl_G_to_I_xyzw"]
                    state = dict(parent_frame="backend_world", child_frame="imu",
                        position_world_m=native["position_world_m"], orientation_wxyz=[q[3], *q[:3]],
                        orientation_convention="JPL_GtoI_xyzw_equivalent_Hamilton_ItoG_xyzw",
                        velocity_world_mps=native["velocity_world_mps"], velocity_frame="backend_world",
                        gyro_bias_radps=native["gyro_bias_radps"], accel_bias_mps2=native["accel_bias_mps2"],
                        translation_unit="m", velocity_unit="m/s",
                        camera_imu_offset_seconds=native["camera_imu_offset_seconds"],
                        state_time_seconds=last["state_time_seconds"],
                        state_time_double_bits=last["state_time_double_bits"],
                        state_time_semantic="NATIVE_ESTIMATOR_CAMERA_STATE_DOUBLE_NOT_RAW_SENSOR_NS",
                        source_event_sequence=last["sequence"], native_monotonic_ns=last["monotonic_ns"])
                transactions.append(dict(camera_ids=begin["sensor_ids"],
                    camera_time_seconds=begin["sensor_time_seconds"],
                    camera_time_double_bits=begin["sensor_time_double_bits"],
                    begin_event_sequence=begin["sequence"], end_event_sequence=row["sequence"],
                    begin_native_monotonic_ns=begin["monotonic_ns"], end_native_monotonic_ns=row["monotonic_ns"],
                    accepted_update_sequences=[update["sequence"] for update in accepted],
                    accepted_update_count=len(accepted),
                    merged_measurement_support=[dict(camera_id=camera, sensor_ns=ns,
                        observation_use_count=count) for (camera, ns), count in sorted(support.items())],
                    support_count_semantic="accepted observation uses summed across updates; not unique features or information rank",
                    newest_accepted_measurement_ns=max((ns for _, ns in support), default=None),
                    last_visual_measurement_ns=last_visual_ns,
                    advances_visual_measurement_time=last_visual_ns != active_transaction["previous_visual_ns"],
                    state_source="NATIVE_ACCEPTED_VISUAL_CORRECTION" if last else "NO_ACCEPTED_VISUAL_UPDATE",
                    last_accepted_snapshot=state, returned_state_event_sequence=None,
                    returned_state_matches_last_accepted_snapshot=None,
                    initialization_commit_event_sequence=initializations[-1]["sequence"] if initializations else None,
                    tracking_state="UNKNOWN", validity=False, contract_complete=False,
                    native_epoch=None, native_reset_count=None, covariance=None,
                    native_publication_timestamp=None, sensor_to_native_clock_mapping=None))
                last_transaction_native_state = last["state"] if last else None
                active_transaction = None
            elif kind == "camera_returned_state":
                require(active_transaction is None, "OPENVINS_TRANSACTION_BOUNDARY_INVALID",
                        "returned state precedes camera transaction completion")
                if not transactions:
                    unbound_returned_states += 1
                else:
                    transaction = transactions[-1]
                    require(transaction["returned_state_event_sequence"] is None,
                            "OPENVINS_TRANSACTION_BOUNDARY_INVALID", "duplicate returned state for camera transaction")
                    transaction["returned_state_event_sequence"] = row["sequence"]
                    native = _state(row)
                    last = transaction["last_accepted_snapshot"]
                    if last is not None:
                        transaction["returned_state_matches_last_accepted_snapshot"] = (
                            row["sensor_time_double_bits"] == last["state_time_double_bits"]
                            and native == last_transaction_native_state)
    if include_camera_transactions:
        require(active_transaction is None, "OPENVINS_TRANSACTION_BOUNDARY_INVALID", "camera transaction has no return")
    observed = bool(initializations and updates)
    input_coverage = (_imu_callback_coverage(imu_callbacks, input_directory) if input_directory is not None
                      else dict(status='NOT_RUN', reason='independent observer input was not supplied'))
    result = dict(schema_version=1, status="PASS" if observed else "NOT_RUN",
        reason=None if observed else "metric initialization or accepted visual correction not observed",
        observation_integrity="PASS", scope="NATIVE_OBSERVATION_INTEGRITY_ONLY",
        event_counts=dict(counts), initialization_events=initializations, visual_updates=updates,
        accepted_visual_update_count=len(updates), distinct_visual_time_advance_count=sum(
            row["advances_visual_measurement_time"] for row in updates),
        last_visual_measurement_ns=last_visual_ns, exact_native_camera_callback_count=len(callbacks),
        independent_tick_count=len(ticks), observation_complete=True, truth_used=False,
        health_qualification="NOT_RUN", automatic_reset="NOT_RUN", raw_input_equality="NOT_RUN",
        native_imu_callback_coverage=input_coverage,
        time_association="EXACT_DOUBLE_BITS_TO_EARLIER_NATIVE_INTEGER_ROS_STAMP",
        scope_limits="accepted feature support is not per-row information rank, raw image ingestion, tracking health or flight qualification")
    if include_camera_transactions:
        result["camera_transactions"] = dict(schema_version=1,
            status="PASS" if transactions and not unbound_updates and not unbound_returned_states else "NOT_RUN",
            scope="OFFLINE_NATIVE_CAMERA_TRANSACTION_PROJECTION_ONLY", records=transactions,
            source=dict(path=str(path), sha256=file_hash(path),
                        event_identity="original source event sequence and process-local native monotonic_ns"),
            transaction_count=len(transactions), unbound_accepted_update_count=unbound_updates,
            unbound_returned_state_count=unbound_returned_states,
            accepted_transaction_count=sum(bool(row["accepted_update_count"]) for row in transactions),
            visual_time_advance_transaction_count=sum(row["advances_visual_measurement_time"] for row in transactions),
            measurement_time_semantic="original integer native ROS camera stamp of accepted support; not state or publication time",
            native_clock_semantic="one native steady clock; no observer or sensor clock mapping inferred",
            health_qualification="NOT_RUN", automatic_reset="NOT_RUN", truth_used=False,
            limitations=["Numerical metadata only; all tracking UNKNOWN and validity false.",
                         "Transactions without accepted updates, ZUPT, propagation and reused older support never refresh visual measurement time.",
                         "Merged support counts observation uses, not unique observations or independent information.",
                         "Native epoch/reset, publication time, covariance and clock mappings remain unknown."])
    return result


def qualify_state_freshness(path, output, *, policy, session_id, sensor_clock):
    """Project actual native events using explicit offline freshness limits.

    CURRENT means the observed state/visual/input evidence meets these test
    deadlines. It never means operational TRACKING: OpenVINS has no qualified
    native health or reset signal in this build. Preserve the partial v2
    UNKNOWN/invalid contract, independently of the numerical trajectory.
    Native event times are serialization witnesses, not hardware receipt or
    publication times. No sensor/native clock mapping is inferred.
    """
    from rm27.perception.vision.localization.state_output import StateOutputPolicy

    require(isinstance(policy, dict) and set(policy) == set(StateOutputPolicy.__dataclass_fields__),
            'OPENVINS_HEALTH_CONFIG_INVALID', 'exact StateOutputPolicy fields required')
    limits = StateOutputPolicy(**policy)
    require(isinstance(session_id, str) and session_id.strip()
            and isinstance(sensor_clock, str) and sensor_clock.strip() and sensor_clock != 'UNKNOWN',
            'OPENVINS_HEALTH_CONFIG_INVALID', 'explicit process session and recorded sensor clock required')
    # The existing decoder remains the sole authority for accepted support and
    # the original integer camera timestamps. It also checks the normal footer.
    observed = analyze_openvins_observations(path, include_camera_transactions=True)
    accepted = {row['sequence']: row for row in observed['visual_updates']}
    require(not accepted or observed['camera_transactions']['status'] == 'PASS',
            'OPENVINS_HEALTH_TRANSACTION_INCOMPLETE', 'accepted correction requires completed camera transaction evidence')
    callbacks = {}
    last_sensor = last_imu = last_progress = last_visual_event = last_visual_ns = None
    sensor_horizon = last_prediction_ns = None
    state_time = state_kind = None
    init_commit = ready = False
    imu_previous = None
    input_gap_after = None
    counts = Counter()
    transitions = []
    previous_freshness = None
    first_sensor_observation = None
    first_ready = None
    last_row = None
    output = Path(output)
    with output.open('x') as target:
        for row in _events(path):
            kind, now = row['kind'], row['monotonic_ns']
            relevant = False
            if kind in {'camera_ros_callback', 'imu_ros_callback'}:
                ns = row['raw_sec']*10**9 + row['raw_nsec']
                last_sensor = now
                first_sensor_observation = now if first_sensor_observation is None else first_sensor_observation
                sensor_horizon = ns if sensor_horizon is None else max(sensor_horizon, ns)
                if kind == 'imu_ros_callback':
                    last_imu = now
                    key = row['sensor_time_double_bits']
                    require(key not in callbacks, 'OPENVINS_HEALTH_TIME_AMBIGUOUS',
                            'prediction cannot associate colliding raw IMU timestamp doubles')
                    callbacks[key] = ns
                    if imu_previous is not None:
                        require(ns > imu_previous, 'OPENVINS_HEALTH_TIME_INVALID', 'IMU timestamp did not advance')
                        if ns-imu_previous > limits.max_imu_gap_ns:
                            input_gap_after = ns
                    imu_previous = ns
            elif kind == 'metric_initialization_committed':
                require(not init_commit, 'OPENVINS_HEALTH_EPOCH_UNOBSERVED',
                        'second initialization requires a real native reset/epoch witness')
                init_commit = True
                relevant = True
            elif kind == 'camera_returned_state':
                require(init_commit, 'OPENVINS_HEALTH_INITIALIZATION_INVALID', 'ready state before initialization commit')
                # This event is emitted only inside the actual initialized()
                # branch, after asynchronous initialization catch-up returns.
                require(row['values'][0:2] == [1, 1.0], 'OPENVINS_HEALTH_INITIALIZATION_INVALID',
                        'native initialized() witness is absent')
                if not ready:
                    ready, first_ready, relevant = True, now, True
            elif kind in VISUAL_UPDATES:
                update = accepted[row['sequence']]
                state_kind = 'VISUAL_CORRECTION'
                state_time = dict(seconds=row['sensor_time_seconds'], double_bits=row['sensor_time_double_bits'],
                                  semantic='NATIVE_CAMERA_STATE_DOUBLE')
                if update['advances_visual_measurement_time']:
                    last_visual_ns = update['last_visual_measurement_ns']
                    last_visual_event = last_progress = now
                    if input_gap_after is not None and last_visual_ns >= input_gap_after:
                        input_gap_after = None
                relevant = True
            elif kind in {'imu_fast_prediction_completed', 'imu_fast_prediction_unavailable'}:
                values = row['values']
                success = kind == 'imu_fast_prediction_completed'
                require(len(values) == (7 if success else 5) and all(_finite(value) for value in values)
                        and min(values[:4]) >= 0,
                        'OPENVINS_HEALTH_PREDICTION_INVALID', 'complete native prediction witness required')
                require(row['sensor_time_double_bits'] in callbacks and init_commit,
                        'OPENVINS_HEALTH_PREDICTION_INVALID', 'prediction lacks earlier native IMU callback/initialization')
                ns = callbacks[row['sensor_time_double_bits']]
                # Use operands witnessed inside this prediction call. Camera
                # updates and fast prediction can overlap: the most recently
                # serialized main-state offset is not this cache's offset.
                require(row['sensor_time_seconds'] >= 0
                        and _bits(values[0]) == _bits(values[3]+values[4])
                        and _bits(values[1]) == _bits(row['sensor_time_seconds']+values[4]),
                        'OPENVINS_HEALTH_PREDICTION_INVALID', 'integration interval differs from native cache operands')
                if kind == 'imu_fast_prediction_completed':
                    require(values[0] < values[1] and values[5:] == [1, 1],
                            'OPENVINS_HEALTH_PREDICTION_INVALID', 'prediction interval or finite state/covariance failed')
                    if last_prediction_ns is None or ns > last_prediction_ns:
                        last_progress = now
                        last_prediction_ns = ns
                    state_kind = 'IMU_PREDICTION'
                    state_time = dict(seconds=values[1], double_bits=_bits(values[1]),
                        semantic='NATIVE_FAST_PROPAGATION_INTEGRATION_ENDPOINT', request_sensor_ns=ns,
                        integration_start_seconds=values[0], main_filter_camera_state_seconds=values[2],
                        main_filter_snapshot_semantic='ASYNCHRONOUS_SAMPLE_NOT_COHERENT_PREDICTION_STATE',
                        cache_state_time_seconds=values[3], cache_camera_imu_offset_seconds=values[4])
                relevant = True
            elif kind == 'imu_state_propagation_completed':
                # Camera-driven clone propagation is a different lane from
                # the high-rate prediction output; do not replace that output.
                relevant = True
            elif kind == 'independent_observation_tick':
                require(row['sensor_time_seconds'] == -1 and not row['values'],
                        'OPENVINS_HEALTH_TIME_INVALID', 'watchdog must not carry a sensor timestamp/state')
                relevant = True
            elif kind in {'reset', 'estimator_reset', 'reinitialization'}:
                require(False, 'OPENVINS_HEALTH_EPOCH_UNOBSERVED', 'unsupported native reset witness')
            if not relevant:
                continue
            counts[kind] += 1
            reasons = []
            if not ready:
                reasons.append('INITIALIZATION_NOT_READY')
            if last_visual_ns is None:
                reasons.append('ACCEPTED_VISUAL_UPDATE_UNOBSERVED')
            else:
                if sensor_horizon is not None and sensor_horizon-last_visual_ns > limits.max_visual_age_ns:
                    reasons.append('VISUAL_MEASUREMENT_STALE')
                if now-last_visual_event > limits.max_visual_receive_age_ns:
                    reasons.append('VISUAL_EVENT_STALE')
            if last_sensor is None or now-last_sensor > limits.max_receive_age_ns:
                reasons.append('SENSOR_STREAM_STOPPED')
            if last_imu is None or now-last_imu > limits.max_receive_age_ns:
                reasons.append('IMU_STREAM_STOPPED')
            if last_progress is None or now-last_progress > limits.max_receive_age_ns:
                reasons.append('STATE_PROGRESS_STOPPED')
            if input_gap_after is not None:
                reasons.append('IMU_INPUT_GAP_AFTER_LAST_VISUAL')
            freshness = 'CURRENT' if not reasons else ('INITIALIZING' if not ready else 'STALE')
            last_row = dict(native_event_sequence=row['sequence'], event_kind=kind,
                native_observation_ns=now, session_id=session_id, adapter_epoch=0,
                native_epoch=None, native_reset_count=None,
                lifecycle='INITIALIZED' if ready else ('INITIALIZATION_COMMITTED' if init_commit else 'INITIALIZING'),
                freshness=freshness, freshness_reasons=reasons,
                state_source=state_kind, state_time=state_time,
                last_sensor_observation_ns=last_sensor, last_imu_observation_ns=last_imu,
                last_state_progress_observation_ns=last_progress,
                last_visual_observation_ns=last_visual_event, last_visual_sensor_ns=last_visual_ns,
                sensor_clock=sensor_clock, tracking_state='UNKNOWN', valid=False, contract_complete=False,
                operational_health='UNKNOWN', native_publication_timestamp=None)
            if previous_freshness != (freshness, tuple(reasons)):
                transitions.append(dict(last_row))
                previous_freshness = freshness, tuple(reasons)
            target.write(json.dumps(last_row, allow_nan=False, separators=(',', ':'))+'\n')
    complete = bool(ready and accepted and counts['imu_fast_prediction_completed'] and counts['independent_observation_tick'])
    return dict(schema_version=1, status='PASS' if complete else 'PARTIAL',
        scope='PC_OFFLINE_NATIVE_OBSERVABILITY_AND_CONFIGURED_FRESHNESS_ONLY', truth_used=False,
        policy=policy, policy_scope='TEST_LIMITS_NOT_FLIGHT_SAFETY; max_buffer_samples unused (no independent integration)',
        session_id=session_id, adapter_epoch_semantic='ONE_ISOLATED_PROCESS_SESSION_NOT_NATIVE_RESET_COUNT',
        timeline=dict(path=output.name, sha256=file_hash(output), event_counts=dict(counts)),
        initialization=dict(commit_observed=init_commit, ready_observed=ready,
            first_sensor_observation_ns=first_sensor_observation, first_ready_observation_ns=first_ready,
            native_observation_elapsed_ns=(first_ready-first_sensor_observation if ready else None)),
        transitions=transitions, final=last_row,
        native_event_source=dict(path=str(path), sha256=file_hash(path)),
        native_clock_semantic='PROCESS_LOCAL_STEADY_EVENT_SERIALIZATION_NOT_HARDWARE_RECEIPT',
        sensor_to_native_clock_mapping=None, operational_health='UNKNOWN', operational_valid=False,
        failure_detection='CONFIGURED_FRESHNESS_ONLY_NOT_NATIVE_TRACKING_FAILURE',
        recovery='FRESHNESS_ONLY_NOT_REINITIALIZATION_OR_TRACKING_QUALIFICATION',
        native_reset=dict(status='BLOCKED', execution='NOT_RUN', reason='pinned OpenVINS has no runtime reset API'),
        covariance=dict(status='UNQUALIFIED', native_finite_witness_only=True,
            reason='native marginal/fast covariance exists; finite values do not establish observability or tracking health'))


def _openvins_ros_header_ns(seconds):
    """Pinned ROSVisualizerHelper::get_time_from_seconds, including carry.

    This is the upstream publication header conversion of the *request*, not
    the prediction's integration endpoint or an original integer sensor stamp.
    C++ std::round rounds nonnegative halfway values away from zero.
    """
    require(_finite(seconds) and 0 <= seconds < 2**32,
            'OPENVINS_LAB_TIME_INVALID', 'request is outside native ROS time range')
    sec = math.floor(seconds)
    nsec = math.floor((seconds-sec)*1e9 + .5)
    return (sec+nsec//10**9)*10**9 + nsec % 10**9


def _openvins_mcap_odometry(path):
    """Read actual complete messages; JSON observers omit angular velocity."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    while reader.has_next():
        topic, data, recorded_ns = reader.read_next()
        require(topic in ('/poseimu', '/odomimu'), 'OPENVINS_LAB_MCAP_INVALID', 'unexpected output topic')
        if topic != '/odomimu':
            continue
        message = deserialize_message(data, Odometry)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear, message.twist.twist.angular
        yield dict(state_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
            frame_id=message.header.frame_id, child_frame_id=message.child_frame_id,
            position=[p.x, p.y, p.z], quaternion_xyzw=[q.x, q.y, q.z, q.w],
            velocity_local=[v.x, v.y, v.z], angular_velocity_local=[w.x, w.y, w.z],
            cdr_sha256=hashlib.sha256(data).hexdigest(), bag_record_timestamp_ns=recorded_ns)


def load_openvins_laboratory_samples(run, *, event_sequences):
    """Extract sealed real outputs for explicitly unqualified reception tests.

    Each selector names an existing health-timeline event. Select the first
    successful prediction at or after it, then require its exact request-header
    match to one complete MCAP ODOMETRY message. No nearest-time association,
    interpolation, truth, zero substitution, clock mapping or validity upgrade
    occurs. The run manifest and required artifacts are checked before/after;
    the returned manifest hash must be anchored by the caller's experiment.
    """
    from rm27.perception.vision.localization.experiments.openvins import PIN
    from rm27.perception.vision.localization.experiments.trajectory import rotation_wxyz

    code = 'OPENVINS_LAB_SOURCE_INVALID'
    require(isinstance(event_sequences, list) and 0 < len(event_sequences) <= 16
            and all(type(seq) is int and seq >= 0 for seq in event_sequences)
            and len(set(event_sequences)) == len(event_sequences), code, 'one to sixteen unique event selectors required')
    root = Path(run).resolve()
    manifest_path = root/'run.json'
    manifest_hash = file_hash(manifest_path)
    manifest = json.loads(manifest_path.read_text())
    install, config = manifest.get('installation', {}), manifest.get('config', {})
    require(manifest.get('status') == 'EXECUTION_COMPLETED' and manifest.get('exit_code') == 0
            and config.get('backend') == 'OpenVINS' and install.get('commit') == PIN
            and install.get('native_health_observation', {}).get('schema_version') == 2,
            code, 'completed pinned native cache-witness run required')
    artifacts = manifest.get('output_sha256', {})
    needed = ['raw/native-events.jsonl', 'native-health.json', 'online/native-health.jsonl',
              'raw/odom.jsonl', 'raw/online-output/metadata.yaml']
    mcap = sorted(root.glob('raw/online-output/*.mcap'))
    require(mcap, code, 'sealed MCAP data absent')
    needed += [str(path.relative_to(root)) for path in mcap]
    require({key for key in artifacts if key.startswith('raw/online-output/')} == set(needed[4:]),
            code, 'MCAP artifact set differs from sealed manifest')
    def verify():
        require(file_hash(manifest_path) == manifest_hash, code, 'run manifest changed during extraction')
        for name in needed:
            path = root/name
            require(not path.is_symlink() and path.resolve().is_relative_to(root) and path.is_file()
                    and name in artifacts and file_hash(path) == artifacts[name],
                    code, 'required sealed artifact differs: '+name)
    verify()
    health = json.loads((root/'native-health.json').read_text())
    session = config.get('run_id')
    require(isinstance(session, str) and session and health.get('session_id') == session
            and health.get('status') == 'PASS' and health.get('truth_used') is False
            and health.get('native_event_source', {}).get('sha256') == artifacts['raw/native-events.jsonl']
            and health.get('timeline', {}).get('sha256') == artifacts['online/native-health.jsonl'],
            code, 'health summary is not bound to sealed native events/timeline/session')
    observed = analyze_openvins_observations(root/'raw/native-events.jsonl', include_camera_transactions=True)
    require(observed['camera_transactions']['status'] == 'PASS', code, 'complete accepted camera transactions required')
    callbacks, predictions = {}, []
    for row in _events(root/'raw/native-events.jsonl'):
        if row['kind'] == 'imu_ros_callback':
            key = row['sensor_time_double_bits']
            require(key not in callbacks, code, 'ambiguous native IMU request double')
            callbacks[key] = row['raw_sec']*10**9+row['raw_nsec']
        elif row['kind'] == 'imu_fast_prediction_completed':
            v = row['values']
            require(len(v) == 7 and all(_finite(value) for value in v) and v[5:] == [1, 1]
                    and row['sensor_time_double_bits'] in callbacks
                    and 0 <= v[0] < v[1] and _bits(v[0]) == _bits(v[3]+v[4])
                    and _bits(v[1]) == _bits(row['sensor_time_seconds']+v[4]),
                    code, 'prediction lacks exact native cache/IMU witnesses')
            predictions.append((row, callbacks[row['sensor_time_double_bits']], _openvins_ros_header_ns(row['sensor_time_seconds'])))
    require(predictions and len({item[2] for item in predictions}) == len(predictions),
            code, 'prediction request headers absent or ambiguous')
    sequences = [item[0]['sequence'] for item in predictions]
    chosen = []
    for selector in event_sequences:
        index = bisect_left(sequences, selector)
        require(index < len(predictions), code, 'no successful prediction at or after selected health event')
        chosen.append(predictions[index])
    wanted = set(event_sequences) | {item[0]['sequence'] for item in chosen}
    timeline = {}
    with (root/'online/native-health.jsonl').open() as stream:
        previous = -1
        for line in stream:
            row = json.loads(line)
            seq = row.get('native_event_sequence')
            require(type(seq) is int and seq > previous and row.get('session_id') == session,
                    code, 'health timeline sequence/session differs')
            previous = seq
            if seq in wanted:
                require(row.get('tracking_state') == 'UNKNOWN' and row.get('valid') is False
                        and row.get('native_epoch') is None and row.get('native_reset_count') is None,
                        code, 'unqualified native health/epoch must remain unknown and invalid')
                timeline[seq] = row
    require(wanted == set(timeline), code, 'selector or selected prediction lacks an exact health row')
    observer = []
    with (root/'raw/odom.jsonl').open() as stream:
        observer = [json.loads(line) for line in stream]
    selected_headers = {item[2] for item in chosen}
    messages, headers = {}, set()
    count = 0
    for ordinal, row in enumerate(_openvins_mcap_odometry(root/'raw/online-output')):
        stamp = row['state_ns']
        require(type(stamp) is int and stamp not in headers and ordinal < len(observer),
                code, 'duplicate MCAP odometry header or count mismatch')
        headers.add(stamp)
        require(row.get('frame_id') == 'global' and row.get('child_frame_id') == 'imu'
                and all(row[key] == observer[ordinal][key] for key in
                        ('state_ns', 'position', 'quaternion_xyzw', 'velocity_local')),
                code, 'MCAP and sealed same-message observer/frame differ')
        for key, size in [('position', 3), ('quaternion_xyzw', 4), ('velocity_local', 3), ('angular_velocity_local', 3)]:
            require(isinstance(row.get(key), list) and len(row[key]) == size and all(_finite(v) for v in row[key]),
                    code, 'complete finite native message required: '+key)
        q = row['quaternion_xyzw']
        rotation_wxyz([q[3], *q[:3]])
        if stamp in selected_headers:
            messages[stamp] = dict(row, mcap_odometry_ordinal=ordinal)
        count += 1
    require(count == len(observer) == len(predictions) and set(messages) == selected_headers
            and headers == {item[2] for item in predictions}, code, 'complete exact prediction/MCAP/observer association required')
    result = []
    update_sequences = [row['sequence'] for row in observed['visual_updates']]
    for selector, (event, raw_ns, stamp) in zip(event_sequences, chosen):
        message, state_health = messages[stamp], timeline[event['sequence']]
        require(state_health.get('event_kind') == event['kind']
                and state_health.get('state_time', {}).get('double_bits') == _bits(event['values'][1]),
                code, 'health row does not describe this actual prediction endpoint')
        last_index = bisect_left(update_sequences, event['sequence'])-1
        last_update = observed['visual_updates'][last_index] if last_index >= 0 else None
        require(last_update is not None and state_health.get('last_visual_sensor_ns') == last_update['last_visual_measurement_ns'],
                code, 'health visual support differs from accepted native measurement evidence')
        v = event['values']
        result.append(dict(schema_version=1, scope='UNQUALIFIED_LABORATORY_RECEPTION_ONLY',
            requested_event_sequence=selector, selected_event_sequence=event['sequence'],
            selection_semantic='FIRST_SUCCESSFUL_NATIVE_PREDICTION_AT_OR_AFTER_HEALTH_EVENT',
            odometry={key:message[key] for key in ('state_ns', 'frame_id', 'child_frame_id', 'position',
                'quaternion_xyzw', 'velocity_local', 'angular_velocity_local')},
            selected_health_row=timeline[selector], health_row=state_health, health_summary=health,
            accepted_visual_support=last_update,
            source=dict(run=str(root), run_manifest_sha256=manifest_hash, session_id=session,
                artifacts_sha256={name:artifacts[name] for name in needed},
                native_commit=install['commit'], native_binary_sha256=install.get('binary_sha256'),
                mcap_odometry_ordinal=message['mcap_odometry_ordinal'], cdr_sha256=message['cdr_sha256'],
                bag_record_timestamp_ns=message['bag_record_timestamp_ns'],
                sensor_clock=state_health['sensor_clock'], clock_mapping=None, native_epoch=None, native_reset_count=None,
                tracking_health='UNKNOWN', covariance_qualification='UNQUALIFIED', operational_valid=False, truth_used=False,
                native_prediction=dict(request_seconds=event['sensor_time_seconds'], request_double_bits=event['sensor_time_double_bits'],
                request_raw_sensor_ns=raw_ns, native_observation_ns=event['monotonic_ns'], integration_start_seconds=v[0],
                integration_endpoint_seconds=v[1], integration_endpoint_double_bits=_bits(v[1]),
                main_filter_camera_state_seconds=v[2], main_filter_snapshot_semantic='ASYNCHRONOUS_NOT_COHERENT_PREDICTION_STATE',
                cache_state_time_seconds=v[3], cache_camera_imu_offset_seconds=v[4],
                header_semantic='PRESERVED_UPSTREAM_ROS_REQUEST_DOUBLE_CONVERSION_NOT_INTEGRATION_ENDPOINT'))))
    verify()
    return result


RAW_KEYS = {"schema_version", "provenance_valid", "occurrence", "camera_id",
    "raw_sensor_timestamp_ns", "frame_id_hex", "encoding_hex", "raw_width",
    "raw_height", "raw_step", "raw_data_size", "raw_data_sha256"}
PIXEL_KEYS = {"width", "height", "step", "type", "channels", "continuous",
    "logical_byte_count", "hash_valid", "logical_pixels_sha256"}
PIXEL_STAGES = {"camera_decoded_queue_insert", "camera_feed_measurement_enter",
    "camera_manager_enter", "camera_frontend_feed_enter", "camera_frontend_feed_returned",
    "camera_feed_measurement_returned", "camera_queued_not_fed_before_shutdown"}
TERMINALS = {"camera_track_frequency_rejected", "camera_cv_bridge_rejected",
    "camera_cv_bridge_bundle_rejected", "camera_subscription_not_forwarded_before_shutdown",
    "camera_receipt_pointer_reused_without_forward", "camera_queued_not_fed_before_shutdown",
    "camera_feed_measurement_returned"}
PAYLOAD_KINDS = PIXEL_STAGES | TERMINALS | {"camera_subscription_received",
    "camera_matched_callback_enter", "camera_receipt_capacity_or_closed_rejected",
    "camera_matched_callback_receipt_missing"}
PIPELINE = ["camera_subscription_received", "camera_matched_callback_enter",
    "camera_decoded_queue_insert", "camera_feed_measurement_enter", "camera_manager_enter",
    "camera_frontend_feed_enter", "camera_frontend_feed_returned", "camera_feed_measurement_returned"]
CAMERA_PROCESSING_KINDS = set(PIPELINE[3:]) | {
    "camera_processing_begin", "camera_processing_returned", "camera_returned_state"}
CAMERA_CALLBACK_KINDS = {
    "camera_subscription_received", "camera_matched_callback_enter", "camera_ros_callback",
    "camera_decoded_queue_insert", "camera_track_frequency_rejected", "camera_cv_bridge_rejected",
    "camera_cv_bridge_bundle_rejected", "camera_receipt_pointer_reused_without_forward",
    "camera_receipt_capacity_or_closed_rejected", "camera_matched_callback_receipt_missing"}
HEX = re.compile(r"(?:[0-9a-f]{2})*\Z")
SHA = re.compile(r"[0-9a-f]{64}\Z")


def _need(value, detail, code="OPENVINS_CAMERA_OBSERVATION_INVALID"):
    require(value, code, detail)


def _raw_identity(payload):
    return {key: payload[key] for key in RAW_KEYS - {"occurrence"}}


def _pixels(image):
    return dict(width=int(image.shape[1]), height=int(image.shape[0]),
                logical_pixels_sha256=hashlib.sha256(image.tobytes()).hexdigest())


def _reference_record(camera_id, ns, frame, encoding, width, height, step, data, frontend):
    return dict(schema_version=1, provenance_valid=True, camera_id=camera_id,
        raw_sensor_timestamp_ns=ns, frame_id_hex=frame.encode().hex(), encoding_hex=encoding.encode().hex(),
        raw_width=width, raw_height=height, raw_step=step, raw_data_size=len(data),
        raw_data_sha256=hashlib.sha256(data).hexdigest(), decoded=frontend)


def build_camera_reference(dataset, input_directory, *, downsample_cameras, sample_limit=None):
    """Reuse the frozen ComparisonDataset; read original pixels, CDR and observer.

    ``sample_limit`` selects the first N messages of each declared camera for a
    bounded diagnostic, not an estimator-consumption qualification. No native log
    is read here. Delayed/reordered artificial inputs use declared delivery order.
    """
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    from tools.openvins.comparison_transport import image_pixels
    import cv2

    _need(type(downsample_cameras) is bool, "explicit downsample configuration required")
    if sample_limit is not None:
        _integer(sample_limit, "reference sample limit", 1)
    dataset.verify_bag()
    source_before = file_hash(dataset.archive)
    _need(source_before == dataset.provenance['source_sha256'], "source differs from frozen dataset manifest")
    cameras = [0, 1] if dataset.mode == "stereo" else [0]
    _need(dataset.mode in {"mono", "stereo"}, "unsupported camera configuration")
    streams = getattr(dataset, "delivered_streams", dataset.streams)
    records, evidence = {}, {}
    database = next(dataset.bag.glob("*.db3"))
    database_before = file_hash(database)
    with zipfile.ZipFile(dataset.archive) as source, sqlite3.connect(database.as_uri()+"?mode=ro", uri=True) as db:
        for camera_id in cameras:
            sensor = "cam"+str(camera_id)
            wanted = streams[sensor]
            wanted = wanted if sample_limit is None else wanted[:sample_limit]
            observer_path = Path(input_directory)/(sensor+".jsonl")
            observer_before = file_hash(observer_path)
            observer = []
            total_bytes = 0
            with observer_path.open('rb') as stream:
                while line := stream.readline(MAX_LINE_BYTES+1):
                    total_bytes += len(line)
                    _need(len(line) <= MAX_LINE_BYTES and total_bytes <= MAX_BYTES, "observer byte bound exceeded")
                    _need(len(observer) < MAX_EVENTS, "observer count limit")
                    observer.append(json.loads(line))
            _need(len(observer) == len(streams[sensor]), "observer source count differs")
            stored = db.execute("SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
                                "WHERE t.name=? ORDER BY m.id", ("/"+sensor+"/image_raw",))
            if sample_limit is not None:
                stored = itertools.islice(stored, sample_limit)
            current = []
            for ordinal, (expected, wire) in enumerate(itertools.zip_longest(wanted, stored)):
                _need(expected is not None and wire is not None, "CDR camera count differs")
                ns, filename = expected
                message = deserialize_message(wire[0], Image)
                pixels = image_pixels(source, sensor, filename)
                raw = bytes(message.data)
                _need((message.header.stamp.sec*10**9+message.header.stamp.nanosec == ns
                    and message.header.frame_id == sensor and message.encoding == "mono8"
                    and message.is_bigendian == 0 and message.height == pixels.shape[0]
                    and message.width == pixels.shape[1] and message.step == message.width
                    and raw == pixels.tobytes()), "source ZIP and CDR pixels/identity differ")
                obs = observer[ordinal]
                _need((obs.get("state_ns") == ns and obs.get("frame_id") == sensor
                    and obs.get("encoding") == "mono8" and obs.get("is_bigendian") == 0
                    and obs.get("height") == message.height and obs.get("width") == message.width
                    and obs.get("step") == message.step
                    and obs.get("pixels_sha256") == hashlib.sha256(raw).hexdigest()),
                    "source/CDR and observer pixels/identity/order differ")
                front = pixels
                if downsample_cameras:
                    _need(pixels.shape[0] % 2 == pixels.shape[1] % 2 == 0,
                          "odd-size pyrDown rounding not qualified by this decoder")
                    front = cv2.pyrDown(pixels, dstsize=(pixels.shape[1]//2, pixels.shape[0]//2))
                current.append(dict(source_ordinal=ordinal,
                    raw=_reference_record(camera_id, ns, sensor, "mono8", message.width,
                                          message.height, message.step, raw, _pixels(pixels)),
                    frontend=_pixels(front)))
            _need(observer_before == file_hash(observer_path), "observer changed during reference audit")
            records[camera_id] = current
            evidence[sensor] = dict(source_count=len(streams[sensor]), verified_count=len(current),
                observer_sha256=observer_before, observer_path=str(observer_path))
    _need(source_before == file_hash(dataset.archive) and database_before == file_hash(database),
          "source or database changed during reference audit")
    dataset.verify_bag()
    return dict(schema_version=1, complete=sample_limit is None,
        scope="SOURCE_CDR_OBSERVER_REFERENCE" if sample_limit is None else "BOUNDED_REFERENCE_SAMPLE_ONLY",
        downsample_cameras=downsample_cameras, camera_ids=cameras, records=records,
        source_archive=str(dataset.archive), source_archive_sha256=source_before,
        database=str(database), database_sha256=database_before,
        source_verification_sha256=dataset.manifest_sha256, evidence=evidence,
        native_consumption="NOT_RUN", truth_used=False)


def _payload(row):
    kind, payload = row["kind"], row.get("camera_consumption")
    _need(kind in PAYLOAD_KINDS and isinstance(payload, dict), "unsupported camera event or missing payload")
    _need(set(payload) == RAW_KEYS | ({"decoded"} if kind in PIXEL_STAGES else set()),
          "unsupported camera payload fields")
    _need(type(payload.get("schema_version")) is int and payload["schema_version"] == 1,
          "unsupported camera payload schema")
    _need(payload.get("provenance_valid") is True, "native camera provenance is incomplete",
          "OPENVINS_CAMERA_OBSERVATION_INCOMPLETE")
    for key in ("occurrence", "raw_width", "raw_height", "raw_step", "raw_data_size"):
        _integer(payload.get(key), key, 1)
    camera = _integer(payload.get("camera_id"), "camera ID")
    ns = _integer(payload.get("raw_sensor_timestamp_ns"), "raw camera timestamp")
    _need(payload["raw_step"] >= payload["raw_width"]
          and payload["raw_data_size"] == payload["raw_step"]*payload["raw_height"],
          "raw MONO8 dimensions and storage differ")
    sec, nsec = row.get("raw_sec"), row.get("raw_nsec")
    _need(type(sec) is int and type(nsec) is int and 0 <= nsec < 10**9
          and sec*10**9+nsec == ns and row["sensor_ids"] == [camera],
          "raw integer camera timestamp or ID differs")
    for key in ("frame_id_hex", "encoding_hex"):
        value = payload.get(key)
        _need(isinstance(value, str) and len(value) <= 1024 and HEX.fullmatch(value) is not None,
              "camera identity is not bounded hexadecimal bytes")
    _need(isinstance(payload.get("raw_data_sha256"), str)
          and SHA.fullmatch(payload["raw_data_sha256"]) is not None, "raw digest is invalid")
    if kind in PIXEL_STAGES:
        pixels = payload["decoded"]
        _need(isinstance(pixels, dict) and set(pixels) == PIXEL_KEYS, "unsupported decoded pixel schema")
        for key in ("width", "height", "step", "logical_byte_count"):
            _integer(pixels.get(key), "decoded "+key, 1)
        _need(type(pixels.get("type")) is int and pixels["type"] == 0
              and type(pixels.get("channels")) is int and pixels["channels"] == 1
              and type(pixels.get("continuous")) is bool and pixels.get("hash_valid") is True
              and pixels["step"] >= pixels["width"]
              and pixels["logical_byte_count"] == pixels["width"]*pixels["height"]
              and isinstance(pixels.get("logical_pixels_sha256"), str)
              and SHA.fullmatch(pixels["logical_pixels_sha256"]) is not None,
              "decoded MONO8 dimensions, storage or hash are invalid")
    return payload


def _camera_metadata(row, *, state_time=False):
    _need(type(row.get("raw_sec")) is int and type(row.get("raw_nsec")) is int
          and row["raw_sec"] == row["raw_nsec"] == -1
          and row["sensor_ids"] == []
          and (row["sensor_time_seconds"] >= 0 if state_time else row["sensor_time_seconds"] == -1),
          "camera lifecycle metadata/time differs")


def _camera_raw_stamp_valid(row):
    sec, nsec = row.get("raw_sec"), row.get("raw_nsec")
    return (type(sec) is int and sec >= 0 and type(nsec) is int and 0 <= nsec < 10**9
            and row["sensor_ids"] == [0]
            and row["sensor_time_double_bits"] == _bits(sec+nsec*1e-9))


def _camera_flags(values, size):
    return len(values) == size and all(type(value) in (int, float) and value in (0, 1) for value in values)


class _CameraTerminalObservation:
    """Decode native stop/export witnesses without qualifying queue or ROS delivery.

    The completed-IMU watermark is checked against raw callback identity/order.
    It is not evidence that every sample was integrated downstream. Older ledgers
    have no terminal summary, preserving their original sealed summary shape.
    """

    def __init__(self, camera_ids):
        self.camera_count = len(camera_ids)
        self.started = self.closed = False
        self.quiescent = self.boundary = self.enter = self.returned = None
        self.initializers = []
        self.last_imu = None
        self.last_imu_valid = False
        self.imu_valid = self.imu_ordered = True
        self.feed_rows = 0
        self.exports = dict(count=0, live_context_call_count=0, text_bytes_written=0,
                            unknown_text_bytes_count=0, ros_delivery="NOT_RUN")

    def observe(self, row):
        kind, values = row["kind"], row["values"]
        if kind == "camera_consumption_observation_started":
            self.started = True
        elif kind == "imu_ros_callback":
            _need(self.quiescent is None, "IMU callback after executor quiescence")
            valid = _camera_raw_stamp_valid(row) and len(values) == 6 and all(_finite(v) for v in values)
            self.imu_valid &= valid
            if valid and self.last_imu_valid:
                self.imu_ordered &= ((row["raw_sec"], row["raw_nsec"]) >
                                     (self.last_imu["raw_sec"], self.last_imu["raw_nsec"]))
            self.last_imu = row
            self.last_imu_valid = valid
        elif kind == "graceful_executor_callbacks_quiescent":
            _camera_metadata(row)
            _need(self.started and not self.closed and self.quiescent is None
                  and self.boundary is None and _camera_flags(values, 1), "executor quiescence differs or repeats")
            self.quiescent = row
        elif kind == "camera_terminal_drain_boundary":
            _camera_metadata(row)
            _need(self.started and not self.closed and self.quiescent is not None
                  and self.boundary is None and not self.initializers and _camera_flags(values, 3),
                  "terminal camera boundary lacks quiescence or repeats")
            _need(self.imu_valid and bool(values[0]) == (self.last_imu is not None)
                  and bool(values[1]) == self.imu_ordered, "terminal completed-IMU watermark contradicts callbacks")
            self.boundary = row
            return True
        elif kind in {"camera_terminal_ready_drain_enter", "camera_terminal_ready_drain_returned"}:
            _need(self.boundary is not None and not self.closed and all(self.boundary["values"])
                  and not self.initializers and _camera_raw_stamp_valid(row) and values == [],
                  "terminal drain lacks available ordered live watermark or has invalid endpoint")
            _need((row["raw_sec"], row["raw_nsec"], row["sensor_time_double_bits"]) ==
                  (self.last_imu["raw_sec"], self.last_imu["raw_nsec"], self.last_imu["sensor_time_double_bits"]),
                  "terminal drain endpoint differs from completed callback watermark")
            if kind.endswith("_enter"):
                _need(self.enter is None, "terminal drain entered more than once")
                self.enter = row
            else:
                _need(self.enter is not None and self.returned is None, "terminal drain return missing entry or repeats")
                self.returned = row
            return True
        elif kind == "initializer_worker_joined":
            self.initializers.append(row)
            if self.boundary is not None:
                _camera_metadata(row)
                _need(not self.closed and len(self.initializers) == 1 and values == []
                      and (not all(self.boundary["values"]) or self.returned is not None),
                      "initializer joined before terminal drain returned or repeats")
        elif kind == "camera_state_export_returned":
            _camera_metadata(row, state_time=True)
            _need(self.started and not self.closed and (self.boundary is None or
                  self.enter is not None and self.returned is None), "state export outside live/drain lifecycle")
            _need(len(values) == 5 and _camera_flags(values[:4], 4)
                  and _finite(values[4]) and values[4] >= -1 and int(values[4]) == values[4],
                  "state export flags or byte delta invalid")
            before, after, save, good, size = values
            _need((not good or save and size >= 0) and (save or not good and size == -1),
                  "state export save/file/byte evidence contradicts itself")
            self.exports["count"] += 1
            self.exports["live_context_call_count"] += int(bool(before and after))
            self.exports["text_bytes_written"] += max(0, int(size))
            self.exports["unknown_text_bytes_count"] += int(size == -1)
            return True
        elif kind == "camera_receipts_closed":
            if self.boundary is not None:
                _need(len(self.initializers) == 1 and
                      (not all(self.boundary["values"]) or self.returned is not None),
                      "camera receipts closed before drain/initializer joined")
            self.closed = True
        if kind in CAMERA_CALLBACK_KINDS:
            _need(self.quiescent is None, "camera callback after executor quiescence")
        elif self.boundary is not None and kind in CAMERA_PROCESSING_KINDS:
            _need(self.enter is not None and self.returned is None, "camera processing outside terminal drain")
            self.feed_rows += int(kind == "camera_feed_measurement_enter")
        return False

    def summary(self):
        if self.boundary is None:
            _need(self.quiescent is None and self.exports["count"] == 0,
                  "new camera lifecycle lacks terminal boundary")
            return {}
        attempted = bool(all(self.boundary["values"]))
        _need(len(self.initializers) == 1 and (not attempted or self.returned is not None),
              "terminal camera lifecycle is incomplete", "OPENVINS_CAMERA_OBSERVATION_INCOMPLETE")
        return dict(terminal_drain=dict(watermark_available=bool(self.boundary["values"][0]),
            watermark_ordered=bool(self.boundary["values"][1]), context_live=bool(self.boundary["values"][2]),
            attempted=attempted, completed=self.returned is not None,
            endpoint_sensor_ns=None if self.enter is None else self.enter["raw_sec"]*10**9+self.enter["raw_nsec"],
            feed_bundle_count=self.feed_rows//self.camera_count,
            downstream_imu_integration="UNOBSERVED", all_pending_consumed="NOT_RUN"), state_export=self.exports)


def analyze_camera_consumption(path, *, camera_ids, downsample_cameras, reference=None):
    """Audit occurrence lineage, exact pixel boundaries and joined completion.

    PASS is scoped to observation integrity. Consumption is independently FAIL
    when any source input is missing/rejected/unfed; no health is assigned. The
    original integer stamps remain distinct even when their double bits collide.
    """
    _need(type(camera_ids) is list and all(type(value) is int for value in camera_ids)
          and camera_ids in ([0], [0, 1]), "only declared mono/stereo camera configurations are qualified")
    _need(type(downsample_cameras) is bool, "explicit downsample configuration required")
    if reference is not None:
        _need(type(reference.get("schema_version")) is int and reference["schema_version"] == 1
            and reference.get("complete") is True
            and reference.get("camera_ids") == camera_ids
            and reference.get("downsample_cameras") is downsample_cameras,
            "complete matching camera reference required")
    occurrences, counts, callbacks = {}, Counter(), defaultdict(deque)
    groups = defaultdict(list)
    started = closed = joined = False
    marker_sequences = {}
    terminal = _CameraTerminalObservation(camera_ids)
    for row in _events(path):
        kind = row["kind"]
        counts[kind] += 1
        _need("camera_consumption" not in row or kind in PAYLOAD_KINDS,
              "camera payload attached to an unsupported event")
        if terminal.observe(row):
            continue
        if kind == "camera_consumption_observation_started":
            _need(not started and not closed and row["values"] == [1, 10000], "camera start marker differs or repeats")
            started = True
            marker_sequences["started"] = row["sequence"]
        elif kind == "camera_receipts_closed":
            _need(started and not closed and row["values"] == [len(occurrences)], "camera receipt footer/count differs")
            closed = True
            marker_sequences["closed"] = row["sequence"]
        elif kind == "camera_observation_workers_joined":
            _need(closed and not joined and row["values"] == [0], "camera worker not joined or marker repeats")
            joined = True
            marker_sequences["joined"] = row["sequence"]
        elif "camera_consumption" in row or kind in PAYLOAD_KINDS:
            _need(started and not closed, "camera evidence outside open receipt lifecycle")
            payload = _payload(row)
            occurrence, camera = payload["occurrence"], payload["camera_id"]
            _need(camera in camera_ids, "undeclared native camera ID")
            _need(kind != "camera_receipt_capacity_or_closed_rejected", "native camera receipt capacity was exhausted",
                  "OPENVINS_CAMERA_OBSERVATION_INCOMPLETE")
            if kind == "camera_subscription_received":
                _need(occurrence == len(occurrences)+1, "camera occurrence gap, duplicate or order change")
                _need(row["sensor_time_double_bits"] == _bits(row["raw_sec"]+row["raw_nsec"]*1e-9),
                      "subscription raw stamp and processing double differ")
                occurrences[occurrence] = dict(raw=_raw_identity(payload), events=[], callback_seen=False)
            _need(occurrence in occurrences and _raw_identity(payload) == occurrences[occurrence]["raw"],
                  "camera provenance differs across pipeline boundaries")
            item = occurrences[occurrence]
            history = [event["kind"] for event in item["events"]]
            _need(kind not in history and not (history and history[-1] in TERMINALS),
                  "duplicated camera boundary or event after terminal result")
            if kind in PIPELINE:
                _need(history == PIPELINE[:PIPELINE.index(kind)], "camera pipeline order or boundary is missing")
            elif kind in {"camera_track_frequency_rejected", "camera_cv_bridge_rejected", "camera_cv_bridge_bundle_rejected"}:
                _need(history == PIPELINE[:2] and item["callback_seen"], "camera rejection before matched callback")
            elif kind in {"camera_subscription_not_forwarded_before_shutdown", "camera_receipt_pointer_reused_without_forward"}:
                _need(history == PIPELINE[:1], "unforwarded receipt was already matched")
            elif kind == "camera_queued_not_fed_before_shutdown":
                _need(history == PIPELINE[:3], "shutdown queue evidence contradicts feed processing")
            if kind == "camera_matched_callback_enter":
                callbacks[(camera, payload["raw_sensor_timestamp_ns"])].append(occurrence)
                _need(row["sensor_time_double_bits"] == _bits(row["raw_sec"]+row["raw_nsec"]*1e-9),
                      "matched callback raw stamp differs")
            if kind == "camera_decoded_queue_insert":
                _need(item["callback_seen"], "decoded camera lacks matched ROS callback")
            item["events"].append(dict(kind=kind, sequence=row["sequence"], monotonic_ns=row["monotonic_ns"],
                sensor_time_double_bits=row["sensor_time_double_bits"], decoded=payload.get("decoded")))
            if kind in PIXEL_STAGES:
                groups[kind].append(occurrence)
        elif kind == "camera_ros_callback":
            sec, nsec = row.get("raw_sec"), row.get("raw_nsec")
            _need(type(sec) is int and type(nsec) is int and 0 <= nsec < 10**9
                  and len(row["sensor_ids"]) == 1, "camera callback integer timestamp/ID invalid")
            key = row["sensor_ids"][0], sec*10**9+nsec
            _need(callbacks[key], "unmatched or duplicate native ROS camera callback")
            item = occurrences[callbacks[key].popleft()]
            raw = item["raw"]
            _need(row["values"] == [raw["raw_height"], raw["raw_width"], raw["raw_step"]]
                  and row["sensor_time_double_bits"] == _bits(sec+nsec*1e-9), "native ROS callback shape/time differs")
            item["callback_seen"] = True
        elif kind.startswith("camera_"):
            _need(kind in {"camera_processing_begin", "camera_processing_returned", "camera_returned_state"},
                  "unsupported native camera boundary")
    _need(started and closed and joined and not any(callbacks.values()), "camera lifecycle is incomplete",
          "OPENVINS_CAMERA_OBSERVATION_INCOMPLETE")
    for occurrence, item in occurrences.items():
        _need(item["events"][-1]["kind"] in TERMINALS, "camera occurrence has no terminal result",
              "OPENVINS_CAMERA_OBSERVATION_INCOMPLETE")
        decoded = [event for event in item["events"] if event["decoded"] is not None]
        if decoded:
            original = decoded[0]["decoded"]
            for event in decoded:
                if event["kind"] in {"camera_frontend_feed_enter", "camera_frontend_feed_returned"} and downsample_cameras:
                    continue
                _need(event["decoded"] == original, "decoded original pixels/storage changed before or after frontend")
            front = [event["decoded"] for event in decoded if event["kind"] in
                     {"camera_frontend_feed_enter", "camera_frontend_feed_returned"}]
            _need(not front or len(front) == 2 and front[0] == front[1], "frontend changed source image or did not return")
    # CameraData emits one row per camera, in its declared sensor_ids order.
    # All stage groups must retain the exact occurrences and cam0 processing time.
    memberships = {}
    for kind, ordered in groups.items():
        _need(len(ordered) % len(camera_ids) == 0, "partial stereo observation bundle")
        memberships[kind] = []
        for offset in range(0, len(ordered), len(camera_ids)):
            bundle = ordered[offset:offset+len(camera_ids)]
            items = [occurrences[value] for value in bundle]
            _need([item["raw"]["camera_id"] for item in items] == camera_ids, "camera bundle IDs/order differ")
            ns = items[0]["raw"]["raw_sensor_timestamp_ns"]
            bits = _bits(ns//10**9+(ns%10**9)*1e-9)
            _need(all(next(event for event in item["events"] if event["kind"] == kind)["sensor_time_double_bits"] == bits
                      for item in items), "bundle processing timestamp changed or cam1 raw time was substituted")
            memberships[kind].append(bundle)
    feeds = memberships.get("camera_feed_measurement_enter", [])
    for kind in ("camera_manager_enter", "camera_frontend_feed_enter", "camera_frontend_feed_returned", "camera_feed_measurement_returned"):
        _need(memberships.get(kind, []) == feeds, "camera feed/manager/frontend bundle lineage differs")
    received = defaultdict(list)
    for occurrence, item in occurrences.items():
        received[item["raw"]["camera_id"]].append((occurrence, item))
    coverage = dict(status="NOT_RUN", reason="independent source/CDR/observer reference not supplied")
    if reference is not None:
        per_camera = {}
        for camera in camera_ids:
            expected = reference["records"][camera]
            lookup = defaultdict(list)
            for index, record in enumerate(expected):
                identity = tuple(sorted((key,value) for key,value in record["raw"].items() if key != "decoded"))
                lookup[identity].append(index)
            cursor, indices = 0, []
            for occurrence, item in received[camera]:
                candidates = lookup[tuple(sorted(item["raw"].items()))]
                index = bisect_left(candidates, cursor)
                _need(index < len(candidates), "native subscription is unexpected, reordered or has different raw bytes",
                      "OPENVINS_NATIVE_CAMERA_INPUT_MISMATCH")
                source_index = candidates[index]
                cursor = source_index+1
                indices.append(source_index)
                item["source_ordinal"] = expected[source_index]["source_ordinal"]
                for event in item["events"]:
                    if event["decoded"] is None:
                        continue
                    front = event["kind"] in {"camera_frontend_feed_enter", "camera_frontend_feed_returned"}
                    want = expected[source_index]["frontend"] if front else expected[source_index]["raw"]["decoded"]
                    _need(all(event["decoded"][key] == value for key, value in want.items()),
                          "actual decoded pixels differ from independent source/conversion",
                          "OPENVINS_NATIVE_CAMERA_INPUT_MISMATCH")
            missing = sorted(set(range(len(expected)))-set(indices))
            # A missing member among byte-identical duplicate inputs has no
            # independently observable original occurrence ID. Preserve that
            # ambiguity instead of reporting the greedy ordinal as measured.
            ambiguous = [values for values in lookup.values()
                         if len(values) > 1 and any(value in missing for value in values)]
            for _, item in received[camera]:
                matching = [values for values in ambiguous if item["source_ordinal"] in values]
                if matching:
                    item["source_ordinal"] = None
                    item["source_ordinal_candidates"] = [expected[index]["source_ordinal"] for values in matching for index in values]
            per_camera[camera] = dict(expected_count=len(expected), subscription_count=len(received[camera]),
                missing_source_ordinals=missing, source_order_exact=not missing,
                missing_identical_occurrence_identity="AMBIGUOUS" if ambiguous else "UNIQUE",
                pixel_identity="PASS_FOR_OBSERVED_BOUNDARIES", status="FAIL" if missing else "PASS")
        coverage = dict(status="PASS" if all(row["status"] == "PASS" for row in per_camera.values()) else "FAIL",
            per_camera=per_camera, scope="SOURCE_ZIP_CDR_OBSERVER_TO_NATIVE_SUBSCRIPTION_AND_PIXEL_BOUNDARIES")
    terminal_counts = Counter(item["events"][-1]["kind"] for item in occurrences.values())
    all_fed = bool(occurrences) and terminal_counts["camera_feed_measurement_returned"] == len(occurrences)
    return dict(schema_version=1, status="PASS", observation_integrity="PASS", observation_complete=True,
        scope="NATIVE_CAMERA_OBSERVATION_INTEGRITY_ONLY",
        camera_ids=camera_ids, downsample_cameras=downsample_cameras, event_counts=dict(counts),
        occurrence_count=len(occurrences), terminal_counts=dict(terminal_counts), marker_sequences=marker_sequences,
        per_stage_bundle_count={kind:len(value) for kind,value in memberships.items()},
        input_coverage=coverage, full_source_consumption="PASS" if all_fed and coverage["status"] == "PASS" else
            ("NOT_RUN" if coverage["status"] == "NOT_RUN" else "FAIL"),
        occurrences=[dict(occurrence=key, **value) for key,value in occurrences.items()],
        frontend_completion_scope="trackFEATS->feed_new_camera returned; not accepted EKF constraints or health",
        shutdown_scope="executor returned; owned update worker joined; receipt/queue terminals precede shared footer",
        health_qualification="NOT_RUN", automatic_reset="NOT_RUN", truth_used=False,
        native_dataset_qualification="NOT_RUN_UNTIL_FRESH_REPLAY_AND_INDEPENDENT_SOURCE_AUDIT", **terminal.summary())


if __name__ == '__main__':
    import argparse
    import sys
    parser = argparse.ArgumentParser(description='Validate native reader matches before paused playback.')
    parser.add_argument('--input-publisher-matches', type=Path, required=True)
    parser.add_argument('--mode', choices=('mono', 'stereo'), required=True)
    parser.add_argument('--native-pid', type=int, required=True)
    args = parser.parse_args()
    try:
        read_input_publisher_matches(args.input_publisher_matches, mode=args.mode, native_pid=args.native_pid)
    except (ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
