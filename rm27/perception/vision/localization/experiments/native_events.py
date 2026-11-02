"""Bounded Schur native observations, independent of truth and output health.

Integer camera timestamps associate events exactly. Only paired timestamps from
one native steady clock measure durations. A frontend tracking stage, a Schur
Backward return, and a nonzero feature count are not accepted visual corrections.
The explicit ``schur_visual_correction`` event is required for that observation.
No event in this module alone qualifies a state for flight-controller fusion.
"""

from collections import Counter, defaultdict, deque
import json
import math
from pathlib import Path
import struct

import numpy as np

from .common import ExperimentError, file_hash, require

NATIVE_EVENT_LIMIT = 200_000
MAX_EVENT_LINE_BYTES = 32_768
IMU_LEDGER_BYTE_LIMIT = 128 * 1024**2
PAIRED_PHASES = ("frontend", "schur_forward", "schur_backward")
INPUT_FAULT_REASONS = {"input_rejected_camera_timestamp": 1, "input_rejected_camera_order": 2,
    "input_invalid_imu_nonfinite": 3, "input_invalid_imu_order": 4,
    "input_invalid_imu_gap": 5, "input_invalid_imu_coverage": 6,
    "input_rejected_camera_native_resolution": 7}
INPUT_BOUNDARIES = set(INPUT_FAULT_REASONS) | {"input_invalidated", "input_waiting_imu_coverage",
                                                        "input_rejected_stereo_time"}


def _integer(value, name, minimum=0):
    require(type(value) is int and value >= minimum, "NATIVE_EVENT_INVALID", name + " must be an integer")
    return value


def _count(value):
    return _finite(value) and value >= 0 and value <= 2**53 and int(value) == value


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _stats(values, *, unit="ns"):
    if not values:
        return dict(status="NOT_RUN", count=0, unit=unit, p50=None, p95=None, max=None)
    return dict(status="PASS", count=len(values), unit=unit, p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), max=max(values))


def _frequency(times, clock):
    if len(times) < 2:
        return dict(status="NOT_RUN", sample_count=len(times), clock=clock, hz=None)
    deltas = [end-start for start, end in zip(times, times[1:])]
    if any(dt <= 0 for dt in deltas):
        return dict(status="FAIL", sample_count=len(times), clock=clock, hz=None,
                    reason="timestamps are not strictly increasing")
    return dict(status="PASS", sample_count=len(times), clock=clock,
                hz=(len(times)-1)*1_000_000_000/(times[-1]-times[0]), interval_ns=_stats(deltas))


def analyze_native_imu_ledger(path, observer_rows):
    """Audit the separate callback ledger against reconciled observer occurrences.

    This records callback receipt and the actual ImuHandler insertion return,
    never integration or filter reset ownership. Sensor timestamps may repeat
    or move backwards in declared fault inputs; matching preserves delivery
    order, raw ROS header sequence and every occurrence, without sorting.
    Native ``toSec`` bits are retained and checked against their serialized
    double, rather than recomputed by dividing the integer nanoseconds.
    """
    path = Path(path)
    unavailable = dict(status='NOT_RUN', reason='native IMU callback ledger absent',
        callback_coverage=dict(status='NOT_RUN'), buffer_insertion=dict(status='NOT_RUN'),
        downstream_integration='UNOBSERVED', reset_mapping='NOT_RUN', truth_used=False)
    if not path.is_file() or path.stat().st_size == 0:
        return unavailable

    def valid(condition, reason):
        require(condition, 'NATIVE_IMU_LEDGER_INVALID', reason)

    def uint(value, name, maximum=2**64-1):
        valid(type(value) is int and 0 <= value <= maximum, name+' must be an unsigned integer')
        return value

    def bits(value):
        valid(_finite(value), 'finite raw IMU doubles required')
        return struct.unpack('=Q', struct.pack('=d', value))[0]

    def payload(row):
        gyro, accel = row.get('gyro'), row.get('accel')
        valid(isinstance(gyro, list) and len(gyro) == 3
              and isinstance(accel, list) and len(accel) == 3, 'observer needs two three-axis vectors')
        return tuple(bits(value) for value in gyro+accel)

    def ranges(indices):
        groups = []
        for index in indices:
            if groups and index == groups[-1]['last_observer_index']+1:
                groups[-1]['last_observer_index'] = index
                groups[-1]['count'] += 1
            else:
                groups.append(dict(first_observer_index=index, last_observer_index=index, count=1))
        return groups

    expected, occurrences = [], defaultdict(deque)
    for row in observer_rows:
        valid(isinstance(row, dict), 'observer IMU object required')
        ns = uint(row.get('state_ns'), 'observer sensor time', (2**32)*10**9-1)
        sequence = uint(row.get('native_sequence'), 'observer ROS header sequence', 2**32-1)
        frame = row.get('frame_id')
        valid(isinstance(frame, str), 'observer raw frame identity required')
        key = (sequence, ns, frame.encode('utf-8').hex(), payload(row))
        occurrences[key].append(len(expected))
        expected.append(key)
        valid(len(expected) <= NATIVE_EVENT_LIMIT, 'observer occurrence limit exceeded')
    valid(expected, 'nonempty independently reconciled observer IMU required')
    valid(path.stat().st_size <= IMU_LEDGER_BYTE_LIMIT, 'native IMU byte limit exceeded')

    def reject_constant(value):
        raise ExperimentError('NATIVE_IMU_LEDGER_INVALID', 'nonfinite JSON constant '+value)

    dispositions = {'health_rejected', 'paused', 'no_handler', 'buffer_accepted', 'buffer_rejected'}
    counts, matched, durations = Counter(), [], []
    previous_completion, first_receipt, footer, callback_count = -1, None, None, 0
    time_bits = set()
    with path.open('rb') as stream:
        while line := stream.readline(8193):
            valid(len(line) <= 8192 and line.endswith(b'\n'), 'bounded complete JSONL line required')
            valid(footer is None, 'event after closure footer')
            try:
                row = json.loads(line, parse_constant=reject_constant)
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                raise ExperimentError('NATIVE_IMU_LEDGER_INVALID', 'malformed JSONL') from exc
            valid(isinstance(row, dict) and type(row.get('schema_version')) is int
                  and row['schema_version'] == 1, 'ledger schema version 1 required')
            if row.get('kind') == 'schur_imu_ledger_closed':
                footer = row
                continue
            valid(row.get('kind') == 'schur_imu_callback', 'unknown callback ledger event')
            valid(uint(row.get('receipt_sequence'), 'callback ordinal') == callback_count,
                  'callback ordinal gap, duplicate or reorder')
            callback_count += 1
            valid(callback_count <= NATIVE_EVENT_LIMIT, 'native callback record limit exceeded')
            sec = uint(row.get('raw_sec'), 'raw seconds', 2**32-1)
            nsec = uint(row.get('raw_nsec'), 'raw nanoseconds', 10**9-1)
            ns = uint(row.get('sensor_ns'), 'sensor time')
            sequence = uint(row.get('header_sequence'), 'ROS header sequence', 2**32-1)
            valid(ns == sec*10**9+nsec and row.get('sensor_id') == 'imu0', 'raw timestamp or stream mismatch')
            frame_hex = row.get('frame_id_hex')
            size = uint(row.get('frame_id_size'), 'frame ID byte count')
            valid(isinstance(frame_hex, str) and size <= 128 and len(frame_hex) == size*2
                  and all(c in '0123456789abcdef' for c in frame_hex)
                  and row.get('frame_id_truncated') is False, 'incomplete raw frame identity')
            native_bits = uint(row.get('native_seconds_bits'), 'native time bits')
            valid(native_bits == bits(row.get('native_seconds')), 'serialized native toSec bits mismatch')
            stamp_seconds = float(sec)+float(nsec)*1e-9
            valid(abs(row['native_seconds']-stamp_seconds) <= math.ulp(stamp_seconds),
                  'native toSec is inconsistent with its raw ROS stamp')
            # Retain actual native bits, including valid rounding differences
            # between sec+nsec*1e-9 and integer_ns/1e9.
            time_bits.add(native_bits)
            values, encoded = row.get('values'), row.get('value_bits')
            valid(isinstance(values, list) and len(values) == 6
                  and isinstance(encoded, list) and len(encoded) == 6, 'six raw axes and bits required')
            value_bits = tuple(uint(value, 'axis bits') for value in encoded)
            valid(value_bits == tuple(bits(value) for value in values), 'serialized axis bits mismatch')
            receipt = uint(row.get('receipt_monotonic_ns'), 'receipt monotonic time')
            completion = uint(row.get('completion_monotonic_ns'), 'completion monotonic time')
            valid(previous_completion <= receipt <= completion, 'native single-thread callback clock/order mismatch')
            if first_receipt is None:
                first_receipt = receipt
            previous_completion = completion
            disposition = row.get('disposition')
            health = row.get('health_code')
            valid(disposition in dispositions and type(health) is int
                  and ((disposition == 'health_rejected' and health in (3, 4, 5))
                       or (disposition != 'health_rejected' and health == 0)),
                  'unfinished callback or invalid disposition/health evidence')
            counts[disposition] += 1
            durations.append(completion-receipt)
            key = (sequence, ns, frame_hex, value_bits)
            valid(occurrences[key], 'native header/payload has no matching observer occurrence or is duplicated')
            index = occurrences[key].popleft()
            valid(not matched or index > matched[-1], 'native callbacks reordered relative to observer delivery')
            matched.append(index)

    require(footer is not None, 'NATIVE_IMU_LEDGER_INCOMPLETE', 'normal-close footer absent')
    for name in ('attempted_callbacks', 'retained_records', 'written_records', 'dropped_records',
                 'output_dropped_records', 'record_capacity', 'byte_limit', 'monotonic_ns', 'startup_monotonic_ns'):
        uint(footer.get(name), 'footer '+name)
    valid(0 < footer['record_capacity'] <= NATIVE_EVENT_LIMIT
          and footer['byte_limit'] == IMU_LEDGER_BYTE_LIMIT
          and callback_count <= footer['record_capacity']
          and footer['monotonic_ns'] >= previous_completion, 'footer capacity/clock mismatch')
    valid(footer.get('close_boundary') == 'svo_interface_callback_threads_joined'
          and 0 < footer['startup_monotonic_ns'] <= (first_receipt if first_receipt is not None else footer['monotonic_ns']),
          'explicit startup and joined-thread closure boundary required')
    require(footer['attempted_callbacks'] == footer['retained_records']
            == footer['written_records'] == callback_count
            and footer['dropped_records'] == footer['output_dropped_records'] == 0
            and footer.get('allocation_failed') is False and footer.get('io_failed') is False,
            'NATIVE_IMU_LEDGER_INCOMPLETE', 'native callback records dropped or serialization failed')
    observed = set(matched)
    missing = [index for index in range(len(expected)) if index not in observed]
    complete = not missing
    status = 'PASS' if complete else 'FAIL'
    return dict(schema_version=1, status=status, scope='NATIVE_IMU_CALLBACK_AND_BUFFER_INSERTION_ONLY',
        source=str(path.resolve()), source_sha256=file_hash(path),
        callback_coverage=dict(status=status, observer_count=len(expected), callback_count=callback_count,
            matched_count=len(matched), missing_count=len(missing), missing_observer_ranges=ranges(missing),
            association='EXACT_ORDERED_OCCURRENCES_HEADER_SEQUENCE_RAW_NS_FRAME_ID_AND_SIX_AXIS_BITS',
            observer_completeness='requires independent source-to-observer reconciliation'),
        buffer_insertion=dict(status='OBSERVED', disposition_counts=dict(counts),
            accepted_count=counts['buffer_accepted'], rejected_count=counts['buffer_rejected'],
            meaning='actual addImuMeasurement return only; accepted is not proof of later integration'),
        callback_duration=_stats(durations), native_toSec_distinct_bit_count=len(time_bits),
        lifecycle=dict(startup_monotonic_ns=footer['startup_monotonic_ns'],
            close_monotonic_ns=footer['monotonic_ns'], close_boundary=footer['close_boundary']),
        native_time_semantics='actual native toSec double bits; raw integer stamps retained independently',
        downstream_integration='UNOBSERVED', reset_mapping='NOT_RUN', health_qualification='NOT_RUN',
        truth_used=False)


def _camera_times(ledger, camera_id):
    require(isinstance(ledger, (list, tuple)) and ledger, "NATIVE_LEDGER_INVALID", "nonempty frozen camera ledger required")
    if all(type(row) is int for row in ledger):
        times = list(ledger)
    else:
        require(all(isinstance(row, dict) for row in ledger), "NATIVE_LEDGER_INVALID", "integer times or source ledger rows")
        times = [row["state_ns"] for row in ledger if row.get("stream") == camera_id]
    require(times, "NATIVE_LEDGER_INVALID", "selected camera stream is empty")
    for i, time in enumerate(times):
        _integer(time, "camera time")
        require(i == 0 or time > times[i-1], "NATIVE_LEDGER_INVALID", "camera times must strictly increase; no sorting")
    return times


def _read_events(path, max_events):
    def bad_constant(value):
        raise ExperimentError("NATIVE_EVENT_INVALID", "non-finite JSON constant " + value)

    previous = None
    with path.open("rb") as source:
        count = 0
        while True:
            line = source.readline(MAX_EVENT_LINE_BYTES + 1)
            if not line:
                break
            require(len(line) <= MAX_EVENT_LINE_BYTES, "NATIVE_EVENT_LIMIT", "native event line exceeds bounded schema")
            if not line.strip():
                continue
            count += 1
            require(count <= max_events, "NATIVE_EVENT_LIMIT", "event count exceeds configured source limit")
            try:
                event = json.loads(line, parse_constant=bad_constant)
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                raise ExperimentError("NATIVE_EVENT_INVALID", f"malformed event {count}") from exc
            require(isinstance(event, dict) and type(event.get("schema_version")) is int
                    and event["schema_version"] == 1, "NATIVE_EVENT_INVALID", "schema_version must be integer 1")
            require(isinstance(event.get("kind"), str) and event["kind"], "NATIVE_EVENT_INVALID", "event kind required")
            for name in ("sensor_ns", "bundle_id"):
                _integer(event.get(name), name, -1)
            _integer(event.get("monotonic_ns"), "native monotonic time")
            require(previous is None or event["monotonic_ns"] >= previous,
                    "NATIVE_CLOCK_REWIND", "native clock/session changed; do not concatenate runs")
            previous = event["monotonic_ns"]
            values = event.get("values")
            require(isinstance(values, list) and len(values) <= 64, "NATIVE_EVENT_INVALID", "bounded numerical values list required")
            require(all(x is None or _finite(x) for x in values),
                    "NATIVE_EVENT_INVALID", "native null denotes missing/non-finite evidence; booleans/NaN are invalid")
            yield count, event


def analyze_native_events(path, camera_ledger, *, camera_id="cam0", first_pose_sensor_ns=None,
                          max_events=NATIVE_EVENT_LIMIT):
    """Return strict-JSON observations, never a LocalizationEstimate or score.

    ``camera_ledger`` is an increasing integer ns list or existing expected.jsonl
    rows. ``first_pose_sensor_ns`` is optional, explicitly labelled proxy data.
    A missing trace returns NOT_RUN; malformed/unassociated traces raise
    ExperimentError. At the logger cap, results are a prefix and cannot establish
    complete initialization failure/reset counts or continuous tracking health.
    """
    require(type(max_events) is int and 0 < max_events <= NATIVE_EVENT_LIMIT,
            "NATIVE_EVENT_LIMIT", "max_events must be within the native 200000-event limit")
    cameras = _camera_times(camera_ledger, camera_id)
    camera_index = {time: i for i, time in enumerate(cameras)}
    path = Path(path)
    absent = dict(status="NOT_RUN", reason="native event file missing or empty", event_count=0,
                  initialization=dict(status="NOT_RUN"), visual_corrections=dict(status="NOT_RUN", count=None),
                  timings={phase: _stats([]) for phase in PAIRED_PHASES}, health_qualification="NOT_RUN")
    if not path.is_file() or path.stat().st_size == 0:
        return absent
    proxy = dict(status="NOT_RUN", sensor_ns=None, source_elapsed_s=None,
                 meaning="first pose is a publication proxy, never metric initialization")
    if first_pose_sensor_ns is not None:
        _integer(first_pose_sensor_ns, "first pose sensor time")
        require(first_pose_sensor_ns in camera_index, "NATIVE_CAMERA_UNMATCHED", "first pose must exactly match camera ledger")
        proxy.update(status="PASS", sensor_ns=first_pose_sensor_ns,
                     source_elapsed_s=(first_pose_sensor_ns-cameras[0])/1e9)

    event_counts, problems = Counter(), []
    pending, spans, bundle_time = {}, {p: [] for p in PAIRED_PHASES}, {}
    processed, started, reprojects, imu_intervals, raw_imu_intervals = [], [], [], [], []
    corrections, rejected_corrections, init_events, commits, geometry, reset_events = [], [], [], [], [], []
    alignment_attempts, frontend_failures, restarts = [], [], []
    stereo_geometry, stereo_commits, rejected_stereo = [], [], []
    quality_decisions, input_health = [], []
    segment, event_count, previous_epoch = 0, 0, None
    first_event_monotonic = last_event_monotonic = None
    native_epoch_transitions = []

    def record(event, ordinal):
        return dict(event_ordinal=ordinal, observation_segment=segment, sensor_ns=event["sensor_ns"],
                    camera_index=camera_index.get(event["sensor_ns"]), bundle_id=event["bundle_id"],
                    monotonic_ns=event["monotonic_ns"])

    for ordinal, event in _read_events(path, max_events):
        event_count = ordinal
        event_counts[event["kind"]] += 1
        kind, sensor_ns, bundle = event["kind"], event["sensor_ns"], event["bundle_id"]
        if first_event_monotonic is None:
            first_event_monotonic = event["monotonic_ns"]
        last_event_monotonic = event["monotonic_ns"]
        if sensor_ns >= 0:
            require(sensor_ns in camera_index, "NATIVE_CAMERA_UNMATCHED",
                    f"event {ordinal} sensor_ns={sensor_ns} is absent from exact source ledger")
            key = (segment, bundle)
            require(bundle >= 0 or kind in INPUT_BOUNDARIES,
                    "NATIVE_EVENT_INVALID", "only explicit input boundaries may precede a native bundle")
            if bundle >= 0:
                require(key not in bundle_time or bundle_time[key] == sensor_ns,
                        "NATIVE_BUNDLE_TIME_CONFLICT", f"native bundle {bundle} changed exact source time")
                bundle_time[key] = sensor_ns
        else:
            require(kind == "reset" and bundle == -1, "NATIVE_EVENT_INVALID", "only no-frame reset may omit source association")
        values, info = event["values"], record(event, ordinal)
        if kind == "reset":
            reset_events.append(dict(info, startup=not started))
            for key, begin in list(pending.items()):
                problems.append(dict(reason="RESET_INTERRUPTED_SPAN", phase=key[0], begin=begin))
            pending.clear()
            segment += 1
            continue
        for phase in PAIRED_PHASES:
            key = (phase, segment, bundle)
            if kind == phase + "_begin":
                require(key not in pending, "NATIVE_SPAN_AMBIGUOUS", f"repeated begin for {phase} bundle {bundle}")
                pending[key] = info
            elif kind == phase + "_end":
                begin = pending.pop(key, None)
                if begin is None:
                    problems.append(dict(reason="SPAN_BEGIN_MISSING", phase=phase, end=info))
                else:
                    require(begin["sensor_ns"] == sensor_ns, "NATIVE_BUNDLE_TIME_CONFLICT", "paired span source mismatch")
                    spans[phase].append(dict(info, begin_monotonic_ns=begin["monotonic_ns"],
                                              duration_ns=event["monotonic_ns"]-begin["monotonic_ns"]))
        if kind == "frontend_begin":
            started.append(info)
        elif kind == "frontend_end":
            processed.append(info)
            if len(values) >= 3 and values[1] == 2:
                frontend_failures.append(dict(info, raw_stage=values[0], raw_update_result=values[1], raw_quality=values[2]))
        elif kind == "mono_geometric_commit":
            geometry.append(info)
        elif kind == "metric_filter_commit":
            commits.append(dict(info, raw_values=values, initialization_method="MONOCULAR_VISUAL_IMU_ALIGNMENT"))
        elif kind == "stereo_metric_geometry_ready":
            if (len(values) != 6 or not all(_finite(v) for v in values)
                    or not all(_count(values[i]) for i in (0, 1, 3, 4))
                    or values[0] == values[1] or values[2] <= 0
                    or values[3] <= 0 or values[4] <= 0 or values[5] != 1):
                rejected_stereo.append(dict(info, reason="STEREO_METRIC_GEOMETRY_INVALID", raw_values=values))
            else:
                stereo_geometry.append(dict(info, camera_ids=[int(values[0]), int(values[1])],
                    baseline_m=values[2], left_landmarks=int(values[3]), right_landmarks=int(values[4]),
                    imu_ready=True, scale_source="PINNED_CALIBRATED_STEREO_BASELINE", raw_values=values))
        elif kind == "stereo_filter_state_commit":
            if (len(values) != 40 or not all(_finite(v) for v in values)
                    or not all(_count(values[i]) for i in (1, 2, 3, 4, 5, 6, 7))
                    or abs(values[0]-sensor_ns/1e9) > 1e-6 or values[3] != 1
                    or values[4] != 21 or values[5] != 21 or values[6] <= 0 or values[7] <= 0
                    or values[8] != 1 or abs(math.hypot(*values[9:13])-1) > 1e-6
                    or abs(math.hypot(*values[28:32])-1) > 1e-6
                    or math.hypot(*values[25:28]) == 0 or values[38] <= 0 or values[39] != 1):
                rejected_stereo.append(dict(info, reason="STEREO_FILTER_STATE_COMMIT_INVALID", raw_values=values))
            else:
                commit = dict(info, raw_values=values, native_epoch=int(values[1]), native_state_id=int(values[2]),
                    clone_count=int(values[3]), covariance_shape=[int(values[4]), int(values[5])],
                    registered_feature_count=int(values[6]), local_point_count=int(values[7]),
                    orientation_wxyz=values[9:13], position=values[13:16], velocity_parent_m_s=values[16:19],
                    accel_bias_m_s2=values[19:22], gyro_bias_rad_s=values[22:25], gravity_parent_m_s2=values[25:28],
                    fej_orientation_wxyz=values[28:32], fej_position=values[32:35], fej_velocity_parent_m_s=values[35:38],
                    baseline_m=values[38], covariance_status="NATIVE_FINITE_STORAGE_NOT_QUALIFIED_OUTPUT",
                    initialization_method="CALIBRATED_STEREO_BASELINE_WITH_UPSTREAM_IMU_PRIORS",
                    upstream_initialization_assumptions=["FIRST_IMU_ACCELERATION_GRAVITY_DIRECTION", "ZERO_INITIAL_VELOCITY",
                                                         "ZERO_INITIAL_ACCELEROMETER_BIAS", "ZERO_INITIAL_GYROSCOPE_BIAS"])
                stereo_commits.append(commit)
                commits.append(commit)
        elif kind == "stereo_metric_initialized":
            geometry_rows = [row for row in stereo_geometry if row["observation_segment"] == segment
                             and row["bundle_id"] == bundle]
            commit_rows = [row for row in stereo_commits if row["observation_segment"] == segment
                           and row["bundle_id"] == bundle]
            shape_valid = (len(values) == 7 and all(_finite(v) for v in values)
                and all(_count(values[i]) for i in (0, 1, 3, 4)) and values[0] != values[1]
                and values[2] > 0 and values[3] > 0 and values[4] > 0
                and values[5] == 2 and values[6] in (0, 2))
            linked = (shape_valid and len(geometry_rows) == 1 and len(commit_rows) == 1
                and geometry_rows[0]["event_ordinal"] < commit_rows[0]["event_ordinal"]
                and geometry_rows[0]["camera_ids"] == [int(values[0]), int(values[1])]
                and all(math.isclose(row["baseline_m"], values[2], rel_tol=1e-9, abs_tol=1e-12)
                        for row in (geometry_rows[0], commit_rows[0])))
            if not linked:
                rejected_stereo.append(dict(info, reason="STEREO_INITIALIZATION_WITHOUT_MATCHED_GEOMETRY_AND_FILTER_COMMIT",
                                            raw_values=values))
            else:
                commit = commit_rows[0]
                init_events.append(dict(info, raw_values=values, same_bundle_filter_commit_observed=True,
                    same_bundle_stereo_geometry_observed=True, native_epoch=commit["native_epoch"],
                    camera_ids=geometry_rows[0]["camera_ids"], baseline_m=values[2],
                    initialization_method=commit["initialization_method"],
                    scale_source="PINNED_CALIBRATED_STEREO_BASELINE",
                    upstream_initialization_assumptions=commit["upstream_initialization_assumptions"],
                    source_elapsed_s=(sensor_ns-cameras[0])/1e9,
                    visual_correction_observed=False, native_stage=int(values[5]),
                    native_tracking_quality=int(values[6])))
        elif kind in INPUT_BOUNDARIES:
            valid = (kind not in INPUT_FAULT_REASONS and not values) or (
                kind in INPUT_FAULT_REASONS and len(values) in (1, 2)
                and values[0] == INPUT_FAULT_REASONS[kind]
                and (len(values) == 1 or values[1] in (0, 1)))
            source_time, stream = event.get("source_sensor_ns"), event.get("source_stream")
            if source_time is not None or stream is not None:
                valid = valid and type(source_time) is int and source_time >= 0 and stream in ("imu0", "cam1")
            if not valid:
                problems.append(dict(info, reason="NATIVE_INPUT_HEALTH_EVENT_INVALID", kind=kind, raw_values=values))
            else:
                input_health.append(dict(info, kind=kind, raw_values=values,
                    source_sensor_ns=source_time, source_stream=stream,
                    automatic_reinitialization=bool(values[1]) if len(values) == 2 else None,
                    observer_notification_timestamp=None,
                    meaning="native input boundary; not a visual correction or observer clock mapping"))
        elif kind == "quality_decision":
            valid = (len(values) == 12 and all(_count(v) for v in values)
                and all(values[i] in (0, 1) for i in (4, 5, 6, 8, 9, 11))
                and values[7] == 2 and values[10] in (0, 2))
            if valid:
                count, previous, minimum, maximum_drop = values[:4]
                minimum_failed = count < minimum
                drop_failed = bool(values[11] and values[4] and not values[5]
                                   and previous > count and previous-count > maximum_drop)
                expected_quality = 2 if values[6] and not minimum_failed and not drop_failed else 0
                valid = (values[8] == minimum_failed and values[9] == drop_failed
                         and values[10] == expected_quality)
            if not valid:
                problems.append(dict(info, reason="NATIVE_QUALITY_DECISION_INVALID", raw_values=values))
            else:
                quality_decisions.append(dict(info, inliers=int(count), previous_inliers=int(previous),
                    minimum=int(minimum), maximum_drop=int(maximum_drop), has_previous=bool(values[4]),
                    last_was_keyframe=bool(values[5]), visual_correction=bool(values[6]),
                    population="SCHUR_INLIERS", minimum_failed=minimum_failed, drop_failed=drop_failed,
                    tracking_quality=expected_quality, compare_previous=bool(values[11])))
        elif kind == "mono_metric_initialized":
            committed = any(row["observation_segment"] == segment and row["bundle_id"] == bundle
                            and row.get("initialization_method") == "MONOCULAR_VISUAL_IMU_ALIGNMENT" for row in commits)
            if not committed:
                problems.append(dict(info, reason="METRIC_INITIALIZATION_WITHOUT_FILTER_COMMIT_EVENT"))
            init_events.append(dict(info, raw_values=values, same_bundle_filter_commit_observed=committed,
                                    initialization_method="MONOCULAR_VISUAL_IMU_ALIGNMENT",
                                    scale_source="NATIVE_VISUAL_IMU_ALIGNMENT_NO_TRUTH",
                                    source_elapsed_s=(sensor_ns-cameras[0])/1e9))
        elif kind == "mono_alignment_attempt":
            alignment_attempts.append(dict(info, raw_values=values))
        elif kind == "mono_bootstrap_restart_requested":
            restarts.append(dict(info, raw_values=values))
        elif kind == "reproject":
            reprojects.append(dict(info, raw_values=values))
        elif kind in ("imu_interval", "imu_raw_interval"):
            destination = imu_intervals if kind == "imu_interval" else raw_imu_intervals
            destination.append(dict(info, raw_values=values))
        elif kind == "schur_visual_correction":
            reason = None
            if len(values) != 21 or any(value is None for value in values):
                reason = "COMPLETE_FINITE_NATIVE_STATE_REQUIRED"
            elif not _count(values[0]) or values[0] <= 0:
                reason = "ACTUALLY_USED_CONSTRAINTS_REQUIRED"
            elif not _count(values[20]):
                reason = "NATIVE_EPOCH_INVALID"
            elif abs(math.hypot(*values[4:8])-1) > 1e-6:
                reason = "NATIVE_QUATERNION_NOT_UNIT"
            elif previous_epoch is not None and values[20] < previous_epoch:
                reason = "NATIVE_EPOCH_DECREASED"
            elif corrections and int(values[20]) == previous_epoch and sensor_ns <= corrections[-1]["sensor_ns"]:
                reason = "NATIVE_CORRECTION_TIME_NOT_INCREASING_WITHIN_EPOCH"
            if reason:
                rejected_corrections.append(dict(info, reason=reason, raw_values=values))
            else:
                epoch = int(values[20])
                if corrections and epoch != previous_epoch:
                    transition = dict(info, previous_epoch=previous_epoch, native_epoch=epoch)
                    if corrections[-1]["observation_segment"] == segment:
                        transition["reset_event_missing"] = True
                        problems.append(dict(transition, reason="NATIVE_EPOCH_CHANGED_WITHOUT_RESET_EVENT"))
                    native_epoch_transitions.append(transition)
                previous_epoch = epoch
                corrections.append(dict(info, constraint_count=int(values[0]), position=values[1:4],
                                        orientation_wxyz=values[4:8], velocity_parent_m_s=values[8:11],
                                        accel_bias_m_s2=values[11:14], gyro_bias_rad_s=values[14:17],
                                        gravity_parent_m_s2=values[17:20], native_epoch=epoch,
                                        state_source="EXPLICIT_NATIVE_ACCEPTED_VISUAL_CORRECTION",
                                        health_qualification="NOT_RUN"))
    if not event_count:
        return absent
    for key, begin in pending.items():
        problems.append(dict(reason="SPAN_END_MISSING", phase=key[0], begin=begin))
    if rejected_corrections:
        problems.append(dict(reason="INVALID_NATIVE_VISUAL_CORRECTION_EVENTS", count=len(rejected_corrections)))
    if rejected_stereo:
        problems.append(dict(reason="INVALID_STEREO_METRIC_INITIALIZATION_EVENTS", count=len(rejected_stereo)))
    capped = event_count == max_events
    if capped:
        problems.append(dict(reason="EVENT_LIMIT_REACHED", meaning="logger may have truncated subsequent evidence"))

    completed_ids = {row["camera_index"] for row in processed}
    timing = {phase: dict(_stats([row["duration_ns"] for row in rows]),
                          clock="ONE_NATIVE_STEADY_CLOCK", boundary=f"{phase}_begin to {phase}_end; same bundle and reset segment")
              for phase, rows in spans.items()}
    epochs = []
    for epoch, epoch_segment in sorted({(row["native_epoch"], row["observation_segment"]) for row in corrections}):
        rows = [row for row in corrections if row["native_epoch"] == epoch and row["observation_segment"] == epoch_segment]
        epoch_times = [row["sensor_ns"] for row in rows]
        epochs.append(dict(native_epoch=epoch, observation_segment=epoch_segment, count=len(rows),
                           first_sensor_ns=epoch_times[0], last_sensor_ns=epoch_times[-1],
                           sensor_clock_frequency=_frequency(epoch_times, "CAMERA_SENSOR"),
                           native_clock_frequency=_frequency([row["monotonic_ns"] for row in rows], "NATIVE_STEADY")))
    unavailable = dict(status="NOT_RUN", reason="no explicit same-clock native boundary/evidence is exposed")
    return dict(
        schema_version=1, status="FAIL" if problems else "PASS", event_count=event_count,
        verification_scope="NATIVE_OBSERVATION_PARSING_ONLY", estimator_reproduction="NOT_RUN",
        source=dict(path=str(path), sha256=file_hash(path),
                    schema_version=1, bounded_event_limit=max_events),
        coverage=dict(complete_trace=False if capped or pending else None,
                      complete_trace_status="NOT_RUN", paired_spans_complete=not pending,
                      completeness_reason="runtime must verify native exit and trace provenance; no end-of-trace footer exists",
                      event_limit_reached=capped,
                      source_association="EXACT_INTEGER_NANOSECONDS_NO_NEAREST_TIME_FIT", camera_id=camera_id,
                      input_count=len(cameras), frontend_started_count=len(started), frontend_completed_count=len(processed),
                      camera_observation_segments=[dict(camera_index=row['camera_index'],
                          sensor_ns=row['sensor_ns'], observation_segment=row['observation_segment'])
                          for row in processed],
                      unique_completed_camera_count=len(completed_ids), input_completed_ratio=len(completed_ids)/len(cameras),
                      missing_camera_indices=[i for i in range(len(cameras)) if i not in completed_ids],
                      input_sensor_frequency=_frequency(cameras, "CAMERA_SENSOR")),
        event_counts=dict(event_counts), protocol_issues=problems, timings=timing,
        tracking_quality=dict(status="PASS" if quality_decisions else "NOT_RUN", events=quality_decisions,
                              interpretation="current native Schur inliers; bootstrap is not visual correction"),
        input_health=dict(status="PASS" if input_health else "NOT_RUN", events=input_health,
                          source_imu_payload_reconciliation="NOT_RUN", truth_used=False),
        initialization=dict(status="PASS" if init_events else "NOT_RUN", exact_metric_events=init_events,
                            exact_metric_completion_observed=bool(init_events), metric_filter_commits=commits,
                            geometric_commits=geometry, geometric_is_metric=False,
                            stereo_geometry_events=stereo_geometry, stereo_filter_commits=stereo_commits,
                            rejected_stereo_events=rejected_stereo,
                            alignment_attempts=alignment_attempts, first_pose_proxy=proxy,
                            meaning="explicit mono alignment or stereo geometry/filter bootstrap; no health/flight qualification"),
        visual_corrections=dict(status="PASS" if corrections else ("FAIL" if rejected_corrections else "NOT_RUN"),
                                count=len(corrections) if event_counts["schur_visual_correction"] else None,
                                events=corrections, rejected_events=rejected_corrections, epochs=epochs,
                                numerical_pose_is_correction=False, backward_end_is_correction=False,
                                per_camera_accepted_update_ratio=(len({row['sensor_ns'] for row in corrections})/len(cameras)
                                                                 if event_counts["schur_visual_correction"] else None)),
        lifecycle=dict(observed_reset_call_count=len(reset_events),
                       post_frontend_reset_call_count=sum(not row["startup"] for row in reset_events),
                       reset_events=reset_events, native_epoch_transitions=native_epoch_transitions,
                       segment_semantics="observation segments delimit reset calls; not substituted for native epochs",
                       frontend_failures=frontend_failures, bootstrap_restart_requests=restarts,
                       tracking_validity_ratio=unavailable),
        imu=dict(intervals=imu_intervals, raw_intervals=raw_imu_intervals,
                 ready_false_count=sum(bool(row["raw_values"]) and row["raw_values"][0] == 0 for row in imu_intervals),
                 ready_unknown_count=sum(not row["raw_values"] or row["raw_values"][0] not in (0, 1)
                                         for row in imu_intervals),
                 exact_used_sample_ids=unavailable, maximum_sample_gap_ns=unavailable,
                 endpoint_semantics="values retain native double seconds; not original integer sample times"),
        reproject=dict(events=reprojects, accepted_constraint_count_is_not_reprojection_count=True),
        publication_latency=unavailable, processing_queue_latency=unavailable, covariance=unavailable,
        health_qualification="NOT_RUN", truth_used=False,
    )
