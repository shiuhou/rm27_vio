"""Bounded Schur native observations, independent of truth and output health.

Integer camera timestamps associate events exactly. Only paired timestamps from
one native steady clock measure durations. A frontend tracking stage, a Schur
Backward return, and a nonzero feature count are not accepted visual corrections.
The explicit ``schur_visual_correction`` event is required for that observation.
No event in this module alone qualifies a state for flight-controller fusion.
"""

from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

from .common import ExperimentError, file_hash, require

NATIVE_EVENT_LIMIT = 200_000
MAX_EVENT_LINE_BYTES = 32_768
PAIRED_PHASES = ("frontend", "schur_forward", "schur_backward")


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
            require(bundle >= 0, "NATIVE_EVENT_INVALID", "camera event requires native bundle ID")
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
            commits.append(dict(info, raw_values=values))
        elif kind == "mono_metric_initialized":
            committed = any(row["observation_segment"] == segment and row["bundle_id"] == bundle for row in commits)
            if not committed:
                problems.append(dict(info, reason="METRIC_INITIALIZATION_WITHOUT_FILTER_COMMIT_EVENT"))
            init_events.append(dict(info, raw_values=values, same_bundle_filter_commit_observed=committed,
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
        initialization=dict(status="PASS" if init_events else "NOT_RUN", exact_metric_events=init_events,
                            exact_metric_completion_observed=bool(init_events), metric_filter_commits=commits,
                            geometric_commits=geometry, geometric_is_metric=False,
                            alignment_attempts=alignment_attempts, first_pose_proxy=proxy,
                            meaning="explicit derivative metric initialization event; no health/flight qualification"),
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
