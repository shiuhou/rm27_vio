"""Native diagnostic evidence tests: no estimator, truth, or hardware runs."""

import json

import pytest

from rm27.perception.vision.localization.experiments.common import ExperimentError
from rm27.perception.vision.localization.experiments.native_events import analyze_native_events

BASE = 1_403_715_273_262_142_976
CAMERAS = [BASE + i*50_000_001 for i in range(5)]


def event(kind, *, index=0, monotonic_ns=0, values=None, sensor_ns=None, bundle_id=None):
    return dict(schema_version=1, kind=kind, sensor_ns=CAMERAS[index] if sensor_ns is None else sensor_ns,
                bundle_id=index if bundle_id is None else bundle_id,
                monotonic_ns=monotonic_ns, values=[] if values is None else values)


def correction_values(epoch=1, constraints=42):
    return [constraints, 1, 2, 3, 1, 0, 0, 0, 0.1, 0.2, 0.3, 0, 0, 0, 0, 0, 0, 0, 0, -9.81, epoch]


def trace(tmp_path, rows):
    path = tmp_path/"native-events.jsonl"
    path.write_text("".join(json.dumps(row, allow_nan=False)+"\n" for row in rows))
    return path


def test_exact_source_association_and_paired_native_timers(tmp_path):
    rows = [event("reset", sensor_ns=-1, bundle_id=-1, monotonic_ns=1)]
    for i in range(3):
        start = 1_000+1_000*i
        rows += [event("frontend_begin", index=i, monotonic_ns=start),
                 event("schur_forward_begin", index=i, monotonic_ns=start+10),
                 event("schur_forward_end", index=i, monotonic_ns=start+30+i),
                 event("schur_backward_begin", index=i, monotonic_ns=start+40),
                 event("schur_visual_correction", index=i, monotonic_ns=start+45,
                       values=correction_values()),
                 event("schur_backward_end", index=i, monotonic_ns=start+50+2*i, values=[20, 3]),
                 event("frontend_end", index=i, monotonic_ns=start+100+10*i, values=[2, 0, 2])]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "PASS"
    assert report["coverage"]["source_association"].startswith("EXACT_INTEGER")
    assert report["timings"]["frontend"]["count"] == 3
    assert report["timings"]["frontend"]["p50"] == 110
    assert report["timings"]["frontend"]["p95"] == pytest.approx(119)
    assert report["timings"]["frontend"]["max"] == 120
    assert report["timings"]["schur_forward"]["p50"] == 21
    assert report["timings"]["schur_backward"]["max"] == 14
    assert report["visual_corrections"]["count"] == 3
    assert report["visual_corrections"]["events"][1]["sensor_ns"] == CAMERAS[1]
    assert report["visual_corrections"]["epochs"][0]["sensor_clock_frequency"]["hz"] == pytest.approx(1e9/50_000_001)
    assert report["coverage"]["missing_camera_indices"] == [3, 4]
    assert report["coverage"]["input_completed_ratio"] == 3/5
    assert report["health_qualification"] == "NOT_RUN" and not report["truth_used"]
    assert report["publication_latency"]["status"] == "NOT_RUN"
    json.dumps(report, allow_nan=False)


def test_geometry_and_tracking_pose_proxy_are_not_metric_initialization(tmp_path):
    rows = [event("mono_geometric_commit", monotonic_ns=1),
            event("schur_backward_begin", monotonic_ns=2),
            event("schur_backward_end", monotonic_ns=3, values=[100, 1])]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS, first_pose_sensor_ns=CAMERAS[1])
    assert report["initialization"]["status"] == "NOT_RUN"
    assert not report["initialization"]["exact_metric_completion_observed"]
    assert report["initialization"]["first_pose_proxy"]["source_elapsed_s"] == 0.050000001
    assert report["initialization"]["geometric_is_metric"] is False
    assert report["visual_corrections"]["count"] is None
    assert report["visual_corrections"]["status"] == "NOT_RUN"


def test_explicit_metric_completion_requires_same_bundle_filter_commit(tmp_path):
    rows = [event("mono_alignment_attempt", monotonic_ns=1, values=[1, 2.3, 1, 0, .01, 9.81, 0, 0]),
            event("metric_filter_commit", index=1, monotonic_ns=2, values=[CAMERAS[1]/1e9, 1, 0, 0, 0, 0, 0, 0, 0, 0]),
            event("mono_metric_initialized", index=1, monotonic_ns=3, values=[2.3, 1, 0, .01])]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    initialization = report["initialization"]
    assert initialization["status"] == "PASS"
    assert initialization["exact_metric_events"][0]["sensor_ns"] == CAMERAS[1]
    assert initialization["exact_metric_events"][0]["source_elapsed_s"] == 0.050000001
    assert initialization["exact_metric_events"][0]["same_bundle_filter_commit_observed"]
    assert report["status"] == "PASS"
    missing = analyze_native_events(trace(tmp_path, rows[2:]), CAMERAS)
    assert missing["status"] == "FAIL"
    assert missing["protocol_issues"][0]["reason"] == "METRIC_INITIALIZATION_WITHOUT_FILTER_COMMIT_EVENT"


@pytest.mark.parametrize("change,reason", [
    ({0: 0}, "ACTUALLY_USED_CONSTRAINTS_REQUIRED"),
    ({5: None}, "COMPLETE_FINITE_NATIVE_STATE_REQUIRED"),
    ({4: 2}, "NATIVE_QUATERNION_NOT_UNIT"),
    ({20: -1}, "NATIVE_EPOCH_INVALID"),
])
def test_unqualified_correction_events_cannot_become_valid_updates(tmp_path, change, reason):
    values = correction_values()
    for i, value in change.items():
        values[i] = value
    report = analyze_native_events(trace(tmp_path, [event("schur_visual_correction", values=values)]), CAMERAS)
    assert report["status"] == "FAIL"
    assert report["visual_corrections"]["count"] == 0
    assert report["visual_corrections"]["rejected_events"][0]["reason"] == reason


def test_reset_segments_spans_and_native_epochs_remain_distinct(tmp_path):
    rows = [event("reset", sensor_ns=-1, bundle_id=-1, monotonic_ns=0),
            event("frontend_begin", monotonic_ns=1),
            event("frontend_end", monotonic_ns=2),
            event("schur_visual_correction", monotonic_ns=3, values=correction_values(1)),
            event("schur_forward_begin", index=1, monotonic_ns=4),
            event("reset", index=1, monotonic_ns=5),
            event("schur_forward_end", index=1, monotonic_ns=6),
            event("schur_visual_correction", index=2, monotonic_ns=7, values=correction_values(3))]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "FAIL"  # The interrupted span is retained.
    assert report["lifecycle"]["observed_reset_call_count"] == 2
    assert report["lifecycle"]["post_frontend_reset_call_count"] == 1
    assert report["timings"]["schur_forward"]["status"] == "NOT_RUN"
    assert len(report["visual_corrections"]["epochs"]) == 2
    assert [row["native_epoch"] for row in report["visual_corrections"]["epochs"]] == [1, 3]
    assert report["lifecycle"]["native_epoch_transitions"][0]["previous_epoch"] == 1
    assert all(row["sensor_clock_frequency"]["status"] == "NOT_RUN" for row in report["visual_corrections"]["epochs"])


def test_changed_native_epoch_without_reset_is_reported(tmp_path):
    rows = [event("schur_visual_correction", monotonic_ns=1, values=correction_values(1)),
            event("schur_visual_correction", index=1, monotonic_ns=2, values=correction_values(2))]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "FAIL"
    assert report["lifecycle"]["native_epoch_transitions"][0]["reset_event_missing"]


def test_duplicate_correction_timestamp_is_not_a_new_visual_update(tmp_path):
    rows = [event("schur_visual_correction", monotonic_ns=1, values=correction_values()),
            event("schur_visual_correction", monotonic_ns=2, values=correction_values())]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["visual_corrections"]["count"] == 1
    assert report["visual_corrections"]["rejected_events"][0]["reason"] == "NATIVE_CORRECTION_TIME_NOT_INCREASING_WITHIN_EPOCH"


def test_imu_double_endpoints_do_not_invent_sample_gap_or_used_ids(tmp_path):
    rows = [event("imu_raw_interval", monotonic_ns=1, values=[11, BASE/1e9, CAMERAS[1]/1e9, .001]),
            event("imu_interval", monotonic_ns=2, values=[0, 0, -1, -1]),
            event("mono_bootstrap_restart_requested", monotonic_ns=3)]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["imu"]["ready_false_count"] == 1
    assert report["imu"]["maximum_sample_gap_ns"]["status"] == "NOT_RUN"
    assert report["imu"]["exact_used_sample_ids"]["status"] == "NOT_RUN"
    assert report["imu"]["raw_intervals"][0]["raw_values"][3] == .001
    assert len(report["lifecycle"]["bootstrap_restart_requests"]) == 1


def test_frontend_failure_enum_is_not_tracking_or_reset_inference(tmp_path):
    rows = [event("frontend_begin", monotonic_ns=1),
            event("frontend_end", monotonic_ns=2, values=[1, 2, 0])]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert len(report["lifecycle"]["frontend_failures"]) == 1
    assert report["lifecycle"]["observed_reset_call_count"] == 0
    assert report["lifecycle"]["tracking_validity_ratio"]["status"] == "NOT_RUN"


def test_exact_integer_ns_no_tolerance_or_float_coercion(tmp_path):
    with pytest.raises(ExperimentError, match="NATIVE_CAMERA_UNMATCHED"):
        analyze_native_events(trace(tmp_path, [event("frontend_begin", sensor_ns=BASE+1)]), CAMERAS)
    with pytest.raises(ExperimentError, match="integer"):
        analyze_native_events(trace(tmp_path, [event("frontend_begin", sensor_ns=float(BASE))]), CAMERAS)
    with pytest.raises(ExperimentError, match="no sorting"):
        analyze_native_events(trace(tmp_path, []), list(reversed(CAMERAS)))


def test_existing_camera_ledger_rows_filter_only_selected_camera(tmp_path):
    ledger = [dict(stream="imu0", state_ns=1), dict(stream="cam0", state_ns=BASE),
              dict(stream="cam1", state_ns=BASE), dict(stream="cam0", state_ns=CAMERAS[1])]
    report = analyze_native_events(trace(tmp_path, [event("frontend_begin"), event("frontend_end", monotonic_ns=1)]), ledger)
    assert report["coverage"]["input_count"] == 2
    assert report["coverage"]["unique_completed_camera_count"] == 1


def test_incomplete_spans_and_logger_cap_prevent_complete_observation_claim(tmp_path):
    path = trace(tmp_path, [event("frontend_begin")])
    report = analyze_native_events(path, CAMERAS, max_events=1)
    assert report["status"] == "FAIL"
    assert report["coverage"]["event_limit_reached"]
    assert not report["coverage"]["complete_trace"]
    assert report["timings"]["frontend"]["status"] == "NOT_RUN"
    with pytest.raises(ExperimentError, match="NATIVE_EVENT_LIMIT"):
        analyze_native_events(trace(tmp_path, [event("unknown"), event("unknown", monotonic_ns=1)]), CAMERAS, max_events=1)


def test_missing_empty_traces_stay_not_run(tmp_path):
    assert analyze_native_events(tmp_path/"missing", CAMERAS)["status"] == "NOT_RUN"
    assert analyze_native_events(trace(tmp_path, []), CAMERAS)["status"] == "NOT_RUN"


def test_monotonic_rewind_and_ambiguous_or_conflicting_bundles_are_rejected(tmp_path):
    with pytest.raises(ExperimentError, match="NATIVE_CLOCK_REWIND"):
        analyze_native_events(trace(tmp_path, [event("frontend_begin", monotonic_ns=2), event("frontend_end", monotonic_ns=1)]), CAMERAS)
    with pytest.raises(ExperimentError, match="NATIVE_SPAN_AMBIGUOUS"):
        analyze_native_events(trace(tmp_path, [event("frontend_begin"), event("frontend_begin", monotonic_ns=1)]), CAMERAS)
    with pytest.raises(ExperimentError, match="NATIVE_BUNDLE_TIME_CONFLICT"):
        analyze_native_events(trace(tmp_path, [event("frontend_begin"), event("frontend_end", index=1, bundle_id=0, monotonic_ns=1)]), CAMERAS)


def test_raw_nan_and_bool_evidence_are_rejected_but_native_null_is_preserved(tmp_path):
    path = tmp_path/"events.jsonl"
    path.write_text(json.dumps(event("imu_interval", values=[float("nan")]))+"\n")
    with pytest.raises(ExperimentError, match="non-finite"):
        analyze_native_events(path, CAMERAS)
    with pytest.raises(ExperimentError, match="booleans"):
        analyze_native_events(trace(tmp_path, [event("imu_interval", values=[True])]), CAMERAS)
    report = analyze_native_events(trace(tmp_path, [event("imu_interval", values=[None])]), CAMERAS)
    assert report["imu"]["intervals"][0]["raw_values"] == [None]
    json.dumps(report, allow_nan=False)
