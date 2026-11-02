"""Native diagnostic evidence tests: no estimator, truth, or hardware runs."""

import copy
import hashlib
import json
from pathlib import Path
import struct

import pytest

from rm27.perception.vision.localization.experiments.common import ExperimentError
from rm27.perception.vision.localization.experiments.native_events import (
    analyze_native_events, analyze_native_imu_ledger,
)
from tools.openvins.native_observations import analyze_openvins_observations

BASE = 1_403_715_273_262_142_976
CAMERAS = [BASE + i*50_000_001 for i in range(5)]


def imu_ledger_fixture(tmp_path, *, times=None, dispositions=None):
    """Small wire-shaped callback evidence, independent from camera events."""
    from rm27.perception.vision.localization.experiments.native_events import IMU_LEDGER_BYTE_LIMIT
    times = times or [BASE+i*5_000_000 for i in range(4)]
    dispositions = dispositions or ['buffer_accepted']*len(times)
    observer, rows = [], []
    bits = lambda x: struct.unpack('=Q', struct.pack('=d', x))[0]
    for index, (ns, disposition) in enumerate(zip(times, dispositions)):
        sec, nsec = divmod(ns, 10**9)
        values = [index+.1, -.2, .3, -0., .5, 9.81]
        frame = 'imu0'
        observer.append(dict(state_ns=ns, native_sequence=index+10, frame_id=frame,
                             gyro=values[:3], accel=values[3:]))
        native_seconds = float(sec)+float(nsec)*1e-9
        rows.append(dict(schema_version=1, kind='schur_imu_callback', receipt_sequence=index,
            header_sequence=index+10, sensor_id='imu0', raw_sec=sec, raw_nsec=nsec, sensor_ns=ns,
            frame_id_hex=frame.encode().hex(), frame_id_size=len(frame), frame_id_truncated=False,
            native_seconds=native_seconds, native_seconds_bits=bits(native_seconds),
            receipt_monotonic_ns=100+index*10, completion_monotonic_ns=105+index*10,
            disposition=disposition, health_code=5 if disposition=='health_rejected' else 0,
            values=values, value_bits=list(map(bits, values))))
    rows.append(dict(schema_version=1, kind='schur_imu_ledger_closed', attempted_callbacks=len(times),
        retained_records=len(times), written_records=len(times), dropped_records=0, output_dropped_records=0,
        record_capacity=200000, byte_limit=IMU_LEDGER_BYTE_LIMIT, allocation_failed=False,
        io_failed=False, monotonic_ns=1000, startup_monotonic_ns=1,
        close_boundary='svo_interface_callback_threads_joined'))
    return tmp_path/'native-imu-ledger.jsonl', observer, rows


def write_imu_ledger(path, rows):
    path.write_text(''.join(json.dumps(row, allow_nan=False)+'\n' for row in rows))
    return path


def test_native_imu_ledger_exact_callbacks_are_not_downstream_integration(tmp_path):
    path, observer, rows = imu_ledger_fixture(tmp_path)
    report = analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)
    assert report['status'] == 'PASS'
    assert report['callback_coverage']['matched_count'] == 4
    assert report['buffer_insertion']['accepted_count'] == 4
    assert report['downstream_integration'] == 'UNOBSERVED'
    assert report['reset_mapping'] == report['health_qualification'] == 'NOT_RUN'
    assert report['callback_duration']['max'] == 5
    assert report['truth_used'] is False
    json.dumps(report, allow_nan=False)


def test_native_imu_ledger_legacy_absence_is_not_run(tmp_path):
    result = analyze_native_imu_ledger(tmp_path/'missing', [])
    assert result['status'] == result['callback_coverage']['status'] == 'NOT_RUN'


def test_native_imu_ledger_preserves_duplicate_and_reversed_sensor_time_occurrences(tmp_path):
    path, observer, rows = imu_ledger_fixture(tmp_path, times=[BASE, BASE, BASE-1, BASE+5])
    # Even identical ROS header/payload occurrences remain separate records.
    import copy
    observer[1] = copy.deepcopy(observer[0])
    rows[1] = dict(copy.deepcopy(rows[0]), receipt_sequence=1,
                   receipt_monotonic_ns=110, completion_monotonic_ns=115)
    result = analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)
    assert result['status'] == 'PASS' and result['callback_coverage']['callback_count'] == 4


def test_native_imu_ledger_missing_occurrences_remain_failed_with_ranges(tmp_path):
    path, observer, rows = imu_ledger_fixture(tmp_path)
    rows = [rows[0], rows[3], rows[4]]
    rows[1]['receipt_sequence'] = 1
    for field in ('attempted_callbacks', 'retained_records', 'written_records'):
        rows[-1][field] = 2
    result = analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)
    assert result['status'] == 'FAIL'
    assert result['callback_coverage']['missing_observer_ranges'] == [
        dict(first_observer_index=1, last_observer_index=2, count=2)]


@pytest.mark.parametrize('disposition', ['health_rejected', 'paused', 'no_handler', 'buffer_rejected'])
def test_native_imu_ledger_buffer_rejections_do_not_hide_callback_receipt(tmp_path, disposition):
    path, observer, rows = imu_ledger_fixture(tmp_path, dispositions=[disposition]*4)
    result = analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)
    assert result['callback_coverage']['status'] == 'PASS'
    assert result['buffer_insertion']['accepted_count'] == 0
    assert result['buffer_insertion']['disposition_counts'] == {disposition: 4}
    assert result['downstream_integration'] == 'UNOBSERVED'


@pytest.mark.parametrize('field,value', [
    ('receipt_sequence', 1), ('header_sequence', 11), ('sensor_ns', BASE+1),
    ('raw_nsec', 10**9), ('raw_sec', True), ('frame_id_hex', '00'),
    ('frame_id_truncated', True), ('frame_id_size', 129), ('sensor_id', 'different'),
    ('values', [0]*6), ('value_bits', [0]*6), ('native_seconds_bits', 0),
    ('disposition', 'callback_unfinished'), ('health_code', 3),
    ('completion_monotonic_ns', 99), ('receipt_monotonic_ns', -1),
])
def test_native_imu_ledger_rejects_timestamp_identity_payload_and_completion_corruption(tmp_path, field, value):
    path, observer, rows = imu_ledger_fixture(tmp_path)
    rows[0][field] = value
    with pytest.raises(ExperimentError, match='NATIVE_IMU_LEDGER_INVALID'):
        analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)


@pytest.mark.parametrize('field,value', [
    ('attempted_callbacks', 5), ('retained_records', 3), ('written_records', 3),
    ('dropped_records', 1), ('output_dropped_records', 1),
    ('allocation_failed', True), ('io_failed', True),
])
def test_native_imu_ledger_overflow_and_io_failure_cannot_claim_complete(tmp_path, field, value):
    path, observer, rows = imu_ledger_fixture(tmp_path)
    rows[-1][field] = value
    with pytest.raises(ExperimentError, match='NATIVE_IMU_LEDGER_INCOMPLETE'):
        analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)


def test_native_imu_ledger_requires_last_footer_and_ordered_observer_association(tmp_path):
    path, observer, rows = imu_ledger_fixture(tmp_path)
    with pytest.raises(ExperimentError, match='footer absent'):
        analyze_native_imu_ledger(write_imu_ledger(path, rows[:-1]), observer)
    with pytest.raises(ExperimentError, match='event after closure'):
        analyze_native_imu_ledger(write_imu_ledger(path, rows+[rows[0]]), observer)
    rows[0], rows[1] = rows[1], rows[0]
    rows[0].update(receipt_sequence=0, receipt_monotonic_ns=100, completion_monotonic_ns=105)
    rows[1].update(receipt_sequence=1, receipt_monotonic_ns=110, completion_monotonic_ns=115)
    with pytest.raises(ExperimentError, match='reordered relative to observer'):
        analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)


def test_native_imu_ledger_keeps_actual_toSec_rounding_and_signed_zero_bits(tmp_path):
    # ROS sec + nsec*1e-9 and integer_ns/1e9 can differ by one ULP.
    ns = 1_403_715_280_462_143_104
    path, observer, rows = imu_ledger_fixture(tmp_path, times=[ns])
    assert rows[0]['native_seconds'] != ns/1e9
    result = analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)
    assert result['status'] == 'PASS'
    rows[0]['values'][3] = 0.
    with pytest.raises(ExperimentError, match='axis bits mismatch'):
        analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)


def test_native_imu_ledger_rejects_self_consistent_double_bits_from_wrong_raw_time(tmp_path):
    path, observer, rows = imu_ledger_fixture(tmp_path)
    rows[0]['native_seconds'] = 42.
    rows[0]['native_seconds_bits'] = struct.unpack('=Q', struct.pack('=d', 42.))[0]
    with pytest.raises(ExperimentError, match='inconsistent with its raw ROS stamp'):
        analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)


def test_native_imu_ledger_camera_fault_code_cannot_label_an_imu_rejection(tmp_path):
    path, observer, rows = imu_ledger_fixture(tmp_path, dispositions=['health_rejected']*4)
    rows[0]['health_code'] = 1
    with pytest.raises(ExperimentError, match='disposition/health'):
        analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)


@pytest.mark.parametrize('field,value', [
    ('close_boundary', None), ('close_boundary', 'standalone_test_explicit_close'),
    ('startup_monotonic_ns', 0), ('startup_monotonic_ns', 101),
])
def test_native_imu_ledger_requires_eager_start_and_joined_callback_threads(tmp_path, field, value):
    path, observer, rows = imu_ledger_fixture(tmp_path)
    rows[-1][field] = value
    with pytest.raises(ExperimentError, match='startup and joined-thread'):
        analyze_native_imu_ledger(write_imu_ledger(path, rows), observer)


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


def stereo_geometry_values():
    return [0, 1, .11, 80, 80, 1]


def stereo_filter_commit_values(index=0, epoch=1):
    return [CAMERAS[index]/1e9, epoch, 0, 1, 21, 21, 160, 80, 1,
            1, 0, 0, 0, 0, 0, 0, .1, .2, .3, 0, 0, 0, 0, 0, 0,
            0, 0, -9.81007, 1, 0, 0, 0, 0, 0, 0, .1, .2, .3, .11, 1]


def stereo_bootstrap_events(index=0, epoch=1):
    return [event("stereo_metric_geometry_ready", index=index, monotonic_ns=1, values=stereo_geometry_values()),
            event("stereo_filter_state_commit", index=index, monotonic_ns=2,
                  values=stereo_filter_commit_values(index, epoch)),
            event("stereo_metric_initialized", index=index, monotonic_ns=3, values=[0, 1, .11, 80, 80, 2, 2])]


def test_stereo_bootstrap_requires_actual_geometry_and_clone_commit_without_inventing_visual_correction(tmp_path):
    report = analyze_native_events(trace(tmp_path, stereo_bootstrap_events()), CAMERAS)
    init = report["initialization"]
    assert report["status"] == "PASS"
    assert init["status"] == "PASS" and init["exact_metric_completion_observed"]
    actual = init["exact_metric_events"][0]
    assert actual["initialization_method"] == "CALIBRATED_STEREO_BASELINE_WITH_UPSTREAM_IMU_PRIORS"
    assert actual["scale_source"] == "PINNED_CALIBRATED_STEREO_BASELINE"
    assert actual["same_bundle_filter_commit_observed"] and actual["same_bundle_stereo_geometry_observed"]
    assert actual["baseline_m"] == .11 and actual["camera_ids"] == [0, 1]
    assert actual["native_epoch"] == 1
    assert "ZERO_INITIAL_VELOCITY" in actual["upstream_initialization_assumptions"]
    assert not actual["visual_correction_observed"]
    commit = init["stereo_filter_commits"][0]
    assert commit["covariance_shape"] == [21, 21] and commit["registered_feature_count"] == 160
    assert commit["covariance_status"] == "NATIVE_FINITE_STORAGE_NOT_QUALIFIED_OUTPUT"
    assert commit["velocity_parent_m_s"] == [.1, .2, .3]
    assert report["visual_corrections"]["status"] == "NOT_RUN"
    assert report["visual_corrections"]["count"] is None
    assert report["health_qualification"] == "NOT_RUN"


def test_stereo_bootstrap_may_be_initialized_without_a_good_visual_correction(tmp_path):
    rows = stereo_bootstrap_events()
    rows[-1]["values"][-1] = 0
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "PASS"
    initialization = report["initialization"]["exact_metric_events"][0]
    assert initialization["native_stage"] == 2 and initialization["native_tracking_quality"] == 0
    assert initialization["visual_correction_observed"] is False
    assert report["visual_corrections"]["status"] == "NOT_RUN"


@pytest.mark.parametrize("values,quality", [
    ([223, 224, 40, 40, 1, 0, 1, 2, 0, 0, 2, 1], 2),
    ([7, 223, 40, 40, 1, 0, 1, 2, 1, 1, 0, 1], 0),
    ([182, 223, 40, 40, 1, 1, 1, 2, 0, 0, 2, 1], 2),
    ([0, 223, 40, 40, 1, 0, 0, 2, 1, 1, 0, 1], 0),
])
def test_native_current_quality_decision_retains_schur_population_and_exact_gates(tmp_path, values, quality):
    report = analyze_native_events(trace(tmp_path, [event("quality_decision", values=values)]), CAMERAS)
    assert report["status"] == "PASS"
    decision = report["tracking_quality"]["events"][0]
    assert decision["tracking_quality"] == quality and decision["population"] == "SCHUR_INLIERS"
    assert report["visual_corrections"]["status"] == "NOT_RUN"


@pytest.mark.parametrize("index,value", [(7, 1), (8, 1), (9, 1), (10, 0), (11, 2)])
def test_native_quality_decision_cannot_claim_a_result_inconsistent_with_its_evidence(tmp_path, index, value):
    values = [223, 224, 40, 40, 1, 0, 1, 2, 0, 0, 2, 1]
    values[index] = value
    report = analyze_native_events(trace(tmp_path, [event("quality_decision", values=values)]), CAMERAS)
    assert report["status"] == "FAIL"
    assert report["protocol_issues"][0]["reason"] == "NATIVE_QUALITY_DECISION_INVALID"


def test_native_input_boundary_retains_original_integer_imu_time_and_no_false_correction(tmp_path):
    sample = event("input_invalid_imu_gap", bundle_id=-1, values=[5, 0])
    sample.update(source_sensor_ns=CAMERAS[0]+1, source_stream="imu0")
    rows = [sample, event("input_invalidated", bundle_id=-1, monotonic_ns=1),
            event("reset", bundle_id=-1, sensor_ns=-1, monotonic_ns=2)]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "PASS"
    boundary = report["input_health"]["events"][0]
    assert boundary["source_sensor_ns"] == CAMERAS[0]+1 and boundary["source_stream"] == "imu0"
    assert boundary["observer_notification_timestamp"] is None
    assert report["visual_corrections"]["status"] == "NOT_RUN"
    assert report["lifecycle"]["observed_reset_call_count"] == 1


@pytest.mark.parametrize("change", [dict(values=[4]), dict(source_sensor_ns=1.5),
                                  dict(source_sensor_ns=True), dict(source_stream="unknown")])
def test_native_input_health_malformed_evidence_cannot_pass(tmp_path, change):
    row = event("input_invalid_imu_gap", bundle_id=-1, values=[5, 0])
    row.update(source_sensor_ns=CAMERAS[0]+1, source_stream="imu0")
    row.update(change)
    report = analyze_native_events(trace(tmp_path, [row]), CAMERAS)
    assert report["status"] == "FAIL"
    assert report["protocol_issues"][0]["reason"] == "NATIVE_INPUT_HEALTH_EVENT_INVALID"


@pytest.mark.parametrize("event_index,value_index,value", [
    (0, 1, 0), (0, 2, 0), (0, 3, 0), (0, 5, 0),
    (1, 0, CAMERAS[0]/1e9+.01), (1, 1, -1), (1, 3, 2), (1, 4, 15),
    (1, 6, 0), (1, 7, 0), (1, 8, 0), (1, 9, 2), (1, 16, None),
    (1, 28, 2), (1, 38, .2), (1, 39, 0),
    (2, 1, 0), (2, 2, .2), (2, 5, 3), (2, 6, 1),
])
def test_stereo_unqualified_or_inconsistent_native_events_fail_closed(tmp_path, event_index, value_index, value):
    rows = stereo_bootstrap_events()
    rows[event_index]["values"][value_index] = value
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "FAIL"
    assert not report["initialization"]["exact_metric_completion_observed"]
    assert report["initialization"]["rejected_stereo_events"]
    assert report["visual_corrections"]["status"] == "NOT_RUN"


@pytest.mark.parametrize("omitted", [0, 1])
def test_stereo_init_cannot_use_missing_or_other_bundle_prerequisites(tmp_path, omitted):
    rows = stereo_bootstrap_events()
    del rows[omitted]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "FAIL" and not report["initialization"]["exact_metric_completion_observed"]
    rows = stereo_bootstrap_events()
    rows[omitted].update(sensor_ns=CAMERAS[1], bundle_id=1)
    if omitted == 1:
        rows[1]["values"][0] = CAMERAS[1]/1e9
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "FAIL" and not report["initialization"]["exact_metric_completion_observed"]


def test_stereo_events_cannot_cross_reset_or_claim_mono_initialization(tmp_path):
    rows = stereo_bootstrap_events()
    rows.insert(2, event("reset", monotonic_ns=2))
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "FAIL" and not report["initialization"]["exact_metric_completion_observed"]

    rows = stereo_bootstrap_events()[:2] + [event("mono_metric_initialized", monotonic_ns=3, values=[1, 1, 0, .1])]
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "FAIL"
    assert not report["initialization"]["exact_metric_events"][0]["same_bundle_filter_commit_observed"]
    rows = stereo_bootstrap_events()
    rows[0], rows[1] = rows[1], rows[0]
    for i, row in enumerate(rows):
        row["monotonic_ns"] = i
    report = analyze_native_events(trace(tmp_path, rows), CAMERAS)
    assert report["status"] == "FAIL" and not report["initialization"]["exact_metric_completion_observed"]


def ov_native_event(kind, ns=BASE, *, values=None, camera=None, support=None):
    sec, nsec = divmod(ns, 10**9)
    time = sec+nsec*1e-9
    row = dict(schema_version=1, kind=kind, monotonic_ns=0, sequence=0,
        sensor_time_seconds=time, sensor_time_double_bits=struct.unpack("=Q", struct.pack("=d", time))[0],
        raw_sec=sec if camera is not None else -1, raw_nsec=nsec if camera is not None else -1,
        sensor_ids=[] if camera is None else [camera], values=[] if values is None else values)
    if support is not None:
        row["measurement_support"] = support
    return row


def ov_state_values():
    # Actual native state envelope: detail count, dt, JPL xyzw, p/v/bg/ba.
    return [0, .003, 0, 0, 0, 1, 1, 2, 3, .1, .2, .3, .01, .02, .03, .04, .05, .06]


def ov_support(*pairs):
    entries = []
    for camera, ns in pairs:
        row = ov_native_event("camera_ros_callback", ns, camera=camera)
        entries.append(dict(camera_id=camera, sensor_time_seconds=row["sensor_time_seconds"],
            sensor_time_double_bits=row["sensor_time_double_bits"], observation_count=2))
    return dict(schema_version=1, feature_count=2, measurement_count=2*len(entries),
                invalid_timestamps=0, overflow_measurements=0, complete=True, entries=entries)


def ov_trace(tmp_path, rows):
    for i, row in enumerate(rows):
        row.update(sequence=i, monotonic_ns=100+i)
    return trace(tmp_path, rows+[dict(schema_version=1, kind="observation_closed", monotonic_ns=100+len(rows),
                                    event_count=len(rows), dropped_events=0, io_failed=False)])


def ov_visual_rows():
    return [ov_native_event("camera_ros_callback", camera=0),
            ov_native_event("camera_ros_callback", CAMERAS[1], camera=1),
            ov_native_event("metric_initialization_committed", values=ov_state_values()),
            ov_native_event("msckf_visual_ekf_completed", CAMERAS[2], values=ov_state_values(),
                            support=ov_support((0, BASE), (1, CAMERAS[1])))]


def test_openvins_visual_support_preserves_integer_stamp_and_separates_state_time(tmp_path):
    result = analyze_openvins_observations(ov_trace(tmp_path, ov_visual_rows()))
    update = result["visual_updates"][0]
    assert result["status"] == "PASS" and result["accepted_visual_update_count"] == 1
    assert update["oldest_measurement_ns"] == BASE
    assert update["newest_measurement_ns"] == CAMERAS[1]
    assert update["state_time_seconds"] != CAMERAS[1]/1e9
    assert update["state"]["velocity_world_mps"] == [.1, .2, .3]
    assert update["state"]["camera_imu_offset_seconds"] == .003
    assert result["health_qualification"] == result["raw_input_equality"] == "NOT_RUN"
    assert not result["truth_used"]


def test_openvins_nonvisual_events_and_reused_old_support_do_not_refresh_visual_time(tmp_path):
    rows = ov_visual_rows()
    rows += [ov_native_event(kind, CAMERAS[3], values=ov_state_values()) for kind in
             ("nonvisual_zupt_ekf_completed", "imu_state_propagation_completed", "camera_returned_state")]
    rows += [ov_native_event("independent_observation_tick"),
             ov_native_event("landmark_visual_ekf_completed", CAMERAS[4], values=ov_state_values(),
                             support=ov_support((0, BASE)))]
    result = analyze_openvins_observations(ov_trace(tmp_path, rows))
    assert result["last_visual_measurement_ns"] == CAMERAS[1]
    assert result["distinct_visual_time_advance_count"] == 1
    assert result["accepted_visual_update_count"] == 2
    assert result["visual_updates"][1]["advances_visual_measurement_time"] is False
    assert result["independent_tick_count"] == 1


def test_openvins_complete_empty_or_nonvisual_trace_is_not_visual_acceptance(tmp_path):
    for rows in ([], [ov_native_event("independent_observation_tick")],
                 [ov_native_event("metric_initialization_committed", values=ov_state_values())]):
        report = analyze_openvins_observations(ov_trace(tmp_path, rows))
        assert report["status"] == "NOT_RUN" and report["observation_integrity"] == "PASS"
        assert report["accepted_visual_update_count"] == 0
        assert report["last_visual_measurement_ns"] is None


@pytest.mark.parametrize("defect", ["missing", "overflow", "invalid", "count", "duplicate", "time_bits", "camera", "future"])
def test_openvins_incomplete_or_unassociated_accepted_feature_times_are_rejected(tmp_path, defect):
    rows = ov_visual_rows()
    support = rows[-1]["measurement_support"]
    if defect == "missing":
        del rows[-1]["measurement_support"]
    elif defect == "overflow":
        support.update(overflow_measurements=1, complete=False)
    elif defect == "invalid":
        support.update(invalid_timestamps=1, complete=False)
    elif defect == "count":
        support["measurement_count"] += 1
    elif defect == "duplicate":
        support["entries"][1] = dict(support["entries"][0])
    elif defect == "time_bits":
        support["entries"][0]["sensor_time_double_bits"] += 1
    elif defect == "camera":
        support["entries"][0]["camera_id"] = 2
    else:
        rows[-1]["sensor_time_seconds"] = BASE/1e9
        rows[-1]["sensor_time_double_bits"] = struct.unpack("=Q", struct.pack("=d", BASE/1e9))[0]
    with pytest.raises(ValueError):
        analyze_openvins_observations(ov_trace(tmp_path, rows))


@pytest.mark.parametrize("defect", ["footer", "dropped", "io", "sequence", "rewind", "after_footer", "oversize", "stamp", "collision", "state", "quaternion", "initialization"])
def test_openvins_broken_native_evidence_never_becomes_qualified(tmp_path, defect):
    rows = ov_visual_rows()
    if defect == "stamp":
        rows[0]["raw_nsec"] = 10**9
    elif defect == "collision":
        rows.insert(1, dict(rows[0]))
    elif defect == "state":
        rows[-1]["values"][7] = None
    elif defect == "quaternion":
        rows[-1]["values"][5] = 2
    elif defect == "initialization":
        del rows[2]
    path = ov_trace(tmp_path, rows)
    lines = [json.loads(line) for line in path.read_text().splitlines()]
    if defect == "footer":
        lines.pop()
    elif defect == "dropped":
        lines[-1]["dropped_events"] = 1
    elif defect == "io":
        lines[-1]["io_failed"] = True
    elif defect == "sequence":
        lines[1]["sequence"] += 1
    elif defect == "rewind":
        lines[1]["monotonic_ns"] = 0
    elif defect == "after_footer":
        lines.append(dict(lines[0]))
    elif defect == "oversize":
        lines[0]["oversize"] = "x"*8192
    path.write_text("".join(json.dumps(row)+"\n" for row in lines))
    with pytest.raises(ValueError):
        analyze_openvins_observations(path)


# Preserved output from the actual pinned C++ callback/manager-prefix helper.
# Its tiny app/tracker are explicit stubs. These fixtures verify the decoder;
# neither a native estimator nor dataset consumption is certified here.
from tools.openvins import native_observations as ov_camera_parser


@pytest.fixture(scope='session')
def ov_camera_helper_fixtures():
    path=Path(__file__).parent/'fixtures/openvins_camera_consumption.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == 'ba3df968a9b7c47141a38dc4a6c3c69de2bd8ce6d019aa73cb254f0144ee6412'
    fixture=json.loads(path.read_text())
    assert fixture['provenance']['original_receipt_sha256'] == '622e002ace76838a884647c6cc4a746ec344d338f61869cf19382f12abc651c9'
    assert fixture['provenance']['dataset_consumption'] == 'NOT_RUN'
    return fixture['fixtures']


def ov_camera_reference(mode):
    import numpy as np
    import cv2
    pixels=np.array([[10*r+c for c in range(6)] for r in range(4)],dtype=np.uint8)
    camera_ids = [0,1] if mode == 'stereo' else [0]
    records={camera:[] for camera in camera_ids}
    stamps = [(0,262142976,mode!='stereo','mono8')]
    if mode == 'stereo':
        stamps += [(1,262142977,False,'mono8')]
    elif mode == 'rejects':
        stamps += [(0,263142976,False,'mono8'),(0,500000000,False,'unsupported'),
                   (0,700000000,False,'mono8'),(0,900000000,False,'mono8')]
    else:
        stamps += [(0,500000000,False,'mono8')]
    for camera,nsec,padded,encoding in stamps:
        data=bytearray([251]*36) if padded else bytearray(pixels.tobytes())
        if padded:
            for row in range(4):
                data[row*9:row*9+6]=pixels[row].tobytes()
        raw=dict(schema_version=1,provenance_valid=True,camera_id=camera,
            raw_sensor_timestamp_ns=1403715273*10**9+nsec,frame_id_hex='cam\0'.encode().hex(),
            encoding_hex=encoding.encode().hex(),raw_width=6,raw_height=4,raw_step=9 if padded else 6,
            raw_data_size=len(data),raw_data_sha256=hashlib.sha256(data).hexdigest(),
            decoded=dict(width=6,height=4,logical_pixels_sha256=hashlib.sha256(pixels.tobytes()).hexdigest()))
        front=cv2.pyrDown(pixels,dstsize=(3,2)) if mode=='downsample' else pixels
        records[camera].append(dict(source_ordinal=len(records[camera]),raw=raw,frontend=dict(width=int(front.shape[1]),height=int(front.shape[0]),logical_pixels_sha256=hashlib.sha256(front.tobytes()).hexdigest())))
    return dict(schema_version=1,complete=True,camera_ids=camera_ids,
                downsample_cameras=mode=='downsample',records=records)


def ov_camera_run_rows(tmp_path, rows, mode='mono', ref=True):
    path=tmp_path/'native.jsonl'
    path.write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in rows))
    return ov_camera_parser.analyze_camera_consumption(path,camera_ids=[0,1] if mode=='stereo' else [0],
        downsample_cameras=mode=='downsample',reference=ov_camera_reference(mode) if ref is True else (None if ref is False else ref))


def ov_camera_resequence(rows):
    count=0
    for row in rows:
        if row['kind']=='observation_closed':
            row['event_count']=count
        else:
            row['sequence']=count
            count+=1
    return rows


@pytest.mark.parametrize('mode', ['mono','downsample','stereo'])
def test_openvins_camera_actual_cpp_pipeline_and_source_reference(ov_camera_helper_fixtures,tmp_path,mode):
    result=ov_camera_run_rows(tmp_path,ov_camera_helper_fixtures[mode],mode)
    assert result['observation_integrity']=='PASS'
    assert result['full_source_consumption']=='PASS'
    assert result['occurrence_count']==2
    assert result['health_qualification']=='NOT_RUN'
    assert result['automatic_reset']=='NOT_RUN'
    assert result['per_stage_bundle_count']['camera_frontend_feed_returned']==(1 if mode=='stereo' else 2)
    if mode=='stereo':
        a,b=result['occurrences']
        assert a['raw']['raw_sensor_timestamp_ns']+1==b['raw']['raw_sensor_timestamp_ns']
        assert a['events'][0]['sensor_time_double_bits']==b['events'][0]['sensor_time_double_bits']
    if mode=='downsample':
        assert result['occurrences'][0]['events'][2]['decoded']['width']==6
        assert result['occurrences'][0]['events'][5]['decoded']['width']==3


def test_openvins_camera_actual_rejections_and_unforwarded_are_not_hidden(ov_camera_helper_fixtures,tmp_path):
    result=ov_camera_run_rows(tmp_path,ov_camera_helper_fixtures['rejects'],'rejects')
    assert result['input_coverage']['status']=='PASS'
    assert result['full_source_consumption']=='FAIL'
    assert result['terminal_counts']=={'camera_feed_measurement_returned':1,'camera_track_frequency_rejected':1,
        'camera_cv_bridge_rejected':1,'camera_subscription_not_forwarded_before_shutdown':1,
        'camera_queued_not_fed_before_shutdown':1}


def test_openvins_camera_absent_reference_does_not_qualify_source(ov_camera_helper_fixtures,tmp_path):
    result=ov_camera_run_rows(tmp_path,ov_camera_helper_fixtures['mono'],ref=False)
    assert result['input_coverage']['status']==result['full_source_consumption']=='NOT_RUN'


def test_openvins_camera_sample_reference_cannot_close_gate(ov_camera_helper_fixtures,tmp_path):
    partial=ov_camera_reference('mono');partial['complete']=False
    with pytest.raises(ExperimentError,match='complete matching camera reference'):
        ov_camera_run_rows(tmp_path,ov_camera_helper_fixtures['mono'],ref=partial)


@pytest.mark.parametrize('failure', ['schema','unknown_field','unknown_kind','missing_footer','footer_drop',
    'footer_count','after_footer','duplicate_occurrence','raw_time','raw_hash','missing_join','worker_running',
    'close_count','after_close','capacity','pixels_changed','duplicate_callback','pipeline_order',
    'missing_front_return','reference_pixels','reference_raw','stereo_bundle_order','provenance_invalid'])
def test_openvins_camera_fail_closed_boundaries(ov_camera_helper_fixtures,tmp_path,failure):
    mode='stereo' if failure=='stereo_bundle_order' else 'mono'
    rows=copy.deepcopy(ov_camera_helper_fixtures[mode]);ref=ov_camera_reference(mode)
    first=next(row for row in rows if row['kind']=='camera_subscription_received')
    queue=next(row for row in rows if row['kind']=='camera_decoded_queue_insert')
    if failure=='schema':first['camera_consumption']['schema_version']=2
    elif failure=='unknown_field':first['camera_consumption']['guessed_valid']=True
    elif failure=='unknown_kind':first['kind']='camera_new_unsupported'
    elif failure=='missing_footer':rows.pop()
    elif failure=='footer_drop':rows[-1]['dropped_events']=1
    elif failure=='footer_count':rows[-1]['event_count']+=1
    elif failure=='after_footer':rows.append(copy.deepcopy(first))
    elif failure=='duplicate_occurrence':
        second=next(row for row in rows if row['kind']=='camera_subscription_received' and row is not first)
        second['camera_consumption']['occurrence']=1
    elif failure=='raw_time':first['camera_consumption']['raw_sensor_timestamp_ns']+=1
    elif failure=='raw_hash':queue['camera_consumption']['raw_data_sha256']='0'*64
    elif failure=='missing_join':rows=[row for row in rows if row['kind']!='camera_observation_workers_joined'];ov_camera_resequence(rows)
    elif failure=='worker_running':next(row for row in rows if row['kind']=='camera_observation_workers_joined')['values']=[1]
    elif failure=='close_count':next(row for row in rows if row['kind']=='camera_receipts_closed')['values']=[3]
    elif failure=='after_close':
        row=copy.deepcopy(first);row['monotonic_ns']=rows[-1]['monotonic_ns'];rows.insert(-1,row);ov_camera_resequence(rows)
    elif failure=='capacity':
        row=copy.deepcopy(first);row['kind']='camera_receipt_capacity_or_closed_rejected';rows.insert(rows.index(first)+1,row);ov_camera_resequence(rows)
    elif failure=='pixels_changed':queue['camera_consumption']['decoded']['logical_pixels_sha256']='0'*64
    elif failure=='duplicate_callback':
        row=next(row for row in rows if row['kind']=='camera_ros_callback');rows.insert(rows.index(row)+1,copy.deepcopy(row));ov_camera_resequence(rows)
    elif failure=='pipeline_order':queue['kind']='camera_feed_measurement_enter'
    elif failure=='missing_front_return':rows=[row for row in rows if row['kind']!='camera_frontend_feed_returned'];ov_camera_resequence(rows)
    elif failure=='reference_pixels':ref['records'][0][0]['raw']['decoded']['logical_pixels_sha256']='0'*64
    elif failure=='reference_raw':ref['records'][0][0]['raw']['raw_data_sha256']='0'*64
    elif failure=='stereo_bundle_order':
        events=[row for row in rows if row['kind']=='camera_decoded_queue_insert']
        events[0]['camera_consumption'],events[1]['camera_consumption']=events[1]['camera_consumption'],events[0]['camera_consumption']
        events[0]['sensor_ids'],events[1]['sensor_ids']=events[1]['sensor_ids'],events[0]['sensor_ids']
        events[0]['raw_nsec'],events[1]['raw_nsec']=events[1]['raw_nsec'],events[0]['raw_nsec']
    elif failure=='provenance_invalid':first['camera_consumption']['provenance_valid']=False
    with pytest.raises((ExperimentError,ValueError)):
        ov_camera_run_rows(tmp_path,rows,mode,ref)


def test_openvins_camera_missing_source_subscription_remains_fail(ov_camera_helper_fixtures,tmp_path):
    rows=copy.deepcopy(ov_camera_helper_fixtures['mono'])
    second_ns=1403715273500000000
    rows=[row for row in rows if row.get('camera_consumption',{}).get('occurrence')!=2
          and not (row['kind']=='camera_ros_callback' and row['raw_sec']*10**9+row['raw_nsec']==second_ns)]
    next(row for row in rows if row['kind']=='camera_receipts_closed')['values']=[1]
    result=ov_camera_run_rows(tmp_path,ov_camera_resequence(rows))
    assert result['input_coverage']['status']=='FAIL'
    assert result['input_coverage']['per_camera'][0]['missing_source_ordinals']==[1]
    assert result['full_source_consumption']=='FAIL'


def test_openvins_camera_missing_identical_occurrences_preserve_separate_ambiguity_groups(ov_camera_helper_fixtures,tmp_path):
    ref=ov_camera_reference('mono')
    first,second=ref['records'][0]
    ref['records'][0]=[copy.deepcopy(row) for row in (first,first,second,second)]
    for index,row in enumerate(ref['records'][0]):row['source_ordinal']=index
    report=ov_camera_run_rows(tmp_path,ov_camera_helper_fixtures['mono'],ref=ref)
    assert report['input_coverage']['status']=='FAIL'
    assert [row['source_ordinal'] for row in report['occurrences']]==[None,None]
    assert [row['source_ordinal_candidates'] for row in report['occurrences']]==[[0,1],[2,3]]
    assert report['full_source_consumption']=='FAIL'


def test_openvins_camera_unknown_boundary_without_payload_is_rejected(ov_camera_helper_fixtures,tmp_path):
    rows=copy.deepcopy(ov_camera_helper_fixtures['mono'])
    first=next(row for row in rows if row['kind']=='camera_subscription_received')
    first['kind']='camera_unsupported_new_boundary'
    first.pop('camera_consumption')
    with pytest.raises(ExperimentError,match='unsupported native camera boundary'):
        ov_camera_run_rows(tmp_path,rows)


@pytest.mark.parametrize('case', ['mono-int-pubs-1','stereo-term-pubs-0','stereo-repeated-pubs-1'])
def test_openvins_actual_binary_terminal_lifecycle_without_data(tmp_path,case):
    path=Path(__file__).parent/'fixtures/openvins_camera_terminal_lifecycle.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest()=='54666cf68535c6484e246f8136db7e64f0311542af99835085aa7822ed87499c'
    fixture=json.loads(path.read_text())
    assert fixture['provenance']['receipt_sha256']=='2a29ffafd46349291edc770a20f395858cda73a5a02728467254ab4737c489af'
    result=ov_camera_run_rows(tmp_path,fixture['cases'][case]['rows'],
        'mono' if case.startswith('mono') else 'stereo',ref=False)
    assert result['observation_integrity']=='PASS'
    assert result['full_source_consumption']=='NOT_RUN'
    assert result['terminal_drain']['attempted'] is False
    assert result['terminal_drain']['watermark_available'] is False
    assert result['state_export']['count']==0
    assert result['state_export']['ros_delivery']=='NOT_RUN'
    assert result['health_qualification']=='NOT_RUN'


def ov_camera_terminal_rows(fixture):
    """Compose actual C++ pipeline rows with explicit terminal wire examples.

    This tests decoder ordering, not an estimator or middleware trajectory.
    The separate installed-binary fixture above retains its original ledger.
    """
    rows=copy.deepcopy(fixture['mono'])
    tail=[r for r in rows if r.get('camera_consumption',{}).get('occurrence')==2
          and r['kind'] in ov_camera_parser.PIPELINE[3:]]
    rows=[r for r in rows if r not in tail]
    closing=next(i for i,r in enumerate(rows) if r['kind']=='camera_receipts_closed')
    endpoint=1403715273600000000
    added=[ov_native_event('imu_ros_callback',endpoint,camera=0,values=[0]*6),
        ov_native_event('graceful_executor_callbacks_quiescent',-10**9,values=[1]),
        ov_native_event('camera_terminal_drain_boundary',-10**9,values=[1,1,1]),
        ov_native_event('camera_terminal_ready_drain_enter',endpoint,camera=0),*tail,
        ov_native_event('camera_state_export_returned',1403715273500000000,values=[1,1,1,1,128]),
        ov_native_event('camera_terminal_ready_drain_returned',endpoint,camera=0),
        ov_native_event('initializer_worker_joined',-10**9)]
    rows[closing:closing]=added
    for i,row in enumerate(rows):row['monotonic_ns']=100+i
    return ov_camera_resequence(rows)


def test_openvins_terminal_drain_preserves_camera_lineage_and_export_limits(ov_camera_helper_fixtures,tmp_path):
    result=ov_camera_run_rows(tmp_path,ov_camera_terminal_rows(ov_camera_helper_fixtures))
    assert result['full_source_consumption']=='PASS'
    assert result['terminal_drain']['attempted'] is True
    assert result['terminal_drain']['completed'] is True
    assert result['terminal_drain']['endpoint_sensor_ns']==1403715273600000000
    assert result['terminal_drain']['feed_bundle_count']==1
    assert result['state_export']['count']==1
    assert result['state_export']['live_context_call_count']==1
    assert result['state_export']['text_bytes_written']==128
    assert result['state_export']['ros_delivery']=='NOT_RUN'
    assert result['health_qualification']==result['automatic_reset']=='NOT_RUN'


@pytest.mark.parametrize('failure', ['missing_quiescent','duplicate_boundary','boundary_flags','boundary_time',
    'missing_enter','missing_return','duplicate_enter','endpoint_time','endpoint_raw','missing_imu',
    'unordered_imu','closed_before_return','late_subscription','export_shape','export_bytes','export_after_return',
    'missing_initializer','missing_boundary'])
def test_openvins_terminal_drain_rejects_incomplete_or_contradictory_evidence(ov_camera_helper_fixtures,tmp_path,failure):
    rows=ov_camera_terminal_rows(ov_camera_helper_fixtures)
    boundary=next(r for r in rows if r['kind']=='camera_terminal_drain_boundary')
    enter=next(r for r in rows if r['kind']=='camera_terminal_ready_drain_enter')
    returned=next(r for r in rows if r['kind']=='camera_terminal_ready_drain_returned')
    export=next(r for r in rows if r['kind']=='camera_state_export_returned')
    if failure=='missing_quiescent':rows=[r for r in rows if r['kind']!='graceful_executor_callbacks_quiescent']
    elif failure=='duplicate_boundary':rows.insert(rows.index(boundary),copy.deepcopy(boundary))
    elif failure=='boundary_flags':boundary['values']=[1,2,1]
    elif failure=='boundary_time':boundary.update(sensor_time_seconds=0,sensor_time_double_bits=0)
    elif failure=='missing_enter':rows.remove(enter)
    elif failure=='missing_return':rows.remove(returned)
    elif failure=='duplicate_enter':rows.insert(rows.index(enter),copy.deepcopy(enter))
    elif failure=='endpoint_time':returned.update(sensor_time_seconds=0,sensor_time_double_bits=0)
    elif failure=='endpoint_raw':returned['raw_nsec']+=1
    elif failure=='missing_imu':rows=[r for r in rows if r['kind']!='imu_ros_callback']
    elif failure=='unordered_imu':
        imu=next(r for r in rows if r['kind']=='imu_ros_callback');rows.insert(rows.index(imu),copy.deepcopy(imu))
    elif failure=='closed_before_return':
        closing=next(r for r in rows if r['kind']=='camera_receipts_closed');rows.remove(closing);rows.insert(rows.index(returned),closing)
    elif failure=='late_subscription':
        subscription=next(r for r in rows if r['kind']=='camera_subscription_received');rows.remove(subscription);rows.insert(rows.index(enter),subscription)
    elif failure=='export_shape':export['values'].pop()
    elif failure=='export_bytes':export['values'][-1]=.5
    elif failure=='export_after_return':rows.remove(export);rows.insert(rows.index(returned)+1,export)
    elif failure=='missing_initializer':rows=[r for r in rows if r['kind']!='initializer_worker_joined']
    elif failure=='missing_boundary':rows.remove(boundary)
    for i,row in enumerate(rows):row['monotonic_ns']=100+i
    with pytest.raises((ExperimentError,ValueError)):
        ov_camera_run_rows(tmp_path,ov_camera_resequence(rows))


def test_openvins_legacy_camera_summary_has_no_terminal_upgrade(ov_camera_helper_fixtures,tmp_path):
    report=ov_camera_run_rows(tmp_path,ov_camera_helper_fixtures['mono'])
    assert 'terminal_drain' not in report and 'state_export' not in report
    assert report['full_source_consumption']=='PASS'
    assert report['health_qualification']==report['automatic_reset']=='NOT_RUN'


@pytest.mark.parametrize('location', ['live','joined_worker','terminal'])
def test_openvins_export_can_finish_before_or_during_terminal_drain(ov_camera_helper_fixtures,tmp_path,location):
    rows=ov_camera_terminal_rows(ov_camera_helper_fixtures)
    export=next(r for r in rows if r['kind']=='camera_state_export_returned')
    rows.remove(export)
    before={'live':'graceful_executor_callbacks_quiescent','joined_worker':'camera_terminal_drain_boundary',
            'terminal':'camera_terminal_ready_drain_returned'}[location]
    rows.insert(next(i for i,r in enumerate(rows) if r['kind']==before),export)
    for i,row in enumerate(rows):row['monotonic_ns']=100+i
    report=ov_camera_run_rows(tmp_path,ov_camera_resequence(rows))
    assert report['state_export']['count']==1
    assert report['state_export']['ros_delivery']=='NOT_RUN'


@pytest.mark.parametrize('flags', [[0,1,1],[1,0,1],[1,1,0]])
def test_openvins_terminal_skip_retains_unfed_queue_evidence(ov_camera_helper_fixtures,tmp_path,flags):
    rows=ov_camera_terminal_rows(ov_camera_helper_fixtures)
    rows=[r for r in rows if r['kind'] not in {'camera_terminal_ready_drain_enter',
        'camera_terminal_ready_drain_returned','camera_state_export_returned'} and
        not (r.get('camera_consumption',{}).get('occurrence')==2 and r['kind'] in ov_camera_parser.PIPELINE[3:])]
    boundary=next(r for r in rows if r['kind']=='camera_terminal_drain_boundary');boundary['values']=flags
    if not flags[0]:rows=[r for r in rows if r['kind']!='imu_ros_callback']
    if not flags[1]:
        imu=next(r for r in rows if r['kind']=='imu_ros_callback');rows.insert(rows.index(imu),copy.deepcopy(imu))
    queued=copy.deepcopy(next(r for r in rows if r['kind']=='camera_decoded_queue_insert' and r['camera_consumption']['occurrence']==2))
    queued['kind']='camera_queued_not_fed_before_shutdown'
    rows.insert(next(i for i,r in enumerate(rows) if r['kind']=='camera_receipts_closed'),queued)
    for i,row in enumerate(rows):row['monotonic_ns']=100+i
    report=ov_camera_run_rows(tmp_path,ov_camera_resequence(rows))
    assert report['terminal_drain']['attempted'] is False
    assert report['terminal_drain']['completed'] is False
    assert report['terminal_drain']['feed_bundle_count']==0
    assert report['full_source_consumption']=='FAIL'


def test_openvins_returned_drain_does_not_qualify_future_unready_queue(ov_camera_helper_fixtures,tmp_path):
    rows=ov_camera_terminal_rows(ov_camera_helper_fixtures)
    rows=[r for r in rows if not (r.get('camera_consumption',{}).get('occurrence')==2 and r['kind'] in ov_camera_parser.PIPELINE[3:])]
    queued=copy.deepcopy(next(r for r in rows if r['kind']=='camera_decoded_queue_insert' and r['camera_consumption']['occurrence']==2))
    queued['kind']='camera_queued_not_fed_before_shutdown'
    rows.insert(next(i for i,r in enumerate(rows) if r['kind']=='camera_receipts_closed'),queued)
    for i,row in enumerate(rows):row['monotonic_ns']=100+i
    report=ov_camera_run_rows(tmp_path,ov_camera_resequence(rows))
    assert report['terminal_drain']['completed'] is True
    assert report['terminal_drain']['all_pending_consumed']=='NOT_RUN'
    assert report['full_source_consumption']=='FAIL'


@pytest.mark.parametrize('values', [[1,1,0,0,-1],[0,0,1,1,128],[1,0,1,0,-1]])
def test_openvins_export_preserves_unknown_or_shutdown_delivery(ov_camera_helper_fixtures,tmp_path,values):
    rows=ov_camera_terminal_rows(ov_camera_helper_fixtures)
    next(r for r in rows if r['kind']=='camera_state_export_returned')['values']=values
    report=ov_camera_run_rows(tmp_path,rows)
    assert report['state_export']['live_context_call_count']==int(values[0]==values[1]==1)
    assert report['state_export']['ros_delivery']=='NOT_RUN'
    assert report['health_qualification']=='NOT_RUN'


@pytest.mark.parametrize('kind', ['camera_track_frequency_rejected','camera_cv_bridge_rejected',
                                  'camera_cv_bridge_bundle_rejected','camera_receipt_pointer_reused_without_forward'])
def test_openvins_callback_rejection_cannot_follow_executor_quiescence(ov_camera_helper_fixtures,tmp_path,kind):
    rows=copy.deepcopy(ov_camera_helper_fixtures['rejects'])
    if kind in ('camera_cv_bridge_bundle_rejected','camera_receipt_pointer_reused_without_forward'):
        original='camera_cv_bridge_rejected' if kind=='camera_cv_bridge_bundle_rejected' else 'camera_subscription_not_forwarded_before_shutdown'
        rejection=next(r for r in rows if r['kind']==original);rejection['kind']=kind
    else:rejection=next(r for r in rows if r['kind']==kind)
    rows.remove(rejection)
    closing=next(i for i,r in enumerate(rows) if r['kind']=='camera_receipts_closed')
    rows[closing:closing]=[ov_native_event('graceful_executor_callbacks_quiescent',-10**9,values=[1]),
        ov_native_event('camera_terminal_drain_boundary',-10**9,values=[0,1,1]),
        rejection,ov_native_event('initializer_worker_joined',-10**9)]
    for i,row in enumerate(rows):row['monotonic_ns']=100+i
    with pytest.raises(ExperimentError,match='callback after executor quiescence'):
        ov_camera_run_rows(tmp_path,ov_camera_resequence(rows),'rejects')


def openvins_transaction_fixture(tmp_path):
    """Actual source rows with explicit fixture-only sequence compaction.

    This closure excludes unrelated native events. Its constructed footer is
    for decoder unit tests, not evidence of complete source replay/health.
    """
    fixture = json.loads((Path(__file__).parent/'fixtures/openvins_camera_transactions.json').read_text())
    rows, source_to_fixture = [], {}
    for item in fixture['events']:
        raw = item['raw_jsonl']
        assert hashlib.sha256(raw.encode()).hexdigest() == item['raw_jsonl_sha256']
        row = json.loads(raw)
        assert row['sequence'] == item['original_sequence']
        source_to_fixture[row['sequence']] = len(rows)
        row['fixture_original_source_sequence'] = row['sequence']
        row['sequence'] = len(rows)
        rows.append(row)
    rows.append(dict(schema_version=1,kind='observation_closed',event_count=len(rows),
        dropped_events=0,io_failed=False,monotonic_ns=rows[-1]['monotonic_ns']+1))
    path = tmp_path/'actual-camera-transaction-closure.jsonl'
    path.write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in rows))
    return fixture, path, source_to_fixture


def test_openvins_actual_transactions_last_snapshot_support_and_legacy_shape(tmp_path):
    fixture, path, identities = openvins_transaction_fixture(tmp_path)
    legacy = analyze_openvins_observations(path)
    report = analyze_openvins_observations(path,include_camera_transactions=True)
    projection = report.pop('camera_transactions')
    assert report == legacy and 'camera_transactions' not in legacy
    assert projection['status'] == 'PASS'
    assert projection['transaction_count'] == 4
    assert projection['accepted_transaction_count'] == projection['visual_time_advance_transaction_count'] == 3
    assert projection['unbound_accepted_update_count'] == projection['unbound_returned_state_count'] == 0
    assert projection['source']['sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert fixture['source']['run_sha256'] == '25d0a6debae38994ed765753c1dcd8b9013d2a6fea0cc6f57927e267fe57dc55'
    for row, expected in zip(projection['records'],fixture['expected_transactions']):
        assert row['begin_event_sequence'] == identities[expected['begin_original_sequence']]
        assert row['end_event_sequence'] == identities[expected['end_original_sequence']]
        assert row['accepted_update_count'] == expected['accepted_update_count']
        assert row['merged_measurement_support'] == expected['merged_measurement_support']
        assert row['camera_ids'] == [0]
        assert row['tracking_state'] == 'UNKNOWN' and row['validity'] is False and row['contract_complete'] is False
        assert all(row[key] is None for key in ('native_epoch','native_reset_count','covariance',
            'native_publication_timestamp','sensor_to_native_clock_mapping'))
        if not expected['accepted_update_count']:
            assert row['last_accepted_snapshot'] is None
            assert row['state_source'] == 'NO_ACCEPTED_VISUAL_UPDATE'
            assert row['last_visual_measurement_ns'] is None and not row['advances_visual_measurement_time']
            continue
        state = row['last_accepted_snapshot']
        assert state['source_event_sequence'] == identities[expected['last_accepted_original_sequence']]
        assert row['returned_state_matches_last_accepted_snapshot'] is True
        assert (state['parent_frame'],state['child_frame'],state['velocity_frame']) == ('backend_world','imu','backend_world')
        assert (state['translation_unit'],state['velocity_unit']) == ('m','m/s')
        assert state['velocity_world_mps'] == expected['native_state']['velocity_world_mps']
        assert any(value != 0 for value in state['velocity_world_mps'])
        assert state['gyro_bias_radps'] == expected['native_state']['gyro_bias_radps']
        assert state['accel_bias_mps2'] == expected['native_state']['accel_bias_mps2']
        wire = expected['actual_ros_pose']
        x,y,z,w = wire['quaternion_xyzw']
        assert state['orientation_wxyz'] == [w,x,y,z]
        assert state['position_world_m'] == wire['position']
        assert state['orientation_wxyz'] != [1.,0.,0.,0.]
    assert [row['accepted_update_count'] for row in projection['records']] == [0,1,41,3]
    json.dumps(projection,allow_nan=False)


def test_openvins_transaction_old_support_and_nonvisual_states_never_refresh(tmp_path):
    first_ns, later_ns, stale_ns = CAMERAS[2], CAMERAS[3], CAMERAS[4]+10_000_000_000
    values = ov_state_values()
    rows = ov_visual_rows()[:3]
    def boundary(kind, ns):
        return dict(ov_native_event(kind,ns),sensor_ids=[0,1])
    rows += [boundary('camera_processing_begin',first_ns),ov_native_event('msckf_visual_ekf_completed',first_ns,
        values=values,support=ov_support((0,BASE),(1,CAMERAS[1]))),boundary('camera_processing_returned',first_ns),
        ov_native_event('camera_returned_state',first_ns,values=values),
        boundary('camera_processing_begin',later_ns),ov_native_event('landmark_visual_ekf_completed',later_ns,
        values=values,support=ov_support((0,BASE))),boundary('camera_processing_returned',later_ns),
        ov_native_event('camera_returned_state',later_ns,values=values),
        boundary('camera_processing_begin',stale_ns),ov_native_event('nonvisual_zupt_ekf_completed',stale_ns,values=values),
        ov_native_event('imu_state_propagation_completed',stale_ns,values=values),
        boundary('camera_processing_returned',stale_ns),ov_native_event('camera_returned_state',stale_ns,values=values),
        ov_native_event('independent_observation_tick')]
    report = analyze_openvins_observations(ov_trace(tmp_path,rows),include_camera_transactions=True)
    projection = report['camera_transactions']
    a,b,c = projection['records']
    assert projection['transaction_count'] == 3  # stereo IDs stay one CameraData transaction
    assert [row['camera_ids'] for row in projection['records']] == [[0,1]]*3
    assert [row['advances_visual_measurement_time'] for row in projection['records']] == [True,False,False]
    assert [row['last_visual_measurement_ns'] for row in projection['records']] == [CAMERAS[1]]*3
    assert a['last_accepted_snapshot'] is not None and b['last_accepted_snapshot'] is not None
    assert c['last_accepted_snapshot'] is None and c['accepted_update_count'] == 0
    assert all(not row['validity'] and row['tracking_state'] == 'UNKNOWN' for row in projection['records'])
    assert projection['health_qualification'] == projection['automatic_reset'] == 'NOT_RUN'
    json.dumps(projection,allow_nan=False)


@pytest.mark.parametrize('defect',['nested','missing_return','return_without_begin','time','ids','update_time','state_before_return','double_returned'])
def test_openvins_transaction_boundaries_are_observed_not_guessed(tmp_path,defect):
    rows = ov_visual_rows()[:3]
    begin = dict(ov_native_event('camera_processing_begin',CAMERAS[2]),sensor_ids=[0])
    update = ov_native_event('msckf_visual_ekf_completed',CAMERAS[2],values=ov_state_values(),support=ov_support((0,BASE)))
    end = dict(ov_native_event('camera_processing_returned',CAMERAS[2]),sensor_ids=[0])
    returned = ov_native_event('camera_returned_state',CAMERAS[2],values=ov_state_values())
    body = [begin,update,end,returned]
    if defect == 'nested':body.insert(1,copy.deepcopy(begin))
    elif defect == 'missing_return':body=[begin,update]
    elif defect == 'return_without_begin':body=[update,end,returned]
    elif defect == 'time':
        other=ov_native_event('x',BASE)
        end.update(sensor_time_seconds=other['sensor_time_seconds'],sensor_time_double_bits=other['sensor_time_double_bits'])
    elif defect == 'ids':end['sensor_ids']=[1]
    elif defect == 'update_time':
        other=ov_native_event('x',CAMERAS[3])
        update.update(sensor_time_seconds=other['sensor_time_seconds'],sensor_time_double_bits=other['sensor_time_double_bits'])
    elif defect == 'state_before_return':body=[begin,update,returned,end]
    else:body.append(copy.deepcopy(returned))
    with pytest.raises(ValueError,match='camera|accepted update|returned state'):
        analyze_openvins_observations(ov_trace(tmp_path,rows+body),include_camera_transactions=True)


def test_openvins_transaction_absent_boundaries_remain_not_run(tmp_path):
    report=analyze_openvins_observations(ov_trace(tmp_path,ov_visual_rows()),include_camera_transactions=True)
    projection=report['camera_transactions']
    assert projection['status']=='NOT_RUN' and projection['records']==[]
    assert projection['unbound_accepted_update_count']==1
    assert report['health_qualification']==report['automatic_reset']=='NOT_RUN'


@pytest.mark.parametrize('difference', ['time', 'velocity'])
def test_openvins_transaction_returned_state_mismatch_is_only_diagnostic(tmp_path, difference):
    values = ov_state_values()
    returned_values = values.copy()
    returned_time = CAMERAS[2]
    if difference == 'time':
        returned_time = CAMERAS[3]
    else:
        returned_values[9] += 1  # Native world velocity, not a visual observation.
    rows = ov_visual_rows()[:3] + [
        dict(ov_native_event('camera_processing_begin', CAMERAS[2]), sensor_ids=[0]),
        ov_native_event('msckf_visual_ekf_completed', CAMERAS[2], values=values,
                        support=ov_support((0, BASE))),
        dict(ov_native_event('camera_processing_returned', CAMERAS[2]), sensor_ids=[0]),
        ov_native_event('camera_returned_state', returned_time, values=returned_values),
    ]
    report = analyze_openvins_observations(ov_trace(tmp_path, rows), include_camera_transactions=True)
    projection = report['camera_transactions']
    transaction = projection['records'][0]
    assert projection['status'] == 'PASS'  # Boundary projection only; mismatch remains explicit.
    assert transaction['returned_state_matches_last_accepted_snapshot'] is False
    assert transaction['last_accepted_snapshot']['velocity_world_mps'] == [.1, .2, .3]
    assert transaction['last_visual_measurement_ns'] == BASE
    assert not transaction['validity'] and transaction['tracking_state'] == 'UNKNOWN'
    assert projection['health_qualification'] == projection['automatic_reset'] == 'NOT_RUN'


@pytest.mark.parametrize('value',[None,float('nan'),float('inf')])
def test_openvins_transaction_nonfinite_velocity_is_never_projected(tmp_path,value):
    rows=ov_visual_rows()[:3]
    state=ov_state_values();state[1+0+8]=value
    rows += [dict(ov_native_event('camera_processing_begin',CAMERAS[2]),sensor_ids=[0]),
        ov_native_event('msckf_visual_ekf_completed',CAMERAS[2],values=state,support=ov_support((0,BASE))),
        dict(ov_native_event('camera_processing_returned',CAMERAS[2]),sensor_ids=[0])]
    with pytest.raises(ValueError):
        analyze_openvins_observations(ov_trace(tmp_path,rows),include_camera_transactions=True)


def ov_health_rows(*, origin=BASE, request_delta_ns=50_000_000):
    """Constructed wire evidence for decoder tests, never native-run evidence."""
    rows = []
    def add(kind, elapsed_ms, ns=None, *, values=None, camera=None, support=None):
        ns = origin if ns is None else ns
        row = ov_native_event(kind, ns, values=values, camera=camera, support=support)
        row['monotonic_ns'] = elapsed_ms*1_000_000
        rows.append(row)
        return row
    add('camera_ros_callback', 0, camera=0)
    add('imu_ros_callback', 1, origin+request_delta_ns-5_000_000, camera=0, values=[0]*6)
    add('metric_initialization_committed', 2, values=ov_state_values())
    add('camera_processing_begin', 3)['sensor_ids'] = [0]
    add('msckf_visual_ekf_completed', 4, values=ov_state_values(), support=ov_support((0, origin)))
    add('camera_processing_returned', 5)['sensor_ids'] = [0]
    add('camera_returned_state', 6, values=[1, 1.0, *ov_state_values()[1:]])
    request = add('imu_ros_callback', 50, origin+request_delta_ns, camera=0, values=[0]*6)
    initial_time = ov_native_event('unused', origin)['sensor_time_seconds']
    add('imu_fast_prediction_completed', 51, origin+request_delta_ns,
        values=[initial_time+.003, request['sensor_time_seconds']+.003,
                initial_time, initial_time, .003, 1, 1])
    add('independent_observation_tick', 52, -1_000_000_000)
    return rows


def ov_health_run(tmp_path, rows, *, name='health', session='TEST_FIXTURE_ONLY'):
    from tools.openvins.native_observations import qualify_state_freshness
    for index, row in enumerate(rows):
        row['sequence'] = index
    footer = dict(schema_version=1, kind='observation_closed', event_count=len(rows),
        monotonic_ns=rows[-1]['monotonic_ns']+1, dropped_events=0, io_failed=False)
    path = trace(tmp_path, rows+[footer])
    target = tmp_path/(name+'.jsonl')
    result = qualify_state_freshness(path, target,
        policy=dict(max_visual_age_ns=300_000_000, max_imu_gap_ns=20_000_000,
            max_receive_age_ns=100_000_000, max_visual_receive_age_ns=600_000_000,
            max_buffer_samples=100), session_id=session, sensor_clock='EuRoC:TEST_FIXTURE_ONLY:sensor_time')
    return result, [json.loads(line) for line in target.read_text().splitlines()]


def test_openvins_health_complete_evidence_is_current_but_operationally_unknown(tmp_path):
    report, timeline = ov_health_run(tmp_path, ov_health_rows())
    assert report['status'] == 'PASS'
    assert report['initialization']['commit_observed'] and report['initialization']['ready_observed']
    assert report['final']['freshness'] == 'CURRENT'
    assert all(row['tracking_state'] == 'UNKNOWN' and row['valid'] is False
               and row['native_epoch'] is None and row['native_reset_count'] is None for row in timeline)
    assert report['operational_health'] == 'UNKNOWN' and report['operational_valid'] is False
    assert report['native_reset']['status'] == 'BLOCKED'
    assert report['covariance']['status'] == 'UNQUALIFIED'
    assert report['truth_used'] is False


@pytest.mark.parametrize('cache_offset', [0., -.003, .009])
def test_openvins_health_direct_cache_witness_can_differ_from_latest_serialized_state(tmp_path, cache_offset):
    rows = ov_health_rows()
    prediction = next(row for row in rows if row['kind']=='imu_fast_prediction_completed')
    cache_time = ov_native_event('unused')['sensor_time_seconds']
    request_time = prediction['sensor_time_seconds']
    # The concurrently updated main filter is not the fast-cache snapshot.
    prediction['values'] = [cache_time+cache_offset, request_time+cache_offset,
                            cache_time+.025, cache_time, cache_offset, 1, 1]
    report, timeline = ov_health_run(tmp_path, rows)
    assert report['status'] == 'PASS' and timeline[-1]['freshness'] == 'CURRENT'
    assert timeline[-1]['state_time']['seconds'] == request_time+cache_offset
    assert timeline[-1]['last_visual_sensor_ns'] == BASE
    assert timeline[-1]['tracking_state'] == 'UNKNOWN' and timeline[-1]['valid'] is False


def test_openvins_health_historic_interleaved_dt_false_positive_has_exact_cache_equations(tmp_path):
    """Use retained failed-run numbers with an explicitly constructed v2 witness.

    The old event 72249 did not record its exact cache offset. This test does
    not retrospectively claim that missing evidence: it supplies a compatible
    cache witness to reproduce the false assumption about event 72248's dt.
    """
    camera_ns = 1403715329312142976
    rows = ov_health_rows(origin=camera_ns, request_delta_ns=10_000_000)
    latest_dt = .00018745826338403484
    for row in rows:
        if row['kind'] in ('metric_initialization_committed', 'msckf_visual_ekf_completed', 'camera_returned_state'):
            row['values'][1+int(row['values'][0])] = latest_dt
    prediction = next(row for row in rows if row['kind']=='imu_fast_prediction_completed')
    request_time = prediction['sensor_time_seconds']
    assert request_time == 1403715329.322143
    cache_time = 1403715329.312143
    start, end = 1403715329.3123295, 1403715329.3223295
    compatible_cache_offset = end-request_time
    assert cache_time+compatible_cache_offset == start
    assert request_time+compatible_cache_offset == end
    assert request_time+latest_dt != end
    prediction['values'] = [start, end, cache_time, cache_time, compatible_cache_offset, 1, 1]
    report, timeline = ov_health_run(tmp_path, rows)
    assert report['status'] == 'PASS' and timeline[-1]['freshness'] == 'CURRENT'
    assert timeline[-1]['state_time']['seconds'] == end


def test_openvins_health_watchdog_preserves_all_actual_state_and_sensor_times(tmp_path):
    rows = ov_health_rows()
    tick = ov_native_event('independent_observation_tick', -1_000_000_000)
    tick['monotonic_ns'] = 300_000_000
    rows.append(tick)
    _, timeline = ov_health_run(tmp_path, rows)
    before, after = timeline[-2:]
    assert before['freshness'] == 'CURRENT' and after['freshness'] == 'STALE'
    for field in ('state_time', 'state_source', 'last_sensor_observation_ns',
                  'last_state_progress_observation_ns', 'last_visual_observation_ns', 'last_visual_sensor_ns'):
        assert before[field] == after[field]
    assert set(after['freshness_reasons']) == {'SENSOR_STREAM_STOPPED', 'IMU_STREAM_STOPPED', 'STATE_PROGRESS_STOPPED'}


def test_openvins_health_camera_receipt_cannot_hide_stopped_imu(tmp_path):
    rows = ov_health_rows()
    camera = ov_native_event('camera_ros_callback', BASE+200_000_000, camera=0)
    camera['monotonic_ns'] = 300_000_000
    tick = ov_native_event('independent_observation_tick', -1_000_000_000)
    tick['monotonic_ns'] = 301_000_000
    rows += [camera, tick]
    _, timeline = ov_health_run(tmp_path, rows)
    final = timeline[-1]
    assert final['freshness'] == 'STALE'
    assert 'IMU_STREAM_STOPPED' in final['freshness_reasons']
    assert 'SENSOR_STREAM_STOPPED' not in final['freshness_reasons']
    assert final['last_sensor_observation_ns'] == 300_000_000
    assert final['last_state_progress_observation_ns'] == 51_000_000


def test_openvins_health_repeated_prediction_of_same_sample_cannot_refresh_progress(tmp_path):
    rows = ov_health_rows()
    repeated = copy.deepcopy(next(row for row in rows if row['kind']=='imu_fast_prediction_completed'))
    repeated['monotonic_ns'] = 200_000_000
    rows.append(repeated)
    _, timeline = ov_health_run(tmp_path, rows)
    assert timeline[-1]['last_state_progress_observation_ns'] == 51_000_000
    assert timeline[-1]['state_time'] == timeline[-2]['state_time']
    assert timeline[-1]['last_visual_sensor_ns'] == BASE
    assert timeline[-1]['freshness'] == 'STALE'


def test_openvins_health_prediction_only_never_refreshes_visual_time(tmp_path):
    rows = ov_health_rows()
    callback = ov_native_event('imu_ros_callback', BASE+400_000_000, camera=0, values=[0]*6)
    callback['monotonic_ns'] = 400_000_000
    request = callback['sensor_time_seconds']
    prediction = ov_native_event('imu_fast_prediction_completed', BASE+400_000_000,
        values=[request-.01, request, ov_native_event('unused')['sensor_time_seconds'], request-.01, 0., 1, 1])
    prediction['monotonic_ns'] = 401_000_000
    rows += [callback, prediction]
    _, timeline = ov_health_run(tmp_path, rows)
    last = timeline[-1]
    assert last['state_source'] == 'IMU_PREDICTION'
    assert last['last_state_progress_observation_ns'] == 401_000_000
    assert last['last_visual_observation_ns'] == 4_000_000
    assert last['last_visual_sensor_ns'] == BASE
    assert last['freshness'] == 'STALE' and 'VISUAL_MEASUREMENT_STALE' in last['freshness_reasons']


def test_openvins_health_reused_old_visual_support_cannot_revive_stale_state(tmp_path):
    rows = ov_health_rows()
    for kind, when in [('camera_ros_callback', 500), ('camera_processing_begin', 501),
                       ('msckf_visual_ekf_completed', 502), ('camera_processing_returned', 503),
                       ('camera_returned_state', 504)]:
        event = ov_native_event(kind, BASE+500_000_000, camera=0 if kind=='camera_ros_callback' else None,
            values=([1, 1.0, *ov_state_values()[1:]] if kind=='camera_returned_state' else
                    ov_state_values() if kind=='msckf_visual_ekf_completed' else None),
            support=ov_support((0, BASE)) if kind=='msckf_visual_ekf_completed' else None)
        if kind in ('camera_processing_begin', 'camera_processing_returned'):
            event['sensor_ids'] = [0]
        event['monotonic_ns'] = when*1_000_000
        rows.append(event)
    _, timeline = ov_health_run(tmp_path, rows)
    final = timeline[-1]
    assert final['last_visual_sensor_ns'] == BASE and final['last_visual_observation_ns'] == 4_000_000
    assert final['last_state_progress_observation_ns'] == 51_000_000
    assert final['freshness'] == 'STALE' and 'STATE_PROGRESS_STOPPED' in final['freshness_reasons']


@pytest.mark.parametrize('failure', ['shape', 'bad_mean', 'bad_covariance', 'unassociated', 'future_endpoint',
    'negative_start', 'cache_start_mismatch', 'cache_end_mismatch', 'legacy_unwitnessed_payload'])
def test_openvins_health_rejects_malformed_or_unassociated_native_prediction(tmp_path, failure):
    rows = ov_health_rows()
    prediction = next(row for row in rows if row['kind']=='imu_fast_prediction_completed')
    if failure == 'shape': prediction['values'].pop()
    elif failure == 'bad_mean': prediction['values'][5] = 0
    elif failure == 'bad_covariance': prediction['values'][6] = 0
    elif failure == 'unassociated': rows.remove(next(row for row in rows if row['kind']=='imu_ros_callback'
                                                    and row['raw_nsec'] != (BASE+45_000_000)%10**9))
    elif failure == 'future_endpoint': prediction['values'][1] += 100
    elif failure == 'negative_start': prediction['values'][0] = -1
    elif failure == 'cache_start_mismatch': prediction['values'][3] += .001
    elif failure == 'cache_end_mismatch': prediction['values'][4] += .001
    elif failure == 'legacy_unwitnessed_payload': prediction['values'] = prediction['values'][:3]+prediction['values'][5:]
    with pytest.raises(ExperimentError):
        ov_health_run(tmp_path, rows)


@pytest.mark.parametrize('cache_ahead_of_request', [False, True])
def test_openvins_health_unavailable_prediction_does_not_refresh_progress_or_claim_health(tmp_path, cache_ahead_of_request):
    rows = ov_health_rows()
    initial_time = ov_native_event('unused')['sensor_time_seconds']
    request_time = next(row['sensor_time_seconds'] for row in rows if row['kind']=='imu_fast_prediction_completed')
    cache_time = request_time+.01 if cache_ahead_of_request else initial_time
    unavailable = ov_native_event('imu_fast_prediction_unavailable', BASE+50_000_000,
        values=[cache_time, request_time, initial_time, cache_time, 0.])
    unavailable['monotonic_ns'] = 200_000_000
    rows.append(unavailable)
    _, timeline = ov_health_run(tmp_path, rows)
    assert timeline[-1]['last_state_progress_observation_ns'] == timeline[-2]['last_state_progress_observation_ns']
    assert timeline[-1]['state_time'] == timeline[-2]['state_time']
    assert timeline[-1]['freshness'] == 'STALE'
    assert not timeline[-1]['valid'] and timeline[-1]['operational_health'] == 'UNKNOWN'


@pytest.mark.parametrize('failure', ['missing_ready', 'missing_prediction', 'missing_transaction'])
def test_openvins_health_missing_required_evidence_cannot_pass(tmp_path, failure):
    rows = ov_health_rows()
    remove = {'missing_ready': {'camera_returned_state'}, 'missing_prediction': {'imu_fast_prediction_completed'},
              'missing_transaction': {'camera_processing_begin', 'camera_processing_returned', 'camera_returned_state'}}[failure]
    rows = [row for row in rows if row['kind'] not in remove]
    try:
        report, _ = ov_health_run(tmp_path, rows)
    except ExperimentError:
        return
    assert report['status'] != 'PASS'


@pytest.mark.parametrize('kind', ['reset', 'estimator_reset', 'reinitialization', 'metric_initialization_committed'])
def test_openvins_health_reset_requires_real_supported_native_lifecycle(tmp_path, kind):
    rows = ov_health_rows()
    event = ov_native_event(kind, values=ov_state_values() if kind=='metric_initialization_committed' else None)
    event['monotonic_ns'] = 60_000_000
    rows.append(event)
    with pytest.raises(ExperimentError, match='OPENVINS_HEALTH_EPOCH_UNOBSERVED'):
        ov_health_run(tmp_path, rows)


def test_openvins_health_new_process_session_is_not_a_native_reset_counter(tmp_path):
    first, _ = ov_health_run(tmp_path, ov_health_rows(), name='first', session='FIRST_TEST_SESSION')
    second, _ = ov_health_run(tmp_path, ov_health_rows(), name='second', session='SECOND_TEST_SESSION')
    assert first['session_id'] != second['session_id']
    for result in (first, second):
        assert result['final']['adapter_epoch'] == 0
        assert result['final']['native_epoch'] is result['final']['native_reset_count'] is None
        assert result['native_reset']['execution'] == 'NOT_RUN'


def openvins_laboratory_fixture(tmp_path, monkeypatch):
    """Constructed sealed files; only MCAP I/O is replaced, never the decoder."""
    from tools.openvins import native_observations as native
    from rm27.perception.vision.localization.experiments.openvins import PIN
    rows = ov_health_rows()
    callback = ov_native_event('imu_ros_callback', BASE+400_000_000, camera=0, values=[0]*6)
    callback['monotonic_ns'] = 400_000_000
    request = callback['sensor_time_seconds']
    prediction = ov_native_event('imu_fast_prediction_completed', BASE+400_000_000,
        values=[request-.01, request, request-.01, request-.01, 0., 1, 1])
    prediction['monotonic_ns'] = 401_000_000
    rows += [callback, prediction]
    health, timeline = ov_health_run(tmp_path, rows, session='LAB_UNIT_FIXTURE_NOT_NATIVE')
    (tmp_path/'raw/online-output').mkdir(parents=True)
    (tmp_path/'online').mkdir()
    (tmp_path/'native-events.jsonl').rename(tmp_path/'raw/native-events.jsonl')
    (tmp_path/'health.jsonl').rename(tmp_path/'online/native-health.jsonl')
    health['native_event_source']['path'] = str(tmp_path/'raw/native-events.jsonl')
    (tmp_path/'native-health.json').write_text(json.dumps(health, allow_nan=False))
    (tmp_path/'raw/online-output/metadata.yaml').write_text('unit-fixture: not a real MCAP\n')
    (tmp_path/'raw/online-output/online-output_0.mcap').write_bytes(b'MCAP I/O replaced by test adapter')
    messages = []
    for index, event in enumerate(row for row in rows if row['kind']=='imu_fast_prediction_completed'):
        messages.append(dict(state_ns=native._openvins_ros_header_ns(event['sensor_time_seconds']),
            frame_id='global', child_frame_id='imu', position=[1., 2., 3.],
            quaternion_xyzw=[0., 0., 0., 1.], velocity_local=[.1, .2, .3],
            angular_velocity_local=[.01, .02, .03], cdr_sha256='0'*64, bag_record_timestamp_ns=100+index))
    observed = [{key:value for key, value in row.items() if key in
                 ('state_ns', 'frame_id', 'child_frame_id', 'position', 'quaternion_xyzw', 'velocity_local')}
                for row in messages]
    (tmp_path/'raw/odom.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in observed))
    manifest = dict(status='EXECUTION_COMPLETED', exit_code=0,
        config=dict(backend='OpenVINS', run_id='LAB_UNIT_FIXTURE_NOT_NATIVE'),
        installation=dict(commit=PIN, binary_sha256='1'*64, native_health_observation=dict(schema_version=2)))
    def seal():
        manifest['output_sha256'] = {str(path.relative_to(tmp_path)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in tmp_path.rglob('*') if path.is_file() and path.name != 'run.json'}
        (tmp_path/'run.json').write_text(json.dumps(manifest, allow_nan=False))
    seal()
    monkeypatch.setattr(native, '_openvins_mcap_odometry', lambda path: iter(copy.deepcopy(messages)))
    return native, rows, timeline, messages, manifest, seal


def test_openvins_laboratory_extracts_coherent_message_and_keeps_unknown_health(tmp_path, monkeypatch):
    native, rows, timeline, messages, manifest, seal = openvins_laboratory_fixture(tmp_path, monkeypatch)
    first, after_tick, stale = native.load_openvins_laboratory_samples(tmp_path, event_sequences=[6, 9, 11])
    assert [row['selected_event_sequence'] for row in (first, after_tick, stale)] == [8, 11, 11]
    assert first['odometry'] == {key:value for key, value in messages[0].items()
                                 if key not in ('cdr_sha256', 'bag_record_timestamp_ns')}
    assert first['odometry']['angular_velocity_local'] == [.01, .02, .03]
    assert first['health_row']['freshness'] == 'CURRENT' and stale['health_row']['freshness'] == 'STALE'
    assert first['source']['native_prediction']['integration_endpoint_seconds'] != rows[8]['sensor_time_seconds']
    assert first['odometry']['state_ns'] == native._openvins_ros_header_ns(rows[8]['sensor_time_seconds'])
    for sample in (first, after_tick, stale):
        source = sample['source']
        assert sample['health_row']['valid'] is source['operational_valid'] is False
        assert source['clock_mapping'] is source['native_epoch'] is source['native_reset_count'] is None
        assert source['tracking_health'] == 'UNKNOWN' and source['covariance_qualification'] == 'UNQUALIFIED'
        assert source['truth_used'] is False
        assert sample['accepted_visual_support']['last_visual_measurement_ns'] == BASE
    assert first['source']['mcap_odometry_ordinal'] == 0 and stale['source']['mcap_odometry_ordinal'] == 1
    json.dumps([first, after_tick, stale], allow_nan=False)


@pytest.mark.parametrize('selectors', [[], [True], [-1], [6, 6], [6]*17, [7], [12], [999]])
def test_openvins_laboratory_rejects_ambiguous_absent_or_nonhealth_selectors(tmp_path, monkeypatch, selectors):
    native, *_ = openvins_laboratory_fixture(tmp_path, monkeypatch)
    with pytest.raises(ExperimentError, match='OPENVINS_LAB_SOURCE_INVALID'):
        native.load_openvins_laboratory_samples(tmp_path, event_sequences=selectors)


@pytest.mark.parametrize('name', ['raw/native-events.jsonl', 'native-health.json',
    'online/native-health.jsonl', 'raw/odom.jsonl', 'raw/online-output/metadata.yaml',
    'raw/online-output/online-output_0.mcap'])
def test_openvins_laboratory_checks_every_needed_sealed_hash(tmp_path, monkeypatch, name):
    native, *_ = openvins_laboratory_fixture(tmp_path, monkeypatch)
    with (tmp_path/name).open('ab') as stream:
        stream.write(b'changed')
    with pytest.raises(ExperimentError, match='sealed artifact differs'):
        native.load_openvins_laboratory_samples(tmp_path, event_sequences=[6])


@pytest.mark.parametrize('failure', ['duplicate_header', 'missing_message', 'endpoint_as_header',
    'wrong_frame', 'wrong_child', 'wrong_velocity', 'missing_rate', 'nonfinite_rate', 'nonunit_quaternion'])
def test_openvins_laboratory_never_repairs_or_joins_wrong_messages(tmp_path, monkeypatch, failure):
    native, rows, timeline, messages, manifest, seal = openvins_laboratory_fixture(tmp_path, monkeypatch)
    if failure == 'duplicate_header': messages.append(copy.deepcopy(messages[0]))
    elif failure == 'missing_message': messages.pop()
    elif failure == 'endpoint_as_header': messages[0]['state_ns'] = native._openvins_ros_header_ns(rows[8]['values'][1])
    elif failure == 'wrong_frame': messages[0]['frame_id'] = 'NED'
    elif failure == 'wrong_child': messages[0]['child_frame_id'] = 'BODY_FRD'
    elif failure == 'wrong_velocity': messages[0]['velocity_local'][0] += .1
    elif failure == 'missing_rate': del messages[0]['angular_velocity_local']
    elif failure == 'nonfinite_rate': messages[0]['angular_velocity_local'][0] = float('nan')
    elif failure == 'nonunit_quaternion': messages[0]['quaternion_xyzw'][3] = 2.
    with pytest.raises((ExperimentError, ValueError)):
        native.load_openvins_laboratory_samples(tmp_path, event_sequences=[6])


@pytest.mark.parametrize('failure', ['not_completed', 'nonzero_exit', 'unwitnessed_native',
    'wrong_commit', 'false_session', 'valid_upgrade', 'epoch_upgrade', 'wrong_visual_time', 'wrong_endpoint'])
def test_openvins_laboratory_rejects_unqualified_source_or_forged_health(tmp_path, monkeypatch, failure):
    native, rows, timeline, messages, manifest, seal = openvins_laboratory_fixture(tmp_path, monkeypatch)
    if failure == 'not_completed': manifest['status'] = 'FAILED'
    elif failure == 'nonzero_exit': manifest['exit_code'] = 1
    elif failure == 'unwitnessed_native': manifest['installation']['native_health_observation']['schema_version'] = 1
    elif failure == 'wrong_commit': manifest['installation']['commit'] = '0'*40
    else:
        selected = next(row for row in timeline if row['native_event_sequence'] == 8)
        if failure == 'false_session': selected['session_id'] = 'other'
        elif failure == 'valid_upgrade': selected['valid'] = True
        elif failure == 'epoch_upgrade': selected['native_epoch'] = 0
        elif failure == 'wrong_visual_time': selected['last_visual_sensor_ns'] += 1
        elif failure == 'wrong_endpoint': selected['state_time']['double_bits'] += 1
        path = tmp_path/'online/native-health.jsonl'
        path.write_text(''.join(json.dumps(row)+'\n' for row in timeline))
        health = json.loads((tmp_path/'native-health.json').read_text())
        health['timeline']['sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        (tmp_path/'native-health.json').write_text(json.dumps(health))
    seal()
    with pytest.raises(ExperimentError, match='OPENVINS_LAB_SOURCE_INVALID'):
        native.load_openvins_laboratory_samples(tmp_path, event_sequences=[6])


def test_openvins_laboratory_detects_source_changed_while_mcap_is_read(tmp_path, monkeypatch):
    native, rows, timeline, messages, manifest, seal = openvins_laboratory_fixture(tmp_path, monkeypatch)
    def changed(path):
        yield from messages
        with (tmp_path/'raw/online-output/online-output_0.mcap').open('ab') as stream:
            stream.write(b'changed during read')
    monkeypatch.setattr(native, '_openvins_mcap_odometry', changed)
    with pytest.raises(ExperimentError, match='sealed artifact differs'):
        native.load_openvins_laboratory_samples(tmp_path, event_sequences=[6])


def test_openvins_request_header_conversion_uses_native_half_away_rounding_and_carry():
    from tools.openvins.native_observations import _openvins_ros_header_ns
    assert _openvins_ros_header_ns(.5e-9) == 1
    assert _openvins_ros_header_ns(1.9999999996) == 2_000_000_000
    assert _openvins_ros_header_ns(1403715278.872143) == 1403715278872143030
    assert _openvins_ros_header_ns(1403715278.871349) != 1403715278872143030
    for value in (True, -1, float('nan'), 2**32):
        with pytest.raises(ExperimentError, match='OPENVINS_LAB_TIME_INVALID'):
            _openvins_ros_header_ns(value)


def test_openvins_full_marginal_numeric_checks_never_qualify_consistency():
    import numpy as np
    from tools.openvins.native_observations import inspect_openvins_imu_covariance

    covariance = np.diag(np.linspace(.01, .15, 15))
    covariance[0, 3] = covariance[3, 0] = .005
    original = covariance.copy()
    result = inspect_openvins_imu_covariance(covariance.ravel().tolist())
    assert result['numerical_status'] == 'PASS' and result['symmetric_part_rank'] == 15
    assert result['statistical_consistency'] == result['flight_covariance'] == 'UNQUALIFIED'
    assert result['covariance_modified'] is False
    assert 'THETA_IMU_POSITION_GLOBAL' in result['error_chart']
    assert np.array_equal(covariance, original)
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('failure', ['positive_diagonal_indefinite', 'asymmetric', 'negative_diagonal',
    'short', 'bool', 'null', 'infinity', 'enormous_integer'])
def test_openvins_covariance_rejects_bad_matrices_without_repair(failure):
    import numpy as np
    from tools.openvins.native_observations import inspect_openvins_imu_covariance

    covariance = np.eye(15)
    if failure == 'positive_diagonal_indefinite':
        covariance[0, 1] = covariance[1, 0] = 2
    elif failure == 'asymmetric':
        covariance[0, 1] = .01
    elif failure == 'negative_diagonal':
        covariance[0, 0] = -.01
    values = covariance.ravel().tolist()
    if failure == 'short': values.pop()
    elif failure == 'bool': values[0] = True
    elif failure == 'null': values[0] = None
    elif failure == 'infinity': values[0] = float('inf')
    elif failure == 'enormous_integer': values[0] = 10**400
    before = copy.deepcopy(values)
    result = inspect_openvins_imu_covariance(values)
    assert result['numerical_status'] == 'FAIL' and result['reasons']
    assert values == before and result['covariance_modified'] is False
    assert result['statistical_consistency'] == 'UNQUALIFIED'
    json.dumps(result, allow_nan=False)


def test_openvins_covariance_does_not_fill_unobserved_directions_or_hide_asymmetry():
    from tools.openvins.native_observations import inspect_openvins_imu_covariance

    zero = inspect_openvins_imu_covariance([0.]*225)
    assert zero['numerical_status'] == 'PASS' and zero['symmetric_part_rank'] == 0
    assert zero['roundoff_tolerance'] == 0
    assert zero['flight_covariance'] == 'UNQUALIFIED'
    values = [0.]*225
    for index in range(15): values[15*index+index] = 1.
    values[1] = .01
    result = inspect_openvins_imu_covariance(values)
    assert result['minimum_symmetric_part_eigenvalue'] > 0
    assert result['numerical_status'] == 'FAIL'
    assert result['reasons'] == ['NATIVE_COVARIANCE_NOT_SYMMETRIC']


def ov_quality_rows():
    """Constructed second native lane; must be paired to ov_health_rows()."""
    native = ov_native_event('unused')
    common = dict(schema_version=1, state_time_seconds=native['sensor_time_seconds'],
                  state_time_double_bits=native['sensor_time_double_bits'])
    batch = dict(common, kind='msckf_gate_batch', sequence=0, monotonic_ns=4_500_000,
        input_feature_count=3, gate_count=3, accepted_count=2, rejected_count=1,
        nonfinite_count=0, entries_dropped=0, completed_ekf=True,
        gates=[[10, 1., 3, 3, 7.815, 1., False, True, True],
               [12, 2., 3, 3, 7.815, 1., False, True, True],
               [11, 10., 3, 3, 7.815, 1., True, True, True]])
    covariance = [float(i==j) for i in range(15) for j in range(15)]
    snapshot = dict(common, kind='camera_state_snapshot', sequence=1, monotonic_ns=6_500_000,
        camera_time_seconds=native['sensor_time_seconds'], camera_time_double_bits=native['sensor_time_double_bits'],
        camera_ids=[0], camera_imu_offset_seconds=.003, mean=ov_state_values()[2:],
        covariance_row_major=covariance, mean_all_finite=True, covariance_all_finite=True)
    return [batch,snapshot]


def write_ov_quality(path, rows, **footer_changes):
    encoded = ''.join(json.dumps(row,allow_nan=False)+'\n' for row in rows)
    footer = dict(schema_version=1, kind='quality_closed', row_count=len(rows), dropped_rows=0,
        bytes_before_footer=len(encoded.encode()), io_failed=False,
        monotonic_ns=max([r['monotonic_ns'] for r in rows]+[0])+1)
    footer.update(footer_changes)
    path.write_text(encoded+json.dumps(footer)+'\n')
    return path


def test_openvins_quality_native_gates_and_full_covariance_remain_unqualified(tmp_path):
    from tools.openvins.native_observations import analyze_openvins_tracking_quality, qualify_openvins_tracking_health
    summary,_ = ov_health_run(tmp_path,ov_health_rows())
    quality = write_ov_quality(tmp_path/'quality.jsonl',ov_quality_rows())
    report = analyze_openvins_tracking_quality(quality,tmp_path/'native-events.jsonl',session_id='TEST_FIXTURE_ONLY')
    assert report['counts']['accepted_gates'] == 2 and report['counts']['rejected_gates'] == 1
    record = report['records'][0]
    assert record['visual_correction_observed'] and record['last_visual_sensor_ns'] == BASE
    assert record['accepted_observation_uses'] == 2
    assert record['covariance']['numerical_status'] == 'PASS'
    assert not report['output_eligible'] and report['covariance_consistency'] == 'UNQUALIFIED'
    before = quality.read_bytes()
    result = qualify_openvins_tracking_health(quality,tmp_path/'native-events.jsonl',tmp_path/'health.jsonl',
        summary,tmp_path/'measurement.jsonl',session_id='TEST_FIXTURE_ONLY')
    assert result['measurement_health']['counts']['PREDICTION_ONLY'] > 0
    assert result['measurement_health']['output_eligible_rows'] == 0
    assert result['time_qualification']['source_to_px4_boot_mapping'] is None
    assert quality.read_bytes() == before
    with pytest.raises(FileExistsError):
        qualify_openvins_tracking_health(quality,tmp_path/'native-events.jsonl',tmp_path/'health.jsonl',
            summary,tmp_path/'measurement.jsonl',session_id='TEST_FIXTURE_ONLY')


@pytest.mark.parametrize('defect',['sequence','state_bits','time_negative','bool_camera','camera_time','mean',
    'offset','gate_outside_transaction','gate_predicate','gate_population','gate_dof','snapshot_before_return',
    'missing_snapshot','duplicate_snapshot','finite_witness','covariance_shape','unknown_kind',
    'dropped_rows','io_failed','byte_count','missing_footer','duplicate_key','accepted_population','before_native_update'])
def test_openvins_quality_rejects_incomplete_or_misassociated_native_evidence(tmp_path,defect):
    from tools.openvins.native_observations import analyze_openvins_tracking_quality
    ov_health_run(tmp_path,ov_health_rows())
    rows=ov_quality_rows(); footer={}
    if defect=='sequence':rows[1]['sequence']+=1
    elif defect=='state_bits':rows[0]['state_time_double_bits']+=1
    elif defect=='time_negative':rows[0]['monotonic_ns']=-1
    elif defect=='bool_camera':rows[1]['camera_ids']=[False]
    elif defect=='camera_time':rows[1]['camera_time_double_bits']+=1
    elif defect=='mean':rows[1]['mean'][4]+=.01
    elif defect=='offset':rows[1]['camera_imu_offset_seconds']+=.01
    elif defect=='gate_outside_transaction':rows[0]['monotonic_ns']=2_500_000
    elif defect=='gate_predicate':rows[0]['gates'][0][6]=True
    elif defect=='gate_population':rows[0]['accepted_count']+=1
    elif defect=='gate_dof':rows[0]['gates'][0][3]=4
    elif defect=='accepted_population':
        rows[0]['gates'][1][1]=10.;rows[0]['gates'][1][6]=True
        rows[0].update(accepted_count=1,rejected_count=2)
    elif defect=='before_native_update':rows[0]['monotonic_ns']=3_500_000
    elif defect=='snapshot_before_return':rows[1]['monotonic_ns']=5_500_000
    elif defect=='missing_snapshot':rows.pop()
    elif defect=='duplicate_snapshot':rows.append(dict(rows[1],sequence=2))
    elif defect=='finite_witness':rows[1]['covariance_all_finite']=False
    elif defect=='covariance_shape':rows[1]['covariance_row_major'].pop()
    elif defect=='unknown_kind':rows[0]['kind']='watchdog_visual_update'
    elif defect=='dropped_rows':footer['dropped_rows']=1
    elif defect=='io_failed':footer['io_failed']=True
    elif defect=='byte_count':footer['bytes_before_footer']=0
    path=write_ov_quality(tmp_path/'quality.jsonl',rows,**footer)
    if defect=='missing_footer':path.write_text('\n'.join(path.read_text().splitlines()[:-1])+'\n')
    elif defect=='duplicate_key':path.write_text(path.read_text().replace('"sequence": 0','"sequence": 0, "sequence": 0'))
    with pytest.raises((ExperimentError,ValueError)):
        analyze_openvins_tracking_quality(path,tmp_path/'native-events.jsonl',session_id='TEST_FIXTURE_ONLY')


def test_openvins_quality_nonfinite_native_gate_is_observed_but_numerically_rejected(tmp_path):
    from tools.openvins.native_observations import analyze_openvins_tracking_quality
    ov_health_run(tmp_path,ov_health_rows())
    rows=ov_quality_rows()
    # Actual C++ NaN > threshold is false: do not relabel that branch PASS.
    rows[0]['gates'][0][1]=None
    rows[0]['nonfinite_count']=1
    report=analyze_openvins_tracking_quality(write_ov_quality(tmp_path/'quality.jsonl',rows),
        tmp_path/'native-events.jsonl',session_id='TEST_FIXTURE_ONLY')
    assert report['records'][0]['native_gate_counts']['invalid_gate_count']==1
    assert report['counts']['accepted_gates']==2 and report['counts']['invalid_gates']==1
    assert not report['output_eligible']


def test_openvins_covariance_extreme_finite_range_fails_closed():
    from tools.openvins.native_observations import inspect_openvins_imu_covariance
    values=[0.]*225
    values[1]=1.7e308;values[15]=-1.7e308
    report=inspect_openvins_imu_covariance(values)
    assert report['numerical_status']=='FAIL'
    assert report['reasons']==['COVARIANCE_DIAGNOSTIC_NUMERIC_RANGE']


@pytest.mark.parametrize('threshold,multiplier',[(1e300,1e300),(1e-300,1e-300)])
def test_openvins_quality_effective_gate_numeric_range_does_not_qualify(threshold,multiplier,tmp_path):
    from tools.openvins.native_observations import analyze_openvins_tracking_quality
    ov_health_run(tmp_path,ov_health_rows())
    rows=ov_quality_rows();gate=rows[0]['gates'][0]
    gate[4:7]=[threshold,multiplier,1. > threshold*multiplier]
    rows[0]['accepted_count']=sum(not entry[6] for entry in rows[0]['gates'])
    rows[0]['rejected_count']=len(rows[0]['gates'])-rows[0]['accepted_count']
    # Maintain the independent native feature population when the branch changes.
    if rows[0]['accepted_count']!=2:
        native=[json.loads(line) for line in (tmp_path/'native-events.jsonl').read_text().splitlines()]
        next(r for r in native if r['kind']=='msckf_visual_ekf_completed')['measurement_support']['feature_count']=rows[0]['accepted_count']
        (tmp_path/'native-events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in native))
    report=analyze_openvins_tracking_quality(write_ov_quality(tmp_path/'quality.jsonl',rows),
        tmp_path/'native-events.jsonl',session_id='TEST_FIXTURE_ONLY')
    assert report['records'][0]['native_gate_counts']['invalid_gate_count']==1
    assert report['counts']['nonfinite_gates']==0 and not report['output_eligible']
    json.dumps(report,allow_nan=False)


@pytest.mark.parametrize('defect',['native_source','health_timeline','session'])
def test_openvins_quality_rejects_health_from_other_sealed_source(tmp_path,defect):
    from tools.openvins.native_observations import qualify_openvins_tracking_health
    summary,_=ov_health_run(tmp_path,ov_health_rows())
    quality=write_ov_quality(tmp_path/'quality.jsonl',ov_quality_rows())
    if defect=='native_source':summary['native_event_source']['sha256']='0'*64
    elif defect=='health_timeline':summary['timeline']['sha256']='0'*64
    else:summary['session_id']='OTHER'
    with pytest.raises(ExperimentError,match='OPENVINS_QUALITY_INVALID'):
        qualify_openvins_tracking_health(quality,tmp_path/'native-events.jsonl',tmp_path/'health.jsonl',summary,
            tmp_path/'measurement.jsonl',session_id='TEST_FIXTURE_ONLY')
    assert not (tmp_path/'measurement.jsonl').exists()


def test_openvins_quality_overflowing_aggregate_is_explicit_numeric_range_failure(tmp_path):
    from tools.openvins.native_observations import analyze_openvins_tracking_quality
    ov_health_run(tmp_path,ov_health_rows())
    rows=ov_quality_rows()
    for gate in rows[0]['gates']:gate[1:7]=[1e308,3,3,1e308,1.,False]
    rows[0].update(accepted_count=3,rejected_count=0)
    native=[json.loads(line) for line in (tmp_path/'native-events.jsonl').read_text().splitlines()]
    next(r for r in native if r['kind']=='msckf_visual_ekf_completed')['measurement_support']['feature_count']=3
    (tmp_path/'native-events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in native))
    with pytest.raises(ExperimentError,match='aggregate numeric range'):
        analyze_openvins_tracking_quality(write_ov_quality(tmp_path/'quality.jsonl',rows),
            tmp_path/'native-events.jsonl',session_id='TEST_FIXTURE_ONLY')


def test_openvins_quality_cannot_reuse_old_snapshot_after_next_camera_transaction(tmp_path):
    from tools.openvins.native_observations import analyze_openvins_tracking_quality
    rows=ov_health_rows()
    second=BASE+100_000_000
    for kind,ms,values,support in [('camera_ros_callback',60,None,None),('camera_processing_begin',61,None,None),
        ('msckf_visual_ekf_completed',62,ov_state_values(),ov_support((0,second))),
        ('camera_processing_returned',63,None,None),('camera_returned_state',64,[1,1.,*ov_state_values()[1:]],None)]:
        row=ov_native_event(kind,second,values=values,support=support,camera=0 if kind=='camera_ros_callback' else None)
        row['monotonic_ns']=ms*1_000_000
        if kind in ('camera_processing_begin','camera_processing_returned'):row['sensor_ids']=[0]
        rows.append(row)
    ov_health_run(tmp_path,rows)
    quality=ov_quality_rows()
    quality[1]['monotonic_ns']=61_500_000
    with pytest.raises(ExperimentError,match='camera lifecycle'):
        analyze_openvins_tracking_quality(write_ov_quality(tmp_path/'quality.jsonl',quality),
            tmp_path/'native-events.jsonl',session_id='TEST_FIXTURE_ONLY')
