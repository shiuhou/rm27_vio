"""Software evidence tests; none is a live VIO/flight qualification."""

from dataclasses import replace
import json

import pytest

from rm27.perception.vision.localization.schema import (
    LocalizationEstimate, ScaleState, TimestampEvidence, TrackingState,
)
from rm27.perception.vision.localization.state_output import (
    NativeCorrectionEvidence, PropagationSample, StateOutputPolicy, StateOutputTracker,
    evaluate_openvins_operational_validity, evaluate_openvins_measurement_health,
)

BASE = 1_403_636_580_000_000_123
MS = 1_000_000


def sensor_time(delta=0):
    return TimestampEvidence(BASE + delta, "ns", "DATASET_SENSOR", "STATE_ESTIMATE", "VERIFIED")


def receive_time(delta=0):
    return TimestampEvidence(delta, "ns", "HOST_MONOTONIC", "RECEIVE", "VERIFIED")


def correction(delta=0, *, epoch=0, position=(0, 0, 0), velocity=(1, 0, 0)):
    estimate = LocalizationEstimate(
        source_time=sensor_time(delta), publish_time=receive_time(delta),
        parent_frame="vio_world", child_frame="imu", translation=position,
        orientation_wxyz=(1, 0, 0, 0), tracking_state=TrackingState.TRACKING,
        valid=True, initialized=True, relocalizing=False, localization_epoch=epoch,
        scale_state=ScaleState.METRIC, translation_unit="m", velocity=velocity,
        velocity_unit="m/s", velocity_frame="vio_world",
        quality={"native_covariance_placeholder": [0, 0, 0]},
    )
    return NativeCorrectionEvidence(
        estimate=estimate, last_accepted_visual_time=sensor_time(delta),
        correction_id=f"mock-correction-{delta}", initialization_source="MOCK initialized state",
        scale_source="MOCK metric initialization", bias_source="MOCK calibrated biases",
        gravity_source="MOCK world gravity", gyro_bias_rad_s=(0, 0, 0),
        accel_bias_m_s2=(0, 0, 0), gravity_parent_m_s2=(0, 0, -9.81), evidence_status="VERIFIED",
    )


def sample(delta=0, *, sequence=0, gyro=(0, 0, 0), force=(0, 0, 9.81)):
    return PropagationSample(sensor_time(delta), sequence, "mock-imu", "imu", gyro, force,
                             "MOCK SI/axis calibration", "VERIFIED")


def tracker(**changes):
    policy = StateOutputPolicy(100*MS, 20*MS, 50*MS, 150*MS, 100)
    return StateOutputTracker(replace(policy, **changes))


def output(instance, receive=0, sensor=None):
    return instance.output(publish_time=receive_time(receive), now_receive_time=receive_time(receive), sensor_now=sensor)


def test_numeric_pose_does_not_supply_initialization_evidence():
    instance = tracker()
    assert output(instance).estimate is None
    assert not output(instance).usable
    with pytest.raises(ValueError, match="VERIFIED"):
        replace(correction(), evidence_status="UNKNOWN")
    with pytest.raises(ValueError, match="metric"):
        replace(correction(), estimate=replace(correction().estimate, scale_state=ScaleState.ARBITRARY,
                                               translation_unit="arbitrary", velocity_unit="arbitrary/s"))
    with pytest.raises(ValueError, match="velocity"):
        replace(correction(), estimate=replace(correction().estimate, velocity=None,
                                               velocity_unit=None, velocity_frame=None))


def test_propagation_is_independent_keeps_integer_time_and_last_visual_time():
    instance = tracker()
    snapshot = correction()
    original = snapshot.estimate.to_json()
    instance.accept_correction(snapshot, received_time=receive_time())
    instance.push_imu(sample(), received_time=receive_time())
    instance.push_imu(sample(10*MS, sequence=1), received_time=receive_time(10*MS))
    instance.push_imu(sample(20*MS+1, sequence=2), received_time=receive_time(20*MS+1))
    result = output(instance, 20*MS+1)
    assert result.usable
    assert result.estimate.translation == pytest.approx((0.020000001, 0, 0))
    assert result.estimate.velocity == pytest.approx((1, 0, 0))
    assert result.estimate.source_time.raw_value == BASE+20*MS+1
    assert result.last_accepted_visual_time.raw_value == BASE
    assert result.estimate.quality["state_output"]["state_source"] == "IMU_PROPAGATION"
    assert result.estimate.quality["state_output"]["propagated_intervals"] == 2
    assert snapshot.estimate.to_json() == original
    json.dumps(result.to_dict(), allow_nan=False)


def test_specific_force_bias_and_rotation_are_applied_once():
    instance = tracker()
    native = replace(correction(velocity=(0, 0, 0)), gyro_bias_rad_s=(0, 0, 1), accel_bias_m_s2=(1, 0, 0))
    instance.accept_correction(native, received_time=receive_time())
    instance.push_imu(sample(gyro=(0, 0, 1), force=(3, 0, 9.81)), received_time=receive_time())
    instance.push_imu(sample(10*MS, sequence=1), received_time=receive_time(10*MS))
    result = output(instance, 10*MS)
    assert result.estimate.velocity == pytest.approx((0.02, 0, 0))
    assert result.estimate.translation == pytest.approx((0.0001, 0, 0))
    assert result.estimate.orientation_wxyz == pytest.approx((1, 0, 0, 0))


def test_propagated_rotation_and_force_use_imu_axes():
    import math

    instance = tracker(max_imu_gap_ns=2_000*MS, max_visual_age_ns=2_000*MS,
                       max_receive_age_ns=2_000*MS, max_visual_receive_age_ns=2_000*MS)
    instance.accept_correction(correction(velocity=(0, 0, 0)), received_time=receive_time())
    # Constant body yaw pi/2 rad/s and force along body x; midpoint integration
    # rotates the interval's force to 45 degrees in the world.
    instance.push_imu(sample(gyro=(0, 0, math.pi/2), force=(1, 0, 9.81)), received_time=receive_time())
    instance.push_imu(sample(1_000*MS, sequence=1), received_time=receive_time(1_000*MS))
    result = output(instance, 1_000*MS)
    assert result.estimate.orientation_wxyz == pytest.approx((math.sqrt(.5), 0, 0, math.sqrt(.5)))
    assert result.estimate.velocity == pytest.approx((math.sqrt(.5), math.sqrt(.5), 0))
    assert result.estimate.translation == pytest.approx((math.sqrt(.5)/2, math.sqrt(.5)/2, 0))


def test_new_correction_rebuilds_snapshot_and_replays_only_later_samples():
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    for i in range(3):
        instance.push_imu(sample(i*10*MS, sequence=i), received_time=receive_time(i*10*MS))
    instance.accept_correction(correction(10*MS, position=(5, 0, 0), velocity=(2, 0, 0)),
                               received_time=receive_time(30*MS))
    result = output(instance, 30*MS)
    assert result.estimate.translation == pytest.approx((5.02, 0, 0))
    assert result.estimate.source_time.raw_value == BASE+20*MS
    assert result.last_accepted_visual_time.raw_value == BASE+10*MS
    assert result.estimate.quality["state_output"]["propagated_intervals"] == 1


@pytest.mark.parametrize("correction_ms", [20, 40])
def test_visual_snapshot_can_overtake_multiple_ordered_imu_callbacks(correction_ms):
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    instance.push_imu(sample(), received_time=receive_time())
    # Camera and IMU subscriptions have separate delivery queues. A native
    # correction can arrive before several older, correctly ordered IMU samples.
    snapshot = correction(correction_ms*MS, position=(5, 0, 0), velocity=(2, 0, 0))
    instance.accept_correction(snapshot, received_time=receive_time(10*MS))
    for sequence, delta in enumerate(range(5, correction_ms+6, 5), start=1):
        received = (10+sequence)*MS
        instance.push_imu(sample(delta*MS, sequence=sequence), received_time=receive_time(received))
        result = output(instance, received)
        assert result.usable
        assert result.last_accepted_visual_time == snapshot.last_accepted_visual_time
        if delta <= correction_ms:
            assert result.estimate.source_time == snapshot.estimate.source_time
            assert result.estimate.translation == (5, 0, 0)
            assert result.estimate.quality["state_output"]["propagated_intervals"] == 0
    assert result.estimate.source_time.raw_value == BASE+(correction_ms+5)*MS
    assert result.estimate.translation == pytest.approx((5.01, 0, 0))
    assert result.estimate.quality["state_output"]["propagated_intervals"] == 1


@pytest.mark.parametrize("delta,sequence", [(5, 2), (4, 2), (10, 3)])
def test_pending_imu_before_snapshot_still_rejects_duplicates_reordering_and_loss(delta, sequence):
    instance = tracker()
    instance.accept_correction(correction(40*MS), received_time=receive_time())
    instance.push_imu(sample(5*MS, sequence=1), received_time=receive_time(MS))
    with pytest.raises(ValueError):
        instance.push_imu(sample(delta*MS, sequence=sequence), received_time=receive_time(2*MS))
    assert not output(instance, 2*MS).usable


@pytest.mark.parametrize("delta,sequence", [(0, 1), (10*MS, 0), (10*MS, 2), (21*MS, 1)])
def test_bad_imu_order_duplicates_or_gaps_latch_invalid(delta, sequence):
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    instance.push_imu(sample(), received_time=receive_time())
    with pytest.raises(ValueError):
        instance.push_imu(sample(delta, sequence=sequence), received_time=receive_time(30*MS))
    result = output(instance, 30*MS)
    assert not result.usable
    assert result.estimate.tracking_state == TrackingState.LOST
    assert not result.estimate.valid


def test_missing_anchor_prevents_propagation_but_native_correction_can_recover():
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    instance.push_imu(sample(10*MS, sequence=1), received_time=receive_time(10*MS))
    assert output(instance, 10*MS).reasons == ("IMU_ANCHOR_UNAVAILABLE",)
    instance.accept_correction(correction(10*MS), received_time=receive_time(11*MS))
    instance.push_imu(sample(20*MS, sequence=2), received_time=receive_time(20*MS))
    assert output(instance, 20*MS).usable


def test_imu_propagation_does_not_refresh_visual_health():
    instance = tracker(max_visual_age_ns=15*MS)
    instance.accept_correction(correction(), received_time=receive_time())
    for i in range(3):
        instance.push_imu(sample(i*10*MS, sequence=i), received_time=receive_time(i*10*MS))
    result = output(instance, 20*MS)
    assert result.reasons == ("VISUAL_CORRECTION_STALE",)
    assert not result.usable


def test_independent_watchdog_detects_stopped_callbacks_and_stale_vision():
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    result = output(instance, 200*MS)
    assert set(result.reasons) == {"SENSOR_STREAM_STOPPED", "VISUAL_STREAM_STOPPED"}
    assert result.estimate.source_time == sensor_time()
    assert result.estimate.publish_time == receive_time(200*MS)
    assert not result.usable


def test_sensor_now_can_show_staleness_without_new_output_samples():
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    assert output(instance, 10*MS, sensor_time(101*MS)).reasons == ("VISUAL_CORRECTION_STALE",)


def test_reset_clears_history_requires_new_metric_initialization():
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    instance.push_imu(sample(), received_time=receive_time())
    instance.reset(1, reason="MOCK estimator reset", received_time=receive_time(10*MS))
    reset = output(instance, 10*MS)
    assert reset.estimate is None and not reset.usable and reset.localization_epoch == 1
    instance.accept_correction(correction(20*MS, epoch=1), received_time=receive_time(20*MS))
    fresh = output(instance, 20*MS)
    assert fresh.usable
    assert fresh.estimate.quality["state_output"]["state_progress_receive_time"] == receive_time(20*MS).to_dict()
    instance.push_imu(sample(30*MS, sequence=0), received_time=receive_time(30*MS))
    assert not output(instance, 30*MS).usable  # No cross-epoch anchor survives.
    with pytest.raises(ValueError, match="increase"):
        instance.reset(1, reason="duplicate", received_time=receive_time(31*MS))


def test_epoch_frame_clock_and_correction_rewind_are_explicit():
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    with pytest.raises(ValueError, match="explicit reset"):
        instance.accept_correction(correction(10*MS, epoch=1), received_time=receive_time(10*MS))
    with pytest.raises(ValueError, match="increase"):
        instance.accept_correction(correction(), received_time=receive_time(10*MS))
    with pytest.raises(ValueError, match="clocks differ"):
        output(instance, sensor=replace(sensor_time(1), clock_domain="OTHER_SENSOR"))
    with pytest.raises(ValueError, match="verified"):
        output(instance, sensor=replace(sensor_time(1), evidence_status="UNKNOWN"))


def test_imu_change_requires_reset_and_cannot_be_hidden_by_next_sample():
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    instance.push_imu(sample(), received_time=receive_time())
    with pytest.raises(ValueError, match="change requires reset"):
        instance.push_imu(replace(sample(10*MS, sequence=1), calibration_source="changed"),
                          received_time=receive_time(10*MS))
    instance.push_imu(sample(20*MS, sequence=1), received_time=receive_time(20*MS))
    assert not output(instance, 20*MS).usable


@pytest.mark.parametrize("values", [(float("nan"), 0, 0), (float("inf"), 0, 0), (True, 0, 0)])
def test_nonfinite_or_boolean_measurements_are_not_constructible(values):
    with pytest.raises(ValueError, match="finite"):
        sample(force=values)


def test_bounded_buffer_cannot_hide_missing_repropagation_history():
    instance = tracker(max_buffer_samples=2)
    instance.accept_correction(correction(), received_time=receive_time())
    for i in range(5):
        instance.push_imu(sample(i*10*MS, sequence=i), received_time=receive_time(i*10*MS))
    instance.accept_correction(correction(10*MS), received_time=receive_time(45*MS))
    assert not output(instance, 45*MS).usable
    assert output(instance, 45*MS).reasons == ("IMU_ANCHOR_UNAVAILABLE",)


def test_watchdog_clock_rewind_cannot_restore_health_or_retime_callbacks():
    instance = tracker()
    instance.accept_correction(correction(), received_time=receive_time())
    assert not output(instance, 200*MS).usable
    with pytest.raises(ValueError, match="backwards"):
        output(instance, 10*MS)
    with pytest.raises(ValueError, match="backwards"):
        instance.accept_correction(correction(10*MS), received_time=receive_time(10*MS))
    assert not output(instance, 200*MS).usable


def test_preinitialization_watchdog_clock_also_protects_first_callback():
    instance = tracker()
    assert not output(instance, 200*MS).usable
    with pytest.raises(ValueError, match="backwards"):
        instance.accept_correction(correction(), received_time=receive_time(10*MS))


@pytest.mark.parametrize("imu_delta", [0, 20*MS, 40*MS])
def test_delayed_imu_receipt_cannot_revive_stalled_snapshot(imu_delta):
    instance = tracker(max_visual_age_ns=1000*MS, max_receive_age_ns=100*MS,
                       max_visual_receive_age_ns=1000*MS)
    instance.accept_correction(correction(40*MS), received_time=receive_time())
    stale = output(instance, 200*MS)
    assert stale.reasons == ("SENSOR_STREAM_STOPPED",)
    instance.push_imu(sample(imu_delta), received_time=receive_time(201*MS))
    result = output(instance, 201*MS)
    assert not result.usable
    assert result.reasons == ("STATE_PROGRESS_STOPPED",)
    assert result.estimate.source_time == stale.estimate.source_time
    assert result.last_accepted_visual_time == stale.last_accepted_visual_time
    evidence = result.estimate.quality["state_output"]
    assert evidence["state_source"] == "VISUAL_CORRECTION"
    assert evidence["receive_time"] == receive_time(201*MS).to_dict()
    assert evidence["state_progress_receive_time"] == receive_time().to_dict()
    tick = output(instance, 202*MS)
    assert not tick.usable
    assert tick.estimate.source_time == result.estimate.source_time
    assert tick.estimate.quality["state_output"]["state_progress_receive_time"] == evidence["state_progress_receive_time"]


@pytest.mark.parametrize("recovery", ["prediction", "correction"])
def test_stalled_state_requires_actual_prediction_or_new_correction_to_recover(recovery):
    instance = tracker(max_visual_age_ns=1000*MS, max_receive_age_ns=100*MS,
                       max_visual_receive_age_ns=1000*MS)
    instance.accept_correction(correction(40*MS), received_time=receive_time())
    assert not output(instance, 200*MS).usable
    instance.push_imu(sample(40*MS), received_time=receive_time(201*MS))
    assert not output(instance, 201*MS).usable
    if recovery == "prediction":
        instance.push_imu(sample(50*MS, sequence=1), received_time=receive_time(202*MS))
    else:
        instance.accept_correction(correction(50*MS), received_time=receive_time(202*MS))
    result = output(instance, 202*MS)
    assert result.usable and result.estimate.source_time == sensor_time(50*MS)
    evidence = result.estimate.quality["state_output"]
    assert evidence["state_progress_receive_time"] == receive_time(202*MS).to_dict()
    if recovery == "prediction":
        assert result.last_accepted_visual_time == sensor_time(40*MS)
        assert evidence["state_source"] == "IMU_PROPAGATION"
        assert evidence["visual_receive_time"] == receive_time().to_dict()
    else:
        assert result.last_accepted_visual_time == sensor_time(50*MS)
        assert evidence["state_source"] == "VISUAL_CORRECTION"


def test_watchdog_does_not_refresh_stalled_state_progress():
    instance = tracker(max_visual_age_ns=1000*MS, max_receive_age_ns=100*MS,
                       max_visual_receive_age_ns=1000*MS)
    instance.accept_correction(correction(), received_time=receive_time())
    for instant in (200*MS, 201*MS, 202*MS):
        result = output(instance, instant)
        assert not result.usable and result.reasons == ("SENSOR_STREAM_STOPPED",)
        assert result.estimate.source_time == result.last_accepted_visual_time == sensor_time()
        evidence = result.estimate.quality["state_output"]
        assert evidence["receive_time"] == evidence["state_progress_receive_time"] == receive_time().to_dict()


def native_policy_fixture():
    """MOCK of the verified sidecar shape, never a native health qualification."""
    summary = dict(schema_version=1, status='PASS', truth_used=False,
        scope='PC_OFFLINE_NATIVE_OBSERVABILITY_AND_CONFIGURED_FRESHNESS_ONLY',
        session_id='MOCK native sidecar',
        native_clock_semantic='PROCESS_LOCAL_STEADY_EVENT_SERIALIZATION_NOT_HARDWARE_RECEIPT',
        adapter_epoch_semantic='ONE_ISOLATED_PROCESS_SESSION_NOT_NATIVE_RESET_COUNT',
        policy=dict(max_visual_age_ns=100*MS, max_receive_age_ns=50*MS,
                    max_visual_receive_age_ns=150*MS, max_imu_gap_ns=20*MS, max_buffer_samples=100),
        initialization=dict(commit_observed=True, ready_observed=True),
        covariance=dict(status='UNQUALIFIED', native_finite_witness_only=True),
        native_reset=dict(status='BLOCKED', execution='NOT_RUN'),
        operational_health='UNKNOWN', operational_valid=False)
    row = dict(native_event_sequence=10, event_kind='landmark_visual_ekf_completed',
        native_observation_ns=200*MS, session_id=summary['session_id'],
        adapter_epoch=0, native_epoch=None, native_reset_count=None, lifecycle='INITIALIZED',
        freshness='CURRENT', freshness_reasons=[], state_source='VISUAL_CORRECTION',
        state_time=dict(seconds=BASE/10**9, semantic='NATIVE_CAMERA_STATE_DOUBLE'),
        last_sensor_observation_ns=190*MS, last_imu_observation_ns=190*MS,
        last_state_progress_observation_ns=195*MS, last_visual_observation_ns=195*MS,
        last_visual_sensor_ns=BASE, sensor_clock='MOCK sensor clock', tracking_state='UNKNOWN',
        valid=False, contract_complete=False, operational_health='UNKNOWN', native_publication_timestamp=None)
    return row, summary


def native_quality_fixture(row):
    """Constructed decoder output for policy tests, not native qualification."""
    return dict(schema_version=1,qualification='VERIFIED_NATIVE_MEASUREMENT_DIAGNOSTIC_ONLY',
        truth_used=False,session_id=row['session_id'],native_observation_ns=row['native_observation_ns'],
        quality_sequence=2,last_visual_sensor_ns=row['last_visual_sensor_ns'],
        operational_tracking_health='UNKNOWN',output_eligible=False,native_epoch=None,native_reset_count=None,
        visual_correction_observed=True,accepted_visual_update_count=1,accepted_observation_uses=6,
        native_gate_counts=dict(input_feature_count=4,gate_count=3,accepted_count=2,rejected_count=1,
                                nonfinite_count=0,invalid_gate_count=0),
        covariance=dict(numerical_status='PASS',statistical_consistency='UNQUALIFIED',
                        flight_covariance='UNQUALIFIED',covariance_modified=False,reasons=[]))


def test_measurement_policy_accepted_support_never_qualifies_tracking_or_covariance():
    row,summary=native_policy_fixture();quality=native_quality_fixture(row)
    before=json.dumps((row,summary,quality),sort_keys=True)
    result=evaluate_openvins_measurement_health(row,summary,quality)
    assert result['measurement_health']=='VISUAL_CONSTRAINTS_OBSERVED'
    assert result['tracking_health']=='UNKNOWN' and result['covariance_consistency']=='UNQUALIFIED'
    assert not result['output_eligible'] and result['truth_used'] is False
    assert result['rejected_native_gate_fraction']==pytest.approx(1/3)
    assert json.dumps((row,summary,quality),sort_keys=True)==before


@pytest.mark.parametrize('failure',['missing','other_session','future_quality','other_visual','truth',
    'tracking','eligible','reset','bool_sequence','population','nonfinite_population','covariance_trust','repaired'])
def test_measurement_policy_missing_or_unverifiable_quality_fails_closed(failure):
    row,summary=native_policy_fixture();quality=native_quality_fixture(row)
    if failure=='missing':quality=None
    elif failure=='other_session':quality['session_id']='OTHER'
    elif failure=='future_quality':quality['native_observation_ns']+=1
    elif failure=='other_visual':quality['last_visual_sensor_ns']+=1
    elif failure=='truth':quality['truth_used']=True
    elif failure=='tracking':quality['operational_tracking_health']='TRACKING'
    elif failure=='eligible':quality['output_eligible']=True
    elif failure=='reset':quality['native_epoch']=1
    elif failure=='bool_sequence':quality['quality_sequence']=True
    elif failure=='population':quality['native_gate_counts']['accepted_count']+=1
    elif failure=='nonfinite_population':quality['native_gate_counts']['nonfinite_count']=1
    elif failure=='covariance_trust':quality['covariance']['statistical_consistency']='QUALIFIED'
    elif failure=='repaired':quality['covariance']['covariance_modified']=True
    result=evaluate_openvins_measurement_health(row,summary,quality)
    assert result['measurement_health']=='UNKNOWN' and result['reasons']
    assert not result['output_eligible'] and result['tracking_health']=='UNKNOWN'


def test_measurement_policy_prediction_watchdog_stale_and_new_visual_are_distinct():
    row,summary=native_policy_fixture();quality=native_quality_fixture(row)
    row.update(event_kind='imu_fast_prediction_completed',state_source='IMU_PREDICTION',
        state_time=dict(seconds=(BASE+20*MS)/10**9,request_sensor_ns=BASE+20*MS,
                        semantic='NATIVE_FAST_PROPAGATION_INTEGRATION_ENDPOINT'))
    prediction=evaluate_openvins_measurement_health(row,summary,quality)
    assert prediction['measurement_health']=='PREDICTION_ONLY' and prediction['last_visual_sensor_ns']==BASE
    stale=evaluate_openvins_measurement_health(row,summary,quality,now_observation_ns=500*MS)
    assert stale['measurement_health']=='STALE' and stale['last_visual_sensor_ns']==BASE
    row.update(native_observation_ns=600*MS,last_sensor_observation_ns=600*MS,last_imu_observation_ns=600*MS)
    assert evaluate_openvins_measurement_health(row,summary,quality)['measurement_health']=='STALE'
    # Only a new, causally paired accepted visual correction restores freshness.
    row.update(event_kind='landmark_visual_ekf_completed',state_source='VISUAL_CORRECTION',
        last_state_progress_observation_ns=600*MS,last_visual_observation_ns=600*MS,
        last_visual_sensor_ns=BASE+400*MS,
        state_time=dict(seconds=(BASE+400*MS)/10**9,semantic='NATIVE_CAMERA_STATE_DOUBLE'))
    assert evaluate_openvins_measurement_health(row,summary,quality)['measurement_health']=='UNKNOWN'
    quality=native_quality_fixture(row)
    restored=evaluate_openvins_measurement_health(row,summary,quality)
    assert restored['measurement_health']=='VISUAL_CONSTRAINTS_OBSERVED'
    assert restored['tracking_health']=='UNKNOWN' and not restored['output_eligible']


@pytest.mark.parametrize('failure',['innovation','covariance','all_rejected','none'])
def test_measurement_policy_native_numeric_and_gate_rejection_has_explicit_reason(failure):
    row,summary=native_policy_fixture();quality=native_quality_fixture(row)
    if failure=='innovation':quality['native_gate_counts']['invalid_gate_count']=1
    elif failure=='covariance':quality['covariance'].update(numerical_status='FAIL',reasons=['NATIVE_COVARIANCE_NOT_PSD'])
    else:
        quality.update(visual_correction_observed=False,accepted_visual_update_count=0,accepted_observation_uses=0)
        if failure=='all_rejected':quality['native_gate_counts'].update(accepted_count=0,rejected_count=3)
        else:quality['native_gate_counts']={key:0 for key in quality['native_gate_counts']}
    result=evaluate_openvins_measurement_health(row,summary,quality)
    expected={'innovation':'NUMERICAL_REJECTION','covariance':'NUMERICAL_REJECTION',
              'all_rejected':'ALL_NATIVE_GATES_REJECTED','none':'NO_NEW_VISUAL_CONSTRAINTS'}
    assert result['measurement_health']==expected[failure]
    assert not result['output_eligible']


def test_operational_policy_normal_fresh_native_evidence_does_not_qualify_tracking():
    row, summary = native_policy_fixture()
    decision = evaluate_openvins_operational_validity(row, summary)
    assert decision.freshness == 'CURRENT'
    assert decision.tracking_health == 'UNKNOWN'
    assert decision.covariance_qualification == 'UNQUALIFIED'
    assert decision.lifecycle_qualification == 'PROCESS_SESSION_ONLY'
    assert not decision.output_eligible
    assert set(decision.reasons) == {'TRACKING_HEALTH_UNQUALIFIED', 'COVARIANCE_UNQUALIFIED',
                                     'NATIVE_RESET_EPOCH_UNQUALIFIED', 'NATIVE_PUBLICATION_TIME_UNOBSERVED'}
    json.dumps(decision.to_dict(), allow_nan=False)


def test_operational_policy_prediction_does_not_renew_last_visual_and_recovery_is_freshness_only():
    row, summary = native_policy_fixture()
    row.update(event_kind='imu_fast_prediction_completed', state_source='IMU_PREDICTION',
               state_time=dict(seconds=(BASE+101*MS)/10**9, request_sensor_ns=BASE+101*MS,
                               semantic='NATIVE_FAST_PROPAGATION_INTEGRATION_ENDPOINT'))
    prediction = evaluate_openvins_operational_validity(row, summary)
    assert prediction.freshness == 'STALE'
    assert 'VISUAL_MEASUREMENT_STALE' in prediction.reasons
    assert prediction.last_visual_sensor_ns == BASE
    assert not prediction.output_eligible
    # Native accepted support advances the visual evidence; it does not create
    # a tracking-health, covariance or native-reset witness.
    row.update(event_kind='landmark_visual_ekf_completed', state_source='VISUAL_CORRECTION',
               state_time=dict(seconds=(BASE+101*MS)/10**9, semantic='NATIVE_CAMERA_STATE_DOUBLE'),
               last_visual_sensor_ns=BASE+101*MS)
    recovered = evaluate_openvins_operational_validity(row, summary)
    assert recovered.freshness == 'CURRENT'
    assert recovered.last_visual_sensor_ns == BASE+101*MS
    assert recovered.tracking_health == 'UNKNOWN' and not recovered.output_eligible


def test_operational_policy_prediction_without_accepted_visual_cannot_initialize_health():
    row, summary = native_policy_fixture()
    row.update(state_source='IMU_PREDICTION', last_visual_sensor_ns=None,
               state_time=dict(seconds=BASE/10**9, request_sensor_ns=BASE,
                               semantic='NATIVE_FAST_PROPAGATION_INTEGRATION_ENDPOINT'))
    decision = evaluate_openvins_operational_validity(row, summary)
    assert decision.freshness == 'STALE'
    assert 'ACCEPTED_VISUAL_UPDATE_UNOBSERVED' in decision.reasons
    assert decision.last_visual_sensor_ns is None and not decision.output_eligible


@pytest.mark.parametrize('age_ms,expected', [(0, 'CURRENT'), (100, 'CURRENT'), (101, 'STALE')])
def test_operational_policy_prediction_freshness_deadline_never_relabels_prediction_as_vision(age_ms, expected):
    row, summary = native_policy_fixture()
    row.update(event_kind='imu_fast_prediction_completed', state_source='IMU_PREDICTION',
               state_time=dict(seconds=(BASE+age_ms*MS)/10**9, request_sensor_ns=BASE+age_ms*MS,
                               semantic='NATIVE_FAST_PROPAGATION_INTEGRATION_ENDPOINT'))
    result = evaluate_openvins_operational_validity(row, summary)
    assert result.freshness == expected and result.state_source == 'IMU_PREDICTION'
    assert result.last_visual_sensor_ns == BASE and not result.output_eligible


def test_operational_policy_stopped_watchdog_never_refreshes_source_evidence():
    row, summary = native_policy_fixture()
    before = json.dumps((row, summary), sort_keys=True)
    decision = evaluate_openvins_operational_validity(row, summary, now_observation_ns=500*MS)
    assert decision.freshness == 'STALE'
    assert {'SENSOR_STREAM_STOPPED', 'IMU_STREAM_STOPPED', 'STATE_PROGRESS_STOPPED',
            'VISUAL_EVENT_STALE'} <= set(decision.reasons)
    assert decision.last_visual_sensor_ns == BASE
    assert json.dumps((row, summary), sort_keys=True) == before
    # An old state's fresh input receipt cannot replace actual state progress.
    row.update(native_observation_ns=500*MS, last_sensor_observation_ns=500*MS,
               last_imu_observation_ns=500*MS)
    revival = evaluate_openvins_operational_validity(row, summary)
    assert revival.freshness == 'STALE' and 'STATE_PROGRESS_STOPPED' in revival.reasons
    assert revival.last_visual_sensor_ns == BASE and not revival.output_eligible


@pytest.mark.parametrize('freshness,reasons', [
    ('STALE', ['VISUAL_MEASUREMENT_STALE']), ('INITIALIZING', ['INITIALIZATION_NOT_READY'])])
def test_operational_policy_retains_decoder_stale_and_initializing(freshness, reasons):
    row, summary = native_policy_fixture()
    row.update(freshness=freshness, freshness_reasons=reasons)
    if freshness == 'INITIALIZING':
        row['lifecycle'] = 'INITIALIZATION_COMMITTED'
    result = evaluate_openvins_operational_validity(row, summary)
    assert result.freshness == freshness
    assert set(reasons) <= set(result.reasons) and not result.output_eligible


@pytest.mark.parametrize('field,value', [
    ('native_observation_ns', True), ('native_observation_ns', -1),
    ('native_observation_ns', float('nan')), ('last_state_progress_observation_ns', 201*MS),
    ('last_visual_observation_ns', False), ('sensor_clock', 'UNKNOWN'),
    ('freshness_reasons', ['VISUAL_EVENT_STALE']), ('session_id', 'another session')])
def test_operational_policy_bad_time_or_identity_fails_closed(field, value):
    row, summary = native_policy_fixture()
    row[field] = value
    result = evaluate_openvins_operational_validity(row, summary)
    assert result.freshness == 'UNKNOWN' and not result.output_eligible
    json.dumps(result.to_dict(), allow_nan=False)


@pytest.mark.parametrize('now', [True, -1, 199*MS, float('nan')])
def test_operational_policy_watchdog_time_cannot_rewind_or_be_non_integer(now):
    row, summary = native_policy_fixture()
    result = evaluate_openvins_operational_validity(row, summary, now_observation_ns=now)
    assert result.freshness == 'UNKNOWN' and 'NATIVE_OBSERVATION_TIME_INVALID' in result.reasons
    assert not result.output_eligible


@pytest.mark.parametrize('seconds', [float('nan'), float('inf'), True, -1, 10**400])
def test_operational_policy_invalid_state_seconds_fail_closed_without_exception(seconds):
    row, summary = native_policy_fixture()
    row['state_time']['seconds'] = seconds
    result = evaluate_openvins_operational_validity(row, summary)
    assert result.freshness == 'UNKNOWN' and 'STATE_TIME_OR_SOURCE_UNQUALIFIED' in result.reasons
    assert not result.output_eligible


@pytest.mark.parametrize('change', ['policy', 'clock', 'initialization', 'epoch_missing'])
def test_operational_policy_required_evidence_cannot_be_omitted_or_assumed(change):
    row, summary = native_policy_fixture()
    if change == 'policy':
        summary['policy']['max_receive_age_ns'] = True
    elif change == 'clock':
        summary['native_clock_semantic'] = 'HOST_MONOTONIC_ASSUMED_SAME_AS_SENSOR'
    elif change == 'initialization':
        summary['initialization']['ready_observed'] = False
    else:
        row.pop('native_reset_count')
    result = evaluate_openvins_operational_validity(row, summary)
    assert not result.output_eligible
    if change in ('policy', 'clock'):
        assert result.freshness == 'UNKNOWN'
    elif change == 'initialization':
        assert result.freshness == 'INITIALIZING'
    else:
        assert result.lifecycle_qualification == 'UNKNOWN'
        assert 'NATIVE_RESET_QUALIFICATION_UNSUPPORTED' in result.reasons


@pytest.mark.parametrize('lane', ['tracking', 'covariance', 'reset', 'publication'])
def test_operational_policy_downstream_claims_cannot_upgrade_native_capabilities(lane):
    row, summary = native_policy_fixture()
    if lane == 'tracking':
        row.update(tracking_state='TRACKING', operational_health='QUALIFIED', valid=True, contract_complete=True)
        summary.update(operational_health='QUALIFIED', operational_valid=True)
    elif lane == 'covariance':
        summary['covariance']['status'] = 'QUALIFIED'
    elif lane == 'reset':
        row.update(native_epoch=1, native_reset_count=1, adapter_epoch=1)
        summary['native_reset'].update(status='PASS', execution='PASS')
    else:
        row['native_publication_timestamp'] = BASE
    result = evaluate_openvins_operational_validity(row, summary)
    assert result.freshness == 'CURRENT' and not result.output_eligible
    assert result.tracking_health == 'UNKNOWN'
    assert any('UNSUPPORTED' in reason for reason in result.reasons)


def test_operational_policy_malformed_or_truth_based_inputs_never_authorize_output():
    for row, summary in [(None, None), ({}, {}), native_policy_fixture()]:
        if summary:
            summary['truth_used'] = True
        result = evaluate_openvins_operational_validity(row, summary)
        assert not result.output_eligible and result.freshness == 'UNKNOWN'
        assert 'NATIVE_HEALTH_SCHEMA_UNSUPPORTED' in result.reasons
