"""Software evidence tests; none is a live VIO/flight qualification."""

from dataclasses import replace
import json

import pytest

from rm27.perception.vision.localization.schema import (
    LocalizationEstimate, ScaleState, TimestampEvidence, TrackingState,
)
from rm27.perception.vision.localization.state_output import (
    NativeCorrectionEvidence, PropagationSample, StateOutputPolicy, StateOutputTracker,
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
