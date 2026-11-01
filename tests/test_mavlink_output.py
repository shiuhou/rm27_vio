"""MAVLink software-only tests; no flight controller or physical port is used."""

from dataclasses import asdict, replace
import json
import math
import subprocess
import sys

import pytest

from rm27.perception.vision.localization.mavlink import (
    AngularVelocityEvidence, EpochResetCounter, MavlinkOdometryCodec, MavlinkTimeMapping,
    OdometryTransformEvidence, RigidTransformEvidence, prepare_odometry, verify_localhost_loopback,
)
from rm27.perception.vision.localization.schema import LocalizationEstimate, TimestampEvidence, TrackingState
from rm27.perception.vision.localization.state_output import (
    NativeCorrectionEvidence, PropagationSample, StateOutputPolicy, StateOutputTracker,
)

BASE = 1_403_636_580_000_000_123
MS = 1_000_000


def timestamp(value=BASE, clock="MOCK_SENSOR"):
    return TimestampEvidence(value, "ns", clock, "STATE_ESTIMATE" if clock == "MOCK_SENSOR" else "RECEIVE", "VERIFIED")


def native_correction():
    record = LocalizationEstimate(
        source_time=timestamp(), publish_time=timestamp(0, "MOCK_RECEIVE"), parent_frame="world", child_frame="imu",
        translation=(1, 2, 3), orientation_wxyz=(1, 0, 0, 0), tracking_state=TrackingState.TRACKING,
        valid=True, initialized=True, relocalizing=False, localization_epoch=5, scale_state="METRIC",
        translation_unit="m", velocity=(1, 0, 0), velocity_unit="m/s", velocity_frame="world",
    )
    return NativeCorrectionEvidence(record, timestamp(), "mock-correction", "MOCK init", "MOCK metric",
                                    "MOCK biases", "MOCK gravity", (0, 0, 0), (0, 0, 0), (0, 0, -9.81), "VERIFIED")


def healthy_state():
    instance = StateOutputTracker(StateOutputPolicy(100*MS, 20*MS, 50*MS, 150*MS, 100))
    instance.accept_correction(native_correction(), received_time=timestamp(0, "MOCK_RECEIVE"))
    return instance.output(publish_time=timestamp(0, "MOCK_RECEIVE"), now_receive_time=timestamp(0, "MOCK_RECEIVE"))


def transforms(*, world_quaternion=(1, 0, 0, 0), body_quaternion=(1, 0, 0, 0), lever_arm=(0, 0, 0)):
    return OdometryTransformEvidence(
        RigidTransformEvidence("LOCAL_NED", "world", (0, 0, 0), world_quaternion, "MOCK world heading/alignment", "VERIFIED"),
        RigidTransformEvidence("imu", "BODY_FRD", lever_arm, body_quaternion, "MOCK calibrated body extrinsics", "VERIFIED"),
    )


def mapping():
    return MavlinkTimeMapping("MOCK_SENSOR", "FC_BOOT", -BASE+123, "MOCK fixed clock offset", "VERIFIED")


def angular_velocity(rate=(0, 0, 0)):
    return AngularVelocityEvidence(timestamp(), "imu", rate, "MOCK debiased angular rate", "VERIFIED")


def payload(state=None, *, rate=(0, 0, 0), transformation=None, now=0):
    return prepare_odometry(state or healthy_state(), transforms=transformation or transforms(), time_mapping=mapping(),
                            angular_velocity=angular_velocity(rate), reset_counter=7,
                            now_receive_time=timestamp(now, "MOCK_RECEIVE"))


def test_identity_transform_preserves_pose_twist_and_integer_time():
    actual = payload()
    assert actual.position_ned_m == (1, 2, 3)
    assert actual.velocity_body_m_s == (1, 0, 0)
    assert actual.orientation_ned_body_wxyz == (1, 0, 0, 0)
    assert actual.time_usec == 0
    assert actual.evidence["microsecond_quantization_residual_ns"] == 123
    assert actual.evidence["publication_time"]["clock_domain"] == "MOCK_RECEIVE"
    assert actual.to_dict()["pose_covariance"] is None
    assert actual.to_dict()["quality"] == 0  # common.xml defines zero as unknown.
    json.dumps(actual.to_dict(), allow_nan=False)


def test_rotations_lever_arm_velocity_and_body_angular_rate():
    s = math.sqrt(0.5)
    actual = payload(rate=(0, 0, 2), transformation=transforms(
        world_quaternion=(s, 0, 0, s), body_quaternion=(s, s, 0, 0), lever_arm=(1, 0, 0)))
    # World 90-degree yaw: p_body_origin=(2,2,3) -> NED=(-2,2,3).
    assert actual.position_ned_m == pytest.approx((-2, 2, 3))
    # IMU-origin velocity=(1,0,0); omega x lever=(0,2,0).
    # Body 90-degree roll maps IMU-y to BODY_FRD minus-z.
    assert actual.velocity_body_m_s == pytest.approx((1, 0, -2))
    assert actual.angular_velocity_body_rad_s == pytest.approx((0, 2, 0))
    assert actual.orientation_ned_body_wxyz == pytest.approx((0.5, 0.5, 0.5, 0.5))


def test_unknown_world_alignment_is_refused_and_not_retagged_ned():
    with pytest.raises(ValueError, match="VERIFIED"):
        RigidTransformEvidence("LOCAL_NED", "world", (0, 0, 0), (1, 0, 0, 0), "unqualified heading")
    with pytest.raises(ValueError, match="LOCAL_NED"):
        OdometryTransformEvidence(replace(transforms().ned_from_world, parent_frame="VIO_WORLD"), transforms().imu_from_body)
    with pytest.raises(ValueError, match="match the state"):
        payload(transformation=OdometryTransformEvidence(
            replace(transforms().ned_from_world, child_frame="wrong-world"), transforms().imu_from_body))


def test_missing_native_velocity_and_old_numeric_records_cannot_be_emitted():
    state = healthy_state()
    missing_velocity = replace(state.estimate, velocity=None, velocity_frame=None, velocity_unit=None)
    with pytest.raises(ValueError, match="velocity"):
        payload(replace(state, estimate=missing_velocity))
    with pytest.raises(ValueError, match="evidence"):
        payload(replace(state, estimate=replace(state.estimate, quality=None)))


def test_invalid_state_and_stale_queued_state_suppress_odometry():
    state = healthy_state()
    assert payload(replace(state, usable=False, reasons=("STALE",))) is None
    assert payload(now=51*MS) is None
    with pytest.raises(ValueError, match="clocks differ"):
        prepare_odometry(state, transforms=transforms(), time_mapping=mapping(), angular_velocity=angular_velocity(),
                         reset_counter=0, now_receive_time=timestamp(0, "OTHER_RECEIVE"))


def test_time_and_angular_rate_require_explicit_matching_evidence():
    with pytest.raises(ValueError, match="verified mapping"):
        MavlinkTimeMapping("MOCK_SENSOR", "UNIX", 0, "mock")
    with pytest.raises(ValueError, match="explicit mapping"):
        replace(mapping(), source_clock_domain="OTHER").map(timestamp())
    with pytest.raises(ValueError, match="state time"):
        prepare_odometry(healthy_state(), transforms=transforms(), time_mapping=mapping(),
                         angular_velocity=replace(angular_velocity(), time=timestamp(BASE-1)), reset_counter=0,
                         now_receive_time=timestamp(0, "MOCK_RECEIVE"))


def test_reset_counter_preserves_native_epoch_progression_and_wraps():
    counter = EpochResetCounter(5, 254)
    assert counter.observe(5) == 254
    assert counter.observe(6) == 255
    assert counter.observe(7) == 0
    with pytest.raises(ValueError, match="decrease"):
        counter.observe(6)


def test_real_mavlink2_encoder_decoder_and_unknown_covariance_wire_sentinel():
    codec = MavlinkOdometryCodec()
    raw = codec.encode(payload(rate=(0, 0, 2)))
    message = codec.decode(raw)
    assert raw[0] == 0xFD
    assert message.get_msgId() == 331
    assert (message.frame_id, message.child_frame_id) == (1, 12)
    assert message.q == pytest.approx([1, 0, 0, 0])
    assert (message.x, message.y, message.z) == pytest.approx((1, 2, 3))
    assert (message.vx, message.vy, message.vz) == pytest.approx((1, 0, 0))
    assert message.yawspeed == pytest.approx(2)
    assert message.reset_counter == 7 and message.quality == 0 and message.estimator_type == 2
    assert math.isnan(message.pose_covariance[0]) and math.isnan(message.velocity_covariance[0])
    assert len(message.pose_covariance) == len(message.velocity_covariance) == 21
    assert codec.provenance["pymavlink_version"] == "2.4.49"
    assert len(codec.provenance["message_definition_sha256"]) == 64
    assert codec.decode(codec.encode(payload())).get_seq() == 1


def test_real_localhost_udp_mock_uses_own_receiver_and_cleans_up():
    result = verify_localhost_loopback(payload())
    assert result["message_id"] == 331
    assert result["bytes_received"] > 0
    assert result["unknown_covariance_wire_sentinel"]
    assert result["provenance"]["evidence_class"] == "EMULATED / MOCK VERIFIED"


def test_incomplete_or_multiple_packets_are_rejected():
    codec = MavlinkOdometryCodec()
    packet = codec.encode(payload())
    for invalid in (packet[:-1], packet+packet, b"not-mavlink"):
        with pytest.raises(ValueError):
            codec.decode(invalid)


def test_codec_dependency_version_is_pinned(monkeypatch):
    import rm27.perception.vision.localization.mavlink as module

    monkeypatch.setattr(module.importlib.metadata, "version", lambda name: "2.4.50")
    with pytest.raises(RuntimeError, match="2.4.49 required"):
        MavlinkOdometryCodec()


def test_payload_json_never_contains_covariance_nan():
    with pytest.raises(ValueError, match="strict JSON"):
        replace(payload(), evidence={"unqualified_covariance": float("nan")})


def write_replay_fixture(destination):
    configuration = dict(policy=asdict(StateOutputPolicy(100*MS, 20*MS, 50*MS, 150*MS, 100)),
                         odometry=dict(ned_from_world=asdict(transforms().ned_from_world),
                                       imu_from_body=asdict(transforms().imu_from_body),
                                       time_mapping=asdict(mapping()), epoch_origin=5))
    config_path = destination / "mock-config.json"
    events_path = destination / "mock-events.jsonl"
    native = native_correction()
    native_payload = asdict(native)
    native_payload["estimate"] = native.estimate.to_dict()
    native_payload["last_accepted_visual_time"] = native.last_accepted_visual_time.to_dict()
    events = [dict(type="correction", correction=native_payload, received_time=timestamp(0, "MOCK_RECEIVE").to_dict())]
    for i in range(3):
        sample = PropagationSample(timestamp(BASE+i*10*MS), i, "mock-imu", "imu", (0, 0, 0), (0, 0, 9.81),
                                   "MOCK calibrated inputs", "VERIFIED")
        events.append(dict(type="imu", sample=asdict(sample), received_time=timestamp(i*10*MS, "MOCK_RECEIVE").to_dict()))
    for instant in (20*MS, 200*MS):
        rate = asdict(angular_velocity())
        rate["time"] = timestamp(BASE+20*MS).to_dict()
        events.append(dict(type="tick", publish_time=timestamp(instant, "MOCK_RECEIVE").to_dict(),
                           now_receive_time=timestamp(instant, "MOCK_RECEIVE").to_dict(), angular_velocity=rate))
    config_path.write_text(json.dumps(configuration, allow_nan=False))
    events_path.write_text("".join(json.dumps(event, allow_nan=False)+"\n" for event in events))
    return config_path, events_path


def test_recorded_jsonl_cli_runs_production_state_codec_and_loopback(tmp_path):
    config_path, events_path = write_replay_fixture(tmp_path)
    destination = tmp_path / "result"
    command = [sys.executable, "-m", "rm27.perception.vision.localization.state_output",
               "--config", str(config_path), "--events", str(events_path), "--output-dir", str(destination), "--mavlink-loopback"]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=20)
    assert completed.returncode == 0, completed.stderr + completed.stdout
    summary = json.loads((destination/"summary.json").read_text())
    assert summary["status"] == "PASS"
    assert summary["estimator_reproduction"] == "NOT_RUN"
    assert summary["counts"]["loopback_packets"] == 1
    assert summary["counts"]["suppressed_odometry"] == 1
    rows = [json.loads(line) for line in (destination/"states.jsonl").read_text().splitlines()]
    assert rows[-2]["state"]["estimate"]["translation"] == pytest.approx((1.02, 2, 3))
    assert not rows[-1]["state"]["usable"]
    assert rows[-1]["odometry"] is None
    original = (destination/"states.jsonl").read_bytes()
    rerun = subprocess.run(command, capture_output=True, text=True, timeout=20)
    assert rerun.returncode == 2
    assert (destination/"states.jsonl").read_bytes() == original


def test_jsonl_rejection_invalidates_health_and_records_failure(tmp_path):
    from rm27.perception.vision.localization.state_output import replay_recorded_events

    config_path, events_path = write_replay_fixture(tmp_path)
    events = [json.loads(line) for line in events_path.read_text().splitlines()]
    events[2]["sample"]["sequence"] = 0  # Preserve duplicate evidence; never sort/fill it.
    events_path.write_text("".join(json.dumps(event)+"\n" for event in events))
    result = replay_recorded_events(config_path, events_path, tmp_path/"rejected", mavlink_loopback=True)
    assert result["status"] == "FAIL"
    assert result["counts"]["rejected_events"] >= 1
    assert result["counts"]["loopback_packets"] == 0
    rows = [json.loads(line) for line in (tmp_path/"rejected"/"states.jsonl").read_text().splitlines()]
    assert not rows[-1]["state"]["usable"]


def test_sitl_listener_requires_received_topic_frames_vectors_and_reset():
    from tools.schurvins.verify_sitl import parse_received
    text = '''TOPIC: vehicle_visual_odometry instance 0 #1
  pose_frame: 1
  velocity_frame: 3
  position: [1.25, -2.5, 0.75]
  q: [0.7071068, 0, 0, 0.7071068]
  velocity: [0.1, 0.2, 0.3]
  angular_velocity: [0.01, 0.02, 0.03]
  reset_counter: 17
'''
    expected = dict(position=(1.25,-2.5,.75), q=(2**-.5,0,0,2**-.5),
                    velocity=(.1,.2,.3), angular_velocity=(.01,.02,.03), reset_counter=17)
    parse_received(text, expected)
    for before, after in [('TOPIC: vehicle_visual_odometry','TOPIC: other'),
                           ('pose_frame: 1','pose_frame: 2'),
                           ('velocity_frame: 3','velocity_frame: 1'),
                           ('reset_counter: 17','reset_counter: 18'),
                           ('1.25','nan'), ('0.1, 0.2, 0.3','0.1, 0.2, 0.4')]:
        with pytest.raises(ValueError):
            parse_received(text.replace(before,after), expected)
