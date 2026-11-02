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


def test_odometry_emission_clock_cannot_precede_state_watchdog_evaluation():
    instance = StateOutputTracker(StateOutputPolicy(100*MS, 20*MS, 50*MS, 150*MS, 100))
    instance.accept_correction(native_correction(), received_time=timestamp(0, "MOCK_RECEIVE"))
    state = instance.output(publish_time=timestamp(10*MS, "MOCK_RECEIVE"),
                            now_receive_time=timestamp(10*MS, "MOCK_RECEIVE"))
    with pytest.raises(ValueError, match="backwards"):
        payload(state, now=9*MS)


def laboratory_sample():
    """Declared MOCK of sealed source shape; not a native qualification."""
    summary = dict(schema_version=1, status='PASS', truth_used=False,
        scope='PC_OFFLINE_NATIVE_OBSERVABILITY_AND_CONFIGURED_FRESHNESS_ONLY',
        session_id='MOCK laboratory source',
        native_clock_semantic='PROCESS_LOCAL_STEADY_EVENT_SERIALIZATION_NOT_HARDWARE_RECEIPT',
        adapter_epoch_semantic='ONE_ISOLATED_PROCESS_SESSION_NOT_NATIVE_RESET_COUNT',
        policy=asdict(StateOutputPolicy(100*MS, 20*MS, 50*MS, 150*MS, 100)),
        initialization=dict(commit_observed=True, ready_observed=True),
        covariance=dict(status='UNQUALIFIED', native_finite_witness_only=True),
        native_reset=dict(status='BLOCKED', execution='NOT_RUN'),
        operational_health='UNKNOWN', operational_valid=False)
    health = dict(native_event_sequence=10, native_observation_ns=200*MS,
        session_id=summary['session_id'], adapter_epoch=0, native_epoch=None, native_reset_count=None,
        lifecycle='INITIALIZED', freshness='CURRENT', freshness_reasons=[], state_source='IMU_PREDICTION',
        state_time=dict(seconds=BASE/10**9, request_sensor_ns=BASE,
                        semantic='NATIVE_FAST_PROPAGATION_INTEGRATION_ENDPOINT'),
        last_sensor_observation_ns=195*MS, last_imu_observation_ns=195*MS,
        last_state_progress_observation_ns=195*MS, last_visual_observation_ns=195*MS,
        last_visual_sensor_ns=BASE, sensor_clock='MOCK sensor clock', tracking_state='UNKNOWN',
        valid=False, contract_complete=False, operational_health='UNKNOWN', native_publication_timestamp=None)
    return dict(odometry=dict(state_ns=BASE, frame_id='global', child_frame_id='imu', position=[1, 2, 3],
                quaternion_xyzw=[0, 0, 0, 1], velocity_local=[.1, .2, .3],
                angular_velocity_local=[.01, .02, .03]),
                health_row=health, health_summary=summary,
                source=dict(scope='MOCK sealed I/O for conversion test, not estimator evidence'))


def laboratory_payload():
    from rm27.perception.vision.localization.mavlink import prepare_observer_only_odometry
    return prepare_observer_only_odometry(laboratory_sample(), laboratory_mode=True)


def laboratory_records():
    from rm27.perception.vision.localization.state_output import evaluate_openvins_operational_validity
    sample = laboratory_sample()
    return dict(schema_version=1, mode='OBSERVER_ONLY_LABORATORY', qualification='UNQUALIFIED',
                records=[dict(case='MOCK-normal', payload=laboratory_payload().to_dict(),
                              policy=evaluate_openvins_operational_validity(
                                  sample['health_row'], sample['health_summary']).to_dict(),
                              source=sample['source'])])


def test_laboratory_conversion_is_disabled_by_default_and_does_not_upgrade_health():
    from copy import deepcopy
    from rm27.perception.vision.localization.mavlink import prepare_observer_only_odometry
    sample = laboratory_sample()
    original = deepcopy(sample)
    for mode in (False, None, 1, 'yes'):
        with pytest.raises(ValueError, match='disabled'):
            prepare_observer_only_odometry(sample, laboratory_mode=mode)
    actual = prepare_observer_only_odometry(sample, laboratory_mode=True)
    assert sample == original
    assert actual.frame_id == 20 and actual.to_dict()['child_frame_id'] == 12
    assert actual.position_ned_m == pytest.approx((1, -2, -3))
    assert actual.orientation_ned_body_wxyz == pytest.approx((0, 1, 0, 0))
    assert actual.velocity_body_m_s == pytest.approx((.1, .2, .3))
    assert actual.angular_velocity_body_rad_s == pytest.approx((.01, .02, .03))
    assert actual.time_usec == BASE//1000
    assert actual.evidence['microsecond_quantization_residual_ns'] == BASE % 1000
    assert actual.reset_counter == 0 and actual.evidence['native_reset_count'] is None
    assert not actual.evidence['control_use_permitted']
    assert not actual.evidence['arming_permitted']
    assert not actual.evidence['operational_policy']['output_eligible']
    assert actual.evidence['operational_policy']['tracking_health'] == 'UNKNOWN'
    assert actual.to_dict()['pose_covariance'] is actual.to_dict()['velocity_covariance'] is None
    json.dumps(actual.to_dict(), allow_nan=False)


def test_laboratory_quaternion_is_reordered_without_conjugation_or_velocity_frame_change():
    from rm27.perception.vision.localization.mavlink import prepare_observer_only_odometry
    sample = laboratory_sample()
    sample['odometry']['quaternion_xyzw'] = [0, 0, math.sqrt(.5), math.sqrt(.5)]
    actual = prepare_observer_only_odometry(sample, laboratory_mode=True)
    assert actual.orientation_ned_body_wxyz == pytest.approx((0, math.sqrt(.5), -math.sqrt(.5), 0))
    assert actual.velocity_body_m_s == pytest.approx((.1, .2, .3))
    assert actual.angular_velocity_body_rad_s == pytest.approx((.01, .02, .03))


@pytest.mark.parametrize('field,value', [('state_ns', True), ('state_ns', -1), ('frame_id', 'NED'),
    ('child_frame_id', 'camera'), ('velocity_local', None), ('angular_velocity_local', None),
    ('angular_velocity_local', [0, 0, float('nan')]), ('position', [True, 0, 0]),
    ('quaternion_xyzw', [0, 0, 0, 0])])
def test_laboratory_conversion_refuses_missing_or_invalid_native_fields(field, value):
    from rm27.perception.vision.localization.mavlink import prepare_observer_only_odometry
    sample = laboratory_sample()
    sample['odometry'][field] = value
    with pytest.raises(ValueError):
        prepare_observer_only_odometry(sample, laboratory_mode=True)


def test_local_frd_payload_cannot_be_mislabeled_as_operational_ned():
    with pytest.raises(ValueError, match='observer evidence'):
        replace(payload(), frame_id=20)
    with pytest.raises(ValueError, match='unsupported local'):
        replace(payload(), frame_id=2)


def test_laboratory_packet_preserves_integer_header_and_float32_values():
    from tools.schurvins.verify_sitl import verify_wire
    actual = laboratory_payload()
    codec = MavlinkOdometryCodec()
    packet = codec.encode(actual)
    decoded = codec.decode(packet)
    assert decoded.frame_id == 20 and decoded.child_frame_id == 12
    assert decoded.time_usec == BASE//1000
    assert decoded.reset_counter == decoded.quality == 0
    assert math.isnan(decoded.pose_covariance[0]) and math.isnan(decoded.velocity_covariance[0])
    assert verify_wire(codec, packet, actual)['covariance_sentinel_verified']
    for field, value in [('time_usec', actual.time_usec+1), ('position_ned_m', (2, 3, 4)),
                          ('velocity_body_m_s', (9, 9, 9)), ('reset_counter', 1)]:
        with pytest.raises(ValueError, match='wire'):
            verify_wire(codec, packet, replace(actual, **{field:value}))


def test_laboratory_records_need_both_opt_ins_before_any_transport(tmp_path):
    from tools.schurvins.verify_sitl import load_laboratory_records
    path = tmp_path/'records.json'
    path.write_text(json.dumps(laboratory_records(), allow_nan=False))
    assert load_laboratory_records(None, enabled=False) is None
    with pytest.raises(ValueError, match='requires laboratory records'):
        load_laboratory_records(None, enabled=True)
    with pytest.raises(ValueError, match='require --observer'):
        load_laboratory_records(path, enabled=False)
    assert load_laboratory_records(path, enabled=True)['records'][0]['policy']['output_eligible'] is False


@pytest.mark.parametrize('lane,value', [('frame_id', 1), ('child_frame_id', 1), ('quality', 100),
    ('reset_counter', 1), ('pose_covariance', [0]*21), ('velocity_covariance', [0]*21),
    ('angular_velocity_body_rad_s', None), ('time_usec', True)])
def test_laboratory_loader_refuses_unsafe_payload_changes(tmp_path, lane, value):
    from tools.schurvins.verify_sitl import load_laboratory_records
    value_json = laboratory_records()
    value_json['records'][0]['payload'][lane] = value
    path = tmp_path/'records.json'
    path.write_text(json.dumps(value_json, allow_nan=False))
    with pytest.raises(ValueError):
        load_laboratory_records(path, enabled=True)


@pytest.mark.parametrize('lane', ['qualified', 'eligible', 'control', 'duplicate', 'empty', 'unbounded'])
def test_laboratory_loader_refuses_qualification_and_scope_escalation(tmp_path, lane):
    from tools.schurvins.verify_sitl import load_laboratory_records
    value = laboratory_records()
    if lane == 'qualified': value['qualification'] = 'QUALIFIED'
    if lane == 'eligible': value['records'][0]['policy']['output_eligible'] = True
    if lane == 'control': value['records'][0]['payload']['evidence']['control_use_permitted'] = True
    if lane == 'duplicate': value['records'] *= 2
    if lane == 'empty': value['records'] = []
    if lane == 'unbounded': value['records'] *= 13
    path = tmp_path/'records.json'
    path.write_text(json.dumps(value, allow_nan=False))
    with pytest.raises(ValueError): load_laboratory_records(path, enabled=True)


def test_laboratory_listener_requires_actual_frames_quality_and_variance_sentinels():
    from tools.schurvins.verify_sitl import parse_received
    text = '''TOPIC: vehicle_visual_odometry instance 0 #1
  timestamp: 0
  timestamp_sample: 0
  pose_frame: 2
  velocity_frame: 3
  position: [1, -2, -3]
  q: [0, 1, 0, 0]
  velocity: [0.1, 0.2, 0.3]
  angular_velocity: [0.01, 0.02, 0.03]
  position_variance: [nan, 0, 0]
  orientation_variance: [0, 0, 0]
  velocity_variance: [nan, 0, 0]
  reset_counter: 0
  quality: 0
'''
    expected = dict(position=[1, -2, -3], q=[0, 1, 0, 0], velocity=[.1, .2, .3],
        angular_velocity=[.01, .02, .03], position_variance=[None, 0, 0],
        orientation_variance=[0, 0, 0], velocity_variance=[None, 0, 0],
        pose_frame=2, velocity_frame=3, reset_counter=0, quality=0)
    parse_received(text, expected)
    for before, after in [('pose_frame: 2', 'pose_frame: 1'), ('quality: 0', 'quality: 100'),
                          ('[nan, 0, 0]', '[0, 0, 0]'), ('[0.1, 0.2, 0.3]', '[0.1, 0.2, 0.4]')]:
        with pytest.raises(ValueError): parse_received(text.replace(before, after), expected)


def test_portable_sitl_artifact_lock_requires_exact_binary_dependencies_and_image(tmp_path):
    from tools.schurvins.verify_sitl import digest, load_artifact_lock
    binary, version, dep = (tmp_path/name for name in ['px4', 'version.h', 'libmock.so'])
    for p in [binary, version, dep]: p.write_text('MOCK immutable dependency '+p.name)
    image = 'sha256:'+'1'*64
    value = dict(schema_version=1, purpose='PX4_SITL_RECEPTION_ARTIFACT',
        qualification='ARCHIVED_FIXED_ARTIFACT_NOT_HARDWARE', source_head='2'*40, image_id=image,
        binary=dict(path=str(binary), sha256=digest(binary)),
        build_version=dict(path=str(version), sha256=digest(version)),
        dynamic_dependencies_sha256={str(dep):digest(dep)})
    path = tmp_path/'lock.json'
    path.write_text(json.dumps(value))
    assert load_artifact_lock(path, image) == value
    with pytest.raises(ValueError): load_artifact_lock(path, 'sha256:'+'3'*64)
    binary.write_text('changed')
    with pytest.raises(ValueError, match='hash differs'): load_artifact_lock(path, image)
    binary.write_text('MOCK immutable dependency px4')
    dep.write_text('changed')
    with pytest.raises(ValueError, match='dependency hash'): load_artifact_lock(path, image)


def test_sitl_existing_output_is_refused_before_artifact_or_container_access(tmp_path):
    from tools.schurvins.verify_sitl import main
    with pytest.raises(FileExistsError):
        main(['--artifact-lock', str(tmp_path/'missing-lock.json'), '--image', 'sha256:'+'1'*64,
              '--output', str(tmp_path)])


def test_pinned_ekf2_absence_uses_explicit_native_stop_probe_without_starting_estimator():
    from tools.schurvins.verify_sitl import module_absence_command, verify_module_absence
    assert module_absence_command('commander') == 'status'
    assert module_absence_command('ekf2') == 'stop'
    verify_module_absence('ekf2', 'stop', 0, 'WARN  [px4] not running\n')
    verify_module_absence('commander', 'status', 1, 'INFO  [px4] not running\n')
    with pytest.raises(ValueError): module_absence_command('other')


@pytest.mark.parametrize('command,code,text', [('status', 0, ''), ('stop', 0, ''),
    ('stop', 1, 'WARN  [px4] not running\n'),
    ('stop', 0, 'WARN  [px4] module locked, try again later\n'),
    ('stop', 0, 'INFO  [px4] stopping ekf2 instance 0\nWARN  [px4] not running\n'),
    ('stop', 0, 'not running'), ('stop', 0, 'INFO [other] not running')])
def test_module_absence_never_accepts_empty_ambiguous_locked_or_stopped_present_instance(command, code, text):
    from tools.schurvins.verify_sitl import verify_module_absence
    with pytest.raises(RuntimeError, match='absence'):
        verify_module_absence('ekf2', command, code, text)
