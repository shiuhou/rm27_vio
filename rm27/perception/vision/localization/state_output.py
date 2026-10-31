"""Evidence-driven VIO output health and independent IMU snapshot propagation.

This is an offline/interface component, not a flight safety policy. Thresholds
are supplied by the experiment. A numerical pose or an old adapter's stage
does not establish initialization, metric scale, or an accepted visual update.
"""

from dataclasses import dataclass, replace
import math

import numpy as np

from .experiments.trajectory import rotation_wxyz
from .schema import LocalizationEstimate, ScaleState, TimestampEvidence, TrackingState


def _verified_time(value):
    if not isinstance(value, TimestampEvidence) or value.unit != "ns" or value.evidence_status != "VERIFIED":
        raise ValueError("verified integer nanosecond timestamp required")
    if value.clock_domain == "UNKNOWN" or value.semantic == "UNKNOWN":
        raise ValueError("timestamp clock and semantic must be known")
    return value.raw_value


def _elapsed(later, earlier):
    end, start = _verified_time(later), _verified_time(earlier)
    if later.clock_domain != earlier.clock_domain:
        raise ValueError("timestamp clocks differ; an explicit mapping is required")
    if end < start:
        raise ValueError("timestamp moved backwards")
    return end - start


def _vector(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} requires three finite numbers")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(f"{name} requires three finite numbers")
    return tuple(float(x) for x in value)


def _provenance(value, name):
    if not isinstance(value, str) or not value.strip() or value.upper() in ("UNKNOWN", "UNVERIFIED"):
        raise ValueError(f"{name} requires a known provenance")
    return value


def _quaternion_multiply(a, b):
    w, x, y, z = a
    s, u, v, t = b
    return (w*s-x*u-y*v-z*t, w*u+x*s+y*t-z*v,
            w*v-x*t+y*s+z*u, w*t+x*v-y*u+z*s)


def _rotation_increment(omega, seconds):
    magnitude = math.hypot(*omega)
    angle = magnitude * seconds
    if magnitude == 0:
        return (1.0, 0.0, 0.0, 0.0)
    scale = math.sin(angle / 2) / magnitude
    return (math.cos(angle / 2), *(float(x)*scale for x in omega))


@dataclass(frozen=True)
class StateOutputPolicy:
    """Explicit test limits; none is a qualified flight safety threshold."""

    max_visual_age_ns: int
    max_imu_gap_ns: int
    max_receive_age_ns: int
    max_visual_receive_age_ns: int
    max_buffer_samples: int

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class PropagationSample:
    """A calibrated raw specific-force/gyro sample, after input validation.

    Samples are held constant until the next sample (zero-order hold). The
    integrator uses midpoint orientation for acceleration. It neither fills
    input gaps nor alters sensor timestamps. Full raw-input metadata belongs
    to the platform-independent sensor event contract.
    """

    time: TimestampEvidence
    sequence: int
    imu_id: str
    imu_frame: str
    gyro_rad_s: tuple[float, float, float]
    specific_force_m_s2: tuple[float, float, float]
    calibration_source: str
    evidence_status: str = "UNKNOWN"

    def __post_init__(self):
        _verified_time(self.time)
        if type(self.sequence) is not int or self.sequence < 0:
            raise ValueError("IMU sequence must be a non-negative integer")
        for name in ("imu_id", "imu_frame", "calibration_source"):
            _provenance(getattr(self, name), name)
        if self.evidence_status != "VERIFIED":
            raise ValueError("IMU units, axes and sample semantics must be VERIFIED")
        object.__setattr__(self, "gyro_rad_s", _vector(self.gyro_rad_s, "gyro_rad_s"))
        object.__setattr__(self, "specific_force_m_s2", _vector(self.specific_force_m_s2, "specific_force_m_s2"))


@dataclass(frozen=True)
class NativeCorrectionEvidence:
    """A native accepted metric visual correction, never inferred from truth.

    Bias and gravity are native snapshot values with named sources. Covariance
    is not inferred from placeholders or propagated by this first-order helper.
    """

    estimate: LocalizationEstimate
    last_accepted_visual_time: TimestampEvidence
    correction_id: str
    initialization_source: str
    scale_source: str
    bias_source: str
    gravity_source: str
    gyro_bias_rad_s: tuple[float, float, float]
    accel_bias_m_s2: tuple[float, float, float]
    gravity_parent_m_s2: tuple[float, float, float]
    evidence_status: str = "UNKNOWN"

    def __post_init__(self):
        record = self.estimate
        if not isinstance(record, LocalizationEstimate):
            raise ValueError("native correction requires LocalizationEstimate v2")
        if self.evidence_status != "VERIFIED":
            raise ValueError("native initialization and accepted correction must be VERIFIED")
        if not record.valid or not record.initialized or record.tracking_state != TrackingState.TRACKING:
            raise ValueError("native correction must be initialized and tracking")
        if record.scale_state != ScaleState.METRIC or record.translation_unit != "m":
            raise ValueError("native metric scale is required; downstream scale fitting is forbidden")
        if record.velocity is None or record.velocity_frame != record.parent_frame or record.velocity_unit != "m/s":
            raise ValueError("native velocity in the parent frame is required")
        _verified_time(record.source_time)
        _verified_time(record.publish_time)
        _elapsed(record.source_time, self.last_accepted_visual_time)
        for name in ("correction_id", "initialization_source", "scale_source", "bias_source", "gravity_source"):
            _provenance(getattr(self, name), name)
        for name in ("gyro_bias_rad_s", "accel_bias_m_s2", "gravity_parent_m_s2"):
            object.__setattr__(self, name, _vector(getattr(self, name), name))


@dataclass(frozen=True)
class StateOutputResult:
    estimate: LocalizationEstimate | None
    usable: bool
    reasons: tuple[str, ...]
    localization_epoch: int | None
    last_accepted_visual_time: TimestampEvidence | None

    def to_dict(self):
        return dict(estimate=self.estimate.to_dict() if self.estimate is not None else None,
                    usable=self.usable, reasons=list(self.reasons), localization_epoch=self.localization_epoch,
                    last_accepted_visual_time=(self.last_accepted_visual_time.to_dict()
                                               if self.last_accepted_visual_time else None))


class StateOutputTracker:
    """One producer session; no estimator filter or ground truth is referenced.

    A correction replaces the propagation snapshot and replays only buffered
    IMU intervals after it. Every derived state is separate from that snapshot.
    Call ``output`` periodically even when sensors stop: its independent receive
    clock is the watchdog. Reset clears all cross-epoch IMU history.
    """

    def __init__(self, policy):
        if not isinstance(policy, StateOutputPolicy):
            raise ValueError("StateOutputPolicy is required")
        self.policy = policy
        self.epoch = None
        self._correction = None
        self._state = None
        self._samples = []
        self._anchor = None
        self._last_receive = None
        self._visual_receive = None
        self._fault = None
        self._kind = None
        self._sample_count = 0

    def _receive(self, time):
        _verified_time(time)
        if self._last_receive is not None:
            _elapsed(time, self._last_receive)
        self._last_receive = time

    def accept_correction(self, correction, *, received_time):
        if not isinstance(correction, NativeCorrectionEvidence):
            raise ValueError("NativeCorrectionEvidence is required")
        record = correction.estimate
        if self.epoch is not None and record.localization_epoch != self.epoch:
            raise ValueError("a new native epoch requires an explicit reset")
        if self._correction is not None:
            if _elapsed(correction.last_accepted_visual_time, self._correction.last_accepted_visual_time) == 0:
                raise ValueError("visual correction time must increase")
            if record.source_time.raw_value < self._correction.estimate.source_time.raw_value:
                raise ValueError("corrected state time moved backwards")
            previous = self._correction.estimate
            if (record.parent_frame, record.child_frame) != (previous.parent_frame, previous.child_frame):
                raise ValueError("frame changes require an explicit reset")
        for sample in self._samples:
            if sample.time.clock_domain != record.source_time.clock_domain:
                raise ValueError("IMU and correction clocks require an explicit mapping")
            if sample.imu_frame != record.child_frame:
                raise ValueError("IMU sample frame differs from state child frame")
        self._receive(received_time)
        self._visual_receive = received_time
        self.epoch = record.localization_epoch
        # Copy the contract, including its JSON metadata, to prevent caller mutation.
        self._state = LocalizationEstimate.from_json(record.to_json())
        self._correction = correction
        self._fault = None
        self._kind = "VISUAL_CORRECTION"
        self._sample_count = 0
        before = [s for s in self._samples if s.time.raw_value <= record.source_time.raw_value]
        self._anchor = before[-1] if before else None
        if self._anchor is not None and _elapsed(record.source_time, self._anchor.time) > self.policy.max_imu_gap_ns:
            self._anchor = None
        for sample in self._samples:
            if sample.time.raw_value > record.source_time.raw_value:
                self._integrate(sample)

    def push_imu(self, sample, *, received_time):
        if not isinstance(sample, PropagationSample):
            self._fault = "IMU_EVIDENCE_INVALID"
            raise ValueError("PropagationSample is required")
        try:
            self._receive(received_time)
            if self._samples:
                previous = self._samples[-1]
                gap = _elapsed(sample.time, previous.time)
                if gap == 0 or sample.sequence <= previous.sequence:
                    raise ValueError("duplicate or out-of-order IMU")
                if (sample.imu_id, sample.imu_frame, sample.calibration_source) != (
                        previous.imu_id, previous.imu_frame, previous.calibration_source):
                    raise ValueError("IMU identity/frame/calibration change requires reset")
                if sample.sequence != previous.sequence + 1 or gap > self.policy.max_imu_gap_ns:
                    raise ValueError("IMU sequence or sampling gap")
            if self._state is not None:
                if sample.imu_frame != self._state.child_frame:
                    raise ValueError("IMU sample frame differs from state child frame")
                if sample.time.clock_domain != self._state.source_time.clock_domain:
                    raise ValueError("IMU and state clocks require an explicit mapping")
            self._samples.append(sample)
            self._samples = self._samples[-self.policy.max_buffer_samples:]
            if self._state is not None:
                if sample.time.raw_value < self._state.source_time.raw_value:
                    # An IMU anchor before a new snapshot is not a state rewind.
                    if self._anchor is not None:
                        raise ValueError("IMU sample precedes current propagated state")
                    if _elapsed(self._state.source_time, sample.time) > self.policy.max_imu_gap_ns:
                        raise ValueError("IMU anchor is too old")
                    self._anchor = sample
                elif sample.time.raw_value == self._state.source_time.raw_value:
                    self._anchor = sample
                else:
                    self._integrate(sample)
        except ValueError:
            self._fault = "IMU_INPUT_INVALID_OR_GAP"
            raise

    def _integrate(self, sample):
        state = self._state
        if self._fault is not None:
            return
        if self._anchor is None:
            self._fault = "IMU_ANCHOR_UNAVAILABLE"
            return
        seconds_ns = _elapsed(sample.time, state.source_time)
        if seconds_ns > self.policy.max_imu_gap_ns:
            self._fault = "IMU_INPUT_INVALID_OR_GAP"
            return
        correction = self._correction
        seconds = seconds_ns / 1_000_000_000
        omega = np.array(self._anchor.gyro_rad_s) - correction.gyro_bias_rad_s
        force = np.array(self._anchor.specific_force_m_s2) - correction.accel_bias_m_s2
        midpoint = _quaternion_multiply(state.orientation_wxyz, _rotation_increment(omega, seconds / 2))
        acceleration = rotation_wxyz(midpoint) @ force + correction.gravity_parent_m_s2
        position = np.array(state.translation) + np.array(state.velocity)*seconds + acceleration*(seconds**2/2)
        velocity = np.array(state.velocity) + acceleration*seconds
        orientation = _quaternion_multiply(state.orientation_wxyz, _rotation_increment(omega, seconds))
        # Normalize only our own floating-point integration, never an input pose.
        norm = math.hypot(*orientation)
        orientation = tuple(x/norm for x in orientation)
        self._state = replace(state, source_time=replace(sample.time, semantic="STATE_ESTIMATE"),
                              translation=tuple(position), velocity=tuple(velocity), orientation_wxyz=orientation)
        self._anchor = sample
        self._kind = "IMU_PROPAGATION"
        self._sample_count += 1

    def mark_lost(self, reason):
        self._fault = _provenance(reason, "loss reason")

    def reset(self, new_epoch, *, reason, received_time):
        if type(new_epoch) is not int or new_epoch < 0 or (self.epoch is not None and new_epoch <= self.epoch):
            raise ValueError("reset must explicitly increase the native epoch")
        self._receive(received_time)
        self.epoch = new_epoch
        self._correction = self._state = self._anchor = None
        self._samples.clear()
        self._visual_receive = None
        self._sample_count = 0
        self._kind = None
        self._fault = "RESET:" + _provenance(reason, "reset reason")

    def output(self, *, publish_time, now_receive_time, sensor_now=None):
        """Compute fresh health, including when no sensor callbacks arrive."""
        _verified_time(publish_time)
        _verified_time(now_receive_time)
        if self._state is None:
            return StateOutputResult(None, False, (self._fault or "NATIVE_INITIALIZATION_UNKNOWN",), self.epoch, None)
        reasons = [self._fault] if self._fault else []
        state, correction = self._state, self._correction
        now_sensor = sensor_now if sensor_now is not None else state.source_time
        _elapsed(now_sensor, state.source_time)
        if _elapsed(now_sensor, correction.last_accepted_visual_time) > self.policy.max_visual_age_ns:
            reasons.append("VISUAL_CORRECTION_STALE")
        if _elapsed(now_receive_time, self._last_receive) > self.policy.max_receive_age_ns:
            reasons.append("SENSOR_STREAM_STOPPED")
        if _elapsed(now_receive_time, self._visual_receive) > self.policy.max_visual_receive_age_ns:
            reasons.append("VISUAL_STREAM_STOPPED")
        usable = not reasons
        quality = dict(state.quality or {})
        quality["state_output"] = dict(
            schema_version=1, evidence_status="VERIFIED", state_source=self._kind,
            last_accepted_visual_time=correction.last_accepted_visual_time.to_dict(),
            correction_id=correction.correction_id, initialization_source=correction.initialization_source,
            scale_source=correction.scale_source, bias_source=correction.bias_source,
            gravity_source=correction.gravity_source, health_reasons=reasons,
            propagated_intervals=self._sample_count,
            integration_method=("ZOH_SPECIFIC_FORCE_MIDPOINT_ORIENTATION" if self._sample_count else None),
            covariance_status=("NOT_PROPAGATED" if self._sample_count else "UNQUALIFIED"),
            receive_time=self._last_receive.to_dict(),
            visual_receive_time=self._visual_receive.to_dict(),
            watchdog_time=now_receive_time.to_dict(),
            health_limits_ns=dict(max_receive_age_ns=self.policy.max_receive_age_ns,
                                  max_visual_receive_age_ns=self.policy.max_visual_receive_age_ns),
        )
        result = replace(state, publish_time=publish_time,
                         tracking_state=TrackingState.TRACKING if usable else TrackingState.LOST,
                         valid=usable, quality=quality)
        return StateOutputResult(result, usable, tuple(reasons), self.epoch, correction.last_accepted_visual_time)


def replay_recorded_events(config_path, events_path, output_dir, *, mavlink_loopback=False):
    """Replay explicit evidence JSONL; this never runs or qualifies a VIO estimator.

    Event types are ``correction`` (NativeCorrectionEvidence dictionary), ``imu``
    (PropagationSample dictionary), ``reset``, ``lost``, and ``tick``. A tick
    supplies current publication/receive time and optional sensor-now time.
    A loopback tick additionally supplies AngularVelocityEvidence. This path
    intentionally records rejected inputs and exits unsuccessfully for them.
    """
    import hashlib
    import json
    from pathlib import Path

    from .mavlink import (
        AngularVelocityEvidence, EpochResetCounter, MavlinkTimeMapping, OdometryTransformEvidence,
        RigidTransformEvidence, prepare_odometry, verify_localhost_loopback,
    )

    def read_json(text):
        def bad_constant(value):
            raise ValueError(f"non-finite JSON constant {value} is prohibited")
        return json.loads(text, parse_constant=bad_constant)

    config_path, events_path, destination = Path(config_path), Path(events_path), Path(output_dir)
    configuration = read_json(config_path.read_text())
    instance = StateOutputTracker(StateOutputPolicy(**configuration["policy"]))
    transforms = mapping = counter = None
    if mavlink_loopback:
        odometry = configuration["odometry"]
        transforms = OdometryTransformEvidence(**{
            name: RigidTransformEvidence(**odometry[name]) for name in ("ned_from_world", "imu_from_body")})
        mapping = MavlinkTimeMapping(**odometry["time_mapping"])
        counter = EpochResetCounter(odometry["epoch_origin"], odometry.get("counter_origin", 0))
    destination.mkdir(parents=True, exist_ok=False)
    counts = dict(events=0, rejected_events=0, state_outputs=0, usable_state_outputs=0,
                  suppressed_odometry=0, loopback_packets=0)
    with events_path.open() as source, (destination / "states.jsonl").open("x") as target:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            counts["events"] += 1
            row = dict(line=line_number)
            event_type = None
            try:
                event = read_json(line)
                event_type = row["event_type"] = event["type"]
                if event_type == "correction":
                    raw = dict(event["correction"])
                    raw["estimate"] = LocalizationEstimate.from_dict(raw["estimate"])
                    raw["last_accepted_visual_time"] = TimestampEvidence.from_dict(raw["last_accepted_visual_time"])
                    instance.accept_correction(NativeCorrectionEvidence(**raw),
                                               received_time=TimestampEvidence.from_dict(event["received_time"]))
                elif event_type == "imu":
                    raw = dict(event["sample"])
                    raw["time"] = TimestampEvidence.from_dict(raw["time"])
                    instance.push_imu(PropagationSample(**raw), received_time=TimestampEvidence.from_dict(event["received_time"]))
                elif event_type == "reset":
                    instance.reset(event["new_epoch"], reason=event["reason"],
                                   received_time=TimestampEvidence.from_dict(event["received_time"]))
                elif event_type == "lost":
                    instance.mark_lost(event["reason"])
                elif event_type == "tick":
                    now_receive = TimestampEvidence.from_dict(event["now_receive_time"])
                    state = instance.output(publish_time=TimestampEvidence.from_dict(event["publish_time"]),
                                            now_receive_time=now_receive,
                                            sensor_now=(TimestampEvidence.from_dict(event["sensor_now"])
                                                        if "sensor_now" in event else None))
                    row["state"] = state.to_dict()
                    counts["state_outputs"] += 1
                    counts["usable_state_outputs"] += int(state.usable)
                    if mavlink_loopback:
                        payload = None
                        if state.usable:
                            raw = dict(event["angular_velocity"])
                            raw["time"] = TimestampEvidence.from_dict(raw["time"])
                            payload = prepare_odometry(state, transforms=transforms, time_mapping=mapping,
                                                       angular_velocity=AngularVelocityEvidence(**raw),
                                                       reset_counter=counter.observe(state.localization_epoch),
                                                       now_receive_time=now_receive)
                        if payload is None:
                            counts["suppressed_odometry"] += 1
                            row["odometry"] = None
                        else:
                            row["odometry"] = payload.to_dict()
                            row["loopback"] = verify_localhost_loopback(payload)
                            counts["loopback_packets"] += 1
                else:
                    raise ValueError("unknown recorded event type")
                row["status"] = "PASS"
            except (ValueError, TypeError, KeyError, RuntimeError, OSError, ImportError) as exc:
                if event_type != "tick":
                    instance.mark_lost("RECORDED_INPUT_REJECTED")
                row.update(status="FAIL", error=f"{type(exc).__name__}: {exc}")
                counts["rejected_events"] += 1
            target.write(json.dumps(row, allow_nan=False, sort_keys=True) + "\n")
    status = "FAIL" if counts["rejected_events"] else ("PASS" if counts["state_outputs"] else "NOT_RUN")
    summary = dict(status=status, evidence_class="EMULATED / MOCK VERIFIED",
                   verification_scope="STATE_CONTRACT_REPLAY_AND_OPTIONAL_LOCALHOST_ONLY",
                   estimator_reproduction="NOT_RUN", hardware="NOT_RUN", counts=counts,
                   config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                   events_sha256=hashlib.sha256(events_path.read_bytes()).hexdigest())
    (destination / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary


def main(argv=None):
    import argparse
    import json

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--events", required=True)
    parser.add_argument("--output-dir", required=True, help="Must not exist; evidence is never overwritten")
    parser.add_argument("--mavlink-loopback", action="store_true", help="Verify real MAVLink2 via owned localhost UDP sockets")
    args = parser.parse_args(argv)
    try:
        summary = replay_recorded_events(args.config, args.events, args.output_dir, mavlink_loopback=args.mavlink_loopback)
    except (ValueError, TypeError, KeyError, OSError, RuntimeError, ImportError) as exc:
        parser.exit(2, f"state replay failed: {exc}\n")
    print(json.dumps(summary, allow_nan=False))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    # The codec imports this module's public types. Execute the canonical module
    # instance so ``python -m`` and ordinary imports share those identities.
    from rm27.perception.vision.localization.state_output import main as canonical_main
    raise SystemExit(canonical_main())
