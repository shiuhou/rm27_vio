"""Pure VIO-to-ODOMETRY conversion and localhost-only MAVLink2 verification.

No serial device, autopilot, or external network endpoint is opened. MAVLink is
imported lazily and pinned independently of the estimator environment. Unknown
covariances remain ``None`` in strict JSON and become NaN only on the wire, as
specified by the ODOMETRY definition in the pinned common.xml dialect.
"""

from dataclasses import dataclass
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import socket

import numpy as np

from .experiments.trajectory import rotation_wxyz
from .schema import ScaleState, TimestampEvidence, TrackingState
from .state_output import (
    StateOutputResult, _elapsed, _provenance, _quaternion_multiply, _vector, _verified_time,
)

PYMAVLINK_VERSION = "2.4.49"
LOCAL_NED = 1
BODY_FRD = 12
VISION_ESTIMATOR = 2


@dataclass(frozen=True)
class RigidTransformEvidence:
    """T_parent_child maps child coordinates into parent; translation in m."""

    parent_frame: str
    child_frame: str
    translation_m: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    provenance: str
    evidence_status: str = "UNKNOWN"

    def __post_init__(self):
        for name in ("parent_frame", "child_frame", "provenance"):
            _provenance(getattr(self, name), name)
        if self.evidence_status != "VERIFIED":
            raise ValueError("transform and alignment evidence must be VERIFIED")
        object.__setattr__(self, "translation_m", _vector(self.translation_m, "translation_m"))
        rotation_wxyz(self.orientation_wxyz)
        object.__setattr__(self, "orientation_wxyz", tuple(float(x) for x in self.orientation_wxyz))


@dataclass(frozen=True)
class OdometryTransformEvidence:
    ned_from_world: RigidTransformEvidence
    imu_from_body: RigidTransformEvidence

    def __post_init__(self):
        if not isinstance(self.ned_from_world, RigidTransformEvidence) or not isinstance(self.imu_from_body, RigidTransformEvidence):
            raise ValueError("verified world alignment and body extrinsics are required")
        if self.ned_from_world.parent_frame != "LOCAL_NED" or self.imu_from_body.child_frame != "BODY_FRD":
            raise ValueError("explicit LOCAL_NED alignment and BODY_FRD body axes are required")


@dataclass(frozen=True)
class MavlinkTimeMapping:
    """An evidenced fixed offset; no clock fit is inferred by the encoder."""

    source_clock_domain: str
    target_clock_domain: str
    offset_ns: int
    provenance: str
    evidence_status: str = "UNKNOWN"

    def __post_init__(self):
        _provenance(self.source_clock_domain, "source clock")
        _provenance(self.provenance, "clock mapping provenance")
        if self.target_clock_domain not in ("UNIX", "FC_BOOT") or self.evidence_status != "VERIFIED":
            raise ValueError("verified mapping to UNIX or FC_BOOT time is required")
        if type(self.offset_ns) is not int:
            raise ValueError("clock offset must be integer nanoseconds")

    def map(self, time):
        value = _verified_time(time)
        if time.clock_domain != self.source_clock_domain:
            raise ValueError("source clock does not match the explicit mapping")
        mapped = value + self.offset_ns
        if mapped < 0 or mapped // 1000 >= 2**64:
            raise ValueError("mapped MAVLink timestamp is out of uint64 range")
        return divmod(mapped, 1000)


@dataclass(frozen=True)
class AngularVelocityEvidence:
    time: TimestampEvidence
    frame: str
    rad_s: tuple[float, float, float]
    provenance: str
    evidence_status: str = "UNKNOWN"

    def __post_init__(self):
        _verified_time(self.time)
        _provenance(self.frame, "angular velocity frame")
        _provenance(self.provenance, "angular velocity provenance")
        if self.evidence_status != "VERIFIED":
            raise ValueError("measured/debiased angular velocity evidence must be VERIFIED")
        object.__setattr__(self, "rad_s", _vector(self.rad_s, "angular velocity"))


@dataclass(frozen=True)
class OdometryPayload:
    time_usec: int
    position_ned_m: tuple[float, float, float]
    orientation_ned_body_wxyz: tuple[float, float, float, float]
    velocity_body_m_s: tuple[float, float, float]
    angular_velocity_body_rad_s: tuple[float, float, float]
    reset_counter: int
    evidence: dict

    def __post_init__(self):
        if type(self.time_usec) is not int or not 0 <= self.time_usec < 2**64:
            raise ValueError("time_usec must be uint64")
        if type(self.reset_counter) is not int or not 0 <= self.reset_counter <= 255:
            raise ValueError("reset counter must be uint8")
        for name in ("position_ned_m", "velocity_body_m_s", "angular_velocity_body_rad_s"):
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        rotation_wxyz(self.orientation_ned_body_wxyz)
        object.__setattr__(self, "orientation_ned_body_wxyz", tuple(float(x) for x in self.orientation_ned_body_wxyz))
        try:
            json.dumps(self.evidence, allow_nan=False)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("ODOMETRY evidence must be strict JSON") from exc

    def to_dict(self):
        return dict(time_usec=self.time_usec, frame_id=LOCAL_NED, child_frame_id=BODY_FRD,
                    position_ned_m=list(self.position_ned_m),
                    orientation_ned_body_wxyz=list(self.orientation_ned_body_wxyz),
                    velocity_body_m_s=list(self.velocity_body_m_s),
                    angular_velocity_body_rad_s=list(self.angular_velocity_body_rad_s),
                    pose_covariance=None, velocity_covariance=None,
                    reset_counter=self.reset_counter, estimator_type=VISION_ESTIMATOR,
                    quality=0, evidence=dict(self.evidence))


def prepare_odometry(state, *, transforms, time_mapping, angular_velocity, reset_counter, now_receive_time):
    """Suppress unhealthy output; otherwise transform IMU origin to BODY_FRD.

    v_body_origin = v_imu_origin + R_world_imu (omega_imu x t_imu_body).
    Twist is expressed in BODY_FRD. Pose is expressed in explicitly aligned
    LOCAL_NED. No absent velocity, rate, direction or clock is filled with zero.
    Covariance remains unknown until a full sourced covariance/Jacobian path is
    implemented; zero estimator placeholders are never used.
    """
    if not isinstance(state, StateOutputResult):
        raise ValueError("an evidence-driven StateOutputResult is required")
    if not state.usable or state.estimate is None:
        return None
    record = state.estimate
    metadata = (record.quality or {}).get("state_output", {})
    if (not record.valid or not record.initialized or record.tracking_state != TrackingState.TRACKING
            or state.reasons or metadata.get("evidence_status") != "VERIFIED"
            or metadata.get("health_reasons") != []
            or metadata.get("state_source") not in ("VISUAL_CORRECTION", "IMU_PROPAGATION")):
        raise ValueError("state lacks verified healthy native output evidence")
    # Recheck at the transport boundary: a once-healthy object can become stale
    # while waiting in a queue, without another sensor callback.
    limits = metadata.get("health_limits_ns", {})
    for stamp_name, limit_name in (("receive_time", "max_receive_age_ns"),
                                   ("visual_receive_time", "max_visual_receive_age_ns")):
        limit = limits.get(limit_name)
        if type(limit) is not int or limit <= 0 or stamp_name not in metadata:
            raise ValueError("explicit watchdog evidence is required at emission")
        if _elapsed(now_receive_time, TimestampEvidence.from_dict(metadata[stamp_name])) > limit:
            return None
    if record.scale_state != ScaleState.METRIC or record.translation_unit != "m":
        raise ValueError("metric state is required")
    if record.velocity is None or record.velocity_unit != "m/s":
        raise ValueError("native velocity is required; zero substitution is forbidden")
    if not isinstance(transforms, OdometryTransformEvidence) or not isinstance(time_mapping, MavlinkTimeMapping):
        raise ValueError("explicit frame and timestamp mappings are required")
    if not isinstance(angular_velocity, AngularVelocityEvidence):
        raise ValueError("angular velocity evidence is required")
    if _elapsed(record.source_time, angular_velocity.time) != 0 or angular_velocity.frame != record.child_frame:
        raise ValueError("angular velocity must match the state time and IMU frame")
    world, body = transforms.ned_from_world, transforms.imu_from_body
    if world.child_frame != record.parent_frame or body.parent_frame != record.child_frame:
        raise ValueError("transform frames do not match the state")
    r_nw = rotation_wxyz(world.orientation_wxyz)
    r_wi = rotation_wxyz(record.orientation_wxyz)
    r_ib = rotation_wxyz(body.orientation_wxyz)
    r_nb = r_nw @ r_wi @ r_ib
    lever_arm = np.array(body.translation_m)
    position = r_nw @ (np.array(record.translation) + r_wi @ lever_arm) + world.translation_m
    velocity_world = np.array(record.velocity)
    if record.velocity_frame == record.child_frame:
        velocity_world = r_wi @ velocity_world
    elif record.velocity_frame != record.parent_frame:
        raise ValueError("velocity frame requires an explicit mapping")
    omega_imu = np.array(angular_velocity.rad_s)
    velocity_world += r_wi @ np.cross(omega_imu, lever_arm)
    velocity_body = r_nb.T @ r_nw @ velocity_world
    omega_body = r_ib.T @ omega_imu
    orientation = _quaternion_multiply(_quaternion_multiply(world.orientation_wxyz, record.orientation_wxyz), body.orientation_wxyz)
    time_usec, residual_ns = time_mapping.map(record.source_time)
    return OdometryPayload(time_usec, tuple(position), orientation, tuple(velocity_body), tuple(omega_body),
                           reset_counter, dict(
                               state_time=record.source_time.to_dict(), publication_time=record.publish_time.to_dict(),
                               last_accepted_visual_time=state.last_accepted_visual_time.to_dict(),
                               state_source=metadata["state_source"], native_epoch=record.localization_epoch,
                               world_alignment_source=world.provenance, body_calibration_source=body.provenance,
                               angular_velocity_source=angular_velocity.provenance,
                               time_mapping_source=time_mapping.provenance,
                               time_clock_domain=time_mapping.target_clock_domain,
                               microsecond_quantization_residual_ns=residual_ns,
                               covariance_status="UNKNOWN_NOT_TRANSFORMED"))


class EpochResetCounter:
    """Map an observed native epoch counter to MAVLink's modulo-256 counter."""

    def __init__(self, epoch_origin, counter_origin=0):
        if type(epoch_origin) is not int or epoch_origin < 0:
            raise ValueError("epoch origin must be a non-negative native epoch")
        if type(counter_origin) is not int or not 0 <= counter_origin < 256:
            raise ValueError("counter origin must be uint8")
        self.epoch_origin, self.counter_origin = epoch_origin, counter_origin
        self._last_epoch = epoch_origin

    def observe(self, native_epoch):
        if type(native_epoch) is not int or native_epoch < self._last_epoch:
            raise ValueError("native epoch must not decrease")
        self._last_epoch = native_epoch
        return (self.counter_origin + native_epoch - self.epoch_origin) % 256


class MavlinkOdometryCodec:
    """Pinned real MAVLink2 encoding; no hardware transport is constructed."""

    def __init__(self, *, source_system=1, source_component=191):
        version = importlib.metadata.version("pymavlink")
        if version != PYMAVLINK_VERSION:
            raise RuntimeError(f"pymavlink {PYMAVLINK_VERSION} required; installed {version}")
        from pymavlink.dialects.v20 import common
        for name, value in (("MAV_FRAME_LOCAL_NED", LOCAL_NED), ("MAV_FRAME_BODY_FRD", BODY_FRD),
                            ("MAV_ESTIMATOR_TYPE_VISION", VISION_ESTIMATOR)):
            if getattr(common, name) != value:
                raise RuntimeError("pinned MAVLink dialect constants changed")
        for name, value in (("source_system", source_system), ("source_component", source_component)):
            if type(value) is not int or not 1 <= value <= 255:
                raise ValueError(f"{name} must be 1..255")
        self._dialect = common
        self._encoder = common.MAVLink(None, srcSystem=source_system, srcComponent=source_component)
        module_file = Path(common.__file__)
        definition_file = module_file.with_suffix(".xml")
        self.provenance = dict(pymavlink_version=version, dialect="v20.common",
                               generated_source_sha256=hashlib.sha256(module_file.read_bytes()).hexdigest(),
                               message_definition_sha256=hashlib.sha256(definition_file.read_bytes()).hexdigest(),
                               evidence_class="EMULATED / MOCK VERIFIED")

    def encode(self, payload):
        if not isinstance(payload, OdometryPayload):
            raise ValueError("OdometryPayload is required")
        unknown_covariance = [math.nan] + [0.0]*20
        message = self._encoder.odometry_encode(
            payload.time_usec, LOCAL_NED, BODY_FRD, *payload.position_ned_m,
            payload.orientation_ned_body_wxyz, *payload.velocity_body_m_s,
            *payload.angular_velocity_body_rad_s, unknown_covariance, unknown_covariance,
            payload.reset_counter, VISION_ESTIMATOR, 0,
        )
        packet = message.pack(self._encoder, force_mavlink1=False)
        self._encoder.seq = (self._encoder.seq + 1) % 256
        return packet

    def decode(self, packet):
        if not isinstance(packet, bytes) or not packet or packet[0] != 0xFD:
            raise ValueError("a complete MAVLink2 packet is required")
        parser = self._dialect.MAVLink(None)
        messages = parser.parse_buffer(packet) or []
        if len(messages) != 1 or messages[0].get_msgId() != 331 or parser.buf_len() != 0:
            raise ValueError("exactly one complete ODOMETRY message is required")
        return messages[0]


def verify_localhost_loopback(payload, *, timeout_s=1.0):
    """Send one real packet to our own ephemeral localhost UDP receiver."""
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not 0 < timeout_s <= 10:
        raise ValueError("loopback timeout must be positive and at most ten seconds")
    codec = MavlinkOdometryCodec()
    packet = codec.encode(payload)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(timeout_s)
        sender.sendto(packet, receiver.getsockname())
        received, peer = receiver.recvfrom(65535)
        if received != packet or peer[0] != "127.0.0.1":
            raise RuntimeError("localhost packet integrity failed")
        decoded = codec.decode(received)
    return dict(message_id=decoded.get_msgId(), bytes_received=len(received),
                time_usec=decoded.time_usec, reset_counter=decoded.reset_counter,
                unknown_covariance_wire_sentinel=(math.isnan(decoded.pose_covariance[0])
                                                 and math.isnan(decoded.velocity_covariance[0])),
                provenance=codec.provenance)
