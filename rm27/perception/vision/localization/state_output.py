"""Evidence-driven VIO output health and independent IMU snapshot propagation.

This is an offline/interface component, not a flight safety policy. Thresholds
are supplied by the experiment. A numerical pose or an old adapter's stage
does not establish initialization, metric scale, or an accepted visual update.
"""

from dataclasses import dataclass, replace
import math
from collections.abc import Mapping

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
class OperationalValidityResult:
    """A consumer decision, separate from estimator freshness and v2 validity.

    The supported OpenVINS native-health v1 evidence has no qualified tracking,
    covariance or runtime reset signal. It therefore cannot authorize production
    output. A laboratory transport must implement its own explicit isolation;
    this result never changes to eligible merely because that transport is on.
    """

    freshness: str
    tracking_health: str
    covariance_qualification: str
    lifecycle_qualification: str
    output_eligible: bool
    reasons: tuple[str, ...]
    session_id: str | None
    state_source: str | None
    last_visual_sensor_ns: int | None

    def to_dict(self):
        return dict(freshness=self.freshness, tracking_health=self.tracking_health,
                    covariance_qualification=self.covariance_qualification,
                    lifecycle_qualification=self.lifecycle_qualification,
                    output_eligible=self.output_eligible, reasons=list(self.reasons),
                    session_id=self.session_id, state_source=self.state_source,
                    last_visual_sensor_ns=self.last_visual_sensor_ns)


def evaluate_openvins_operational_validity(row, summary, *, now_observation_ns=None):
    """Fail closed on an already decoded native-health v1 row and its summary.

    Callers must first verify the native manifest and timeline identity; a dict
    is not proof of its source. The existing native decoder owns accepted visual
    support and sensor-time reconstruction. This policy adds consumer-side
    consistency/expiry checks without rewriting that decoder or estimator state.
    ``now_observation_ns``, if supplied, must use the *same process-local native
    serialization clock*. It is never compared with a sensor timestamp, and no
    timestamp is refreshed by evaluating the policy. There is deliberately no
    downstream parameter which promotes UNKNOWN tracking/covariance to qualified.
    """
    reasons = []
    row = row if isinstance(row, Mapping) else {}
    summary = summary if isinstance(summary, Mapping) else {}
    nonnegative_int = lambda value: type(value) is int and value >= 0
    known = lambda value: isinstance(value, str) and bool(value.strip()) and value != 'UNKNOWN'
    session = row.get('session_id') if known(row.get('session_id')) else None
    state_source = row.get('state_source')
    state_source = state_source if state_source in ('VISUAL_CORRECTION', 'IMU_PREDICTION') else None
    visual = row.get('last_visual_sensor_ns')
    visual = visual if nonnegative_int(visual) else None
    freshness = row.get('freshness', 'UNKNOWN')
    freshness = freshness if freshness in ('CURRENT', 'STALE', 'INITIALIZING') else 'UNKNOWN'
    covariance = lifecycle = 'UNKNOWN'
    schema_supported = (type(summary.get('schema_version')) is int and summary['schema_version'] == 1
                        and summary.get('scope') == 'PC_OFFLINE_NATIVE_OBSERVABILITY_AND_CONFIGURED_FRESHNESS_ONLY'
                        and summary.get('truth_used') is False)
    if not schema_supported:
        reasons.append('NATIVE_HEALTH_SCHEMA_UNSUPPORTED')
        freshness = 'UNKNOWN'
    if summary.get('status') != 'PASS':
        reasons.append('NATIVE_OBSERVATION_QUALIFICATION_INCOMPLETE')
    if session is None or session != summary.get('session_id'):
        reasons.append('PROCESS_SESSION_UNQUALIFIED')
        freshness = 'UNKNOWN'
    if (summary.get('native_clock_semantic') != 'PROCESS_LOCAL_STEADY_EVENT_SERIALIZATION_NOT_HARDWARE_RECEIPT'
            or not known(row.get('sensor_clock'))):
        reasons.append('CLOCK_SEMANTICS_UNQUALIFIED')
        freshness = 'UNKNOWN'
    try:
        policy = summary['policy']
        if not isinstance(policy, Mapping) or set(policy) != set(StateOutputPolicy.__dataclass_fields__):
            raise ValueError('exact freshness policy required')
        policy = StateOutputPolicy(**policy)
    except (KeyError, TypeError, ValueError):
        reasons.append('FRESHNESS_POLICY_UNQUALIFIED')
        policy = None
        freshness = 'UNKNOWN'
    initialization = summary.get('initialization', {})
    initialized = (isinstance(initialization, Mapping)
                   and initialization.get('commit_observed') is True
                   and initialization.get('ready_observed') is True
                   and row.get('lifecycle') == 'INITIALIZED')
    if not initialized:
        reasons.append('INITIALIZATION_NOT_READY')
        if freshness != 'UNKNOWN':
            freshness = 'INITIALIZING'
    declared_reasons = row.get('freshness_reasons')
    if (not isinstance(declared_reasons, list)
            or any(not known(reason) for reason in declared_reasons)
            or (row.get('freshness') == 'CURRENT') != (declared_reasons == [])):
        reasons.append('FRESHNESS_EVIDENCE_INCONSISTENT')
        freshness = 'UNKNOWN'
    else:
        reasons.extend(declared_reasons)
    observed = row.get('native_observation_ns')
    now = observed if now_observation_ns is None else now_observation_ns
    clock_valid = nonnegative_int(observed) and nonnegative_int(now) and now >= observed
    if not clock_valid:
        reasons.append('NATIVE_OBSERVATION_TIME_INVALID')
        freshness = 'UNKNOWN'
    for name, reason, limit in (
            ('last_sensor_observation_ns', 'SENSOR_STREAM_STOPPED', 'max_receive_age_ns'),
            ('last_imu_observation_ns', 'IMU_STREAM_STOPPED', 'max_receive_age_ns'),
            ('last_state_progress_observation_ns', 'STATE_PROGRESS_STOPPED', 'max_receive_age_ns'),
            ('last_visual_observation_ns', 'VISUAL_EVENT_STALE', 'max_visual_receive_age_ns')):
        previous = row.get(name)
        if previous is None:
            reasons.append(reason)
            if initialized and freshness != 'UNKNOWN':
                freshness = 'STALE'
        elif not nonnegative_int(previous) or not clock_valid or previous > observed:
            reasons.append('NATIVE_OBSERVATION_TIME_INVALID')
            freshness = 'UNKNOWN'
        elif policy is not None and now-previous > getattr(policy, limit):
            reasons.append(reason)
            if initialized and freshness != 'UNKNOWN':
                freshness = 'STALE'
    if visual is None:
        reasons.append('ACCEPTED_VISUAL_UPDATE_UNOBSERVED')
        if initialized and freshness != 'UNKNOWN':
            freshness = 'STALE'
    state_time = row.get('state_time')
    state_semantic = {'VISUAL_CORRECTION': 'NATIVE_CAMERA_STATE_DOUBLE',
                      'IMU_PREDICTION': 'NATIVE_FAST_PROPAGATION_INTEGRATION_ENDPOINT'}
    try:
        state_time_valid = (isinstance(state_time, Mapping)
                            and state_time.get('semantic') == state_semantic.get(state_source)
                            and type(state_time.get('seconds')) in (int, float)
                            and math.isfinite(state_time['seconds']) and state_time['seconds'] >= 0)
    except OverflowError:
        state_time_valid = False
    if initialized and (state_source is None or not state_time_valid):
        reasons.append('STATE_TIME_OR_SOURCE_UNQUALIFIED')
        freshness = 'UNKNOWN'
    if state_source == 'IMU_PREDICTION' and isinstance(state_time, Mapping):
        request = state_time.get('request_sensor_ns')
        if not nonnegative_int(request) or (visual is not None and request < visual):
            reasons.append('SENSOR_STATE_TIME_INVALID')
            freshness = 'UNKNOWN'
        elif visual is not None and policy is not None and request-visual > policy.max_visual_age_ns:
            reasons.append('VISUAL_MEASUREMENT_STALE')
            if initialized and freshness != 'UNKNOWN':
                freshness = 'STALE'
    # A new label supplied by a consumer is not native qualification. These
    # pinned capabilities must evolve through an independently verified source,
    # schema and policy change, rather than by toggling a caller's flags.
    if (summary.get('operational_health') != 'UNKNOWN' or summary.get('operational_valid') is not False
            or row.get('operational_health') != 'UNKNOWN' or row.get('tracking_state') != 'UNKNOWN'
            or row.get('valid') is not False or row.get('contract_complete') is not False):
        reasons.append('NATIVE_TRACKING_QUALIFICATION_UNSUPPORTED')
    reasons.append('TRACKING_HEALTH_UNQUALIFIED')
    cov = summary.get('covariance')
    if isinstance(cov, Mapping) and cov.get('status') == 'UNQUALIFIED' and cov.get('native_finite_witness_only') is True:
        covariance = 'UNQUALIFIED'
    else:
        reasons.append('NATIVE_COVARIANCE_QUALIFICATION_UNSUPPORTED')
    reasons.append('COVARIANCE_UNQUALIFIED')
    reset = summary.get('native_reset')
    if ('native_epoch' in row and 'native_reset_count' in row
            and row.get('native_epoch') is None and row.get('native_reset_count') is None
            and type(row.get('adapter_epoch')) is int and row['adapter_epoch'] == 0
            and summary.get('adapter_epoch_semantic') == 'ONE_ISOLATED_PROCESS_SESSION_NOT_NATIVE_RESET_COUNT'
            and isinstance(reset, Mapping) and reset.get('status') == 'BLOCKED' and reset.get('execution') == 'NOT_RUN'):
        lifecycle = 'PROCESS_SESSION_ONLY'
    else:
        reasons.append('NATIVE_RESET_QUALIFICATION_UNSUPPORTED')
    reasons.append('NATIVE_RESET_EPOCH_UNQUALIFIED')
    if row.get('native_publication_timestamp') is None:
        reasons.append('NATIVE_PUBLICATION_TIME_UNOBSERVED')
    else:
        reasons.append('NATIVE_PUBLICATION_TIME_QUALIFICATION_UNSUPPORTED')
    return OperationalValidityResult(freshness, 'UNKNOWN', covariance, lifecycle, False,
                                     tuple(dict.fromkeys(reasons)), session, state_source, visual)


def evaluate_openvins_measurement_health(row, summary, quality, *, now_observation_ns=None):
    """Assess observed measurement support; never promote it to tracking trust.

    ``quality`` must be the exact-session output of the native quality decoder,
    selected causally by its serialization time. Accepted innovation tests are
    neither independent residual samples nor proof of trajectory correctness.
    This diagnostic adds no feature-count threshold or flight safety limit.
    """
    decision = evaluate_openvins_operational_validity(row, summary, now_observation_ns=now_observation_ns)
    result = dict(schema_version=1, session_id=decision.session_id,
        freshness=decision.freshness, measurement_health='UNKNOWN',
        tracking_health='UNKNOWN', covariance_consistency='UNQUALIFIED',
        output_eligible=False, truth_used=False, reasons=[],
        state_source=decision.state_source, last_visual_sensor_ns=decision.last_visual_sensor_ns,
        covariance_numerical_status='UNKNOWN', quality_sequence=None)
    if decision.freshness in ('STALE','INITIALIZING'):
        result['measurement_health'] = decision.freshness
        result['reasons'] = list(decision.reasons)
        return result
    if decision.freshness != 'CURRENT':
        result['reasons'] = ['FRESHNESS_OR_NATIVE_IDENTITY_UNQUALIFIED']
        return result
    if not isinstance(quality, Mapping):
        result['reasons'] = ['NATIVE_QUALITY_UNOBSERVED']
        return result
    integer = lambda x: type(x) is int and x >= 0
    if (type(quality.get('schema_version')) is not int or quality['schema_version'] != 1
            or quality.get('qualification') != 'VERIFIED_NATIVE_MEASUREMENT_DIAGNOSTIC_ONLY'
            or quality.get('truth_used') is not False or quality.get('session_id') != decision.session_id
            or not integer(quality.get('native_observation_ns'))
            or not integer(quality.get('quality_sequence'))
            or quality['native_observation_ns'] > row['native_observation_ns']
            or quality.get('last_visual_sensor_ns') != decision.last_visual_sensor_ns
            or quality.get('operational_tracking_health') != 'UNKNOWN'
            or quality.get('output_eligible') is not False
            or quality.get('native_epoch', 'missing') is not None
            or quality.get('native_reset_count', 'missing') is not None):
        result['reasons'] = ['NATIVE_QUALITY_IDENTITY_TIME_OR_CAPABILITY_INVALID']
        return result
    counts = quality.get('native_gate_counts')
    required = {'input_feature_count','gate_count','accepted_count','rejected_count','nonfinite_count','invalid_gate_count'}
    if (not isinstance(counts, Mapping) or set(counts) != required
            or any(not integer(counts[k]) for k in required)
            or counts['accepted_count']+counts['rejected_count'] != counts['gate_count']
            or counts['gate_count'] > counts['input_feature_count']
            or not counts['nonfinite_count'] <= counts['invalid_gate_count'] <= counts['gate_count']
            or not integer(quality.get('accepted_visual_update_count'))
            or not integer(quality.get('accepted_observation_uses'))
            or type(quality.get('visual_correction_observed')) is not bool
            or quality['visual_correction_observed'] != (quality['accepted_visual_update_count'] > 0)):
        result['reasons'] = ['NATIVE_QUALITY_POPULATION_INVALID']
        return result
    covariance = quality.get('covariance')
    if (not isinstance(covariance, Mapping) or covariance.get('numerical_status') not in ('PASS','FAIL')
            or covariance.get('statistical_consistency') != 'UNQUALIFIED'
            or covariance.get('flight_covariance') != 'UNQUALIFIED'
            or covariance.get('covariance_modified') is not False):
        result['reasons'] = ['NATIVE_COVARIANCE_DIAGNOSTIC_INVALID']
        return result
    result.update(quality_sequence=quality.get('quality_sequence'),
        covariance_numerical_status=covariance['numerical_status'],
        rejected_native_gate_fraction=(counts['rejected_count']/counts['gate_count'] if counts['gate_count'] else None))
    if counts['invalid_gate_count'] or covariance['numerical_status'] == 'FAIL':
        result['measurement_health'] = 'NUMERICAL_REJECTION'
        result['reasons'] = (['NATIVE_INNOVATION_INVALID'] if counts['invalid_gate_count'] else []) + (
            list(covariance.get('reasons', ['NATIVE_COVARIANCE_NUMERIC_FAILURE'])) if covariance['numerical_status']=='FAIL' else [])
    elif decision.state_source == 'IMU_PREDICTION':
        result['measurement_health'] = 'PREDICTION_ONLY'
        result['reasons'] = ['NO_NEW_VISUAL_CORRECTION_IN_PREDICTION']
    elif quality['visual_correction_observed'] and quality['accepted_observation_uses'] > 0:
        result['measurement_health'] = 'VISUAL_CONSTRAINTS_OBSERVED'
        result['reasons'] = ['ACCEPTED_SUPPORT_IS_NOT_TRACKING_OR_COVARIANCE_QUALIFICATION']
    elif counts['gate_count'] and counts['rejected_count'] == counts['gate_count']:
        result['measurement_health'] = 'ALL_NATIVE_GATES_REJECTED'
        result['reasons'] = ['NO_ACCEPTED_VISUAL_CORRECTION']
    else:
        result['measurement_health'] = 'NO_NEW_VISUAL_CONSTRAINTS'
        result['reasons'] = ['NO_ACCEPTED_VISUAL_CORRECTION']
    return result


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
        self._state_receive = None
        self._latest_receive_clock = None
        self._visual_receive = None
        self._fault = None
        self._kind = None
        self._sample_count = 0

    def _receive(self, time):
        _verified_time(time)
        if self._latest_receive_clock is not None:
            _elapsed(time, self._latest_receive_clock)
        self._latest_receive_clock = time
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
        self._state_receive = received_time
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
                    # A newer visual snapshot can overtake several IMU
                    # callbacks. Their source order/sequence was checked above;
                    # keep the newest usable anchor without reintegrating time
                    # already covered by that snapshot. An older anchor cannot
                    # support propagation until a sufficiently recent one arrives.
                    self._anchor = (sample if _elapsed(self._state.source_time, sample.time)
                                    <= self.policy.max_imu_gap_ns else None)
                elif sample.time.raw_value == self._state.source_time.raw_value:
                    self._anchor = sample
                else:
                    previous_state_ns = self._state.source_time.raw_value
                    self._integrate(sample)
                    if self._state.source_time.raw_value > previous_state_ns:
                        self._state_receive = received_time
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
        self._state_receive = None
        self._samples.clear()
        self._visual_receive = None
        self._sample_count = 0
        self._kind = None
        self._fault = "RESET:" + _provenance(reason, "reset reason")

    def output(self, *, publish_time, now_receive_time, sensor_now=None):
        """Compute fresh health, including when no sensor callbacks arrive."""
        _verified_time(publish_time)
        _verified_time(now_receive_time)
        # A watchdog evaluation observes the same monotonic receive clock as
        # callbacks, without refreshing either actual receipt or visual time.
        if self._latest_receive_clock is not None:
            _elapsed(now_receive_time, self._latest_receive_clock)
        self._latest_receive_clock = now_receive_time
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
        elif _elapsed(now_receive_time, self._state_receive) > self.policy.max_receive_age_ns:
            # An ordered but delayed pre-snapshot sample can refresh raw receipt
            # evidence without advancing the state. It cannot revive that state.
            reasons.append("STATE_PROGRESS_STOPPED")
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
            state_progress_receive_time=self._state_receive.to_dict(),
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
