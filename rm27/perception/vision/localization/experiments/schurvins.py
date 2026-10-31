"""Pinned SchurVINS online pose projection, retaining incomplete v2 evidence.

Pinned svo_ros/visualizer.cpp publishes Eigen's Hamilton T_world_imu at the
image timestamp. A pose publication does not establish flight validity, an
IMU integration endpoint, measured covariance, or a native reset counter.
"""
from decimal import Decimal

from ..schema import TimestampEvidence
from .common import canonical, number, require
from .online import OnlinePoseRecord
from .trajectory import rotation_wxyz

PIN = 'd8ab6dff20860694d8d879d716d7fd6cb2c2530e'


def pose_observation(*, source_frame_id, image_ns, state_ns,
                     observed_monotonic_ns, position, quaternion_xyzw, run_id,
                     epoch, binary_sha256, sequence, raw_reference,
                     stage=None, quality=None, processing_time_s=None,
                     reset_evidence=None, dataset='EuRoC'):
    """Convert an already associated visual-state observation; never retime it.

    ``epoch`` belongs to the observer run. ``reset_evidence`` may describe an
    observed stage transition, but is not promoted to a native reset event.
    The upstream all-zero covariance is intentionally not assigned semantics.
    """
    require(isinstance(run_id, str) and bool(run_id), 'RUN_ID_INVALID', 'run ID required')
    require(type(epoch) is int and epoch >= 0, 'EPOCH_INVALID', 'adapter-owned epoch')
    require(isinstance(sequence, str) and bool(sequence), 'SEQUENCE_INVALID', 'dataset sequence required')
    require(isinstance(binary_sha256, str) and len(binary_sha256) == 64
            and all(c in '0123456789abcdef' for c in binary_sha256),
            'BINARY_HASH_INVALID', 'SHA256')
    require(isinstance(raw_reference, dict)
            and raw_reference.get('topic') == '/svo/pose_imu',
            'RAW_REFERENCE_INVALID', 'visual-state stream only')
    require(len(position) == 3 and len(quaternion_xyzw) == 4,
            'POSE_INVALID', 'pose dimensions')
    p = [number(v, 'position') for v in position]
    x, y, z, w = [number(v, 'quaternion') for v in quaternion_xyzw]
    q = [w, x, y, z]
    rotation_wxyz(q)
    for label, value in [('stage', stage), ('quality', quality)]:
        require(value is None or type(value) is int, 'STATE_EVIDENCE_INVALID', label)
    if processing_time_s is not None:
        number(processing_time_s, 'processing time')
        require(processing_time_s >= 0, 'ONLINE_TIME_INVALID', 'processing time')
    require(reset_evidence is None or isinstance(reset_evidence, dict),
            'RESET_EVIDENCE_INVALID', 'raw observer evidence only')
    require(dataset in ('EuRoC', 'UZH_FPV'), 'DATASET_INVALID', 'explicit sensor clock identity')
    clock = dataset + ':' + sequence + ':sensor_time'
    image = TimestampEvidence(image_ns, 'ns', clock, 'DATASET_CAMERA_TIMESTAMP', 'VERIFIED').to_dict()
    require(state_ns == image_ns, 'POSE_TIMESTAMP_MISMATCH',
            'pinned pose_imu header must equal its exact source image timestamp')
    state = TimestampEvidence(state_ns, 'ns', clock,
                              'VISUAL_STATE_HEADER_IMAGE_TIME', 'CODE_SUPPORTED').to_dict()
    receipt = TimestampEvidence(observed_monotonic_ns, 'ns', run_id + ':observer_monotonic',
                                'ADAPTER_CALLBACK_RECEIPT', 'OBSERVED').to_dict()
    pose = dict(parent_frame='backend_world', child_frame='imu', translation=p,
                orientation_wxyz=q, scale_state='METRIC', translation_unit='m',
                valid=False, localization_epoch=epoch, source_time=state)
    return OnlinePoseRecord(
        source_frame_id=source_frame_id, source_timestamp=image, backend='SchurVINS',
        backend_version=dict(commit=PIN, binary_sha256=binary_sha256),
        raw_backend_state=canonical(dict(stage=stage, quality=quality)),
        tracking_state='UNKNOWN', initialization_state='UNKNOWN', validity=False,
        reason='NUMERICAL_POSE_ONLY_OPERATIONAL_STATE_UNQUALIFIED',
        encoded_pts=str(Decimal(image_ns) / Decimal(10**9)),
        localization_fields=pose, backend_timestamp=state,
        observation_timestamp=receipt, processing_time_s=processing_time_s,
        localization_epoch=epoch, backend_pose_convention='Hamilton_T_world_imu_xyzw',
        raw_output={**raw_reference, 'stage': stage, 'quality': quality,
                    'epoch_owner': 'adapter_run', 'epoch_boundary_evidence': reset_evidence,
                    'upstream_reset_event': None, 'imu_data_coverage': None,
                    'visual_update_completion': None,
                    'inertial_only_state': None,
                    'covariance_semantics': 'UPSTREAM_ZERO_PLACEHOLDER_NOT_UNCERTAINTY'}
    ).to_dict()


def project_native_corrections(native_analysis, raw_info, *, policy, run_id, sequence,
                               binary_sha256, dataset='EuRoC', watchdog_events=(),
                               watchdog_complete=False):
    """Replay explicit native correction evidence through the output contract.

    This verifies an offline candidate interface, never flight readiness. Native
    publication/completion times remain unknown. ``publish_time`` in emitted v2
    records is the actual offline adapter emission on a separate clock; recorded
    observer receipts only drive the receive-clock watchdog. Independent ticks
    are merged by the same recorded observer clock without refreshing receipts
    or inventing sensor time. Completeness requires explicit post-exit evidence
    and ticks spanning the recorded Info interval. No truth is read.
    """
    import time
    from ..schema import LocalizationEstimate, ScaleState, TrackingState
    from ..state_output import NativeCorrectionEvidence, StateOutputPolicy, StateOutputTracker

    require(isinstance(policy, StateOutputPolicy), 'STATE_POLICY_REQUIRED', 'explicit experiment health limits')
    require(isinstance(run_id, str) and run_id and isinstance(sequence, str) and sequence,
            'RUN_ID_INVALID', 'run and sequence required')
    require(isinstance(binary_sha256, str) and len(binary_sha256) == 64
            and all(c in '0123456789abcdef' for c in binary_sha256), 'BINARY_HASH_INVALID', 'SHA256')
    require(dataset in ('EuRoC', 'UZH_FPV'), 'DATASET_INVALID', 'explicit sensor clock identity')
    require(isinstance(native_analysis, dict) and isinstance(raw_info, list),
            'NATIVE_STATE_INPUT_INVALID', 'native analysis and recorded Info list required')
    result = dict(status='NOT_RUN', verification_scope='OFFLINE_NATIVE_STATE_INTERFACE_ONLY',
                  estimator_reproduction='NOT_RUN', flight_qualification='NOT_RUN',
                  native_publication_timestamp=None, native_processing_completed_timestamp=None,
                  corrections=[], outputs=[], rejected_events=[], policy=dict(policy.__dict__))
    require(isinstance(watchdog_events, (list, tuple)) and type(watchdog_complete) is bool,
            'WATCHDOG_INPUT_INVALID', 'recorded tick sequence and explicit completeness boolean required')
    last_tick = None
    for tick in watchdog_events:
        require(isinstance(tick, dict) and tick.get('schema_version') == 1
                and tick.get('event') == 'WATCHDOG_TICK'
                and tick.get('clock_source') == 'OBSERVER_CLOCK_MONOTONIC'
                and tick.get('trigger') == 'INDEPENDENT_SCHEDULE'
                and 'sensor_now' in tick and tick['sensor_now'] is None,
                'WATCHDOG_INPUT_INVALID', 'explicit independent observer tick; no fabricated sensor time')
        received = tick.get('observed_monotonic_ns')
        require(type(received) is int and received >= 0
                and (last_tick is None or received > last_tick),
                'WATCHDOG_INPUT_INVALID', 'watchdog clock must increase; no sorting')
        last_tick = received
    previous_sensor, previous_receive = None, None
    for info in raw_info:
        require(isinstance(info, dict), 'INFO_INVALID', 'recorded Info object required')
        sensor, received = info.get('state_ns'), info.get('observed_monotonic_ns')
        require(type(sensor) is int and sensor >= 0 and type(received) is int and received >= 0,
                'INFO_INVALID', 'exact integer sensor and observer receipt timestamps required')
        require(previous_sensor is None or sensor > previous_sensor,
                'INFO_INVALID', 'Info sensor times must increase; no sorting')
        require(previous_receive is None or received >= previous_receive,
                'INFO_INVALID', 'Info observer clock must not rewind')
        require(type(info.get('stage')) is int and type(info.get('tracking_quality')) is int,
                'INFO_INVALID', 'raw native stage and quality enums required')
        previous_sensor, previous_receive = sensor, received
    spans_info = bool(watchdog_events and raw_info
        and watchdog_events[0]['observed_monotonic_ns'] <= raw_info[0]['observed_monotonic_ns']
        and watchdog_events[-1]['observed_monotonic_ns'] >= raw_info[-1]['observed_monotonic_ns'])
    complete = watchdog_complete and spans_info
    result['watchdog'] = dict(evaluation_status='PASS' if complete else 'NOT_RUN',
        trigger='INDEPENDENT_SCHEDULE', timeline_complete=complete, truth_used=False,
        tick_count=len(watchdog_events), invalid_tick_count=0,
        post_exit_completeness_verified=watchdog_complete, spans_recorded_info=spans_info,
        maximum_tick_interval_ns=max((right['observed_monotonic_ns']-left['observed_monotonic_ns']
            for left, right in zip(watchdog_events, watchdog_events[1:])), default=None),
        clock_domain=run_id+':observer_monotonic', sensor_clock_mapping=None,
        completeness_scope='owned independent schedule through recorded Info interval; no timer deadline guarantee')
    candidates = native_analysis.get('visual_corrections', {}).get('events', [])
    if native_analysis.get('status') != 'PASS' or not candidates:
        result['reason'] = 'COMPLETE_ACCEPTED_NATIVE_CORRECTION_EVIDENCE_UNAVAILABLE'
        return result
    initialization = native_analysis.get('initialization', {})
    initialized = initialization.get('exact_metric_events', [])
    commits = initialization.get('metric_filter_commits', [])
    by_time = {row['sensor_ns']: row for row in candidates}
    tracker = StateOutputTracker(policy)
    sensor_clock = dataset+':'+sequence+':sensor_time'
    receive_clock, publish_clock = run_id+':observer_monotonic', run_id+':offline_projection_monotonic'
    seen = set()
    # Merge validated streams rather than silently sorting either source.
    i, j, timeline = 0, 0, []
    while i < len(raw_info) or j < len(watchdog_events):
        if i < len(raw_info) and (j == len(watchdog_events)
                or raw_info[i]['observed_monotonic_ns'] <= watchdog_events[j]['observed_monotonic_ns']):
            timeline.append(('INFO_CALLBACK', i, raw_info[i])); i += 1
        else:
            timeline.append(('INDEPENDENT_SCHEDULE', j, watchdog_events[j])); j += 1
    latest_info_sensor = None
    for trigger, ordinal, info in timeline:
        received = info['observed_monotonic_ns']
        if trigger == 'INDEPENDENT_SCHEDULE':
            receipt = TimestampEvidence(received, 'ns', receive_clock,
                                       'RECORDED_INDEPENDENT_WATCHDOG_TICK', 'VERIFIED')
            publication = TimestampEvidence(time.monotonic_ns(), 'ns', publish_clock,
                                           'OFFLINE_ADAPTER_EMISSION', 'VERIFIED')
            # No sensor clock mapping exists for a wall-clock tick. Calling
            # output does not refresh the tracker's last sensor/visual receipt.
            output = tracker.output(publish_time=publication, now_receive_time=receipt)
            result['outputs'].append(dict(sensor_ns=None, latest_info_sensor_ns=latest_info_sensor,
                trigger=trigger, observation_timestamp=receipt.to_dict(), **output.to_dict()))
            continue
        sensor = info['state_ns']
        latest_info_sensor = sensor
        source = TimestampEvidence(sensor, 'ns', sensor_clock, 'NATIVE_ACCEPTED_VISUAL_STATE', 'VERIFIED')
        receipt = TimestampEvidence(received, 'ns', receive_clock, 'RECORDED_INFO_CALLBACK_RECEIPT', 'VERIFIED')
        publication = TimestampEvidence(time.monotonic_ns(), 'ns', publish_clock,
                                        'OFFLINE_ADAPTER_EMISSION', 'VERIFIED')
        event = by_time.get(sensor)
        if event is not None:
            seen.add(sensor)
            reason = None
            if (type(event.get('constraint_count')) is not int or event['constraint_count'] <= 0
                    or event.get('state_source') != 'EXPLICIT_NATIVE_ACCEPTED_VISUAL_CORRECTION'):
                reason = 'EXPLICIT_USED_VISUAL_CONSTRAINTS_UNAVAILABLE'
            if info['stage'] != 2 or info['tracking_quality'] != 2:
                reason = 'NATIVE_INFO_NOT_TRACKING_GOOD'
            matches = [row for row in initialized
                       if row.get('observation_segment') == event.get('observation_segment')
                       and row.get('event_ordinal', -1) < event.get('event_ordinal', -1)
                       and row.get('same_bundle_filter_commit_observed') is True]
            matched = next((row for row in reversed(matches) if any(
                commit.get('observation_segment') == row.get('observation_segment')
                and commit.get('bundle_id') == row.get('bundle_id')
                and commit.get('event_ordinal', -1) < row.get('event_ordinal', -1)
                for commit in commits)), None)
            if matched is None:
                reason = reason or 'SAME_SEGMENT_METRIC_INITIALIZATION_UNAVAILABLE'
            if reason is not None:
                result['rejected_events'].append(dict(sensor_ns=sensor, reason=reason))
                tracker.mark_lost(reason)
            else:
                try:
                    epoch = event['native_epoch']
                    if tracker.epoch is not None and epoch != tracker.epoch:
                        transitions=native_analysis.get('lifecycle', {}).get('native_epoch_transitions', [])
                        require(any(row.get('native_epoch') == epoch
                                    and row.get('previous_epoch') == tracker.epoch
                                    and row.get('event_ordinal') == event['event_ordinal']
                                    and row.get('observation_segment') == event['observation_segment']
                                    and not row.get('reset_event_missing') for row in transitions),
                                'NATIVE_RESET_EVIDENCE_REQUIRED', 'explicit parsed reset segment transition')
                        tracker.reset(epoch, reason='EXPLICIT_NATIVE_RESET_SEGMENT', received_time=receipt)
                    provenance = dict(binary_sha256=binary_sha256, native_source=native_analysis.get('source'),
                                      observation_segment=event['observation_segment'],
                                      event_ordinal=event['event_ordinal'], info_ordinal=ordinal,
                                      native_publication_timestamp=None, native_processing_completed_timestamp=None,
                                      covariance_status='UNKNOWN_NOT_EXPORTED',
                                      candidate_scope='OFFLINE_INTERFACE_NOT_FLIGHT_QUALIFIED')
                    estimate = LocalizationEstimate(source_time=source, publish_time=publication,
                        parent_frame='backend_world', child_frame='imu', translation=event['position'],
                        orientation_wxyz=event['orientation_wxyz'], tracking_state=TrackingState.TRACKING,
                        valid=True, initialized=True, relocalizing=False, localization_epoch=epoch,
                        scale_state=ScaleState.METRIC, translation_unit='m',
                        velocity=event['velocity_parent_m_s'], velocity_frame='backend_world', velocity_unit='m/s',
                        quality=dict(native_correction=provenance))
                    evidence = NativeCorrectionEvidence(estimate=estimate, last_accepted_visual_time=source,
                        correction_id=run_id+':native:'+str(event['event_ordinal']),
                        initialization_source='metric_filter_commit+mono_metric_initialized:same_native_segment',
                        scale_source='NATIVE_VISUAL_IMU_ALIGNMENT_NO_TRUTH',
                        bias_source='NATIVE_SCHUR_CORRECTION_SNAPSHOT',
                        gravity_source='NATIVE_METRIC_INITIALIZATION_SNAPSHOT',
                        gyro_bias_rad_s=event['gyro_bias_rad_s'], accel_bias_m_s2=event['accel_bias_m_s2'],
                        gravity_parent_m_s2=event['gravity_parent_m_s2'], evidence_status='VERIFIED')
                    tracker.accept_correction(evidence, received_time=receipt)
                    result['corrections'].append(dict(estimate=estimate.to_dict(),
                        last_accepted_visual_time=source.to_dict(), correction_id=evidence.correction_id,
                        initialization_source=evidence.initialization_source, scale_source=evidence.scale_source,
                        bias_source=evidence.bias_source, gravity_source=evidence.gravity_source,
                        gyro_bias_rad_s=list(evidence.gyro_bias_rad_s), accel_bias_m_s2=list(evidence.accel_bias_m_s2),
                        gravity_parent_m_s2=list(evidence.gravity_parent_m_s2), evidence_status='VERIFIED'))
                except (KeyError, TypeError, ValueError) as exc:
                    reason = 'NATIVE_CORRECTION_REJECTED:'+str(exc)
                    result['rejected_events'].append(dict(sensor_ns=sensor, reason=reason))
                    tracker.mark_lost(reason)
        elif info['stage'] != 2 or info['tracking_quality'] != 2:
            tracker.mark_lost('NATIVE_INFO_NOT_TRACKING_GOOD')
        output = tracker.output(publish_time=publication, now_receive_time=receipt, sensor_now=source)
        result['outputs'].append(dict(sensor_ns=sensor, trigger=trigger,
                                      observation_timestamp=receipt.to_dict(), **output.to_dict()))
    for sensor in by_time.keys()-seen:
        result['rejected_events'].append(dict(sensor_ns=sensor, reason='EXACT_INFO_ASSOCIATION_UNAVAILABLE'))
    result['status'] = 'FAIL' if result['rejected_events'] else ('PASS' if result['corrections'] else 'NOT_RUN')
    return result
