"""Summarize retained replay evidence without changing any acceptance result.

Native frontend timers, native process resources and whole-container samples
have distinct scopes. Failed and unevaluable runs remain in every denominator.
"""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np

from tools.openvins.native_accuracy import digest, rotations
from tools.schurvins.evaluate import canonical_row, load_rows


def distribution(values):
    values = np.asarray(values, float)
    if not len(values):
        return dict(status='NOT_RUN', samples=0)
    if not np.isfinite(values).all():
        raise ValueError('nonfinite measurement')
    return dict(status='OBSERVED', samples=len(values), minimum=float(values.min()),
                maximum=float(values.max()), mean=float(values.mean()),
                median=float(np.median(values)), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)),
                sample_stddev=float(np.std(values, ddof=1)) if len(values) > 1 else None)


def native_resources(text):
    """Linux GNU time -v; maximum RSS is KiB, not container memory usage."""
    fields = {}
    for line in text.splitlines():
        if ': ' in line:
            key, value = line.strip().rsplit(': ', 1)
            fields[key] = value
    required = ['User time (seconds)', 'System time (seconds)',
                'Maximum resident set size (kbytes)',
                'Elapsed (wall clock) time (h:mm:ss or m:ss)']
    if not all(k in fields for k in required):
        return dict(status='NOT_RUN', reason='complete GNU time resource record unavailable')
    wall = 0.
    for value in fields[required[3]].split(':'):
        wall = 60*wall + float(value)
    user, system = float(fields[required[0]]), float(fields[required[1]])
    rss = int(fields[required[2]])
    if not np.isfinite([wall, user, system]).all() or min(wall, user, system, rss) < 0:
        raise ValueError('invalid native resource record')
    return dict(status='OBSERVED', scope='native estimator process tree (GNU time)',
                user_cpu_s=user, system_cpu_s=system, elapsed_s=wall,
                average_cpu_cores=(user+system)/wall if wall else None,
                maximum_rss_kib=rss, exit_status=fields.get('Exit status'))


def container_resources(rows):
    if len(rows) < 2 or not all('cgroup_cpu' in r and 'cgroup_memory_bytes' in r for r in rows):
        return dict(status='NOT_RUN', reason='whole-container cgroup samples unavailable')
    stamps = [r['observed_monotonic_ns'] for r in rows]
    if any(type(t) is not int for t in stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('cgroup monotonic timestamps must increase')
    usage = [int(dict(line.split() for line in r['cgroup_cpu'].splitlines())['usage_usec']) for r in rows]
    if any(b < a for a, b in zip(usage, usage[1:])):
        raise ValueError('cgroup CPU counter reset')
    memory = [int(r['cgroup_memory_bytes']) for r in rows]
    if min(memory) < 0:
        raise ValueError('negative cgroup memory')
    elapsed = (stamps[-1]-stamps[0])/1e9
    cpu = (usage[-1]-usage[0])/1e6
    return dict(status='OBSERVED', samples=len(rows), sampled_window_s=elapsed,
                cpu_s_in_sampled_window=cpu, average_cpu_cores=cpu/elapsed,
                sampled_peak_memory_current_bytes=max(memory),
                scope='whole container including estimator, transport, observer and ROS',
                window=('first to last recorded resource sample; inspect runner commands for '
                        'startup/playback/shutdown coverage; OpenVINS paused-match protocol '
                        'starts sampling before its resume service call'),
                memory_semantics='cgroup memory.current includes cache; this is not RSS')


def frontend_timing(backend, info=None, csv_text=None):
    if backend == 'SchurVINS' and info:
        # The wrapper reports its last timer even while paused; those values
        # cannot be interpreted as new frontend work or processing completion.
        active = [r['processing_time_s'] for r in info if r['stage'] == 2]
        if any(v < 0 for v in active):
            raise ValueError('negative frontend timer')
        return dict(status='OBSERVED', native_tracking_stage_s=distribution(active),
                    stage_counts={str(s):sum(r['stage'] == s for r in info)
                                  for s in sorted({r['stage'] for r in info})},
                    scope='native Info frontend timer at stage=2 only',
                    excluded='initializing/relocalizing/paused Info; paused timer may be stale',
                    end_to_end_latency='NOT_RUN')
    if backend == 'OpenVINS' and csv_text:
        lines = csv_text.splitlines()
        expected = '# timestamp (sec),tracking,propagation,msckf update,slam update,slam delayed,re-tri & marg,total'
        if not lines or lines[0].strip() != expected:
            raise ValueError('unrecognized pinned OpenVINS timing header')
        rows = list(csv.reader(line for line in lines[1:] if line.strip()))
        if any(len(r) != 8 for r in rows):
            raise ValueError('invalid OpenVINS timing columns')
        values = [float(r[-1]) for r in rows]
        if any(v < 0 for v in values):
            raise ValueError('negative frontend timer')
        return dict(status='OBSERVED', native_camera_update_s=distribution(values),
                    scope='pinned OpenVINS camera update total, microseconds*1e-6 in VioManager.cpp',
                    end_to_end_latency='NOT_RUN')
    return dict(status='NOT_RUN', reason='native frontend timing unavailable')


def compact_metrics(score):
    return dict(ate_rmse_m=score['ate']['rmse_m'],
                rpe_adjacent_translation_rmse_m=score['rpe_adjacent']['translation_rmse_m'],
                rpe_adjacent_rotation_rmse_deg=score['rpe_adjacent']['rotation_rmse_deg'],
                rpe_1s_translation_rmse_m=score['rpe_1s']['translation_rmse_m'],
                rpe_1s_rotation_rmse_deg=score['rpe_1s']['rotation_rmse_deg'],
                absolute_orientation_rmse_deg=score.get('orientation_absolute', {}).get('rmse_deg'),
                alignment=score['alignment']['type'], scale=score['alignment']['scale'],
                path_length_bias_percent=score['scale_diagnostic']['path_length_bias_percent'])



def openvins_native_observation_summary(native):
    """Retain accepted measurement support separately from state/health claims."""
    if (native.get('schema_version') != 1
            or native.get('scope') != 'NATIVE_OBSERVATION_INTEGRITY_ONLY'):
        return dict(status='NOT_RUN', reason='recognized OpenVINS native observation schema required')
    initializations = native.get('initialization_events', [])
    updates = native.get('visual_updates', [])
    advances = sum(row['advances_visual_measurement_time'] for row in updates)
    return dict(status=native.get('status'), reason=native.get('reason'),
        scope='native observation integrity only; original run acceptance unchanged',
        observation_integrity=native.get('observation_integrity'),
        initialization=dict(status='OBSERVED' if initializations else 'NOT_RUN',
            exact_metric_completion_observed=bool(initializations),
            metric_commit_count=len(initializations), exact_metric_events=initializations,
            timestamp_semantics='native commit monotonic time and estimator double state seconds retained separately',
            duration=dict(status='NOT_RUN', reason='initialization start event and source-time association unavailable'),
            qualification='metric filter commit observation; tracking health and operational readiness unqualified'),
        visual_corrections=dict(count=len(updates),
            distinct_measurement_time_advance_count=advances,
            updates_without_visual_time_advance=len(updates)-advances,
            first_update_measurement_support=updates[0]['measurement_support'] if updates else [],
            last_update_measurement_support=updates[-1]['measurement_support'] if updates else [],
            last_visual_measurement_ns=native.get('last_visual_measurement_ns'),
            measurement_time_association=native.get('time_association'),
            qualification='accepted feature support; reused measurements and propagation do not refresh visual time'),
        first_accepted_visual_sensor_ns=updates[0]['newest_measurement_ns'] if updates else None,
        exact_native_camera_callback_count=native.get('exact_native_camera_callback_count'),
        native_imu_callback_coverage=native.get('native_imu_callback_coverage',
            dict(status='NOT_RUN', reason='sealed decoder did not record independent IMU callback coverage')),
        trace_completeness=native.get('observation_complete'),
        independent_tick_count=native.get('independent_tick_count'),
        health_qualification=native.get('health_qualification', 'NOT_RUN'),
        automatic_reset=native.get('automatic_reset', 'NOT_RUN'),
        raw_input_equality=native.get('raw_input_equality', 'NOT_RUN'),
        truth_used=native.get('truth_used'),
        state_output=dict(status='NOT_RUN', reason='native observation decoder does not qualify state validity'),
        publication_latency=dict(status='NOT_RUN', reason='native publication and mapped completion times unavailable'),
        scope_limits=native.get('scope_limits'))


def native_observation_summary(native, states, *, backend='SchurVINS'):
    """Summarize sealed observations without promoting them to estimator PASS."""
    if not native:
        return dict(status='NOT_RUN', reason='explicit native observations unavailable')
    if backend == 'OpenVINS':
        return openvins_native_observation_summary(native)
    imu_ledger = native.get('imu', {}).get('callback_ledger', dict(
        status='NOT_RUN', reason='sealed decoder did not record a native IMU callback ledger',
        callback_coverage=dict(status='NOT_RUN'), buffer_insertion=dict(status='NOT_RUN'),
        downstream_integration='UNOBSERVED', reset_mapping='NOT_RUN', health_qualification='NOT_RUN'))
    if native.get('status') == 'NOT_RUN':
        return dict(status='NOT_RUN', reason='explicit native camera observations unavailable',
            native_imu_callback_ledger=imu_ledger)
    initialization = native.get('initialization', {})
    corrections = native.get('visual_corrections', {})
    lifecycle = native.get('lifecycle', {})
    outputs = states.get('outputs', []) if states else []
    qualified = [row for row in outputs if type(row.get('usable')) is bool]
    return dict(status=native.get('status'), scope='native observations; original run acceptance unchanged',
        initialization=initialization, timings=native.get('timings'),
        visual_corrections={key:value for key,value in corrections.items() if key != 'events'},
        first_accepted_visual_sensor_ns=(corrections['events'][0]['sensor_ns']
                                         if corrections.get('events') else None),
        native_imu_callback_ledger=imu_ledger,
        input_frequency=native.get('coverage', {}).get('input_sensor_frequency'),
        native_reset_calls=lifecycle.get('observed_reset_call_count'),
        post_frontend_reset_calls=lifecycle.get('post_frontend_reset_call_count'),
        native_epoch_transitions=lifecycle.get('native_epoch_transitions'),
        frontend_failures=lifecycle.get('frontend_failures'),
        trace_completeness=native.get('coverage', {}).get('complete_trace'),
        counts_scope='observed events only; a logger cap or absent footer cannot prove exhaustive counts',
        state_output=dict(status=states.get('status') if states else 'NOT_RUN',
            emitted_samples=len(qualified), usable_samples=sum(row['usable'] for row in qualified),
            usable_sample_ratio=(sum(row['usable'] for row in qualified)/len(qualified) if qualified else None),
            sampling='recorded Info/watchdog ticks; sample ratio is not duration-weighted availability',
            rejected_events=states.get('rejected_events', []) if states else [],
            qualification=states.get('verification_scope') if states else None,
            watchdog=states.get('watchdog') if states else None),
        publication_latency=dict(status='NOT_RUN', reason='native publication and mapped completion times unavailable'))


def fault_response(data, info, native=None, states=None):
    """Temporal fault observations only; no truth is used for online health.

    Recovery requires two actual accepted visual updates in one native epoch,
    tracking-good Info throughout their interval, and usable projected states
    when those were recorded. This is a declared observation rule, not a flight
    safety limit or causal proof that the perturbation caused a loss. A later
    loss/recovery remains observable, but only a source-time detection inside
    the half-open fault window or an overlapping receipt bracket associates
    that detection with the declared fault interval.
    """
    derivation = data.get('derivation') if isinstance(data, dict) else None
    absent = dict(status='NOT_RUN', first_sensor_ns=None)
    report = dict(status='NOT_RUN', experiment_kind='ORIGINAL_DATA' if not derivation else 'ARTIFICIAL_DISTURBANCE',
                  online_failure=dict(absent), recovery=dict(absent),
                  fault_interval_association=dict(status='NOT_RUN', evidence_kind=None,
                      source_sensor_ns=None, sensor_time_bracket_ns=None),
                  offline_error_boundary=dict(status='NOT_RUN', reason='per-sample truth error boundary not requested or recorded'),
                  truth_drives_health=False, causality='temporal association only; no causal attribution',
                  recovery_rule='two accepted native visual corrections, one epoch, tracking-good Info throughout')
    if not derivation:
        report['reason']='no artificial fault derivation'
        return report
    profile=derivation.get('profile', {})
    report['profile']=profile
    first=data.get('source_first_ns', data.get('first_ns', {}))
    known=[value for value in first.values() if type(value) is int and value >= 0] if isinstance(first, dict) else []
    start, end=profile.get('start_elapsed_ns'), profile.get('end_elapsed_ns')
    if (not known or type(start) is not int or type(end) is not int or start < 0 or end <= start
            or profile.get('epoch', 0) != 0):
        report['reason']='source-origin or single-epoch fault interval evidence unavailable'
        return report
    origin=min(known)
    onset, finish=origin+start, origin+end
    report.update(status='OBSERVED', fault_window=dict(origin_sensor_ns=origin,
        origin_source='minimum original source_first_ns across recorded sensors',
        start_elapsed_ns=start, end_elapsed_ns=end, start_sensor_ns=onset,
        end_sensor_ns=finish, interval='half-open source sensor time; delivery delay never retimes acquisition'),
        changed_source_events=len(derivation.get('changes', [])))
    info=info or []
    if any(type(row.get('state_ns')) is not int for row in info):
        raise ValueError('fault Info timestamps must be integer nanoseconds')
    if any(b['state_ns'] <= a['state_ns'] for a,b in zip(info,info[1:])):
        raise ValueError('fault Info timestamps must increase; no sorting')
    all_state_rows=states.get('outputs', []) if states else []
    state_rows=[row for row in all_state_rows if type(row.get('sensor_ns')) is int]
    timer_rows=[row for row in all_state_rows if row.get('trigger') == 'INDEPENDENT_SCHEDULE']
    if any(row.get('sensor_ns') is not None for row in timer_rows):
        raise ValueError('independent watchdog ticks cannot assert mapped sensor time')
    timer_failure=next((row for row in timer_rows if row.get('usable') is False),None)
    report['independent_watchdog_detection']=dict(
        status='OBSERVED' if timer_failure else 'NOT_RUN',
        timeline=(states or {}).get('watchdog'),
        first_invalid_observation_timestamp=(timer_failure.get('observation_timestamp') if timer_failure else None),
        last_sensor_reference_ns=(timer_failure.get('latest_info_sensor_ns') if timer_failure else None),
        reasons=timer_failure.get('reasons', []) if timer_failure else [],
        fault_interval_association='NOT_RUN: no observer-to-sensor clock mapping',
        reference_semantic='latest Info sensor timestamp is a reference, never tick sensor time')
    if all_state_rows:
        observations=[dict(sensor_ns=row['sensor_ns'], usable=row['usable'],
                           reasons=row.get('reasons', []),
                           observation_timestamp=row.get('observation_timestamp')) for row in state_rows]
        detection='OFFLINE_PROJECTED_NATIVE_HEALTH_AT_RECORDED_CALLBACK_OR_WATCHDOG_TICK'
    else:
        observations=[dict(sensor_ns=row['state_ns'],
                           usable=row.get('stage') == 2 and row.get('tracking_quality') == 2,
                           reasons=['NATIVE_INFO_STAGE_OR_QUALITY']) for row in info]
        detection='NATIVE_INFO_ENUM_OBSERVATION_ONLY_NOT_QUALIFIED_VALIDITY'
    def qualified_receipt(row):
        stamp = row.get('observation_timestamp')
        if not isinstance(stamp, dict):
            return None
        clock = stamp.get('clock_domain')
        if (type(stamp.get('raw_value')) is not int or stamp['raw_value'] < 0
                or stamp.get('unit') != 'ns' or stamp.get('evidence_status') != 'VERIFIED'
                or not isinstance(clock, str) or not clock.strip() or clock.strip().upper() == 'UNKNOWN'):
            return None
        return stamp
    receipt_observations = [row for row in observations if qualified_receipt(row) is not None]
    timer_receipts = [(row, qualified_receipt(row)) for row in timer_rows]
    before=[row for row in observations if row['sensor_ns'] < onset]
    report['online_failure']['evidence_kind']=detection
    if not before or not before[-1]['usable']:
        report['online_failure'].update(reason='no healthy observation immediately before fault; preexisting failure retained')
        return report
    loss=next((row for row in observations if row['sensor_ns'] >= onset and not row['usable']), None)
    # A timer has no sensor time. Associate a detection only by its ordering
    # between two same-clock Info receipts whose source interval overlaps the
    # fault window. Keep those bounds; never interpolate a sensor timestamp.
    bracketed_timer_loss = None
    for tick, stamp in timer_receipts:
        if tick.get('usable') is not False or stamp is None:
            continue
        comparable = [row for row in receipt_observations
            if row['observation_timestamp']['clock_domain'] == stamp['clock_domain']]
        left = next((row for row in reversed(comparable)
                     if row['observation_timestamp']['raw_value'] <= stamp['raw_value']), None)
        right = next((row for row in comparable
                      if row['observation_timestamp']['raw_value'] >= stamp['raw_value']), None)
        if (left is None or right is None or left['sensor_ns'] > right['sensor_ns'] or left['sensor_ns'] >= finish
                or right['sensor_ns'] < onset):
            continue
        bracketed_timer_loss = dict(sensor_ns=None, usable=False, reasons=tick.get('reasons', []),
            observation_timestamp=stamp, sensor_time_bracket_ns=[left['sensor_ns'], right['sensor_ns']],
            association='receipt order between Info source bounds overlapping fault; no cross-clock subtraction')
        break
    if bracketed_timer_loss is not None:
        a, b = bracketed_timer_loss['observation_timestamp'], qualified_receipt(loss or {})
        if loss is None or (b is not None and a['clock_domain'] == b['clock_domain']
                            and a['raw_value'] < b['raw_value']):
            loss = bracketed_timer_loss
    if loss is None:
        report['online_failure'].update(status='NOT_OBSERVED',
            reason='no recorded loss; stream after final recorded tick is unobserved')
        report['fault_interval_association'].update(status='NOT_OBSERVED',
            reason='no detection within half-open fault interval or overlapping source bracket')
        return report
    report['online_failure'].update(status='OBSERVED', first_sensor_ns=loss['sensor_ns'],
        elapsed_from_fault_s=(loss['sensor_ns']-onset)/1e9 if loss['sensor_ns'] is not None else None,
        first_observation_timestamp=loss.get('observation_timestamp'),
        sensor_time_bracket_ns=loss.get('sensor_time_bracket_ns'),
        association=loss.get('association', 'recorded Info sensor event'), reasons=loss['reasons'],
        healthy_before_fault_sensor_ns=before[-1]['sensor_ns'])
    if loss['sensor_ns'] is not None:
        associated = onset <= loss['sensor_ns'] < finish
        report['fault_interval_association'].update(
            status='OBSERVED' if associated else 'NOT_OBSERVED', evidence_kind='SAME_SOURCE_SENSOR_CLOCK',
            source_sensor_ns=loss['sensor_ns'],
            reason=('detection lies within declared half-open source sensor interval' if associated else
                    'observed loss lies outside declared half-open source sensor interval'))
    else:
        # The timer selection above requires same-receipt-clock bounds whose
        # closed source bracket intersects [onset, finish). Preserve bounds;
        # this is never a mapped sensor timestamp for the timer itself.
        report['fault_interval_association'].update(status='OBSERVED',
            evidence_kind='SAME_RECEIPT_CLOCK_SOURCE_BRACKET',
            sensor_time_bracket_ns=loss['sensor_time_bracket_ns'],
            reason='same-clock receipt ordering gives source bounds overlapping half-open fault interval')
    accepted=(native or {}).get('visual_corrections', {}).get('events', []) if (native or {}).get('status') == 'PASS' else []
    info_by_time={row['state_ns']:row for row in info}
    states_by_time={row['sensor_ns']:row for row in state_rows}
    candidates=[]
    for row in accepted:
        time=row['sensor_ns']; observation=info_by_time.get(time)
        if (time >= max(finish,loss['sensor_ns'] if loss['sensor_ns'] is not None else onset)
                and row.get('constraint_count', 0) > 0
                and row.get('state_source') == 'EXPLICIT_NATIVE_ACCEPTED_VISUAL_CORRECTION'
                and observation and observation.get('stage') == 2 and observation.get('tracking_quality') == 2
                and (not state_rows or states_by_time.get(time, {}).get('usable') is True)):
            candidates.append(row)
    unqualified_timer_continuity = False
    for left,right in zip(candidates,candidates[1:]):
        if left.get('native_epoch') != right.get('native_epoch'):
            continue
        between=[row for row in info if left['sensor_ns'] <= row['state_ns'] <= right['sensor_ns']]
        if any(row.get('stage') != 2 or row.get('tracking_quality') != 2 for row in between):
            continue
        projected=[row for row in state_rows if left['sensor_ns'] <= row['sensor_ns'] <= right['sensor_ns']]
        left_state,right_state=states_by_time.get(left['sensor_ns']),states_by_time.get(right['sensor_ns'])
        if timer_receipts:
            a,b=qualified_receipt(left_state or {}),qualified_receipt(right_state or {})
            if (a is None or b is None or a['clock_domain'] != b['clock_domain']
                    or a['raw_value'] > b['raw_value'] or any(stamp is None
                        or stamp['clock_domain'] != a['clock_domain'] for _,stamp in timer_receipts)):
                unqualified_timer_continuity = True
                continue
            projected += [row for row,stamp in timer_receipts
                if a['raw_value'] <= stamp['raw_value'] <= b['raw_value']]
        if any(row.get('usable') is not True for row in projected):
            continue
        report['recovery'].update(status='OBSERVED', first_sensor_ns=left['sensor_ns'],
            continuous_evidence_through_sensor_ns=right['sensor_ns'], native_epoch=left.get('native_epoch'),
            elapsed_from_fault_end_s=(left['sensor_ns']-finish)/1e9,
            elapsed_from_detection_s=((left['sensor_ns']-loss['sensor_ns'])/1e9
                                     if loss['sensor_ns'] is not None else None),
            qualification='native visual recovery observation; hardware/flight qualification NOT_RUN')
        break
    if report['recovery']['status'] == 'NOT_RUN':
        report['recovery']['reason']=('qualified same-clock receipt evidence unavailable for timer continuity'
            if unqualified_timer_continuity else 'accepted visual correction and continuous tracking evidence unavailable')
    return report


def health_episode_summary(info, native=None, states=None, *, fault=None):
    """Observed health episodes only; never an acceptance or availability gate.

    Info enums and projected usability are distinct timelines. Epochs and
    source indices require exact native event association; reset segments are
    not epochs. First-to-last recorded spans never imply continuous health.
    """
    absent = dict(status='NOT_RUN', reason='recorded SchurVINS Info unavailable')
    report = dict(schema_version=1, status='NOT_RUN', native_info=dict(absent),
        projected_output=dict(status='NOT_RUN', reason='qualified state output unavailable'),
        after_recovery=dict(status='NOT_RUN', reason='qualified two-update recovery unavailable'),
        final_info=None, final_projected_output=None, final_watchdog=None,
        qualification='recorded observations only; original recovery and run acceptance unchanged',
        duration_semantics='first-to-last observed spans, not continuous availability')
    if not info:
        return report

    def integer(value):
        return type(value) is int and value >= 0

    def receipt(row):
        stamp = row.get('observation_timestamp')
        if (not isinstance(stamp, dict) or not integer(stamp.get('raw_value'))
                or stamp.get('unit') != 'ns' or stamp.get('evidence_status') != 'VERIFIED'
                or not isinstance(stamp.get('clock_domain'), str)
                or not stamp['clock_domain'].strip()
                or stamp['clock_domain'].strip().upper() == 'UNKNOWN'):
            return None
        return dict(clock_domain=stamp['clock_domain'], raw_value=stamp['raw_value'])

    native = native or {}
    trusted = native.get('status') == 'PASS'
    qualities, corrections, metadata = {}, {}, {}
    if trusted:
        for source, target in ((native.get('tracking_quality', {}).get('events', []), qualities),
                (native.get('visual_corrections', {}).get('events', []), corrections)):
            for event in source:
                sensor = event.get('sensor_ns')
                if not integer(sensor) or sensor in target:
                    raise ValueError('ambiguous health episode native source association')
                target[sensor] = event
        sources = [*native.get('coverage', {}).get('camera_observation_segments', []),
                   *qualities.values(), *corrections.values()]
        for event in sources:
            sensor = event.get('sensor_ns')
            if not integer(sensor):
                raise ValueError('invalid health episode native source time')
            fields = metadata.setdefault(sensor, {})
            for key in ('camera_index', 'observation_segment', 'bundle_id'):
                value = event.get(key)
                if value is not None:
                    if not integer(value) or (key in fields and fields[key] != value):
                        raise ValueError('conflicting health episode native identity')
                    fields[key] = value

    projected, callbacks = [], {}
    if states and states.get('status') == 'PASS':
        previous_by_clock, previous_callback_sensor = {}, None
        for row in states.get('outputs', []):
            sensor, trigger = row.get('sensor_ns'), row.get('trigger')
            if (type(row.get('usable')) is not bool
                    or trigger not in ('INFO_CALLBACK', 'INDEPENDENT_SCHEDULE')
                    or (trigger == 'INDEPENDENT_SCHEDULE' and sensor is not None)
                    or (trigger == 'INFO_CALLBACK' and not integer(sensor))):
                raise ValueError('invalid health episode projected output')
            stamp = receipt(row)
            if stamp is not None:
                clock, value = stamp['clock_domain'], stamp['raw_value']
                if value < previous_by_clock.get(clock, value):
                    raise ValueError('health episode receipt clock moved backwards')
                previous_by_clock[clock] = value
            if trigger == 'INFO_CALLBACK':
                if (sensor in callbacks or (previous_callback_sensor is not None
                        and sensor <= previous_callback_sensor)):
                    raise ValueError('duplicate or unordered health episode Info output')
                previous_callback_sensor = sensor
                callbacks[sensor] = row
            epoch = row.get('localization_epoch')
            if epoch is not None and not integer(epoch):
                raise ValueError('invalid health episode snapshot epoch')
            projected.append(dict(classification='USABLE' if row['usable'] else 'INVALID',
                sensor_ns=sensor, source_index=metadata.get(sensor, {}).get('camera_index'),
                observation_segment=None,
                epoch=epoch, epoch_semantics='PROJECTED_SNAPSHOT', receipt=stamp,
                trigger=trigger, reasons=list(row.get('reasons', [])),
                last_accepted_visual_time=row.get('last_accepted_visual_time'),
                accepted_visual_correction=False))

    rows, previous_sensor = [], None
    for row in info:
        sensor, received = row.get('state_ns'), row.get('observed_monotonic_ns')
        if (not integer(sensor) or (previous_sensor is not None and sensor <= previous_sensor)
                or type(row.get('stage')) is not int or type(row.get('tracking_quality')) is not int
                or (received is not None and not integer(received))):
            raise ValueError('invalid or unordered health episode Info; no sorting')
        previous_sensor = sensor
        quality, correction = qualities.get(sensor, {}), corrections.get(sensor, {})
        fields = metadata.get(sensor, {})
        if quality and quality.get('tracking_quality') != row['tracking_quality']:
            raise ValueError('health episode native quality differs from Info')
        epoch = correction.get('native_epoch')
        if epoch is not None and not integer(epoch):
            raise ValueError('invalid health episode native epoch')
        callback = callbacks.get(sensor, {})
        stamp = receipt(callback)
        if stamp is not None and received is not None and stamp['raw_value'] != received:
            raise ValueError('health episode Info receipt differs from projection')
        rows.append(dict(classification=('GOOD' if row['stage'] == 2
                and row['tracking_quality'] == 2 else 'NOT_GOOD'),
            sensor_ns=sensor, source_index=fields.get('camera_index'),
            observation_segment=fields.get('observation_segment'), epoch=epoch,
            epoch_semantics='EXACT_NATIVE_EVENT' if epoch is not None else 'UNKNOWN',
            event_ordinal=quality.get('event_ordinal', correction.get('event_ordinal')),
            receipt=stamp, trigger='INFO_CALLBACK', stage=row['stage'],
            tracking_quality=row['tracking_quality'], projected_usable=callback.get('usable'),
            reasons=list(callback.get('reasons', [])),
            last_accepted_visual_time=callback.get('last_accepted_visual_time'),
            accepted_visual_correction=(correction.get('state_source')
                == 'EXPLICIT_NATIVE_ACCEPTED_VISUAL_CORRECTION'
                and type(correction.get('constraint_count')) is int
                and correction['constraint_count'] > 0)))

    def describe(values, good, span='source'):
        episodes = []
        def episode_key(row):
            # Timer ticks share the projected snapshot epoch, not a native
            # callback segment. Receipt spans require one qualified clock.
            boundary = ((row['receipt']['clock_domain'] if row['receipt'] else None)
                if span == 'receipt' else row['observation_segment'])
            return row['classification'], row['epoch'], boundary
        for _, group in itertools.groupby(values, key=episode_key):
            selected = list(group)
            left, right = selected[0], selected[-1]
            stamps = [r['receipt'] for r in selected]
            same_clock = (all(s is not None for s in stamps)
                and len({s['clock_domain'] for s in stamps}) == 1)
            source_span = (right['sensor_ns']-left['sensor_ns']
                if all(integer(r['sensor_ns']) for r in selected) else None)
            episodes.append(dict(classification=left['classification'],
                epoch=left['epoch'], epoch_semantics=left['epoch_semantics'],
                observation_segment=left['observation_segment'],
                first_source_index=left['source_index'], last_source_index=right['source_index'],
                first_sensor_ns=left['sensor_ns'], last_sensor_ns=right['sensor_ns'],
                first_receipt=left['receipt'], last_receipt=right['receipt'],
                observed_source_span_ns=source_span,
                observed_receipt_span_ns=(stamps[-1]['raw_value']-stamps[0]['raw_value']
                    if same_clock else None),
                observed_samples=len(selected),
                accepted_visual_correction_count=sum(r['accepted_visual_correction'] for r in selected),
                next_transition=None))
        offset = 0
        for episode in episodes:
            offset += episode['observed_samples']
            if offset < len(values):
                following = values[offset]
                episode['next_transition'] = {key:following[key] for key in (
                    'classification', 'sensor_ns', 'source_index', 'receipt', 'epoch', 'observation_segment')}
        good_episodes = [e for e in episodes if e['classification'] == good]
        timed_good = [e for e in good_episodes if e['observed_'+span+'_span_ns'] is not None]
        return dict(status='OBSERVED' if values else 'NOT_RUN', observed_samples=len(values), episodes=episodes,
            completeness_scope='recorded observations only; unobserved intervals and tail remain unknown',
            longest_good_episode_basis=span+' clock first-to-last observed span',
            first_good_episode=good_episodes[0] if good_episodes else None,
            longest_good_episode=(max(timed_good,
                key=lambda e: e['observed_'+span+'_span_ns']) if timed_good else None),
            unknown_epoch_samples=sum(r['epoch'] is None for r in values))

    report.update(status='OBSERVED', native_info=describe(rows, 'GOOD'), final_info=rows[-1])
    report['native_info']['native_trace_completeness'] = {
        key:native.get('coverage', {}).get(key) for key in (
            'complete_trace', 'complete_trace_status', 'event_limit_reached', 'paired_spans_complete')}
    report['native_info']['epoch_evidence'] = 'same-ns native correction only; unknowns are not forward-filled'
    if projected:
        report['projected_output'] = describe(projected, 'USABLE', 'receipt')
        report['final_projected_output'] = projected[-1]
        report['final_watchdog'] = next((r for r in reversed(projected)
            if r['trigger'] == 'INDEPENDENT_SCHEDULE'), None)
    propagation = ((states or {}).get('propagation', {})
        if (states or {}).get('status') == 'PASS' else {})
    report['propagation_visual_time_contract'] = dict(
        status='OBSERVED' if type(propagation.get('refreshes_visual_time')) is bool else 'NOT_RUN',
        refreshes_visual_time=propagation.get('refreshes_visual_time'),
        output_complete=propagation.get('output_complete'),
        qualification='sealed adapter metadata; no new per-IMU-row validation')

    fault = fault or {}
    finish = fault.get('fault_window', {}).get('end_sensor_ns')
    if integer(finish):
        postfault = [r for r in rows if r['sensor_ns'] >= finish]
        report['postfault'] = describe(postfault, 'GOOD')
        report['postfault']['not_good_samples'] = sum(r['classification'] == 'NOT_GOOD' for r in postfault)
        report['postfault']['boundary'] = 'fault end inclusive; recorded source time'
    recovery = fault.get('recovery', {})
    end, epoch = recovery.get('continuous_evidence_through_sensor_ns'), recovery.get('native_epoch')
    anchor = next((r for r in rows if r['sensor_ns'] == end), None)
    if (recovery.get('status') == 'OBSERVED' and integer(end) and integer(epoch)
            and anchor is not None and anchor['epoch'] == epoch
            and integer(anchor['observation_segment']) and integer(anchor['event_ordinal'])):
        resets = [r for r in native.get('lifecycle', {}).get('reset_events', [])
            if integer(r.get('event_ordinal')) and r['event_ordinal'] > anchor['event_ordinal']]
        reset = resets[0] if resets else None
        subsequent, boundary = [], 'END_OF_RECORDED_INFO'
        for row in rows:
            if row['sensor_ns'] <= end:
                continue
            if (reset is not None and integer(row['event_ordinal'])
                    and row['event_ordinal'] >= reset['event_ordinal']):
                boundary = 'NEXT_OBSERVED_RESET'
                break
            if (row['observation_segment'] is not None
                    and row['observation_segment'] != anchor['observation_segment']):
                boundary = 'NEXT_OBSERVED_SEGMENT_BOUNDARY'
                break
            if (reset is not None and not integer(row['event_ordinal'])
                    and row['observation_segment'] is None):
                boundary = 'RESET_ORDER_UNKNOWN'
                break
            if row['epoch'] is not None and row['epoch'] != epoch:
                boundary = 'NEXT_OBSERVED_NATIVE_EPOCH'
                break
            item = dict(row)
            if row['epoch'] is None or row['observation_segment'] is None:
                item['classification'] = 'UNKNOWN_EPOCH'
            subsequent.append(item)
        followup = describe(subsequent, 'GOOD')
        bad = [r for r in subsequent if r['classification'] == 'NOT_GOOD']
        followup.update(native_epoch=epoch, second_recovery_sensor_ns=end,
            boundary='second recovery update exclusive; '+boundary,
            next_observed_reset=reset,
            not_good_samples=len(bad),
            not_good_episode_count=sum(e['classification'] == 'NOT_GOOD' for e in followup['episodes']),
            first_subsequent_not_good=bad[0] if bad else None)
        report['after_recovery'] = followup
    return report


def summarize_run(directory):
    directory = Path(directory).resolve()
    hashes, frozen = {}, None

    def read(name, kind='json'):
        path = directory/name
        if not path.is_file():
            return None
        payload = path.read_bytes()
        actual = hashlib.sha256(payload).hexdigest()
        # Read and hash the same bytes; a post-exit modification must not be
        # silently relabelled as original experiment evidence.
        if frozen is not None and path.is_relative_to(directory) and name != 'run.json':
            relative = str(path.relative_to(directory))
            if frozen.get(relative) != actual:
                raise ValueError('replay artifact hash mismatch or undeclared artifact: '+relative)
        hashes[str(path)] = actual
        content = payload.decode()
        if kind == 'jsonl':
            return [json.loads(line) for line in content.splitlines() if line.strip()]
        return json.loads(content) if kind == 'json' else content

    result = read('run.json')
    if result is None:
        return dict(directory=str(directory), status='NOT_RUN', reason='run.json unavailable',
                    input_sha256=hashes, health_episodes=dict(status='NOT_RUN', reason='run.json unavailable'))
    config = result.get('config', {})
    backend = config.get('backend')
    report = dict(directory=str(directory), run_id=config.get('run_id'), backend=backend,
                  status=result['status'], failure_code=result.get('failure_code'),
                  failure_reason=result.get('failure_reason'), sequence=config.get('sequence'),
                  mode=config.get('mode'), rate=config.get('rate'),
                  transport_protocol=result.get('transport_protocol'),
                  execution_started=result.get('execution_started', False),
                  acceptance='original run status retained; summary adds no acceptance gate',
                  operational_validity='UNKNOWN/UNQUALIFIED; numerical poses are not valid-flight states',
                  health_episodes=dict(status='NOT_RUN', reason='sealed SchurVINS health artifacts required'))
    report['runtime_provenance'] = {k:result[k] for k in ['runtime_sha256', 'adapter_sha256',
        'installation_sha256', 'tool_sha256', 'runtime_sources_sha256'] if k in result}
    if backend == 'OpenVINS' and 'mode' not in config:
        # The preserved runner's single supported public baseline has a fixed
        # cam0+IMU half-rate command, unlike the new Schur config's explicit keys.
        report.update(sequence='V1_01_easy', mode='mono', rate=.5,
                      configuration_source='preserved OpenVINS dedicated runner')
    elif backend == 'OpenVINS':
        report['configuration_source'] = 'independent OpenVINS comparison entry'
    data = result.get('input', result.get('dataset', {}))
    report['sensor_counts'] = data.get('counts', data.get('verified_counts'))
    report['source_counts'] = data.get('source_counts', report['sensor_counts'])
    report['input_truth_included'] = data.get('truth_included')
    report['input_reconciliation'] = result.get('input_reconciliation', result.get('evidence'))
    if data.get('dataset') == 'SENSOR_RECORDING':
        report.update(recording_transport_status=result.get('recording_transport_status', 'NOT_RUN'),
            recording_interface_status=result.get('recording_interface_status', 'NOT_RUN'),
            estimator_reproduction='NOT_RUN_NO_REFERENCE', accuracy_qualification='NOT_RUN',
            physical_camera_imu_qualification=False)
    if (result['status'] == 'EXECUTING'
            or result.get('artifact_finalization') == 'NOT_FROZEN_CONTAINER_CLEANUP_UNVERIFIED'):
        # Logs can end mid-write while the estimator is active. Only the run
        # manifest is a stable snapshot; final metrics require process exit.
        reason = ('execution is active' if result['status'] == 'EXECUTING'
                  else 'owned-container absence unverified; outputs may still change')
        report.update(measurements='NOT_RUN: '+reason, input_sha256=hashes,
                      container_cleanup=result.get('container_cleanup'),
                      artifact_integrity=dict(status='NOT_CHECKED', reason=reason))
        return report
    declared = result.get('output_sha256')
    if declared is None or declared == {}:
        # The preserved OpenVINS runner does not finalize output hashes when a
        # historical accuracy gate fails. Keep that failure visible, without
        # claiming its unsealed artifacts have passed an integrity check.
        report['artifact_integrity'] = dict(status='UNLOCKED',
            reason='run manifest has no declared output artifact hashes')
    else:
        if not isinstance(declared, dict):
            raise ValueError('output_sha256 must be a declared artifact map')
        for name, expected in declared.items():
            relative = Path(name)
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('invalid replay artifact path: '+str(name))
            path = directory/relative
            if not path.resolve().is_relative_to(directory) or not path.is_file():
                raise ValueError('declared replay artifact missing or outside run: '+str(name))
            if (not isinstance(expected, str) or len(expected) != 64
                    or any(c not in '0123456789abcdef' for c in expected)
                    or digest(path) != expected):
                raise ValueError('replay artifact hash mismatch: '+str(name))
        frozen = declared
        report['artifact_integrity'] = dict(status='VERIFIED', declared_artifact_count=len(declared),
            policy='all declared artifacts checked; consumed run artifacts must be declared and unchanged')
    execution = read('execution.json')
    if execution is not None:
        report['execution'] = execution
    report['resources_native'] = native_resources(read('native-resource.txt', 'text') or
                                                  read('raw/native-resource.txt', 'text') or '')
    report['resources_container'] = container_resources(read('resources.jsonl', 'jsonl') or
                                                        read('raw/resources.jsonl', 'jsonl') or [])
    info = read('raw/info.jsonl', 'jsonl')
    report['frontend_timing'] = frontend_timing(backend, info, read('raw/timing.txt', 'text'))
    native = read('native-observations.json')
    native_states = read('native-state-output.json')
    if report['artifact_integrity']['status'] == 'VERIFIED':
        report['native_observations'] = native_observation_summary(native, native_states, backend=backend)
        report['fault_response'] = fault_response(data, info, native, native_states)
        report['health_episodes'] = (health_episode_summary(info, native, native_states,
            fault=report['fault_response']) if backend == 'SchurVINS' else dict(
                status='NOT_RUN', reason='native health remains unqualified for this backend'))
    else:
        report['native_observations'] = dict(status='NOT_RUN', reason='sealed native artifacts required')
        report['fault_response'] = dict(status='NOT_RUN', reason='sealed artifacts required for fault-response claims')
    observed = read('observations.json')
    if observed:
        report['initialization_proxy'] = observed.get('initialization_proxy')
        report['native_tail_seen'] = observed.get('native_tail_seen')
        losses = observed.get('tracking_loss_observations')
        observed_losses = isinstance(losses, list) and observed.get('tracking_loss_detection') != 'NOT_RUN'
        report['tracking_loss_observation_count'] = len(losses) if observed_losses else None
        report['tracking_loss_detection'] = observed.get('tracking_loss_detection',
            'native Info stage observations' if observed_losses else 'NOT_RUN')
        report['native_reset_count'] = observed.get('native_reset_count')
        report['camera_lag_at_info_receipt'] = observed.get('camera_lag_at_info_receipt')
    evaluation = read('evaluation.json')
    unscored = evaluation is not None and evaluation.get('status') == 'NOT_RUN'
    if unscored:
        # A sealed recording may expose numerical poses without a reference.
        # Its sensor coverage remains measurable; accuracy and reproduction do
        # not acquire evidence from that coverage or an interface-only result.
        report['reference_scoring'] = {k:evaluation[k] for k in ('status', 'reason', 'qualification')
                                       if k in evaluation}
        report['accuracy_qualification'] = 'NOT_RUN'
    elif evaluation is not None:
        report['coverage'] = {k:v for k,v in evaluation.items() if k in [
            'raw_pose_count', 'epoch_count', 'scored_epoch_count', 'camera_input_count',
            'output_camera_fraction', 'missing_before_first', 'missing_after_last',
            'missing_within_output_span', 'full_source_coverage']}
        report['epochs'] = [{k:v for k,v in e.items() if k not in ['metrics', 'associations']}
                            | ({'metrics':compact_metrics(e['metrics'])} if 'metrics' in e else {})
                            for e in evaluation['epochs']]
        if evaluation['epoch_count'] == 1 and evaluation['scored_epoch_count'] == 1:
            report['metrics'] = compact_metrics(evaluation['epochs'][0]['metrics'])
        report['gt_csv_sha256'] = evaluation['gt_csv_sha256']
    elif data.get('dataset') == 'SENSOR_RECORDING':
        report['reference_scoring'] = dict(status='NOT_RUN', reason='NO_REFERENCE_SUPPLIED',
                                          evaluation_artifact='NOT_AVAILABLE')
    elif backend == 'OpenVINS':
        evaluation = read('native-evaluation/metrics.json')
        if evaluation is not None:
            report['metrics'] = compact_metrics(evaluation)
            report['coverage'] = {k:evaluation[k] for k in [
                'raw_pose_count', 'evaluated_pose_count', 'evaluated_duration_s',
                'excluded_outside_gt', 'association'] if k in evaluation}
            report['gt_csv_sha256'] = evaluation['gt_csv_sha256']
            if config.get('baseline'):
                baseline = read(config['baseline'])
                if baseline is not None:
                    from rm27.perception.vision.localization.experiments.openvins_runtime import (
                        metric_consistency, coverage_consistency)
                    report['historical_baseline_metrics'] = metric_consistency(evaluation, baseline)
                    report['historical_baseline_coverage'] = coverage_consistency(evaluation, baseline)
    # Read the normalized output even when its run failed a later accuracy gate.
    rows = read('online/trajectory.json')
    if rows is not None:
        if isinstance(rows, dict):
            rows = rows['records']
        canonical = [canonical_row(r) for r in rows]
        report['trajectory_count'] = len(canonical)
        if unscored:
            report.setdefault('coverage', {}).update(
                epoch_count=len({r.get('epoch', 0) for r in canonical}), scored_epoch_count=0)
        if canonical:
            report['first_camera_index'] = canonical[0]['source_frame_id']
            report['last_camera_index'] = canonical[-1]['source_frame_id']
            # Fault inputs retain original camera IDs. Delivered counts cannot
            # be the original sequence's denominator after dropping images.
            inputs = (report.get('source_counts') or {}).get('cam0')
            delivered = (report.get('sensor_counts') or {}).get('cam0')
            if inputs is not None:
                coverage = report.setdefault('coverage', {})
                coverage.update(raw_pose_count=len(canonical), camera_input_count=inputs,
                                camera_delivered_input_count=delivered,
                                output_camera_fraction=len(canonical)/inputs,
                                missing_before_first=canonical[0]['source_frame_id'],
                                missing_after_last=inputs-canonical[-1]['source_frame_id']-1,
                                missing_within_output_span=(canonical[-1]['source_frame_id']-
                                    canonical[0]['source_frame_id']+1-len(canonical)))
            report.setdefault('initialization_proxy', dict(
                first_pose_camera_index=canonical[0]['source_frame_id'],
                exact_initialization_completion_time=None,
                source='first saved pose only; not exact internal initialization'))
            if backend == 'OpenVINS':
                cameras = read('raw/cam0.jsonl', 'jsonl')
                if cameras:
                    report['initialization_proxy']['first_pose_sensor_elapsed_s'] = (
                        canonical[0]['image_ns']-cameras[0]['state_ns'])/1e9
    report['input_sha256'] = hashes
    return report


def pairwise_trajectory(left, right, tolerance_ns=6000):
    """First-pose gauge registration, fixed scale, matched original camera IDs."""
    a, b = [canonical_row(r) for r in left], [canonical_row(r) for r in right]
    if not a or not b or len({r.get('epoch', 0) for r in a+b}) != 1:
        return dict(status='NOT_RUN', reason='one nonempty matching epoch required')
    da, db = {r['source_frame_id']:r for r in a}, {r['source_frame_id']:r for r in b}
    if len(da) != len(a) or len(db) != len(b):
        raise ValueError('duplicate source camera ID')
    ids = sorted(da.keys() & db.keys())
    if not ids:
        return dict(status='NOT_RUN', reason='no common source camera frames')
    a, b = [da[i] for i in ids], [db[i] for i in ids]
    # The preserved OpenVINS adapter reconstructs image time from rounded
    # native text, with <=6us source association residual per run. Match the
    # already validated original camera ID; retain that rounding separately.
    image_residual = [x['image_ns']-y['image_ns'] for x,y in zip(a,b)]
    if any(abs(value) > 2*tolerance_ns for value in image_residual):
        raise ValueError('same camera ID has inconsistent associated timestamps')
    ra = rotations([[r['quaternion_xyzw'][3], *r['quaternion_xyzw'][:3]] for r in a])
    rb = rotations([[r['quaternion_xyzw'][3], *r['quaternion_xyzw'][:3]] for r in b])
    pa, pb = np.asarray([r['position'] for r in a]), np.asarray([r['position'] for r in b])
    gauge = rb[0] @ ra[0].T
    translation = np.linalg.norm((pa-pa[0]) @ gauge.T + pb[0]-pb, axis=1)
    error = np.transpose(rb, (0,2,1)) @ (gauge @ ra)
    angle = np.degrees(np.arccos(np.clip((np.trace(error, axis1=1, axis2=2)-1)/2, -1, 1)))
    meaningful = (translation > .001) | (angle > .01)
    first = next((ids[i] for i in range(len(ids)-2)
                  if all(meaningful[i:i+3]) and ids[i+2] == ids[i]+2), None)
    return dict(status='OBSERVED', common_camera_count=len(ids),
                first_camera_index=ids[0], last_camera_index=ids[-1],
                registration='first common pose rigid gauge only; scale=1',
                translation_difference_m=distribution(translation),
                orientation_difference_deg=distribution(angle),
                state_time_difference_ns=distribution([x['state_ns']-y['state_ns'] for x,y in zip(a,b)]),
                reconstructed_image_time_difference_ns=distribution(image_residual),
                per_run_source_association_tolerance_ns=tolerance_ns,
                first_sustained_divergence_camera_index=first,
                divergence_policy='>1mm OR >0.01deg for 3 consecutive camera IDs; diagnostic only')


def summarize_group(name, directories, reports):
    """Explicit cohorts only: never select the successful five after observing outcomes."""
    paths = [str(Path(p).resolve()) for p in directories]
    if len(paths) != len(set(paths)):
        raise ValueError('repeat cohort contains duplicate runs')
    selected = [reports[p] for p in paths]
    groups = {(r.get('backend'), r.get('sequence'), r.get('mode'), r.get('rate')) for r in selected
              if r['status'] != 'NOT_RUN'}
    if len(groups) > 1:
        raise ValueError('repeat cohort mixes backend, sequence, mode or playback rate')
    # Validate the actual fixed config too, not just its descriptive labels.
    configs = []
    for p in paths:
        run = Path(p)/'run.json'
        if run.exists():
            c = json.loads(run.read_text()).get('config', {})
            configs.append({k:v for k,v in c.items() if k not in ['run_id', 'output']})
    if configs and any(c != configs[0] for c in configs[1:]):
        raise ValueError('repeat cohort contains different fixed runtime configurations')
    provenance = [json.dumps(r.get('runtime_provenance', {}), sort_keys=True) for r in selected
                  if r['status'] != 'NOT_RUN']
    scored = [r for r in selected if 'metrics' in r]
    native_initialized = sum(r.get('native_observations', {}).get('initialization', {}).get(
        'exact_metric_completion_observed') is True for r in selected)
    pairs = []
    for left, right in itertools.combinations(scored, 2):
        delta = abs(left['metrics']['ate_rmse_m']-right['metrics']['ate_rmse_m'])
        pair = dict(left=left['run_id'], right=right['run_id'], ate_difference_m=delta,
                    exceeds_1cm_ate_difference=delta > .01)
        lp, rp = Path(left['directory'])/'online/trajectory.json', Path(right['directory'])/'online/trajectory.json'
        if lp.exists() and rp.exists():
            pair['trajectory'] = pairwise_trajectory(load_rows(lp), load_rows(rp))
        pairs.append(pair)
    return dict(name=name, planned_run_count=len(paths), required_run_count=5,
                cohort_complete=len(paths) == 5 and all(r['status'] not in ['NOT_RUN','EXECUTING'] for r in selected),
                run_directories=paths, successful_run_count=sum(r['status'] in ['PASS','EXECUTION_COMPLETED'] for r in selected),
                failed_run_count=sum(r['status'] in ['FAILED','REFUSED'] for r in selected),
                scored_run_count=len(scored),
                initialization_proxy_count=sum(bool(r.get('initialization_proxy')) for r in selected),
                initialization_policy='first saved pose proxy only; exact transition and operational readiness unknown',
                native_metric_initialization_observed_run_count=native_initialized,
                native_metric_initialization_observed_fraction_of_planned=(native_initialized/len(paths) if paths else None),
                native_initialization_policy=('sealed native metric commit/completion observations only; later '
                    'failures and commit-only traces counted, missing members retained in denominator; not readiness'),
                recorded_provenance_variant_count=len(set(provenance)),
                provenance_policy='configuration equality does not assert identical tool bytes; inspect each run provenance',
                ate_rmse_m=distribution([r['metrics']['ate_rmse_m'] for r in scored]),
                rpe_1s_translation_rmse_m=distribution([r['metrics']['rpe_1s_translation_rmse_m']
                    for r in scored if r['metrics']['rpe_1s_translation_rmse_m'] is not None]),
                pair_count=len(pairs), pairs_exceeding_1cm=sum(p['exceeds_1cm_ate_difference'] for p in pairs),
                pairs=pairs, qualification='observed cohort only; original 1cm gate unchanged, no population or flight bound')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', action='append', type=Path, default=[], help='include immediate child directories with run.json')
    parser.add_argument('--run', action='append', type=Path, default=[])
    parser.add_argument('--repeat-group', action='append', nargs='+', default=[], metavar='NAME_OR_RUN',
                        help='explicit name followed by the five planned run directories')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    paths = set(args.run)
    for root in args.root:
        paths.update(p.parent for p in root.glob('*/run.json'))
    for group in args.repeat_group:
        if len(group) < 2:
            raise ValueError('repeat group requires a name and planned directories')
        paths.update(Path(p) for p in group[1:])
    if not paths:
        raise ValueError('no replay runs selected')
    reports = {str(p.resolve()):summarize_run(p) for p in sorted(paths)}
    groups = [summarize_group(g[0], g[1:], reports) for g in args.repeat_group]
    result = dict(schema_version=1, runs=list(reports.values()), repeat_groups=groups,
                  interpretation='mode/input differences remain explicit; no inter-algorithm winner gate',
                  source_sha256=digest(Path(__file__)))
    with args.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
        f.write('\n')
    return result


if __name__ == '__main__':
    main()
