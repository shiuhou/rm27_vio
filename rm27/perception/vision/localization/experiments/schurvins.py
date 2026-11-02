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
    require(dataset in ('EuRoC', 'UZH_FPV', 'SENSOR_RECORDING'), 'DATASET_INVALID', 'explicit sensor clock identity')
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


def _verified_imu_delivery(contract):
    """Verify a declared delivered ledger without changing original identities.

    The caller's frozen prepared manifest is the trust root. The local ledger,
    declaration and parent-source hashes must remain bound to it. This proves
    declared offline payload provenance, not physical raw samples/integration.
    Legacy source-only contracts deliberately need no new file proof.
    """
    if 'delivered_rows' not in contract and 'delivery_evidence' not in contract:
        return None
    import json
    from pathlib import Path
    from .common import checked_file, read_json
    proof = contract.get('delivery_evidence')
    require(isinstance(proof, dict) and set(proof) == {
        'schema_version', 'experiment_kind', 'prepared_input_path', 'prepared_input_sha256'}
        and proof['schema_version'] == 1 and type(proof['schema_version']) is int
        and proof['experiment_kind'] == 'ARTIFICIAL_DISTURBANCE'
        and contract.get('delivery_semantics') == 'DECLARED_DISTURBANCE_PAYLOAD_ORIGINAL_SAMPLE_IDENTITY',
        'PROPAGATION_INPUT_INVALID', 'explicit sealed artificial delivery proof required')

    def local(path, digest):
        path = Path(path)
        require(path.is_absolute() and str(path) == str(path.resolve()),
                'PROPAGATION_INPUT_INVALID', 'canonical non-symlink proof path required')
        checked_file(path, digest)
        return path

    manifest_path = local(proof['prepared_input_path'], proof['prepared_input_sha256'])
    require(manifest_path.name == 'input.json', 'PROPAGATION_INPUT_INVALID', 'prepared input.json required')
    manifest = read_json(manifest_path)
    declaration = manifest.get('derivation', {})
    require(manifest.get('purpose') == 'EUROC_PUBLIC_REPLAY' and manifest.get('truth_included') is False
            and declaration.get('experiment_kind') == 'ARTIFICIAL_DISTURBANCE'
            and declaration.get('validation', {}).get('status') == 'PASS',
            'PROPAGATION_INPUT_INVALID', 'qualified declared EuRoC disturbance required')
    hashes = manifest.get('artifacts_sha256', {})
    root = manifest_path.parent
    paths = {name:local(root/name, hashes[name]) for name in
             ('source-expected.jsonl', 'expected.jsonl', 'derivation.json')}
    require(read_json(paths['derivation.json']) == declaration,
            'PROPAGATION_INPUT_INVALID', 'manifest and sealed derivation differ')
    parent = Path(declaration['parent_input_path'])
    local(parent/'input.json', declaration['parent_input_sha256'])
    parent_hashes = declaration['parent_artifacts_sha256']
    parent_name = 'source-expected.jsonl' if 'source-expected.jsonl' in parent_hashes else 'expected.jsonl'
    local(parent/parent_name, parent_hashes[parent_name])
    require(parent_hashes[parent_name] == hashes['source-expected.jsonl']
            and contract.get('source_imu_id') == 'EuRoC:'+manifest['original_archive_sha256']+':imu0',
            'PROPAGATION_INPUT_INVALID', 'full original source identity must remain unchanged')

    def imu_rows(path):
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        require(all(isinstance(row, dict) for row in rows), 'PROPAGATION_INPUT_INVALID', 'ledger objects required')
        return [row for row in rows if row.get('stream') == 'imu0']

    sources, delivered = imu_rows(paths['source-expected.jsonl']), imu_rows(paths['expected.jsonl'])
    require(sources and delivered and contract.get('source_rows') == sources
            and contract.get('delivered_rows') == delivered
            and len(sources) == manifest.get('source_counts', manifest['counts'])['imu0']
            and len(delivered) == manifest['counts']['imu0'],
            'PROPAGATION_INPUT_INVALID', 'contract must retain exact sealed original/delivered rows')
    identities = set()
    for rows in (sources, delivered):
        previous = None
        for row in rows:
            stamp = row.get('state_ns')
            require(set(row) == {'stream', 'state_ns', 'gyro', 'accel'}
                    and type(stamp) is int and stamp >= 0
                    and (previous is None or stamp > previous),
                    'PROPAGATION_INPUT_INVALID', 'unique increasing original/delivered IMU identities required')
            for field in ('gyro', 'accel'):
                require(isinstance(row[field], list) and len(row[field]) == 3,
                        'PROPAGATION_INPUT_INVALID', 'three finite SI components required')
                for value in row[field]:number(value, 'declared delivered IMU '+field)
            if rows is sources:identities.add(stamp)
            else:require(stamp in identities, 'PROPAGATION_INPUT_INVALID', 'delivery cannot invent source identities')
            previous = stamp
    return {row['state_ns']:row for row in delivered}


def project_native_corrections(native_analysis, raw_info, *, policy, run_id, sequence,
                               binary_sha256, dataset='EuRoC', watchdog_events=(),
                               watchdog_complete=False, raw_imu=None, imu_input_contract=None,
                               propagation_output_path=None):
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
    from ..state_output import NativeCorrectionEvidence, PropagationSample, StateOutputPolicy, StateOutputTracker

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
                  corrections=[], outputs=[], rejected_events=[], health_transitions=[], evidence_errors=[],
                  policy=dict(policy.__dict__))
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
    # Consume same-run observer IMU with its original sample identities and,
    # where declared, the sealed artificial disturbance's delivered values.
    # A missing/unqualified binding keeps the historical correction-only path.
    if raw_imu is not None and isinstance(imu_input_contract, dict):
        require(imu_input_contract.get('status') != 'FAIL', 'PROPAGATION_INPUT_INVALID',
                imu_input_contract.get('reason', 'qualified IMU binding failed'))
    imu_enabled = raw_imu is not None and isinstance(imu_input_contract, dict) and imu_input_contract.get('status') == 'PASS'
    imu_rows, imu_meta, state_timestamp_epochs = [], None, {}
    if raw_imu is not None:
        result['propagation'] = dict(status='NOT_RUN', reason=(
            imu_input_contract.get('reason', 'IMU_INPUT_BINDING_UNAVAILABLE') if isinstance(imu_input_contract, dict)
            else 'IMU_INPUT_BINDING_UNAVAILABLE'), source='SAME_RUN_OBSERVER_IMU',
            numerical_method='INDEPENDENT_SNAPSHOT_NOT_NATIVE_FILTER_OUTPUT')
    if imu_enabled:
        import hashlib
        from pathlib import Path
        require(isinstance(raw_imu, list) and raw_imu and propagation_output_path is not None,
                'PROPAGATION_INPUT_INVALID', 'observed IMU list and fresh bounded JSONL path required')
        contract = imu_input_contract
        require(contract.get('evidence_status') == 'VERIFIED' and contract.get('measurement_kind') == 'RAW_SPECIFIC_FORCE'
                and contract.get('gyro_unit') == 'rad/s' and contract.get('accel_unit') == 'm/s^2'
                and contract.get('native_imu_frame') == 'imu' and contract.get('timestamp_offset_ns') == 0,
                'PROPAGATION_INPUT_INVALID', 'audited SI/frame/delay-zero binding required')
        sources = contract.get('source_rows')
        require(isinstance(sources, list) and sources, 'PROPAGATION_INPUT_INVALID', 'full original IMU source ledger required')
        by_sensor = {}
        for index, row in enumerate(sources):
            sensor = row.get('state_ns')
            require(type(sensor) is int and sensor >= 0 and sensor not in by_sensor
                    and (not by_sensor or sensor > sources[index-1]['state_ns']),
                    'PROPAGATION_INPUT_INVALID', 'original IMU ledger order/identity required')
            by_sensor[sensor] = (index, row)
        try:
            delivered = _verified_imu_delivery(contract)
        except (ValueError, KeyError, OSError, TypeError) as exc:
            require(False, 'PROPAGATION_INPUT_INVALID', 'derived delivery proof invalid: '+str(exc))
        previous_sensor = previous_receive = None
        for ordinal, row in enumerate(raw_imu):
            sensor, received = row.get('state_ns'), row.get('observed_monotonic_ns')
            require(type(sensor) is int and sensor in by_sensor and type(received) is int and received >= 0
                    and (previous_sensor is None or sensor > previous_sensor)
                    and (previous_receive is None or received >= previous_receive),
                    'PROPAGATION_INPUT_INVALID', 'IMU source/observer order must increase; no sorting')
            source_index, source = by_sensor[sensor]
            expected = source if delivered is None else delivered.get(sensor)
            require(expected is not None and row.get('frame_id') == contract.get('raw_ros_frame_id')
                    and row.get('gyro') == expected.get('gyro') and row.get('accel') == expected.get('accel'),
                    'PROPAGATION_INPUT_INVALID', 'raw IMU payload/frame differs from reconciled delivery')
            imu_rows.append(dict(row, source_sequence=source_index, observer_ordinal=ordinal))
            previous_sensor, previous_receive = sensor, received
        target = Path(propagation_output_path)
        require(not target.exists() and target.parent.is_dir(), 'PROPAGATION_OUTPUT_INVALID', 'fresh existing-parent JSONL required')
        # The fixed byte cap is checked per row. Overflow fails; no output is
        # silently omitted and a partial artifact cannot carry PASS.
        imu_meta = dict(status='NOT_RUN', source='SAME_RUN_OBSERVER_IMU',
            numerical_method='INDEPENDENT_SNAPSHOT_NOT_NATIVE_FILTER_OUTPUT',
            input_contract={k:v for k,v in contract.items() if k not in ('source_rows', 'delivered_rows')},
            sample_count=len(imu_rows), output_count=0, usable_output_count=0,
            rejected_sample_count=0, first_state_ns=None, last_state_ns=None,
            first_output_sensor_ns=None, last_output_sensor_ns=None,
            output_sensor_frequency_basis='EMITTED_ROWS_OVER_ORIGINAL_IMU_SOURCE_TIME',
            state_timestamp_frequency_semantic='distinct emitted state timestamps per epoch over active state-time span; not visual updates, usable-state availability or native filter output rate',
            artifact=str(target), maximum_record_bytes=4096, bytes_written=0,
            output_complete=False, output_kind='FULL_LOCALIZATION_ESTIMATE_V2_COMPACT_JSONL',
            preserves_raw_sensor_time=True, refreshes_visual_time=False)
        result['propagation'] = imu_meta
    candidates = native_analysis.get('visual_corrections', {}).get('events', [])
    if native_analysis.get('status') != 'PASS' or not candidates:
        result['reason'] = 'COMPLETE_ACCEPTED_NATIVE_CORRECTION_EVIDENCE_UNAVAILABLE'
        return result
    quality_analysis = native_analysis.get('tracking_quality', {})
    if quality_analysis.get('status') != 'PASS' or not quality_analysis.get('events'):
        result['reason'] = 'CURRENT_NATIVE_QUALITY_DECISION_EVIDENCE_UNAVAILABLE'
        return result
    qualities = {}
    for decision in quality_analysis['events']:
        qualities.setdefault(decision['sensor_ns'], []).append(decision)
    initialization = native_analysis.get('initialization', {})
    initialized = initialization.get('exact_metric_events', [])
    commits = initialization.get('metric_filter_commits', [])
    by_time = {row['sensor_ns']: row for row in candidates}
    tracker = StateOutputTracker(policy)
    sensor_clock = dataset+':'+sequence+':sensor_time'
    receive_clock, publish_clock = run_id+':observer_monotonic', run_id+':offline_projection_monotonic'
    seen = set()
    # Merge validated streams rather than silently sorting either source.
    streams = [('INFO_CALLBACK', raw_info), ('INDEPENDENT_SCHEDULE', watchdog_events)]
    if imu_enabled:
        streams.append(('IMU_CALLBACK', imu_rows))
    cursors, timeline = [0]*len(streams), []
    while any(cursors[n] < len(rows) for n, (_, rows) in enumerate(streams)):
        n = min((n for n, (_, rows) in enumerate(streams) if cursors[n] < len(rows)),
                key=lambda n: (streams[n][1][cursors[n]]['observed_monotonic_ns'], n))
        trigger, rows = streams[n]
        timeline.append((trigger, cursors[n], rows[cursors[n]]))
        cursors[n] += 1
    def reject(sensor, reason, *, health=False, correction_candidate=True):
        row = dict(sensor_ns=sensor, reason=reason,
                   category='HEALTH_TRANSITION' if health else 'EVIDENCE_ERROR')
        if correction_candidate:
            result['rejected_events'].append(row)
        result['health_transitions' if health else 'evidence_errors'].append(row)
        tracker.mark_lost(reason)

    latest_imu_source = None
    correction_hashes = {}
    propagation_stream = open(propagation_output_path, 'x') if imu_enabled else None
    try:
        latest_info_sensor = None
        for trigger, ordinal, info in timeline:
            received = info['observed_monotonic_ns']
            if trigger == 'IMU_CALLBACK':
                source = TimestampEvidence(info['state_ns'], 'ns', sensor_clock, 'STATE_ESTIMATE', 'VERIFIED')
                latest_imu_source = source
                receipt = TimestampEvidence(received, 'ns', receive_clock, 'RECORDED_IMU_CALLBACK_RECEIPT', 'VERIFIED')
                publication = TimestampEvidence(time.monotonic_ns(), 'ns', publish_clock, 'OFFLINE_ADAPTER_EMISSION', 'VERIFIED')
                try:
                    sample = PropagationSample(time=source, sequence=info['source_sequence'],
                        imu_id=imu_input_contract['source_imu_id'], imu_frame=imu_input_contract['native_imu_frame'],
                        gyro_rad_s=info['gyro'], specific_force_m_s2=info['accel'],
                        calibration_source=imu_input_contract['calibration_source'], evidence_status='VERIFIED')
                    tracker.push_imu(sample, received_time=receipt)
                except (KeyError, TypeError, ValueError) as exc:
                    imu_meta['rejected_sample_count'] += 1
                    tracker.mark_lost('PROPAGATION_IMU_REJECTED')
                    reject(info['state_ns'], 'PROPAGATION_IMU_REJECTED:'+str(exc),
                           health=str(exc) in ('duplicate or out-of-order IMU', 'IMU sequence or sampling gap'))
                state = tracker.output(publish_time=publication, now_receive_time=receipt)
                if state.estimate is not None:
                    record = state.estimate.to_dict()
                    quality = dict(record.get('quality') or {})
                    state_quality = quality.get('state_output', {})
                    correction_id = state_quality.get('correction_id')
                    quality['native_correction'] = dict(artifact='native-state-output.json',
                        correction_id=correction_id, correction_sha256=correction_hashes.get(correction_id))
                    record['quality'] = quality
                    derived = dict(raw_sensor_ns=info['state_ns'], source_sequence=info['source_sequence'],
                        observer_ordinal=info['observer_ordinal'], estimate=record, usable=state.usable, reasons=list(state.reasons))
                    encoded = canonical(derived)+'\n'
                    count = len(encoded.encode())
                    require(count <= imu_meta['maximum_record_bytes'], 'PROPAGATION_RECORD_LIMIT',
                            'complete derived record exceeds explicit cap; artifact is incomplete')
                    propagation_stream.write(encoded)
                    imu_meta['bytes_written'] += count
                    imu_meta['output_count'] += 1
                    imu_meta['usable_output_count'] += int(state.usable)
                    if imu_meta['first_state_ns'] is None:
                        imu_meta['first_state_ns'] = state.estimate.source_time.raw_value
                    imu_meta['last_state_ns'] = state.estimate.source_time.raw_value
                    if imu_meta['first_output_sensor_ns'] is None:
                        imu_meta['first_output_sensor_ns'] = info['state_ns']
                    imu_meta['last_output_sensor_ns'] = info['state_ns']
                    state_ns, epoch = state.estimate.source_time.raw_value, record['localization_epoch']
                    if epoch not in state_timestamp_epochs:
                        state_timestamp_epochs[epoch] = dict(native_epoch=epoch, row_count=0,
                            first_state_ns=state_ns, last_state_ns=state_ns,
                            distinct_state_timestamp_count=1, repeated_state_timestamp_count=0,
                            state_time_rewind_count=0)
                    epoch_stats = state_timestamp_epochs[epoch]
                    if epoch_stats['row_count']:
                        if state_ns < epoch_stats['last_state_ns']:
                            epoch_stats['state_time_rewind_count'] += 1
                            epoch_stats['distinct_state_timestamp_count'] = None
                        elif state_ns == epoch_stats['last_state_ns']:
                            epoch_stats['repeated_state_timestamp_count'] += 1
                        elif epoch_stats['distinct_state_timestamp_count'] is not None:
                            epoch_stats['distinct_state_timestamp_count'] += 1
                    epoch_stats['row_count'] += 1
                    epoch_stats['last_state_ns'] = state_ns
                continue
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
                method = matched.get('initialization_method', 'MONOCULAR_VISUAL_IMU_ALIGNMENT') if matched else None
                if method not in ('MONOCULAR_VISUAL_IMU_ALIGNMENT', 'CALIBRATED_STEREO_BASELINE_WITH_UPSTREAM_IMU_PRIORS'):
                    reason = reason or 'NATIVE_INITIALIZATION_METHOD_UNSUPPORTED'
                if matched and matched.get('native_epoch', event['native_epoch']) != event['native_epoch']:
                    reason = reason or 'NATIVE_INITIALIZATION_EPOCH_MISMATCH'
                decisions = [row for row in qualities.get(sensor, [])
                             if row.get('observation_segment') == event.get('observation_segment')
                             and row.get('bundle_id') == event.get('bundle_id')]
                if (len(decisions) != 1 or decisions[0].get('visual_correction') is not True
                        or decisions[0].get('tracking_quality') != info['tracking_quality']):
                    reason = reason or 'EXACT_CURRENT_NATIVE_QUALITY_ASSOCIATION_UNAVAILABLE'
                epoch = event.get('native_epoch')
                if tracker.epoch is not None and epoch != tracker.epoch:
                    transitions = native_analysis.get('lifecycle', {}).get('native_epoch_transitions', [])
                    if not any(row.get('native_epoch') == epoch
                               and row.get('previous_epoch') == tracker.epoch
                               and type(row.get('event_ordinal')) is int
                               and row['event_ordinal'] <= event.get('event_ordinal', -1)
                               and row.get('observation_segment') == event.get('observation_segment')
                               and not row.get('reset_event_missing') for row in transitions):
                        reason = reason or 'NATIVE_RESET_EVIDENCE_REQUIRED'
                # Validate evidence first: bad quality cannot conceal a missing
                # initialization, reset or accepted visual constraint.
                unhealthy = info['stage'] != 2 or info['tracking_quality'] != 2
                if reason is not None:
                    reject(sensor, reason)
                elif unhealthy:
                    reject(sensor, 'NATIVE_INFO_NOT_TRACKING_GOOD', health=True)
                else:
                    try:
                        epoch = event['native_epoch']
                        if tracker.epoch is not None and epoch != tracker.epoch:
                            transitions=native_analysis.get('lifecycle', {}).get('native_epoch_transitions', [])
                            require(any(row.get('native_epoch') == epoch
                                        and row.get('previous_epoch') == tracker.epoch
                                        and type(row.get('event_ordinal')) is int
                                        and row['event_ordinal'] <= event['event_ordinal']
                                        and row.get('observation_segment') == event['observation_segment']
                                        and not row.get('reset_event_missing') for row in transitions),
                                    'NATIVE_RESET_EVIDENCE_REQUIRED', 'explicit parsed reset segment transition')
                            tracker.reset(epoch, reason='EXPLICIT_NATIVE_RESET_SEGMENT', received_time=receipt)
                            latest_imu_source = None
                        provenance = dict(binary_sha256=binary_sha256, native_source=native_analysis.get('source'),
                                          observation_segment=event['observation_segment'],
                                          event_ordinal=event['event_ordinal'], info_ordinal=ordinal,
                                          native_publication_timestamp=None, native_processing_completed_timestamp=None,
                                          covariance_status='UNKNOWN_NOT_EXPORTED',
                                          candidate_scope='OFFLINE_INTERFACE_NOT_FLIGHT_QUALIFIED',
                                          initialization_method=method,
                                          upstream_initialization_assumptions=matched.get('upstream_initialization_assumptions', []))
                        estimate = LocalizationEstimate(source_time=source, publish_time=publication,
                            parent_frame='backend_world', child_frame='imu', translation=event['position'],
                            orientation_wxyz=event['orientation_wxyz'], tracking_state=TrackingState.TRACKING,
                            valid=True, initialized=True, relocalizing=False, localization_epoch=epoch,
                            scale_state=ScaleState.METRIC, translation_unit='m',
                            velocity=event['velocity_parent_m_s'], velocity_frame='backend_world', velocity_unit='m/s',
                            quality=dict(native_correction=provenance))
                        evidence = NativeCorrectionEvidence(estimate=estimate, last_accepted_visual_time=source,
                            correction_id=run_id+':native:'+str(event['event_ordinal']),
                            initialization_source=('stereo_metric_geometry_ready+stereo_filter_state_commit+stereo_metric_initialized:same_native_segment'
                                if method == 'CALIBRATED_STEREO_BASELINE_WITH_UPSTREAM_IMU_PRIORS'
                                else 'metric_filter_commit+mono_metric_initialized:same_native_segment'),
                            scale_source=('PINNED_CALIBRATED_STEREO_BASELINE'
                                if method == 'CALIBRATED_STEREO_BASELINE_WITH_UPSTREAM_IMU_PRIORS'
                                else 'NATIVE_VISUAL_IMU_ALIGNMENT_NO_TRUTH'),
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
                        if imu_enabled:
                            correction_hashes[evidence.correction_id] = hashlib.sha256(canonical(result['corrections'][-1]).encode()).hexdigest()
                    except (KeyError, TypeError, ValueError) as exc:
                        reason = 'NATIVE_CORRECTION_REJECTED:'+str(exc)
                        reject(sensor, reason)
            elif info['stage'] != 2 or info['tracking_quality'] != 2:
                reject(sensor, 'NATIVE_INFO_NOT_TRACKING_GOOD', health=True, correction_candidate=False)
            # Delayed Info can describe a correction older than buffered IMU. The
            # tracker snapshot has already replayed that buffer; never rewind its now.
            output = tracker.output(publish_time=publication, now_receive_time=receipt,
                sensor_now=(max((source, latest_imu_source), key=lambda value:value.raw_value)
                            if imu_enabled and latest_imu_source is not None else source))
            result['outputs'].append(dict(sensor_ns=sensor, trigger=trigger,
                                          observation_timestamp=receipt.to_dict(), **output.to_dict()))
    finally:
        if propagation_stream is not None:
            propagation_stream.close()
    if imu_enabled:
        from .common import file_hash
        imu_meta.update(status='FAIL' if imu_meta['rejected_sample_count'] else ('PASS' if imu_meta['output_count'] else 'NOT_RUN'),
            output_complete=True, artifact_sha256=file_hash(propagation_output_path))
        # Every emitted row has an original IMU acquisition timestamp. Invalid
        # snapshots may retain an old state timestamp, so their row count must
        # never be divided by the shorter state-time span.
        span = (imu_meta['last_output_sensor_ns']-imu_meta['first_output_sensor_ns']) if imu_meta['output_count'] else 0
        imu_meta['output_sensor_frequency_hz'] = ((imu_meta['output_count']-1)*1e9/span if span > 0 else None)
        for epoch in state_timestamp_epochs.values():
            state_span = epoch['last_state_ns']-epoch['first_state_ns']
            distinct = epoch['distinct_state_timestamp_count']
            epoch['state_timestamp_frequency_hz'] = ((distinct-1)*1e9/state_span
                if distinct is not None and state_span > 0 else None)
        imu_meta['state_timestamp_epochs'] = list(state_timestamp_epochs.values())
    for sensor in by_time.keys()-seen:
        reject(sensor, 'EXACT_INFO_ASSOCIATION_UNAVAILABLE')
    result['status'] = 'FAIL' if result['evidence_errors'] else ('PASS' if result['corrections'] else 'NOT_RUN')
    result['status_semantics'] = 'evidence/interface integrity; health transitions remain invalid and do not imply continuous availability'
    return result
