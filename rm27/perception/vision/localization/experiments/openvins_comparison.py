"""Independent EuRoC mono/stereo comparison; original OpenVINS gate unchanged.

Reuses verified transport, native state reconciliation and fixed-scale SE3
evaluation. This entry does not confer VIO-P, RK3576 or operational validity.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import shlex
import sqlite3
import subprocess
import uuid
import zipfile

from .common import checked_file, cleanup_container, file_hash, read_json, require, write_json
from .openvins import PIN
from .openvins_runtime import StreamOrder, create_output, lines, normalize, state_row
from tools.openvins.comparison_transport import artificial_input, events_from, image_pixels, storage_events
from tools.openvins.native_accuracy import associate
from tools.schurvins.evaluate import evaluate_rows, read_reference


class ComparisonDataset:
    """Explicit sequence/mode input, without weakening EurocVioDataset."""
    def __init__(self, archive, bag, sequence, mode, source_sha256, *,
                 original_reference_archive=None, original_reference_sha256=None, prepared_input=None, rate=None):
        self.archive, self.bag = Path(archive).resolve(), Path(bag).resolve()
        self.artificial = original_reference_archive is not None
        self.reference_archive = Path(original_reference_archive).resolve() if self.artificial else self.archive
        self.sequence, self.mode = sequence, mode
        require(isinstance(sequence, str) and bool(sequence), 'SEQUENCE_INVALID', 'explicit EuRoC name required')
        v = read_json(self.bag/'source_verification.json')
        purpose = 'EUROC_ARTIFICIAL_COMPARISON' if self.artificial else 'EUROC_PUBLIC_COMPARISON'
        require(v.get('purpose') == purpose and v.get('sequence') == sequence
                and v.get('mode') == mode, 'COMPARISON_INPUT_MISMATCH', 'sequence/mode differ')
        require(v.get('readback_exact') is True and v.get('truth_included') is False,
                'COMPARISON_INPUT_UNVERIFIED', 'complete sensor-only bag readback required')
        require(v.get('source_sha256') == source_sha256, 'COMPARISON_SOURCE_MISMATCH', 'explicit source hash')
        checked_file(self.archive, source_sha256)
        if self.artificial:
            checked_file(self.reference_archive, original_reference_sha256)
            manifest, sensor_archive, schedule = artificial_input(prepared_input, self.reference_archive, mode)
            require(sensor_archive == self.archive and v.get('derived_sensor_archive_sha256') == source_sha256
                    and v.get('original_reference_archive_sha256') == original_reference_sha256
                    and v.get('prepared_input_sha256') == file_hash(Path(prepared_input)/'input.json')
                    and v.get('delivery_schedule') == schedule and v.get('playback_rate') == rate,
                    'COMPARISON_FAULT_MISMATCH', 'derived source, reference, schedule or playback rate differs')
            self.prepared_input = Path(prepared_input).resolve()
        self.provenance = v
        if self.artificial:
            # Keep the sealed bag manifest unchanged while carrying the full
            # original denominator and fault interval into runtime summaries.
            self.provenance = dict(v, derivation=manifest['derivation'],
                source_counts=manifest['source_counts'], source_first_ns=manifest['source_first_ns'],
                source_last_ns=manifest['source_last_ns'])
        self.manifest_sha256 = file_hash(self.bag/'source_verification.json')
        self.verify_bag()
        expected = {'/cam0/image_raw': 'sensor_msgs/msg/Image', '/imu0': 'sensor_msgs/msg/Imu'}
        if mode == 'stereo':
            expected['/cam1/image_raw'] = 'sensor_msgs/msg/Image'
        dbs = list(self.bag.glob('*.db3'))
        require(len(dbs) == 1, 'BAG_FORMAT_UNSUPPORTED', 'single verified sqlite3 bag')
        with sqlite3.connect(dbs[0].as_uri() + '?mode=ro', uri=True) as db:
            topics = dict(db.execute('SELECT name, type FROM topics'))
        require(topics == expected, 'TRUTH_ISOLATION_FAILED', 'exact camera/IMU topic set required')
        with zipfile.ZipFile(self.archive) as z:
            streams, _ = events_from(z, mode)
            def original_rows(sensor):
                return list(csv.reader(line for line in z.read('mav0/' + sensor + '/data.csv').decode().splitlines()
                                       if line.strip() and not line.lstrip().startswith('#')))
            self.camera, self.imu = original_rows('cam0'), original_rows('imu0')
        for sensor, rows in streams.items():
            require(len(rows) == v['verified_counts'][sensor], 'COMPARISON_COUNT_MISMATCH', sensor)
        self.streams = streams
        self.camera_ns = [r[0] for r in streams['cam0']]
        self.imu_ns = [r[0] for r in streams['imu0']]
        self.original_camera_ns = self.camera_ns
        self.source_ids = list(range(len(self.camera_ns)))
        if self.artificial:
            with zipfile.ZipFile(self.reference_archive) as z:
                original_streams, _ = events_from(z, mode)
            self.original_camera_ns = [ns for ns, _ in original_streams['cam0']]
            source_id = {ns:i for i, ns in enumerate(self.original_camera_ns)}
            require(all(ns in source_id for ns in self.camera_ns),
                    'COMPARISON_SOURCE_ID_INVALID', 'derived image lacks an original camera ID')
            self.source_ids = [source_id[ns] for ns in self.camera_ns]
            self.delivered_streams = {s:[] for s in streams}
            for _, ns, sensor, payload in storage_events(streams, rate, schedule):
                self.delivered_streams[sensor].append((ns, payload))

    def verify_bag(self):
        """Every mounted file must belong to the sensor-only frozen bag."""
        artifacts = self.provenance.get('artifacts_sha256', {})
        db_names = [name for name in artifacts if name.endswith('.db3')]
        require(len(db_names) == 1 and set(artifacts) == {'metadata.yaml', db_names[0]},
                'BAG_ARTIFACT_INVALID', 'one frozen database and metadata required')
        for name in artifacts:
            require(Path(name).name == name, 'BAG_ARTIFACT_INVALID', 'bag-local file names')
        entries = list(self.bag.iterdir())
        require({p.name for p in entries} == set(artifacts) | {'source_verification.json'}
                and all(p.is_file() and not p.is_symlink() for p in entries),
                'TRUTH_ISOLATION_FAILED', 'only frozen sensor bag files may be mounted')
        checked_file(self.bag/'source_verification.json', self.manifest_sha256)
        for name, digest in artifacts.items():
            checked_file(self.bag/name, digest)


def set_sequence_clock(record, sequence):
    """Name this explicit dataset clock while preserving raw integer values."""
    clock = 'EuRoC:' + sequence + ':sensor_time'
    for key in ['source_timestamp', 'backend_timestamp']:
        record[key]['clock_domain'] = clock
    record['localization_fields']['source_time']['clock_domain'] = clock
    return record


def verify_second_camera(raw, dataset):
    if dataset.mode != 'stereo':
        return None
    observed = lines(raw/'cam1.jsonl')
    expected = dataset.streams['cam1']
    order = StreamOrder()
    for r in observed:
        order.push('cam1', r['state_ns'])
    require([r['state_ns'] for r in observed] == [ns for ns, _ in expected],
            'CAM1_DELIVERY_MISMATCH', 'complete exactly synchronized right-camera observer stream')
    import hashlib
    with zipfile.ZipFile(dataset.archive) as z:
        for row, (_, filename) in zip(observed, expected):
            pixels = image_pixels(z, 'cam1', filename)
            require((row['height'], row['width']) == pixels.shape and row['encoding'] == 'mono8'
                    and row['step'] == row['width'] and row['is_bigendian'] == 0
                    and row['frame_id'] == 'cam1'
                    and row['pixels_sha256'] == hashlib.sha256(pixels.tobytes()).hexdigest(),
                    'CAM1_PIXELS_MISMATCH', 'observer right image versus source')
    return dict(count=len(observed), first_ns=observed[0]['state_ns'], last_ns=observed[-1]['state_ns'],
                scope='observer receipt only; estimator image callback coverage not instrumented')


def audit_artificial_delivery(raw, dataset):
    """Compare receipt order with declared delivery, without sorting raw data."""
    import hashlib
    evidence = {}
    with zipfile.ZipFile(dataset.archive) as source:
        for sensor, wanted in dataset.delivered_streams.items():
            observed = lines(raw/(sensor+'.jsonl'))
            require([row['state_ns'] for row in observed] == [ns for ns, _ in wanted],
                    'ARTIFICIAL_DELIVERY_MISMATCH', sensor+' declared schedule versus receipt order')
            for row, (_, payload) in zip(observed, wanted):
                require(row.get('frame_id') == sensor, 'INPUT_FRAME_INVALID', sensor)
                if sensor == 'imu0':
                    require(row['gyro']+row['accel'] == payload,
                            'IMU_VALUES_MISMATCH', 'derived SI input versus observer')
                else:
                    pixels = image_pixels(source, sensor, payload)
                    require((row['height'], row['width']) == pixels.shape and row['encoding'] == 'mono8'
                            and row['step'] == row['width'] and row['is_bigendian'] == 0
                            and row['pixels_sha256'] == hashlib.sha256(pixels.tobytes()).hexdigest(),
                            'CAMERA_PIXELS_MISMATCH', sensor+' derived bytes versus observer')
            evidence[sensor] = dict(count=len(observed), delivery_order_exact=True,
                source_time_inversions=sum(b['state_ns']<a['state_ns'] for a,b in zip(observed,observed[1:])),
                estimator_ingestion_verified=False)
    return evidence


def normalize_artificial(root, dataset, install, run_id):
    """Project the existing native exports after an explicit disturbance audit.

    The historical adapter remains untouched. Inputs may arrive out of sensing
    order; native output still must form a strictly ordered visual trajectory.
    Neither numerical publication nor IMU propagation establishes health.
    """
    import numpy as np
    from ..schema import TimestampEvidence
    from .openvins import pose_observation
    from .openvins_runtime import reconcile_outputs, wire_pose, world_velocity
    raw = root/'raw'
    delivery = audit_artificial_delivery(raw, dataset)
    observed = {name:lines(raw/(name+'.jsonl')) for name in ('pose','odom')}
    order = StreamOrder()
    for name, rows in observed.items():
        for row in rows:
            order.push(name, row['state_ns'])
    reconciled = reconcile_outputs(raw, observed)
    states = [state_row(line) for line in (raw/'state_estimate.txt').read_text().splitlines()
              if line.strip() and not line.startswith('#')]
    poses = observed['pose']
    require(len(states) == len(poses) > 0, 'POSE_COUNT_MISMATCH', 'native text and live adapter')
    require(associate([s['state_ns'] for s in states], [p['state_ns'] for p in poses], 6000) == list(range(len(poses))),
            'STATE_ASSOCIATION_MISMATCH', 'one-to-one native state timestamps')
    camera_ids = associate(dataset.camera_ns, [s['image_ns'] for s in states], 6000)
    records = []
    for i, (state, pose, index) in enumerate(zip(states, poses, camera_ids)):
        require(pose['frame_id'] == 'global', 'FRAME_INVALID', 'pose parent')
        require(np.max(abs(np.array(state['position'])-pose['position'])) <= 5.01e-7,
                'POSITION_MISMATCH', 'text versus observer')
        q = pose['quaternion_xyzw']
        require(np.max(abs(np.array(state['quaternion_wxyz'])-[q[3],*q[:3]])) <= 5.01e-7,
                'QUATERNION_MISMATCH', 'text versus observer')
        record = pose_observation(source_frame_id=dataset.source_ids[index], image_ns=dataset.camera_ns[index],
            state_ns=pose['state_ns'], recorder_ns=pose['observed_system_ns'], position=pose['position'],
            quaternion_xyzw=q, run_id=run_id, epoch=0, binary_sha256=install['binary_sha256'],
            raw_reference=dict(file='raw/pose.jsonl', topic='/poseimu', ordinal=i))
        record['observation_timestamp'] = TimestampEvidence(pose['observed_monotonic_ns'], 'ns',
            run_id+':observer_monotonic', 'ADAPTER_CALLBACK_RECEIPT', 'OBSERVED').to_dict()
        record['localization_fields'].update(velocity=state['velocity_world'], velocity_frame='backend_world', velocity_unit='m/s')
        record['raw_output'].update(velocity_source=f'raw/state_estimate.txt:row{i}', velocity_precision_decimal_places=6,
            initialization_evidence='CODE_SUPPORTED: native visualizer returns before publication when uninitialized',
            experiment_kind='ARTIFICIAL_DISTURBANCE', delivered_camera_ordinal=index,
            original_camera_id=dataset.source_ids[index], estimator_camera_ingestion_count=None)
        records.append(set_sequence_clock(record, dataset.sequence))
    (root/'online').mkdir()
    write_json(root/'online/trajectory.json', dict(schema_version=1, trajectory_variant='online', records=records,
        normalized_after_run=True, source='live observer plus matched native text; artificial input',
        contract='LocalizationEstimate v2 partial projections', contract_complete=False))
    propagated = []
    for pose in observed['odom']:
        require(pose['frame_id'] == 'global' and pose['child_frame_id'] == 'imu', 'FRAME_INVALID', 'odom convention')
        fields = wire_pose(pose['position'], pose['quaternion_xyzw'], pose['state_ns'])
        fields['source_time']['clock_domain'] = 'EuRoC:'+dataset.sequence+':sensor_time'
        fields.update(velocity=world_velocity(pose['quaternion_xyzw'], pose['velocity_local']),
                      velocity_frame='backend_world', velocity_unit='m/s')
        propagated.append(dict(localization_fields=fields, trajectory_variant='online', stream='imu_propagated',
            contract_complete=False, tracking_state='UNKNOWN', publish_timestamp=None, localization_epoch=0,
            observed_monotonic_ns=pose['observed_monotonic_ns']))
    write_json(root/'online/propagated.json', dict(trajectory_variant='online', records=propagated))
    evidence = dict(experiment_kind='ARTIFICIAL_DISTURBANCE', sequence=dataset.sequence, mode=dataset.mode,
        input_observation=delivery, output_transport_exact=reconciled, native_reset_count=None,
        estimator_camera_ingestion_count=None, tracking_loss_detection='NOT_RUN', recovery_status='NOT_RUN',
        validity_policy='UNKNOWN and invalid partial v2; no native health proof', final_optimized_trajectory=None,
        reference_archive_sha256=file_hash(dataset.reference_archive), derived_sensor_archive_sha256=file_hash(dataset.archive))
    write_json(root/'online/evidence.json', evidence)
    write_json(root/'observations.json', dict(initialization_proxy=dict(status='CODE_SUPPORTED_PROXY',
        first_visual_pose_camera_index=records[0]['source_frame_id'],
        first_visual_pose_image_ns=records[0]['source_timestamp']['raw_value'],
        sensor_time_from_first_image_s=(records[0]['source_timestamp']['raw_value']-dataset.original_camera_ns[0])/1e9),
        native_reset_count=None, tracking_loss_observations=[], tracking_loss_detection='NOT_RUN', recovery_status='NOT_RUN'))
    return records, evidence


def normalize_comparison(root, dataset, install, run_id):
    if getattr(dataset, 'artificial', False):
        return normalize_artificial(root, dataset, install, run_id)
    # The preserved adapter validates left pixels/IMU values and timestamps.
    # This independent entry also checks the input frame envelopes it records.
    for sensor in ['cam0', 'imu0']:
        observed = lines(root/'raw'/(sensor+'.jsonl'))
        require(all(r.get('frame_id') == sensor for r in observed),
                'INPUT_FRAME_INVALID', sensor+' observer frame identity')
        if sensor == 'cam0':
            require(all(r.get('is_bigendian') == 0 for r in observed),
                    'CAMERA_ENCODING_INVALID', 'little-endian mono8 envelope required')
    right = verify_second_camera(root/'raw', dataset)
    records, evidence = normalize(root, dataset, install, run_id)
    for record in records:
        set_sequence_clock(record, dataset.sequence)
    trajectory = read_json(root/'online/trajectory.json')
    trajectory['records'] = records
    write_json(root/'online/trajectory.json', trajectory)
    propagated = read_json(root/'online/propagated.json')
    for record in propagated['records']:
        record['localization_fields']['source_time']['clock_domain'] = 'EuRoC:' + dataset.sequence + ':sensor_time'
    write_json(root/'online/propagated.json', propagated)
    evidence.update(sequence=dataset.sequence, mode=dataset.mode, right_camera=right,
                    historical_v101_mono_acceptance='not evaluated by this independent comparison',
                    native_reset_count=None, estimator_camera_ingestion_count=None)
    write_json(root/'online/evidence.json', evidence)
    write_json(root/'observations.json', dict(
        initialization_proxy=dict(status='CODE_SUPPORTED_PROXY',
            first_visual_pose_camera_index=records[0]['source_frame_id'],
            first_visual_pose_image_ns=records[0]['source_timestamp']['raw_value'],
            sensor_time_from_first_image_s=(records[0]['source_timestamp']['raw_value']-dataset.camera_ns[0])/1e9,
            scope='first published visual pose after native initialized gate; no instrumented transition timestamp'),
        native_reset_count=None, tracking_loss_observations=[], tracking_loss_detection='NOT_RUN',
        native_tail_seen=None, camera_lag_at_info_receipt=None))
    return records, evidence


def evaluate_comparison(root, dataset, records):
    reference = read_reference(getattr(dataset, 'reference_archive', dataset.archive))
    poses = lines(root/'raw/pose.jsonl')
    states = [state_row(line) for line in (root/'raw/state_estimate.txt').read_text().splitlines()
              if line and not line.startswith('#')]
    camera_ids = associate(dataset.camera_ns, [r['image_ns'] for r in states], 6000)
    require(len(poses) == len(states), 'RAW_STATE_COUNT_MISMATCH', 'raw poses versus native text states')
    # The existing adapter preserves the exact source camera stamp after
    # validating native-text association. Report its rounding residual separately.
    source_ids = getattr(dataset, 'source_ids', list(range(len(dataset.camera_ns))))
    raw = [dict(state_ns=p['state_ns'], image_ns=dataset.camera_ns[index], position=p['position'],
                quaternion_xyzw=p['quaternion_xyzw'], source_frame_id=source_ids[index], epoch=0)
           for p, index in zip(poses, camera_ids)]
    raw_score = evaluate_rows(raw, reference)
    score = evaluate_rows(records, reference)
    require(score == raw_score, 'ADAPTER_METRICS_DIFFER', 'same-run raw ROS pose versus OnlinePoseRecord')
    score.update(same_run_metrics_exact=True, raw_pose_source='raw/pose.jsonl',
                 projection_source='online/trajectory.json', source_sha256=file_hash(root/'online/trajectory.json'),
                 camera_association=dict(tolerance_ns=6000,
                     max_native_text_residual_ns=max(abs(dataset.camera_ns[i]-s['image_ns'])
                                                    for i, s in zip(camera_ids, states)),
                     policy='original camera ID and source timestamp; native-text residual retained separately'))
    if getattr(dataset, 'artificial', False):
        original = dataset.original_camera_ns
        score.update(experiment_kind='ARTIFICIAL_DISTURBANCE',
            original_reference_archive_sha256=file_hash(dataset.reference_archive),
            derived_sensor_archive_sha256=file_hash(dataset.archive),
            full_source_coverage=dict(original_camera_count=len(original), delivered_camera_count=len(dataset.camera_ns),
                trajectory_camera_count=len(records), original_first_ns=original[0], original_last_ns=original[-1],
                trajectory_fraction_of_original=len(records)/len(original)))
    write_json(root/'evaluation.json', score)
    write_json(root/'native-evaluation.json', raw_score)
    require(score['scored_epoch_count'] > 0, 'TRAJECTORY_NOT_EVALUABLE', 'no scored visual trajectory')
    return score


def native_observation_spec(install):
    """Only an explicitly frozen observation build can enable native logging."""
    if 'native_observation' not in install:
        return None
    from tools.openvins.native_observations import MAX_BYTES, MAX_EVENTS
    spec = install['native_observation']
    require(isinstance(spec, dict) and type(spec.get('schema_version')) is int
            and spec['schema_version'] == 1
            and spec.get('environment_variable') == 'RM27_OPENVINS_NATIVE_EVENTS'
            and type(spec.get('max_events')) is int and spec['max_events'] == MAX_EVENTS
            and type(spec.get('max_bytes')) is int and spec['max_bytes'] == MAX_BYTES,
            'OPENVINS_OBSERVATION_CONFIG_INVALID', 'unsupported native observation schema or bounds')
    return spec


def state_health_spec(install, configuration):
    """Opt-in observability/freshness, with operational UNKNOWN preserved."""
    if configuration is None:
        return None
    from ..state_output import StateOutputPolicy
    expected = dict(schema_version=2, success_kind='imu_fast_prediction_completed',
        unavailable_kind='imu_fast_prediction_unavailable',
        success_values=['integration_start_seconds', 'integration_end_seconds',
            'main_filter_camera_state_seconds', 'cache_state_time_seconds', 'cache_camera_imu_offset_seconds',
            'mean_all_finite', 'covariance_all_finite'],
        unavailable_values=['integration_start_seconds', 'integration_end_seconds', 'main_filter_camera_state_seconds',
            'cache_state_time_seconds', 'cache_camera_imu_offset_seconds'],
        sensor_time_semantic='REQUESTED_IMU_CALLBACK_DOUBLE', tracking_health='UNKNOWN',
        native_reset='UNSUPPORTED', covariance='FINITE_WITNESS_ONLY_UNQUALIFIED')
    require(native_observation_spec(install) is not None
            and install.get('native_health_observation') == expected
            and type(install['native_health_observation']['schema_version']) is int
            and isinstance(configuration, dict) and set(configuration) == {'schema_version', 'policy'}
            and type(configuration['schema_version']) is int and configuration['schema_version'] == 1
            and isinstance(configuration['policy'], dict)
            and set(configuration['policy']) == set(StateOutputPolicy.__dataclass_fields__),
            'OPENVINS_HEALTH_CONFIG_INVALID', 'explicit native witness capability and freshness policy required')
    StateOutputPolicy(**configuration['policy'])
    return configuration



def tracking_quality_spec(install, configuration):
    """Default-off native diagnostic capability; never tracking qualification."""
    if configuration is None:
        return None
    from tools.openvins.native_observations import QUALITY_MAX_BYTES, QUALITY_MAX_ROWS
    spec = install.get('native_quality_observation')
    required = {'schema_version','environment_variable','default_enabled','max_bytes','max_rows',
                'max_gate_entries_per_batch','schema','schema_sha256','tracking_health','covariance',
                'native_reset','selected_by_runner','native_replay'}
    require(isinstance(configuration,dict) and set(configuration)=={'schema_version'}
            and type(configuration['schema_version']) is int and configuration['schema_version']==1
            and isinstance(spec,dict) and set(spec)==required
            and type(spec['schema_version']) is int and spec['schema_version']==1
            and spec['environment_variable']=='RM27_OPENVINS_NATIVE_QUALITY'
            and spec['default_enabled'] is False and type(spec['max_bytes']) is int
            and spec['max_bytes']==QUALITY_MAX_BYTES and type(spec['max_rows']) is int
            and spec['max_rows']==QUALITY_MAX_ROWS and type(spec['max_gate_entries_per_batch']) is int
            and spec['max_gate_entries_per_batch']==2048
            and spec['tracking_health']=='UNQUALIFIED; accepted gate support is not overall tracking health'
            and spec['covariance']=='NATIVE_15X15_ERROR_CHART_UNQUALIFIED'
            and spec['native_reset']=='UNSUPPORTED' and spec['selected_by_runner'] is False
            and spec['native_replay']=='NOT_RUN' and native_observation_spec(install) is not None,
            'OPENVINS_QUALITY_CONFIG_INVALID','explicit default-off native quality capability required')
    checked_file(spec['schema'],spec['schema_sha256'])
    return spec


def camera_observation_spec(install):
    """Enable only the declared raw-pixel observer, never inferred health."""
    if 'camera_consumption_observation' not in install:
        return None
    spec = install['camera_consumption_observation']
    fields = {'status', 'environment_variable', 'payload_key', 'schema_version', 'pending_receipt_limit',
              'hash', 'selected_by_runner', 'native_replay', 'adapter_integration', 'tracking_health',
              'raw_subscription_before_stereo_synchronizer', 'stereo_synchronizer', 'pixel_digest',
              'shutdown', 'ownership_fix'}
    require(native_observation_spec(install) is not None and isinstance(spec, dict)
            and set(spec) == fields
            and type(spec.get('schema_version')) is int and spec['schema_version'] == 1
            and spec.get('environment_variable') == 'RM27_OPENVINS_NATIVE_EVENTS'
            and spec.get('payload_key') == 'camera_consumption'
            and type(spec.get('pending_receipt_limit')) is int and spec['pending_receipt_limit'] == 10000
            and spec.get('hash') == 'OpenSSL EVP SHA-256'
            and spec.get('raw_subscription_before_stereo_synchronizer') is True
            and spec.get('stereo_synchronizer') == 'upstream ApproximateTime(10), unchanged'
            and spec.get('pixel_digest') == 'decoded CV_8UC1 row bytes only; excludes row padding; feed and frontend boundaries independent'
            and spec.get('shutdown') in (
                'executor returned, camera worker joined, unresolved subscriptions/queue exposed before shared footer',
                'executor returned; all three owned worker levels joined; then queue/receipt footer; independent of observation enabled')
            and spec.get('ownership_fix') == 'callback ImuData captured by value, estimator input values unchanged'
            and spec.get('tracking_health') == 'UNKNOWN'
            and spec.get('status') == 'BUILD_AND_HELPER_VERIFIED_NATIVE_REPLAY_NOT_RUN'
            and spec.get('selected_by_runner') is False
            and spec.get('native_replay') == 'NOT_RUN'
            and spec.get('adapter_integration') == 'NOT_RUN',
            'OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID', 'unsupported native camera capability or stage claims')
    return spec


def input_publisher_match_spec(install):
    """Select a qualified optional native-reader snapshot, with frozen receipts."""
    if 'input_publisher_match_observation' not in install:
        return None
    from tools.openvins.native_observations import INPUT_MATCH_MAX_BYTES
    spec = install['input_publisher_match_observation']
    expected = dict(schema_version=1, environment_variable='RM27_OPENVINS_INPUT_MATCHES',
        path_semantics='fresh exclusive atomic final JSON; sibling .pending reserved exclusively; one bounded attempt',
        max_bytes=INPUT_MATCH_MAX_BYTES,
        api='actual owned rclcpp subscription get_publisher_count -> rcl -> rmw matched publishers',
        handles='sub_imu; mono subs_cam; stereo sync_subs_cam.getSubscriber; mode and camera IDs explicit',
        timer='existing20ms independent observation timer; weak visualizer ownership; executor quiesces before destruction',
        scope='reader matched publishers at snapshot only; not writer completeness, input ingestion, tracking health or old failure cause',
        status='BUILD_AND_HELPER_VERIFIED_NATIVE_REPLAY_NOT_RUN', selected_by_runner=False,
        native_replay='NOT_RUN', old_failure_root_cause='UNKNOWN')
    receipts = {'helper_receipt', 'helper_receipt_sha256', 'closure_receipt', 'closure_receipt_sha256'}
    require(camera_observation_spec(install) is not None and isinstance(spec, dict)
            and set(spec) == set(expected) | receipts
            and all(type(spec[key]) is type(value) and spec[key] == value for key, value in expected.items()),
            'OPENVINS_INPUT_MATCH_CONFIG_INVALID', 'unsupported or unqualified native reader-match capability')
    for name in ('helper', 'closure'):
        require(isinstance(spec[name+'_receipt'], str) and bool(spec[name+'_receipt']),
                'OPENVINS_INPUT_MATCH_CONFIG_INVALID', 'explicit qualification receipt required')
        checked_file(spec[name+'_receipt'], spec[name+'_receipt_sha256'])
    return spec


def camera_observation_configuration(install):
    """Resolve the actual locked OpenCV YAML used by the fixed launch command."""
    import cv2
    script = Path(__file__).resolve().parents[5]/'tools/openvins/run_comparison_ros2.sh'
    tokens = shlex.split(script.read_text(), comments=True)
    require([value for value in tokens if value.startswith('config_path:=')]
            == ['config_path:=/upstream/config/euroc_mav/estimator_config.yaml']
            and not any('downsample_cameras' in value or value == '--params-file' for value in tokens),
            'OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID', 'fixed launcher configuration changed or overrides downsampling')
    path = Path(install['source']).resolve()/'config/euroc_mav/estimator_config.yaml'
    wanted = install['files_sha256'].get(str(path))
    require(isinstance(wanted, str), 'OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID',
            'actual estimator configuration must belong to locked installation files')
    checked_file(path, wanted)
    try:
        storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        require(storage.isOpened(), 'OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID', 'estimator OpenCV YAML did not open')
        try:
            node = storage.getNode('downsample_cameras')
            require(storage.root().keys().count('downsample_cameras') == 1 and not node.empty(),
                    'OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID', 'one explicit downsample_cameras node required')
            if node.isInt() and node.real() in (0, 1):
                value = bool(node.real())
            else:
                # Match the pinned C++ bool parser: OpenCV retains inline comments
                # as part of unquoted strings; do not guess a missing/default value.
                text = node.string().split('#', 1)[0].split(' ', 1)[0] if node.isString() else ''
                require(text in ('0', '1', 'false', 'False', 'FALSE', 'true', 'True', 'TRUE'),
                        'OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID', 'explicit OpenCV YAML bool or integer 0/1 required')
                value = text in ('1', 'true', 'True', 'TRUE')
        finally:
            storage.release()
    except (cv2.error, SystemError) as exc:
        require(False, 'OPENVINS_CAMERA_OBSERVATION_CONFIG_INVALID', str(exc))
    checked_file(path, wanted)
    return dict(downsample_cameras=value, estimator_config=str(path), estimator_config_sha256=wanted,
                launcher=str(script), launcher_sha256=file_hash(script), parameter_override=False,
                source='locked config/euroc_mav/estimator_config.yaml; pinned OpenCV bool semantics')


def observe_camera_consumption(root, dataset, configuration):
    """Keep raw-input evidence failures separate from numerical trajectory scores."""
    from tools.openvins.native_observations import analyze_camera_consumption, build_camera_reference
    import cv2
    try:
        reference = build_camera_reference(dataset, root/'raw',
            downsample_cameras=configuration['downsample_cameras'], sample_limit=None)
        native = analyze_camera_consumption(root/'raw/native-events.jsonl',
            camera_ids=reference['camera_ids'], downsample_cameras=configuration['downsample_cameras'],
            reference=reference)
        # Keep a compact audit envelope; occurrence-level evidence is in the native
        # summary and the complete reference remains reproducible from sealed inputs.
        native['input_reference'] = {key:value for key,value in reference.items() if key != 'records'}
    except (ValueError, KeyError, OSError, sqlite3.Error, zipfile.BadZipFile, cv2.error) as exc:
        native = dict(schema_version=1, status='FAIL', observation_integrity='FAIL', observation_complete=False,
            input_coverage=dict(status='FAIL'), full_source_consumption='FAIL',
            failure_code=getattr(exc, 'code', 'OPENVINS_CAMERA_OBSERVATION_ERROR'), failure_reason=str(exc),
            health_qualification='NOT_RUN', automatic_reset='NOT_RUN', truth_used=False)
    native['configuration'] = configuration
    write_json(root/'native-camera-observations.json', native)
    passed = (native['observation_integrity'] == 'PASS' and native['input_coverage']['status'] == 'PASS'
              and native['full_source_consumption'] == 'PASS')
    return dict(path='native-camera-observations.json', status='PASS' if passed else 'FAIL',
        observation_integrity=native['observation_integrity'], input_coverage=native['input_coverage'],
        full_source_consumption=native['full_source_consumption'],
        configuration=configuration, scope='raw source/CDR/observer/subscription/decoded/feed/frontend boundaries only',
        numerical_trajectory_status='INDEPENDENT', ekf_acceptance='NOT_INFERRED', health_qualification='NOT_RUN')


def command_for(root, dataset, install, config, container):
    repo = Path(__file__).resolve().parents[5]
    camera_observation_spec(install)
    native_env = (['-e', 'RM27_OPENVINS_NATIVE_EVENTS=/out/native-events.jsonl']
                  if native_observation_spec(install) is not None else [])
    if tracking_quality_spec(install, config.get('native_quality')) is not None:
        native_env += ['-e', 'RM27_OPENVINS_NATIVE_QUALITY=/out/native-quality.jsonl']
    if input_publisher_match_spec(install) is not None:
        native_env += ['-e', 'RM27_OPENVINS_INPUT_MATCHES=/out/native-inputs-matched.json']
    return ['docker', 'run', '--rm', '--name', container, '--network', 'none', '--cpus', '4', '--memory', '12g',
            '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'VIO_MODE=' + dataset.mode,
            '-e', 'VIO_RATE=' + str(float(config['rate'])), *native_env,
            '-v', f'{repo}:/repo:ro', '-v', f'{install["source"]}:/upstream:ro',
            '-v', f'{install["workspace"]}:/ws:ro', '-v', f'{dataset.bag}:/data:ro',
            '-v', f'{root/"raw"}:/out', install['image'], 'bash', '/repo/tools/openvins/run_comparison_ros2.sh']


def run(config):
    result = dict(status='REFUSED', execution_started=False, stage_result='PARTIAL', config=config)
    root = None
    try:
        keys = {'schema_version', 'purpose', 'run_id', 'backend', 'archive', 'archive_sha256', 'bag',
                'installation', 'output', 'sequence', 'mode', 'rate'}
        artificial = config.get('purpose') == 'EUROC_ARTIFICIAL_COMPARISON'
        if artificial:
            keys = keys-{'archive','archive_sha256'} | {'original_reference_archive','original_reference_archive_sha256',
                'derived_sensor_archive','derived_sensor_archive_sha256','prepared_input'}
        if 'state_health' in config:
            keys |= {'state_health'}
        if 'native_quality' in config:
            keys |= {'native_quality'}
        require(set(config) == keys and config['schema_version'] == 1 and config['backend'] == 'OpenVINS'
                and config['purpose'] in ('EUROC_PUBLIC_COMPARISON','EUROC_ARTIFICIAL_COMPARISON'),
                'COMPARISON_CONFIG_INVALID', 'explicit comparison v1')
        require(config['mode'] in ('mono', 'stereo') and type(config['rate']) in (int, float)
                and config['rate'] in (.5, 1.), 'COMPARISON_CONFIG_INVALID', 'mono/stereo and rate .5/1 required')
        if artificial:
            dataset = ComparisonDataset(config['derived_sensor_archive'], config['bag'], config['sequence'], config['mode'],
                config['derived_sensor_archive_sha256'], original_reference_archive=config['original_reference_archive'],
                original_reference_sha256=config['original_reference_archive_sha256'], prepared_input=config['prepared_input'], rate=config['rate'])
        else:
            dataset = ComparisonDataset(config['archive'], config['bag'], config['sequence'], config['mode'], config['archive_sha256'])
        install = read_json(config['installation'])
        native_spec = native_observation_spec(install)
        camera_spec = camera_observation_spec(install)
        match_spec = input_publisher_match_spec(install)
        health_spec = state_health_spec(install, config.get('state_health'))
        quality_spec = tracking_quality_spec(install, config.get('native_quality'))
        require(quality_spec is None or health_spec is not None, 'OPENVINS_QUALITY_CONFIG_INVALID',
                'quality diagnostics require original native freshness evidence')
        require(install['commit'] == PIN, 'BACKEND_VERSION_UNSUPPORTED', PIN)
        require(install['image'].startswith('sha256:') and len(install['image']) == 71,
                'IMAGE_UNPINNED', 'immutable image required')
        for path, digest in install['files_sha256'].items():
            checked_file(path, digest)
        binary = Path(install['workspace'])/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
        checked_file(binary, install['binary_sha256'])
        camera_configuration = camera_observation_configuration(install) if camera_spec is not None else None
        output = Path(config['output']).resolve()
        protected_paths = [dataset.bag, Path(install['workspace']).resolve(), Path(install['source']).resolve()]
        if artificial:
            protected_paths.append(dataset.prepared_input)
        for protected in protected_paths:
            require(output != protected and protected not in output.parents,
                    'OUTPUT_INVALID', 'output must be outside frozen bag/source/workspace')
        root = create_output(config['output'])
        container = 'rm27-openvins-comparison-' + uuid.uuid4().hex
        command = command_for(root, dataset, install, config, container)
        repo = Path(__file__).resolve().parents[5]
        sources = [Path(__file__), repo/'tools/openvins/run_comparison_ros2.sh',
                   repo/'tools/openvins/observe_comparison_ros2.py', repo/'tools/openvins/comparison_transport.py',
                   repo/'tools/openvins/comparison_qos.yaml',
                   repo/'tools/openvins/sample_comparison_resources.py',
                   repo/'tools/openvins/asl_transport.py', repo/'tools/schurvins/ros_transport.py',
                   repo/'tools/schurvins/evaluate.py', repo/'tools/openvins/native_accuracy.py',
                   Path(__file__).with_name('openvins.py'), Path(__file__).with_name('online.py'),
                   Path(__file__).with_name('trajectory.py'), Path(__file__).parent.parent/'schema.py',
                   Path(__file__).with_name('openvins_runtime.py'), Path(__file__).with_name('common.py')]
        if native_spec is not None:
            sources.append(repo/'tools/openvins/native_observations.py')
        if health_spec is not None:
            sources.append(Path(__file__).parent.parent/'state_output.py')
        if quality_spec is not None:
            sources.append(Path(quality_spec['schema']))
        hashes = {str(p): file_hash(p) for p in sources}
        result.update(status='EXECUTING', execution_started=True, command=command, installation=install,
                      dataset=dataset.provenance, runtime_sources_sha256=hashes, final_optimized_trajectory=None,
                      experiment_kind='ARTIFICIAL_DISTURBANCE' if artificial else 'ORIGINAL_PUBLIC_DATA',
                      reference_archive_sha256=(config['original_reference_archive_sha256'] if artificial else config['archive_sha256']),
                      sensor_archive_sha256=(config['derived_sensor_archive_sha256'] if artificial else config['archive_sha256']),
                      transport_protocol=('explicit-reliable-publisher-observer-and-native-reader-paused-match-v2'
                          if match_spec is not None else 'explicit-reliable-publisher-and-observer-paused-match-v1'),
                      qualification='independent public-data comparison; original VIO-P acceptance unchanged')
        if camera_configuration is not None:
            result['native_camera_observation_configuration'] = camera_configuration
        write_json(root/'run.json', result)
        with (root/'docker.log').open('w') as log:
            try:
                completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=900)
            except BaseException as exc:
                result.update(status='FAILED', failure_code='EXECUTION_INTERRUPTED',
                              failure_reason=type(exc).__name__)
                raise
            finally:
                result['container_cleanup'] = cleanup_container(container, log)
        result['exit_code'] = completed.returncode
        require(completed.returncode == 0, 'BACKEND_EXECUTION_FAILED', 'retained docker/raw logs')
        require(result['container_cleanup']['absence_verified'], 'CONTAINER_CLEANUP_UNVERIFIED',
                'owned replay container absence not verified; outputs may still change')
        if match_spec is not None:
            from tools.openvins.native_observations import read_input_publisher_matches
            pid_text = (root/'raw/native-process-id.txt').read_text()
            require(pid_text.endswith('\n') and pid_text[:-1].isascii() and pid_text[:-1].isdigit(),
                    'OPENVINS_INPUT_MATCH_INVALID', 'actual container child PID record required')
            matched = read_input_publisher_matches(root/'raw/native-inputs-matched.json',
                mode=dataset.mode, native_pid=int(pid_text))
            result['native_input_match_evidence'] = dict(path='raw/native-inputs-matched.json',
                native_pid_path='raw/native-process-id.txt', snapshot=matched,
                status='MATCH_SNAPSHOT_VERIFIED', full_input_delivery='INDEPENDENT_AUDIT_REQUIRED',
                tracking_health='UNKNOWN', old_failure_root_cause='UNKNOWN')
        if camera_configuration is not None:
            result['camera_consumption_evidence_gate'] = observe_camera_consumption(root, dataset, camera_configuration)
        records, evidence = normalize_comparison(root, dataset, install, config['run_id'])
        evaluate_comparison(root, dataset, records)
        if native_spec is not None:
            from tools.openvins.native_observations import analyze_openvins_observations
            require((root/'raw/native-events.jsonl').is_file(), 'OPENVINS_OBSERVATION_MISSING',
                    'explicit native observation build did not produce its evidence file')
            native = analyze_openvins_observations(root/'raw/native-events.jsonl', input_directory=root/'raw',
                include_camera_transactions=camera_configuration is not None)
            write_json(root/'native-observations.json', native)
            result['native_observations'] = dict(path='native-observations.json',
                status=native['status'], observation_integrity=native['observation_integrity'],
                accepted_visual_update_count=native['accepted_visual_update_count'],
                native_imu_callback_coverage=native['native_imu_callback_coverage'],
                health_qualification='NOT_RUN', automatic_reset='NOT_RUN')
            if camera_configuration is not None:
                transactions = native['camera_transactions']
                evidence['native_camera_transactions'] = dict(path='native-observations.json',
                    section='camera_transactions', status=transactions['status'],
                    transaction_count=transactions['transaction_count'],
                    qualification='offline numerical/time metadata only; tracking UNKNOWN/invalid, no reset qualification')
                write_json(root/'online/evidence.json', evidence)
            require(native['native_imu_callback_coverage']['status'] == 'PASS',
                    'OPENVINS_NATIVE_INPUT_INCOMPLETE', 'actual native IMU callbacks differ from reconciled input')
            if health_spec is not None:
                from tools.openvins.native_observations import qualify_state_freshness
                health = qualify_state_freshness(root/'raw/native-events.jsonl', root/'online/native-health.jsonl',
                    policy=health_spec['policy'], session_id=config['run_id'],
                    sensor_clock='EuRoC:'+dataset.sequence+':sensor_time')
                write_json(root/'native-health.json', health)
                result['native_state_health'] = dict(path='native-health.json', status=health['status'],
                    scope=health['scope'], operational_health='UNKNOWN', operational_valid=False,
                    native_reset=health['native_reset'])
            if quality_spec is not None:
                from tools.openvins.native_observations import qualify_openvins_tracking_health
                quality = qualify_openvins_tracking_health(root/'raw/native-quality.jsonl', root/'raw/native-events.jsonl',
                    root/'online/native-health.jsonl', health, root/'online/measurement-health.jsonl', session_id=config['run_id'])
                write_json(root/'native-quality.json', quality)
                result['native_tracking_quality'] = dict(path='native-quality.json', status=quality['status'],
                    scope=quality['scope'], operational_tracking_health='UNKNOWN', output_eligible=False,
                    timeline='online/measurement-health.jsonl', covariance_consistency='UNQUALIFIED')
        for path, digest in {**install['files_sha256'], **hashes}.items():
            checked_file(path, digest)
        checked_file(binary, install['binary_sha256'])
        checked_file(dataset.archive, config['derived_sensor_archive_sha256'] if artificial else config['archive_sha256'])
        if artificial:
            checked_file(dataset.reference_archive, config['original_reference_archive_sha256'])
            checked_file(dataset.prepared_input/'input.json', dataset.provenance['prepared_input_sha256'])
            artificial_input(config['prepared_input'], dataset.reference_archive, dataset.mode)
        dataset.verify_bag()
        result.update(status='EXECUTION_COMPLETED', stage_result='COMPARISON_RUNTIME_AND_METRIC_PASS',
                      online_trajectory='online/trajectory.json', evaluation='evaluation.json', evidence=evidence)
    except (ValueError, KeyError, OSError, sqlite3.Error, subprocess.SubprocessError) as exc:
        if 'failure_code' not in result:
            result.update(status='FAILED' if result['execution_started'] else 'REFUSED',
                          failure_code=getattr(exc, 'code', 'COMPARISON_RUNTIME_ERROR'), failure_reason=str(exc))
    finally:
        if root is not None:
            if result.get('container_cleanup', {}).get('absence_verified') is not True:
                result['artifact_finalization'] = 'NOT_FROZEN_CONTAINER_CLEANUP_UNVERIFIED'
            else:
                result['output_sha256'] = {str(p.relative_to(root)): file_hash(p) for p in root.rglob('*')
                                          if p.is_file() and p.name != 'run.json'}
            write_json(root/'run.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    report = run(read_json(args.config))
    print(json.dumps({k: report.get(k) for k in ['status', 'stage_result', 'failure_code', 'failure_reason']}, indent=2))
    raise SystemExit(0 if report['status'] == 'EXECUTION_COMPLETED' else 1)
