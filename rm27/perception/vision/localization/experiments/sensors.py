"""Platform-independent, sensor-only recording, validation and timed replay.

A recording contains config.json, events.jsonl, referenced mono8 images and a
recording.json hash manifest. Raw TimestampEvidence, source sequence IDs and
clock identities are never rewritten by replay or fault injection. A separate
mapped_timestamp requires a configured, evidenced affine clock mapping. Unknown
sensor/exposure/calibration metadata is allowed for audit, not promoted to a
qualified VIO input. No hardware device is opened by this module.

Public entry points: validate_config, validate_events, load_recording,
write_recording, replay, derive_recording, mock_recording and CameraSelection.
The CLI supplies validate/replay/derive/mock. Fault profiles use one kind,
explicit sensor_ids, seed, and a half-open elapsed-nanosecond window. Invalid
IMU fault recordings are retained as strict JSON and rejected before replay;
IEEE nonfinite injection is represented by the explicit string "NaN", never
silently converted into a finite sample or nonstandard JSON.
"""
from __future__ import annotations

import argparse
import copy
from decimal import Decimal
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import random
import shutil
import time
import zipfile

import cv2
import numpy as np

from ..schema import TimestampEvidence
from .common import canonical, file_hash, number, read_json, require, write_json
from .vio import validate_imu_sample

_UNIT_NS = {'ps': Fraction(1, 1000), 'ns': 1, 'us': 1000, 'ms': 1000000, 's': 1000000000}
_SEQUENCE = {'SOURCE_SAMPLE_COUNTER', 'RECORD_COUNTER', 'UNKNOWN'}
_STATUS = {'UNKNOWN', 'CODE_SUPPORTED', 'OBSERVED', 'VERIFIED'}
_FAULTS = {'camera_drop', 'camera_gap', 'camera_occlusion', 'camera_blur',
           'camera_dim', 'camera_texture', 'delivery_delay', 'imu_gap',
           'imu_duplicate', 'imu_reorder', 'imu_nonfinite', 'imu_bias'}


def _identity(value, name):
    require(isinstance(value, str) and value not in ('', 'UNKNOWN'),
            'SENSOR_IDENTITY_INVALID', name)
    return value


def _sha(value):
    return (isinstance(value, str) and len(value) == 64
            and all(c in '0123456789abcdef' for c in value))


def _integer(value, name, minimum=0):
    require(type(value) is int and value >= minimum, 'SENSOR_INTEGER_INVALID', name)
    return value


def _stamp_contract(value):
    require(isinstance(value, dict) and set(value) ==
            {'unit', 'clock_domain', 'semantic', 'evidence_status'},
            'SENSOR_CLOCK_INVALID', 'timestamp_contract fields')
    TimestampEvidence(0, **value)
    require(value['evidence_status'] in _STATUS, 'SENSOR_CLOCK_INVALID', 'evidence status')


def _evidence(value, name):
    require(isinstance(value, dict) and value.get('status') in _STATUS,
            'SENSOR_CALIBRATION_INVALID', name)
    if value['status'] != 'UNKNOWN':
        require(_sha(value.get('source_sha256')) and bool(value.get('source_reference')),
                'SENSOR_CALIBRATION_INVALID', name + ' requires source identity')
    return value['status']


def _rigid(value):
    require(isinstance(value, list) and len(value) == 4
            and all(isinstance(row, list) and len(row) == 4 for row in value),
            'SENSOR_CALIBRATION_INVALID', '4x4 numeric rigid transform required')
    for row in value:
        for item in row:
            number(item, 'T_imu_camera element')
    a = np.asarray(value, dtype=float)
    require(np.isfinite(a).all()
            and np.allclose(a[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            and np.allclose(a[:3, :3].T @ a[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            and abs(np.linalg.det(a[:3, :3])-1) < 1e-6,
            'SENSOR_CALIBRATION_INVALID', 'rigid T_imu_camera required')


def validate_config(config):
    """Validate identities and claims without certifying unknown hardware facts."""
    require(isinstance(config, dict) and type(config.get('schema_version')) is int
            and config['schema_version'] == 1, 'SENSOR_CONFIG_INVALID', 'schema version 1')
    require(set(config) <= {'schema_version', 'config_version', 'provenance', 'imu', 'cameras',
            'selected_camera_ids', 'mode', 'stereo', 'clock_mappings'},
            'SENSOR_PROVENANCE_INVALID', 'only sensor configuration belongs in the recording')
    _identity(config.get('config_version'), 'config_version')
    provenance = config.get('provenance', {})
    require(provenance.get('kind') in ('PUBLIC_DATASET', 'HARDWARE_RECORDING', 'MOCK')
            and provenance.get('truth_included') is False,
            'SENSOR_PROVENANCE_INVALID', 'explicit sensor-only provenance required')
    _identity(provenance.get('source_reference'), 'source_reference')
    require(_sha(provenance.get('source_sha256')), 'SENSOR_PROVENANCE_INVALID', 'source SHA256')
    imu = config.get('imu', {})
    cameras = config.get('cameras', [])
    require(isinstance(imu, dict) and isinstance(cameras, list) and cameras,
            'SENSOR_CONFIG_INVALID', 'one IMU and camera definitions required')
    ids = []
    for sensor in [imu, *cameras]:
        require(isinstance(sensor, dict), 'SENSOR_CONFIG_INVALID', 'sensor object')
        ids.append(_identity(sensor.get('sensor_id'), 'sensor_id'))
        _identity(sensor.get('frame_id'), 'sensor frame_id')
        _stamp_contract(sensor.get('timestamp_contract'))
        require(sensor.get('sequence_semantics') in _SEQUENCE,
                'SENSOR_CONFIG_INVALID', 'sequence semantics must be explicit')
        if 'sequence_evidence' in sensor:
            _evidence(sensor['sequence_evidence'], 'sequence semantics')
        axes = sensor.get('axes')
        require(isinstance(axes, dict) and axes.get('handedness') in ('RIGHT', 'UNKNOWN')
                and isinstance(axes.get('description'), str) and bool(axes['description']),
                'SENSOR_AXES_INVALID', 'axis description and handedness required')
    require(len(set(ids)) == len(ids), 'SENSOR_IDENTITY_INVALID', 'duplicate sensor IDs')
    require(imu.get('measurement_kind') == 'RAW_IMU' and imu.get('gyro_unit') == 'rad/s'
            and imu.get('accel_unit') == 'm/s^2', 'IMU_UNITS_INVALID', 'raw SI IMU required')
    for camera in cameras:
        require(camera.get('role') in ('FRONT', 'DOWN', 'LEFT', 'RIGHT', 'UNKNOWN'),
                'SENSOR_CONFIG_INVALID', 'camera role')
        cal = camera.get('calibration', {})
        _identity(cal.get('version'), 'calibration version')
        for key in ('intrinsics', 'T_imu_camera', 'camera_imu_offset'):
            status = _evidence(cal.get(key), key)
            item = cal[key]
            if status != 'UNKNOWN' and key == 'intrinsics':
                require(item.get('model') in ('pinhole-radtan', 'pinhole-equidistant'),
                        'SENSOR_CALIBRATION_INVALID', 'explicit supported camera model')
                params = item.get('parameters', [])
                require(isinstance(params, list) and len(params) >= 4,
                        'SENSOR_CALIBRATION_INVALID', 'camera parameters')
                for x in params:
                    number(x, 'camera parameter')
                require(params[0] > 0 and params[1] > 0, 'SENSOR_CALIBRATION_INVALID', 'focal length')
            elif status != 'UNKNOWN' and key == 'T_imu_camera':
                require(item.get('convention') == 'camera_to_imu',
                        'SENSOR_CALIBRATION_INVALID', 'T_imu_camera direction')
                _rigid(item.get('matrix'))
            elif status != 'UNKNOWN' and key == 'camera_imu_offset':
                require(type(item.get('value_ns')) is int and item.get('convention') ==
                        'imu_time_equals_mapped_camera_time_plus_offset',
                        'SENSOR_CALIBRATION_INVALID', 'time offset sign and integer ns')
    selected = config.get('selected_camera_ids')
    camera_ids = {s['sensor_id'] for s in cameras}
    require(isinstance(selected, list) and selected and len(selected) == len(set(selected))
            and set(selected) <= camera_ids, 'SENSOR_CONFIG_INVALID', 'selected cameras')
    mode = config.get('mode')
    require(mode in ('mono', 'stereo'), 'CAMERA_MODE_UNSUPPORTED', 'mono or verified stereo only')
    if mode == 'mono':
        require(len(selected) == 1, 'CAMERA_MODE_UNSUPPORTED', 'independent mono selects one camera')
    else:
        sync = config.get('stereo', {})
        require(len(selected) == 2 and sync.get('overlapping_view') is True
                and sync.get('synchronization') == 'EXACT_SOURCE_TIME'
                and sync.get('evidence_status') in ('CODE_SUPPORTED', 'VERIFIED')
                and _sha(sync.get('source_sha256')),
                'CAMERA_MODE_UNSUPPORTED', 'nonoverlapping/unsynchronized multicamera fusion unverified')
    mapping_ids = set()
    for mapping in config.get('clock_mappings', []):
        _identity(mapping.get('mapping_id'), 'mapping_id')
        require(mapping['mapping_id'] not in mapping_ids, 'SENSOR_CLOCK_INVALID', 'duplicate clock mapping')
        mapping_ids.add(mapping['mapping_id'])
        for key in ('source_clock', 'target_clock'):
            _identity(mapping.get(key), key)
        for key in ('source_anchor_ns', 'target_anchor_ns', 'uncertainty_ns'):
            _integer(mapping.get(key), key)
        for key in ('rate_numerator', 'rate_denominator'):
            _integer(mapping.get(key), key, 1)
        require(mapping.get('evidence_status') in ('CODE_SUPPORTED', 'VERIFIED')
                and _sha(mapping.get('source_sha256')),
                'SENSOR_CLOCK_INVALID', 'evidenced affine mapping required')
        if 'quantization' in mapping:
            require(mapping['quantization'] == 'ROUND_HALF_EVEN'
                    and mapping['uncertainty_ns'] >= 1,
                    'SENSOR_CLOCK_INVALID', 'quantization policy and at least 1ns representation uncertainty required')
    canonical(config)
    return config


def _ns(stamp):
    t = TimestampEvidence.from_dict(stamp)
    require(t.unit in _UNIT_NS and t.clock_domain != 'UNKNOWN',
            'SENSOR_CLOCK_UNQUALIFIED', 'replay needs a known clock and time unit')
    return t.raw_value * _UNIT_NS[t.unit]


def event_time(event, config):
    """Return (clock domain, integer ns), checking any separate affine mapping."""
    raw = TimestampEvidence.from_dict(event['timestamp'])
    mapped = event.get('mapped_timestamp')
    if mapped is None:
        require(event.get('clock_mapping_id') is None, 'SENSOR_CLOCK_INVALID', 'mapping output missing')
        exact = Fraction(_ns(event['timestamp']))
        require(exact.denominator == 1, 'SENSOR_CLOCK_UNQUALIFIED',
                'fractional ns needs an explicit, evidenced quantization mapping')
        return raw.clock_domain, exact.numerator
    candidates = [m for m in config.get('clock_mappings', [])
                  if m['mapping_id'] == event.get('clock_mapping_id')]
    require(len(candidates) == 1, 'SENSOR_CLOCK_INVALID', 'mapping identity missing')
    m = candidates[0]
    target = TimestampEvidence.from_dict(mapped)
    exact = Fraction(_ns(event['timestamp'])-m['source_anchor_ns']) * m['rate_numerator'] / m['rate_denominator']
    exact += m['target_anchor_ns']
    if m.get('quantization') == 'ROUND_HALF_EVEN':
        expected = round(exact)
    else:
        require(exact.denominator == 1, 'SENSOR_CLOCK_INVALID', 'mapping cannot invent fractional nanoseconds')
        expected = exact.numerator
    require(raw.clock_domain == m['source_clock'] and target.clock_domain == m['target_clock']
            and target.unit == 'ns' and target.raw_value == expected
            and target.semantic == 'MAPPED_SENSOR_TIME'
            and target.evidence_status == m['evidence_status'],
            'SENSOR_CLOCK_INVALID', 'mapped time differs from configured evidence')
    return target.clock_domain, target.raw_value


def _relative_image(name):
    require(isinstance(name, str) and bool(name), 'SENSOR_IMAGE_INVALID', 'image path string required')
    path = Path(name)
    require(not path.is_absolute() and path.parts and '..' not in path.parts
            and path.parts[0] == 'images' and path.suffix == '.png',
            'SENSOR_IMAGE_INVALID', 'recording-local PNG under images/ required')
    return path


def _image_path(base, name):
    path = _relative_image(name)
    require(base is not None, 'SENSOR_IMAGE_INVALID', 'image source directory required')
    path = Path(base)/path
    require(path.is_file() and not path.is_symlink()
            and path.resolve().is_relative_to(Path(base).resolve()),
            'SENSOR_IMAGE_INVALID', 'image must remain inside recording')
    return path


def validate_events(events, config, base_dir=None):
    """Audit raw stream order; report gaps, never repair or silently sort them."""
    validate_config(config)
    require(isinstance(events, list) and bool(events), 'SENSOR_EVENTS_INVALID', 'nonempty event list')
    sensors = {s['sensor_id']: s for s in [config['imu'], *config['cameras']]}
    imu_id = config['imu']['sensor_id']
    previous, counts, gaps, intervals, clocks = {}, {}, [], {}, set()
    epoch_counts = {}
    all_times_known = True
    last_epoch = -1
    camera_times = {s: [] for s in config['selected_camera_ids']}
    for i, event in enumerate(events):
        require(isinstance(event, dict) and type(event.get('schema_version')) is int
                and event['schema_version'] == 1,
                'SENSOR_EVENTS_INVALID', 'event schema version')
        require(set(event) <= {'schema_version', 'kind', 'sensor_id', 'frame_id', 'sequence',
                'epoch', 'config_version', 'timestamp', 'source_reference', 'delivery_delay_ns',
                'mapped_timestamp', 'clock_mapping_id', 'payload', 'exposure'},
                'SENSOR_PROVENANCE_INVALID', 'only sensor events belong in the recording')
        sensor_id = event.get('sensor_id')
        require(sensor_id in sensors, 'SENSOR_IDENTITY_INVALID', str(sensor_id))
        sensor = sensors[sensor_id]
        epoch = _integer(event.get('epoch'), 'epoch')
        require(epoch >= last_epoch, 'SENSOR_EPOCH_INVALID', 'epoch order')
        last_epoch = epoch
        require(event.get('config_version') == config['config_version'],
                'SENSOR_CONFIG_CHANGED', 'configuration change requires a new recording/epoch')
        sequence = _integer(event.get('sequence'), 'sequence')
        require(isinstance(event.get('timestamp'), dict) and set(event['timestamp']) ==
                {'raw_value', 'unit', 'clock_domain', 'semantic', 'evidence_status'},
                'SENSOR_CLOCK_INVALID', 'timestamp fields')
        stamp = TimestampEvidence.from_dict(event.get('timestamp'))
        require({k:v for k,v in stamp.to_dict().items() if k != 'raw_value'} == sensor['timestamp_contract'],
                'SENSOR_CLOCK_CHANGED', 'raw timestamp contract changed')
        require(event.get('frame_id') == sensor['frame_id'], 'SENSOR_AXES_INVALID', 'frame identity')
        source = event.get('source_reference', {})
        require(_sha(source.get('sha256')) and type(source.get('row')) is int and source['row'] >= 0,
                'SENSOR_PROVENANCE_INVALID', 'event source row/hash')
        _integer(event.get('delivery_delay_ns', 0), 'delivery delay')
        require(event.get('mapped_timestamp') is not None or event.get('clock_mapping_id') is None,
                'SENSOR_CLOCK_INVALID', 'mapping output missing')
        key = (epoch, sensor_id)
        raw_ns = stamp.raw_value * _UNIT_NS[stamp.unit] if stamp.unit in _UNIT_NS else None
        if key in previous:
            before_seq, before_stamp, before_index = previous[key]
            require(sequence > before_seq and stamp.raw_value > before_stamp,
                    'SENSOR_ORDER_INVALID', 'duplicate/reordered source sequence or time')
            if sequence != before_seq + 1:
                missing = sequence-before_seq-1
                gaps.append(dict(sensor_id=sensor_id, epoch=epoch, after_event=before_index,
                    before_event=i, unrepresented_sequence_values=missing,
                    missing_sample_count=missing if (sensor['sequence_semantics'] == 'SOURCE_SAMPLE_COUNTER'
                        and sensor.get('sequence_evidence', {}).get('status') == 'VERIFIED') else None))
            if raw_ns is not None:
                intervals.setdefault(key, []).append((stamp.raw_value-before_stamp)*_UNIT_NS[stamp.unit])
        previous[key] = (sequence, stamp.raw_value, i)
        counts[sensor_id] = counts.get(sensor_id, 0)+1
        epoch_counts.setdefault(epoch, {})[sensor_id] = epoch_counts.get(epoch, {}).get(sensor_id, 0)+1
        if event.get('mapped_timestamp') is not None:
            require(isinstance(event['mapped_timestamp'], dict) and set(event['mapped_timestamp']) ==
                    {'raw_value', 'unit', 'clock_domain', 'semantic', 'evidence_status'},
                    'SENSOR_CLOCK_INVALID', 'mapped timestamp fields')
            clocks.add(event_time(event, config)[0])
        elif raw_ns is not None and stamp.clock_domain != 'UNKNOWN':
            clocks.add(stamp.clock_domain)
            require(event.get('clock_mapping_id') is None, 'SENSOR_CLOCK_INVALID', 'mapping output missing')
            if Fraction(raw_ns).denominator != 1:
                all_times_known = False
        else:
            all_times_known = False
        payload = event.get('payload', {})
        require(isinstance(payload, dict), 'SENSOR_EVENTS_INVALID', 'sensor payload object')
        if sensor_id == imu_id:
            require(event.get('kind') == 'IMU', 'SENSOR_EVENTS_INVALID', 'IMU event kind')
            require(set(payload) <= {'measurement_kind', 'gyro_xyz', 'accel_xyz', 'gyro_unit',
                    'accel_unit', 'saturation'}, 'SENSOR_PROVENANCE_INVALID', 'raw IMU payload only')
            validate_imu_sample(dict(schema_version=1, sequence=sequence,
                measurement_kind=payload.get('measurement_kind'), device_id=sensor_id,
                timestamp=event['timestamp'], gyro_xyz=payload.get('gyro_xyz'),
                accel_xyz=payload.get('accel_xyz'), gyro_unit=payload.get('gyro_unit'),
                accel_unit=payload.get('accel_unit'), saturation=payload.get('saturation')))
        else:
            require(event.get('kind') == 'CAMERA' and sensor_id in config['selected_camera_ids'],
                    'SENSOR_EVENTS_INVALID', 'unselected camera or event kind')
            require(payload.get('encoding') == 'mono8', 'SENSOR_IMAGE_INVALID', 'mono8 required')
            require(set(payload) == {'encoding', 'path', 'width', 'height', 'pixels_sha256',
                    'encoded_sha256'}, 'SENSOR_PROVENANCE_INVALID', 'camera payload fields')
            _relative_image(payload.get('path'))
            for name in ('width', 'height'):
                _integer(payload.get(name), name, 1)
            require(_sha(payload.get('pixels_sha256')) and _sha(payload.get('encoded_sha256')),
                    'SENSOR_IMAGE_INVALID', 'image hashes required')
            exposure = event.get('exposure')
            if exposure is not None:
                status = _evidence(exposure, 'exposure')
                if status != 'UNKNOWN':
                    require(exposure.get('event') in ('START', 'MIDPOINT', 'END')
                            and type(exposure.get('duration_ns')) is int and exposure['duration_ns'] >= 0
                            and exposure.get('shutter') in ('GLOBAL', 'ROLLING'),
                            'SENSOR_EXPOSURE_INVALID', 'explicit exposure event/duration/shutter')
            if base_dir is not None:
                path = _image_path(base_dir, payload.get('path', ''))
                require(file_hash(path) == payload['encoded_sha256'], 'HASH_MISMATCH', str(path))
                pixels = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
                require(pixels is not None and pixels.dtype == np.uint8
                        and pixels.shape == (payload['height'], payload['width'])
                        and hashlib.sha256(pixels.tobytes()).hexdigest() == payload['pixels_sha256'],
                        'SENSOR_IMAGE_INVALID', 'decoded pixels/dimensions differ')
            camera_times[sensor_id].append((epoch, stamp.to_dict()))
    require(counts.get(imu_id, 0) > 0 and all(counts.get(s, 0) > 0 for s in camera_times),
            'SENSOR_STREAM_MISSING', 'selected camera and IMU streams required')
    require(all(c.get(imu_id, 0) > 0 and all(c.get(s, 0) > 0 for s in camera_times)
                for c in epoch_counts.values()), 'SENSOR_STREAM_MISSING', 'selected streams required in each epoch')
    if config['mode'] == 'stereo':
        a, b = config['selected_camera_ids']
        require(camera_times[a] == camera_times[b], 'SENSOR_STEREO_UNSYNCHRONIZED',
                'exact original source time/clock/evidence pairing required')
    canonical(events)
    cadence = []
    for (epoch, sensor_id), values in intervals.items():
        cadence.append(dict(sensor_id=sensor_id, epoch=epoch, min_interval_ns=float(min(values)),
            max_interval_ns=float(max(values)), observed_rate_hz=float(1e9*len(values)/sum(values))))
    return dict(status='PASS', qualification='SENSOR_RECORD_AUDIT_ONLY', counts=counts,
                epochs=sorted({r['epoch'] for r in events}), sequence_gaps=gaps, cadence=cadence,
                common_replay_clock=all_times_known and len(clocks) == 1,
                physical_camera_imu_qualification=False,
                missing_sample_policy='only verified source-sample counters establish missing samples')


def load_recording(directory, *, allow_invalid=False):
    """Verify all frozen artifacts before parsing; extra/unlisted files are refused."""
    root = Path(directory).resolve()
    manifest = read_json(root/'recording.json')
    require(type(manifest.get('schema_version')) is int and manifest['schema_version'] == 1
            and manifest.get('truth_included') is False,
            'SENSOR_PROVENANCE_INVALID', 'sensor-only recording manifest')
    artifacts = manifest.get('artifacts_sha256', {})
    require({'config.json', 'events.jsonl'} <= set(artifacts),
            'SENSOR_RECORDING_INVALID', 'config/events hashes required')
    require(not any(p.is_symlink() for p in root.rglob('*')), 'SENSOR_RECORDING_INVALID', 'no symlink artifacts')
    actual = {str(p.relative_to(root)) for p in root.rglob('*')
              if p.is_file() and p != root/'recording.json'}
    require(actual == set(artifacts), 'SENSOR_RECORDING_INVALID', 'artifact membership mismatch')
    for name, digest in artifacts.items():
        require(name in ('config.json', 'events.jsonl') or
                str(_relative_image(name)) == name, 'SENSOR_RECORDING_INVALID', 'sensor artifacts only')
        path = root/name
        require(_sha(digest) and file_hash(path) == digest, 'HASH_MISMATCH', name)
    config = read_json(root/'config.json')
    events = [json.loads(line, parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
              for line in (root/'events.jsonl').read_text().splitlines() if line.strip()]
    validate_config(config)
    expected = {'config.json', 'events.jsonl'} | {
        e.get('payload', {}).get('path') for e in events if e.get('kind') == 'CAMERA'}
    require(set(artifacts) == expected, 'SENSOR_RECORDING_INVALID', 'only referenced sensor images are accepted')
    require(manifest.get('provenance') == config['provenance'],
            'SENSOR_PROVENANCE_INVALID', 'manifest/config provenance differs')
    if not allow_invalid:
        validation = validate_events(events, config, root)
        require(manifest.get('validation') == validation, 'SENSOR_RECORDING_INVALID', 'frozen validation differs')
    return config, events, manifest


def _seal_recording(root, config, events, derivation):
    """Write the one metadata/manifest format after images have been written."""
    try:
        validation = validate_events(events, config, root)
    except ValueError as exc:
        if derivation is None:
            raise
        validation = dict(status='FAIL', reason=str(exc), qualification='INJECTED_INVALID_INPUT')
    write_json(root/'config.json', config)
    with (root/'events.jsonl').open('x') as stream:
        for event in events:
            stream.write(canonical(event)+'\n')
    manifest = dict(schema_version=1, truth_included=False, provenance=config['provenance'],
        validation=validation, derivation=derivation,
        artifacts_sha256={str(p.relative_to(root)): file_hash(p)
            for p in sorted(root.rglob('*')) if p.is_file()})
    write_json(root/'recording.json', manifest)
    return manifest


def write_recording(output, config, events, image_source=None, *, derivation=None):
    """Create a fresh output only. Invalid injected cases retain an explicit failure."""
    validate_config(config)
    canonical(events)
    # Validation occurs before creating anything. Intentional invalid inputs are
    # created only by derive_recording, which records the original event ledger.
    require(derivation is None, 'SENSOR_RECORDING_INVALID', 'use derive_recording for fault provenance')
    validate_events(events, config, image_source)
    sources = {e['payload']['path']: _image_path(image_source, e['payload']['path'])
               for e in events if e.get('kind') == 'CAMERA'}
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    try:
        for name, source in sources.items():
            destination = output/name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
        return _seal_recording(output, config, events, None)
    except Exception:
        shutil.rmtree(output)
        raise


def replay(events, config, emit, rate=1.0, *, clock_ns=time.monotonic_ns, sleep=time.sleep):
    """Replay delivery deadlines; sensor times remain intact even under delay.

    The complete source sequence is validated before emitting its first event.
    Delayed delivery is scheduled explicitly and may expose out-of-order sensor
    acquisition times to a consumer. Nothing silently sorts acquisition records.
    """
    validate_events(events, config)
    number(rate, 'replay rate', positive=True)
    timed = [event_time(e, config) for e in events]
    require(len({c for c, _ in timed}) == 1, 'SENSOR_CLOCK_UNMAPPED', 'one replay clock required')
    factor = Decimal(str(rate))
    begin = clock_ns()
    lateness, boundaries = [], []
    # A reset can move acquisition time backwards. Never sort across epochs or
    # infer a physical time gap between independent clock histories.
    for epoch in sorted({e['epoch'] for e in events}):
        indices = [i for i, event in enumerate(events) if event['epoch'] == epoch]
        origin = min(timed[i][1] for i in indices)
        schedule = [(int(Decimal(timed[i][1]-origin)/factor)+events[i].get('delivery_delay_ns', 0),
                     0 if events[i]['kind'] == 'IMU' else 1, i) for i in indices]
        schedule.sort()
        epoch_begin = clock_ns()
        boundaries.append(dict(epoch=epoch, sensor_origin_ns=origin, replay_start_monotonic_ns=epoch_begin))
        for n, (offset, _, i) in enumerate(schedule):
            deadline = epoch_begin+offset
            remaining = deadline-clock_ns()
            if remaining > 0:
                sleep(remaining/1e9)
            observed = clock_ns()
            lateness.append(max(0, observed-deadline))
            emit(copy.deepcopy(events[i]), dict(delivery_monotonic_ns=observed,
                scheduled_monotonic_ns=deadline, scheduler_lateness_ns=observed-deadline,
                epoch=epoch, epoch_started=n == 0, epoch_sensor_origin_ns=origin))
    return dict(status='PASS', qualification='REPLAY_TRANSPORT_ONLY', event_count=len(events),
                rate=rate, elapsed_ns=clock_ns()-begin,
                maximum_scheduler_lateness_ns=max(lateness), sensor_timestamps_modified=False,
                epoch_boundaries=boundaries, inter_epoch_policy='NO_INFERRED_TIME_GAP')


class CameraSelection:
    """Explicit independent-mono selection; every actual switch starts a new epoch."""
    def __init__(self, config, epoch=0):
        validate_config(config)
        require(config['mode'] == 'mono', 'CAMERA_MODE_UNSUPPORTED', 'independent mono selection only')
        self.config = copy.deepcopy(config)
        self.epoch = _integer(epoch, 'epoch')

    def switch(self, camera_id):
        require(camera_id in {s['sensor_id'] for s in self.config['cameras']},
                'SENSOR_IDENTITY_INVALID', 'camera selection')
        old = self.config['selected_camera_ids'][0]
        if old != camera_id:
            self.epoch += 1
            self.config['selected_camera_ids'] = [camera_id]
        return dict(camera_id=camera_id, previous_camera_id=old, epoch=self.epoch,
                    reset_required=old != camera_id,
                    calibration_version=next(s for s in self.config['cameras'] if s['sensor_id'] == camera_id)['calibration']['version'])


def _profile(profile, config):
    require(isinstance(profile, dict) and type(profile.get('schema_version')) is int
            and profile['schema_version'] == 1 and profile.get('kind') in _FAULTS,
            'SENSOR_FAULT_INVALID', 'one supported fault kind')
    _integer(profile.get('seed'), 'fault seed')
    _integer(profile.get('start_elapsed_ns'), 'fault start')
    if 'epoch' in profile:
        _integer(profile['epoch'], 'fault epoch')
    require(type(profile.get('end_elapsed_ns')) is int
            and profile['end_elapsed_ns'] > profile['start_elapsed_ns'], 'SENSOR_FAULT_INVALID', 'half-open fault window')
    ids = profile.get('sensor_ids')
    known = {s['sensor_id'] for s in [config['imu'], *config['cameras']]}
    require(isinstance(ids, list) and ids and len(set(ids)) == len(ids) and set(ids) <= known,
            'SENSOR_FAULT_INVALID', 'explicit sensor IDs')
    kind = profile['kind']
    require(all((s == config['imu']['sensor_id']) == kind.startswith('imu_')
                for s in ids) if kind != 'delivery_delay' else True,
            'SENSOR_FAULT_INVALID', 'fault/sensor kind differs')
    if kind == 'camera_drop':
        p = number(profile.get('probability'), 'drop probability')
        require(0 <= p <= 1, 'SENSOR_FAULT_INVALID', 'drop probability range')
    if 'camera_bundle_policy' in profile:
        require(kind == 'camera_drop' and profile['camera_bundle_policy'] == 'SAME_SOURCE_TIME'
                and config['mode'] == 'stereo' and set(ids) == set(config['selected_camera_ids']),
                'SENSOR_FAULT_INVALID', 'paired dropout requires both verified stereo cameras')
    if kind == 'delivery_delay':
        _integer(profile.get('delay_ns'), 'delay')
    if kind == 'camera_blur':
        kernel = profile.get('kernel_size')
        require(type(kernel) is int and kernel >= 3 and kernel % 2 == 1,
                'SENSOR_FAULT_INVALID', 'odd blur kernel >=3')
    if kind == 'camera_dim':
        gain = number(profile.get('gain'), 'image gain')
        require(0 <= gain <= 1, 'SENSOR_FAULT_INVALID', 'gain in [0,1]')
    if kind == 'camera_texture':
        fraction = number(profile.get('fraction'), 'texture mask fraction')
        require(0 < fraction <= 1, 'SENSOR_FAULT_INVALID', 'mask fraction in (0,1]')
    if kind == 'imu_bias':
        for field in ('gyro_bias_rad_s', 'accel_bias_m_s2'):
            require(isinstance(profile.get(field), list) and len(profile[field]) == 3,
                    'SENSOR_FAULT_INVALID', field)
            for x in profile[field]:
                number(x, field)
    canonical(profile)


def derive_recording(source_directory, output_directory, profile):
    """Derive one reproducible disturbance; original files/IDs/times are immutable."""
    source = Path(source_directory).resolve()
    config, events, manifest = load_recording(source)
    _profile(profile, config)
    root = Path(output_directory).resolve()
    require(root != source and source not in root.parents,
            'SENSOR_OUTPUT_INVALID', 'derived recording must be outside source')
    times = [event_time(e, config) for e in events]
    require(len({c for c, _ in times}) == 1, 'SENSOR_CLOCK_UNMAPPED', 'fault windows need common clock')
    epochs = {e['epoch'] for e in events}
    require(len(epochs) == 1 or 'epoch' in profile, 'SENSOR_FAULT_INVALID', 'multi-epoch faults require an explicit epoch')
    epoch = profile.get('epoch', min(epochs))
    require(epoch in epochs, 'SENSOR_FAULT_INVALID', 'fault epoch absent')
    origin = min(t for e, (_, t) in zip(events, times) if e['epoch'] == epoch)
    rng = random.Random(profile['seed'])
    derived, changes, selected, images = [], [], [], []
    bundle_drop = {}
    kind = profile['kind']
    def change(index, event, action):
        return dict(source_event=index, sensor_id=event['sensor_id'], sequence=event['sequence'],
                    epoch=event['epoch'], timestamp=copy.deepcopy(event['timestamp']), action=action)
    for i, event in enumerate(events):
        row = copy.deepcopy(event)
        inside = (row['epoch'] == epoch and row['sensor_id'] in profile['sensor_ids'] and
                  profile['start_elapsed_ns'] <= times[i][1]-origin < profile['end_elapsed_ns'])
        action = None
        if inside:
            drop = False
            if kind == 'camera_drop':
                if profile.get('camera_bundle_policy') == 'SAME_SOURCE_TIME':
                    key=(row['epoch'], times[i])
                    if key not in bundle_drop:
                        bundle_drop[key] = rng.random() < profile['probability']
                    drop = bundle_drop[key]
                else:
                    drop = rng.random() < profile['probability']
            if kind in ('camera_gap', 'imu_gap') or drop:
                changes.append(change(i, event, 'DROP'))
                continue
            if kind == 'delivery_delay':
                row['delivery_delay_ns'] = row.get('delivery_delay_ns', 0)+profile['delay_ns']
                action = 'DELAY'
            elif kind == 'imu_nonfinite':
                row['payload']['gyro_xyz'][0] = 'NaN'
                action = 'IEEE754_NONFINITE_REPRESENTATION'
            elif kind == 'imu_bias':
                for field, bias in [('gyro_xyz', profile['gyro_bias_rad_s']), ('accel_xyz', profile['accel_bias_m_s2'])]:
                    row['payload'][field] = [x+b for x, b in zip(row['payload'][field], bias)]
                action = 'BIAS'
            elif kind in ('imu_duplicate', 'imu_reorder'):
                selected.append((len(derived), i, event))
            elif kind in ('camera_occlusion', 'camera_blur', 'camera_dim', 'camera_texture'):
                action = kind.upper()
        if row['kind'] == 'CAMERA':
            images.append((row, row['payload']['path'], bool(action and kind.startswith('camera_')), i))
        if action:
            changes.append(change(i, event, action))
        derived.append(row)
    if kind == 'imu_duplicate' and selected:
        index, source_index, event = selected[0]
        derived.insert(index+1, copy.deepcopy(derived[index]))
        changes.append(change(source_index, event, 'IMU_DUPLICATE'))
    elif kind == 'imu_reorder' and len(selected) >= 2:
        (a, source_a, event_a), (b, source_b, event_b) = selected[:2]
        derived[a], derived[b] = derived[b], derived[a]
        changes.extend([change(source_a, event_a, 'IMU_REORDER'), change(source_b, event_b, 'IMU_REORDER')])
    require(bool(changes), 'SENSOR_FAULT_EMPTY', 'profile affected no source events')
    provenance = dict(parent_recording_sha256=file_hash(source/'recording.json'),
        parent_artifacts_sha256=manifest['artifacts_sha256'], profile=profile, changes=changes,
        experiment_kind='ARTIFICIAL_DISTURBANCE', original_event_count=len(events), derived_event_count=len(derived))
    # Selection and all preflight checks finish before creating a new output.
    # Cleanup on write failure affects only this invocation's fresh directory.
    root.mkdir(parents=True, exist_ok=False)
    try:
        used = {e['payload']['path'] for e in events if e['kind'] == 'CAMERA'}
        for row, name, transform, index in images:
            old = _image_path(source, name)
            if transform:
                destination_name = 'images/disturbance/'+str(index)+'.png'
                while destination_name in used:
                    destination_name = str(Path(destination_name).with_stem(Path(destination_name).stem+'_derived'))
                used.add(destination_name)
                row['payload']['path'] = destination_name
            destination = root/row['payload']['path']
            destination.parent.mkdir(parents=True, exist_ok=True)
            if transform:
                pixels = cv2.imread(str(old), cv2.IMREAD_UNCHANGED)
                if kind == 'camera_occlusion':
                    pixels[:] = 0
                elif kind == 'camera_blur':
                    pixels = cv2.GaussianBlur(pixels, (profile['kernel_size'], profile['kernel_size']), 0)
                elif kind == 'camera_dim':
                    pixels = np.rint(pixels.astype(float)*profile['gain']).astype(np.uint8)
                else:
                    width = max(1, int(pixels.shape[1]*profile['fraction']))
                    pixels[:, :width] = 0
                ok, png = cv2.imencode('.png', pixels)
                require(ok, 'SENSOR_IMAGE_INVALID', 'derived image encode failed')
                destination.write_bytes(png.tobytes())
                row['payload'].update(encoded_sha256=file_hash(destination),
                    pixels_sha256=hashlib.sha256(pixels.tobytes()).hexdigest())
            elif not destination.exists():
                shutil.copyfile(old, destination)
        output_manifest = _seal_recording(root, config, derived, provenance)
        for name, digest in manifest['artifacts_sha256'].items():
            require(file_hash(source/name) == digest, 'HASH_MISMATCH', 'source changed during derivation')
        return output_manifest
    except Exception:
        shutil.rmtree(root)
        raise


def mock_recording(output, camera_count=1, frame_count=4, frame_period_ns=50000000, *, selected_camera_id='front'):
    """Create labeled mock data only; this is never a public-dataset VIO result."""
    _integer(camera_count, 'camera count', 1)
    require(camera_count <= 2, 'CAMERA_MODE_UNSUPPORTED', 'at most front/down mock definitions')
    require(selected_camera_id in (('front', 'down') if camera_count == 2 else ('front',)),
            'SENSOR_IDENTITY_INVALID', 'selected mock camera must have a definition')
    _integer(frame_count, 'frame count', 2)
    _integer(frame_period_ns, 'frame period', 1)
    root = Path(output).resolve()
    root.mkdir(parents=True, exist_ok=False)
    stamp_contract = dict(unit='ns', clock_domain='MOCK:clock', semantic='MOCK_ACQUISITION', evidence_status='CODE_SUPPORTED')
    origin = 1400000000000000000
    source_hash = file_hash(__file__)
    sensor = dict(frame_id='imu0', sensor_id='imu0', timestamp_contract=stamp_contract,
        sequence_semantics='RECORD_COUNTER', axes=dict(handedness='RIGHT', description='mock x forward y left z up'),
        measurement_kind='RAW_IMU', gyro_unit='rad/s', accel_unit='m/s^2')
    cameras = []
    for j in range(camera_count):
        cameras.append(dict(sensor_id='front' if j == 0 else 'down', frame_id='camera'+str(j),
            role='FRONT' if j == 0 else 'DOWN', timestamp_contract=stamp_contract,
            sequence_semantics='RECORD_COUNTER', axes=dict(handedness='RIGHT', description='mock optical x right y down z forward'),
            calibration=dict(version='MOCK_UNKNOWN_CALIBRATION', intrinsics=dict(status='UNKNOWN'),
                T_imu_camera=dict(status='UNKNOWN'), camera_imu_offset=dict(status='UNKNOWN'))))
    config = dict(schema_version=1, config_version='MOCK_V1', mode='mono', selected_camera_ids=[selected_camera_id],
        imu=sensor, cameras=cameras, clock_mappings=[],
        provenance=dict(kind='MOCK', source_reference='MOCK_SENSOR_SOURCE_ONLY', source_sha256=source_hash, truth_included=False))
    events = []
    for i in range(frame_count):
        stamp = dict(raw_value=origin+i*frame_period_ns, **stamp_contract)
        ref = dict(row=i, sha256=source_hash)
        base = dict(schema_version=1, sequence=i, epoch=0, timestamp=stamp,
                    config_version=config['config_version'], source_reference=ref)
        events.append(dict(base, kind='IMU', sensor_id='imu0', frame_id='imu0',
            payload=dict(measurement_kind='RAW_IMU', gyro_xyz=[0., 0., 0.], accel_xyz=[0., 0., 9.81],
                         gyro_unit='rad/s', accel_unit='m/s^2', saturation=None)))
        pixels = np.indices((24, 32)).sum(axis=0).astype(np.uint8)*7
        ok, encoded = cv2.imencode('.png', pixels)
        require(ok, 'SENSOR_IMAGE_INVALID', 'mock PNG encoding')
        name = 'images/'+str(i)+'.png'
        (root/'images').mkdir(exist_ok=True)
        (root/name).write_bytes(encoded.tobytes())
        events.append(dict(base, kind='CAMERA', sensor_id=selected_camera_id,
            frame_id='camera0' if selected_camera_id == 'front' else 'camera1',
            exposure=dict(status='UNKNOWN'), payload=dict(path=name, encoding='mono8', width=32, height=24,
                pixels_sha256=hashlib.sha256(pixels.tobytes()).hexdigest(), encoded_sha256=file_hash(root/name))))
    return _seal_recording(root, config, events, None)


def _decimal_picoseconds(text):
    """Parse printed dataset seconds exactly, retaining its representation only."""
    try:
        seconds = Decimal(text)
        picoseconds = seconds*10**12
    except ArithmeticError:
        require(False, 'SENSOR_CLOCK_INVALID', 'invalid decimal seconds')
    require(seconds.is_finite() and seconds >= 0 and picoseconds == picoseconds.to_integral_value(),
            'SENSOR_CLOCK_INVALID', 'source seconds must be exactly representable in integer ps')
    return int(picoseconds)


def import_dataset(archive, output, dataset, *, camera_id='cam0', mode='mono',
                   calibration=None, reserve_bytes=20*1024**3):
    """Import real EuRoC/UZH sensor-only ZIP members into the shared contract.

    UZH printed decimal seconds are preserved as text and integer ps. An explicit
    half-even representation mapping supplies ns deadlines with <=0.5ns rounding;
    neither the mapping nor printed digits establish physical clock resolution,
    camera/IMU synchronization, exposure timing or calibration. Right/left UZH
    inputs remain independent mono configurations until their offsets are known.
    """
    require(dataset in ('EUROC', 'UZH_FPV') and camera_id in ('cam0', 'cam1'),
            'SENSOR_DATASET_INVALID', 'EUROC/UZH_FPV and explicit camera ID required')
    require(mode in ('mono', 'stereo') and (dataset == 'EUROC' or mode == 'mono'),
            'CAMERA_MODE_UNSUPPORTED', 'UZH stereo requires separately qualified camera time offsets')
    _integer(reserve_bytes, 'storage reserve')
    archive, root = Path(archive).resolve(), Path(output).resolve()
    if root.exists():
        raise FileExistsError(root)
    require(archive.is_file() and archive not in root.parents,
            'SENSOR_DATASET_INVALID', 'existing external source archive required')
    source_hash = file_hash(archive)
    clock = dataset+':'+source_hash[:16]+':RECORDED_CLOCK'
    contract = dict(unit='ns' if dataset == 'EUROC' else 'ps', clock_domain=clock,
                    semantic='DATASET_RECORDED_TIME', evidence_status='OBSERVED')
    axes_imu = dict(handedness='UNKNOWN', description='source gyro/accelerometer axes; physical mounting unqualified')
    imu = dict(sensor_id='imu0', frame_id='imu0', timestamp_contract=contract,
        sequence_semantics='RECORD_COUNTER', axes=axes_imu,
        measurement_kind='RAW_IMU', gyro_unit='rad/s', accel_unit='m/s^2')
    cameras = [dict(sensor_id=name, frame_id=name, role='LEFT' if name == 'cam0' else 'RIGHT',
        timestamp_contract=contract, sequence_semantics='RECORD_COUNTER',
        axes=dict(handedness='UNKNOWN', description='source camera optical axes; calibration required'),
        calibration=copy.deepcopy((calibration or {}).get(name, dict(version='UNKNOWN:'+source_hash[:16],
            intrinsics=dict(status='UNKNOWN'), T_imu_camera=dict(status='UNKNOWN'),
            camera_imu_offset=dict(status='UNKNOWN'))))) for name in ('cam0', 'cam1')]
    selected = ['cam0', 'cam1'] if mode == 'stereo' else [camera_id]
    config = dict(schema_version=1, config_version=dataset+':'+source_hash[:16]+':'+mode+':'+camera_id,
        mode=mode, selected_camera_ids=selected, imu=imu, cameras=cameras, clock_mappings=[],
        provenance=dict(kind='PUBLIC_DATASET', source_reference=str(archive), source_sha256=source_hash,
            dataset=dataset, truth_included=False, physical_timestamp_resolution='UNKNOWN',
            exposure_synchronization='UNKNOWN', imported_calibration='UNKNOWN' if calibration is None else 'EXPLICIT_SOURCE'))
    if dataset == 'UZH_FPV':
        config['clock_mappings'] = [dict(mapping_id='PRINTED_SECONDS_TO_REPLAY_NS', source_clock=clock,
            target_clock=clock+':QUANTIZED_NS', source_anchor_ns=0, target_anchor_ns=0,
            rate_numerator=1, rate_denominator=1, quantization='ROUND_HALF_EVEN', uncertainty_ns=1,
            evidence_status='CODE_SUPPORTED', source_sha256=file_hash(__file__))]
    if mode == 'stereo':
        config['stereo'] = dict(overlapping_view=True, synchronization='EXACT_SOURCE_TIME',
            evidence_status='CODE_SUPPORTED', source_sha256=source_hash)
    validate_config(config)
    events, image_members = [], []
    with zipfile.ZipFile(archive) as source:
        names = source.namelist()
        require(len(names) == len(set(names)), 'SENSOR_DATASET_INVALID', 'duplicate ZIP members')
        if dataset == 'EUROC':
            # Reuse the existing strict integer timestamp/IMU CSV parser.
            from tools.schurvins.ros_transport import rows
            streams = ['imu0', *selected]
            tables = {s: rows(source.read('mav0/'+s+'/data.csv').decode(), s) for s in streams}
            if mode == 'stereo':
                require([v[0] for v in tables['cam0']] == [v[0] for v in tables['cam1']],
                        'SENSOR_STEREO_UNSYNCHRONIZED', 'original timestamps differ')
            raw_rows = {}
            for sensor, table in tables.items():
                member = 'mav0/'+sensor+'/data.csv'
                rows_hash = hashlib.sha256(source.read(member)).hexdigest()
                raw_rows[sensor] = [(i, t, value, member, rows_hash, str(t))
                                    for i, (t, value) in enumerate(table)]
        else:
            raw_rows = {}
            for sensor in ['imu0', *selected]:
                member = 'imu.txt' if sensor == 'imu0' else ('left_images.txt' if sensor == 'cam0' else 'right_images.txt')
                content = source.read(member)
                rows_hash = hashlib.sha256(content).hexdigest()
                table = []
                previous_id, previous_time = -1, -1
                for line in content.decode().splitlines():
                    if not line.strip() or line.lstrip().startswith('#'):
                        continue
                    parts = line.split()
                    require(len(parts) == (8 if sensor == 'imu0' else 3),
                            'SENSOR_DATASET_INVALID', 'UZH source column count')
                    sequence, raw = int(parts[0]), _decimal_picoseconds(parts[1])
                    require(sequence > previous_id and raw > previous_time,
                            'SENSOR_ORDER_INVALID', 'UZH source ID/time must increase')
                    previous_id, previous_time = sequence, raw
                    value = [float(x) for x in parts[2:]] if sensor == 'imu0' else parts[2]
                    table.append((sequence, raw, value, member, rows_hash, parts[1]))
                require(bool(table), 'SENSOR_STREAM_MISSING', member)
                raw_rows[sensor] = table
        for sensor, table in raw_rows.items():
            for sequence, raw, value, member, rows_hash, text in table:
                event = dict(schema_version=1, kind='IMU' if sensor == 'imu0' else 'CAMERA',
                    sensor_id=sensor, frame_id=sensor, sequence=sequence, epoch=0,
                    config_version=config['config_version'], timestamp=dict(raw_value=raw, **contract),
                    source_reference=dict(row=sequence, sha256=rows_hash, member=member, timestamp_text=text))
                if dataset == 'UZH_FPV':
                    event['clock_mapping_id'] = 'PRINTED_SECONDS_TO_REPLAY_NS'
                    event['mapped_timestamp'] = dict(raw_value=round(Fraction(raw, 1000)), unit='ns',
                        clock_domain=clock+':QUANTIZED_NS', semantic='MAPPED_SENSOR_TIME', evidence_status='CODE_SUPPORTED')
                    residual = Fraction(event['mapped_timestamp']['raw_value']*1000-raw, 1000)
                    event['source_reference']['representation_rounding_error_ns'] = dict(
                        numerator=residual.numerator, denominator=residual.denominator)
                if sensor == 'imu0':
                    event['payload'] = dict(measurement_kind='RAW_IMU', gyro_xyz=value[:3], accel_xyz=value[3:],
                                            gyro_unit='rad/s', accel_unit='m/s^2', saturation=None)
                else:
                    image_member = 'mav0/'+sensor+'/data/'+value if dataset == 'EUROC' else value
                    require(not Path(image_member).is_absolute() and '..' not in Path(image_member).parts
                            and image_member in names, 'SENSOR_IMAGE_INVALID', 'missing/local source PNG required')
                    encoded = source.read(image_member)
                    pixels = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_UNCHANGED)
                    require(pixels is not None and pixels.ndim == 2 and pixels.dtype == np.uint8,
                            'SENSOR_IMAGE_INVALID', 'source mono8 PNG required')
                    name = 'images/'+sensor+'/'+str(sequence)+'.png'
                    event['exposure'] = dict(status='UNKNOWN')
                    event['payload'] = dict(path=name, encoding='mono8', width=pixels.shape[1], height=pixels.shape[0],
                        pixels_sha256=hashlib.sha256(pixels.tobytes()).hexdigest(),
                        encoded_sha256=hashlib.sha256(encoded).hexdigest())
                    image_members.append((name, image_member))
                events.append(event)
        # Merge distinct source streams only. Their own ID/time order is checked
        # first; no acquisition record is silently repaired by this merge.
        events.sort(key=lambda event: (event_time(event, config)[1], event['kind'] != 'IMU', event['sensor_id']))
        validate_events(events, config)
        parent = root.parent
        while not parent.exists():
            parent = parent.parent
        estimate = sum(source.getinfo(member).file_size for _, member in image_members)+len(events)*2000
        require(shutil.disk_usage(parent).free-estimate >= reserve_bytes,
                'SENSOR_STORAGE_BLOCKED', 'estimated output would breach configured storage reserve')
        root.mkdir(parents=True, exist_ok=False)
        try:
            for name, member in image_members:
                destination = root/name
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(source.read(member))
            report = _seal_recording(root, config, events, None)
            require(file_hash(archive) == source_hash, 'HASH_MISMATCH', 'dataset archive changed during import')
            return report
        except Exception:
            shutil.rmtree(root)
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    for command in ('validate', 'replay'):
        sub = commands.add_parser(command)
        sub.add_argument('--input', required=True)
        if command == 'replay':
            sub.add_argument('--rate', type=float, default=1.)
    sub = commands.add_parser('derive')
    sub.add_argument('--input', required=True)
    sub.add_argument('--output', required=True)
    sub.add_argument('--profile', required=True)
    sub = commands.add_parser('mock')
    sub.add_argument('--output', required=True)
    sub.add_argument('--frames', type=int, default=4)
    sub.add_argument('--cameras', type=int, default=1)
    sub.add_argument('--camera', choices=('front', 'down'), default='front')
    sub = commands.add_parser('import')
    sub.add_argument('--archive', required=True)
    sub.add_argument('--output', required=True)
    sub.add_argument('--dataset', choices=('EUROC', 'UZH_FPV'), required=True)
    sub.add_argument('--camera', choices=('cam0', 'cam1'), default='cam0')
    sub.add_argument('--mode', choices=('mono', 'stereo'), default='mono')
    sub.add_argument('--calibration', help='explicit sourced per-camera calibration JSON; unknown if absent')
    args = parser.parse_args(argv)
    if args.command == 'mock':
        report = mock_recording(args.output, args.cameras, args.frames, selected_camera_id=args.camera)
    elif args.command == 'import':
        report = import_dataset(args.archive, args.output, args.dataset, camera_id=args.camera, mode=args.mode,
                                calibration=None if args.calibration is None else read_json(args.calibration))
    elif args.command == 'derive':
        report = derive_recording(args.input, args.output, read_json(args.profile))
    else:
        config, events, manifest = load_recording(args.input)
        if args.command == 'validate':
            report = manifest['validation']
        else:
            report = replay(events, config, lambda event, receipt: print(canonical(dict(event=event, replay=receipt))), args.rate)
    print(canonical(dict(result=report)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
