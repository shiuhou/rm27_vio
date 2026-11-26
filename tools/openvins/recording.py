"""Seal shared sensor recordings for the existing ROS-free OpenVINS runner.

The ASL directory layout is a transport format, not a EuRoC provenance claim.
Native input uses an explicitly selected source epoch and its mapped integer
clock. Original events, identities and clocks remain in the source sidecars.
This module neither executes an estimator nor qualifies physical sensors.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml

from rm27.perception.vision.localization.experiments import sensors
from rm27.perception.vision.localization.experiments.common import (
    canonical, file_hash, number, object_hash, read_json, require, write_json,
)

_NOISE = ('gyroscope_noise_density', 'gyroscope_random_walk',
          'accelerometer_noise_density', 'accelerometer_random_walk')
_MATRICES = ('Tw', 'Ta', 'R_IMUtoACC', 'R_IMUtoGYRO', 'Tg')
_SYNCHRONOUS = dict(max_cameras=1, use_stereo=False, multi_threading_subs=False,
                    multi_threading_pubs=False, record_timing_information=False,
                    save_total_state=False, use_aruco=False, use_mask=False,
                    downsample_cameras=False)


class _UniqueLoader(yaml.SafeLoader):
    pass


def _unique_mapping(loader, node):
    pairs = loader.construct_pairs(node, deep=True)
    require(all(isinstance(key, str) for key, _ in pairs)
            and len({key for key, _ in pairs}) == len(pairs),
            'OPENVINS_CONFIG_INVALID', 'YAML mapping keys must be unique strings')
    return dict(pairs)


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def _yaml_load(text):
    # OpenCV's YAML 1.0 header is not a valid PyYAML directive.
    lines = text.splitlines()
    if lines and lines[0].startswith('%YAML:'):
        lines = lines[1:]
    value = yaml.load('\n'.join(lines), Loader=_UniqueLoader)
    require(isinstance(value, dict), 'OPENVINS_CONFIG_INVALID', 'YAML mapping required')
    canonical(value)
    return value


class _OpenCVDumper(yaml.SafeDumper):
    pass


_OpenCVDumper.add_representer(list, lambda dumper, value:
    dumper.represent_sequence('tag:yaml.org,2002:seq', value, flow_style=True))


def _yaml_text(value):
    # OpenCV FileStorage needs block mappings and flow-style numeric vectors.
    return '%YAML:1.0\n' + yaml.dump(value, Dumper=_OpenCVDumper, sort_keys=True,
                                    default_flow_style=False, width=10000)


def _evidence(value, kind, hardware, *, verify_files):
    accepted = ('VERIFIED',) if hardware else ('CODE_SUPPORTED', 'VERIFIED')
    require(sensors._evidence(value, kind) in accepted,
            'SENSOR_CALIBRATION_UNQUALIFIED', kind + ' needs qualified source evidence')
    reference = Path(value['source_reference'])
    require(reference.is_absolute(), 'SENSOR_EVIDENCE_INVALID', kind + ' needs an absolute source path')
    if verify_files:
        require(reference.is_file() and file_hash(reference) == value['source_sha256'],
                'HASH_MISMATCH', kind + ' source path/hash differs')


def _parameters(parameters, config, hardware, *, verify_files):
    sensors._sensor_only(parameters)
    require(isinstance(parameters, dict) and set(parameters) == {
        'schema_version', 'purpose', 'measurement_binding', 'imu_parameters',
        'gravity_mag', 'gravity_evidence'} and type(parameters['schema_version']) is int
        and parameters['schema_version'] == 1
        and parameters['purpose'] == 'OPENVINS_RECORDING_PARAMETERS',
        'NATIVE_PARAMETERS_INVALID', 'complete explicit OpenVINS parameters required')
    binding = parameters['measurement_binding']
    _evidence(binding, 'IMU measurement binding', hardware, verify_files=verify_files)
    require(binding.get('imu_id') == config['imu']['sensor_id']
            and binding.get('imu_frame') == config['imu']['frame_id']
            and binding.get('source_imu_config_sha256') == object_hash(config['imu'])
            and binding.get('measurement_kind') == 'RAW_SPECIFIC_FORCE'
            and binding.get('native_axes_transform') == 'IDENTITY',
            'SENSOR_AXES_INVALID', 'raw specific force/gyro must bind the exact sourced IMU frame')
    imu = parameters['imu_parameters']
    require(isinstance(imu, dict) and set(imu) == {*_NOISE, *_MATRICES, 'update_rate', 'model'}
            and imu['model'] == 'kalibr', 'NATIVE_PARAMETERS_INVALID',
            'explicit kalibr IMU noise, rate and intrinsic matrices required')
    for name in _NOISE:
        require(number(imu[name], name) >= 0, 'NATIVE_PARAMETERS_INVALID', name)
    number(imu['update_rate'], 'update_rate', positive=True)
    for name in _MATRICES:
        matrix = imu[name]
        require(isinstance(matrix, list) and len(matrix) == 3
                and all(isinstance(row, list) and len(row) == 3 for row in matrix),
                'NATIVE_PARAMETERS_INVALID', name + ' needs a 3x3 matrix')
        for row in matrix:
            for item in row:
                number(item, name)
        a = np.asarray(matrix, dtype=float)
        if name in ('Tw', 'Ta'):
            require(abs(np.linalg.det(a)) > 1e-12, 'NATIVE_PARAMETERS_INVALID', name + ' is singular')
        if name.startswith('R_'):
            require(np.allclose(a.T @ a, np.eye(3), atol=1e-6, rtol=0)
                    and abs(np.linalg.det(a) - 1) < 1e-6,
                    'SENSOR_AXES_INVALID', name + ' must be a proper rotation')
    number(parameters['gravity_mag'], 'gravity_mag', positive=True)
    _evidence(parameters['gravity_evidence'], 'gravity', hardware, verify_files=verify_files)


def _derive(config, events, parameters, tuning, source_epoch, *, verify_files):
    validation = sensors.validate_events(events, config)
    require(config['mode'] == 'mono' and len(config['selected_camera_ids']) == 1,
            'CAMERA_MODE_UNSUPPORTED', 'existing headless runtime supports one independent camera')
    epochs = validation['epochs']
    if source_epoch is None:
        require(len(epochs) == 1, 'SENSOR_EPOCH_INVALID', 'multiple epochs require explicit selection')
        source_epoch = epochs[0]
    require(type(source_epoch) is int and source_epoch in epochs,
            'SENSOR_EPOCH_INVALID', 'selected source epoch is unavailable')
    camera_id, imu_id = config['selected_camera_ids'][0], config['imu']['sensor_id']
    camera = next(c for c in config['cameras'] if c['sensor_id'] == camera_id)
    hardware = config['provenance']['kind'] == 'HARDWARE_RECORDING'
    selected = [(i, event) for i, event in enumerate(events) if event['epoch'] == source_epoch]
    for sensor in (config['imu'], camera):
        contract = sensor['timestamp_contract']
        if hardware:
            acquisition = ('ACQUISITION',) if sensor['sensor_id'] == imu_id else (
                'ACQUISITION', 'EXPOSURE_START', 'EXPOSURE_MIDPOINT', 'EXPOSURE_END')
            require(contract['evidence_status'] == 'VERIFIED'
                    and contract['semantic'] in acquisition,
                    'SENSOR_CLOCK_UNQUALIFIED', 'hardware needs verified acquisition/exposure timestamps')
        else:
            allowed = contract['evidence_status'] in ('CODE_SUPPORTED', 'VERIFIED')
            observed_public = (config['provenance']['kind'] == 'PUBLIC_DATASET'
                               and contract['evidence_status'] == 'OBSERVED'
                               and contract['semantic'] == 'DATASET_RECORDED_TIME')
            require((allowed and contract['semantic'] in ('ACQUISITION', 'MOCK_ACQUISITION',
                    'DATASET_RECORDED_TIME', 'EXPOSURE_START', 'EXPOSURE_MIDPOINT', 'EXPOSURE_END'))
                    or observed_public, 'SENSOR_CLOCK_UNQUALIFIED', 'source acquisition semantics required')
        if hardware:
            require(sensor['axes']['handedness'] == 'RIGHT', 'SENSOR_AXES_INVALID',
                    'hardware axes must be explicitly right handed')
    for mapping in config.get('clock_mappings', []):
        if hardware and any(e.get('clock_mapping_id') == mapping['mapping_id'] for _, e in selected):
            require(mapping['evidence_status'] == 'VERIFIED', 'SENSOR_CLOCK_UNQUALIFIED',
                    'hardware affine clock mapping must be verified')
    calibration = camera['calibration']
    for name in ('intrinsics', 'T_imu_camera', 'camera_imu_offset'):
        _evidence(calibration[name], name, hardware, verify_files=verify_files)
    require(len(calibration['intrinsics']['parameters']) == 8, 'SENSOR_CALIBRATION_INVALID',
            'camera requires fx,fy,cx,cy and exactly four distortion coefficients')
    _parameters(parameters, config, hardware, verify_files=verify_files)
    require(all(type(tuning.get(k)) is type(v) and tuning[k] == v for k, v in _SYNCHRONOUS.items()),
            'OPENVINS_CONFIG_UNSUPPORTED', 'explicit mono, synchronous, full-size, no-mask/no-dump tuning required')
    require(not any(k.startswith('mask') or k.startswith('sim_') for k in tuning),
            'OPENVINS_CONFIG_UNSUPPORTED', 'external masks and simulation inputs are unsupported')
    allowed_paths = {'relative_config_imu', 'relative_config_imucam', 'record_timing_filepath',
                     'filepath_est', 'filepath_std', 'filepath_gt'}
    require(not any(('path' in k or k.startswith('relative_config_')) and k not in allowed_paths
                    for k in tuning), 'OPENVINS_CONFIG_UNSUPPORTED', 'unowned external configuration path')
    timed = [(i, e, *sensors.event_time(e, config)) for i, e in selected]
    require(len({clock for _, _, clock, _ in timed}) == 1,
            'SENSOR_CLOCK_UNMAPPED', 'one mapped camera/IMU replay clock required')
    require(all(type(ns) is int and 0 < ns <= 2**63 - 1 for _, _, _, ns in timed),
            'SENSOR_CLOCK_UNQUALIFIED', 'existing headless transport requires positive int64 nanoseconds')
    require(all(not e.get('delivery_delay_ns', 0) for _, e in selected),
            'OPENVINS_DELIVERY_UNSUPPORTED', 'headless scheduling cannot reproduce delivery-delay faults')
    cameras = [(i, e, ns) for i, e, _, ns in timed if e['kind'] == 'CAMERA']
    imus = [(i, e, ns) for i, e, _, ns in timed if e['kind'] == 'IMU']
    require(len(cameras) >= 2 and len(imus) >= 2,
            'SENSOR_STREAM_MISSING', 'at least two camera and IMU samples required')
    for stream in (cameras, imus):
        native_seconds = [float(ns) * 1e-9 for _, _, ns in stream]
        require(all(a < b for a, b in zip(native_seconds, native_seconds[1:])),
                'SENSOR_CLOCK_UNQUALIFIED', 'native double-second API collapses distinct source timestamps')
    require(not any(e['payload'].get('saturation') is True for _, e, _ in imus),
            'IMU_SATURATED', 'observed saturated samples cannot qualify this replay input')
    dimensions = {(e['payload']['width'], e['payload']['height']) for _, e, _ in cameras}
    require(len(dimensions) == 1, 'SENSOR_CALIBRATION_INVALID', 'one camera resolution per session')
    width, height = next(iter(dimensions))
    resolution = calibration['intrinsics'].get('resolution')
    require(isinstance(resolution, list) and len(resolution) == 2
            and all(type(value) is int for value in resolution) and resolution == [width, height],
            'SENSOR_CALIBRATION_INVALID', 'intrinsic calibration resolution must match recorded camera mode')
    offset = calibration['camera_imu_offset']['value_ns']
    require(imus[0][2] <= min(cameras[0][2], cameras[0][2] + offset)
            and imus[-1][2] > max(cameras[-1][2], cameras[-1][2] + offset),
            'IMU_COVERAGE_INVALID', 'real IMU samples must bracket original and calibrated camera endpoints')
    require(float(imus[-1][2]) * 1e-9 > float(cameras[-1][2]) * 1e-9 + offset / 1e9,
            'IMU_COVERAGE_INVALID', 'real IMU tail must bracket the endpoint in the native double-second API')
    native_tuning = copy.deepcopy(tuning)
    changes = {}
    for name, value in dict(relative_config_imu='imu.yaml', relative_config_imucam='imucam.yaml',
                            gravity_mag=parameters['gravity_mag']).items():
        if native_tuning.get(name) != value:
            changes[name] = dict(source=native_tuning.get(name), prepared=value)
        native_tuning[name] = value
    # Disabled path values are retained only inside the immutable source YAML.
    for name in ('filepath_est', 'filepath_std', 'filepath_gt'):
        if name in native_tuning:
            changes[name] = dict(source=native_tuning.pop(name), prepared='OMITTED_DISABLED')
    # The pinned parser requires this value even with timing output disabled.
    changes['record_timing_filepath'] = dict(source=native_tuning.get('record_timing_filepath'),
                                           prepared='disabled-timing.csv')
    native_tuning['record_timing_filepath'] = 'disabled-timing.csv'
    intrinsic = calibration['intrinsics']
    imucam = dict(cam0=dict(T_imu_cam=calibration['T_imu_camera']['matrix'], cam_overlaps=[],
        camera_model='pinhole', distortion_coeffs=intrinsic['parameters'][4:],
        distortion_model='radtan' if intrinsic['model'] == 'pinhole-radtan' else 'equidistant',
        intrinsics=intrinsic['parameters'][:4], resolution=[width, height],
        timeshift_cam_imu=offset / 1e9))
    imu_config = dict(imu0=copy.deepcopy(parameters['imu_parameters']))
    camera_csv = '#timestamp [ns],filename\n' + ''.join(f'{ns},{ns}.png\n' for _, _, ns in cameras)
    imu_csv = '#timestamp [ns],wx,wy,wz,ax,ay,az\n' + ''.join(
        str(ns) + ',' + ','.join(repr(float(v)) for v in
        e['payload']['gyro_xyz'] + e['payload']['accel_xyz']) + '\n' for _, e, ns in imus)
    lineage = []
    for index, event, _, ns in timed:
        row = dict(source_event_index=index, source_sensor_id=event['sensor_id'],
            source_frame_id=event['frame_id'], source_sequence=event['sequence'],
            source_epoch=event['epoch'], source_timestamp=event['timestamp'],
            mapped_timestamp=event.get('mapped_timestamp'), clock_mapping_id=event.get('clock_mapping_id'),
            native_stream='cam0' if event['kind'] == 'CAMERA' else 'imu0', native_timestamp_ns=ns)
        if event['kind'] == 'CAMERA':
            row['native_image_path'] = f'dataset/mav0/cam0/data/{ns}.png'
        lineage.append(row)
    generated = {'config/estimator_config.yaml': _yaml_text(native_tuning),
        'config/imu.yaml': _yaml_text(imu_config), 'config/imucam.yaml': _yaml_text(imucam),
        'dataset/mav0/cam0/data.csv': camera_csv, 'dataset/mav0/imu0/data.csv': imu_csv,
        'native-events.jsonl': ''.join(canonical(row) + '\n' for row in lineage)}
    shutters = sorted({e.get('exposure', {}).get('shutter', 'UNKNOWN') for _, e, _ in cameras})
    report = dict(schema_version=1, purpose='OPENVINS_SEALED_SENSOR_RECORDING_REPLAY',
        dataset='SENSOR_RECORDING', dataset_path='dataset', config_path='config/estimator_config.yaml',
        mode='mono_imu', source_epoch=source_epoch, camera_id=camera_id, imu_id=imu_id,
        clock_domain=timed[0][2], source_provenance=config['provenance'],
        counts=dict(cam0=len(cameras), imu0=len(imus)),
        camera_timestamps_ns=[ns for _, _, ns in cameras], imu_timestamps_ns=[ns for _, _, ns in imus],
        first_ns=dict(cam0=cameras[0][2], imu0=imus[0][2]),
        last_ns=dict(cam0=cameras[-1][2], imu0=imus[-1][2]),
        camera_resolution=[width, height], camera_imu_offset_ns=offset,
        source_sensor_timestamps_modified=False, native_clock='MAPPED_SOURCE_INTEGER_NS',
        native_time_convention='imu_time = mapped_camera_time + timeshift_cam_imu',
        native_estimator_time_representation='double_seconds; original/mapped integer ns retained in lineage',
        sequence_gaps=[g for g in validation['sequence_gaps'] if g['epoch'] == source_epoch],
        missing_sample_policy=validation['missing_sample_policy'], input_repair='NONE',
        tuning_changes=changes, shutter_models=shutters, row_readout_compensation=False,
        shutter_qualification='UNQUALIFIED_SINGLE_TIMESTAMP_FRAME_MODEL',
        physical_camera_imu_qualification=False, operational_validity='NOT_RUN',
        diagnostic_only=config['provenance']['kind'] == 'MOCK', truth_included=False,
        reference_scoring='NOT_RUN_NO_REFERENCE_SUPPLIED',
        timestamp_qualification='SOURCE_EVIDENCE_PRESERVED_NOT_PHYSICAL_SYNC_ACCEPTANCE',
        imu_saturation_unknown_samples=sum(e['payload'].get('saturation') is None for _, e, _ in imus))
    return generated, cameras, report


def prepare_recording(recording_directory, output, native_parameters, estimator_config, *, source_epoch=None):
    """Prepare a fresh, independently auditable mono input; never modify sources."""
    source, root = Path(recording_directory).resolve(), Path(output).resolve()
    tuning_path = Path(estimator_config).resolve()
    require(not root.exists(), 'OUTPUT_EXISTS', str(root))
    require(source not in root.parents, 'SENSOR_OUTPUT_INVALID', 'output must be outside recording')
    config, events, recording = sensors.load_recording(source)
    source_hash = file_hash(source/'recording.json')
    tuning_hash = file_hash(tuning_path)
    tuning_text = tuning_path.read_text()
    generated, cameras, report = _derive(config, events, native_parameters,
        _yaml_load(tuning_text), source_epoch, verify_files=True)
    root.mkdir(parents=True, exist_ok=False)
    try:
        for name, text in generated.items():
            path = root/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
        (root/'dataset/mav0/cam0/data').mkdir()
        for _, event, ns in cameras:
            shutil.copyfile(source/event['payload']['path'], root/f'dataset/mav0/cam0/data/{ns}.png')
        (root/'source').mkdir()
        for name in ('config.json', 'events.jsonl', 'recording.json'):
            shutil.copyfile(source/name, root/'source'/name)
        (root/'source/estimator_config.yaml').write_text(tuning_text)
        write_json(root/'source/native-parameters.json', native_parameters)
        sensors.load_recording(source)
        require(file_hash(source/'recording.json') == source_hash and file_hash(tuning_path) == tuning_hash,
                'HASH_MISMATCH', 'source changed during preparation')
        report.update(source_recording_sha256=source_hash, estimator_tuning_sha256=tuning_hash,
            native_parameters_sha256=file_hash(root/'source/native-parameters.json'),
            artifacts_sha256={str(p.relative_to(root)): file_hash(p)
                              for p in sorted(root.rglob('*')) if p.is_file()})
        write_json(root/'input.json', report)
        check_recording_input(root)
        return report
    except Exception:
        shutil.rmtree(root)
        raise


def check_recording_input(directory):
    """Verify hashes and independently rebuild native inputs from frozen sources."""
    root = Path(directory).resolve()
    report = read_json(root/'input.json')
    artifacts = report.get('artifacts_sha256', {})
    require(isinstance(artifacts, dict) and artifacts, 'SENSOR_RECORDING_INVALID', 'artifact hashes required')
    require(not any(p.is_symlink() for p in root.rglob('*')), 'SENSOR_RECORDING_INVALID', 'symlinks forbidden')
    actual = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p != root/'input.json'}
    require(actual == set(artifacts), 'SENSOR_RECORDING_INVALID', 'artifact membership differs')
    for name, digest in artifacts.items():
        require((root/name).resolve().is_relative_to(root) and sensors._sha(digest)
                and file_hash(root/name) == digest, 'HASH_MISMATCH', name)
    source = root/'source'
    config = read_json(source/'config.json')
    events = [json.loads(line) for line in (source/'events.jsonl').read_text().splitlines() if line.strip()]
    seal = read_json(source/'recording.json')
    require(file_hash(source/'recording.json') == report['source_recording_sha256']
            and file_hash(source/'estimator_config.yaml') == report['estimator_tuning_sha256']
            and file_hash(source/'native-parameters.json') == report['native_parameters_sha256'],
            'HASH_MISMATCH', 'source identities differ')
    require(seal.get('truth_included') is False and seal.get('provenance') == config.get('provenance')
            and seal['artifacts_sha256']['config.json'] == file_hash(source/'config.json')
            and seal['artifacts_sha256']['events.jsonl'] == file_hash(source/'events.jsonl')
            and seal.get('validation') == sensors.validate_events(events, config),
            'SENSOR_RECORDING_INVALID', 'source seal disagrees with copied events/config')
    generated, cameras, derived = _derive(config, events, read_json(source/'native-parameters.json'),
        _yaml_load((source/'estimator_config.yaml').read_text()), report.get('source_epoch'), verify_files=False)
    identity_keys = {'source_recording_sha256', 'estimator_tuning_sha256',
                     'native_parameters_sha256', 'artifacts_sha256'}
    require(set(report) == set(derived) | identity_keys
            and all(report.get(k) == v for k, v in derived.items()),
            'SENSOR_RECORDING_INVALID', 'input manifest differs from source-derived semantics')
    expected_artifacts = set(generated) | {'source/config.json', 'source/events.jsonl',
        'source/recording.json', 'source/estimator_config.yaml', 'source/native-parameters.json'}
    for name, text in generated.items():
        require((root/name).read_text() == text, 'SENSOR_INPUT_MISMATCH', name)
    for _, event, ns in cameras:
        name = f'dataset/mav0/cam0/data/{ns}.png'
        expected_artifacts.add(name)
        payload = event['payload']
        require(seal['artifacts_sha256'].get(payload['path']) == payload['encoded_sha256']
                and file_hash(root/name) == payload['encoded_sha256'], 'HASH_MISMATCH', name)
        pixels = cv2.imread(str(root/name), cv2.IMREAD_UNCHANGED)
        require(pixels is not None and pixels.dtype == np.uint8
                and pixels.shape == (payload['height'], payload['width'])
                and hashlib.sha256(pixels.tobytes()).hexdigest() == payload['pixels_sha256'],
                'SENSOR_IMAGE_INVALID', 'native pixels differ from source')
    require(actual == expected_artifacts, 'SENSOR_RECORDING_INVALID', 'unexpected native artifact')
    return dict(status='PASS', readback_exact=True, truth_included=False,
                physical_camera_imu_qualification=False, counts=derived['counts'])
