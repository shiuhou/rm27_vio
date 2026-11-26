"""Shared-recording intake behavior; these fixtures do not execute OpenVINS."""
import copy
import json

import cv2
import numpy as np
import pytest
import yaml

from rm27.perception.vision.localization.experiments import sensors
from rm27.perception.vision.localization.experiments.common import (
    file_hash, object_hash, read_json, write_json,
)
from tools.openvins.recording import prepare_recording, check_recording_input


@pytest.fixture
def native_recording(tmp_path):
    raw = tmp_path/'raw'
    sensors.mock_recording(raw, camera_count=2, selected_camera_id='down', frame_count=8)
    config, events, _ = sensors.load_recording(raw)
    evidence_path = tmp_path/'mock-source-evidence.json'
    evidence_path.write_text('{"purpose":"EXPLICIT_MOCK_ONLY_NOT_HARDWARE"}\n')
    evidence = dict(status='CODE_SUPPORTED', source_reference=str(evidence_path),
                    source_sha256=file_hash(evidence_path))
    camera = config['cameras'][1]
    transform = np.eye(4)
    transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    transform[:3, 3] = [.01, .02, .03]
    camera['calibration'] = dict(version='EXPLICIT_MOCK_CALIBRATION',
        intrinsics=dict(evidence, model='pinhole-radtan', resolution=[32, 24],
                        parameters=[20, 21, 16, 12, .1, .2, .3, .4]),
        T_imu_camera=dict(evidence, convention='camera_to_imu', matrix=transform.tolist()),
        camera_imu_offset=dict(evidence, value_ns=2_000_000,
                              convention='imu_time_equals_mapped_camera_time_plus_offset'))
    # Retain an actual post-camera IMU sample rather than manufacturing padding.
    events = events[:-1]
    for event in events:
        event['sequence'] += 10
    parameters = dict(schema_version=1, purpose='OPENVINS_RECORDING_PARAMETERS',
        measurement_binding=dict(evidence, imu_id='imu0', imu_frame='imu0',
            source_imu_config_sha256=object_hash(config['imu']),
            measurement_kind='RAW_SPECIFIC_FORCE', native_axes_transform='IDENTITY'),
        imu_parameters=dict(gyroscope_noise_density=.001, gyroscope_random_walk=.002,
            accelerometer_noise_density=.01, accelerometer_random_walk=.02,
            update_rate=20., model='kalibr', Tw=np.eye(3).tolist(), Ta=np.eye(3).tolist(),
            R_IMUtoACC=np.eye(3).tolist(), R_IMUtoGYRO=np.eye(3).tolist(), Tg=np.zeros((3, 3)).tolist()),
        gravity_mag=9.81, gravity_evidence=evidence)
    tuning = dict(max_cameras=1, use_stereo=False, multi_threading_subs=False,
        multi_threading_pubs=False, record_timing_information=False, save_total_state=False,
        use_aruco=False, use_mask=False, downsample_cameras=False, gravity_mag=1.,
        relative_config_imu='/old/imu.yaml', relative_config_imucam='/old/camera.yaml',
        record_timing_filepath='/old/timing.csv', filepath_est='/old/output.txt', num_pts=200)
    tuning_path = tmp_path/'tuning.yaml'
    tuning_path.write_text('%YAML:1.0\n' + yaml.safe_dump(tuning))
    return raw, config, events, parameters, tuning_path


def _seal(fixture, directory):
    raw, config, events, parameters, tuning = fixture
    sensors.write_recording(directory, config, events, raw)
    return directory, parameters, tuning


def test_native_package_roundtrip_preserves_pixels_si_ids_transforms_and_no_truth(native_recording, tmp_path):
    source, parameters, tuning = _seal(native_recording, tmp_path/'sealed')
    original_hash = file_hash(source/'recording.json')
    target = tmp_path/'prepared'
    report = prepare_recording(source, target, parameters, tuning)
    assert check_recording_input(target) == dict(status='PASS', readback_exact=True,
        truth_included=False, physical_camera_imu_qualification=False, counts=dict(cam0=7, imu0=8))
    assert report['dataset'] == 'SENSOR_RECORDING'
    assert report['camera_id'] == 'down' and report['source_epoch'] == 0
    assert report['diagnostic_only'] is True
    assert report['camera_imu_offset_ns'] == 2_000_000
    assert report['row_readout_compensation'] is False
    assert report['reference_scoring'] == 'NOT_RUN_NO_REFERENCE_SUPPLIED'
    lineage = [json.loads(s) for s in (target/'native-events.jsonl').read_text().splitlines()]
    assert lineage[1]['source_sequence'] == 10
    assert lineage[1]['source_sensor_id'] == 'down'
    assert lineage[1]['native_timestamp_ns'] > 2**53
    assert (target/'source/events.jsonl').read_bytes() == (source/'events.jsonl').read_bytes()
    for name in ('estimator_config.yaml', 'imucam.yaml', 'imu.yaml'):
        storage = cv2.FileStorage(str(target/'config'/name), cv2.FILE_STORAGE_READ)
        assert storage.isOpened(), name
        storage.release()
    # Independent FileStorage readback verifies native YAML signs and matrix direction.
    storage = cv2.FileStorage(str(target/'config/imucam.yaml'), cv2.FILE_STORAGE_READ)
    camera = storage.getNode('cam0')
    assert camera.getNode('timeshift_cam_imu').real() == .002
    transform = camera.getNode('T_imu_cam')
    assert [[transform.at(i).at(j).real() for j in range(4)] for i in range(4)] == native_recording[1]['cameras'][1]['calibration']['T_imu_camera']['matrix']
    storage.release()
    imu_rows = (target/'dataset/mav0/imu0/data.csv').read_text().splitlines()[1:]
    assert list(map(float, imu_rows[0].split(',')[1:])) == [0., 0., 0., 0., 0., 9.81]
    assert file_hash(source/'recording.json') == original_hash
    with pytest.raises(ValueError, match='OUTPUT_EXISTS'):
        prepare_recording(source, target, parameters, tuning)


@pytest.mark.parametrize('change,match', [
    ('unknown-calibration', 'SENSOR_CALIBRATION_UNQUALIFIED'),
    ('binding-frame', 'SENSOR_AXES_INVALID'), ('evidence-hash', 'HASH_MISMATCH'),
    ('missing-parameters', 'NATIVE_PARAMETERS_INVALID'), ('distortion-count', 'SENSOR_CALIBRATION_INVALID'),
    ('resolution', 'SENSOR_CALIBRATION_INVALID'),
    ('negative-noise', 'NATIVE_PARAMETERS_INVALID'), ('imu-tail', 'IMU_COVERAGE_INVALID'),
    ('delivery-delay', 'OPENVINS_DELIVERY_UNSUPPORTED'), ('saturation', 'IMU_SATURATED'),
    ('clock-receipt', 'SENSOR_CLOCK_UNQUALIFIED'), ('dump', 'OPENVINS_CONFIG_UNSUPPORTED'),
    ('threading', 'OPENVINS_CONFIG_UNSUPPORTED'), ('downsample', 'OPENVINS_CONFIG_UNSUPPORTED'),
])
def test_preflight_rejects_unqualified_inputs_without_creating_output(native_recording, tmp_path, change, match):
    raw, config, events, parameters, tuning = native_recording
    if change == 'unknown-calibration':
        config['cameras'][1]['calibration']['intrinsics'] = dict(status='UNKNOWN')
    elif change == 'binding-frame':
        parameters['measurement_binding']['imu_frame'] = 'another-frame'
    elif change == 'evidence-hash':
        parameters['measurement_binding']['source_sha256'] = '0'*64
    elif change == 'missing-parameters':
        del parameters['imu_parameters']['gyroscope_noise_density']
    elif change == 'distortion-count':
        config['cameras'][1]['calibration']['intrinsics']['parameters'].append(0)
    elif change == 'resolution':
        config['cameras'][1]['calibration']['intrinsics']['resolution'] = [24, 32]
    elif change == 'negative-noise':
        parameters['imu_parameters']['accelerometer_random_walk'] = -.1
    elif change == 'imu-tail':
        events.pop()
    elif change == 'delivery-delay':
        events[1]['delivery_delay_ns'] = 1
    elif change == 'saturation':
        events[0]['payload']['saturation'] = True
    elif change == 'clock-receipt':
        config['cameras'][1]['timestamp_contract']['semantic'] = 'HOST_RECEIPT'
        for event in events:
            if event['kind'] == 'CAMERA':
                event['timestamp']['semantic'] = 'HOST_RECEIPT'
    else:
        text = yaml.safe_load('\n'.join(tuning.read_text().splitlines()[1:]))
        text[{'dump':'save_total_state', 'threading':'multi_threading_subs',
              'downsample':'downsample_cameras'}[change]] = True
        tuning.write_text('%YAML:1.0\n' + yaml.safe_dump(text))
    source, parameters, tuning = _seal(native_recording, tmp_path/'sealed')
    target = tmp_path/'refused'
    with pytest.raises(ValueError, match=match):
        prepare_recording(source, target, parameters, tuning)
    assert not target.exists()


def test_epoch_selection_and_sequence_gaps_are_preserved(native_recording, tmp_path):
    raw, config, events, parameters, tuning = native_recording
    second = copy.deepcopy(events)
    for event in second:
        event['epoch'] = 4
        event['timestamp']['raw_value'] += 1_000_000_000
        event['sequence'] += 20
    # A counter gap remains evidence; no timestamp or source ID is repaired.
    second[-1]['sequence'] += 2
    sensors.write_recording(tmp_path/'sealed', config, events + second, raw)
    with pytest.raises(ValueError, match='SENSOR_EPOCH_INVALID'):
        prepare_recording(tmp_path/'sealed', tmp_path/'ambiguous', parameters, tuning)
    report = prepare_recording(tmp_path/'sealed', tmp_path/'selected', parameters, tuning, source_epoch=4)
    assert report['source_epoch'] == 4 and report['counts'] == dict(cam0=7, imu0=8)
    assert report['sequence_gaps'][0]['unrepresented_sequence_values'] == 2
    assert report['sequence_gaps'][0]['missing_sample_count'] is None
    assert report['input_repair'] == 'NONE'


def test_public_observed_time_is_preserved_but_hardware_cannot_reuse_it(native_recording, tmp_path):
    raw, config, events, parameters, tuning = native_recording
    config['provenance']['kind'] = 'PUBLIC_DATASET'
    for sensor in [config['imu'], *config['cameras']]:
        sensor['timestamp_contract'].update(semantic='DATASET_RECORDED_TIME', evidence_status='OBSERVED')
    for event in events:
        event['timestamp'].update(semantic='DATASET_RECORDED_TIME', evidence_status='OBSERVED')
    parameters['measurement_binding']['source_imu_config_sha256'] = object_hash(config['imu'])
    source, parameters, tuning = _seal(native_recording, tmp_path/'public')
    report = prepare_recording(source, tmp_path/'public-input', parameters, tuning)
    assert report['diagnostic_only'] is False
    assert report['physical_camera_imu_qualification'] is False
    assert read_json(tmp_path/'public-input/source/config.json')['imu']['timestamp_contract']['evidence_status'] == 'OBSERVED'
    config['provenance']['kind'] = 'HARDWARE_RECORDING'
    source, parameters, tuning = _seal(native_recording, tmp_path/'hardware')
    with pytest.raises(ValueError, match='SENSOR_CLOCK_UNQUALIFIED'):
        prepare_recording(source, tmp_path/'hardware-input', parameters, tuning)


@pytest.mark.parametrize('target_name', ['dataset/mav0/imu0/data.csv', 'config/imucam.yaml',
                                        'native-events.jsonl'])
def test_rehashed_native_input_tampering_is_rejected(native_recording, tmp_path, target_name):
    source, parameters, tuning = _seal(native_recording, tmp_path/'sealed')
    target = tmp_path/'prepared'
    prepare_recording(source, target, parameters, tuning)
    path = target/target_name
    path.write_text(path.read_text() + '\n')
    report = read_json(target/'input.json')
    report['artifacts_sha256'][target_name] = file_hash(path)
    write_json(target/'input.json', report)
    with pytest.raises(ValueError, match='SENSOR_INPUT_MISMATCH'):
        check_recording_input(target)


def test_corrupt_or_reordered_sealed_source_is_not_repaired(native_recording, tmp_path):
    source, parameters, tuning = _seal(native_recording, tmp_path/'sealed')
    path = source/'events.jsonl'
    rows = path.read_text().splitlines()
    rows[0], rows[2] = rows[2], rows[0]
    path.write_text('\n'.join(rows) + '\n')
    seal = read_json(source/'recording.json')
    seal['artifacts_sha256']['events.jsonl'] = file_hash(path)
    write_json(source/'recording.json', seal)
    with pytest.raises(ValueError, match='SENSOR_ORDER_INVALID'):
        prepare_recording(source, tmp_path/'refused', parameters, tuning)
    assert not (tmp_path/'refused').exists()


def test_mapped_clock_replay_retains_original_stamps_and_mapping_ids(native_recording, tmp_path):
    raw, config, events, parameters, tuning = native_recording
    origin = events[0]['timestamp']['raw_value']
    selected = [config['imu'], config['cameras'][1]]
    for sensor in selected:
        identity = sensor['sensor_id']
        sensor['timestamp_contract']['clock_domain'] = identity + ':raw-clock'
        config['clock_mappings'].append(dict(mapping_id=identity + ':mapping',
            source_clock=identity + ':raw-clock', target_clock='shared:mapped-clock',
            source_anchor_ns=origin, target_anchor_ns=origin + 7,
            rate_numerator=1, rate_denominator=1, uncertainty_ns=1,
            evidence_status='CODE_SUPPORTED', source_sha256=parameters['gravity_evidence']['source_sha256']))
    for event in events:
        identity = event['sensor_id']
        event['timestamp']['clock_domain'] = identity + ':raw-clock'
        event['clock_mapping_id'] = identity + ':mapping'
        event['mapped_timestamp'] = dict(raw_value=event['timestamp']['raw_value'] + 7,
            unit='ns', clock_domain='shared:mapped-clock', semantic='MAPPED_SENSOR_TIME',
            evidence_status='CODE_SUPPORTED')
    parameters['measurement_binding']['source_imu_config_sha256'] = object_hash(config['imu'])
    source, parameters, tuning = _seal(native_recording, tmp_path/'mapped')
    target = tmp_path/'prepared'
    report = prepare_recording(source, target, parameters, tuning)
    assert report['clock_domain'] == 'shared:mapped-clock'
    assert report['camera_timestamps_ns'][0] == origin + 7
    lineage = [json.loads(s) for s in (target/'native-events.jsonl').read_text().splitlines()]
    assert lineage[0]['source_timestamp']['raw_value'] == origin
    assert lineage[0]['mapped_timestamp']['raw_value'] == origin + 7
    assert lineage[0]['clock_mapping_id'] == 'imu0:mapping'


def test_hardware_imu_cannot_claim_an_exposure_timestamp(native_recording, tmp_path):
    raw, config, events, parameters, tuning = native_recording
    config['provenance']['kind'] = 'HARDWARE_RECORDING'
    for sensor in [config['imu'], *config['cameras']]:
        sensor['timestamp_contract'].update(evidence_status='VERIFIED', semantic='EXPOSURE_START')
    for event in events:
        event['timestamp'].update(evidence_status='VERIFIED', semantic='EXPOSURE_START')
    source, parameters, tuning = _seal(native_recording, tmp_path/'hardware')
    with pytest.raises(ValueError, match='SENSOR_CLOCK_UNQUALIFIED'):
        prepare_recording(source, tmp_path/'refused', parameters, tuning)


def test_native_double_timestamp_collisions_are_rejected_without_time_shift(native_recording, tmp_path):
    raw, config, events, parameters, tuning = native_recording
    camera_events = [e for e in events if e['kind'] == 'CAMERA']
    camera_events[1]['timestamp']['raw_value'] = camera_events[0]['timestamp']['raw_value'] + 1
    source, parameters, tuning = _seal(native_recording, tmp_path/'collision')
    with pytest.raises(ValueError, match='native double-second API collapses'):
        prepare_recording(source, tmp_path/'refused', parameters, tuning)


def test_native_parser_requires_unambiguous_tuning_keys(native_recording, tmp_path):
    source, parameters, tuning = _seal(native_recording, tmp_path/'sealed')
    tuning.write_text(tuning.read_text() + '\nnum_pts: 150\n')
    with pytest.raises(ValueError, match='YAML mapping keys must be unique'):
        prepare_recording(source, tmp_path/'refused', parameters, tuning)
