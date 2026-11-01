"""Synthetic transport contract tests; no estimator or sensor qualification."""
import hashlib
import json
import zipfile
import copy

import cv2
import numpy as np
import pytest
import yaml

from rm27.perception.vision.localization.experiments.common import file_hash
from tools.schurvins.dataset import prepare, prepare_uzh, reconcile
from tools.schurvins.ros_transport import inputs, rows, scheduled_events, uzh_rows


BASE_NS = 1403715273262142976
CAMERA_NS = [BASE_NS, BASE_NS + 50_000_000, BASE_NS + 100_000_000]


def source_zip(path, *, right_stamps=None, duplicate=False):
    """Small real PNG/CSV archive with deliberately ineligible synthetic truth."""
    pixels = {}
    with zipfile.ZipFile(path, 'w') as archive:
        for sensor in ['cam0', 'cam1']:
            stamps = CAMERA_NS if sensor == 'cam0' or right_stamps is None else right_stamps
            archive.writestr('mav0/' + sensor + '/data.csv',
                             '#timestamp [ns],filename\n' + ''.join(
                                 f'{ns},{ns}.png\n' for ns in stamps))
            archive.writestr('mav0/' + sensor + '/sensor.yaml',
                             'purpose: TEST_FIXTURE_ONLY\nsensor: ' + sensor + '\n')
            for index, ns in enumerate(stamps):
                image = np.arange(56, dtype=np.uint8).reshape(7, 8) + index
                if sensor == 'cam1':
                    image = np.flip(image, axis=1).copy()
                ok, encoded = cv2.imencode('.png', image)
                assert ok
                archive.writestr(f'mav0/{sensor}/data/{ns}.png', encoded.tobytes())
                pixels[(sensor, ns)] = image
        imu_csv = '#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n' + ''.join(
            f'{ns},0.1,-0.2,0.3,1.0,2.0,9.8\n' for ns in CAMERA_NS)
        archive.writestr('mav0/imu0/data.csv', imu_csv)
        archive.writestr('mav0/imu0/sensor.yaml', 'purpose: TEST_FIXTURE_ONLY\n')
        archive.writestr('mav0/state_groundtruth_estimate0/data.csv',
                         'SYNTHETIC TEST TRUTH: MUST NEVER ENTER SENSOR INPUT\n')
        archive.writestr('irrelevant.txt', 'TEST_FIXTURE_ONLY')
        if duplicate:
            with pytest.warns(UserWarning, match='Duplicate name'):
                archive.writestr('mav0/imu0/data.csv', imu_csv)
    return pixels


def prepared(tmp_path, mode='mono'):
    archive = tmp_path/'source.zip'
    pixels = source_zip(archive)
    output = tmp_path/'input'
    report = prepare(archive, output, mode)
    return archive, output, report, pixels


def raw_evidence(directory, raw):
    expected = [json.loads(line) for line in (directory/'expected.jsonl').read_text().splitlines()]
    raw.mkdir()
    published = sorted(expected, key=lambda row: (row['state_ns'], row['stream'] != 'imu0'))
    (raw/'published.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in published))
    for sensor in {r['stream'] for r in expected}:
        observed = [{**row, 'observed_monotonic_ns': 1234}
                    for row in expected if row['stream'] == sensor]
        (raw/(sensor+'.jsonl')).write_text(''.join(json.dumps(row)+'\n' for row in observed))
    return expected


def test_csv_preserves_integer_nanoseconds_above_double_precision():
    result = rows(f'{BASE_NS},0,0,0,0,0,9.8\n{BASE_NS+1},0,0,0,0,0,9.8\n', 'imu0')
    assert [stamp for stamp, _ in result] == [BASE_NS, BASE_NS+1]
    assert result[1][0] - result[0][0] == 1


def test_delivery_delay_changes_only_delivery_and_keeps_original_sensing_time():
    data = dict(imu0=[(BASE_NS, [0]*6), (BASE_NS+50_000_000, [0]*6)],
                cam0=[(BASE_NS, 'first.png'), (BASE_NS+50_000_000, 'second.png')])
    schedule = dict(schema_version=1, time_semantics='DELIVERY_DELAY_ONLY_SOURCE_STAMP_UNCHANGED',
        entries=[dict(stream='cam0', state_ns=BASE_NS, source_sequence=0, delivery_delay_ns=100_000_000)])
    events = scheduled_events(data, 1., delivery_schedule=schedule)
    assert [row[1] for row in events if row[2] == 'cam0'] == [BASE_NS+50_000_000, BASE_NS]
    assert events[-1] == (100_000_000, BASE_NS, 'cam0', 'first.png')
    assert data['cam0'][0][0] == BASE_NS


def test_start_offset_is_explicit_prefix_exclusion_without_timestamp_rewrite():
    data = dict(imu0=[(BASE_NS, []), (BASE_NS+10_000_000_001, [])],
                cam0=[(BASE_NS+1, 'a.png'), (BASE_NS+10_000_000_002, 'b.png')])
    events = scheduled_events(data, .5, 10)
    assert events == [(2, BASE_NS+10_000_000_001, 'imu0', []),
                      (4, BASE_NS+10_000_000_002, 'cam0', 'b.png')]
    with pytest.raises(ValueError, match='empty sensor'):
        scheduled_events(data, .5, 20)


def test_uzh_decimal_time_quantization_preserves_source_id_order_and_image_path():
    rows = uzh_rows('7 1540821358.195877075195 img/image_0_7.png\n'
                    '8 1540821358.195877076500 img/image_0_8.png\n', 'cam0')
    assert rows == [(1540821358195877075, 'img/image_0_7.png'),
                    (1540821358195877076, 'img/image_0_8.png')]
    with pytest.raises(ValueError, match='IDs/timestamps'):
        uzh_rows('7 1.0 img/image_0_7.png\n7 2.0 img/image_0_8.png', 'cam0')


@pytest.mark.parametrize('text, message', [
    ('1,0,0,0,0,0,0\n1,0,0,0,0,0,0', 'strictly increase'),
    ('2,0,0,0,0,0,0\n1,0,0,0,0,0,0', 'strictly increase'),
    ('-1,0,0,0,0,0,0', 'strictly increase'),
    ('1,nan,0,0,0,0,0', 'nonfinite'),
    ('1,0,0,inf,0,0,0', 'nonfinite'),
    ('1,0,0,0,0,0', 'column count'),
    ('# no samples\n', 'empty'),
])
def test_imu_csv_rejects_bad_order_counts_and_nonfinite_values(text, message):
    with pytest.raises(ValueError, match=message):
        rows(text, 'imu0')


def test_camera_csv_rejects_filename_timestamp_disagreement():
    with pytest.raises(ValueError, match='filename'):
        rows(f'{BASE_NS},{BASE_NS+1}.png\n', 'cam0')


def test_stereo_requires_exact_timestamps_and_mono_does_not_consume_cam1(tmp_path):
    archive = tmp_path/'source.zip'
    source_zip(archive, right_stamps=[ns+1 for ns in CAMERA_NS])
    with zipfile.ZipFile(archive) as source:
        assert set(inputs(source, 'mono')) == {'imu0', 'cam0'}
        with pytest.raises(ValueError, match='exactly equal'):
            inputs(source, 'stereo')
    with pytest.raises(ValueError, match='exactly equal'):
        prepare(archive, tmp_path/'input', 'stereo')
    assert not (tmp_path/'input').exists()


@pytest.mark.parametrize('mode', ['mono', 'stereo'])
def test_sensor_export_preserves_pixels_imu_and_calibration_but_excludes_truth(tmp_path, mode):
    archive, output, report, pixels = prepared(tmp_path, mode)
    assert report['original_archive_sha256'] == file_hash(archive)
    assert report['truth_included'] is False
    assert report['counts'] == {'imu0': 3, 'cam0': 3, **({'cam1': 3} if mode == 'stereo' else {})}
    assert report['first_ns']['cam0'] == CAMERA_NS[0]
    assert report['last_ns']['cam0'] == CAMERA_NS[-1]
    expected = [json.loads(line) for line in (output/'expected.jsonl').read_text().splitlines()]
    with zipfile.ZipFile(output/'sensors.zip') as frozen, zipfile.ZipFile(archive) as source:
        assert not any('groundtruth' in name or name == 'irrelevant.txt' for name in frozen.namelist())
        assert ('mav0/cam1/data.csv' in frozen.namelist()) == (mode == 'stereo')
        for name in frozen.namelist():
            assert frozen.read(name) == source.read(name)
        for sensor, checksum in report['calibration_sha256'].items():
            assert hashlib.sha256(frozen.read(f'mav0/{sensor}/sensor.yaml')).hexdigest() == checksum
    for row in expected:
        if row['stream'] == 'imu0':
            assert row['gyro'] == [.1, -.2, .3]
            assert row['accel'] == [1., 2., 9.8]
        else:
            image = pixels[row['stream'], row['state_ns']]
            assert row['pixels_sha256'] == hashlib.sha256(image.tobytes()).hexdigest()
            assert (row['height'], row['width'], row['step'], row['encoding']) == (7, 8, 8, 'mono8')
    assert {name: file_hash(output/name) for name in report['artifacts_sha256']} == report['artifacts_sha256']


def uzh_calibration(path):
    T_cam_imu = np.eye(4)
    T_cam_imu[:3, 3] = [.1, -.2, .3]
    camera = dict(camera_model='pinhole', distortion_model='equidistant', intrinsics=[280., 281., 320., 240.],
        distortion_coeffs=[-.01, .02, -.03, .004], resolution=[32, 24], T_cam_imu=T_cam_imu.tolist(),
        timeshift_cam_imu=-.016)
    other = copy.deepcopy(camera)
    other['intrinsics'][0] = 282.
    imu = dict(gyroscope_noise_density=.05, accelerometer_noise_density=.1,
        gyroscope_random_walk=.00004, accelerometer_random_walk=.002, update_rate=500.)
    with zipfile.ZipFile(path,'w') as archive:
        archive.writestr('calib/camchain-imucam-source.yaml',yaml.safe_dump(dict(cam0=camera,cam1=other)))
        archive.writestr('calib/imu-source.yaml',yaml.safe_dump(dict(imu0=imu)))
        archive.writestr('calib/results-imucam-source.txt','T_ci: (imu0 to cam0)\n(t_imu = t_cam + shift)\n')
    return camera


@pytest.mark.parametrize('camera_id, policy', [('cam0','source_calibration'),('cam1','not_applied')])
def test_uzh_native_intake_keeps_source_format_timing_and_calibration(tmp_path, camera_id, policy):
    from test_sensor_inputs import uzh_archive
    from rm27.perception.vision.localization.experiments import sensors
    archive, cal, template = tmp_path/'uzh.zip', tmp_path/'cal.zip', tmp_path/'template.yaml'
    uzh_archive(archive)
    source_camera = uzh_calibration(cal)
    template.write_text(yaml.safe_dump(dict(imu_params=dict(g=9.81, acc_max=150., omega_max=20.,
        max_imu_delta_t=.01, sigma_integration=0.), imu_initialization=dict(velocity=[0.,0.,0.],
        omega_bias=[0.,0.,0.], acc_bias=[0.,0.,0.], velocity_sigma=2.,omega_bias_sigma=.01,acc_bias_sigma=.1))))
    recording = tmp_path/'recording'
    sensors.import_dataset(archive,recording,'UZH_FPV',camera_id=camera_id)
    output = tmp_path/'native'
    report = prepare_uzh(archive,cal,output,camera_id,recording_directory=recording,
        native_template=template,time_shift_policy=policy)
    assert report['purpose'] == 'UZH_FPV_PUBLIC_REPLAY' and report['dataset'] == 'UZH_FPV'
    assert report['truth_included'] is False
    assert report['native_camera_id'] == 'cam0' and report['source_camera_id'] == camera_id
    native = yaml.safe_load((output/'native_calibration.yaml').read_text())
    assert native['cameras'][0]['camera']['distortion']['type'] == 'equidistant'
    assert native['cameras'][0]['camera']['intrinsics']['data'][0] == (280. if camera_id=='cam0' else 282.)
    T_B_C = np.asarray(native['cameras'][0]['T_B_C']['data']).reshape(4,4)
    assert np.allclose(T_B_C @ source_camera['T_cam_imu'], np.eye(4))
    assert native['imu_params']['sigma_acc_c'] == .1
    assert native['imu_params']['sigma_omega_c'] == .05
    assert native['imu_params']['imu_rate'] == 500.
    assert native['imu_params']['delay_imu_cam'] == (.016 if policy == 'source_calibration' else 0.)
    assert native['imu_params']['g'] == 9.81
    assert 'line-delay-nanoseconds' not in native['cameras'][0]['camera']
    calibration = json.loads((output/'calibration.json').read_text())
    assert calibration['timing_application_verified'] is False
    assert calibration['experiment_template']['qualification'] == 'EXPERIMENT_CONFIG'
    with zipfile.ZipFile(output/'sensors.zip') as frozen, zipfile.ZipFile(archive) as original:
        assert not any('mav0' in n or 'truth' in n for n in frozen.namelist())
        assert 'imu.txt' in frozen.namelist()
        assert ('left_images.txt' if camera_id=='cam0' else 'right_images.txt') in frozen.namelist()
        for name in frozen.namelist():
            assert frozen.read(name) == original.read(name)
    expected = [json.loads(line) for line in (output/'expected.jsonl').read_text().splitlines()]
    assert {e['stream'] for e in expected} == {'imu0','cam0'}
    sources = [json.loads(line) for line in (output/'source-events.jsonl').read_text().splitlines()]
    assert sources[0]['timestamp']['raw_value'] == 1540821358195877075195
    assert sources[0]['source_reference']['timestamp_text'] == '1540821358.195877075195'
    assert expected[0]['state_ns'] == 1540821358195877075
    assert {name:file_hash(output/name) for name in report['artifacts_sha256']} == report['artifacts_sha256']
    raw = tmp_path/'raw'
    raw_evidence(output,raw)
    assert reconcile(output,raw)['cam0']['content_exact'] is True


def test_uzh_geometry_without_template_is_explicitly_not_native_complete(tmp_path):
    from test_sensor_inputs import uzh_archive
    archive, cal = tmp_path/'uzh.zip', tmp_path/'cal.zip'
    uzh_archive(archive)
    uzh_calibration(cal)
    output = tmp_path/'native'
    report = prepare_uzh(archive,cal,output)
    assert report['native_imu_config_complete'] is False
    native = yaml.safe_load((output/'native_calibration.yaml').read_text())
    assert 'g' not in native['imu_params'] and 'imu_initialization' not in native
    before = file_hash(output/'input.json')
    with pytest.raises(FileExistsError):
        prepare_uzh(archive,cal,output)
    assert file_hash(output/'input.json') == before


def test_prepare_refuses_existing_output_without_touching_evidence(tmp_path):
    archive, output, _, _ = prepared(tmp_path)
    before = {p.name: p.read_bytes() for p in output.iterdir()}
    with pytest.raises(FileExistsError):
        prepare(archive, output, 'mono')
    assert {p.name: p.read_bytes() for p in output.iterdir()} == before


def test_prepare_refuses_duplicate_zip_members_before_creating_output(tmp_path):
    archive = tmp_path/'duplicate.zip'
    source_zip(archive, duplicate=True)
    with pytest.raises(ValueError, match='duplicate ZIP'):
        prepare(archive, tmp_path/'input', 'mono')
    assert not (tmp_path/'input').exists()


def test_reconciliation_checks_publisher_and_observer_without_claiming_native_ingestion(tmp_path):
    _, directory, _, _ = prepared(tmp_path, 'stereo')
    raw = tmp_path/'raw'
    raw_evidence(directory, raw)
    result = reconcile(directory, raw)
    assert set(result) == {'cam0', 'cam1', 'imu0'}
    for entry in result.values():
        assert entry == {'published': 3, 'observed': 3, 'content_exact': True,
                         'native_imu_ingestion_verified': False}


@pytest.mark.parametrize('file, change', [
    ('published.jsonl', 'missing'), ('imu0.jsonl', 'missing'),
    ('published.jsonl', 'reordered'), ('cam0.jsonl', 'reordered'),
    ('cam0.jsonl', 'pixels'), ('imu0.jsonl', 'values'),
])
def test_reconciliation_rejects_missing_reordered_or_changed_evidence(tmp_path, file, change):
    _, directory, _, _ = prepared(tmp_path)
    raw = tmp_path/'raw'
    raw_evidence(directory, raw)
    path = raw/file
    data = [json.loads(line) for line in path.read_text().splitlines()]
    if change == 'missing':
        data.pop()
    elif change == 'reordered':
        data.reverse()
    elif change == 'pixels':
        data[0]['pixels_sha256'] = '0'*64
    else:
        data[0]['gyro'][1] = 100.
    path.write_text(''.join(json.dumps(row)+'\n' for row in data))
    with pytest.raises(ValueError, match='mismatch'):
        reconcile(directory, raw)
