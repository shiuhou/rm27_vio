"""Synthetic transport contract tests; no estimator or sensor qualification."""
import hashlib
import json
import zipfile
import copy
import ctypes

import cv2
import numpy as np
import pytest
import yaml

from rm27.perception.vision.localization.experiments.common import file_hash
from tools.schurvins.dataset import prepare, prepare_uzh, reconcile
from tools.schurvins.ros_transport import inputs, rows, scheduled_events, uzh_rows
from tools.schurvins import ros_transport


BASE_NS = 1403715273262142976
CAMERA_NS = [BASE_NS, BASE_NS + 50_000_000, BASE_NS + 100_000_000]


def test_monotonic_clock_reuses_typed_function_and_preserves_integer_nanoseconds(monkeypatch):
    calls=[]
    def clock(clock_id, pointer):
        value=ctypes.cast(pointer,ctypes.POINTER(ros_transport.Timespec)).contents
        value.tv_sec=1790000000
        value.tv_nsec=27+len(calls)
        calls.append(clock_id)
        return 0
    monkeypatch.setattr(ros_transport,'_clock_gettime',clock)
    monkeypatch.setattr(ros_transport.ctypes,'CDLL',lambda *a,**k:pytest.fail('library must not reopen per clock read'))
    assert ros_transport.monotonic_ns()==1790000000000000027
    assert ros_transport.monotonic_ns()==1790000000000000028
    assert calls==[1,1]
    monkeypatch.setattr(ros_transport,'_clock_gettime',lambda *a:-1)
    with pytest.raises(RuntimeError,match='MONOTONIC'):ros_transport.monotonic_ns()


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


@pytest.fixture
def sealed_native_recording(tmp_path):
    from rm27.perception.vision.localization.experiments import sensors
    raw=tmp_path/'raw-recording'
    sensors.mock_recording(raw,camera_count=2,selected_camera_id='down')
    config,events,_=sensors.load_recording(raw)
    evidence=dict(status='CODE_SUPPORTED',source_reference='EXPLICIT_MOCK_CONFIG_ONLY',source_sha256='a'*64)
    camera=config['cameras'][1]
    camera['calibration']=dict(version='DOWN_MOCK_EXPLICIT',
        intrinsics=dict(evidence,model='pinhole-radtan',parameters=[20,21,16,12,0,0,0,0]),
        T_imu_camera=dict(evidence,convention='camera_to_imu',matrix=np.eye(4).tolist()),
        camera_imu_offset=dict(evidence,value_ns=2_000_000,
            convention='imu_time_equals_mapped_camera_time_plus_offset'))
    # Existing source IDs remain sparse and must not be renumbered by transport.
    for event in events:
        event['sequence']+=10
    sealed=tmp_path/'sealed'
    sensors.write_recording(sealed,config,events,raw)
    params=tmp_path/'native-parameters.json'
    params.write_text(json.dumps(dict(schema_version=1,purpose='NATIVE_IMU_EXPERIMENT_PARAMETERS',
        measurement_binding=dict(evidence,imu_id='imu0',imu_frame='imu0',
            source_imu_config_sha256=hashlib.sha256(json.dumps(config['imu'],sort_keys=True,separators=(',',':')).encode()).hexdigest(),
            measurement_kind='RAW_SPECIFIC_FORCE',native_axes_transform='IDENTITY'),
        imu_params=dict(max_imu_delta_t=.1,acc_max=100.,omega_max=20.,sigma_omega_c=.001,
            sigma_acc_c=.01,sigma_omega_bias_c=.001,sigma_acc_bias_c=.01,sigma_integration=0.,g=9.81,imu_rate=20.),
        imu_initialization=dict(velocity=[0,0,0],omega_bias=[0,0,0],acc_bias=[0,0,0],
            velocity_sigma=2.,omega_bias_sigma=.1,acc_bias_sigma=.1))))
    return sealed,config,events,params


def test_sealed_recording_native_package_preserves_ids_pixels_source_clock_and_no_truth(sealed_native_recording,tmp_path):
    from tools.schurvins.dataset import prepare_recording
    source,config,events,parameters=sealed_native_recording
    original=file_hash(source/'recording.json')
    target=tmp_path/'native'
    report=prepare_recording(source,target,parameters)
    assert report['dataset']=='SENSOR_RECORDING' and report['purpose']=='SEALED_SENSOR_RECORDING_REPLAY'
    assert report['source_provenance']['kind']=='MOCK' and report['source_camera_id']=='down'
    assert report['sensor_remap']=={'imu0':'imu0','down':'cam0'}
    assert report['reference_scoring']=='NOT_RUN_NO_REFERENCE_SUPPLIED'
    assert not report['timing_application_verified'] and not report['physical_camera_imu_qualification']
    assert (target/'source-events.jsonl').read_bytes()==(source/'events.jsonl').read_bytes()
    with zipfile.ZipFile(target/'sensors.zip') as archive:
        assert not any('mav0/' in name or 'truth' in name for name in archive.namelist())
        transport=[json.loads(line) for line in archive.read('transport-events.jsonl').splitlines()]
        data=inputs(archive,'mono','SENSOR_RECORDING')
        assert [row['source_sequence'] for row in transport]==[event['sequence'] for event in events]
        assert [row['source_timestamp'] for row in transport]==[event['timestamp'] for event in events]
        assert data['cam0'][0][1]==events[1]['payload']['path']
        assert archive.read(data['cam0'][0][1])==(source/events[1]['payload']['path']).read_bytes()
        assert [t for t,_ in data['imu0']]==[e['timestamp']['raw_value'] for e in events if e['kind']=='IMU']
    native=yaml.safe_load((target/'native_calibration.yaml').read_text())
    assert native['cameras'][0]['camera']['label']=='down'
    assert native['cameras'][0]['camera']['intrinsics']['data']==[20,21,16,12]
    assert native['imu_params']['delay_imu_cam']==-.002
    assert native['imu_params']['imu_rate']==20.
    assert 'line-delay-nanoseconds' not in native['cameras'][0]['camera']
    raw=tmp_path/'observer'
    raw_evidence(target,raw)
    assert all(value['content_exact'] for value in reconcile(target,raw).values())
    assert file_hash(source/'recording.json')==original
    with pytest.raises(ValueError,match='OUTPUT_EXISTS'):
        prepare_recording(source,target,parameters)


@pytest.mark.parametrize('kind',['unknown_calibration','native_binding','missing_parameter','nonfinite','truth','unmapped_clock'])
def test_sealed_recording_refuses_unknown_axes_time_or_implicit_native_parameters(sealed_native_recording,tmp_path,kind):
    from rm27.perception.vision.localization.experiments import sensors
    from tools.schurvins.dataset import prepare_recording
    source,config,events,parameters=sealed_native_recording
    if kind=='unknown_calibration':config['cameras'][1]['calibration']['T_imu_camera']={'status':'UNKNOWN'}
    elif kind=='unmapped_clock':
        config['cameras'][1]['timestamp_contract']['clock_domain']='INDEPENDENT_UNMAPPED'
        for event in events:
            if event['kind']=='CAMERA':event['timestamp']['clock_domain']='INDEPENDENT_UNMAPPED'
    else:
        values=json.loads(parameters.read_text())
        if kind=='native_binding':values['measurement_binding']['native_axes_transform']='UNKNOWN'
        elif kind=='missing_parameter':del values['imu_params']['sigma_acc_c']
        elif kind=='nonfinite':values['imu_params']['sigma_acc_c']=float('nan')
        else:values['imu_params']['groundtruth_scale']=2.
        parameters.write_text(json.dumps(values))
    if kind in ('unknown_calibration','unmapped_clock'):
        changed=tmp_path/'changed'
        sensors.write_recording(changed,config,events,source)
        source=changed
    with pytest.raises(ValueError):prepare_recording(source,tmp_path/'refused',parameters)
    assert not (tmp_path/'refused').exists()


def test_sealed_recording_epoch_selection_is_explicit_and_retains_source_epoch(sealed_native_recording,tmp_path):
    from rm27.perception.vision.localization.experiments import sensors
    from tools.schurvins.dataset import prepare_recording
    source,config,events,parameters=sealed_native_recording
    second=copy.deepcopy(events)
    for event in second:event['epoch']=4
    combined=tmp_path/'two-epochs'
    sensors.write_recording(combined,config,events+second,source)
    with pytest.raises(ValueError,match='explicit source_epoch'):
        prepare_recording(combined,tmp_path/'ambiguous',parameters)
    target=tmp_path/'epoch-four'
    report=prepare_recording(combined,target,parameters,source_epoch=4)
    assert report['source_epoch']==4 and report['counts']=={'cam0':4,'imu0':4}
    with zipfile.ZipFile(target/'sensors.zip') as archive:
        rows=[json.loads(line) for line in archive.read('transport-events.jsonl').splitlines()]
        assert all(row['source_epoch']==4 for row in rows)
        assert rows[0]['source_event']==len(events)
        assert len(inputs(archive,'mono','SENSOR_RECORDING')['cam0'])==4


@pytest.mark.parametrize('kind',['duplicate','reorder','nan','identity','epoch','missing_image','stereo'])
def test_recording_transport_rejects_corruption_without_sorting(sealed_native_recording,tmp_path,kind):
    from tools.schurvins.dataset import prepare_recording
    source,_,_,parameters=sealed_native_recording
    target=tmp_path/'native'
    prepare_recording(source,target,parameters)
    with zipfile.ZipFile(target/'sensors.zip') as archive:
        members={name:archive.read(name) for name in archive.namelist()}
    rows=[json.loads(line) for line in members['transport-events.jsonl'].splitlines()]
    if kind=='duplicate':rows.insert(2,copy.deepcopy(rows[0]))
    elif kind=='reorder':rows[0],rows[2]=rows[2],rows[0]
    elif kind=='nan':rows[0]['gyro'][0]=float('nan')
    elif kind=='identity':rows[2]['source_sensor_id']='OTHER_IMU'
    elif kind=='epoch':rows[2]['source_epoch']=4
    elif kind=='missing_image':del members[rows[1]['image_member']]
    members['transport-events.jsonl']=''.join(json.dumps(row)+'\n' for row in rows).encode()
    modified=tmp_path/'corrupt.zip'
    with zipfile.ZipFile(modified,'w') as archive:
        for name,payload in members.items():archive.writestr(name,payload)
    with zipfile.ZipFile(modified) as archive,pytest.raises(ValueError):
        inputs(archive,'stereo' if kind=='stereo' else 'mono','SENSOR_RECORDING')


def test_recording_native_package_retains_source_sequence_gaps_and_delivery_only_delay(sealed_native_recording,tmp_path):
    from rm27.perception.vision.localization.experiments import sensors
    from tools.schurvins.dataset import prepare_recording
    source,config,events,parameters=sealed_native_recording
    events=[event for i,event in enumerate(events) if i!=2]
    events[1]['delivery_delay_ns']=100_000_000
    changed=tmp_path/'gap-delay'
    sensors.write_recording(changed,config,events,source)
    target=tmp_path/'native'
    report=prepare_recording(changed,target,parameters)
    assert report['sequence_gaps'][0]['unrepresented_sequence_values']==1
    assert report['sequence_gaps'][0]['missing_sample_count'] is None
    with zipfile.ZipFile(target/'sensors.zip') as archive:
        data=inputs(archive,'mono','SENSOR_RECORDING')
        rows=[json.loads(line) for line in archive.read('transport-events.jsonl').splitlines()]
    assert [row['source_sequence'] for row in rows if row['stream']=='imu0']==[10,12,13]
    schedule=json.loads((target/'delivery-schedule.json').read_text())
    delivered=scheduled_events(data,1.,delivery_schedule=schedule)
    first_camera=next(row for row in delivered if row[2]=='cam0' and row[1]==events[1]['timestamp']['raw_value'])
    assert first_camera[0]==100_000_000 and first_camera[1]==events[1]['timestamp']['raw_value']
    assert report['source_sensor_timestamps_modified'] is False


@pytest.mark.parametrize('kind',['payload','sequence','clock','calibration','qualification','pixels'])
def test_prepared_recording_crosschecks_source_seal_after_manifest_rehash(sealed_native_recording,tmp_path,kind):
    from tools.schurvins.dataset import prepare_recording,check_recording_input
    source,_,_,parameters=sealed_native_recording
    target=tmp_path/'native'
    prepare_recording(source,target,parameters)
    assert check_recording_input(target)['status']=='PASS'
    report=json.loads((target/'input.json').read_text())
    if kind=='calibration':
        path=target/'native_calibration.yaml'
        values=yaml.safe_load(path.read_text());values['cameras'][0]['T_B_C']['data'][3]=.1
        path.write_text(yaml.safe_dump(values))
    elif kind=='qualification':report['timing_application_verified']=True
    else:
        path=target/'sensors.zip'
        with zipfile.ZipFile(path) as archive:members={n:archive.read(n) for n in archive.namelist()}
        rows=[json.loads(line) for line in members['transport-events.jsonl'].splitlines()]
        if kind=='payload':rows[0]['gyro'][0]=3.
        elif kind=='sequence':rows[0]['source_sequence']=1
        elif kind=='clock':rows[0]['source_timestamp']['clock_domain']='FORGED'
        else:members[rows[1]['image_member']]=b'not the source PNG'
        members['transport-events.jsonl']=''.join(json.dumps(row)+'\n' for row in rows).encode()
        with zipfile.ZipFile(path,'w') as archive:
            for name,payload in members.items():archive.writestr(name,payload)
    report['artifacts_sha256']={name:file_hash(target/name) for name in report['artifacts_sha256']}
    (target/'input.json').write_text(json.dumps(report))
    with pytest.raises(ValueError):check_recording_input(target)
