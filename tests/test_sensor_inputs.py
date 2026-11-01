"""Production sensor event paths with labeled mock data; no VIO/hardware claims."""
import copy
import hashlib
import json
import zipfile

import cv2
import numpy as np
import pytest

from rm27.perception.vision.localization.experiments import sensors
from rm27.perception.vision.localization.experiments.common import file_hash, write_json


@pytest.fixture
def recording(tmp_path):
    source = tmp_path/'source'
    sensors.mock_recording(source, camera_count=2, frame_count=8)
    config, events, manifest = sensors.load_recording(source)
    return source, config, events, manifest


def profile(kind, ids, **params):
    return dict(schema_version=1, kind=kind, seed=37, sensor_ids=ids,
                start_elapsed_ns=50_000_000, end_elapsed_ns=250_000_000, **params)


class Clock:
    def __init__(self):
        self.value = 1_000_000_000

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += round(seconds*1e9)


def test_record_roundtrip_preserves_unknowns_hashes_and_fresh_output(recording, tmp_path):
    source, config, events, manifest = recording
    assert manifest['validation']['physical_camera_imu_qualification'] is False
    assert manifest['validation']['qualification'] == 'SENSOR_RECORD_AUDIT_ONLY'
    assert config['cameras'][0]['calibration']['T_imu_camera'] == {'status': 'UNKNOWN'}
    assert events[1]['exposure'] == {'status': 'UNKNOWN'}
    before = file_hash(source/'recording.json')
    copied = tmp_path/'copied'
    report = sensors.write_recording(copied, config, events, source)
    assert sensors.load_recording(copied) == (config, events, report)
    assert report['artifacts_sha256'] == manifest['artifacts_sha256']
    with pytest.raises(FileExistsError):
        sensors.write_recording(copied, config, events, source)
    assert file_hash(source/'recording.json') == before


def test_downward_mock_and_calibration_selection_are_independent(tmp_path, recording):
    source, config, events, _ = recording
    config['cameras'][0]['calibration']['version'] = 'FRONT_ONLY_UNKNOWN'
    config['cameras'][1]['calibration']['version'] = 'DOWN_ONLY_UNKNOWN'
    selection = sensors.CameraSelection(config, epoch=2)
    assert selection.switch('front')['epoch'] == 2
    switched = selection.switch('down')
    assert switched == dict(camera_id='down', previous_camera_id='front', epoch=3,
                            reset_required=True, calibration_version='DOWN_ONLY_UNKNOWN')
    assert selection.switch('down')['reset_required'] is False
    assert config['selected_camera_ids'] == ['front']
    with pytest.raises(ValueError, match='unselected camera'):
        sensors.validate_events(events, selection.config, source)
    down = tmp_path/'down'
    sensors.mock_recording(down, camera_count=2, selected_camera_id='down')
    down_config, down_events, _ = sensors.load_recording(down)
    assert down_config['selected_camera_ids'] == ['down']
    assert {e['sensor_id'] for e in down_events} == {'imu0', 'down'}


@pytest.mark.parametrize('site', ['config', 'event', 'camera_payload', 'imu_payload'])
def test_truth_payload_is_rejected_before_recording_or_emission(recording, tmp_path, site):
    source, config, events, _ = recording
    target = {'config': config, 'event': events[0],
              'camera_payload': events[1]['payload'], 'imu_payload': events[0]['payload']}[site]
    target['ground_truth'] = [1, 2, 3]
    with pytest.raises(ValueError, match='SENSOR_PROVENANCE_INVALID'):
        sensors.write_recording(tmp_path/'rejected', config, events, source)
    assert not (tmp_path/'rejected').exists()
    emitted = []
    with pytest.raises(ValueError):
        sensors.replay(events, config, lambda *row: emitted.append(row))
    assert emitted == []


@pytest.mark.parametrize('site', ['config', 'event'])
def test_boolean_schema_version_is_not_integer_one(recording, site):
    _, config, events, _ = recording
    (config if site == 'config' else events[0])['schema_version'] = True
    with pytest.raises(ValueError, match='schema version'):
        sensors.validate_events(events, config)


@pytest.mark.parametrize('bad', [True, '1', float('nan')])
def test_rigid_calibration_requires_real_finite_values(recording, bad):
    _, config, _, _ = recording
    matrix = np.eye(4).tolist()
    matrix[0][0] = bad
    config['cameras'][0]['calibration']['T_imu_camera'] = dict(status='CODE_SUPPORTED',
        matrix=matrix, convention='camera_to_imu', source_sha256='a'*64, source_reference='TEST_MOCK_CAL')
    with pytest.raises(ValueError):
        sensors.validate_config(config)


def mapped_camera(config, events, *, quantized=False):
    camera = config['cameras'][0]
    camera['timestamp_contract']['clock_domain'] = 'MOCK_CAMERA_CLOCK'
    if quantized:
        camera['timestamp_contract']['unit'] = 'ps'
    mapping = dict(mapping_id='camera_to_replay', source_clock='MOCK_CAMERA_CLOCK',
        target_clock='MOCK:clock', source_anchor_ns=0, target_anchor_ns=100,
        rate_numerator=1, rate_denominator=1, uncertainty_ns=1 if quantized else 0,
        evidence_status='CODE_SUPPORTED', source_sha256='b'*64)
    if quantized:
        mapping['quantization'] = 'ROUND_HALF_EVEN'
    config['clock_mappings'] = [mapping]
    for event in events:
        if event['kind'] != 'CAMERA':
            continue
        ns = event['timestamp']['raw_value']
        event['timestamp'].update(camera['timestamp_contract'])
        if quantized:
            event['timestamp']['raw_value'] = ns*1000+195
        event['mapped_timestamp'] = dict(raw_value=ns+100, unit='ns', clock_domain='MOCK:clock',
            semantic='MAPPED_SENSOR_TIME', evidence_status='CODE_SUPPORTED')
        event['clock_mapping_id'] = 'camera_to_replay'


def test_clock_mapping_preserves_large_integer_time_and_sign(recording):
    _, config, events, _ = recording
    old = events[1]['timestamp']['raw_value']
    mapped_camera(config, events)
    assert sensors.validate_events(events, config)['common_replay_clock'] is True
    assert sensors.event_time(events[1], config) == ('MOCK:clock', old+100)
    assert events[1]['timestamp']['raw_value'] == old
    events[1]['mapped_timestamp']['raw_value'] = old-100
    with pytest.raises(ValueError, match='mapped time differs'):
        sensors.validate_events(events, config)


def test_decimal_subnanosecond_text_requires_explicit_quantization(recording):
    _, config, events, _ = recording
    mapped_camera(config, events, quantized=True)
    raw_ps = events[1]['timestamp']['raw_value']
    assert raw_ps % 1000 == 195
    assert sensors.validate_events(events, config)['common_replay_clock'] is True
    assert sensors.event_time(events[1], config)[1] == raw_ps//1000+100
    del config['clock_mappings'][0]['quantization']
    with pytest.raises(ValueError, match='fractional nanoseconds'):
        sensors.validate_events(events, config)


def test_unknown_clock_is_auditable_but_never_replay_qualified(recording):
    _, config, events, _ = recording
    config['cameras'][0]['timestamp_contract']['clock_domain'] = 'UNKNOWN'
    for event in events:
        if event['kind'] == 'CAMERA':
            event['timestamp']['clock_domain'] = 'UNKNOWN'
    assert sensors.validate_events(events, config)['common_replay_clock'] is False
    emitted = []
    with pytest.raises(ValueError, match='known clock'):
        sensors.replay(events, config, lambda *row: emitted.append(row))
    assert not emitted


@pytest.mark.parametrize('verified', [False, True])
def test_only_verified_source_counters_prove_missing_samples(recording, verified):
    _, config, events, _ = recording
    config['imu']['sequence_semantics'] = 'SOURCE_SAMPLE_COUNTER'
    if verified:
        config['imu']['sequence_evidence'] = dict(status='VERIFIED', source_sha256='a'*64,
                                                source_reference='TEST_COUNTER_EVIDENCE')
    events.pop(2)
    gap = sensors.validate_events(events, config)['sequence_gaps'][0]
    assert gap['unrepresented_sequence_values'] == 1
    assert gap['missing_sample_count'] == (1 if verified else None)


@pytest.mark.parametrize('change', ['duplicate', 'reorder', 'time_reset', 'config_change'])
def test_source_errors_are_rejected_before_emitting_anything(recording, change):
    _, config, events, _ = recording
    if change == 'duplicate':
        events.insert(1, copy.deepcopy(events[0]))
    elif change == 'reorder':
        events[0], events[2] = events[2], events[0]
    elif change == 'time_reset':
        events[2]['timestamp']['raw_value'] = 0
    else:
        events[2]['config_version'] = 'DIFFERENT'
    output = []
    with pytest.raises(ValueError):
        sensors.replay(events, config, lambda *args: output.append(args))
    assert output == []


def test_replay_preserves_epoch_order_when_clock_restarts(recording):
    _, config, events, _ = recording
    first = copy.deepcopy(events[:4])
    second = copy.deepcopy(events[:4])
    for event in second:
        event['epoch'] = 1
        event['timestamp']['raw_value'] -= 1_000_000_000_000_000_000
    clock, output = Clock(), []
    report = sensors.replay(first+second, config, lambda e, r: output.append((e, r)),
                            clock_ns=clock.now, sleep=clock.sleep)
    assert [e['epoch'] for e, _ in output] == [0]*4+[1]*4
    assert [r['epoch_started'] for _, r in output] == [True, False, False, False]*2
    assert report['inter_epoch_policy'] == 'NO_INFERRED_TIME_GAP'
    assert report['elapsed_ns'] == 100_000_000


def test_delay_changes_delivery_only_and_rate_does_not_change_sensor_time(recording):
    _, config, events, _ = recording
    baseline = copy.deepcopy(events)
    for event in events:
        if event['kind'] == 'CAMERA':
            event['delivery_delay_ns'] = 100_000_000
    clock, output = Clock(), []
    report = sensors.replay(events, config, lambda e, r: output.append((e, r)), rate=2,
                            clock_ns=clock.now, sleep=clock.sleep)
    assert report['elapsed_ns'] == 275_000_000
    assert [e['kind'] for e, _ in output[:4]] == ['IMU']*4
    assert { (e['sensor_id'], e['sequence']): e['timestamp'] for e, _ in output } == {
        (e['sensor_id'], e['sequence']): e['timestamp'] for e in baseline}
    assert events[0] == baseline[0]


def stereo(config, events):
    config['mode'] = 'stereo'
    config['selected_camera_ids'] = ['front', 'down']
    config['stereo'] = dict(overlapping_view=True, synchronization='EXACT_SOURCE_TIME',
                            evidence_status='CODE_SUPPORTED', source_sha256='c'*64)
    result = []
    for event in events:
        result.append(event)
        if event['kind'] == 'CAMERA':
            right = copy.deepcopy(event)
            right.update(sensor_id='down', frame_id='camera1')
            result.append(right)
    return result


def test_stereo_uses_exact_source_time_without_assuming_front_down_overlap(recording):
    _, config, events, _ = recording
    events = stereo(config, events)
    assert sensors.validate_events(events, config)['counts']['down'] == 8
    config['stereo']['overlapping_view'] = False
    with pytest.raises(ValueError, match='nonoverlapping'):
        sensors.validate_events(events, config)
    config['stereo']['overlapping_view'] = True
    events[2]['timestamp']['raw_value'] += 1
    with pytest.raises(ValueError, match='original source time'):
        sensors.validate_events(events, config)


@pytest.mark.parametrize('kind, params', [
    ('camera_occlusion', {}), ('camera_blur', {'kernel_size': 5}),
    ('camera_dim', {'gain': .3}), ('camera_texture', {'fraction': .5}),
])
def test_pixel_faults_change_only_target_images_and_preserve_parent(recording, tmp_path, kind, params):
    source, _, original, manifest = recording
    output = tmp_path/kind
    report = sensors.derive_recording(source, output, profile(kind, ['front'], **params))
    _, derived, loaded = sensors.load_recording(output)
    assert report == loaded
    assert report['validation']['status'] == 'PASS'
    assert report['derivation']['experiment_kind'] == 'ARTIFICIAL_DISTURBANCE'
    changed = {item['source_event'] for item in report['derivation']['changes']}
    assert changed == {3, 5, 7, 9}
    for i, (before, after) in enumerate(zip(original, derived)):
        assert before['timestamp'] == after['timestamp']
        if i in changed:
            assert before['payload']['pixels_sha256'] != after['payload']['pixels_sha256']
            pixels = cv2.imread(str(output/after['payload']['path']), cv2.IMREAD_UNCHANGED)
            assert hashlib.sha256(pixels.tobytes()).hexdigest() == after['payload']['pixels_sha256']
        else:
            assert before == after
    assert {name: file_hash(source/name) for name in manifest['artifacts_sha256']} == manifest['artifacts_sha256']


def test_random_drop_seed_is_repeatable_and_gaps_are_not_hidden(recording, tmp_path):
    source, _, _, _ = recording
    fault = profile('camera_drop', ['front'], probability=.5)
    first = sensors.derive_recording(source, tmp_path/'one', fault)
    second = sensors.derive_recording(source, tmp_path/'two', fault)
    assert first == second
    assert first['validation']['sequence_gaps']
    assert first['derivation']['profile']['seed'] == 37


def test_stereo_random_drop_requires_explicit_pair_policy_to_preserve_source_bundles(recording,tmp_path):
    source,config,events,_=recording
    events=stereo(config,events)
    pair_source=tmp_path/'pair_source'
    sensors.write_recording(pair_source,config,events,source)
    fault=profile('camera_drop',['front','down'],probability=.5,camera_bundle_policy='SAME_SOURCE_TIME')
    fault.update(start_elapsed_ns=0,end_elapsed_ns=400_000_000)
    sensors.derive_recording(pair_source,tmp_path/'pair_drop',fault)
    _,derived,manifest=sensors.load_recording(tmp_path/'pair_drop')
    stamps={s:[sensors.event_time(e,config)[1] for e in derived if e['sensor_id']==s]
            for s in ['front','down']}
    assert 0<len(stamps['front'])<8 and stamps['front']==stamps['down']
    assert manifest['derivation']['profile']['camera_bundle_policy']=='SAME_SOURCE_TIME'
    fault['sensor_ids']=['front']
    with pytest.raises(ValueError,match='paired dropout'):
        sensors.derive_recording(pair_source,tmp_path/'bad',fault)


@pytest.mark.parametrize('kind, expected_changes', [('imu_duplicate', 1), ('imu_reorder', 2), ('imu_nonfinite', 4)])
def test_invalid_faults_are_recorded_but_rejected_for_replay(recording, tmp_path, kind, expected_changes):
    source, _, _, _ = recording
    output = tmp_path/kind
    report = sensors.derive_recording(source, output, profile(kind, ['imu0']))
    assert report['validation']['status'] == 'FAIL'
    assert len(report['derivation']['changes']) == expected_changes
    with pytest.raises(ValueError):
        sensors.load_recording(output)
    config, events, _ = sensors.load_recording(output, allow_invalid=True)
    emitted = []
    with pytest.raises(ValueError):
        sensors.replay(events, config, lambda *row: emitted.append(row))
    assert emitted == []
    if kind == 'imu_nonfinite':
        assert '"NaN"' in (output/'events.jsonl').read_text()
        json.loads((output/'events.jsonl').read_text().splitlines()[0],
                    parse_constant=lambda value: pytest.fail('nonstandard JSON'))


def test_transport_delay_and_imu_gap_have_explicit_unchanged_source_times(recording, tmp_path):
    source, _, events, _ = recording
    sensors.derive_recording(source, tmp_path/'delay', profile('delivery_delay', ['front'], delay_ns=20_000_000))
    _, delayed, report = sensors.load_recording(tmp_path/'delay')
    assert [e['timestamp'] for e in delayed] == [e['timestamp'] for e in events]
    assert len(report['derivation']['changes']) == 4
    sensors.derive_recording(source, tmp_path/'gap', profile('imu_gap', ['imu0']))
    _, gapped, gap_report = sensors.load_recording(tmp_path/'gap')
    assert len(gapped) == len(events)-4
    assert gap_report['validation']['sequence_gaps'][0]['unrepresented_sequence_values'] == 4


def test_imu_bias_is_explicit_finite_disturbance_without_claiming_sensor_health(recording,tmp_path):
    source,config,events,_=recording
    fault=profile('imu_bias',['imu0'],gyro_bias_rad_s=[.2,-.1,.3],accel_bias_m_s2=[1.,2.,-3.])
    sensors.derive_recording(source,tmp_path/'bias',fault)
    _,derived,report=sensors.load_recording(tmp_path/'bias')
    changed={row['source_event'] for row in report['derivation']['changes']}
    assert changed=={2,4,6,8}
    for i,(before,after) in enumerate(zip(events,derived)):
        assert before['timestamp']==after['timestamp']
        if i in changed:
            assert after['payload']['gyro_xyz']==[.2,-.1,.3]
            assert after['payload']['accel_xyz']==[1.,2.,6.8100000000000005]
        else:
            assert after==before
    assert report['validation']['physical_camera_imu_qualification'] is False


@pytest.mark.parametrize('path', ['../outside.png', '/tmp/outside.png', 'events.jsonl'])
def test_camera_path_refused_before_output_creation(recording,tmp_path,path):
    source,config,events,_=recording
    events[1]['payload']['path']=path
    with pytest.raises(ValueError,match='SENSOR_IMAGE_INVALID'):
        sensors.write_recording(tmp_path/'rejected',config,events,source)
    assert not (tmp_path/'rejected').exists()


def test_invalid_empty_or_existing_derivation_does_not_modify_inputs(recording, tmp_path):
    source, _, _, _ = recording
    empty = profile('imu_reorder', ['imu0'])
    empty.update(start_elapsed_ns=0, end_elapsed_ns=1)
    with pytest.raises(ValueError, match='affected no source events'):
        sensors.derive_recording(source, tmp_path/'empty', empty)
    assert not (tmp_path/'empty').exists()
    with pytest.raises(ValueError, match='outside source'):
        sensors.derive_recording(source, source/'child', profile('imu_gap', ['imu0']))
    with pytest.raises(ValueError, match='outside source'):
        sensors.derive_recording(source, source, profile('imu_gap', ['imu0']))
    existing = tmp_path/'existing'
    existing.mkdir()
    (existing/'keep').write_text('existing')
    with pytest.raises(FileExistsError):
        sensors.derive_recording(source, existing, profile('imu_gap', ['imu0']))
    assert (existing/'keep').read_text() == 'existing'


@pytest.mark.parametrize('tamper', ['pixels', 'extra_truth', 'symlink', 'manifest_validation'])
def test_frozen_artifact_or_manifest_changes_are_rejected(recording, tamper):
    source, _, events, report = recording
    if tamper == 'pixels':
        (source/events[1]['payload']['path']).write_bytes(b'corrupt')
    elif tamper == 'extra_truth':
        truth = source/'truth.txt'
        truth.write_text('FORBIDDEN')
        report['artifacts_sha256']['truth.txt'] = file_hash(truth)
        write_json(source/'recording.json', report)
    elif tamper == 'symlink':
        (source/'link').symlink_to(source/'config.json')
    else:
        report['validation']['counts']['imu0'] += 1
        write_json(source/'recording.json', report)
    with pytest.raises(ValueError):
        sensors.load_recording(source)


def uzh_archive(path, *, invalid=None):
    image = np.indices((24, 32)).sum(axis=0).astype(np.uint8)*7
    ok, encoded = cv2.imencode('.png', image)
    assert ok
    camera_times = ['1540821358.224332571030', '1540821358.257545948029']
    imu_times = ['1540821358.195877075195', '1540821358.227874069214', '1540821358.267875116348']
    with zipfile.ZipFile(path, 'w') as archive:
        imu = '# dataset source id decimal seconds gyro accel\n'+''.join(
            f'{i} {timestamp} 0.1 0.2 0.3 1.1 2.2 9.81\n' for i, timestamp in enumerate(imu_times))
        if invalid == 'nonfinite':
            imu = imu.replace('0.1', 'nan', 1)
        if invalid == 'reorder':
            imu = imu.splitlines()[0]+'\n'+'\n'.join(reversed(imu.splitlines()[1:]))
        archive.writestr('imu.txt', imu)
        for camera, prefix in [('left', 0), ('right', 1)]:
            archive.writestr(camera+'_images.txt', '# id timestamp image_name\n'+''.join(
                f'{i} {timestamp} img/image_{prefix}_{i}.png\n' for i, timestamp in enumerate(camera_times)))
            for i in range(2):
                archive.writestr(f'img/image_{prefix}_{i}.png', encoded.tobytes())
        archive.writestr('groundtruth.txt', 'FORBIDDEN: THIS MEMBER MUST NOT ENTER EVENTS')


@pytest.mark.parametrize('camera_id', ['cam0', 'cam1'])
def test_uzh_import_preserves_printed_time_pixels_and_independent_camera(tmp_path, camera_id):
    source, output = tmp_path/'uzh.zip', tmp_path/'imported'
    uzh_archive(source)
    before = file_hash(source)
    result = sensors.import_dataset(source, output, 'UZH_FPV', camera_id=camera_id)
    config, events, loaded = sensors.load_recording(output)
    assert result == loaded
    assert file_hash(source) == before
    assert config['selected_camera_ids'] == [camera_id]
    assert config['provenance']['physical_timestamp_resolution'] == 'UNKNOWN'
    assert {e['sensor_id'] for e in events} == {'imu0', camera_id}
    first = events[0]
    assert first['timestamp']['raw_value'] == 1540821358195877075195
    assert first['timestamp']['unit'] == 'ps'
    assert first['mapped_timestamp']['raw_value'] == 1540821358195877075
    assert first['source_reference']['timestamp_text'] == '1540821358.195877075195'
    assert first['source_reference']['representation_rounding_error_ns'] == dict(numerator=-39, denominator=200)
    assert 'FORBIDDEN' not in (output/'events.jsonl').read_text()
    assert not any('truth' in p.name for p in output.rglob('*'))
    assert loaded['validation']['counts'] == {'imu0': 3, camera_id: 2}
    with pytest.raises(ValueError, match='camera time offsets'):
        sensors.import_dataset(source, tmp_path/'stereo', 'UZH_FPV', mode='stereo')
    assert not (tmp_path/'stereo').exists()


@pytest.mark.parametrize('invalid', ['nonfinite', 'reorder'])
def test_invalid_public_source_is_rejected_before_output_creation(tmp_path, invalid):
    source, output = tmp_path/'uzh.zip', tmp_path/'invalid'
    uzh_archive(source, invalid=invalid)
    with pytest.raises(ValueError):
        sensors.import_dataset(source, output, 'UZH_FPV')
    assert not output.exists()


def test_euroc_import_reuses_existing_strict_csv_parser_and_excludes_truth(tmp_path):
    from test_schurvins_transport import source_zip, CAMERA_NS
    source, output = tmp_path/'euroc.zip', tmp_path/'imported'
    pixels = source_zip(source)
    sensors.import_dataset(source, output, 'EUROC', mode='stereo')
    config, events, manifest = sensors.load_recording(output)
    assert manifest['validation']['counts'] == {'imu0': 3, 'cam0': 3, 'cam1': 3}
    assert {e['timestamp']['raw_value'] for e in events} == set(CAMERA_NS)
    assert config['cameras'][0]['calibration']['T_imu_camera']['status'] == 'UNKNOWN'
    for event in events:
        if event['kind'] == 'CAMERA':
            raw = cv2.imread(str(output/event['payload']['path']), cv2.IMREAD_UNCHANGED)
            assert np.array_equal(raw, pixels[event['sensor_id'], event['timestamp']['raw_value']])
    assert not any('truth' in p.name for p in output.rglob('*'))


def test_storage_reserve_blocks_import_before_any_output(tmp_path):
    source, output = tmp_path/'uzh.zip', tmp_path/'blocked'
    uzh_archive(source)
    with pytest.raises(ValueError, match='storage reserve'):
        sensors.import_dataset(source, output, 'UZH_FPV', reserve_bytes=10**30)
    assert not output.exists()


@pytest.mark.parametrize('kind,parameters', [
    ('camera_gap', {}), ('camera_dim', {'gain': .25}),
    ('delivery_delay', {'delay_ns': 100_000_000}),
    ('imu_gap', {}), ('imu_bias', {'gyro_bias_rad_s': [.2, 0, 0], 'accel_bias_m_s2': [0, 0, 0]}),
])
def test_native_fault_export_keeps_full_source_and_sensor_calibration(tmp_path, kind, parameters):
    from test_schurvins_transport import prepared, CAMERA_NS
    from tools.schurvins.dataset import derive_prepared
    _, source, manifest, _ = prepared(tmp_path)
    original = {p.name: file_hash(p) for p in source.iterdir()}
    fault = profile(kind, ['imu0' if kind.startswith('imu') else 'cam0'], **parameters)
    fault.update(start_elapsed_ns=0, end_elapsed_ns=50_000_000)
    output = tmp_path/'derived'
    report = derive_prepared(source, output, fault)
    assert report['derivation']['validation']['status'] == 'PASS'
    assert report['source_counts'] == manifest['counts']
    assert report['source_first_ns'] == manifest['first_ns']
    assert (output/'source-expected.jsonl').read_bytes() == (source/'expected.jsonl').read_bytes()
    assert {p.name: file_hash(p) for p in source.iterdir()} == original
    with zipfile.ZipFile(source/'sensors.zip') as before, zipfile.ZipFile(output/'sensors.zip') as after:
        assert not any('truth' in name for name in after.namelist())
        for sensor in manifest['counts']:
            assert before.read('mav0/'+sensor+'/sensor.yaml') == after.read('mav0/'+sensor+'/sensor.yaml')
    rows = [json.loads(line) for line in (output/'expected.jsonl').read_text().splitlines()]
    assert {r['state_ns'] for r in rows} <= set(CAMERA_NS)
    if kind == 'camera_gap':
        assert report['counts']['cam0'] == 2
    elif kind == 'camera_dim':
        first = next(r for r in rows if r['stream'] == 'cam0')
        before = next(json.loads(line) for line in (source/'expected.jsonl').read_text().splitlines()
                      if json.loads(line)['stream'] == 'cam0')
        assert first['pixels_sha256'] != before['pixels_sha256']
    elif kind == 'delivery_delay':
        assert file_hash(output/'sensors.zip') == file_hash(source/'sensors.zip')
        assert report['derivation']['native_archive_storage']=='IMMUTABLE_PARENT_HARDLINK'
        assert (output/'sensors.zip').stat().st_ino == (source/'sensors.zip').stat().st_ino
        schedule = json.loads((output/'delivery-schedule.json').read_text())
        assert schedule['entries'] == [dict(stream='cam0',state_ns=CAMERA_NS[0],source_sequence=0,
                                             delivery_delay_ns=100_000_000)]
    with pytest.raises(FileExistsError):
        derive_prepared(source, output, fault)


@pytest.mark.parametrize('kind', ['imu_duplicate', 'imu_reorder', 'imu_nonfinite'])
def test_native_invalid_injection_retains_failure_and_original_source(tmp_path, kind):
    from test_schurvins_transport import prepared
    from tools.schurvins.dataset import derive_prepared
    _, source, _, _ = prepared(tmp_path)
    fault = profile(kind, ['imu0'])
    fault.update(start_elapsed_ns=0, end_elapsed_ns=100_000_001)
    report = derive_prepared(source, tmp_path/'derived', fault)
    assert report['derivation']['validation']['status'] == 'FAIL'
    assert report['source_counts'] == {'imu0': 3, 'cam0': 3}


def test_native_reconciliation_checks_declared_delayed_order_and_explicit_prefix(tmp_path):
    from test_schurvins_transport import prepared, CAMERA_NS
    from tools.schurvins.dataset import derive_prepared, reconcile
    _, source, _, _ = prepared(tmp_path)
    output = tmp_path/'derived'
    fault = profile('delivery_delay', ['cam0'], delay_ns=75_000_000)
    fault.update(start_elapsed_ns=0,end_elapsed_ns=50_000_000)
    derive_prepared(source,output,fault)
    raw = tmp_path/'raw'
    raw.mkdir()
    rows = [json.loads(line) for line in (output/'expected.jsonl').read_text().splitlines()]
    def save(camera_rows, imu_rows):
        (raw/'published.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in camera_rows+imu_rows))
        for sensor,wanted in [('cam0',camera_rows),('imu0',imu_rows)]:
            (raw/(sensor+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in wanted))
    cameras = [r for r in rows if r['stream']=='cam0']
    imus = [r for r in rows if r['stream']=='imu0']
    save([cameras[1],cameras[0],cameras[2]],imus)
    result = reconcile(output,raw,rate=1.)
    assert result['cam0']['source_order_changed'] is True
    assert result['cam0']['adjacent_source_time_inversions'] == 1
    save(cameras,imus)
    with pytest.raises(ValueError,match='content/order mismatch'):
        reconcile(output,raw,rate=1.)
    save(cameras[1:],imus[1:])
    result = reconcile(output,raw,start_offset_ns=50_000_000,rate=1.)
    assert result['cam0']['omitted_prefix_count'] == 1
    assert result['cam0']['replay_cutoff_ns'] == CAMERA_NS[1]
