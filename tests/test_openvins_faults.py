"""Artificial comparison contracts; synthetic cases do not qualify VIO."""
import json
from pathlib import Path
import zipfile

import pytest

from rm27.perception.vision.localization.experiments.common import file_hash
from tools.openvins.comparison_transport import artificial_input, storage_events
from tools.schurvins.dataset import prepare, derive_prepared
from tools.schurvins.ros_transport import scheduled_events


@pytest.mark.parametrize('rate', [.5, 1.])
def test_storage_timeline_preserves_common_wall_delay_and_original_sensor_stamps(rate):
    origin=1403715273262142977
    streams=dict(imu0=[(origin,[0]*6),(origin+50_000_000,[0]*6)],
                 cam0=[(origin,'first.png'),(origin+50_000_000,'second.png')])
    schedule=dict(schema_version=1,time_semantics='DELIVERY_DELAY_ONLY_SOURCE_STAMP_UNCHANGED',entries=[
        dict(stream='cam0',state_ns=origin,delivery_delay_ns=300_000_000)])
    actual=storage_events(streams,rate,schedule)
    expected=scheduled_events(streams,rate,delivery_schedule=schedule)
    assert [(int((storage-origin)/rate),ns,sensor,payload) for storage,ns,sensor,payload in actual]==expected
    assert [ns for _,ns,sensor,_ in actual if sensor=='cam0']==[origin+50_000_000,origin]
    assert actual[-1][1:]==(origin,'cam0','first.png')


def fault_input(tmp_path):
    from test_schurvins_transport import source_zip
    original=tmp_path/'source.zip';source_zip(original)
    base=tmp_path/'base';prepare(original,base,'stereo')
    derived=tmp_path/'fault'
    profile=dict(schema_version=1,kind='camera_gap',seed=270031,sensor_ids=['cam0','cam1'],
                 start_elapsed_ns=50_000_000,end_elapsed_ns=100_000_000)
    derive_prepared(base,derived,profile)
    return original,derived


def test_artificial_intake_retains_original_denominator_and_calibration(tmp_path):
    original,derived=fault_input(tmp_path)
    manifest,archive,schedule=artificial_input(derived,original,'stereo')
    assert schedule is None and archive==derived/'sensors.zip'
    assert manifest['counts']==dict(imu0=3,cam0=2,cam1=2)
    assert manifest['source_counts']==dict(imu0=3,cam0=3,cam1=3)
    with zipfile.ZipFile(archive) as source:
        assert not any('truth' in name for name in source.namelist())


def test_artificial_intake_rejects_reference_change(tmp_path):
    original,derived=fault_input(tmp_path)
    with original.open('ab') as stream:stream.write(b'reference changed')
    with pytest.raises(ValueError,match='HASH_MISMATCH'):
        artificial_input(derived,original,'stereo')


def test_artificial_intake_rejects_hidden_truth_even_after_archive_rehash(tmp_path):
    original,derived=fault_input(tmp_path)
    with zipfile.ZipFile(derived/'sensors.zip','a') as archive:
        archive.writestr('mav0/truth.csv','forbidden synthetic truth')
    manifest=json.loads((derived/'input.json').read_text())
    manifest['artifacts_sha256']['sensors.zip']=file_hash(derived/'sensors.zip')
    (derived/'input.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match='TRUTH_ISOLATION_FAILED'):
        artificial_input(derived,original,'stereo')


def test_artificial_observer_audit_accepts_declared_reorder_and_rejects_silent_sort(tmp_path):
    from types import SimpleNamespace
    from tools.openvins.comparison_transport import events_from,image_pixels
    from rm27.perception.vision.localization.experiments.openvins_comparison import audit_artificial_delivery
    import hashlib
    original,derived=fault_input(tmp_path)
    archive=derived/'sensors.zip'
    with zipfile.ZipFile(archive) as source:
        streams,_=events_from(source,'stereo')
        delivered={sensor:list(reversed(rows)) for sensor,rows in streams.items()}
        for sensor,rows in delivered.items():
            payloads=[]
            for ns,payload in rows:
                row=dict(state_ns=ns,frame_id=sensor)
                if sensor=='imu0':row.update(gyro=payload[:3],accel=payload[3:])
                else:
                    pixels=image_pixels(source,sensor,payload)
                    row.update(height=pixels.shape[0],width=pixels.shape[1],step=pixels.shape[1],
                        encoding='mono8',is_bigendian=0,pixels_sha256=hashlib.sha256(pixels.tobytes()).hexdigest())
                payloads.append(row)
            (tmp_path/(sensor+'.jsonl')).write_text(''.join(json.dumps(row)+'\n' for row in payloads))
    dataset=SimpleNamespace(archive=archive,delivered_streams=delivered)
    report=audit_artificial_delivery(tmp_path,dataset)
    assert report['cam0']['source_time_inversions']==1
    rows=[json.loads(line) for line in (tmp_path/'cam0.jsonl').read_text().splitlines()]
    (tmp_path/'cam0.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in sorted(rows,key=lambda row:row['state_ns'])))
    with pytest.raises(ValueError,match='ARTIFICIAL_DELIVERY_MISMATCH'):
        audit_artificial_delivery(tmp_path,dataset)


def test_artificial_scoring_uses_only_original_truth_and_original_camera_ids(tmp_path):
    from decimal import Decimal
    from types import SimpleNamespace
    from rm27.perception.vision.localization.experiments.openvins import pose_observation
    from rm27.perception.vision.localization.experiments.openvins_comparison import evaluate_comparison
    stamps=[1403715273262142977+i*10_000_000 for i in range(12)]
    points=[[i*.1,(i%3)*.1,(i%2)*.05] for i in range(12)]
    original=tmp_path/'original.zip';derived=tmp_path/'derived.zip'
    with zipfile.ZipFile(original,'w') as source:
        source.writestr('mav0/cam0/data.csv',''.join(f'{ns},{ns}.png\n' for ns in stamps))
        source.writestr('mav0/state_groundtruth_estimate0/data.csv',''.join(
            ','.join(map(str,[ns,*point,1,0,0,0]))+'\n' for ns,point in zip(stamps,points)))
    with zipfile.ZipFile(derived,'w') as source:source.writestr('sensors_only','NO_TRUTH')
    ids=[i for i in range(12) if i!=4]
    (tmp_path/'raw').mkdir();(tmp_path/'online').mkdir()
    (tmp_path/'online/trajectory.json').write_text('[]')
    poses=[dict(state_ns=stamps[i],position=points[i],quaternion_xyzw=[0,0,0,1]) for i in ids]
    (tmp_path/'raw/pose.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in poses))
    (tmp_path/'raw/state_estimate.txt').write_text(''.join(' '.join(map(str,[
        Decimal(stamps[i])/Decimal(10**9),0,0,0,1,*points[i],*([0]*9),0,1]))+'\n' for i in ids))
    records=[pose_observation(source_frame_id=i,image_ns=stamps[i],state_ns=stamps[i],recorder_ns=stamps[i],
        position=points[i],quaternion_xyzw=[0,0,0,1],run_id='TEST_FIXTURE_ONLY',epoch=0,
        binary_sha256='0'*64,raw_reference={'topic':'/poseimu'}) for i in ids]
    dataset=SimpleNamespace(archive=derived,reference_archive=original,artificial=True,camera_ns=[stamps[i] for i in ids],
        source_ids=ids,original_camera_ns=stamps)
    score=evaluate_comparison(tmp_path,dataset,records)
    assert score['same_run_metrics_exact']
    assert score['full_source_coverage']['original_camera_count']==12
    assert score['full_source_coverage']['trajectory_fraction_of_original']==11/12
    assert score['original_reference_archive_sha256']==file_hash(original)


def test_artificial_config_does_not_accept_ambiguous_archive_alias(tmp_path):
    from rm27.perception.vision.localization.experiments.openvins_comparison import run
    result=run(dict(schema_version=1,purpose='EUROC_ARTIFICIAL_COMPARISON',backend='OpenVINS',archive='ambiguous'))
    assert result['status']=='REFUSED' and result['failure_code']=='COMPARISON_CONFIG_INVALID'


def test_existing_matrix_supports_one_openvins_fault_without_claiming_recovery(tmp_path,monkeypatch):
    from test_prehardware import cohort,runtime_output
    from rm27.perception.vision.localization.experiments import prehardware as ph
    from rm27.perception.vision.localization.experiments.common import write_json,read_json
    gate,config=cohort(tmp_path,backend='OpenVINS',gate_id='ov-fault',role='fault_recovery')
    derived=tmp_path/'derived';derived.mkdir();(derived/'sensors.zip').write_text('TEST_FIXTURE_ONLY')
    write_json(derived/'input.json',dict(artifacts_sha256={'sensors.zip':file_hash(derived/'sensors.zip')},
        derivation=dict(experiment_kind='ARTIFICIAL_DISTURBANCE',validation=dict(status='PASS'))))
    config.update(purpose='EUROC_ARTIFICIAL_COMPARISON',original_reference_archive=config.pop('archive'),
        derived_sensor_archive=str(derived/'sensors.zip'),prepared_input=str(derived))
    write_json(gate['config_file'],config)
    gate.update(repeat_count=0,inputs_sha256=ph.config_locks(gate['config_file']))
    ph.validate_matrix(dict(schema_version=1,gates=[gate]))
    calls=[]
    def execute(backend,current):
        calls.append(current);runtime_output(current)
    monkeypatch.setattr(ph,'_execute_backend',execute)
    # This orchestration fixture has no ROS bag; use its separately frozen
    # TEST_FIXTURE_ONLY readback, just as the acceptance orchestration tests do.
    monkeypatch.setattr(ph,'_recheck_openvins_camera',
        lambda root,current:read_json(root/'fixture-camera-readback.json'))
    report=ph.run(dict(schema_version=1,gates=[gate]),tmp_path/'acceptance',execute=True)
    member=report['gates'][0]['members'][0]
    assert len(calls)==1 and member['status']=='NOT_RUN'
    assert member['numerical_replay_status']=='PASS' and member['recovery_state_status']=='NOT_RUN'
    assert read_json(Path(calls[0]['output'])/'run.json')['status']=='EXECUTION_COMPLETED'
    assert str(derived/'input.json') in gate['inputs_sha256']
    assert str(derived/'sensors.zip') in gate['inputs_sha256']


def test_artificial_dataset_requires_rate_lock_and_preserves_original_ids(tmp_path):
    import sqlite3
    from rm27.perception.vision.localization.experiments.openvins_comparison import ComparisonDataset
    original,derived=fault_input(tmp_path)
    prepared=json.loads((derived/'input.json').read_text())
    bag=tmp_path/'bag';bag.mkdir()
    with sqlite3.connect(bag/'test.db3') as database:
        database.execute('CREATE TABLE topics (name TEXT,type TEXT)')
        for topic,kind in [('/cam0/image_raw','Image'),('/cam1/image_raw','Image'),('/imu0','Imu')]:
            database.execute('INSERT INTO topics VALUES (?,?)',(topic,'sensor_msgs/msg/'+kind))
    (bag/'metadata.yaml').write_text('TEST_FIXTURE_ONLY')
    report=dict(purpose='EUROC_ARTIFICIAL_COMPARISON',sequence='TEST_FIXTURE_ONLY',mode='stereo',
        source_sha256=file_hash(derived/'sensors.zip'),readback_exact=True,truth_included=False,
        verified_counts=prepared['counts'],derived_sensor_archive_sha256=file_hash(derived/'sensors.zip'),
        original_reference_archive_sha256=file_hash(original),prepared_input_sha256=file_hash(derived/'input.json'),
        delivery_schedule=None,playback_rate=.5,
        artifacts_sha256={p.name:file_hash(p) for p in bag.iterdir()})
    (bag/'source_verification.json').write_text(json.dumps(report))
    args=(derived/'sensors.zip',bag,'TEST_FIXTURE_ONLY','stereo',file_hash(derived/'sensors.zip'))
    options=dict(original_reference_archive=original,original_reference_sha256=file_hash(original),prepared_input=derived)
    current=ComparisonDataset(*args,**options,rate=.5)
    assert current.source_ids==[0,2] and len(current.original_camera_ns)==3
    with pytest.raises(ValueError,match='COMPARISON_FAULT_MISMATCH'):
        ComparisonDataset(*args,**options,rate=1.)


def native_imu_fixture(directory, stamps=None):
    """Independent observer rows and matching native callback payloads."""
    from test_native_events import BASE, ov_native_event
    stamps = stamps if stamps is not None else [BASE+1+i*5_000_000 for i in range(6)]
    observed = [dict(state_ns=ns, frame_id='imu0', gyro=[i+.1, i+.2, i+.3],
                     accel=[i+.4, i+.5, i+.6]) for i, ns in enumerate(stamps)]
    (directory/'imu0.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in observed))
    return [ov_native_event('imu_ros_callback', row['state_ns'], camera=0,
                            values=row['gyro']+row['accel']) for row in observed]


def test_native_imu_callback_audit_preserves_raw_integer_precision(tmp_path):
    from test_native_events import BASE, ov_trace, ov_visual_rows
    from tools.openvins.native_observations import analyze_openvins_observations
    # Adjacent integer stamps intentionally collapse to the same native double.
    stamps = [BASE+1, BASE+2, BASE+5_000_001]
    callbacks = native_imu_fixture(tmp_path, stamps)
    assert callbacks[0]['sensor_time_double_bits'] == callbacks[1]['sensor_time_double_bits']
    assert int(callbacks[0]['sensor_time_seconds']*10**9) != stamps[0]
    result = analyze_openvins_observations(ov_trace(tmp_path, callbacks+ov_visual_rows()),
                                          input_directory=tmp_path)
    coverage = result['native_imu_callback_coverage']
    assert coverage['status'] == 'PASS'
    assert coverage['expected_observer_count'] == coverage['native_callback_count'] == 3
    assert coverage['missing_count'] == 0 and coverage['missing_intervals'] == []
    assert coverage['present_values_exact'] is True
    assert coverage['source_sha256'] == file_hash(tmp_path/'imu0.jsonl')
    assert result['raw_input_equality'] == result['health_qualification'] == 'NOT_RUN'


def test_native_imu_callback_audit_reports_missing_prefix_burst_and_tail(tmp_path):
    from test_native_events import ov_trace, ov_visual_rows
    from tools.openvins.native_observations import analyze_openvins_observations
    callbacks = native_imu_fixture(tmp_path)
    expected = [json.loads(line) for line in (tmp_path/'imu0.jsonl').read_text().splitlines()]
    result = analyze_openvins_observations(ov_trace(tmp_path, [callbacks[1], callbacks[4]]+ov_visual_rows()),
                                          input_directory=tmp_path)
    coverage = result['native_imu_callback_coverage']
    assert result['status'] == result['observation_integrity'] == 'PASS'
    assert coverage['status'] == 'FAIL' and coverage['missing_count'] == 4
    assert coverage['expected_observer_count'] == 6 and coverage['native_callback_count'] == 2
    assert coverage['present_values_exact'] is True
    assert coverage['missing_intervals'] == [
        dict(first_source_index=first, last_source_index=last, count=last-first+1,
             first_sensor_ns=expected[first]['state_ns'], last_sensor_ns=expected[last]['state_ns'])
        for first, last in [(0, 0), (2, 3), (5, 5)]]


@pytest.mark.parametrize('defect', ['duplicate', 'reordered', 'value', 'raw-integer'])
def test_native_imu_callback_audit_rejects_mismatched_callback_content(tmp_path, defect):
    from test_native_events import ov_trace, ov_visual_rows
    from tools.openvins.native_observations import analyze_openvins_observations
    callbacks = native_imu_fixture(tmp_path)
    if defect == 'duplicate':
        callbacks.insert(1, dict(callbacks[0]))
    elif defect == 'reordered':
        callbacks[1], callbacks[2] = callbacks[2], callbacks[1]
    elif defect == 'value':
        callbacks[1]['values'][0] += .125
    else:
        # A one-nanosecond change is invisible in the native double but must
        # fail exact matching against the independently recorded raw integer.
        callbacks[0]['raw_nsec'] += 1
        assert (callbacks[0]['raw_sec']+callbacks[0]['raw_nsec']*1e-9
                == callbacks[0]['sensor_time_seconds'])
    with pytest.raises(ValueError, match='OPENVINS_NATIVE_INPUT_MISMATCH'):
        analyze_openvins_observations(ov_trace(tmp_path, callbacks+ov_visual_rows()),
                                      input_directory=tmp_path)


@pytest.mark.parametrize('defect', ['duplicate', 'reordered', 'frame', 'timestamp-float', 'nonfinite',
                                  'axis-count', 'axis-type', 'missing-vector', 'nonobject'])
def test_native_imu_callback_audit_rejects_invalid_observer_reference(tmp_path, defect):
    from test_native_events import ov_trace, ov_visual_rows
    from tools.openvins.native_observations import analyze_openvins_observations
    callbacks = native_imu_fixture(tmp_path)
    path = tmp_path/'imu0.jsonl'
    observed = [json.loads(line) for line in path.read_text().splitlines()]
    if defect == 'duplicate':
        observed.insert(1, dict(observed[0]))
    elif defect == 'reordered':
        observed[1], observed[2] = observed[2], observed[1]
    elif defect == 'frame':
        observed[0]['frame_id'] = 'another_imu'
    elif defect == 'timestamp-float':
        observed[0]['state_ns'] = float(observed[0]['state_ns'])
    elif defect == 'nonfinite':
        observed[0]['gyro'][0] = float('nan')
    elif defect == 'axis-count':
        # Six values in total do not prove two correctly shaped 3-axis samples.
        observed[0]['gyro'].append(observed[0]['accel'].pop(0))
    elif defect == 'axis-type':
        observed[0]['gyro'] = {'x': 0, 'y': 0, 'z': 0}
    elif defect == 'missing-vector':
        del observed[0]['accel']
    else:
        observed[0] = []
    path.write_text(''.join(json.dumps(row)+'\n' for row in observed))
    with pytest.raises(ValueError, match='OPENVINS_OBSERVATION_INVALID'):
        analyze_openvins_observations(ov_trace(tmp_path, callbacks+ov_visual_rows()),
                                      input_directory=tmp_path)


@pytest.mark.parametrize('variant', ['canonical', 'observed', 'missing', 'corrupt', 'native-failed',
                                   'cleanup-failed', 'imu-missing', 'imu-duplicate', 'imu-reordered',
                                   'imu-value', 'imu-raw-integer'])
def test_comparison_runner_seals_only_explicit_complete_native_observation(tmp_path, monkeypatch, variant):
    """Exercise runner orchestration; mock transport is not a native replay."""
    from types import SimpleNamespace
    from rm27.perception.vision.localization.experiments import openvins_comparison as comparison
    from test_native_events import ov_trace, ov_visual_rows
    from tools.openvins.native_observations import MAX_BYTES, MAX_EVENTS

    archive=tmp_path/'source.zip'; archive.write_bytes(b'fixture archive')
    bag=tmp_path/'bag'; bag.mkdir()
    workspace=tmp_path/'ws'
    binary=workspace/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    binary.parent.mkdir(parents=True); binary.write_bytes(b'fixture executable')
    source=tmp_path/'upstream'; source.mkdir()
    install=dict(commit=comparison.PIN, image='sha256:'+'a'*64, source=str(source),
                 workspace=str(workspace), files_sha256={}, binary_sha256=file_hash(binary))
    if variant != 'canonical':
        install['native_observation']=dict(schema_version=1,
            environment_variable='RM27_OPENVINS_NATIVE_EVENTS', max_events=MAX_EVENTS, max_bytes=MAX_BYTES)
    installation=tmp_path/'installation.json'; installation.write_text(json.dumps(install))
    dataset=SimpleNamespace(archive=archive,bag=bag,sequence='FIXTURE',mode='mono',
                            provenance={'fixture':True},verify_bag=lambda:None)
    monkeypatch.setattr(comparison,'ComparisonDataset',lambda *args:dataset)
    monkeypatch.setattr(comparison,'normalize_comparison',lambda *args:([],{'fixture':True}))
    def score(root,*args):
        (root/'evaluation.json').write_text('{"fixture":true}')
    monkeypatch.setattr(comparison,'evaluate_comparison',score)
    output=tmp_path/'out'
    def execute(*args,**kwargs):
        if variant not in ('canonical', 'missing'):
            callbacks = native_imu_fixture(output/'raw')
            if variant == 'imu-missing':
                del callbacks[2:4]
            elif variant == 'imu-duplicate':
                callbacks.insert(1, dict(callbacks[0]))
            elif variant == 'imu-reordered':
                callbacks[1], callbacks[2] = callbacks[2], callbacks[1]
            elif variant == 'imu-value':
                callbacks[1]['values'][0] += .125
            elif variant == 'imu-raw-integer':
                callbacks[0]['raw_nsec'] += 1
            path=ov_trace(output/'raw',callbacks+ov_visual_rows())
            if variant=='corrupt':
                rows=path.read_text().splitlines()
                footer=json.loads(rows[-1]); footer['dropped_events']=1
                path.write_text('\n'.join(rows[:-1]+[json.dumps(footer)])+'\n')
        return SimpleNamespace(returncode=7 if variant=='native-failed' else 0)
    monkeypatch.setattr(comparison.subprocess,'run',execute)
    monkeypatch.setattr(comparison,'cleanup_container',lambda *args:dict(
        absence_verified=variant!='cleanup-failed',container='fixture'))
    result=comparison.run(dict(schema_version=1,purpose='EUROC_PUBLIC_COMPARISON',run_id='fixture',
        backend='OpenVINS',archive=str(archive),archive_sha256=file_hash(archive),bag=str(bag),
        installation=str(installation),output=str(output),sequence='FIXTURE',mode='mono',rate=.5))
    enabled=variant!='canonical'
    assert ('RM27_OPENVINS_NATIVE_EVENTS=/out/native-events.jsonl' in result['command']) is enabled
    helper=str(Path(comparison.__file__).resolve().parents[5]/'tools/openvins/native_observations.py')
    assert (helper in result['runtime_sources_sha256']) is enabled
    if variant in ('canonical','observed'):
        assert result['status']=='EXECUTION_COMPLETED'
        assert ('native_observations' in result) is enabled
        if enabled:
            report=json.loads((output/'native-observations.json').read_text())
            assert report['accepted_visual_update_count']==1
            assert report['health_qualification']=='NOT_RUN'
            assert report['raw_input_equality']=='NOT_RUN'
            coverage=report['native_imu_callback_coverage']
            assert coverage['status']=='PASS' and coverage['missing_count']==0
            assert coverage['native_callback_count']==coverage['expected_observer_count']==6
            assert result['native_observations']['native_imu_callback_coverage']==coverage
            assert result['output_sha256']['native-observations.json']==file_hash(output/'native-observations.json')
        else:
            assert not (output/'native-observations.json').exists()
            assert not (output/'raw/imu0.jsonl').exists()
    else:
        assert result['status']=='FAILED'
        assert result['failure_code']==dict(missing='OPENVINS_OBSERVATION_MISSING',
            corrupt='OPENVINS_OBSERVATION_INCOMPLETE',native_failed='BACKEND_EXECUTION_FAILED',
            cleanup_failed='CONTAINER_CLEANUP_UNVERIFIED',imu_missing='OPENVINS_NATIVE_INPUT_INCOMPLETE',
            imu_duplicate='OPENVINS_NATIVE_INPUT_MISMATCH',imu_reordered='OPENVINS_NATIVE_INPUT_MISMATCH',
            imu_value='OPENVINS_NATIVE_INPUT_MISMATCH',
            imu_raw_integer='OPENVINS_NATIVE_INPUT_MISMATCH')[variant.replace('-','_')]
        if variant == 'imu-missing':
            report=json.loads((output/'native-observations.json').read_text())
            assert report['status']==report['observation_integrity']=='PASS'
            coverage=report['native_imu_callback_coverage']
            assert coverage['status']=='FAIL' and coverage['missing_count']==2
            assert coverage['missing_intervals'][0]['first_source_index']==2
            assert coverage['missing_intervals'][0]['last_source_index']==3
            assert result['native_observations']['native_imu_callback_coverage']==coverage
            assert result['output_sha256']['native-observations.json']==file_hash(output/'native-observations.json')
        else:
            assert not (output/'native-observations.json').exists()
        if variant not in ('native-failed', 'cleanup-failed'):
            assert 'evaluation.json' in result['output_sha256']


@pytest.mark.parametrize('change', [dict(schema_version=True),dict(max_events=200001),
                                   dict(environment_variable='OTHER_PATH'),dict(max_bytes=None)])
def test_native_observation_installation_rejects_unknown_schema_or_bounds(change):
    from rm27.perception.vision.localization.experiments.openvins_comparison import native_observation_spec
    from tools.openvins.native_observations import MAX_BYTES, MAX_EVENTS
    spec=dict(schema_version=1,environment_variable='RM27_OPENVINS_NATIVE_EVENTS',
              max_events=MAX_EVENTS,max_bytes=MAX_BYTES)
    spec.update(change)
    with pytest.raises(ValueError,match='OPENVINS_OBSERVATION_CONFIG_INVALID'):
        native_observation_spec({'native_observation':spec})
