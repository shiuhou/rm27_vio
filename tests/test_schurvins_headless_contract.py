"""Recorded native boundary tests; no fabricated runtime acceptance evidence."""
import json

import pytest

from rm27.perception.vision.localization.experiments.online import OnlinePoseRecord
from rm27.perception.vision.localization.experiments.schurvins_headless import export, load_run, project


@pytest.fixture
def recording(tmp_path):
    run = tmp_path/'run'; run.mkdir()
    first, second = 1403715273262142976, 1403715273312143104
    summary = dict(schema_version=1, runtime='ROS_FREE_CPP', status='EXECUTION_COMPLETED',
                   mode='mono', camera_input=2, camera_selected=2, imu_input=12,
                   imu_added=12, pose_count=1, tracking_losses=0)
    (run/'summary.json').write_text(json.dumps(summary))
    (run/'frames.csv').write_text(
        'timestamp_ns,scheduled_ns,start_ns,end_ns,stage,quality,pose_emitted,imu_added,queue_pending_frames\n'
        f'{first},100,101,110,0,0,0,2,0\n{second},200,201,210,2,2,1,12,0\n')
    (run/'poses.csv').write_text('timestamp_ns,tx,ty,tz,qw,qx,qy,qz\n'+f'{second},1,2,3,1,0,0,0\n')
    camera = tmp_path/'data.csv'
    camera.write_text(f'#timestamp [ns],filename\n{first},a.png\n{second},b.png\n')
    return run, camera


def projection(recording):
    return project(*recording, binary_sha256='a'*64, run_id='recorded-run', sequence='V1_01_easy')


def test_native_geometry_and_exact_clocks_do_not_invent_operational_state(recording):
    record, = projection(recording)
    assert OnlinePoseRecord.from_json(json.dumps(record)).to_dict() == record
    assert record['source_frame_id'] == 1
    assert record['source_timestamp']['raw_value'] == 1403715273312143104
    assert record['processing_completed_timestamp']['raw_value'] == 210
    assert record['processing_completed_timestamp']['clock_domain'] == 'recorded-run:native_steady'
    assert record['localization_fields']['orientation_wxyz'] == [1., 0., 0., 0.]
    assert record['localization_fields']['translation'] == [1., 2., 3.]
    assert record['localization_fields']['scale_state'] == 'UNKNOWN'
    assert record['validity'] is False
    assert record['tracking_state'] == record['initialization_state'] == 'UNKNOWN'
    assert record['publish_timestamp'] is record['observation_timestamp'] is None
    assert record['contract_complete'] is False
    assert 'topic' not in record['raw_output']


def test_stereo_retains_native_meter_convention_and_invalidity(recording):
    run, _ = recording
    summary = json.loads((run/'summary.json').read_text()); summary['mode'] = 'stereo'
    (run/'summary.json').write_text(json.dumps(summary))
    record, = projection(recording)
    assert record['localization_fields']['scale_state'] == 'METRIC'
    assert record['localization_fields']['translation_unit'] == 'm'
    assert record['validity'] is False


@pytest.mark.parametrize('filename,old,new', [
    ('poses.csv', '1403715273312143104', '1403715273312143103'),
    ('poses.csv', ',1,0,0,0\n', ',2,0,0,0\n'),
    ('poses.csv', ',1,2,3,', ',nan,2,3,'),
    ('frames.csv', ',201,210,', ',201,190,'),
    ('frames.csv', ',2,2,1,12,0', ',1,2,1,12,0'),
    ('summary.json', '"pose_count": 1', '"pose_count": 2'),
    ('summary.json', '"tracking_losses": 0', '"tracking_losses": 1'),
])
def test_corrupt_recording_refused(recording, filename, old, new):
    run, _ = recording; path = run/filename
    assert old in path.read_text()
    path.write_text(path.read_text().replace(old, new))
    with pytest.raises(ValueError):
        projection(recording)


def test_source_camera_mismatch_refused(recording):
    _, camera = recording
    camera.write_text(camera.read_text().replace('1403715273312143104', '1403715273312143105'))
    with pytest.raises(ValueError, match='HEADLESS_CAMERA_MISMATCH'):
        load_run(*recording)


def test_output_is_immutable_and_inputs_unchanged(recording, tmp_path):
    run, camera = recording
    binary = tmp_path/'binary'; binary.write_bytes(b'test provenance, not executable evidence')
    output = tmp_path/'output'
    before = {p:p.read_bytes() for p in run.iterdir()}
    result = export(run, camera, binary=binary, run_id='run', sequence='V1_01_easy', output=output)
    original = (output/'observations.jsonl').read_bytes()
    assert result['count'] == 1
    assert result['scope'] == 'OFFLINE_RECORDED_CONTRACT_ONLY'
    with pytest.raises(ValueError, match='OUTPUT_EXISTS'):
        export(run, camera, binary=binary, run_id='run', sequence='V1_01_easy', output=output)
    assert (output/'observations.jsonl').read_bytes() == original
    assert all(p.read_bytes() == contents for p, contents in before.items())
