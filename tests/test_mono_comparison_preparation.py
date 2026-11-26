"""Lock preparation fixtures only; no image decoding or estimator result is claimed."""
import hashlib
from pathlib import Path
import subprocess
import sys
import zipfile

import pytest

from tools import prepare_mono_comparison as prep
from rm27.perception.vision.localization.experiments.common import file_hash, read_json, write_json


def paired_matrix(tmp_path, *, image_names=None):
    source_dir = tmp_path/'source'
    source_dir.mkdir()
    source = source_dir/'V1_01_easy.zip'
    payload = {}
    for sensor in prep.SENSORS:
        payload['mav0/'+sensor+'/data.csv'] = ('# synthetic index only\n'+''.join(
            f'{1000000000+i*10000000},'+((image_names[i] if image_names else f'{i}.png')
                if sensor == 'cam0' else '0,0,0,0,0,9.81')+'\n'
            for i in range(5))).encode()
        payload['mav0/'+sensor+'/sensor.yaml'] = b'TEST_FIXTURE_ONLY calibration\n'+sensor.encode()
    for i in range(5):
        payload[f'mav0/cam0/data/{i}.png'] = b'TEST_FIXTURE_ONLY encoded placeholder; never decoded'
    with zipfile.ZipFile(source, 'w') as z:
        for name, data in payload.items():
            z.writestr(name, data)
        z.writestr('mav0/state_groundtruth_estimate0/data.csv', b'TEST_FIXTURE_ONLY original truth stays external')
    source_hash = file_hash(source)
    calibration = {s:hashlib.sha256(payload['mav0/'+s+'/sensor.yaml']).hexdigest() for s in prep.SENSORS}
    configs = tmp_path/'original-configs'
    configs.mkdir()
    gates = []
    for backend, gate_id, role in (('SchurVINS', 'schur', 'schur_mono'), ('OpenVINS', 'openvins', 'openvins')):
        directory = tmp_path/('input-'+gate_id)
        directory.mkdir()
        install = configs/(gate_id+'-installation.json')
        write_json(install, dict(purpose='TEST_FIXTURE_ONLY; native installation not executed'))
        if backend == 'SchurVINS':
            with zipfile.ZipFile(directory/'sensors.zip', 'w', compression=zipfile.ZIP_DEFLATED) as z:
                for name, data in payload.items():
                    z.writestr(name, data)
            manifest = dict(schema_version=1, purpose='EUROC_PUBLIC_REPLAY', mode='mono',
                original_archive_sha256=source_hash, truth_included=False,
                counts={s:5 for s in prep.SENSORS}, first_ns={s:1000000000 for s in prep.SENSORS},
                last_ns={s:1040000000 for s in prep.SENSORS}, calibration_sha256=calibration,
                artifacts_sha256={'sensors.zip':file_hash(directory/'sensors.zip')})
            manifest_name = 'input.json'
            config = dict(input=str(directory), reference_archive=str(source), purpose='EUROC_PUBLIC_BASELINE', timeout_s=900)
        else:
            (directory/'data.db3').write_bytes(b'TEST_FIXTURE_ONLY transport placeholder; never replayed')
            manifest = dict(schema_version=1, purpose='EUROC_PUBLIC_COMPARISON', mode='mono',
                sequence='V1_01_easy', source_sha256=source_hash, truth_included=False,
                verified_counts={s:5 for s in prep.SENSORS}, readback_exact=True,
                topics=['/imu0', '/cam0/image_raw'], estimator_ingestion_verified=False,
                source_calibration_sha256={'mav0/'+s+'/sensor.yaml':h for s, h in calibration.items()},
                artifacts_sha256={'data.db3':file_hash(directory/'data.db3')})
            manifest_name = 'source_verification.json'
            config = dict(bag=str(directory), archive=str(source), archive_sha256=source_hash,
                          purpose='EUROC_PUBLIC_COMPARISON')
        write_json(directory/manifest_name, manifest)
        config.update(schema_version=1, backend=backend, mode='mono', sequence='V1_01_easy', rate=1.,
                      installation=str(install), run_id='original-'+gate_id, output='/unexecuted/original-'+gate_id)
        path = configs/(gate_id+'.json')
        write_json(path, config)
        gates.append(dict(id=gate_id, role=role, backend=backend, kind='cohort', required=True,
            rate=1., repeat_count=5, evidence_class='PC OFFLINE VERIFIED', config_file=str(path),
            inputs_sha256=prep.prehardware.config_locks(path)))
    matrix_path = configs/'matrix.json'
    write_json(matrix_path, dict(schema_version=1, gates=gates))
    return matrix_path


def relock(matrix_path):
    matrix = read_json(matrix_path)
    for gate in matrix['gates']:
        gate['inputs_sha256'] = prep.prehardware.config_locks(gate['config_file'])
    write_json(matrix_path, matrix)


def run(matrix, tmp_path, **kwargs):
    return prep.prepare(matrix, 'schur', 'openvins', kwargs.get('output', tmp_path/'prepared'))


def test_preparation_is_inert_locks_exactly_two_future_cohorts_and_preserves_inputs(tmp_path, monkeypatch):
    matrix = paired_matrix(tmp_path)
    original = {str(p):file_hash(p) for p in tmp_path.rglob('*') if p.is_file()}
    imported_before = set(sys.modules)
    def forbidden(*args, **kwargs):
        pytest.fail('preparation must never import or execute a native runtime')
    monkeypatch.setattr(subprocess, 'run', forbidden)
    monkeypatch.setattr(subprocess, 'Popen', forbidden)
    monkeypatch.setattr(prep.prehardware, '_execute_backend', forbidden)
    monkeypatch.setattr(prep.prehardware, 'run', forbidden)
    report = run(matrix, tmp_path)
    assert report['status'] == 'PREPARED_NOT_RUN' and report['execution_started'] is False
    assert report['estimator_replay'] == report['hardware_qualification'] == report['native_input_consumption'] == 'NOT_RUN'
    assert report['original_source']['counts'] == {'cam0':5, 'imu0':5}
    assert report['original_source']['first_ns'] == {'cam0':1000000000, 'imu0':1000000000}
    assert report['calibration']['source_calibration'] == 'MATCHES_ORIGINAL'
    assert report['calibration']['native_calibration_equivalence'] == 'NOT_VERIFIED'
    assert 'SE3 scale=1' in report['protocol']['primary_alignment']
    assert 'full original' in report['protocol']['coverage']
    assert 'not native tracking loss' in report['protocol']['first_loss']
    members = report['planned_members']
    assert len(members) == 12 and all(m['status'] == 'NOT_RUN' and m['execution_started'] is False for m in members)
    assert not (tmp_path/'prepared/runs').exists()
    assert len({m['output'] for m in members}) == 12
    run_ids = set()
    originals = {g['id']:read_json(g['config_file']) for g in read_json(matrix)['gates']}
    for member in members:
        config = read_json(member['config'])
        run_ids.add(config['run_id'])
        assert {k:v for k,v in config.items() if k not in ('run_id', 'output')} == {
            k:v for k,v in originals[member['gate_id']].items() if k not in ('run_id', 'output')}
        assert file_hash(member['config']) == member['config_sha256']
        assert not Path(config['output']).exists()
    assert len(run_ids) == 12
    assert all(file_hash(path) == digest for path, digest in original.items())
    assert len(report['source_sha256']) == 5
    assert all(file_hash(path) == digest for path, digest in report['source_sha256'].items())
    assert not any(name.endswith(('.schurvins_runtime', '.openvins_comparison'))
                   for name in set(sys.modules)-imported_before)


@pytest.mark.parametrize('field,value', [('mode','stereo'), ('rate',.5), ('rate',True),
    ('sequence','V1_03_difficult'), ('purpose','EUROC_ARTIFICIAL_COMPARISON'), ('start_offset_s',2.),
    ('unknown_crop',True)])
def test_rejects_mixed_pair_or_cropped_runtime_before_writes(tmp_path, field, value):
    matrix = paired_matrix(tmp_path)
    path = Path(read_json(matrix)['gates'][1]['config_file'])
    config = read_json(path)
    config[field] = value
    write_json(path, config)
    if field != 'purpose':
        relock(matrix)
    with pytest.raises(ValueError):
        run(matrix, tmp_path)
    assert not (tmp_path/'prepared').exists()


@pytest.mark.parametrize('side,field,value', [
    ('schur','counts',{'cam0':4,'imu0':5}),
    ('schur','first_ns',{'cam0':1000000001,'imu0':1000000000}),
    ('schur','original_archive_sha256','0'*64),
    ('schur','calibration_sha256',{'cam0':'0'*64,'imu0':'1'*64}),
    ('schur','derivation',{}), ('schur','truth_included',True),
    ('openvins','verified_counts',{'cam0':5,'imu0':4}),
    ('openvins','source_calibration_sha256',{}), ('openvins','truth_included',True),
    ('openvins','readback_exact',False), ('openvins','topics',['/imu0','/cam0/image_raw','/truth']),
])
def test_rejects_freshly_locked_but_incompatible_prepared_evidence(tmp_path, side, field, value):
    matrix = paired_matrix(tmp_path)
    path = tmp_path/('input-'+side)/('input.json' if side == 'schur' else 'source_verification.json')
    manifest = read_json(path)
    manifest[field] = value
    write_json(path, manifest)
    relock(matrix)
    with pytest.raises(ValueError):
        run(matrix, tmp_path)
    assert not (tmp_path/'prepared').exists()


@pytest.mark.parametrize('change', ['crop', 'truth', 'missing_image'])
def test_rejects_payload_crop_or_truth_despite_refreshed_manifest_hashes(tmp_path, change):
    matrix = paired_matrix(tmp_path)
    directory = tmp_path/'input-schur'
    archive = directory/'sensors.zip'
    with zipfile.ZipFile(archive) as z:
        payload = {n:z.read(n) for n in z.namelist()}
    if change == 'truth':
        payload['mav0/state_groundtruth_estimate0/data.csv'] = b'forbidden estimator truth'
    elif change == 'missing_image':
        payload.pop('mav0/cam0/data/4.png')
    else:
        key = 'mav0/cam0/data.csv'
        payload[key] = b'\n'.join(payload[key].splitlines()[:-1])+b'\n'
    with zipfile.ZipFile(archive, 'w') as z:
        for name, data in payload.items():
            z.writestr(name, data)
    manifest = read_json(directory/'input.json')
    manifest['artifacts_sha256']['sensors.zip'] = file_hash(archive)
    write_json(directory/'input.json', manifest)
    relock(matrix)
    with pytest.raises(ValueError):
        run(matrix, tmp_path)
    assert not (tmp_path/'prepared').exists()


@pytest.mark.parametrize('image_names', [
    ['0.png','../escape.png','2.png','3.png','4.png'],
    ['0.png','/absolute.png','2.png','3.png','4.png'],
    ['0.png','0.png','2.png','3.png','4.png'],
])
def test_rejects_unsafe_or_duplicate_original_image_references(tmp_path, image_names):
    matrix = paired_matrix(tmp_path, image_names=image_names)
    with pytest.raises(ValueError, match='MONO_SOURCE_INVALID'):
        run(matrix, tmp_path)
    assert not (tmp_path/'prepared').exists()


def test_required_cohort_must_lock_the_selected_current_config(tmp_path):
    matrix = paired_matrix(tmp_path)
    value = read_json(matrix)
    value['required_cohorts'] = [dict(id='required_'+gate['id'], role=gate['role'],
        config_file=gate['config_file'], config_sha256=file_hash(gate['config_file']),
        protocol='SMOKE_PLUS_FIVE_REPEATS') for gate in value['gates']]
    value['required_cohorts'][0]['config_sha256'] = '0'*64
    write_json(matrix, value)
    with pytest.raises(ValueError, match='MONO_LOCK_INVALID'):
        run(matrix, tmp_path)
    assert not (tmp_path/'prepared').exists()


def test_rejects_stale_locks_artifact_escape_and_output_collision(tmp_path):
    matrix = paired_matrix(tmp_path)
    artifact = tmp_path/'input-openvins/data.db3'
    artifact.write_bytes(b'changed after manifest seal')
    with pytest.raises(ValueError, match='HASH_MISMATCH'):
        run(matrix, tmp_path)
    manifest_path = tmp_path/'input-openvins/source_verification.json'
    manifest = read_json(manifest_path)
    manifest['artifacts_sha256'] = {'../outside.bin':'0'*64}
    write_json(manifest_path, manifest)
    with pytest.raises(ValueError, match='MONO_INPUT_INVALID'):
        run(matrix, tmp_path)
    output = tmp_path/'prepared'
    output.mkdir()
    sentinel = output/'sentinel'
    sentinel.write_text('preserve existing output')
    with pytest.raises(FileExistsError):
        run(matrix, tmp_path)
    assert sentinel.read_text() == 'preserve existing output'


@pytest.mark.parametrize('protected', ['input-schur', 'input-openvins', 'source', 'original-configs'])
def test_output_must_be_outside_existing_source_and_inputs(tmp_path, protected):
    matrix = paired_matrix(tmp_path)
    output = tmp_path/protected/'new-output'
    with pytest.raises(ValueError, match='MONO_OUTPUT_INVALID'):
        run(matrix, tmp_path, output=output)
    assert not output.exists()


def test_selection_requires_actual_declared_backend_role_and_cli_has_no_execute(tmp_path):
    matrix = paired_matrix(tmp_path)
    value = read_json(matrix)
    value['gates'][0]['role'] = 'schur_stereo'
    write_json(matrix, value)
    with pytest.raises(ValueError, match='MONO_GATE_INVALID'):
        run(matrix, tmp_path)
    with pytest.raises(SystemExit) as error:
        prep.main(['--matrix',str(matrix),'--schur-gate','schur','--openvins-gate','openvins',
                   '--output',str(tmp_path/'prepared'),'--execute'])
    assert error.value.code == 2 and not (tmp_path/'prepared').exists()
