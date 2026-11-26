"""Prepare a locked future EuRoC mono+IMU pair; never execute an estimator.

Only the two selected matrix gates are frozen. Prepared transport manifest
claims remain inherited evidence, not fresh bag decoding or native ingestion.
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
from pathlib import Path
import zipfile

from rm27.perception.vision.localization.experiments import prehardware
from rm27.perception.vision.localization.experiments.common import (
    checked_file, file_hash, read_json, require, write_json,
)


SENSORS = ('cam0', 'imu0')
SEQUENCES = ('MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult',
             'MH_05_difficult', 'V1_01_easy', 'V1_02_medium', 'V1_03_difficult',
             'V2_01_easy', 'V2_02_medium', 'V2_03_difficult')


def _input_manifest(directory, name):
    root = Path(directory).resolve()
    require(not (root/name).is_symlink(), 'MONO_INPUT_INVALID', 'input manifest must remain local')
    manifest = read_json(root/name)
    artifacts = manifest.get('artifacts_sha256')
    require(isinstance(artifacts, dict) and 0 < len(artifacts) <= 256,
            'MONO_INPUT_INVALID', 'bounded nonempty prepared artifact manifest required')
    for relative in artifacts:
        path = Path(relative)
        require(not path.is_absolute() and '..' not in path.parts and path.parts
                and (root/path).resolve().is_relative_to(root) and not (root/path).is_symlink(),
                'MONO_INPUT_INVALID', 'prepared artifacts must remain inside their input directory')
    require(manifest.get('mode') == 'mono' and manifest.get('truth_included') is False,
            'MONO_INPUT_INVALID', 'sensor-only monocular input required')
    return root, manifest


def _original_source(path):
    counts, first, last, calibration, indexes = {}, {}, {}, {}, {}
    selected_members = {'mav0/'+s+'/'+suffix for s in SENSORS for suffix in ('data.csv', 'sensor.yaml')}
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        require(len(names) == len(set(names)), 'MONO_SOURCE_INVALID', 'duplicate original ZIP members')
        for sensor in SENSORS:
            prefix = 'mav0/'+sensor+'/'
            raw = archive.read(prefix+'data.csv')
            rows = list(csv.reader(line for line in raw.decode().splitlines()
                                   if line.strip() and not line.lstrip().startswith('#')))
            stamps = [int(row[0]) for row in rows]
            require(stamps and all(t >= 0 for t in stamps) and
                    all(b > a for a, b in zip(stamps, stamps[1:])),
                    'MONO_SOURCE_INVALID', 'strictly ordered complete original sensor indexes required')
            if sensor == 'cam0':
                require(all(len(row) == 2 and row[1] and Path(row[1]).name == row[1]
                            and row[1] not in ('.', '..') and '\\' not in row[1] for row in rows)
                        and len({row[1] for row in rows}) == len(rows),
                        'MONO_SOURCE_INVALID', 'unique local camera image filenames required')
                selected_members.update('mav0/cam0/data/'+row[1] for row in rows)
            counts[sensor], first[sensor], last[sensor] = len(stamps), stamps[0], stamps[-1]
            calibration[sensor] = hashlib.sha256(archive.read(prefix+'sensor.yaml')).hexdigest()
            indexes[sensor] = raw
        require(selected_members <= set(names), 'MONO_SOURCE_INVALID', 'original selected image payload missing')
    return dict(counts=counts, first_ns=first, last_ns=last,
                calibration_sha256=calibration), indexes, selected_members


def _selected(matrix, gate_id, backend, role):
    matches = [gate for gate in matrix['gates'] if gate['id'] == gate_id]
    require(len(matches) == 1, 'MONO_GATE_INVALID', 'selected gate must exist exactly once')
    gate = matches[0]
    require(gate.get('kind') == 'cohort' and gate.get('backend') == backend
            and gate.get('role') == role and gate.get('repeat_count') == 5,
            'MONO_GATE_INVALID', 'selected backend/role must declare a smoke plus five-repeat cohort')
    config = read_json(gate['config_file'])
    common = {'schema_version', 'backend', 'purpose', 'run_id', 'sequence', 'mode',
              'rate', 'installation', 'output'}
    mandatory = common | ({'input', 'reference_archive', 'timeout_s'} if backend == 'SchurVINS'
                          else {'archive', 'archive_sha256', 'bag'})
    optional = ({'start_offset_s', 'native_parameters', 'state_output_policy', 'memory_limit_gib'}
                if backend == 'SchurVINS' else {'state_health', 'native_quality'})
    require(mandatory <= set(config) <= mandatory | optional and type(config.get('schema_version')) is int
            and config['schema_version'] == 1, 'MONO_PAIR_INVALID', 'bounded public baseline configuration required')
    require(config.get('backend') == backend and config.get('mode') == 'mono'
            and config.get('rate') == gate['rate'] and type(config.get('rate')) in (int, float)
            and config.get('sequence') in SEQUENCES,
            'MONO_PAIR_INVALID', 'explicit matching EuRoC monocular mode, sequence and rate required')
    expected_purpose = 'EUROC_PUBLIC_BASELINE' if backend == 'SchurVINS' else 'EUROC_PUBLIC_COMPARISON'
    require(config.get('purpose') == expected_purpose
            and type(config.get('start_offset_s', 0)) in (int, float) and config.get('start_offset_s', 0) == 0,
            'MONO_PAIR_INVALID', 'full original EuRoC inputs only; no recording, crop or artificial input')
    if 'required_cohorts' in matrix:
        requirements = [item for item in matrix['required_cohorts']
                        if Path(item['config_file']).resolve() == Path(gate['config_file']).resolve()]
        require(requirements and all(item['config_sha256'] == file_hash(gate['config_file'])
                and item['role'] == role and item['protocol'] == 'SMOKE_PLUS_FIVE_REPEATS' for item in requirements),
                'MONO_LOCK_INVALID', 'selected required-cohort configuration lock differs')
    directory, manifest = _input_manifest(config['input' if backend == 'SchurVINS' else 'bag'],
        'input.json' if backend == 'SchurVINS' else 'source_verification.json')
    locks = prehardware.config_locks(gate['config_file'])
    require(len(locks) <= 256 and all(gate['inputs_sha256'].get(p) == h for p, h in locks.items()),
            'MONO_LOCK_INVALID', 'selected matrix gate must lock every current config/input/installation artifact')
    for path, digest in gate['inputs_sha256'].items():
        require(isinstance(path, str) and Path(path).is_absolute(), 'MONO_LOCK_INVALID', 'absolute lock paths required')
        checked_file(path, digest)
    return gate, config, directory, manifest, locks


def prepare(matrix_path, schur_gate, openvins_gate, output):
    matrix_path, output = Path(matrix_path).resolve(), Path(output).absolute()
    if output.exists() or output.is_symlink():
        raise FileExistsError(output)
    output = output.resolve()
    matrix_hash = file_hash(matrix_path)
    matrix = prehardware.validate_matrix(read_json(matrix_path))
    a = _selected(matrix, schur_gate, 'SchurVINS', 'schur_mono')
    b = _selected(matrix, openvins_gate, 'OpenVINS', 'openvins')
    require(a[1]['sequence'] == b[1]['sequence'] and a[1]['rate'] == b[1]['rate'],
            'MONO_PAIR_INVALID', 'paired sequence and playback rate must match')
    source, source_hash = checked_file(a[1]['reference_archive'])
    other_source, other_hash = checked_file(b[1]['archive'], b[1].get('archive_sha256'))
    require(source_hash == other_hash, 'MONO_PAIR_INVALID', 'same complete original reference archive required')
    protected = [a[2], b[2], source.parent, other_source.parent, matrix_path.parent,
                 Path(a[0]['config_file']).parent, Path(b[0]['config_file']).parent]
    for selected in (a, b):
        installation = read_json(selected[1]['installation'])
        protected.extend(Path(installation[k]).resolve() for k in ('workspace', 'source')
                         if isinstance(installation.get(k), str) and Path(installation[k]).is_absolute())
    require(not any(output.is_relative_to(root) for root in protected),
            'MONO_OUTPUT_INVALID', 'new preparation output must be outside existing inputs/source/configuration')
    source_info, indexes, selected_members = _original_source(source)
    schur, ov = a[3], b[3]
    require(schur.get('purpose') == 'EUROC_PUBLIC_REPLAY'
            and schur.get('original_archive_sha256') == source_hash
            and not any(k in schur for k in ('derivation', 'source_counts', 'source_first_ns', 'source_last_ns')),
            'MONO_INPUT_INVALID', 'Schur input must retain the uncropped original EuRoC population')
    for key in ('counts', 'first_ns', 'last_ns', 'calibration_sha256'):
        require(schur.get(key) == source_info[key], 'MONO_INPUT_INVALID', 'Schur source '+key+' differs')
        if key != 'calibration_sha256':
            require(all(type(v) is int for v in schur[key].values()),
                    'MONO_INPUT_INVALID', 'integer source counts/endpoints required')
    require(ov.get('purpose') == 'EUROC_PUBLIC_COMPARISON' and ov.get('sequence') == a[1]['sequence']
            and ov.get('source_sha256') == source_hash and ov.get('readback_exact') is True
            and ov.get('verified_counts') == source_info['counts']
            and set(ov.get('topics', [])) == {'/imu0', '/cam0/image_raw'}
            and not any(k in ov for k in ('derivation', 'prepared_input_sha256', 'delivery_schedule')),
            'MONO_INPUT_INVALID', 'OpenVINS manifest must declare exact complete cam0+IMU source readback')
    require(all(type(v) is int for v in ov['verified_counts'].values()),
            'MONO_INPUT_INVALID', 'integer OpenVINS source counts required')
    expected_calibration = {'mav0/'+s+'/sensor.yaml': h for s, h in source_info['calibration_sha256'].items()}
    require(ov.get('source_calibration_sha256') == expected_calibration,
            'MONO_INPUT_INVALID', 'OpenVINS source calibration differs')
    require('sensors.zip' in schur['artifacts_sha256'], 'MONO_INPUT_INVALID', 'sealed Schur sensor archive required')
    with zipfile.ZipFile(a[2]/'sensors.zip') as archive:
        names = archive.namelist()
        require(len(names) == len(set(names)) and set(names) == selected_members,
                'MONO_INPUT_INVALID', 'Schur archive must contain exactly the full selected sensor payloads')
        for sensor in SENSORS:
            require(archive.read('mav0/'+sensor+'/data.csv') == indexes[sensor]
                    and hashlib.sha256(archive.read('mav0/'+sensor+'/sensor.yaml')).hexdigest()
                    == source_info['calibration_sha256'][sensor],
                    'MONO_INPUT_INVALID', 'Schur prepared indexes/calibration must equal the full original source')
    # No import or invocation of either runtime occurs. Only future configs are
    # written; their run directories do not yet exist and commands are inert data.
    output.mkdir(parents=True, exist_ok=False)
    selected, members = [], []
    for gate, config, _, manifest, locks in (a, b):
        selected.append(dict(gate_id=gate['id'], backend=config['backend'],
            config_file=gate['config_file'], config_sha256=file_hash(gate['config_file']),
            inputs_sha256=locks, input_manifest_scope='locked inherited transport evidence; no fresh decoding or native ingestion'))
        for name in ('smoke', 'repeat_1', 'repeat_2', 'repeat_3', 'repeat_4', 'repeat_5'):
            frozen = output/'configs'/gate['id']/(name+'.json')
            frozen.parent.mkdir(parents=True, exist_ok=True)
            future = dict(copy.deepcopy(config), run_id=gate['id']+'_future_'+name,
                          output=str(output/'runs'/gate['id']/name))
            write_json(frozen, future)
            module = 'schurvins_runtime' if config['backend'] == 'SchurVINS' else 'openvins_comparison'
            members.append(dict(gate_id=gate['id'], name=name, status='NOT_RUN', execution_started=False,
                config=str(frozen), config_sha256=file_hash(frozen), output=future['output'],
                future_command=['python3', '-m', 'rm27.perception.vision.localization.experiments.'+module,
                                '--config', str(frozen)]))
    root = Path(__file__).resolve().parents[1]
    code = [Path(__file__).resolve(), Path(prehardware.__file__),
            root/'rm27/perception/vision/localization/experiments/common.py',
            root/'tools/schurvins/evaluate.py', root/'tools/openvins/native_accuracy.py']
    report = dict(schema_version=1, status='PREPARED_NOT_RUN', scope='EUROC_MONO_IMU_LOCK_ONLY',
        execution_started=False, estimator_replay='NOT_RUN', hardware_qualification='NOT_RUN',
        native_input_consumption='NOT_RUN', installation_tree_revalidation='NOT_RUN',
        matrix=dict(path=str(matrix_path), sha256=matrix_hash, total_declared_gates=len(matrix['gates']),
                    selected_gate_ids=[schur_gate, openvins_gate], acceptance='NOT_RUN; no full-matrix qualification'),
        sequence=a[1]['sequence'], mode='mono', rate=a[1]['rate'],
        original_source=dict(path=str(source), sha256=source_hash, **source_info),
        selected=selected, planned_members=members,
        protocol=dict(primary_alignment='SE3 scale=1; no Sim3 or fitted time shift; epochs separate',
            coverage='full original cam0 denominator, including initialization prefix, gaps, suffix and failed members',
            common_interval='supplement only; existing evaluator uses overlapping spans, not necessarily identical present camera IDs',
            initialization='first pose is a proxy; native metric commit and first accepted visual support remain separate',
            first_loss='missing output is not native tracking loss; require native health evidence or NOT_RUN',
            metrics='existing evaluator only; ATE/RPE and scale diagnostics do not establish operational validity',
            timing='native frontend timers have different boundaries; end-to-end latency NOT_RUN',
            resources='native GNU-time process-tree CPU/wall/RSS separate from whole-container cgroup CPU/memory.current',
            cohorts='smoke then five predeclared repeats only after original smoke completion rules; retain every failure; never replace failed members',
            execution='future commands are declarations only; separate explicit replay authorization required'),
        calibration=dict(source_calibration='MATCHES_ORIGINAL',
            native_calibration_equivalence='NOT_VERIFIED', online_calibration_policy_equivalence='NOT_VERIFIED'),
        source_sha256={str(p):file_hash(p) for p in code})
    require(file_hash(matrix_path) == matrix_hash, 'MONO_LOCK_INVALID', 'matrix changed during preparation')
    for _, _, _, _, locks in (a, b):
        for path, digest in locks.items():
            checked_file(path, digest)
    write_json(output/'preparation.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix', required=True, type=Path)
    parser.add_argument('--schur-gate', required=True)
    parser.add_argument('--openvins-gate', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    report = prepare(args.matrix, args.schur_gate, args.openvins_gate, args.output)
    print(report['status'])
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
