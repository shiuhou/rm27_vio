"""Sequential acceptance matrix over existing replay tools and audited evidence.

No estimator, metric or hardware driver is implemented here. Runtime cohorts
invoke the existing dedicated SchurVINS/OpenVINS comparison functions. An
initial smoke success permits exactly five predeclared repeats, including every
failure. Current gates require locked artifacts, current code identities, process
exit and cleanup evidence. Historical receipts are retained without closing a
current gate. Offline/mock/build evidence cannot become on-board qualification.
"""
from __future__ import annotations

import argparse
import copy
import importlib
from pathlib import Path
import re

from .common import checked_file, file_hash, read_json, require, write_json
from tools.schurvins.summarize import summarize_run, summarize_group

ROLES = ('contracts', 'schur_mono', 'schur_stereo', 'openvins', 'high_motion',
         'fault_recovery', 'flight_interface', 'arm64', 'lifecycle')
EVIDENCE = ('PC OFFLINE VERIFIED', 'ARM64 BUILD VERIFIED', 'EMULATED / MOCK VERIFIED',
            'HARDWARE BLOCKED / NOT_RUN')
STATUSES = ('PASS', 'FAIL', 'BLOCKED', 'NOT_RUN')
_MODULES = {'SchurVINS': '.schurvins_runtime', 'OpenVINS': '.openvins_comparison'}
_NATIVE_ROLES = ('schur_mono','schur_stereo','openvins','high_motion','fault_recovery')


def _hashes(entries, name):
    require(isinstance(entries, dict) and entries, 'ACCEPTANCE_LOCK_INVALID', name+' must be nonempty')
    for path, digest in entries.items():
        require(isinstance(path, str) and Path(path).is_absolute(), 'ACCEPTANCE_LOCK_INVALID', 'absolute paths required')
        checked_file(path, digest)
    return entries


def config_locks(path):
    """Return the precise current files a matrix author must explicitly freeze.

    This is a lock creation aid, not acceptance: no experiment is run here.
    Prepared input and installation manifests recursively own their artifacts.
    """
    path = Path(path).resolve()
    config = read_json(path)
    backend = config.get('backend')
    require(backend in _MODULES, 'ACCEPTANCE_BACKEND_INVALID', 'dedicated comparison backends only')
    required = [path, Path(config['installation']).resolve()]
    if backend == 'SchurVINS':
        prepared = Path(config['input']).resolve()
        manifest = read_json(prepared/'input.json')
        required.extend([prepared/'input.json', Path(config['reference_archive']).resolve()])
        artifacts = manifest['artifacts_sha256']
    else:
        prepared = Path(config['bag']).resolve()
        manifest = read_json(prepared/'source_verification.json')
        required.extend([prepared/'source_verification.json', Path(config['archive']).resolve()])
        artifacts = manifest['artifacts_sha256']
    for name, digest in artifacts.items():
        relative = Path(name)
        require(not relative.is_absolute() and '..' not in relative.parts,
                'ACCEPTANCE_LOCK_INVALID', 'input-local artifact paths required')
        required.append(checked_file(prepared/relative, digest)[0])
    return {str(p):file_hash(p) for p in required}


def validate_matrix(matrix):
    require(isinstance(matrix, dict) and {'schema_version','gates'} <= set(matrix)
            <= {'schema_version','gates','required_cohorts'}
            and type(matrix['schema_version']) is int and matrix['schema_version'] == 1,
            'ACCEPTANCE_MATRIX_INVALID', 'matrix schema version 1')
    require(isinstance(matrix['gates'], list) and matrix['gates'],
            'ACCEPTANCE_MATRIX_INVALID', 'nonempty declared gate list')
    ids = set()
    for gate in matrix['gates']:
        require(isinstance(gate, dict) and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', gate.get('id', '')),
                'ACCEPTANCE_MATRIX_INVALID', 'safe unique gate ID')
        require(gate['id'] not in ids, 'ACCEPTANCE_MATRIX_INVALID', 'duplicate gate ID')
        ids.add(gate['id'])
        require(gate.get('role') in ROLES and type(gate.get('required')) is bool
                and gate.get('evidence_class') in EVIDENCE,
                'ACCEPTANCE_MATRIX_INVALID', 'role, required flag and bounded evidence class')
        require(gate.get('kind') in ('cohort', 'evidence', 'unavailable','input_rejection'),
                'ACCEPTANCE_MATRIX_INVALID', 'cohort/evidence/unavailable/input_rejection gate only')
        require(gate['role'] != 'arm64' or gate['kind'] != 'evidence'
                or gate['evidence_class'] == 'ARM64 BUILD VERIFIED',
                'ACCEPTANCE_MATRIX_INVALID', 'ARM64 gate requires ARM64 build evidence')
        require(gate['role'] not in _NATIVE_ROLES
                or gate['kind'] != 'evidence', 'ACCEPTANCE_MATRIX_INVALID',
                'native baseline, high-motion and recovery gates require fixed replay cohorts, not a generic receipt')
        if gate['kind'] in ('cohort','input_rejection'):
            require(gate.get('backend') in _MODULES and gate.get('evidence_class') == 'PC OFFLINE VERIFIED'
                    and gate.get('rate') in (.5, 1.) and type(gate.get('rate')) in (int,float),
                    'ACCEPTANCE_MATRIX_INVALID', 'explicit backend and .5/1 cohort')
            require(type(gate.get('repeat_count')) is int and (gate['repeat_count'] == 5
                    or (gate['role']=='fault_recovery' and gate['repeat_count']==0)),
                    'ACCEPTANCE_MATRIX_INVALID', 'five fixed repeats after baseline smoke, or one fault run')
            require(isinstance(gate.get('config_file'), str) and Path(gate['config_file']).is_absolute()
                    and isinstance(gate.get('inputs_sha256'), dict) and gate['inputs_sha256'],
                    'ACCEPTANCE_MATRIX_INVALID', 'locked runtime configuration and inputs')
            if gate['kind']=='input_rejection':
                require(gate['role']=='fault_recovery' and gate['backend']=='SchurVINS' and gate['repeat_count']==0,
                        'ACCEPTANCE_MATRIX_INVALID','invalid input gate invokes only existing Schur preflight')
        elif gate['kind'] == 'evidence':
            require(isinstance(gate.get('receipt'), str) and Path(gate['receipt']).is_absolute()
                    and isinstance(gate.get('receipt_sha256'), str),
                    'ACCEPTANCE_MATRIX_INVALID', 'explicit hash-locked receipt')
            require(gate.get('currency') in ('CURRENT', 'HISTORICAL'),
                    'ACCEPTANCE_MATRIX_INVALID', 'current or historical evidence')
        else:
            require(gate.get('status') in ('BLOCKED','NOT_RUN') and bool(gate.get('reason'))
                    and isinstance(gate.get('reproduce'), list) and bool(gate['reproduce'])
                    and all(isinstance(x,str) and x for x in gate['reproduce']),
                    'ACCEPTANCE_MATRIX_INVALID', 'concrete reason and reproduction argv required')
    if 'required_cohorts' in matrix:
        requirements=matrix['required_cohorts']
        require(isinstance(requirements,list) and requirements,'ACCEPTANCE_MATRIX_INVALID',
                'required cohort coverage must be a nonempty declaration')
        required_ids=set()
        for item in requirements:
            require(isinstance(item,dict) and set(item)=={'id','role','config_file','config_sha256','protocol'}
                    and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*',item.get('id',''))
                    and item['id'] not in required_ids and item.get('role') in _NATIVE_ROLES,
                    'ACCEPTANCE_MATRIX_INVALID','unique explicit native required cohort')
            required_ids.add(item['id'])
            require(isinstance(item['config_file'],str) and Path(item['config_file']).is_absolute()
                    and isinstance(item['config_sha256'],str)
                    and re.fullmatch(r'[a-f0-9]{64}',item['config_sha256'])
                    and item['protocol'] in ('SMOKE_PLUS_FIVE_REPEATS','SINGLE_RUN','INPUT_REJECTION')
                    and (item['protocol']=='SMOKE_PLUS_FIVE_REPEATS' or item['role']=='fault_recovery'),
                    'ACCEPTANCE_MATRIX_INVALID','locked required config and unrelaxed baseline protocol')
    return matrix


def audit_receipt(path, expected, evidence_class, *, currency='CURRENT'):
    """Check an explicitly recorded non-runtime gate, never trust a PASS alone."""
    path = Path(path).resolve()
    if not path.is_file():
        return dict(status='NOT_RUN', reason='evidence receipt is missing', receipt=str(path))
    try:
        checked_file(path, expected)
        receipt = read_json(path)
        require(type(receipt.get('schema_version')) is int and receipt['schema_version'] == 1
                and receipt.get('status') in STATUSES and receipt.get('evidence_class') == evidence_class,
                'ACCEPTANCE_EVIDENCE_INVALID', 'receipt version/status/class differs')
        report = dict(status=receipt['status'], receipt=str(path), receipt_sha256=expected,
                      evidence_class=evidence_class, currency=currency)
        _hashes(receipt.get('artifacts_sha256'), 'receipt artifacts')
        report['artifacts_sha256'] = receipt['artifacts_sha256']
        if receipt['status'] != 'PASS':
            require(bool(receipt.get('reason')), 'ACCEPTANCE_EVIDENCE_INVALID', 'failure/block/not-run reason required')
            report['reason'] = receipt['reason']
            return report
        require(evidence_class != 'HARDWARE BLOCKED / NOT_RUN',
                'ACCEPTANCE_EVIDENCE_INVALID', 'hardware qualification cannot pass in this workflow')
        execution = receipt.get('execution', {})
        require(type(execution.get('exit_code')) is int and execution['exit_code'] == 0
                and isinstance(execution.get('command'), list) and bool(execution['command'])
                and execution.get('completed') is True
                and receipt.get('cleanup', {}).get('absence_verified') is True,
                'ACCEPTANCE_EXIT_UNVERIFIED', 'completed command, zero exit and cleanup evidence required')
        if currency == 'HISTORICAL':
            report.update(status='NOT_RUN', historical_status='PASS',
                          reason='historical evidence does not close a current gate')
        else:
            _hashes(receipt.get('code_sha256'), 'current code identities')
            report['code_sha256'] = receipt['code_sha256']
        return report
    except FileNotFoundError as exc:
        return dict(status='NOT_RUN', reason=str(exc), receipt=str(path))
    except (ValueError, KeyError, OSError) as exc:
        return dict(status='FAIL', reason=str(exc), receipt=str(path), evidence_class=evidence_class)


def _audit_runtime(directory, config, backend, *, role=None):
    """Runtime success retains its original gates and requires sealed execution."""
    root = Path(directory)
    if not (root/'run.json').is_file():
        return dict(status='NOT_RUN', reason='runner did not produce run.json', output=str(root))
    run = read_json(root/'run.json')
    status = run.get('status')
    if status not in ('PASS','EXECUTION_COMPLETED'):
        return dict(status='FAIL', original_status=status, failure_code=run.get('failure_code'),
                    reason=run.get('failure_reason', 'runtime did not pass its original gate'), output=str(root))
    try:
        require(run.get('config') == config and run.get('execution_started') is True,
                'ACCEPTANCE_RUN_MISMATCH', 'executed configuration differs')
        require(type(run.get('exit_code')) is int and run['exit_code'] == 0
                and run.get('container_cleanup', {}).get('absence_verified') is True,
                'ACCEPTANCE_EXIT_UNVERIFIED', 'runner process and owned container exit unverified')
        artifacts = run.get('output_sha256')
        require(isinstance(artifacts,dict) and artifacts, 'ACCEPTANCE_ARTIFACT_UNSEALED', 'sealed runtime artifacts required')
        for name, digest in artifacts.items():
            path = Path(name)
            require(not path.is_absolute() and '..' not in path.parts and
                    (root/path).resolve().is_relative_to(root.resolve()),
                    'ACCEPTANCE_ARTIFACT_INVALID', 'runtime-local artifact path')
            checked_file(root/path,digest)
        actual = {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p.name != 'run.json'}
        require(actual == set(artifacts), 'ACCEPTANCE_ARTIFACT_UNSEALED', 'runtime artifact membership differs')
        require(run.get('evaluation') in artifacts and run.get('online_trajectory') in artifacts,
                'ACCEPTANCE_ARTIFACT_UNSEALED', 'evaluation and trajectory must be sealed')
        _hashes(run.get('runtime_sources_sha256'), 'runtime code identities')
        runtime_source = Path(__file__).with_name('schurvins_runtime.py' if backend == 'SchurVINS' else 'openvins_comparison.py')
        require(run['runtime_sources_sha256'].get(str(runtime_source)) == file_hash(runtime_source),
                'ACCEPTANCE_CODE_UNVERIFIED', 'current dedicated runtime identity required')
        evaluation = read_json(root/run['evaluation'])
        require(type(evaluation.get('scored_epoch_count')) is int and evaluation['scored_epoch_count'] > 0,
                'ACCEPTANCE_EVALUATION_MISSING', 'at least one scored trajectory epoch required')
        if backend == 'SchurVINS':
            execution = run.get('execution',{})
            require(execution.get('failure') is None and all(type(execution.get('exits',{}).get(p)) is int
                    and execution['exits'][p] == 0 for p in ('native','player','observer','master')),
                    'ACCEPTANCE_EXIT_UNVERIFIED', 'all native pipeline exits required')
            # A numerical replay PASS retains its historical meaning. The
            # stronger pre-hardware gate additionally needs explicit metric
            # initialization and actually accepted native visual constraints.
            # Old stereo binaries may not expose these observations yet; that
            # evidence gap is NOT_RUN, not a rewrite of their numerical result.
            native_path = run.get('native_observations')
            if not isinstance(native_path,str) or native_path not in artifacts:
                return dict(status='NOT_RUN', original_status=status, numerical_replay_status='PASS',
                    native_state_status='NOT_RUN', output=str(root),
                    reason='explicit native metric initialization and accepted visual corrections are unobserved')
            native = read_json(root/native_path)
            initialization = native.get('initialization',{})
            corrections = native.get('visual_corrections',{})
            require(native.get('status') != 'FAIL' and corrections.get('status') != 'FAIL',
                    'ACCEPTANCE_NATIVE_STATE_INVALID', 'native observation validation failed')
            initialized = (initialization.get('status') == 'PASS'
                and initialization.get('exact_metric_completion_observed') is True
                and bool(initialization.get('exact_metric_events'))
                and all(e.get('same_bundle_filter_commit_observed') is True
                        for e in initialization['exact_metric_events']))
            corrected = (corrections.get('status') == 'PASS'
                and type(corrections.get('count')) is int and corrections['count'] > 0
                and len(corrections.get('events',[])) == corrections['count'])
            if not initialized or not corrected:
                return dict(status='NOT_RUN', original_status=status, numerical_replay_status='PASS',
                    native_state_status='NOT_RUN', output=str(root),
                    reason='explicit metric initialization or accepted native visual corrections are unobserved',
                    native_initialization_observed=initialized, accepted_visual_corrections_observed=corrected)
        if role == 'fault_recovery':
            # Native trajectories before/after a disturbance do not establish
            # that a watchdog detected the interval with missing observations.
            # The existing output projection currently runs on Info callbacks;
            # it must not silently stand in for an independent timer trace.
            state_path = run.get('native_state_output')
            states = read_json(root/state_path) if isinstance(state_path,str) and state_path in artifacts else {}
            watchdog = states.get('watchdog',{})
            if (states.get('status') != 'PASS' or watchdog.get('evaluation_status') != 'PASS'
                    or watchdog.get('trigger') != 'INDEPENDENT_SCHEDULE'
                    or watchdog.get('timeline_complete') is not True or watchdog.get('truth_used') is not False):
                return dict(status='NOT_RUN', original_status=status, numerical_replay_status='PASS',
                    recovery_state_status='NOT_RUN', output=str(root),
                    reason='independent watchdog timeline and post-fault accepted visual recovery are unobserved')
            prepared = read_json(Path(config['input'])/'input.json')
            profile = prepared['derivation']['profile']
            origin = min(prepared.get('source_first_ns',prepared['first_ns']).values())
            begin,end = origin+profile['start_elapsed_ns'],origin+profile['end_elapsed_ns']
            outputs=states.get('outputs',[])
            require(all(type(row.get('usable')) is bool and (type(row.get('sensor_ns')) is int
                    or (row.get('sensor_ns') is None and row.get('trigger')=='INDEPENDENT_SCHEDULE'))
                    for row in outputs),
                    'ACCEPTANCE_RECOVERY_INVALID','timestamped explicit output validity required')
            # Observer timer ticks never acquire a fabricated sensor timestamp.
            sensor_outputs=[row for row in outputs if type(row.get('sensor_ns')) is int]
            before=[row for row in sensor_outputs if row['sensor_ns']<begin and row['usable']]
            after=[row for row in sensor_outputs if row['sensor_ns']>=end and row['usable']
                   and isinstance(row.get('last_accepted_visual_time'),dict)
                   and row['last_accepted_visual_time'].get('raw_value',-1)>=end]
            if not before or not after:
                return dict(status='NOT_RUN', original_status=status, numerical_replay_status='PASS',
                    recovery_state_status='NOT_RUN',output=str(root),
                    reason='usable state before fault and new accepted visual correction after fault are unobserved')
        return dict(status='PASS', original_status=status, output=str(root),
            run_sha256=file_hash(root/'run.json'), output_sha256=artifacts,
            runtime_sources_sha256=run['runtime_sources_sha256'], evidence_class='PC OFFLINE VERIFIED')
    except (ValueError,KeyError,OSError) as exc:
        return dict(status='FAIL', reason=str(exc), output=str(root), original_status=status)


def _execute_backend(backend, config):
    # All lifecycle/transport/scoring decisions stay in the existing entry.
    importlib.import_module(_MODULES[backend], __package__).run(config)


def _smoke_numerically_completed(member):
    """Collect repeats after complete numerical smoke despite an evidence gap.

    Missing new native health observations keep the stronger gate NOT_RUN, but
    do not erase a complete historical numerical replay or prevent measuring
    its repeatability. An actual execution, transport or scoring failure never
    qualifies for additional fixed baseline members.
    """
    return member['status']=='PASS' or (member.get('numerical_replay_status')=='PASS'
        and member.get('original_status') in ('PASS','EXECUTION_COMPLETED'))


def _cohort(gate, root, execute):
    report = dict(id=gate['id'], role=gate['role'], required=gate['required'],
                  evidence_class=gate['evidence_class'], members=[], rate=gate['rate'])
    try:
        locks = config_locks(gate['config_file'])
        _hashes(gate['inputs_sha256'], 'matrix input locks')
        require(all(gate['inputs_sha256'].get(p) == h for p,h in locks.items()),
                'ACCEPTANCE_LOCK_INVALID', 'matrix must freeze runtime config, source, installation and prepared inputs')
        template = read_json(gate['config_file'])
        require(template.get('backend') == gate['backend'] and template.get('rate') == gate['rate'],
                'ACCEPTANCE_COHORT_MIXED', 'backend/playback rate differs from matrix')
        if gate['role'] == 'high_motion':
            require(template.get('purpose') == 'UZH_FPV_PUBLIC_BASELINE'
                    and template.get('sequence') in ('indoor_forward_3','indoor_forward_6'),
                    'ACCEPTANCE_COHORT_MIXED','high-motion gate requires the declared original UZH sequences')
        if gate['role'] == 'fault_recovery':
            require(gate['backend']=='SchurVINS','ACCEPTANCE_COHORT_MIXED',
                    'native fault recovery currently requires the evidence-capable dedicated Schur runtime')
            prepared=read_json(Path(template['input'])/'input.json')
            require(prepared.get('derivation',{}).get('experiment_kind') == 'ARTIFICIAL_DISTURBANCE'
                    and prepared['derivation'].get('validation',{}).get('status') == 'PASS',
                    'ACCEPTANCE_COHORT_MIXED','recovery gate requires a valid declared disturbance')
        if gate['role'] in ('schur_mono','schur_stereo','openvins'):
            expected_backend = 'OpenVINS' if gate['role'] == 'openvins' else 'SchurVINS'
            require(gate['backend'] == expected_backend and (gate['role']=='openvins' or
                    template.get('mode') == gate['role'].removeprefix('schur_')),
                    'ACCEPTANCE_COHORT_MIXED', 'gate backend or camera mode differs')
        report['locked_config_sha256'] = file_hash(gate['config_file'])
        report['inputs_sha256'] = gate['inputs_sha256']
        planned=['smoke',*['repeat_'+str(i) for i in range(1,gate['repeat_count']+1)]]
        for name in planned:
            output = root/'runs'/gate['id']/name
            config = dict(copy.deepcopy(template), run_id=gate['id']+'_'+name, output=str(output))
            frozen = root/'configs'/gate['id']/(name+'.json')
            frozen.parent.mkdir(parents=True,exist_ok=True)
            write_json(frozen, config)
            if not execute:
                member = dict(status='NOT_RUN', reason='execution not requested', output=str(output))
            elif name != 'smoke' and not _smoke_numerically_completed(report['members'][0]):
                member = dict(status='NOT_RUN', reason='smoke did not pass; fixed repeat remains unexecuted', output=str(output))
            else:
                try:
                    _hashes(gate['inputs_sha256'], 'matrix input locks before member')
                    _execute_backend(gate['backend'], config)
                    _hashes(gate['inputs_sha256'], 'matrix input locks after member')
                    member = _audit_runtime(output, config, gate['backend'],role=gate['role'])
                except (ValueError,KeyError,OSError,RuntimeError) as exc:
                    member = dict(status='FAIL',reason=str(exc),output=str(output))
                try:
                    member['benchmark'] = summarize_run(output)
                except (ValueError,KeyError,OSError) as exc:
                    member['benchmark'] = dict(status='FAIL',reason=str(exc))
                    member.update(status='FAIL',reason='existing benchmark could not audit this member: '+str(exc))
            report['members'].append(dict(member, name=name, config=str(frozen), config_sha256=file_hash(frozen)))
        if execute and gate['repeat_count'] and _smoke_numerically_completed(report['members'][0]):
            repeats = report['members'][1:]
            summaries = {m['output']:m['benchmark'] for m in repeats}
            report['repeat_benchmark'] = summarize_group(gate['id'],list(summaries),summaries)
        statuses = [m['status'] for m in report['members']]
        report['status'] = 'PASS' if all(s == 'PASS' for s in statuses) else ('FAIL' if 'FAIL' in statuses else 'NOT_RUN')
    except FileNotFoundError as exc:
        report.update(status='BLOCKED', reason=str(exc))
    except (ValueError,KeyError,OSError) as exc:
        report.update(status='FAIL', reason=str(exc))
    planned = ['smoke',*['repeat_'+str(i) for i in range(1,gate['repeat_count']+1)]]
    for name in planned[len(report['members']):]:
        report['members'].append(dict(name=name,status='NOT_RUN',
            reason='cohort preflight or audit failed before this planned member',
            output=str(root/'runs'/gate['id']/name)))
    return report


def _required_coverage(matrix, results):
    """Audit the one matrix's declared scope without embedding dataset policy.

    Required entries refer to complete, hash-locked template configurations.
    The declaration therefore distinguishes rates, camera modes, sequences,
    source windows and individual disturbance inputs using their real configs.
    An omitted cohort remains NOT_RUN even when another cohort of its role
    passed. Which experiments are required belongs to the activity matrix.
    """
    declared=matrix.get('required_cohorts',[])
    coverage=[]
    for item in declared:
        entry=dict(item,status='NOT_RUN',matching_gate_ids=[])
        try:
            checked_file(item['config_file'],item['config_sha256'])
            for gate,result in zip(matrix['gates'],results):
                if (gate['kind']==('input_rejection' if item['protocol']=='INPUT_REJECTION' else 'cohort')
                        and gate['role']==item['role']
                        and Path(gate['config_file']).resolve()==Path(item['config_file']).resolve()
                        and result.get('locked_config_sha256')==item['config_sha256']
                        and gate['repeat_count']==(5 if item['protocol']=='SMOKE_PLUS_FIVE_REPEATS' else 0)):
                    entry['matching_gate_ids'].append(gate['id'])
                    if result['status']=='PASS':
                        entry['status']='PASS'
                    elif entry['status']!='PASS':
                        entry['status']=result['status']
            if not entry['matching_gate_ids']:
                entry['reason']='required locked cohort and protocol were not executed by this matrix'
        except FileNotFoundError as exc:
            entry.update(status='NOT_RUN',reason=str(exc))
        except (ValueError,OSError) as exc:
            entry.update(status='FAIL',reason=str(exc))
        coverage.append(entry)
    status=('PASS' if coverage and all(item['status']=='PASS' for item in coverage)
            else 'FAIL' if any(item['status']=='FAIL' for item in coverage) else 'NOT_RUN')
    return dict(id='required_cohort_coverage',role='lifecycle',required=True,status=status,
        evidence_class='PC OFFLINE VERIFIED',cohorts=coverage,
        reason=None if status=='PASS' else 'declared native experiment coverage is incomplete or absent')


def _input_rejection(gate,root,execute):
    """Exercise the existing runtime's refusal before any estimator can start."""
    report=dict(id=gate['id'],role=gate['role'],required=gate['required'],
        evidence_class='PC OFFLINE VERIFIED',status='NOT_RUN',execution_started=False,
        scope='invalid real-source perturbation preflight rejection only; estimator and recovery NOT_RUN')
    try:
        locks=config_locks(gate['config_file'])
        _hashes(gate['inputs_sha256'],'rejection matrix input locks')
        require(all(gate['inputs_sha256'].get(p)==h for p,h in locks.items()),
                'ACCEPTANCE_LOCK_INVALID','complete rejection source/config/installation locks')
        config=read_json(gate['config_file'])
        require(config.get('backend')=='SchurVINS' and config.get('rate')==gate['rate'],
                'ACCEPTANCE_COHORT_MIXED','invalid-input backend/rate differs')
        prepared=read_json(Path(config['input'])/'input.json')
        require(prepared.get('derivation',{}).get('experiment_kind')=='ARTIFICIAL_DISTURBANCE'
                and prepared['derivation'].get('validation',{}).get('status')=='FAIL',
                'ACCEPTANCE_REJECTION_INVALID','explicit invalid derived sensor recording required')
        report['locked_config_sha256']=file_hash(gate['config_file'])
        output=root/'runs'/gate['id']/'rejected'
        invocation=dict(config,run_id=gate['id']+'_rejected',output=str(output))
        frozen=root/'configs'/gate['id']/'rejection.json'
        frozen.parent.mkdir(parents=True,exist_ok=True)
        write_json(frozen,invocation)
        report.update(config=str(frozen),config_sha256=file_hash(frozen))
        if execute:
            module=importlib.import_module(_MODULES['SchurVINS'],__package__)
            module_path=Path(module.__file__).resolve() if hasattr(module,'__file__') else None
            sources={str(module_path):file_hash(module_path)} if module_path else {}
            try:
                module.preflight(invocation)
                report.update(status='FAIL',reason='invalid derived input was accepted by preflight')
            except ValueError as exc:
                report.update(status='PASS' if str(exc).startswith('FAULT_INPUT_REJECTED:') else 'FAIL',
                    reason=str(exc),expected_rejection_observed=str(exc).startswith('FAULT_INPUT_REJECTED:'))
            require(not output.exists(),'ACCEPTANCE_REJECTION_SIDE_EFFECT','rejected preflight created output')
            _hashes(gate['inputs_sha256'],'rejection inputs unchanged after refusal')
            if sources:
                _hashes(sources,'preflight code unchanged after refusal')
                report['runtime_sources_sha256']=sources
            report['output_absence_verified']=True
        else:
            report['reason']='execution not requested'
    except FileNotFoundError as exc:
        report.update(status='BLOCKED',reason=str(exc))
    except (ValueError,KeyError,OSError) as exc:
        report.update(status='FAIL',reason=str(exc))
    return report


def run(matrix, output, *, execute=False, selected_gates=None):
    """Prepare and optionally execute every gate, retaining independent failures."""
    validate_matrix(matrix)
    if selected_gates is not None:
        require(isinstance(selected_gates,(list,tuple)) and selected_gates
                and len(selected_gates)==len(set(selected_gates))
                and set(selected_gates)<={gate['id'] for gate in matrix['gates']},
                'ACCEPTANCE_SELECTION_INVALID','selection must name unique declared gate IDs')
    sources = {str(path):file_hash(path) for path in (Path(__file__),
        Path(__file__).with_name('common.py'),Path(__file__).resolve().parents[5]/'tools/schurvins/summarize.py')}
    root = Path(output).resolve()
    root.mkdir(parents=True,exist_ok=False)
    write_json(root/'matrix.json',matrix)
    results = []
    for gate in matrix['gates']:
        common = dict(id=gate['id'],role=gate['role'],required=gate['required'],evidence_class=gate['evidence_class'])
        if selected_gates is not None and gate['id'] not in selected_gates:
            result=dict(common,status='NOT_RUN',execution_started=False,
                reason='declared scope retained; gate not selected for this invocation')
            if gate['kind'] in ('cohort','input_rejection') and Path(gate['config_file']).is_file():
                result['locked_config_sha256']=file_hash(gate['config_file'])
        elif gate['kind'] == 'cohort':
            result = _cohort(gate,root,execute)
        elif gate['kind']=='input_rejection':
            result = _input_rejection(gate,root,execute)
        elif gate['kind'] == 'evidence':
            result = dict(common,**audit_receipt(gate['receipt'],gate['receipt_sha256'],
                                                gate['evidence_class'],currency=gate['currency']))
        else:
            result = dict(common,status=gate['status'],reason=gate['reason'],reproduce=gate['reproduce'])
        results.append(result)
        write_json(root/'result.json',dict(schema_version=1,status='PARTIAL',gates=results,
                                         execution_complete=False))
    present = {g['role'] for g in results if g['required']}
    for role in ROLES:
        if role not in present:
            results.append(dict(id='missing_'+role,role=role,required=True,status='NOT_RUN',
                evidence_class='HARDWARE BLOCKED / NOT_RUN' if role == 'arm64' else 'PC OFFLINE VERIFIED',
                reason='required software gate was not declared'))
    results.append(_required_coverage(matrix,results))
    try:
        _hashes(sources,'orchestrator code remained unchanged')
    except (ValueError,OSError) as exc:
        results.append(dict(id='orchestrator_code_changed',role='lifecycle',required=True,status='FAIL',
            evidence_class='PC OFFLINE VERIFIED',reason=str(exc)))
    complete = all(g['status'] == 'PASS' for g in results if g['required'])
    report = dict(schema_version=1,status='PASS' if complete else 'PARTIAL',execution_complete=True,gates=results,
        status_counts={s:sum(g['status'] == s for g in results) for s in STATUSES},
        software_prepared=complete,hardware_qualified=False,
        source_sha256=sources[str(Path(__file__))],orchestrator_sources_sha256=sources,
        matrix_sha256=file_hash(root/'matrix.json'),
        qualification='software gates only; no RK3576 live, flight or on-board timing acceptance')
    report['selected_gates']=list(selected_gates) if selected_gates is not None else None
    write_json(root/'result.json',report)
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--matrix')
    group.add_argument('--lock-config')
    parser.add_argument('--output')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--gate',action='append',help='Execute only this declared gate; all other scope remains NOT_RUN')
    args=parser.parse_args(argv)
    if args.lock_config:
        from .common import canonical
        print(canonical(config_locks(args.lock_config)))
        return 0
    if not args.output:
        parser.error('--matrix requires --output')
    report=run(read_json(args.matrix),args.output,execute=args.execute,selected_gates=args.gate)
    print('Pre-hardware: '+report['status']+'; '+str(report['status_counts']))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
