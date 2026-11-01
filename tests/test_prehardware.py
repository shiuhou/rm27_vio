"""Acceptance orchestration only; mocked runtime evidence is never real VIO."""
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from rm27.perception.vision.localization.experiments import prehardware as ph
from rm27.perception.vision.localization.experiments.common import file_hash,read_json,write_json


def cohort(tmp_path, *, backend='SchurVINS', mode='mono', rate=.5, gate_id='mono', role='schur_mono'):
    root=tmp_path/gate_id
    root.mkdir()
    source=root/'source.zip'
    source.write_bytes(b'TEST_FIXTURE_ONLY source; never sent to estimator')
    install=root/'install.json'
    write_json(install,dict(purpose='TEST_FIXTURE_ONLY'))
    prepared=root/'prepared'
    prepared.mkdir()
    artifact=prepared/'sensors.zip'
    artifact.write_bytes(b'TEST_FIXTURE_ONLY sensors')
    if backend == 'SchurVINS':
        write_json(prepared/'input.json',dict(artifacts_sha256={'sensors.zip':file_hash(artifact)}))
        config=dict(backend=backend,mode=mode,rate=rate,input=str(prepared),reference_archive=str(source),installation=str(install))
    else:
        write_json(prepared/'source_verification.json',dict(artifacts_sha256={'sensors.zip':file_hash(artifact)}))
        config=dict(backend=backend,mode=mode,rate=rate,bag=str(prepared),archive=str(source),installation=str(install))
    config.update(run_id='TEMPLATE_ID',output='TEMPLATE_OUTPUT')
    path=root/'config.json'
    write_json(path,config)
    gate=dict(id=gate_id,role=role,required=True,kind='cohort',backend=backend,rate=rate,repeat_count=5,
              evidence_class='PC OFFLINE VERIFIED',config_file=str(path),inputs_sha256=ph.config_locks(path))
    return gate,config


def runtime_output(config, *, passed=True, cleanup=True):
    root=Path(config['output'])
    root.mkdir(parents=True)
    score=dict(ate=dict(rmse_m=0.),rpe_adjacent=dict(translation_rmse_m=0.,rotation_rmse_deg=0.),
        rpe_1s=dict(translation_rmse_m=0.,rotation_rmse_deg=0.),alignment=dict(type='SE3',scale=1.),
        scale_diagnostic=dict(path_length_bias_percent=0.))
    write_json(root/'evaluation.json',dict(purpose='TEST_FIXTURE_ONLY',scored_epoch_count=1,
        epoch_count=1,epochs=[dict(epoch=0,status='SCORED',metrics=score)],gt_csv_sha256='f'*64))
    write_json(root/'trajectory.json',[dict(purpose='TEST_FIXTURE_ONLY',source_frame_id=0,
        state_ns=1,image_ns=1,position=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.],epoch=0)])
    runtime=Path(ph.__file__).with_name('schurvins_runtime.py' if config['backend']=='SchurVINS' else 'openvins_comparison.py')
    if config['backend']=='SchurVINS':
        write_json(root/'native-observations.json',dict(purpose='TEST_FIXTURE_ONLY',status='PASS',
            initialization=dict(status='PASS',exact_metric_completion_observed=True,
                exact_metric_events=[dict(same_bundle_filter_commit_observed=True)]),
            visual_corrections=dict(status='PASS',count=1,
                events=[dict(purpose='TEST_FIXTURE_ONLY',sensor_ns=1,constraint_count=1,native_epoch=0)])) )
    run=dict(status=('PASS' if config['backend']=='SchurVINS' else 'EXECUTION_COMPLETED') if passed else 'FAILED',
        config=config,execution_started=True,exit_code=0,container_cleanup=dict(absence_verified=cleanup),
        evaluation='evaluation.json',online_trajectory='trajectory.json',
        runtime_sources_sha256={str(runtime):file_hash(runtime)},
        execution=dict(failure=None,exits={p:0 for p in ('native','player','observer','master')}),
        output_sha256={p.name:file_hash(p) for p in root.iterdir()})
    if config['backend']=='SchurVINS':
        run['native_observations']='native-observations.json'
    write_json(root/'run.json',run)
    return root


def unavailable(role, *, status='NOT_RUN', required=True):
    return dict(id='unavailable_'+role,role=role,kind='unavailable',required=required,
        evidence_class='PC OFFLINE VERIFIED',status=status,reason='TEST_FIXTURE_ONLY declared external input unavailable',
        reproduce=['python3','-m','example_existing_entry','--config','/specific/path.json'])


def receipt(tmp_path, *, status='PASS', evidence_class='EMULATED / MOCK VERIFIED', name='evidence'):
    artifact=tmp_path/(name+'.txt')
    artifact.write_text('TEST_FIXTURE_ONLY completed command evidence')
    code=tmp_path/(name+'.py')
    code.write_text('TEST_FIXTURE_ONLY code identity')
    report=dict(schema_version=1,status=status,evidence_class=evidence_class,
        artifacts_sha256={str(artifact):file_hash(artifact)},code_sha256={str(code):file_hash(code)},
        execution=dict(command=['python3','-m','existing_test_entry'],completed=True,exit_code=0),
        cleanup=dict(absence_verified=True))
    if status != 'PASS':
        report['reason']='TEST_FIXTURE_ONLY explicit failed or unrun gate'
    path=tmp_path/(name+'.json')
    write_json(path,report)
    return path,report


def test_smoke_then_five_fixed_members_use_existing_entry_without_mutating_template(tmp_path,monkeypatch):
    gate,template=cohort(tmp_path)
    calls=[]
    def execute(backend,config):
        calls.append(copy.deepcopy(config))
        runtime_output(config)
    monkeypatch.setattr(ph,'_execute_backend',execute)
    report=ph.run(dict(schema_version=1,gates=[gate]),tmp_path/'out',execute=True)
    assert len(calls)==6
    assert [c['run_id'] for c in calls]==['mono_smoke']+['mono_repeat_'+str(i) for i in range(1,6)]
    assert all({k:v for k,v in c.items() if k not in ('run_id','output')} ==
               {k:v for k,v in template.items() if k not in ('run_id','output')} for c in calls)
    assert read_json(gate['config_file'])==template
    assert report['gates'][0]['status']=='PASS'
    assert report['status']=='PARTIAL' and report['software_prepared'] is False
    assert len([g for g in report['gates'] if g['id'].startswith('missing_')])==len(ph.ROLES)-1


def test_smoke_failure_keeps_five_not_run_members_and_continues_independent_gates(tmp_path,monkeypatch):
    gate,_=cohort(tmp_path)
    calls=[]
    def execute(backend,config):
        calls.append(config)
        runtime_output(config,passed=False)
    monkeypatch.setattr(ph,'_execute_backend',execute)
    report=ph.run(dict(schema_version=1,gates=[gate,unavailable('arm64',status='BLOCKED')]),tmp_path/'out',execute=True)
    assert len(calls)==1
    members=report['gates'][0]['members']
    assert [m['status'] for m in members]==['FAIL']+['NOT_RUN']*5
    assert report['gates'][1]['status']=='BLOCKED'


def test_repeat_failures_and_exceptions_are_retained_without_replacements(tmp_path,monkeypatch):
    gate,_=cohort(tmp_path)
    calls=[]
    def execute(backend,config):
        calls.append(config['run_id'])
        if config['run_id']=='mono_repeat_2':
            raise RuntimeError('TEST_FIXTURE_ONLY child failure')
        runtime_output(config,passed=config['run_id']!='mono_repeat_4')
    monkeypatch.setattr(ph,'_execute_backend',execute)
    report=ph.run(dict(schema_version=1,gates=[gate]),tmp_path/'out',execute=True)
    assert len(calls)==6 and len(set(calls))==6
    assert [m['status'] for m in report['gates'][0]['members']]==['PASS','PASS','FAIL','PASS','FAIL','PASS']
    assert report['gates'][0]['status']=='FAIL'


def test_complete_numerical_smoke_still_collects_five_repeats_while_native_state_gate_stays_open(tmp_path,monkeypatch):
    gate,_=cohort(tmp_path)
    calls=[]
    def execute(backend,config):
        calls.append(config)
        root=runtime_output(config)
        run=read_json(root/'run.json')
        run.pop('native_observations')
        write_json(root/'run.json',run)
    monkeypatch.setattr(ph,'_execute_backend',execute)
    report=ph.run(dict(schema_version=1,gates=[gate]),tmp_path/'out',execute=True)
    assert len(calls)==6
    assert all(m['status']=='NOT_RUN' and m['numerical_replay_status']=='PASS'
               for m in report['gates'][0]['members'])
    assert report['gates'][0]['status']=='NOT_RUN' and report['software_prepared'] is False


def test_nonexecuting_matrix_freezes_configs_and_refuses_overwrite(tmp_path,monkeypatch):
    gate,_=cohort(tmp_path)
    monkeypatch.setattr(ph,'_execute_backend',lambda *a:pytest.fail('must not execute'))
    output=tmp_path/'out'
    report=ph.run(dict(schema_version=1,gates=[gate]),output)
    assert all(m['status']=='NOT_RUN' for m in report['gates'][0]['members'])
    assert len(list((output/'configs/mono').glob('*.json')))==6
    assert not (output/'runs').exists()
    before=file_hash(output/'result.json')
    with pytest.raises(FileExistsError):
        ph.run(dict(schema_version=1,gates=[gate]),output)
    assert file_hash(output/'result.json')==before


@pytest.mark.parametrize('change',['rate','backend','mode','input_hash','missing_lock'])
def test_cohort_configuration_or_input_changes_never_execute(tmp_path,monkeypatch,change):
    gate,_=cohort(tmp_path)
    if change=='rate':
        gate['rate']=1.
    elif change=='backend':
        gate['backend']='OpenVINS'
    elif change=='mode':
        gate['role']='schur_stereo'
    elif change=='input_hash':
        first=next(iter(gate['inputs_sha256']))
        gate['inputs_sha256'][first]='0'*64
    else:
        gate['inputs_sha256'].pop(str(Path(gate['config_file']).parent/'prepared/input.json'))
    monkeypatch.setattr(ph,'_execute_backend',lambda *a:pytest.fail('invalid cohort must not execute'))
    report=ph.run(dict(schema_version=1,gates=[gate]),tmp_path/'out',execute=True)
    assert report['gates'][0]['status']=='FAIL'


@pytest.mark.parametrize('change',['cleanup','exit','native_exit','artifact','extra_artifact','source','unscored'])
def test_unsealed_changed_or_incomplete_runtime_evidence_cannot_pass(tmp_path,change):
    gate,config=cohort(tmp_path)
    config['output']=str(tmp_path/'run')
    root=runtime_output(config)
    report=read_json(root/'run.json')
    if change=='cleanup':
        report['container_cleanup']['absence_verified']=False
    elif change=='exit':
        report['exit_code']=1
    elif change=='native_exit':
        report['execution']['exits']['native']=1
    elif change=='artifact':
        (root/'trajectory.json').write_text('changed')
    elif change=='extra_artifact':
        (root/'unsealed.txt').write_text('changed')
    elif change=='source':
        key=next(iter(report['runtime_sources_sha256']))
        report['runtime_sources_sha256'][key]='0'*64
    else:
        write_json(root/'evaluation.json',dict(scored_epoch_count=0))
        report['output_sha256']['evaluation.json']=file_hash(root/'evaluation.json')
    write_json(root/'run.json',report)
    assert ph._audit_runtime(root,config,'SchurVINS')['status']=='FAIL'


@pytest.mark.parametrize('missing',['native_file','init','corrections','filter_commit'])
def test_numerical_schur_pass_does_not_close_missing_native_state_gate(tmp_path,missing):
    _,config=cohort(tmp_path)
    config['output']=str(tmp_path/'run')
    root=runtime_output(config)
    run=read_json(root/'run.json')
    native=read_json(root/'native-observations.json')
    if missing=='native_file':
        run.pop('native_observations')
    elif missing=='init':
        native['initialization']=dict(status='NOT_RUN')
    elif missing=='filter_commit':
        native['initialization']['exact_metric_events'][0]['same_bundle_filter_commit_observed']=False
    else:
        native['visual_corrections']=dict(status='NOT_RUN',count=None)
    write_json(root/'native-observations.json',native)
    run['output_sha256']['native-observations.json']=file_hash(root/'native-observations.json')
    write_json(root/'run.json',run)
    report=ph._audit_runtime(root,config,'SchurVINS')
    assert report['status']=='NOT_RUN'
    assert report['numerical_replay_status']=='PASS' and report['original_status']=='PASS'
    assert read_json(root/'run.json')['status']=='PASS'


def test_current_evidence_needs_actual_artifacts_code_exit_and_cleanup(tmp_path):
    path,report=receipt(tmp_path)
    assert ph.audit_receipt(path,file_hash(path),report['evidence_class'])['status']=='PASS'
    for field in ('execution','cleanup','code_sha256','artifacts_sha256'):
        changed=copy.deepcopy(report)
        del changed[field]
        write_json(path,changed)
        assert ph.audit_receipt(path,file_hash(path),report['evidence_class'])['status']=='FAIL'
    write_json(path,report)
    code=next(iter(report['code_sha256']))
    Path(code).write_text('CHANGED CODE')
    assert ph.audit_receipt(path,file_hash(path),report['evidence_class'])['status']=='FAIL'
    historical=ph.audit_receipt(path,file_hash(path),report['evidence_class'],currency='HISTORICAL')
    assert historical['status']=='NOT_RUN' and historical['historical_status']=='PASS'


@pytest.mark.parametrize('status',['FAIL','BLOCKED','NOT_RUN'])
def test_explicit_negative_receipts_are_preserved(tmp_path,status):
    path,report=receipt(tmp_path,status=status)
    assert ph.audit_receipt(path,file_hash(path),report['evidence_class'])['status']==status


def test_missing_or_wrongly_frozen_receipt_is_not_pass(tmp_path):
    path,report=receipt(tmp_path)
    assert ph.audit_receipt(path,'0'*64,report['evidence_class'])['status']=='FAIL'
    path.unlink()
    assert ph.audit_receipt(path,'0'*64,report['evidence_class'])['status']=='NOT_RUN'


def test_only_all_required_software_roles_close_overall_gate(tmp_path,monkeypatch):
    gates=[]
    for backend,mode,gate_id,role in [('SchurVINS','mono','mono','schur_mono'),
        ('SchurVINS','stereo','stereo','schur_stereo'),('OpenVINS','mono','ov','openvins')]:
        gate,_=cohort(tmp_path,backend=backend,mode=mode,gate_id=gate_id,role=role)
        gates.append(gate)
    for role in ph.ROLES:
        if role not in {g['role'] for g in gates}:
            if role in ('high_motion','fault_recovery'):
                gates.append(unavailable(role))
                continue
            path,report=receipt(tmp_path,name=role,evidence_class='ARM64 BUILD VERIFIED' if role=='arm64' else 'EMULATED / MOCK VERIFIED')
            gates.append(dict(id=role,role=role,required=True,kind='evidence',currency='CURRENT',
                receipt=str(path),receipt_sha256=file_hash(path),evidence_class=report['evidence_class']))
    monkeypatch.setattr(ph,'_execute_backend',lambda backend,config:runtime_output(config))
    report=ph.run(dict(schema_version=1,gates=gates),tmp_path/'complete',execute=True)
    assert report['status']=='PARTIAL' and report['software_prepared'] is False
    assert report['hardware_qualified'] is False
    assert 'REAL-TIME ON-BOARD VERIFIED' not in str(report)


@pytest.mark.parametrize('role',['high_motion','fault_recovery'])
def test_mock_receipts_cannot_substitute_for_native_motion_or_recovery_evidence(tmp_path,role):
    path,report=receipt(tmp_path,name=role)
    gate=dict(id=role,role=role,required=True,kind='evidence',currency='CURRENT',
        receipt=str(path),receipt_sha256=file_hash(path),evidence_class=report['evidence_class'])
    with pytest.raises(ValueError,match='fixed replay cohorts'):
        ph.validate_matrix(dict(schema_version=1,gates=[gate]))


def test_callback_only_state_projection_does_not_close_native_fault_recovery(tmp_path):
    _,config=cohort(tmp_path)
    config['output']=str(tmp_path/'run')
    root=runtime_output(config)
    report=ph._audit_runtime(root,config,'SchurVINS',role='fault_recovery')
    assert report['status']=='NOT_RUN'
    assert 'independent watchdog' in report['reason']


def required_cohort(gate):
    return dict(id='required_'+gate['id'],role=gate['role'],config_file=gate['config_file'],
        config_sha256=file_hash(gate['config_file']),protocol='SMOKE_PLUS_FIVE_REPEATS')


def test_explicit_required_cohort_scope_reports_omitted_rate_without_inventing_dataset_policy(tmp_path,monkeypatch):
    half,_=cohort(tmp_path,gate_id='half',rate=.5)
    real,_=cohort(tmp_path,gate_id='real',rate=1.)
    monkeypatch.setattr(ph,'_execute_backend',lambda backend,config:runtime_output(config))
    matrix=dict(schema_version=1,gates=[half],required_cohorts=[required_cohort(half),required_cohort(real)])
    report=ph.run(matrix,tmp_path/'out',execute=True)
    coverage=next(g for g in report['gates'] if g['id']=='required_cohort_coverage')
    assert [row['status'] for row in coverage['cohorts']]==['PASS','NOT_RUN']
    assert coverage['status']=='NOT_RUN' and report['software_prepared'] is False


def test_changed_required_config_is_failure_and_undeclared_scope_never_ready(tmp_path):
    gate,_=cohort(tmp_path)
    requirement=required_cohort(gate)
    requirement['config_sha256']='0'*64
    coverage=ph._required_coverage(dict(gates=[gate],required_cohorts=[requirement]),[])
    assert coverage['status']=='FAIL'
    assert ph._required_coverage(dict(gates=[]),[])['status']=='NOT_RUN'


def test_declared_fault_cohort_can_use_one_run_without_replacing_baseline_repeat_protocol(tmp_path,monkeypatch):
    gate,config=cohort(tmp_path,role='fault_recovery')
    prepared=Path(config['input'])/'input.json'
    data=read_json(prepared)
    data['derivation']=dict(experiment_kind='ARTIFICIAL_DISTURBANCE',validation=dict(status='PASS'))
    write_json(prepared,data)
    gate['inputs_sha256']=ph.config_locks(gate['config_file'])
    gate['repeat_count']=0
    calls=[]
    def execute(backend,configuration):
        calls.append(configuration)
        runtime_output(configuration)
    monkeypatch.setattr(ph,'_execute_backend',execute)
    report=ph.run(dict(schema_version=1,gates=[gate]),tmp_path/'out',execute=True)
    assert len(calls)==1 and len(report['gates'][0]['members'])==1
    assert report['gates'][0]['status']=='NOT_RUN'
    gate['role']='schur_mono'
    with pytest.raises(ValueError,match='five fixed repeats'):
        ph.validate_matrix(dict(schema_version=1,gates=[gate]))


@pytest.mark.parametrize('outcome',['expected','wrong_refusal','accepted','side_effect'])
def test_invalid_input_gate_only_calls_preflight_and_never_counts_as_native_recovery(tmp_path,monkeypatch,outcome):
    gate,config=cohort(tmp_path,role='fault_recovery')
    prepared=Path(config['input'])/'input.json'
    data=read_json(prepared)
    data['derivation']=dict(experiment_kind='ARTIFICIAL_DISTURBANCE',validation=dict(status='FAIL'))
    write_json(prepared,data)
    gate.update(kind='input_rejection',repeat_count=0,inputs_sha256=ph.config_locks(gate['config_file']))
    required=required_cohort(gate)
    required['protocol']='INPUT_REJECTION'
    calls=[]
    def preflight(configuration):
        calls.append(configuration)
        if outcome=='accepted':return
        if outcome=='side_effect':Path(configuration['output']).mkdir(parents=True)
        raise ValueError(('OTHER:' if outcome=='wrong_refusal' else 'FAULT_INPUT_REJECTED:')+' explicit refusal')
    monkeypatch.setattr(ph.importlib,'import_module',lambda *args:SimpleNamespace(preflight=preflight))
    monkeypatch.setattr(ph,'_execute_backend',lambda *args:pytest.fail('estimator must never start'))
    report=ph.run(dict(schema_version=1,gates=[gate],required_cohorts=[required]),tmp_path/'out',execute=True)
    assert len(calls)==1 and report['gates'][0]['execution_started'] is False
    assert report['gates'][0]['status']==('PASS' if outcome=='expected' else 'FAIL')
    coverage=next(g for g in report['gates'] if g['id']=='required_cohort_coverage')
    assert coverage['status']==('PASS' if outcome=='expected' else 'FAIL')
    assert report['software_prepared'] is False


def test_gate_selection_keeps_unexecuted_scope_open(tmp_path,monkeypatch):
    first,_=cohort(tmp_path,gate_id='first')
    second,_=cohort(tmp_path,gate_id='second',rate=1.)
    calls=[]
    def execute(backend,config):
        calls.append(config)
        runtime_output(config)
    monkeypatch.setattr(ph,'_execute_backend',execute)
    matrix=dict(schema_version=1,gates=[first,second],required_cohorts=[required_cohort(first),required_cohort(second)])
    report=ph.run(matrix,tmp_path/'out',execute=True,selected_gates=['first'])
    assert len(calls)==6 and report['selected_gates']==['first']
    assert [gate['status'] for gate in report['gates'][:2]]==['PASS','NOT_RUN']
    assert report['software_prepared'] is False
    with pytest.raises(ValueError,match='unique declared'):
        ph.run(matrix,tmp_path/'invalid',execute=True,selected_gates=['missing'])
    assert not (tmp_path/'invalid').exists()


@pytest.mark.parametrize('change',['repeats','unsafe_id','duplicate','onboard','generic_native_receipt'])
def test_matrix_rejects_relaxed_cohorts_and_unsupported_qualification(tmp_path,change):
    gate,_=cohort(tmp_path)
    matrix=dict(schema_version=1,gates=[gate])
    if change=='repeats':
        gate['repeat_count']=4
    elif change=='unsafe_id':
        gate['id']='../escape'
    elif change=='duplicate':
        matrix['gates'].append(copy.deepcopy(gate))
    elif change=='onboard':
        gate['evidence_class']='REAL-TIME ON-BOARD VERIFIED'
    else:
        gate['kind']='evidence'
    with pytest.raises(ValueError):
        ph.run(matrix,tmp_path/'invalid')
    assert not (tmp_path/'invalid').exists()


def test_hardware_pass_receipt_is_never_accepted(tmp_path):
    path,report=receipt(tmp_path,evidence_class='HARDWARE BLOCKED / NOT_RUN')
    assert ph.audit_receipt(path,file_hash(path),report['evidence_class'])['status']=='FAIL'
