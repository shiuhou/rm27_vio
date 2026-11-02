"""Synthetic boundary tests only; no official dataset or native execution claim."""
import copy
import math

import numpy as np
import pytest

from rm27.perception.vision.localization.experiments.schurvins import pose_observation
from rm27.perception.vision.localization.experiments.trajectory import pose_matrix
from tools.schurvins.evaluate import canonical_row, compare_rows, evaluate_rows


def projection(**overrides):
    args = dict(source_frame_id=0, image_ns=1403715278862142976,
                state_ns=1403715278862142976, observed_monotonic_ns=123456789,
                position=[1, 2, 3], quaternion_xyzw=[0, 0, math.sqrt(.5), math.sqrt(.5)],
                run_id='test-run', epoch=0, binary_sha256='a'*64, sequence='V1_01_easy',
                raw_reference={'file': 'raw/pose.jsonl', 'topic': '/svo/pose_imu', 'ordinal': 0},
                stage=2, quality=1)
    args.update(overrides)
    return pose_observation(**args)


def test_projection_preserves_numerics_and_unknown_operational_state():
    row = projection()
    assert row['source_timestamp']['raw_value'] == 1403715278862142976
    assert row['source_timestamp']['clock_domain'] != row['observation_timestamp']['clock_domain']
    assert row['localization_fields']['scale_state'] == 'METRIC'
    assert row['tracking_state'] == row['initialization_state'] == 'UNKNOWN'
    assert not row['validity'] and not row['contract_complete']
    assert row['processing_completed_timestamp'] is row['publish_timestamp'] is None
    assert row['reset_event'] is row['map_id'] is None
    assert row['raw_output']['stage'] == 2 and row['raw_output']['quality'] == 1
    assert 'covariance' not in row['localization_fields']
    matrix = pose_matrix(row)
    assert matrix[1, 0] == pytest.approx(1)
    assert matrix[0, 1] == pytest.approx(-1)
    assert canonical_row(row)['quaternion_xyzw'] == [0, 0, math.sqrt(.5), math.sqrt(.5)]


def test_epoch_evidence_and_processing_duration_are_not_promoted():
    row = projection(epoch=1, reset_evidence={'observer_stage_transition': [2, 0]},
                     processing_time_s=.002)
    assert row['localization_epoch'] == 1
    assert row['reset_event'] is None
    assert row['raw_output']['epoch_boundary_evidence']['observer_stage_transition'] == [2, 0]
    assert row['processing_time_s'] == .002
    assert row['processing_completed_timestamp'] is None


@pytest.mark.parametrize('changes', [dict(image_ns=1.5), dict(epoch=True),
    dict(state_ns=1403715278862142977),
    dict(quaternion_xyzw=[0, 0, 0, 2]), dict(position=[float('nan'), 0, 0]),
    dict(stage=True), dict(quality='good'), dict(processing_time_s=-1),
    dict(raw_reference={'topic': '/propagated'}), dict(binary_sha256='unknown')])
def test_projection_rejects_invalid_evidence(changes):
    with pytest.raises(ValueError):
        projection(**changes)


def test_camera_triggered_pose_does_not_invent_visual_update_evidence():
    row = projection()
    assert row['raw_output']['visual_update_completion'] is None
    assert row['raw_output']['inertial_only_state'] is None


def fixture():
    # 12 non-collinear samples at 10ms intervals with exact integer ns.
    ns = [1400000000000000000 + i*10000000 for i in range(12)]
    p = np.array([[i*.1, (i % 3)*.1, (i % 2)*.05] for i in range(12)])
    ref = dict(camera_ns=ns, ns=ns, positions=p,
               quaternions=np.array([[1., 0, 0, 0]]*12),
               normalization={'policy': 'SYNTHETIC TEST FIXTURE'},
               gt_csv_sha256='a'*64, archive_sha256='b'*64)
    rows = [dict(source_frame_id=i, image_ns=t, state_ns=t, epoch=0,
                 position=p[i].tolist(), quaternion_xyzw=[0, 0, 0, 1]) for i, t in enumerate(ns)]
    return ref, rows


def test_fixed_scale_and_epoch_scores_reuse_existing_evaluator():
    ref, rows = fixture()
    score = evaluate_rows(rows, ref)
    assert score['epochs'][0]['metrics']['ate']['rmse_m'] < 1e-10
    assert score['epochs'][0]['metrics']['alignment']['scale'] == 1
    scaled = copy.deepcopy(rows)
    for r in scaled:
        r['position'] = (np.array(r['position'])*2).tolist()
    result = evaluate_rows(scaled, ref)['epochs'][0]['metrics']
    assert result['ate']['rmse_m'] > .1
    assert result['scale_diagnostic']['path_length_bias_percent'] == pytest.approx(100)
    assert result['alignment']['scale'] == 1


def test_standalone_evaluator_records_math_source_and_refuses_overwrite(tmp_path):
    import json
    import zipfile
    from tools.schurvins import evaluate
    from tools.openvins.native_accuracy import digest
    ref, rows = fixture()
    archive = tmp_path/'synthetic-reference.zip'
    with zipfile.ZipFile(archive, 'w') as z:
        z.writestr('mav0/cam0/data.csv', ''.join(f'{ns},{ns}.png\n' for ns in ref['ns']))
        z.writestr('mav0/state_groundtruth_estimate0/data.csv', ''.join(
            f'{ns},{p[0]},{p[1]},{p[2]},1,0,0,0\n' for ns, p in zip(ref['ns'], ref['positions'])))
    poses = tmp_path/'synthetic-poses.json'
    poses.write_text(json.dumps(rows))
    output = tmp_path/'evaluation.json'
    args = ['--archive', str(archive), '--poses', str(poses), '--output', str(output)]
    result = evaluate.main(args)
    math_source = evaluate.Path(evaluate.__file__).resolve().parents[1]/'openvins/native_accuracy.py'
    assert result['inputs_sha256'][str(math_source)] == digest(math_source)
    assert result['epochs'][0]['metrics']['alignment']['scale'] == 1
    frozen = output.read_bytes()
    with pytest.raises(FileExistsError):
        evaluate.main(args)
    assert output.read_bytes() == frozen


def test_common_interval_matches_original_camera_ids_despite_text_rounding():
    ref, rows = fixture()
    rounded = copy.deepcopy(rows)
    for row in rounded:
        row['image_ns'] += 5000
    comparison = compare_rows(rows, rounded, ref)
    common = comparison['common_intervals'][0]
    assert common['first_image_ns'] == rows[0]['image_ns']
    assert common['last_image_ns'] == rows[-1]['image_ns']
    assert common['left']['raw_pose_count'] == common['right']['raw_pose_count'] == 12
    assert rounded[0]['image_ns'] == rows[0]['image_ns']+5000


def test_reset_epochs_keep_separate_scores_and_original_coverage():
    ref, rows = fixture()
    before = copy.deepcopy(rows)
    for r in rows[6:]:
        r['epoch'] = 1
        r['position'][0] += 100
    score = evaluate_rows(rows, ref)
    assert score['epoch_count'] == 2 and score['scored_epoch_count'] == 2
    assert all(e['metrics']['ate']['rmse_m'] < 1e-10 for e in score['epochs'])
    assert 'metrics' not in score
    assert not score['continuous_trajectory_qualified']
    assert score['output_camera_fraction'] == 1
    assert before[0] == rows[0]


def test_common_interval_does_not_hide_missing_start_tail_or_internal_frames():
    ref, rows = fixture()
    other = [r for i, r in enumerate(rows) if i in [2, 3, 5, 6, 7, 8]]
    comparison = compare_rows(rows, other, ref)
    right = comparison['right_full']
    assert right['missing_before_first'] == 2
    assert right['missing_after_last'] == 3
    assert right['missing_within_output_span'] == 1
    assert right['output_camera_fraction'] == .5
    assert comparison['left_full']['output_camera_fraction'] == 1
    assert comparison['common_intervals'][0]['first_image_ns'] == rows[2]['image_ns']
    assert comparison['common_intervals'][0]['last_image_ns'] == rows[8]['image_ns']


def test_unevaluable_epoch_is_retained():
    ref, rows = fixture()
    score = evaluate_rows(rows[:1], ref)
    assert score['epoch_count'] == 1 and score['scored_epoch_count'] == 0
    assert score['epochs'][0]['status'] == 'NOT_EVALUABLE'
    assert score['raw_pose_count'] == 1


@pytest.mark.parametrize('change', ['duplicate', 'nan', 'epoch_rewind', 'frame_id', 'state_rewind'])
def test_bad_rows_fail_without_silent_dropping(change):
    ref, rows = fixture()
    if change == 'duplicate':
        rows[1] = copy.deepcopy(rows[0])
    elif change == 'nan':
        rows[1]['position'][0] = float('nan')
    elif change == 'epoch_rewind':
        rows[0]['epoch'] = 1
    elif change == 'frame_id':
        rows[0]['source_frame_id'] = 2
    else:
        rows[1]['state_ns'] = rows[0]['state_ns']-1
    with pytest.raises(ValueError):
        evaluate_rows(rows, ref)


def test_same_run_raw_and_adapter_evaluations_are_exact():
    ref, rows = fixture()
    projected = [projection(source_frame_id=i, image_ns=r['image_ns'], state_ns=r['state_ns'],
                            position=r['position'], quaternion_xyzw=r['quaternion_xyzw'])
                 for i, r in enumerate(rows)]
    assert evaluate_rows(rows, ref) == evaluate_rows(projected, ref)
    before = copy.deepcopy(rows)
    evaluate_rows(rows, ref)
    assert rows == before


def test_evaluator_cli_reads_zip_and_refuses_overwrite(tmp_path):
    import json
    import zipfile
    from tools.schurvins.evaluate import main
    ref, rows = fixture()
    archive = tmp_path/'TEST_FIXTURE_ONLY.zip'
    with zipfile.ZipFile(archive, 'w') as z:
        z.writestr('mav0/cam0/data.csv', '#timestamp,filename\n'+''.join(
            f'{ns},{ns}.png\n' for ns in ref['ns']))
        z.writestr('mav0/state_groundtruth_estimate0/data.csv', '#time,p,q\n'+''.join(
            ','.join(map(str, [ns, *p, 1, 0, 0, 0]))+'\n'
            for ns, p in zip(ref['ns'], ref['positions'])))
    poses = tmp_path/'raw.jsonl'
    poses.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    output = tmp_path/'metrics.json'
    argv = ['--archive', str(archive), '--poses', str(poses), '--output', str(output)]
    result = main(argv)
    assert result['scored_epoch_count'] == 1
    assert result['epochs'][0]['metrics']['ate']['rmse_m'] < 1e-10
    before = output.read_bytes()
    with pytest.raises(FileExistsError):
        main(argv)
    assert output.read_bytes() == before


def test_resource_summary_keeps_native_rss_separate_from_container_memory():
    from tools.schurvins.summarize import native_resources, container_resources
    native = native_resources('''
User time (seconds): 12.5
System time (seconds): 2.5
Elapsed (wall clock) time (h:mm:ss or m:ss): 2:00.00
Maximum resident set size (kbytes): 2048
Exit status: 0
''')
    assert native['maximum_rss_kib'] == 2048
    assert native['average_cpu_cores'] == .125
    samples = [dict(observed_monotonic_ns=t, cgroup_cpu='usage_usec '+str(cpu),
                    cgroup_memory_bytes=str(memory)) for t,cpu,memory in [
                        (1000000000, 100000, 5000), (3000000000, 3100000, 9000)]]
    whole = container_resources(samples)
    assert whole['cpu_s_in_sampled_window'] == 3
    assert whole['average_cpu_cores'] == 1.5
    assert whole['sampled_peak_memory_current_bytes'] == 9000
    assert 'not RSS' in whole['memory_semantics']
    samples[1]['cgroup_cpu'] = 'usage_usec 1'
    with pytest.raises(ValueError, match='counter reset'):
        container_resources(samples)


def test_paused_info_timer_is_excluded_from_processing_distribution():
    from tools.schurvins.summarize import frontend_timing
    result = frontend_timing('SchurVINS', [
        dict(stage=1, processing_time_s=.1), dict(stage=2, processing_time_s=.002),
        dict(stage=2, processing_time_s=.004), dict(stage=0, processing_time_s=900)])
    assert result['native_tracking_stage_s']['p50'] == .003
    assert result['native_tracking_stage_s']['samples'] == 2
    assert result['stage_counts'] == {'0':1, '1':1, '2':2}
    assert result['end_to_end_latency'] == 'NOT_RUN'


def test_pinned_openvins_timing_keeps_seconds_and_checks_format():
    from tools.schurvins.summarize import frontend_timing
    header = '# timestamp (sec),tracking,propagation,msckf update,slam update,slam delayed,re-tri & marg,total\n'
    result = frontend_timing('OpenVINS', csv_text=header+'1,0,0,0,0,0,0,0.012\n')
    assert result['native_camera_update_s']['p95'] == .012
    with pytest.raises(ValueError, match='header'):
        frontend_timing('OpenVINS', csv_text='timestamp,milliseconds\n1,12\n')


def test_repeat_trajectory_comparison_removes_only_first_pose_rigid_gauge():
    from tools.schurvins.summarize import pairwise_trajectory
    _, rows = fixture()
    translated = copy.deepcopy(rows)
    for row in translated:
        row['position'][0] += 100
        row['state_ns'] += 25
    comparison = pairwise_trajectory(rows, translated)
    assert comparison['translation_difference_m']['maximum'] < 1e-12
    assert comparison['state_time_difference_ns']['minimum'] == -25
    assert comparison['first_sustained_divergence_camera_index'] is None
    for row in translated:
        row['position'] = (np.array(row['position'])*2).tolist()
    comparison = pairwise_trajectory(rows, translated)
    assert comparison['translation_difference_m']['maximum'] > .1
    assert comparison['first_sustained_divergence_camera_index'] == 1


def test_repeat_trajectory_retains_reconstructed_image_rounding():
    from tools.schurvins.summarize import pairwise_trajectory
    _, rows = fixture()
    rounded = copy.deepcopy(rows)
    for row in rounded:
        row['image_ns'] += 5000
    comparison = pairwise_trajectory(rows, rounded)
    assert comparison['common_camera_count'] == len(rows)
    assert comparison['reconstructed_image_time_difference_ns']['minimum'] == -5000
    assert comparison['translation_difference_m']['maximum'] < 1e-12
    rounded[0]['image_ns'] += 1_000_000
    with pytest.raises(ValueError, match='inconsistent'):
        pairwise_trajectory(rows, rounded)


def test_repeat_summary_keeps_failed_unscored_and_missing_planned_runs(tmp_path):
    from tools.schurvins.summarize import summarize_group
    paths = [str(tmp_path/str(i)) for i in range(5)]
    reports = {p:dict(directory=p, status='FAILED', run_id=str(i), backend='SchurVINS',
                     sequence='TEST_FIXTURE_ONLY', mode='mono', rate=.5)
               for i,p in enumerate(paths)}
    reports[paths[-1]]['status'] = 'NOT_RUN'
    for p, status in zip(paths[:2], ['PASS', 'NOT_RUN']):
        reports[p]['native_observations'] = dict(status=status,
            initialization=dict(exact_metric_completion_observed=True))
    for p,ate in zip(paths[:2], [.02,.04]):
        reports[p]['metrics'] = dict(ate_rmse_m=ate, rpe_1s_translation_rmse_m=None)
    report = summarize_group('TEST_FIXTURE_ONLY', paths, reports)
    assert report['planned_run_count'] == 5 and report['scored_run_count'] == 2
    assert report['successful_run_count'] == 0 and report['failed_run_count'] == 4
    assert not report['cohort_complete']
    assert report['pairs_exceeding_1cm'] == 1
    assert report['ate_rmse_m']['samples'] == 2
    assert report['native_metric_initialization_observed_run_count'] == 2
    assert report['native_metric_initialization_observed_fraction_of_planned'] == .4
    reports[paths[0]]['runtime_provenance'] = {'runtime_sha256':'a'*64}
    other = summarize_group('TEST_FIXTURE_ONLY', paths, reports)
    assert other['recorded_provenance_variant_count'] == 2


def test_repeat_summary_refuses_different_fixed_inputs(tmp_path):
    import json
    from tools.schurvins.summarize import summarize_group
    paths = [tmp_path/str(i) for i in range(2)]
    reports = {}
    for i,p in enumerate(paths):
        p.mkdir()
        (p/'run.json').write_text(json.dumps(dict(config=dict(backend='SchurVINS',
            mode='mono', rate=.5, input='different-input-'+str(i)))))
        reports[str(p.resolve())] = dict(directory=str(p.resolve()), backend='SchurVINS',
            mode='mono', rate=.5, status='FAILED', sequence='TEST_FIXTURE_ONLY')
    with pytest.raises(ValueError, match='fixed runtime configurations'):
        summarize_group('TEST_FIXTURE_ONLY', paths, reports)


def test_summary_cli_retains_failure_and_refuses_output_rewrite(tmp_path):
    import json
    from tools.schurvins.summarize import main
    run = tmp_path/'TEST_FIXTURE_ONLY'
    run.mkdir()
    (run/'run.json').write_text(json.dumps(dict(status='FAILED', execution_started=True,
        failure_code='TRACKING_LOST', config=dict(backend='SchurVINS', mode='mono', rate=.5))))
    output = tmp_path/'summary.json'
    argv = ['--run',str(run), '--output',str(output)]
    result = main(argv)
    assert result['runs'][0]['failure_code'] == 'TRACKING_LOST'
    assert result['runs'][0]['resources_native']['status'] == 'NOT_RUN'
    before = output.read_bytes()
    with pytest.raises(FileExistsError):
        main(argv)
    assert output.read_bytes() == before


def test_summary_reads_existing_openvins_record_envelope(tmp_path):
    import json
    from tools.schurvins.summarize import summarize_run
    (tmp_path/'online').mkdir()
    (tmp_path/'run.json').write_text(json.dumps(dict(status='FAILED',
        config=dict(backend='OpenVINS', run_id='TEST_FIXTURE_ONLY'),
        dataset=dict(verified_counts=dict(cam0=2912, imu0=29120)))))
    row = projection()
    (tmp_path/'online/trajectory.json').write_text(json.dumps(dict(records=[row])))
    report = summarize_run(tmp_path)
    assert report['trajectory_count'] == 1
    assert report['coverage']['camera_input_count'] == 2912
    assert report['initialization_proxy']['exact_initialization_completion_time'] is None


@pytest.mark.parametrize('status,interface', [('PARTIAL', 'PASS'), ('FAILED', 'NOT_RUN')])
def test_summary_preserves_unscored_recording_coverage_without_reproduction(tmp_path, status, interface):
    import json
    from tools.openvins.native_accuracy import digest
    from tools.schurvins.summarize import summarize_run
    (tmp_path/'online').mkdir()
    rows = [projection(source_frame_id=i, epoch=i//2,
                       image_ns=1403715278862142976+i*50000000,
                       state_ns=1403715278862142976+i*50000000)
            for i in (1, 2, 4)]
    (tmp_path/'online/trajectory.json').write_text(json.dumps(rows))
    (tmp_path/'evaluation.json').write_text(json.dumps(dict(status='NOT_RUN',
        reason='NO_REFERENCE_SUPPLIED', qualification='UNSCORED_SENSOR_RECORDING_OUTPUT',
        trajectory_count=len(rows))))
    artifacts = {str(p.relative_to(tmp_path)):digest(p)
                 for p in tmp_path.rglob('*') if p.is_file()}
    (tmp_path/'run.json').write_text(json.dumps(dict(status=status,
        failure_code='TRACKING_LOST' if status=='FAILED' else None,
        recording_interface_status=interface,
        config=dict(backend='SchurVINS', purpose='SEALED_SENSOR_RECORDING_REPLAY',
                    run_id='TEST_FIXTURE_ONLY', reference_archive=None),
        input=dict(dataset='SENSOR_RECORDING', counts=dict(cam0=6, imu0=60), truth_included=False),
        output_sha256=artifacts)))
    summary = summarize_run(tmp_path)
    assert summary['status'] == status
    assert summary['failure_code'] == ('TRACKING_LOST' if status=='FAILED' else None)
    assert summary['reference_scoring']['status'] == summary['accuracy_qualification'] == 'NOT_RUN'
    assert summary['reference_scoring']['reason'] == 'NO_REFERENCE_SUPPLIED'
    assert summary['estimator_reproduction'] == 'NOT_RUN_NO_REFERENCE'
    assert summary['recording_interface_status'] == interface
    assert 'metrics' not in summary and 'gt_csv_sha256' not in summary
    assert summary['trajectory_count'] == 3
    assert summary['coverage']['camera_input_count'] == 6
    assert summary['coverage']['output_camera_fraction'] == .5
    assert summary['coverage']['missing_before_first'] == summary['coverage']['missing_after_last'] == 1
    assert summary['coverage']['missing_within_output_span'] == 1
    assert summary['coverage']['epoch_count'] == 3
    assert summary['coverage']['scored_epoch_count'] == 0
    assert summary['artifact_integrity']['status'] == 'VERIFIED'


@pytest.mark.parametrize('transport', ['PASS', 'FAIL', None])
def test_summary_recording_without_pose_or_evaluation_preserves_transport_only(tmp_path, transport):
    import json
    from tools.schurvins.summarize import summarize_run
    result=dict(status='FAILED', execution_started=True, failure_code='NATIVE_OUTPUT_EMPTY',
        config=dict(backend='SchurVINS', purpose='SEALED_SENSOR_RECORDING_REPLAY',
                    run_id='TEST_FIXTURE_ONLY', reference_archive=None),
        input=dict(dataset='SENSOR_RECORDING', counts=dict(cam0=6, imu0=60), truth_included=False))
    if transport is not None:result['recording_transport_status']=transport
    (tmp_path/'run.json').write_text(json.dumps(result))
    report=summarize_run(tmp_path)
    assert report['status']=='FAILED' and report['failure_code']=='NATIVE_OUTPUT_EMPTY'
    assert report['recording_transport_status']==(transport or 'NOT_RUN')
    assert report['recording_interface_status']=='NOT_RUN'
    assert report['accuracy_qualification']=='NOT_RUN'
    assert report['reference_scoring']['status']=='NOT_RUN'
    assert report['reference_scoring']['reason']=='NO_REFERENCE_SUPPLIED'
    assert report['estimator_reproduction']=='NOT_RUN_NO_REFERENCE'
    assert report['physical_camera_imu_qualification'] is False
    assert not any(k in report for k in ('metrics','gt_csv_sha256','trajectory_count'))


@pytest.mark.parametrize('backend', ['SchurVINS', 'OpenVINS'])
def test_summary_fault_coverage_keeps_original_camera_denominator(tmp_path, backend):
    import json
    from tools.schurvins.summarize import summarize_run
    (tmp_path/'online').mkdir()
    (tmp_path/'run.json').write_text(json.dumps(dict(status='FAILED',
        config=dict(backend=backend, run_id='TEST_FIXTURE_ONLY'),
        dataset=dict(counts=dict(cam0=9, imu0=100),
                     source_counts=dict(cam0=10, imu0=100)))))
    # Frame 2 was dropped; remaining source IDs retain their original meaning.
    rows = [projection(source_frame_id=i, image_ns=1403715278862142976+i*50000000,
                       state_ns=1403715278862142976+i*50000000)
            for i in range(10) if i != 2]
    payload = dict(records=rows) if backend == 'OpenVINS' else rows
    (tmp_path/'online/trajectory.json').write_text(json.dumps(payload))
    report = summarize_run(tmp_path)
    assert report['sensor_counts']['cam0'] == 9
    assert report['source_counts']['cam0'] == 10
    assert report['coverage']['camera_input_count'] == 10
    assert report['coverage']['camera_delivered_input_count'] == 9
    assert report['coverage']['output_camera_fraction'] == .9
    assert report['coverage']['missing_after_last'] == 0
    assert report['coverage']['missing_within_output_span'] == 1


def test_summary_preserves_independent_openvins_stereo_configuration(tmp_path):
    import json
    from tools.schurvins.summarize import summarize_run
    (tmp_path/'run.json').write_text(json.dumps(dict(status='FAILED',
        config=dict(backend='OpenVINS', sequence='V1_03_difficult', mode='stereo', rate=1.0))))
    (tmp_path/'observations.json').write_text(json.dumps(dict(
        tracking_loss_observations=[], tracking_loss_detection='NOT_RUN')))
    report = summarize_run(tmp_path)
    assert (report['sequence'], report['mode'], report['rate']) == ('V1_03_difficult', 'stereo', 1.0)
    assert report['tracking_loss_observation_count'] is None
    assert report['tracking_loss_detection'] == 'NOT_RUN'


@pytest.mark.parametrize('missing_imu', [False, True])
def test_sealed_openvins_summary_keeps_native_initialization_visual_support_and_input_coverage(tmp_path, missing_imu):
    import json
    from test_native_events import BASE, CAMERAS, ov_native_event, ov_state_values, ov_support, ov_trace, ov_visual_rows
    from test_openvins_faults import native_imu_fixture
    from tools.openvins.native_accuracy import digest
    from tools.openvins.native_observations import analyze_openvins_observations
    from tools.schurvins.summarize import summarize_run
    raw = tmp_path/'raw'
    raw.mkdir()
    callbacks = native_imu_fixture(raw)
    events = callbacks[:-1] if missing_imu else callbacks
    events += ov_visual_rows()
    events += [ov_native_event('imu_state_propagation_completed', CAMERAS[3], values=ov_state_values()),
               ov_native_event('independent_observation_tick'),
               ov_native_event('landmark_visual_ekf_completed', CAMERAS[4], values=ov_state_values(),
                               support=ov_support((0, BASE)))]
    native = analyze_openvins_observations(ov_trace(raw, events), input_directory=raw)
    (tmp_path/'native-observations.json').write_text(json.dumps(native))
    frozen = {str(p.relative_to(tmp_path)):digest(p) for p in tmp_path.rglob('*') if p.is_file()}
    status = 'FAILED' if missing_imu else 'EXECUTION_COMPLETED'
    failure = 'OPENVINS_NATIVE_INPUT_INCOMPLETE' if missing_imu else None
    (tmp_path/'run.json').write_text(json.dumps(dict(status=status, failure_code=failure,
        config=dict(backend='OpenVINS', run_id='TEST_FIXTURE_ONLY', mode='mono', rate=.5),
        output_sha256=frozen)))
    summary = summarize_run(tmp_path)
    observed = summary['native_observations']
    assert summary['status'] == status and summary['failure_code'] == failure
    assert observed['status'] == 'PASS' and observed['observation_integrity'] == 'PASS'
    assert observed['initialization']['exact_metric_completion_observed']
    assert observed['initialization']['metric_commit_count'] == 1
    assert observed['initialization']['exact_metric_events'] == native['initialization_events']
    assert observed['initialization']['duration']['status'] == 'NOT_RUN'
    assert observed['visual_corrections']['count'] == 2
    assert observed['visual_corrections']['distinct_measurement_time_advance_count'] == 1
    assert observed['visual_corrections']['updates_without_visual_time_advance'] == 1
    assert observed['first_accepted_visual_sensor_ns'] == CAMERAS[1]
    assert observed['visual_corrections']['first_update_measurement_support'][0]['sensor_ns'] == BASE
    assert observed['visual_corrections']['last_visual_measurement_ns'] == CAMERAS[1]
    assert observed['native_imu_callback_coverage'] == native['native_imu_callback_coverage']
    assert observed['native_imu_callback_coverage']['status'] == ('FAIL' if missing_imu else 'PASS')
    assert observed['trace_completeness'] is True
    assert observed['independent_tick_count'] == 1
    assert observed['state_output']['status'] == observed['health_qualification'] == 'NOT_RUN'
    assert observed['automatic_reset'] == observed['raw_input_equality'] == 'NOT_RUN'
    assert observed['publication_latency']['status'] == 'NOT_RUN'
    assert not observed['truth_used']
    assert summary['operational_validity'].startswith('UNKNOWN/UNQUALIFIED')
    (tmp_path/'native-observations.json').write_text(json.dumps(dict(native, status='FAIL')))
    with pytest.raises(ValueError, match='artifact hash'):
        summarize_run(tmp_path)


def test_sealed_openvins_summary_retains_initialization_without_visual_acceptance(tmp_path):
    import json
    from test_native_events import ov_native_event, ov_state_values, ov_trace
    from tools.openvins.native_accuracy import digest
    from tools.openvins.native_observations import analyze_openvins_observations
    from tools.schurvins.summarize import summarize_run
    native = analyze_openvins_observations(ov_trace(tmp_path, [
        ov_native_event('metric_initialization_committed', values=ov_state_values())]))
    (tmp_path/'native-observations.json').write_text(json.dumps(native))
    frozen = {str(p.relative_to(tmp_path)):digest(p) for p in tmp_path.rglob('*') if p.is_file()}
    (tmp_path/'run.json').write_text(json.dumps(dict(status='FAILED', failure_code='NATIVE_OUTPUT_EMPTY',
        config=dict(backend='OpenVINS', mode='mono', rate=.5), output_sha256=frozen)))
    summary = summarize_run(tmp_path)
    observed = summary['native_observations']
    assert summary['status'] == 'FAILED' and observed['status'] == 'NOT_RUN'
    assert observed['initialization']['exact_metric_completion_observed']
    assert observed['visual_corrections']['count'] == 0
    assert observed['first_accepted_visual_sensor_ns'] is None
    assert observed['visual_corrections']['last_visual_measurement_ns'] is None
    assert observed['native_imu_callback_coverage']['status'] == 'NOT_RUN'
    assert observed['state_output']['status'] == 'NOT_RUN'


@pytest.mark.parametrize('active', [True, False])
def test_summary_does_not_parse_mutable_outputs_during_execution(tmp_path, active):
    import json
    from tools.schurvins.summarize import summarize_run
    (tmp_path/'run.json').write_text(json.dumps(dict(status='EXECUTING' if active else 'FAILED',
        artifact_finalization=None if active else 'NOT_FROZEN_CONTAINER_CLEANUP_UNVERIFIED',
        config=dict(backend='SchurVINS', sequence='V1_01_easy', mode='stereo', rate=.5))))
    (tmp_path/'resources.jsonl').write_text('{"partially_written":')
    result = summarize_run(tmp_path)
    assert result['status'] == ('EXECUTING' if active else 'FAILED')
    assert result['measurements'].startswith('NOT_RUN')
    assert 'metrics' not in result
    assert result['artifact_integrity']['status'] == 'NOT_CHECKED'


@pytest.mark.parametrize('change', ['modified', 'missing', 'undeclared'])
def test_summary_refuses_changed_finalized_artifacts(tmp_path, change):
    import json
    from tools.openvins.native_accuracy import digest
    from tools.schurvins.summarize import summarize_run
    observed = tmp_path/'observations.json'
    observed.write_text(json.dumps(dict(native_reset_count=None,
                                       tracking_loss_observations=[])))
    frozen = {'observations.json':digest(observed)}
    if change == 'modified':
        observed.write_text(json.dumps(dict(native_reset_count=999,
                                           tracking_loss_observations=[])))
    elif change == 'missing':
        observed.unlink()
    else:
        # Another declared output is intact, but a later-added observations
        # file must not be mistaken for an original native measurement.
        other = tmp_path/'native.log'
        other.write_text('TEST_FIXTURE_ONLY\n')
        frozen = {'native.log':digest(other)}
    (tmp_path/'run.json').write_text(json.dumps(dict(status='FAILED',
        config=dict(backend='SchurVINS'), output_sha256=frozen)))
    with pytest.raises(ValueError, match='artifact'):
        summarize_run(tmp_path)


def test_summary_verifies_declared_artifacts_and_marks_historical_unlocked(tmp_path):
    import json
    from tools.openvins.native_accuracy import digest
    from tools.schurvins.summarize import summarize_run
    observed = tmp_path/'observations.json'
    observed.write_text(json.dumps(dict(native_reset_count=None,
                                       tracking_loss_observations=[])))
    manifest = dict(status='FAILED', failure_code='NATIVE_BASELINE_DRIFT',
                    config=dict(backend='OpenVINS'))
    run = tmp_path/'run.json'
    run.write_text(json.dumps(manifest))
    unlocked = summarize_run(tmp_path)
    assert unlocked['artifact_integrity']['status'] == 'UNLOCKED'
    assert unlocked['failure_code'] == 'NATIVE_BASELINE_DRIFT'
    manifest['output_sha256'] = {'observations.json':digest(observed)}
    run.write_text(json.dumps(manifest))
    verified = summarize_run(tmp_path)
    assert verified['artifact_integrity']['status'] == 'VERIFIED'
    assert verified['artifact_integrity']['declared_artifact_count'] == 1
    assert verified['failure_code'] == 'NATIVE_BASELINE_DRIFT'


def native_projection_fixture(tmp_path, *, mode="mono"):
    """Production parser path fed explicit synthetic native events, not VIO."""
    import json
    from rm27.perception.vision.localization.experiments.native_events import analyze_native_events
    ns = [1403715278862142976+i*50_000_001 for i in range(4)]
    events = []
    def emit(kind, index, values):
        events.append(dict(schema_version=1, kind=kind, sensor_ns=ns[index], bundle_id=index,
                           monotonic_ns=len(events)+1, values=values))
    if mode == "stereo":
        emit('stereo_metric_geometry_ready', 0, [0, 1, .11, 80, 80, 1])
        emit('stereo_filter_state_commit', 0, [ns[0]/1e9, 1, 0, 1, 21, 21, 160, 80, 1,
            1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, -9.81007, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, .11, 1])
        emit('stereo_metric_initialized', 0, [0, 1, .11, 80, 80, 2, 2])
    else:
        emit('metric_filter_commit', 0, [ns[0]/1e9, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        emit('mono_metric_initialized', 0, [2.3, 1, 0, .01])
    emit('schur_visual_correction', 1, [42, 1, 2, 3, 1, 0, 0, 0, .1, .2, .3,
                                      .01, .02, .03, .001, .002, .003, 0, 0, -9.81, 1])
    emit('quality_decision', 1, [42, 0, 40, 40, 0, 0, 1, 2, 0, 0, 2, 1])
    path = tmp_path/'events.jsonl'
    path.write_text(''.join(json.dumps(row)+'\n' for row in events))
    report = analyze_native_events(path, ns)
    info = [dict(state_ns=t, observed_monotonic_ns=100+i*50_000_001,
                 stage=1 if i == 0 else 2, tracking_quality=2) for i, t in enumerate(ns)]
    return report, info


def native_projection(report, info, **changes):
    from rm27.perception.vision.localization.experiments.schurvins import project_native_corrections
    from rm27.perception.vision.localization.state_output import StateOutputPolicy
    args = dict(policy=StateOutputPolicy(75_000_000, 20_000_000, 75_000_000, 75_000_000, 50),
                run_id='projection-test', sequence='V1_01_easy', binary_sha256='b'*64)
    args.update(changes)
    return project_native_corrections(report, info, **args)


def test_native_candidate_projection_requires_full_provenance_and_keeps_unknown_native_times(tmp_path):
    import json
    report, info = native_projection_fixture(tmp_path)
    result = native_projection(report, info)
    assert result['status'] == 'PASS' and len(result['corrections']) == 1
    assert result['estimator_reproduction'] == result['flight_qualification'] == 'NOT_RUN'
    assert result['native_publication_timestamp'] is result['native_processing_completed_timestamp'] is None
    correction = result['corrections'][0]
    assert correction['estimate']['velocity'] == [.1, .2, .3]
    assert correction['estimate']['localization_epoch'] == 1
    assert correction['estimate']['publish_time']['semantic'] == 'OFFLINE_ADAPTER_EMISSION'
    assert correction['estimate']['publish_time']['clock_domain'] != result['outputs'][1]['observation_timestamp']['clock_domain']
    assert correction['estimate']['source_time']['raw_value'] == info[1]['state_ns']
    assert result['outputs'][0]['estimate'] is None
    assert result['outputs'][1]['usable'] and result['outputs'][2]['usable']
    assert result['outputs'][3]['reasons'] == ['VISUAL_CORRECTION_STALE', 'SENSOR_STREAM_STOPPED', 'VISUAL_STREAM_STOPPED']
    assert not result['outputs'][3]['estimate']['valid']
    assert result['outputs'][3]['estimate']['quality']['state_output']['state_source'] == 'VISUAL_CORRECTION'
    assert result['outputs'][3]['estimate']['quality']['state_output']['propagated_intervals'] == 0
    assert 'covariance' not in correction['estimate']
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('change', [dict(stage=3), dict(tracking_quality=0)])
def test_native_relocalization_or_bad_quality_never_becomes_valid_state(tmp_path, change):
    report, info = native_projection_fixture(tmp_path)
    info[1].update(change)
    if 'tracking_quality' in change:
        report['tracking_quality']['events'][0]['tracking_quality'] = change['tracking_quality']
    result = native_projection(report, info)
    assert result['status'] == 'NOT_RUN' and not result['corrections']
    assert result['rejected_events'][0]['reason'] == 'NATIVE_INFO_NOT_TRACKING_GOOD'
    assert result['rejected_events'][0]['category'] == 'HEALTH_TRANSITION'
    assert not result['evidence_errors']
    assert not any(row['usable'] for row in result['outputs'])


def test_native_initialization_cannot_be_reused_across_uninitialized_reset(tmp_path):
    report, info = native_projection_fixture(tmp_path)
    report['visual_corrections']['events'][0]['observation_segment'] += 1
    result = native_projection(report, info)
    assert result['status'] == 'FAIL' and not result['corrections']
    assert result['rejected_events'][0]['reason'] == 'SAME_SEGMENT_METRIC_INITIALIZATION_UNAVAILABLE'


def test_numeric_legacy_reports_and_missing_info_remain_unqualified(tmp_path):
    report, info = native_projection_fixture(tmp_path)
    assert native_projection({}, info)['status'] == 'NOT_RUN'
    assert native_projection(dict(report, status='FAIL'), info)['status'] == 'NOT_RUN'
    result = native_projection(report, info[:1])
    assert result['status'] == 'FAIL' and not result['corrections']
    assert result['rejected_events'][0]['reason'] == 'EXACT_INFO_ASSOCIATION_UNAVAILABLE'


def test_native_input_failure_invalidates_last_state_until_new_accepted_correction(tmp_path):
    report, info = native_projection_fixture(tmp_path)
    info[2]['stage'] = 3
    result = native_projection(report, info)
    assert result['outputs'][1]['usable']
    assert not result['outputs'][2]['usable'] and not result['outputs'][3]['usable']
    assert 'NATIVE_INFO_NOT_TRACKING_GOOD' in result['outputs'][3]['reasons']


def test_health_loss_is_distinct_from_evidence_corruption_and_never_masks_it(tmp_path):
    report, info = native_projection_fixture(tmp_path)
    info[1]['tracking_quality'] = 0
    report['tracking_quality']['events'][0]['tracking_quality'] = 0
    second = copy.deepcopy(report['visual_corrections']['events'][0])
    second.update(sensor_ns=info[2]['state_ns'], event_ordinal=second['event_ordinal']+1)
    report['visual_corrections']['events'].append(second)
    quality = copy.deepcopy(report['tracking_quality']['events'][0])
    quality.update(sensor_ns=info[2]['state_ns'], tracking_quality=2, bundle_id=second['bundle_id'])
    report['tracking_quality']['events'].append(quality)
    result = native_projection(report, info)
    assert result['status'] == 'PASS' and not result['evidence_errors']
    assert not result['outputs'][1]['usable'] and result['outputs'][2]['usable']
    assert result['health_transitions'][-1]['category'] == 'HEALTH_TRANSITION'
    report['visual_corrections']['events'][0]['constraint_count'] = 0
    corrupt = native_projection(report, info)
    assert corrupt['status'] == 'FAIL'
    assert corrupt['evidence_errors'][0]['reason'] == 'EXPLICIT_USED_VISUAL_CONSTRAINTS_UNAVAILABLE'
    report['visual_corrections']['events'][0]['constraint_count'] = 42
    report['visual_corrections']['events'][0]['observation_segment'] += 1
    corrupt = native_projection(report, info)
    assert corrupt['status'] == 'FAIL'
    assert corrupt['evidence_errors'][0]['reason'] == 'SAME_SEGMENT_METRIC_INITIALIZATION_UNAVAILABLE'


@pytest.mark.parametrize('change', ['absent', 'missing_association', 'stale_good_info'])
def test_native_state_requires_current_quality_evidence_and_rejects_stale_good(tmp_path, change):
    report, info = native_projection_fixture(tmp_path)
    if change == 'absent':
        del report['tracking_quality']
    elif change == 'missing_association':
        report['tracking_quality']['events'][0]['bundle_id'] += 1
    else:
        report['tracking_quality']['events'][0]['tracking_quality'] = 0
    result = native_projection(report, info)
    assert not result['corrections'] and not any(row['usable'] for row in result['outputs'])
    if change == 'absent':
        assert result['status'] == 'NOT_RUN'
        assert result['reason'] == 'CURRENT_NATIVE_QUALITY_DECISION_EVIDENCE_UNAVAILABLE'
    else:
        assert result['status'] == 'FAIL'
        assert result['evidence_errors'][0]['reason'] == 'EXACT_CURRENT_NATIVE_QUALITY_ASSOCIATION_UNAVAILABLE'


def test_native_projection_rejects_unobservable_or_unordered_evidence(tmp_path):
    report, info = native_projection_fixture(tmp_path)
    event = report['visual_corrections']['events'][0]
    event['constraint_count'] = 0
    result = native_projection(report, info)
    assert not result['corrections'] and result['status'] == 'FAIL'
    event['constraint_count'] = 42
    with pytest.raises(ValueError, match='no sorting'):
        native_projection(report, list(reversed(info)))
    with pytest.raises(ValueError, match='explicit experiment health'):
        native_projection(report, info, policy=None)


def test_native_projection_passes_actual_native_epoch_transition_through_tracker(tmp_path):
    import json
    from rm27.perception.vision.localization.experiments.native_events import analyze_native_events
    _, info = native_projection_fixture(tmp_path)
    path = tmp_path/'events.jsonl'
    events = [json.loads(line) for line in path.read_text().splitlines()]
    def append(kind, values):
        events.append(dict(schema_version=1, kind=kind, sensor_ns=info[2]['state_ns'], bundle_id=2,
                           monotonic_ns=len(events)+1, values=values))
    append('reset', [])
    append('metric_filter_commit', [info[2]['state_ns']/1e9, 1, 0, 0, 0, 0, 0, 0, 0, 0])
    append('mono_metric_initialized', [2.3, 1, 0, .01])
    second = copy.deepcopy(events[2])
    second.update(sensor_ns=info[3]['state_ns'], bundle_id=3, monotonic_ns=len(events)+1)
    second['values'][-1] = 2
    events.append(second)
    append('quality_decision', [42, 0, 40, 40, 0, 0, 1, 2, 0, 0, 2, 1])
    events[-1]['sensor_ns'] = info[3]['state_ns']
    events[-1]['bundle_id'] = 3
    path.write_text(''.join(json.dumps(row)+'\n' for row in events))
    report = analyze_native_events(path, [row['state_ns'] for row in info])
    assert report['status'] == 'PASS'
    result = native_projection(report, info)
    assert result['status'] == 'PASS'
    assert [row['estimate']['localization_epoch'] for row in result['corrections']] == [1, 2]
    assert result['outputs'][-1]['usable']
    assert result['outputs'][-1]['last_accepted_visual_time']['raw_value'] == info[3]['state_ns']
    report['lifecycle']['native_epoch_transitions'].clear()
    rejected = native_projection(report, info)
    assert rejected['status'] == 'FAIL'
    assert not rejected['outputs'][-1]['usable']
    assert 'NATIVE_RESET_EVIDENCE_REQUIRED' in rejected['rejected_events'][0]['reason']


def test_new_epoch_can_recover_after_first_native_correction_has_insufficient_quality(tmp_path):
    report, info = native_projection_fixture(tmp_path)
    first = report['visual_corrections']['events'][0]
    for i in (2, 3):
        correction = copy.deepcopy(first)
        correction.update(sensor_ns=info[i]['state_ns'], bundle_id=i, native_epoch=2,
                          observation_segment=1, event_ordinal=10+i)
        report['visual_corrections']['events'].append(correction)
        quality = copy.deepcopy(report['tracking_quality']['events'][0])
        quality.update(sensor_ns=info[i]['state_ns'], bundle_id=i, observation_segment=1,
                       tracking_quality=0 if i == 2 else 2)
        report['tracking_quality']['events'].append(quality)
    info[2]['tracking_quality'] = 0
    initialized = copy.deepcopy(report['initialization']['exact_metric_events'][0])
    initialized.update(native_epoch=2, observation_segment=1, event_ordinal=9, bundle_id=2)
    report['initialization']['exact_metric_events'].append(initialized)
    commit = copy.deepcopy(report['initialization']['metric_filter_commits'][0])
    commit.update(native_epoch=2, observation_segment=1, event_ordinal=8, bundle_id=2)
    report['initialization']['metric_filter_commits'].append(commit)
    report['lifecycle']['native_epoch_transitions'] = [dict(native_epoch=2, previous_epoch=1,
        observation_segment=1, event_ordinal=12, reset_event_missing=False)]
    result = native_projection(report, info)
    assert result['status'] == 'PASS' and not result['evidence_errors']
    assert not result['outputs'][2]['usable'] and result['outputs'][3]['usable']
    assert result['outputs'][3]['estimate']['localization_epoch'] == 2
    report['lifecycle']['native_epoch_transitions'][0]['reset_event_missing'] = True
    assert native_projection(report, info)['status'] == 'FAIL'


def test_sealed_summary_reuses_native_timing_and_never_invents_completion_or_recovery(tmp_path):
    import json
    from tools.schurvins.summarize import summarize_run
    from tools.openvins.native_accuracy import digest
    report, info = native_projection_fixture(tmp_path)
    states = native_projection(report, info)
    (tmp_path/'native-observations.json').write_text(json.dumps(report))
    (tmp_path/'native-state-output.json').write_text(json.dumps(states))
    (tmp_path/'raw').mkdir()
    for row in info:
        row['processing_time_s'] = .002
    (tmp_path/'raw/info.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in info))
    frozen = {str(p.relative_to(tmp_path)):digest(p) for p in tmp_path.rglob('*') if p.is_file()}
    (tmp_path/'run.json').write_text(json.dumps(dict(status='FAILED',
        config=dict(backend='SchurVINS', run_id='TEST_FIXTURE_ONLY'), output_sha256=frozen)))
    summary = summarize_run(tmp_path)
    native = summary['native_observations']
    assert summary['status'] == 'FAILED'
    assert native['initialization']['exact_metric_completion_observed']
    assert native['visual_corrections']['count'] == 1
    assert native['state_output']['usable_samples'] == 2
    assert native['publication_latency']['status'] == 'NOT_RUN'
    assert native['native_imu_callback_ledger']['status'] == 'NOT_RUN'
    assert native['native_imu_callback_ledger']['downstream_integration'] == 'UNOBSERVED'
    assert summary['fault_response']['status'] == 'NOT_RUN'
    assert summary['fault_response']['experiment_kind'] == 'ORIGINAL_DATA'
    frozen_report = (tmp_path/'native-observations.json')
    frozen_report.write_text(json.dumps(dict(report, status='FAIL')))
    with pytest.raises(ValueError, match='artifact hash'):
        summarize_run(tmp_path)


@pytest.mark.parametrize('missing_imu', [False, True])
@pytest.mark.parametrize('missing_camera_trace', [False, True])
def test_sealed_summary_keeps_imu_callback_integrity_separate_from_camera_and_health(
        tmp_path, missing_imu, missing_camera_trace):
    import json
    from test_native_events import imu_ledger_fixture, write_imu_ledger
    from rm27.perception.vision.localization.experiments.native_events import (
        analyze_native_events, analyze_native_imu_ledger)
    from tools.schurvins.summarize import summarize_run
    from tools.openvins.native_accuracy import digest
    path, observer, callbacks = imu_ledger_fixture(tmp_path)
    if missing_imu:
        callbacks.pop(-2)
        for key in ('attempted_callbacks', 'retained_records', 'written_records'):
            callbacks[-1][key] -= 1
    ledger = analyze_native_imu_ledger(write_imu_ledger(path, callbacks), observer)
    if missing_camera_trace:
        native = analyze_native_events(tmp_path/'absent-camera-events.jsonl', [observer[0]['state_ns']])
    else:
        native, _ = native_projection_fixture(tmp_path)
    native.setdefault('imu', {})['callback_ledger'] = ledger
    (tmp_path/'native-observations.json').write_text(json.dumps(native, allow_nan=False))
    frozen = {str(p.relative_to(tmp_path)):digest(p) for p in tmp_path.rglob('*') if p.is_file()}
    (tmp_path/'run.json').write_text(json.dumps(dict(status='FAILED',
        failure_code='TEST_UNQUALIFIED_ESTIMATOR', config=dict(backend='SchurVINS'), output_sha256=frozen)))
    summary = summarize_run(tmp_path)
    observed = summary['native_observations']
    assert summary['status'] == 'FAILED' and summary['failure_code'] == 'TEST_UNQUALIFIED_ESTIMATOR'
    assert observed['status'] == ('NOT_RUN' if missing_camera_trace else 'PASS')
    assert observed['native_imu_callback_ledger'] == ledger
    assert ledger['callback_coverage']['missing_count'] == int(missing_imu)
    assert ledger['callback_coverage']['status'] == ('FAIL' if missing_imu else 'PASS')
    assert ledger['buffer_insertion']['accepted_count'] == 4-int(missing_imu)
    assert ledger['downstream_integration'] == 'UNOBSERVED'
    assert ledger['health_qualification'] == ledger['reset_mapping'] == 'NOT_RUN'
    json.dumps(summary, allow_nan=False)


@pytest.mark.parametrize('backend', ['SchurVINS', 'OpenVINS'])
def test_unsealed_summary_cannot_promote_added_native_evidence(tmp_path, backend):
    import json
    from tools.schurvins.summarize import summarize_run
    report, _ = native_projection_fixture(tmp_path)
    (tmp_path/'native-observations.json').write_text(json.dumps(report))
    (tmp_path/'run.json').write_text(json.dumps(dict(status='FAILED', config=dict(backend=backend))))
    summary = summarize_run(tmp_path)
    assert summary['native_observations']['status'] == 'NOT_RUN'
    assert summary['fault_response']['status'] == 'NOT_RUN'


def fault_response_fixture():
    data = dict(source_first_ns=dict(cam0=1100, imu0=1000), first_ns=dict(cam0=1200, imu0=1200),
                derivation=dict(experiment_kind='ARTIFICIAL_DISTURBANCE',
                    profile=dict(kind='camera_gap', start_elapsed_ns=150, end_elapsed_ns=250, seed=37),
                    changes=[dict(source_event=2, action='DROP')]))
    info = [dict(state_ns=t, stage=3 if t == 1200 else 2, tracking_quality=2)
            for t in (1000, 1100, 1200, 1300, 1400, 1500)]
    events = [dict(sensor_ns=t, native_epoch=1, constraint_count=20,
                   state_source='EXPLICIT_NATIVE_ACCEPTED_VISUAL_CORRECTION') for t in (1100, 1300, 1400)]
    native = dict(status='PASS', visual_corrections=dict(events=events))
    return data, info, native


def test_fault_summary_preserves_source_origin_and_requires_actual_visual_recovery():
    from tools.schurvins.summarize import fault_response
    data, info, native = fault_response_fixture()
    summary = fault_response(data, info, native)
    assert summary['fault_window']['origin_sensor_ns'] == 1000
    assert summary['fault_window']['start_sensor_ns'] == 1150
    assert summary['fault_window']['end_sensor_ns'] == 1250
    assert summary['online_failure']['first_sensor_ns'] == 1200
    assert summary['fault_interval_association']['status'] == 'OBSERVED'
    assert summary['fault_interval_association']['evidence_kind'] == 'SAME_SOURCE_SENSOR_CLOCK'
    assert summary['recovery']['first_sensor_ns'] == 1300
    assert summary['recovery']['continuous_evidence_through_sensor_ns'] == 1400
    assert summary['recovery']['elapsed_from_fault_end_s'] == 50/1e9
    assert summary['offline_error_boundary']['status'] == 'NOT_RUN' and not summary['truth_drives_health']
    assert fault_response(data, info)['recovery']['status'] == 'NOT_RUN'
    native['visual_corrections']['events'][-1]['native_epoch'] = 2
    assert fault_response(data, info, native)['recovery']['status'] == 'NOT_RUN'


@pytest.mark.parametrize('loss_time', [1250, 16_000_001_250])
def test_post_fault_loss_and_recovery_remain_visible_without_fault_interval_association(loss_time):
    from tools.schurvins.summarize import fault_response
    data, _, _ = fault_response_fixture()
    times = (1000, 1100, 1200, loss_time, loss_time+100, loss_time+200)
    info = [dict(state_ns=t, stage=3 if t == loss_time else 2, tracking_quality=2) for t in times]
    native = dict(status='PASS', visual_corrections=dict(events=[dict(sensor_ns=t, native_epoch=1,
        constraint_count=20, state_source='EXPLICIT_NATIVE_ACCEPTED_VISUAL_CORRECTION')
        for t in times[-2:]]))
    result = fault_response(data, info, native)
    assert result['online_failure']['status'] == 'OBSERVED'
    assert result['online_failure']['first_sensor_ns'] == loss_time
    assert result['recovery']['status'] == 'OBSERVED'
    assert result['recovery']['first_sensor_ns'] == loss_time+100
    association = result['fault_interval_association']
    assert association['status'] == 'NOT_OBSERVED'
    assert association['source_sensor_ns'] == loss_time
    assert 'half-open' in association['reason']


def test_fault_summary_keeps_preexisting_invalidity_and_excludes_propagation_as_recovery():
    from tools.schurvins.summarize import fault_response
    data, info, native = fault_response_fixture()
    states = dict(outputs=[dict(sensor_ns=row['state_ns'], usable=row['state_ns'] != 1200,
                                reasons=['WATCHDOG_EXPIRED'] if row['state_ns'] == 1200 else []) for row in info])
    summary = fault_response(data, info, native, states)
    assert summary['online_failure']['evidence_kind'].startswith('OFFLINE_PROJECTED_NATIVE_HEALTH')
    assert summary['recovery']['status'] == 'OBSERVED'
    native['visual_corrections']['events'].clear()
    assert fault_response(data, info, native, states)['recovery']['status'] == 'NOT_RUN'
    states['outputs'][1]['usable'] = False
    before = fault_response(data, info, native, states)
    assert before['online_failure']['status'] == 'NOT_RUN'
    assert before['online_failure']['first_sensor_ns'] is None
    assert 'preexisting failure' in before['online_failure']['reason']


def test_fault_summary_rejects_implicit_sorting_and_unknown_epoch_origin():
    from tools.schurvins.summarize import fault_response
    data, info, native = fault_response_fixture()
    with pytest.raises(ValueError, match='no sorting'):
        fault_response(data, list(reversed(info)), native)
    data['derivation']['profile']['epoch'] = 3
    summary = fault_response(data, info, native)
    assert summary['status'] == 'NOT_RUN' and 'fault_window' not in summary


def watchdog_tick(received):
    return dict(schema_version=1, event='WATCHDOG_TICK', trigger='INDEPENDENT_SCHEDULE',
                clock_source='OBSERVER_CLOCK_MONOTONIC', observed_monotonic_ns=received,
                sensor_now=None)


def test_independent_watchdog_expires_without_refreshing_receipts_or_sensor_time(tmp_path):
    import json
    report, info = native_projection_fixture(tmp_path)
    ticks = [watchdog_tick(t) for t in (0, 80_000_000, 200_000_000)]
    result = native_projection(report, info, watchdog_events=ticks, watchdog_complete=True)
    watchdog = result['watchdog']
    assert watchdog['evaluation_status'] == 'PASS' and watchdog['timeline_complete']
    assert watchdog['tick_count'] == 3 and watchdog['invalid_tick_count'] == 0
    assert watchdog['maximum_tick_interval_ns'] == 120_000_000
    timer = [row for row in result['outputs'] if row['trigger'] == 'INDEPENDENT_SCHEDULE']
    assert timer[0]['estimate'] is None and timer[1]['usable']
    assert timer[2]['sensor_ns'] is None and not timer[2]['usable']
    assert timer[2]['latest_info_sensor_ns'] == info[-1]['state_ns']
    assert timer[2]['reasons'] == ['SENSOR_STREAM_STOPPED', 'VISUAL_STREAM_STOPPED']
    assert timer[2]['last_accepted_visual_time']['raw_value'] == info[1]['state_ns']
    assert timer[2]['estimate']['quality']['state_output']['receive_time']['raw_value'] == info[1]['observed_monotonic_ns']
    assert timer[2]['estimate']['publish_time']['semantic'] == 'OFFLINE_ADAPTER_EMISSION'
    assert timer[2]['estimate']['source_time']['raw_value'] == info[1]['state_ns']
    assert [row['observation_timestamp']['raw_value'] for row in result['outputs']] == sorted(
        row['observation_timestamp']['raw_value'] for row in result['outputs'])
    json.dumps(result, allow_nan=False)


def test_watchdog_completeness_requires_explicit_exit_and_interval_coverage(tmp_path):
    report, info = native_projection_fixture(tmp_path)
    ticks = [watchdog_tick(0), watchdog_tick(200_000_000)]
    default = native_projection(report, info, watchdog_events=ticks)['watchdog']
    assert default['evaluation_status'] == 'NOT_RUN' and not default['timeline_complete']
    cropped = native_projection(report, info, watchdog_events=ticks[1:], watchdog_complete=True)['watchdog']
    assert cropped['post_exit_completeness_verified'] and not cropped['spans_recorded_info']
    assert cropped['evaluation_status'] == 'NOT_RUN'
    absent = native_projection(report, info, watchdog_complete=True)['watchdog']
    assert not absent['timeline_complete'] and absent['tick_count'] == 0


@pytest.mark.parametrize('mutate', ['rewind', 'duplicate', 'clock', 'sensor_time', 'trigger'])
def test_watchdog_rejects_invalid_provenance_without_sorting(tmp_path, mutate):
    report, info = native_projection_fixture(tmp_path)
    ticks = [watchdog_tick(0), watchdog_tick(200_000_000)]
    if mutate == 'rewind':
        ticks.reverse()
    elif mutate == 'duplicate':
        ticks[1]['observed_monotonic_ns'] = 0
    elif mutate == 'clock':
        ticks[1]['clock_source'] = 'NATIVE_STEADY'
    elif mutate == 'sensor_time':
        ticks[1]['sensor_now'] = info[0]['state_ns']
    else:
        ticks[1]['trigger'] = 'INFO_CALLBACK'
    with pytest.raises(ValueError, match='WATCHDOG_INPUT_INVALID'):
        native_projection(report, info, watchdog_events=ticks, watchdog_complete=True)


def test_fault_summary_keeps_watchdog_clock_separate_and_checks_recovery_continuity():
    from tools.schurvins.summarize import fault_response
    data, info, native = fault_response_fixture()
    states = dict(watchdog=dict(evaluation_status='PASS', timeline_complete=True), outputs=[
        dict(sensor_ns=row['state_ns'], usable=row['state_ns'] != 1200, reasons=[],
             observation_timestamp=dict(raw_value=row['state_ns']+10_000, clock_domain='observer',
                 unit='ns', evidence_status='VERIFIED')) for row in info])
    states['outputs'].insert(4, dict(sensor_ns=None, latest_info_sensor_ns=1300,
        trigger='INDEPENDENT_SCHEDULE', usable=False, reasons=['VISUAL_STREAM_STOPPED'],
        observation_timestamp=dict(raw_value=11_350, clock_domain='observer', unit='ns', evidence_status='VERIFIED')))
    summary = fault_response(data, info, native, states)
    timer = summary['independent_watchdog_detection']
    assert timer['status'] == 'OBSERVED' and timer['last_sensor_reference_ns'] == 1300
    assert timer['first_invalid_observation_timestamp']['raw_value'] == 11_350
    assert timer['fault_interval_association'].startswith('NOT_RUN')
    assert summary['online_failure']['first_sensor_ns'] == 1200
    assert summary['recovery']['status'] == 'NOT_RUN'
    states['outputs'][4]['usable'] = True
    assert fault_response(data, info, native, states)['recovery']['status'] == 'OBSERVED'
    states['outputs'][4]['sensor_ns'] = 1350
    with pytest.raises(ValueError, match='cannot assert mapped sensor time'):
        fault_response(data, info, native, states)


def test_camera_silence_is_detected_by_bracketed_timer_without_inventing_sensor_time():
    from tools.schurvins.summarize import fault_response
    data, info, native = fault_response_fixture()
    # No Info arrives inside the interruption. Timer evidence is associated by
    # receipt order only, between the last healthy and first resumed callbacks.
    info = [row for row in info if row['state_ns'] != 1200]
    states = dict(outputs=[dict(sensor_ns=row['state_ns'], usable=True, reasons=[],
        observation_timestamp=dict(raw_value=row['state_ns']+10_000, clock_domain='observer',
            unit='ns', evidence_status='VERIFIED')) for row in info])
    states['outputs'].insert(0, dict(sensor_ns=None, trigger='INDEPENDENT_SCHEDULE', usable=False,
        reasons=['NATIVE_INITIALIZATION_UNKNOWN'], observation_timestamp=dict(raw_value=9_000,
            clock_domain='observer', unit='ns', evidence_status='VERIFIED')))
    states['outputs'].insert(3, dict(sensor_ns=None, trigger='INDEPENDENT_SCHEDULE', usable=False,
        reasons=['VISUAL_STREAM_STOPPED'], observation_timestamp=dict(raw_value=11_200,
            clock_domain='observer', unit='ns', evidence_status='VERIFIED')))
    result = fault_response(data, info, native, states)
    loss = result['online_failure']
    assert loss['status'] == 'OBSERVED' and loss['first_sensor_ns'] is None
    assert loss['elapsed_from_fault_s'] is None and loss['sensor_time_bracket_ns'] == [1100, 1300]
    assert loss['first_observation_timestamp']['raw_value'] == 11_200
    assert result['fault_interval_association']['status'] == 'OBSERVED'
    assert result['fault_interval_association']['evidence_kind'] == 'SAME_RECEIPT_CLOCK_SOURCE_BRACKET'
    assert result['fault_interval_association']['sensor_time_bracket_ns'] == [1100, 1300]
    assert result['recovery']['status'] == 'OBSERVED'
    assert result['recovery']['elapsed_from_detection_s'] is None
    states['outputs'][3]['observation_timestamp']['clock_domain'] = 'unmapped_clock'
    assert fault_response(data, info, native, states)['online_failure']['status'] == 'NOT_OBSERVED'


def test_timer_source_bracket_starting_at_fault_end_cannot_associate_detection():
    from tools.schurvins.summarize import fault_response
    data, info, native = fault_response_fixture()
    data['derivation']['profile'].update(start_elapsed_ns=50, end_elapsed_ns=100)
    info = [row for row in info if row['state_ns'] != 1200]
    states = dict(outputs=[dict(sensor_ns=row['state_ns'], usable=True, reasons=[],
        observation_timestamp=dict(raw_value=row['state_ns']+10_000, clock_domain='observer',
            unit='ns', evidence_status='VERIFIED')) for row in info])
    states['outputs'].insert(2, dict(sensor_ns=None, trigger='INDEPENDENT_SCHEDULE', usable=False,
        reasons=['VISUAL_STREAM_STOPPED'],
        observation_timestamp=dict(raw_value=11_200, clock_domain='observer', unit='ns', evidence_status='VERIFIED')))
    result = fault_response(data, info, native, states)
    assert result['independent_watchdog_detection']['status'] == 'OBSERVED'
    assert result['online_failure']['status'] == 'NOT_OBSERVED'
    assert result['fault_interval_association']['status'] == 'NOT_OBSERVED'


@pytest.mark.parametrize('clock', [None, '', 'UNKNOWN'])
def test_timer_detection_requires_explicit_receipt_clock_identity(clock):
    from tools.schurvins.summarize import fault_response
    data, info, native = fault_response_fixture()
    info = [row for row in info if row['state_ns'] != 1200]
    states = dict(outputs=[dict(sensor_ns=row['state_ns'], usable=True, reasons=[],
        observation_timestamp=dict(raw_value=row['state_ns']+10_000, clock_domain=clock,
            unit='ns', evidence_status='VERIFIED')) for row in info])
    states['outputs'].insert(2, dict(sensor_ns=None, trigger='INDEPENDENT_SCHEDULE', usable=False,
        reasons=['VISUAL_STREAM_STOPPED'],
        observation_timestamp=dict(raw_value=11_200, clock_domain=clock, unit='ns', evidence_status='VERIFIED')))
    result = fault_response(data, info, native, states)
    assert result['independent_watchdog_detection']['status'] == 'OBSERVED'
    assert result['online_failure']['status'] == 'NOT_OBSERVED'
    assert result['fault_interval_association']['status'] == 'NOT_OBSERVED'


@pytest.mark.parametrize('change', [dict(unit='ms'), dict(unit='UNKNOWN'),
    dict(evidence_status='UNKNOWN'), dict(raw_value=-1), dict(raw_value=11_200.0)])
def test_timer_fault_association_requires_qualified_receipt_units_and_values(change):
    from tools.schurvins.summarize import fault_response
    data, info, native = fault_response_fixture()
    info = [row for row in info if row['state_ns'] != 1200]
    stamp = dict(raw_value=11_200, clock_domain='observer', unit='ns', evidence_status='VERIFIED')
    stamp.update(change)
    states = dict(outputs=[dict(sensor_ns=row['state_ns'], usable=True, reasons=[],
        observation_timestamp=dict(raw_value=row['state_ns']+10_000, clock_domain='observer',
            unit='ns', evidence_status='VERIFIED')) for row in info])
    states['outputs'].insert(2, dict(sensor_ns=None, trigger='INDEPENDENT_SCHEDULE', usable=False,
        reasons=['VISUAL_STREAM_STOPPED'], observation_timestamp=stamp))
    result = fault_response(data, info, native, states)
    assert result['online_failure']['status'] == 'NOT_OBSERVED'
    assert result['fault_interval_association']['status'] == 'NOT_OBSERVED'


def test_equal_receipts_cannot_form_an_inverted_source_bracket():
    from tools.schurvins.summarize import fault_response
    data, _, native = fault_response_fixture()
    info = [dict(state_ns=t, stage=2, tracking_quality=2) for t in (1000, 1100, 1175, 1225, 1300, 1400)]
    states = dict(outputs=[dict(sensor_ns=row['state_ns'], usable=True, reasons=[],
        observation_timestamp=dict(raw_value=11_200 if row['state_ns'] in (1175, 1225) else row['state_ns']+10_000,
            clock_domain='observer', unit='ns', evidence_status='VERIFIED')) for row in info])
    states['outputs'].insert(3, dict(sensor_ns=None, trigger='INDEPENDENT_SCHEDULE', usable=False,
        reasons=['VISUAL_STREAM_STOPPED'], observation_timestamp=dict(raw_value=11_200,
            clock_domain='observer', unit='ns', evidence_status='VERIFIED')))
    result = fault_response(data, info, native, states)
    assert result['online_failure']['status'] == 'NOT_OBSERVED'
    assert result['fault_interval_association']['status'] == 'NOT_OBSERVED'


@pytest.mark.parametrize('target', ['timer', 'left_endpoint', 'right_endpoint'])
@pytest.mark.parametrize('change', [dict(clock_domain='UNKNOWN'), dict(unit='ms'),
    dict(evidence_status='UNKNOWN')])
def test_recovery_cannot_skip_an_unqualified_timer_when_checking_continuity(change, target):
    from tools.schurvins.summarize import fault_response
    data, info, native = fault_response_fixture()
    states = dict(outputs=[dict(sensor_ns=row['state_ns'], usable=row['state_ns'] != 1200, reasons=[],
        observation_timestamp=dict(raw_value=row['state_ns']+10_000, clock_domain='observer',
            unit='ns', evidence_status='VERIFIED')) for row in info])
    stamp = dict(raw_value=11_350, clock_domain='observer', unit='ns', evidence_status='VERIFIED')
    if target == 'timer':
        stamp.update(change)
    else:
        states['outputs'][3 if target == 'left_endpoint' else 4]['observation_timestamp'].update(change)
    states['outputs'].insert(4, dict(sensor_ns=None, trigger='INDEPENDENT_SCHEDULE', usable=target != 'timer',
        reasons=['VISUAL_STREAM_STOPPED'], observation_timestamp=stamp))
    result = fault_response(data, info, native, states)
    assert result['fault_interval_association']['status'] == 'OBSERVED'
    assert result['recovery']['status'] == 'NOT_RUN'


def test_stereo_native_projection_preserves_baseline_scale_and_upstream_prior_provenance(tmp_path):
    report, info = native_projection_fixture(tmp_path, mode="stereo")
    result = native_projection(report, info)
    assert result["status"] == "PASS" and len(result["corrections"]) == 1
    correction = result["corrections"][0]
    assert correction["scale_source"] == "PINNED_CALIBRATED_STEREO_BASELINE"
    assert correction["initialization_source"].startswith("stereo_metric_geometry_ready+stereo_filter_state_commit+")
    assert "mono_metric_initialized" not in correction["initialization_source"]
    provenance = correction["estimate"]["quality"]["native_correction"]
    assert provenance["initialization_method"] == "CALIBRATED_STEREO_BASELINE_WITH_UPSTREAM_IMU_PRIORS"
    assert "ZERO_INITIAL_VELOCITY" in provenance["upstream_initialization_assumptions"]
    assert provenance["covariance_status"] == "UNKNOWN_NOT_EXPORTED"
    assert result["outputs"][0]["estimate"] is None  # First clone is not an accepted visual correction.
    assert result["outputs"][1]["usable"]
    assert result["estimator_reproduction"] == result["flight_qualification"] == "NOT_RUN"


@pytest.mark.parametrize("change,reason", [
    ({"initialization_method": "UNVERIFIED_OTHER_INITIALIZATION"}, "NATIVE_INITIALIZATION_METHOD_UNSUPPORTED"),
    ({"native_epoch": 99}, "NATIVE_INITIALIZATION_EPOCH_MISMATCH"),
])
def test_stereo_projection_rejects_unknown_method_or_cross_epoch_bootstrap(tmp_path, change, reason):
    report, info = native_projection_fixture(tmp_path, mode="stereo")
    report["initialization"]["exact_metric_events"][0].update(change)
    result = native_projection(report, info)
    assert not result["corrections"]
    assert result["rejected_events"][0]["reason"] == reason
    assert not any(row["usable"] for row in result["outputs"])


def native_imu_projection_fixture(tmp_path):
    report, info = native_projection_fixture(tmp_path)
    origin = info[1]['state_ns']-50_000_000
    # The correction arrives after later IMU samples. Its buffered snapshot must
    # replay those samples rather than rewind output to the old Info timestamp.
    info[1]['observed_monotonic_ns'] += 15_000_000
    raw, source = [], []
    for index in range(41):
        ns = origin+index*5_000_000
        row = dict(state_ns=ns, gyro=[.001, .002, .003], accel=[.01,.02,9.84])
        source.append(dict(row, stream='imu0'))
        raw.append(dict(copy.deepcopy(row), frame_id='imu0', native_sequence=index+77,
                        observed_monotonic_ns=100+index*5_000_000))
    contract = dict(status='PASS', evidence_status='VERIFIED', measurement_kind='RAW_SPECIFIC_FORCE',
        gyro_unit='rad/s', accel_unit='m/s^2', native_imu_frame='imu', timestamp_offset_ns=0,
        raw_ros_frame_id='imu0', source_imu_id='EuRoC:sourcehash:imu0',
        calibration_source='verified source/native frame binding', source_rows=source,
        evidence_sha256={'/source/manifest.json':'a'*64})
    return report, info, raw, contract


def test_native_raw_imu_drives_same_tracker_and_delayed_correction_replays_snapshot(tmp_path):
    import json
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    frozen = copy.deepcopy((report, info, raw, contract))
    path = tmp_path/'propagation.jsonl'
    result = native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                               propagation_output_path=path)
    assert result['status'] == result['propagation']['status'] == 'PASS'
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert rows and all(row['estimate']['schema_version'] == 2 for row in rows)
    assert any(row['estimate']['quality']['state_output']['state_source'] == 'IMU_PROPAGATION' for row in rows)
    assert result['outputs'][1]['estimate']['source_time']['raw_value'] > info[1]['state_ns']
    first = rows[0]
    dt = (first['estimate']['source_time']['raw_value']-info[1]['state_ns'])/1e9
    assert first['estimate']['translation'] == pytest.approx([1+.1*dt, 2+.2*dt, 3+.3*dt])
    assert all(row['estimate']['quality']['state_output']['last_accepted_visual_time']['raw_value'] == info[1]['state_ns'] for row in rows)
    assert not rows[-1]['usable'] and 'VISUAL_CORRECTION_STALE' in rows[-1]['reasons']
    assert all(row['raw_sensor_ns'] == contract['source_rows'][row['source_sequence']]['state_ns'] for row in rows)
    assert max(len(line.encode())+1 for line in path.read_text().splitlines()) <= 4096
    assert result['propagation']['bytes_written'] == path.stat().st_size
    assert result['propagation']['output_complete'] is True
    assert result['propagation']['output_sensor_frequency_hz'] == pytest.approx(200)
    assert (report, info, raw, contract) == frozen
    assert len(result['outputs']) == len(info)  # high-frequency rows stay separate


@pytest.mark.parametrize('imu_delivery_delay_ns', [40_000_000, 80_000_000])
def test_native_correction_overtaking_ordered_imu_preserves_raw_times_and_snapshot(tmp_path, imu_delivery_delay_ns):
    import json
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    for row in raw:
        row['observed_monotonic_ns'] += imu_delivery_delay_ns
    frozen = copy.deepcopy((report, info, raw, contract))
    path = tmp_path/'pending-imu.jsonl'
    result = native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                               propagation_output_path=path)
    assert result['status'] == result['propagation']['status'] == 'PASS'
    assert result['evidence_errors'] == []
    assert result['propagation']['rejected_sample_count'] == 0
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    before_snapshot = [row for row in rows if row['raw_sensor_ns'] < info[1]['state_ns']]
    assert len(before_snapshot) > 1
    assert all(row['usable'] and row['estimate']['source_time']['raw_value'] == info[1]['state_ns']
               and row['estimate']['translation'] == [1, 2, 3] for row in before_snapshot)
    assert all(row['raw_sensor_ns'] == contract['source_rows'][row['source_sequence']]['state_ns']
               for row in rows)
    assert all(row['estimate']['quality']['state_output']['last_accepted_visual_time']['raw_value']
               == info[1]['state_ns'] for row in rows)
    assert (report, info, raw, contract) == frozen


def test_native_raw_imu_gap_keeps_original_source_ids_and_invalidates_propagation(tmp_path):
    import json
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    del raw[18:20]
    result = native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                               propagation_output_path=tmp_path/'gap.jsonl')
    assert result['propagation']['status'] == 'FAIL'
    assert result['propagation']['rejected_sample_count'] >= 1
    rows = [json.loads(line) for line in (tmp_path/'gap.jsonl').read_text().splitlines()]
    assert any(row['source_sequence'] == 20 for row in rows)
    assert all(not row['usable'] for row in rows if row['source_sequence'] >= 20)
    assert any('IMU sequence or sampling gap' in event['reason'] for event in result['rejected_events'])


@pytest.mark.parametrize('change', ['time', 'payload', 'frame', 'observer_order'])
def test_native_raw_imu_requires_exact_source_payload_frame_and_clock_order(tmp_path, change):
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    if change == 'time':raw[0]['state_ns'] += 1
    elif change == 'payload':raw[0]['gyro'][0] = 9.
    elif change == 'frame':raw[0]['frame_id'] = 'imu'
    else:raw[2]['observed_monotonic_ns'] = 0
    with pytest.raises(ValueError, match='PROPAGATION_INPUT_INVALID'):
        native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                          propagation_output_path=tmp_path/'refused.jsonl')
    assert not (tmp_path/'refused.jsonl').exists()


def test_native_raw_imu_missing_binding_and_uzh_timing_leave_legacy_projection(tmp_path):
    report, info, raw, _ = native_imu_projection_fixture(tmp_path)
    legacy = native_projection(report, info)
    for contract in [None, dict(status='NOT_RUN',reason='UZH_TIMING_APPLICATION_UNQUALIFIED')]:
        result = native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                                   propagation_output_path=tmp_path/'not-run.jsonl')
        assert result['propagation']['status'] == 'NOT_RUN'
        assert len(result['outputs']) == len(legacy['outputs'])
        assert not (tmp_path/'not-run.jsonl').exists()


def test_native_raw_imu_quality_loss_is_not_healed_by_more_propagation(tmp_path):
    import json
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    info[2]['stage'] = 3
    result = native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                               propagation_output_path=tmp_path/'lost.jsonl')
    rows = [json.loads(line) for line in (tmp_path/'lost.jsonl').read_text().splitlines()]
    later = [row for row in rows if row['raw_sensor_ns'] > info[2]['state_ns']]
    assert later and all(not row['usable'] for row in later)
    assert any('NATIVE_INFO_NOT_TRACKING_GOOD' in row['reasons'] for row in later)
    meta = result['propagation']
    callback_span = rows[-1]['raw_sensor_ns']-rows[0]['raw_sensor_ns']
    assert meta['output_sensor_frequency_hz'] == pytest.approx((len(rows)-1)*1e9/callback_span)
    assert meta['first_output_sensor_ns'] == rows[0]['raw_sensor_ns']
    assert meta['last_output_sensor_ns'] == rows[-1]['raw_sensor_ns']
    assert meta['output_sensor_frequency_basis'] == 'EMITTED_ROWS_OVER_ORIGINAL_IMU_SOURCE_TIME'
    state_times = {row['estimate']['source_time']['raw_value'] for row in rows}
    assert len(state_times) < len(rows)  # Invalid frozen snapshots are still emitted.
    epoch = meta['state_timestamp_epochs'][0]
    assert epoch['distinct_state_timestamp_count'] == len(state_times)
    assert epoch['state_timestamp_frequency_hz'] == pytest.approx(
        (len(state_times)-1)*1e9/(max(state_times)-min(state_times)))
    assert 'not visual' in meta['state_timestamp_frequency_semantic']


@pytest.mark.parametrize('reset_received_ns', [160_000_100, 210_000_100])
def test_native_raw_imu_reset_clears_cross_epoch_buffer_and_latest_sensor(tmp_path, reset_received_ns):
    import json
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    first = report['visual_corrections']['events'][0]
    second = copy.deepcopy(first)
    second.update(sensor_ns=info[3]['state_ns'], native_epoch=2, observation_segment=1, event_ordinal=10,
                  position=[10, 20, 30])
    report['visual_corrections']['events'].append(second)
    quality = copy.deepcopy(report['tracking_quality']['events'][0])
    quality.update(sensor_ns=second['sensor_ns'], observation_segment=1, bundle_id=second['bundle_id'])
    report['tracking_quality']['events'].append(quality)
    initialized = copy.deepcopy(report['initialization']['exact_metric_events'][0])
    initialized.update(native_epoch=2, observation_segment=1, event_ordinal=9, bundle_id=3)
    report['initialization']['exact_metric_events'].append(initialized)
    commit = copy.deepcopy(report['initialization']['metric_filter_commits'][0])
    commit.update(native_epoch=2, observation_segment=1, event_ordinal=8, bundle_id=3)
    report['initialization']['metric_filter_commits'].append(commit)
    report['lifecycle']['native_epoch_transitions'] = [dict(native_epoch=2, previous_epoch=1,
        event_ordinal=10, observation_segment=1, reset_event_missing=False)]
    # Buffer contains IMU newer than the second snapshot when reset arrives.
    info[3]['observed_monotonic_ns'] = reset_received_ns
    result = native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
        propagation_output_path=tmp_path/'reset.jsonl')
    state = result['outputs'][-1]
    assert state['estimate']['localization_epoch'] == 2
    assert state['estimate']['source_time']['raw_value'] == info[3]['state_ns']
    assert state['estimate']['translation'] == [10, 20, 30]
    assert state['estimate']['quality']['state_output']['propagated_intervals'] == 0
    rows = [json.loads(line) for line in (tmp_path/'reset.jsonl').read_text().splitlines()]
    assert rows
    epochs = {row['estimate']['localization_epoch'] for row in rows}
    assert epochs == ({1, 2} if reset_received_ns == 160_000_100 else {1})
    statistics = {row['native_epoch']: row for row in result['propagation']['state_timestamp_epochs']}
    assert set(statistics) == epochs
    for epoch in epochs:
        group = [row for row in rows if row['estimate']['localization_epoch'] == epoch]
        state_times = {row['estimate']['source_time']['raw_value'] for row in group}
        stats = statistics[epoch]
        assert stats['row_count'] == len(group)
        assert stats['distinct_state_timestamp_count'] == len(state_times)
        assert stats['repeated_state_timestamp_count'] == len(group)-len(state_times)
        assert stats['first_state_ns'] == min(state_times) and stats['last_state_ns'] == max(state_times)
        expected = ((len(state_times)-1)*1e9/(max(state_times)-min(state_times))
                    if len(state_times) > 1 else None)
        assert stats['state_timestamp_frequency_hz'] == pytest.approx(expected)


def seal_derived_imu_contract(tmp_path, contract, delivered, *, kind='imu_bias'):
    """Seal tiny synthetic ledgers so tests exercise actual file/hash boundaries."""
    import json
    from rm27.perception.vision.localization.experiments.common import file_hash, write_json
    parent, child = tmp_path/'original-input', tmp_path/'derived-input'
    parent.mkdir(); child.mkdir()
    def ledger(path, rows):
        path.write_text(''.join(json.dumps(row, allow_nan=False)+'\n' for row in rows))
    ledger(parent/'expected.jsonl', contract['source_rows'])
    original_hash = 'b'*64
    source = dict(purpose='EUROC_PUBLIC_REPLAY', truth_included=False, mode='mono',
        original_archive_sha256=original_hash, counts=dict(imu0=len(contract['source_rows'])),
        artifacts_sha256={'expected.jsonl':file_hash(parent/'expected.jsonl')})
    write_json(parent/'input.json', source)
    ledger(child/'source-expected.jsonl', contract['source_rows'])
    ledger(child/'expected.jsonl', delivered)
    declaration = dict(experiment_kind='ARTIFICIAL_DISTURBANCE', validation=dict(status='PASS'),
        profile=dict(kind=kind, sensor_ids=['imu0']), parent_input_path=str(parent),
        parent_input_sha256=file_hash(parent/'input.json'), parent_artifacts_sha256=source['artifacts_sha256'])
    write_json(child/'derivation.json', declaration)
    prepared = dict(source, counts=dict(imu0=len(delivered)), source_counts=source['counts'],
        derivation=declaration, artifacts_sha256={name:file_hash(child/name) for name in
            ('source-expected.jsonl', 'expected.jsonl', 'derivation.json')})
    write_json(child/'input.json', prepared)
    contract.update(source_imu_id='EuRoC:'+original_hash+':imu0', delivered_rows=copy.deepcopy(delivered),
        delivery_semantics='DECLARED_DISTURBANCE_PAYLOAD_ORIGINAL_SAMPLE_IDENTITY',
        delivery_evidence=dict(schema_version=1, experiment_kind='ARTIFICIAL_DISTURBANCE',
            prepared_input_path=str(child/'input.json'), prepared_input_sha256=file_hash(child/'input.json')))
    return child


def test_declared_bias_uses_delivered_values_but_original_ids_and_visual_freshness(tmp_path):
    import json
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    original = copy.deepcopy(contract['source_rows'])
    delivered = copy.deepcopy(original)
    for index in range(18, 25):
        delivered[index]['gyro'][0] += .1
        delivered[index]['accel'][2] -= .5
        raw[index].update(gyro=delivered[index]['gyro'][:], accel=delivered[index]['accel'][:])
    seal_derived_imu_contract(tmp_path, contract, delivered)
    path = tmp_path/'derived-propagation.jsonl'
    result = native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                               propagation_output_path=path)
    assert result['propagation']['status'] == 'PASS'
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert rows and contract['source_rows'] == original
    assert all(row['raw_sensor_ns'] == original[row['source_sequence']]['state_ns'] for row in rows)
    assert all(row['estimate']['quality']['state_output']['last_accepted_visual_time']['raw_value']
               == info[1]['state_ns'] for row in rows)
    meta = result['propagation']['input_contract']
    assert meta['delivery_evidence']['experiment_kind'] == 'ARTIFICIAL_DISTURBANCE'
    assert 'source_rows' not in meta and 'delivered_rows' not in meta
    assert not rows[-1]['usable']  # artificial bias cannot refresh visual health


def test_declared_drop_keeps_full_original_ordinals_and_gap_rejection(tmp_path):
    import json
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    delivered = copy.deepcopy(contract['source_rows'])
    del delivered[18:20]; del raw[18:20]
    seal_derived_imu_contract(tmp_path, contract, delivered, kind='imu_gap')
    path = tmp_path/'derived-gap.jsonl'
    result = native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                               propagation_output_path=path)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert result['propagation']['status'] == 'FAIL'
    assert any(row['source_sequence'] == 20 for row in rows)
    assert all(not row['usable'] for row in rows if row['source_sequence'] >= 20)


@pytest.mark.parametrize('change', ['missing_proof', 'missing_rows', 'forged_original',
    'delivered_payload', 'unknown_identity', 'duplicate', 'decreasing', 'nonfinite',
    'hash', 'path', 'symlink', 'file_mutation', 'declaration_mutation', 'parent_mutation'])
def test_derived_delivery_rejects_unqualified_or_mutated_proof_before_output(tmp_path, change):
    import json
    from rm27.perception.vision.localization.experiments.common import file_hash, write_json
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    child = seal_derived_imu_contract(tmp_path, contract, copy.deepcopy(contract['source_rows']))
    proof = contract['delivery_evidence']
    if change == 'missing_proof':del contract['delivery_evidence']
    elif change == 'missing_rows':del contract['delivered_rows']
    elif change == 'forged_original':contract['source_rows'][0]['gyro'][0] += 1
    elif change == 'delivered_payload':raw[20]['accel'][0] += 1
    elif change == 'nonfinite':raw[20]['gyro'][0] = float('nan')
    elif change == 'hash':proof['prepared_input_sha256'] = 'a'*64
    elif change == 'path':proof['prepared_input_path'] = str(child/'..'/'derived-input'/'input.json')
    elif change == 'symlink':
        link = tmp_path/'input.json'; link.symlink_to(child/'input.json')
        proof['prepared_input_path'] = str(link)
    elif change == 'file_mutation':
        path=child/'expected.jsonl'; path.write_text(path.read_text()+'\n')
    elif change == 'parent_mutation':
        path=tmp_path/'original-input'/'input.json'; path.write_text(path.read_text()+'\n')
    elif change == 'declaration_mutation':
        manifest=json.loads((child/'input.json').read_text())
        manifest['derivation']['profile']['kind']='unknown'
        write_json(child/'input.json',manifest)
        proof['prepared_input_sha256']=file_hash(child/'input.json')
    else:
        # Even a self-consistent derived manifest may not invent, duplicate or
        # reorder original sample identities. These fail beyond the hash check.
        delivered=contract['delivered_rows']
        if change == 'unknown_identity':delivered[20]['state_ns'] += 1
        elif change == 'duplicate':delivered[20]['state_ns'] = delivered[19]['state_ns']
        else:delivered[19],delivered[20] = delivered[20],delivered[19]
        (child/'expected.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in delivered))
        manifest=json.loads((child/'input.json').read_text())
        manifest['artifacts_sha256']['expected.jsonl']=file_hash(child/'expected.jsonl')
        write_json(child/'input.json',manifest)
        proof['prepared_input_sha256']=file_hash(child/'input.json')
    path=tmp_path/'rejected-derived.jsonl'
    with pytest.raises(ValueError, match='PROPAGATION_INPUT_INVALID'):
        native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                          propagation_output_path=path)
    assert not path.exists()


def test_unannounced_bias_does_not_get_legacy_payload_exception(tmp_path):
    report, info, raw, contract = native_imu_projection_fixture(tmp_path)
    raw[20]['gyro'][0] += .1
    path=tmp_path/'undeclared.jsonl'
    with pytest.raises(ValueError, match='PROPAGATION_INPUT_INVALID'):
        native_projection(report, info, raw_imu=raw, imu_input_contract=contract,
                          propagation_output_path=path)
    assert not path.exists()


def health_episode_fixture(*, tail=(0,)):
    data, info, native = fault_response_fixture()
    # The same fault fixture gains exact native identities, never native sequence IDs.
    info = info[:-1]+[dict(state_ns=1500+100*index, stage=2, tracking_quality=quality)
        for index, quality in enumerate(tail)]
    for index, row in enumerate(info):
        row['observed_monotonic_ns'] = row['state_ns']+10_000
        row['native_sequence'] = 900+index
    native['tracking_quality'] = dict(events=[dict(sensor_ns=row['state_ns'],
        camera_index=10+index*2, bundle_id=index, observation_segment=1,
        event_ordinal=20*index+12, tracking_quality=row['tracking_quality'])
        for index, row in enumerate(info)])
    native['visual_corrections']['events'] = [dict(sensor_ns=row['state_ns'],
        camera_index=10+index*2, bundle_id=index, observation_segment=1,
        event_ordinal=20*index+10, native_epoch=1, constraint_count=20,
        state_source='EXPLICIT_NATIVE_ACCEPTED_VISUAL_CORRECTION')
        for index, row in enumerate(info) if row['state_ns'] != 1200]
    native['lifecycle'] = dict(reset_events=[])
    native['coverage'] = dict(complete_trace=None, complete_trace_status='NOT_RUN',
        event_limit_reached=False, paired_spans_complete=True)
    states = dict(status='PASS', outputs=[dict(sensor_ns=row['state_ns'],
        trigger='INFO_CALLBACK', usable=row['stage'] == 2 and row['tracking_quality'] == 2,
        reasons=[] if row['stage'] == 2 and row['tracking_quality'] == 2
            else ['NATIVE_INFO_NOT_TRACKING_GOOD'],
        localization_epoch=1, observation_timestamp=dict(raw_value=row['observed_monotonic_ns'],
            unit='ns', clock_domain='observer', evidence_status='VERIFIED'))
        for row in info])
    return data, info, native, states


def test_health_episodes_preserve_two_good_recovery_and_count_later_bad():
    from tools.schurvins.summarize import fault_response, health_episode_summary
    data, info, native, states = health_episode_fixture()
    response = fault_response(data, info, native, states)
    original = copy.deepcopy(response)
    original_inputs = copy.deepcopy((info, native, states))
    report = health_episode_summary(info, native, states, fault=response)
    assert response == original and (info, native, states) == original_inputs
    assert response['recovery']['status'] == 'OBSERVED'
    assert response['recovery']['continuous_evidence_through_sensor_ns'] == 1400
    assert response['fault_window']['end_sensor_ns'] == 1250
    assert sum(not (row['stage'] == 2 and row['tracking_quality'] == 2) for row in info) == 2
    assert report['postfault']['not_good_samples'] == 1  # BAD at 1200 is inside the fault.
    after = report['after_recovery']
    assert after['not_good_samples'] == after['not_good_episode_count'] == 1
    assert after['first_subsequent_not_good']['source_index'] == 20
    assert after['second_recovery_sensor_ns'] == 1400
    assert after['observed_samples'] == 1
    assert 'exclusive' in after['boundary']
    good = report['native_info']['longest_good_episode']
    assert good['first_sensor_ns'] == 1000 and good['last_sensor_ns'] == 1100
    assert good['observed_source_span_ns'] == good['observed_receipt_span_ns'] == 100
    assert report['native_info']['native_trace_completeness']['complete_trace'] is None


@pytest.mark.parametrize('tail,bad,episodes', [((0,2,0,2,0),3,3),
    ((0,0,2,0,2,0,2,0,2,0,0),7,5)])
def test_health_episode_counts_samples_separately_from_later_bad_episodes(tail, bad, episodes):
    from tools.schurvins.summarize import fault_response, health_episode_summary
    data, info, native, states = health_episode_fixture(tail=tail)
    response = fault_response(data, info, native, states)
    after = health_episode_summary(info, native, states, fault=response)['after_recovery']
    assert after['not_good_samples'] == bad
    assert after['not_good_episode_count'] == episodes
    assert after['unknown_epoch_samples'] == 0
    assert after['native_epoch'] == 1 and after['second_recovery_sensor_ns'] == 1400


@pytest.mark.parametrize('change', ['missing_epoch', 'next_epoch', 'reset', 'reset_unknown_order'])
def test_health_episode_followup_never_fills_epoch_or_crosses_reset(change):
    from tools.schurvins.summarize import fault_response, health_episode_summary
    data, info, native, states = health_episode_fixture()
    response = fault_response(data, info, native, states)
    last = native['visual_corrections']['events'][-1]
    if change == 'missing_epoch':
        native['visual_corrections']['events'].pop()
    elif change == 'next_epoch':
        last['native_epoch'] = 2
    else:
        native['lifecycle']['reset_events'] = [dict(event_ordinal=105)]
        if change == 'reset_unknown_order':
            native['visual_corrections']['events'].pop()
            native['tracking_quality']['events'].pop()
    after = health_episode_summary(info, native, states, fault=response)['after_recovery']
    assert after['not_good_samples'] == 0
    assert after['unknown_epoch_samples'] == int(change == 'missing_epoch')
    assert after['observed_samples'] == int(change == 'missing_epoch')
    if change.startswith('reset'):
        assert 'RESET' in after['boundary']
    elif change == 'next_epoch':
        assert 'NATIVE_EPOCH' in after['boundary']


@pytest.mark.parametrize('change', ['unknown_clock', 'other_clock', 'missing_receipt', 'wrong_unit', 'unverified'])
def test_health_episode_source_span_does_not_depend_on_qualified_receipts(change):
    from tools.schurvins.summarize import health_episode_summary
    _, info, native, states = health_episode_fixture()
    stamp = states['outputs'][1]['observation_timestamp']
    if change == 'missing_receipt':
        states['outputs'][1].pop('observation_timestamp')
    elif change == 'wrong_unit':
        stamp['unit'] = 's'
    elif change == 'unverified':
        stamp['evidence_status'] = 'UNKNOWN'
    else:
        stamp['clock_domain'] = 'UNKNOWN' if change == 'unknown_clock' else 'another-observer'
    episode = health_episode_summary(info, native, states)['native_info']['first_good_episode']
    assert episode['observed_source_span_ns'] == 100
    assert episode['observed_receipt_span_ns'] is None


@pytest.mark.parametrize('change', ['info_order', 'info_duplicate', 'receipt_rewind', 'mapped_tick', 'native_duplicate', 'source_conflict'])
def test_health_episode_rejects_order_or_fabricated_watchdog_time(change):
    from tools.schurvins.summarize import health_episode_summary
    _, info, native, states = health_episode_fixture()
    if change == 'info_order':
        info[0],info[1] = info[1],info[0]
    elif change == 'info_duplicate':
        info.insert(1, copy.deepcopy(info[0]))
    elif change == 'receipt_rewind':
        states['outputs'][-1]['observation_timestamp']['raw_value'] = 1
    elif change == 'native_duplicate':
        native['tracking_quality']['events'].append(copy.deepcopy(native['tracking_quality']['events'][0]))
    elif change == 'source_conflict':
        native['tracking_quality']['events'][0]['camera_index'] += 1
    else:
        states['outputs'][-1]['trigger'] = 'INDEPENDENT_SCHEDULE'
    with pytest.raises(ValueError):
        health_episode_summary(info, native, states)


def test_health_episode_missing_or_capped_evidence_stays_observed_only():
    from tools.schurvins.summarize import health_episode_summary
    _, info, native, states = health_episode_fixture()
    assert health_episode_summary(None)['status'] == 'NOT_RUN'
    missing = health_episode_summary(info)
    assert missing['projected_output']['status'] == 'NOT_RUN'
    assert missing['native_info']['unknown_epoch_samples'] == len(info)
    native['status'] = 'FAIL'
    native['coverage'].update(event_limit_reached=True, complete_trace=False)
    capped = health_episode_summary(info, native, states)
    assert capped['native_info']['unknown_epoch_samples'] == len(info)
    assert capped['native_info']['native_trace_completeness']['complete_trace'] is False
    assert capped['after_recovery']['status'] == 'NOT_RUN'


def test_health_episode_native_good_and_final_stopped_watchdog_remain_distinct(tmp_path):
    from tools.schurvins.summarize import health_episode_summary
    report, info = native_projection_fixture(tmp_path)
    states = native_projection(report, info,
        watchdog_events=[watchdog_tick(t) for t in (0,80_000_000,200_000_000)],
        watchdog_complete=True)
    # Real tracker watchdog output keeps the last accepted correction time unchanged.
    states['propagation'] = dict(refreshes_visual_time=False, output_complete=True)
    summary = health_episode_summary(info, report, states)
    assert summary['final_info']['classification'] == 'GOOD'
    assert summary['final_watchdog']['classification'] == 'INVALID'
    assert summary['final_watchdog']['sensor_ns'] is None
    assert 'VISUAL_STREAM_STOPPED' in summary['final_watchdog']['reasons']
    first_visual = next(row['last_accepted_visual_time'] for row in states['outputs']
        if row.get('sensor_ns') == info[1]['state_ns'])
    assert summary['final_watchdog']['last_accepted_visual_time'] == first_visual
    assert summary['propagation_visual_time_contract']['refreshes_visual_time'] is False
    assert states['watchdog']['timeline_complete']
    assert summary['projected_output']['longest_good_episode_basis'].startswith('receipt')


def test_health_episode_interleaved_watchdog_keeps_one_usable_snapshot_episode(tmp_path):
    from tools.schurvins.summarize import health_episode_summary
    report, info = native_projection_fixture(tmp_path)
    states = native_projection(report, info,
        watchdog_events=[watchdog_tick(t) for t in (0,80_000_000,200_000_000)],
        watchdog_complete=True)
    summary = health_episode_summary(info, report, states)
    usable = [episode for episode in summary['projected_output']['episodes']
              if episode['classification'] == 'USABLE']
    assert len(usable) == 1 and usable[0]['observed_samples'] == 3
    assert usable[0]['observed_receipt_span_ns'] == (
        info[2]['observed_monotonic_ns']-info[1]['observed_monotonic_ns'])
    assert usable[0]['observed_source_span_ns'] is None
    assert usable[0]['observation_segment'] is None


@pytest.mark.parametrize('clock', ['another-observer', 'UNKNOWN'])
def test_health_episode_does_not_compare_receipt_values_from_different_clocks(clock):
    from tools.schurvins.summarize import health_episode_summary
    _, info, native, states = health_episode_fixture()
    info[1]['observed_monotonic_ns'] = 100
    states['outputs'][1]['observation_timestamp'].update(raw_value=100, clock_domain=clock)
    episode = health_episode_summary(info, native, states)['native_info']['first_good_episode']
    assert episode['observed_source_span_ns'] == 100
    assert episode['observed_receipt_span_ns'] is None


def test_sealed_health_episode_summary_keeps_run_failure_and_openvins_unknown(tmp_path):
    import json
    from tools.openvins.native_accuracy import digest
    from tools.schurvins.summarize import summarize_run
    _, info, native, states = health_episode_fixture()
    assert summarize_run(tmp_path)['health_episodes']['status'] == 'NOT_RUN'
    (tmp_path/'raw').mkdir()
    for row in info:
        row['processing_time_s'] = .001
    (tmp_path/'raw/info.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in info))
    (tmp_path/'native-observations.json').write_text(json.dumps(native))
    (tmp_path/'native-state-output.json').write_text(json.dumps(states))
    frozen = {str(p.relative_to(tmp_path)):digest(p) for p in tmp_path.rglob('*') if p.is_file()}
    run = dict(status='FAILED', config=dict(backend='SchurVINS'), output_sha256=frozen)
    (tmp_path/'run.json').write_text(json.dumps(run))
    summary = summarize_run(tmp_path)
    assert summary['status'] == 'FAILED' and summary['health_episodes']['status'] == 'OBSERVED'
    run['config']['backend'] = 'OpenVINS'
    (tmp_path/'run.json').write_text(json.dumps(run))
    assert summarize_run(tmp_path)['health_episodes']['status'] == 'NOT_RUN'
    run['config']['backend'] = 'SchurVINS'
    run.pop('output_sha256')
    (tmp_path/'run.json').write_text(json.dumps(run))
    assert summarize_run(tmp_path)['health_episodes']['status'] == 'NOT_RUN'
