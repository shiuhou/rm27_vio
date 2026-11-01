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
    for p,ate in zip(paths[:2], [.02,.04]):
        reports[p]['metrics'] = dict(ate_rmse_m=ate, rpe_1s_translation_rmse_m=None)
    report = summarize_group('TEST_FIXTURE_ONLY', paths, reports)
    assert report['planned_run_count'] == 5 and report['scored_run_count'] == 2
    assert report['successful_run_count'] == 0 and report['failed_run_count'] == 4
    assert not report['cohort_complete']
    assert report['pairs_exceeding_1cm'] == 1
    assert report['ate_rmse_m']['samples'] == 2
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


def native_projection_fixture(tmp_path):
    """Production parser path fed explicit synthetic native events, not VIO."""
    import json
    from rm27.perception.vision.localization.experiments.native_events import analyze_native_events
    ns = [1403715278862142976+i*50_000_001 for i in range(4)]
    events = []
    def emit(kind, index, values):
        events.append(dict(schema_version=1, kind=kind, sensor_ns=ns[index], bundle_id=index,
                           monotonic_ns=len(events)+1, values=values))
    emit('metric_filter_commit', 0, [ns[0]/1e9, 1, 0, 0, 0, 0, 0, 0, 0, 0])
    emit('mono_metric_initialized', 0, [2.3, 1, 0, .01])
    emit('schur_visual_correction', 1, [42, 1, 2, 3, 1, 0, 0, 0, .1, .2, .3,
                                      .01, .02, .03, .001, .002, .003, 0, 0, -9.81, 1])
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


@pytest.mark.parametrize('change', [dict(stage=3), dict(tracking_quality=1)])
def test_native_relocalization_or_bad_quality_never_becomes_valid_state(tmp_path, change):
    report, info = native_projection_fixture(tmp_path)
    info[1].update(change)
    result = native_projection(report, info)
    assert result['status'] == 'FAIL' and not result['corrections']
    assert result['rejected_events'][0]['reason'] == 'NATIVE_INFO_NOT_TRACKING_GOOD'
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
    assert summary['fault_response']['status'] == 'NOT_RUN'
    assert summary['fault_response']['experiment_kind'] == 'ORIGINAL_DATA'
    frozen_report = (tmp_path/'native-observations.json')
    frozen_report.write_text(json.dumps(dict(report, status='FAIL')))
    with pytest.raises(ValueError, match='artifact hash'):
        summarize_run(tmp_path)


def test_unsealed_summary_cannot_promote_added_native_evidence(tmp_path):
    import json
    from tools.schurvins.summarize import summarize_run
    report, _ = native_projection_fixture(tmp_path)
    (tmp_path/'native-observations.json').write_text(json.dumps(report))
    (tmp_path/'run.json').write_text(json.dumps(dict(status='FAILED', config=dict(backend='SchurVINS'))))
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
    assert summary['recovery']['first_sensor_ns'] == 1300
    assert summary['recovery']['continuous_evidence_through_sensor_ns'] == 1400
    assert summary['recovery']['elapsed_from_fault_end_s'] == 50/1e9
    assert summary['offline_error_boundary']['status'] == 'NOT_RUN' and not summary['truth_drives_health']
    assert fault_response(data, info)['recovery']['status'] == 'NOT_RUN'
    native['visual_corrections']['events'][-1]['native_epoch'] = 2
    assert fault_response(data, info, native)['recovery']['status'] == 'NOT_RUN'


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
             observation_timestamp=dict(raw_value=row['state_ns']+10_000, clock_domain='observer')) for row in info])
    states['outputs'].insert(4, dict(sensor_ns=None, latest_info_sensor_ns=1300,
        trigger='INDEPENDENT_SCHEDULE', usable=False, reasons=['VISUAL_STREAM_STOPPED'],
        observation_timestamp=dict(raw_value=11_350, clock_domain='observer')))
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
