"""Acceptance regressions, synthetic observations only."""
import pytest

from tools.schurvins.headless_evaluate import assess, load_poses


def completed(mode='stereo'):
    return dict(schema_version=1, runtime='ROS_FREE_CPP', status='EXECUTION_COMPLETED', mode=mode, camera_selected=100,
        camera_input=100, imu_added=1000, imu_input=1000, tracking_losses=0,
        metric_initialization_verified=False)


def scored():
    return dict(scored_epoch_count=1, missing_after_last=0, missing_within_output_span=0)


def test_mono_numeric_score_never_certifies_unknown_metric_initialization():
    result = assess(completed('mono'), scored())
    assert result['status'] == 'FAIL'
    assert 'MONO_METRIC_INITIALIZATION_UNVERIFIED' in result['failure_codes']


@pytest.mark.parametrize('change,code', [
    ({'status':'FAIL'}, 'EXECUTION_INCOMPLETE'),
    ({'camera_selected':80}, 'PARTIAL_SEQUENCE'),
    ({'imu_added':990}, 'IMU_NOT_FULLY_DELIVERED'),
    ({'tracking_losses':1}, 'TRACKING_LOSS')])
def test_scoring_does_not_erase_failed_reliability(change, code):
    summary = completed(); summary.update(change)
    assert code in assess(summary, scored())['failure_codes']


def test_no_score_and_missing_tail_remain_failures():
    assert 'TRAJECTORY_NOT_EVALUABLE' in assess(completed(), None)['failure_codes']
    numerical = scored(); numerical['missing_after_last'] = 4
    assert 'OUTPUT_TAIL_OR_GAPS' in assess(completed(), numerical)['failure_codes']
    assert assess(completed(), scored())['operational_validity'] == 'NOT_RUN'


def test_headless_pose_preserves_integer_timestamp_and_quaternion(tmp_path):
    p = tmp_path/'poses.csv'
    p.write_text('timestamp_ns,tx,ty,tz,qw,qx,qy,qz\n1403715278862142976,1,2,3,1,0,0,0\n')
    row, = load_poses(p)
    assert row['state_ns'] == row['image_ns'] == 1403715278862142976
    assert row['quaternion_xyzw'] == [0.,0.,0.,1.]


@pytest.mark.parametrize('summary', [
    {'status':'EXECUTION_COMPLETED', 'tracking_losses':0},
    dict(completed(), mode='unknown'),
    dict(completed(), camera_input=True, camera_selected=True),
    dict(completed(), imu_input=-1, imu_added=-1),
])
def test_incomplete_or_invalid_execution_evidence_cannot_pass(summary):
    assert assess(summary, scored())['status'] == 'FAIL'
