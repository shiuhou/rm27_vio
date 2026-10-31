"""Exercise real local process measurement; no hardware or estimator qualification."""
import csv
import json
from pathlib import Path
import sys

import pytest

from tools.schurvins.benchmark import run, summarize, timeline_summary, repeat_summary


def command(code):
    return [sys.executable, '-c', code]


def test_actual_process_resources_and_artifact_provenance(tmp_path):
    source = tmp_path/'input.txt'
    source.write_text('frozen input')
    output = tmp_path/'actual'
    result = run(output, command('import time; data=bytearray(8*1024*1024); time.sleep(.12)'),
                 rate=.5, artifacts=[source], interval_s=.02)
    assert result['status'] == 'PASS'
    assert result['resources']['maximum_rss_kib'] > 0
    assert result['sampled_resources']['peak_sampled_process_tree_rss_kib'] > 0
    assert result['exit_code'] == 0
    assert result['evidence_class'] == 'PC_NATIVE_MEASUREMENT_ONLY'
    report = summarize(benchmark=output/'benchmark.json')
    assert report['estimator_acceptance'] == 'NOT_RUN'
    assert report['effective_valid_output_frequency']['status'] == 'NOT_RUN'
    with (output/'native-resource.txt').open('a') as stream:
        stream.write('changed evidence')
    with pytest.raises(ValueError, match='hash mismatch'):
        summarize(benchmark=output/'benchmark.json')
    with pytest.raises(FileExistsError):
        run(output,command('pass'),rate=.5)


def test_real_failure_and_timeout_preserve_exit_evidence(tmp_path):
    failure = run(tmp_path/'bad', command('raise SystemExit(7)'), rate=1.,interval_s=.02)
    assert failure['status'] == 'FAIL' and failure['exit_code'] == 7
    timeout = run(tmp_path/'timeout', command('import time; time.sleep(10)'),rate=1.,timeout_s=.04,interval_s=.02)
    assert timeout['status'] == 'FAIL' and timeout['failure'] == 'TIMEOUT'
    assert timeout['exit_code'] != 0


def test_artifact_mutation_fails_a_zero_exit_process(tmp_path):
    source=tmp_path/'source';source.write_text('before')
    result=run(tmp_path/'changed',command(f'from pathlib import Path; Path({str(source)!r}).write_text("after")'),
               rate=.5,artifacts=[source],interval_s=.02)
    assert result['exit_code'] == 0 and result['status'] == 'FAIL'
    assert result['failure'] == 'INPUT_ARTIFACT_MUTATED'


@pytest.mark.parametrize('program',['ssh','docker','podman'])
def test_remote_client_is_not_native_resources(tmp_path,program):
    with pytest.raises(ValueError,match='native binary'):
        run(tmp_path/'no', [program,'anything'],rate=1.)
    assert not (tmp_path/'no').exists()


def test_pc_cannot_be_labeled_m3c(tmp_path):
    with pytest.raises(ValueError,match='M3C/MaixCAM2'):
        run(tmp_path/'no',command('pass'),rate=1.,target='m3c-native')
    assert not (tmp_path/'no').exists()


def write_timeline(path,rows):
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['sensor_ns','scheduled_monotonic_ns','start_monotonic_ns','end_monotonic_ns'])
        writer.writeheader();writer.writerows(rows)


def test_pacing_lag_is_separate_from_real_queue_and_valid_output(tmp_path):
    p=tmp_path/'timeline.csv'
    write_timeline(p,[dict(sensor_ns=100+i*10,scheduled_monotonic_ns=1000+i*10,
                           start_monotonic_ns=1000+i*20,end_monotonic_ns=1015+i*20) for i in range(4)])
    report=timeline_summary(p)
    assert report['callback_processing_ns']['p50'] == 15
    assert report['playback_start_lateness_ns']['p95'] == pytest.approx(28.5)
    assert report['playback_schedule_backlog_frames']['max'] == 1
    assert report['actual_input_queue_depth']['status'] == 'NOT_RUN'
    assert report['effective_valid_output_frequency']['status'] == 'NOT_RUN'
    assert report['lateness_change_ns'] == 30
    assert report['processing_completion_frequency']['hz'] == pytest.approx(50_000_000)


def test_timeline_rejects_clock_rewind(tmp_path):
    p=tmp_path/'timeline.csv'
    write_timeline(p,[dict(sensor_ns=2,scheduled_monotonic_ns=4,start_monotonic_ns=5,end_monotonic_ns=8),
                      dict(sensor_ns=1,scheduled_monotonic_ns=5,start_monotonic_ns=9,end_monotonic_ns=10)])
    with pytest.raises(ValueError,match='increase'):
        timeline_summary(p)


def test_cohort_retains_failed_member_and_refuses_mixed_arguments(tmp_path):
    reports=[]
    for i in range(2):
        p=tmp_path/str(i); p.mkdir()
        data=dict(status='PASS' if i == 0 else 'FAIL',failure=None if i == 0 else 'TIMEOUT',rate=.5,target='pc-native',
                  binary_sha256='binary',command_template=['binary','--out','{output}'],artifacts_before_sha256={},
                  evidence_sha256={},resources=dict(status='PASS',user_cpu_s=1.,system_cpu_s=.1,maximum_rss_kib=200))
        path=p/'benchmark.json';path.write_text(json.dumps(data));reports.append(path)
    cohort=repeat_summary(reports)
    assert cohort['status'] == 'FAIL' and cohort['passed'] == 1 and cohort['planned'] == 2
    data['command_template'].append('--different')
    reports[-1].write_text(json.dumps(data))
    with pytest.raises(ValueError,match='differs'):
        repeat_summary(reports)


def test_missing_native_trace_stays_not_run(tmp_path):
    ledger=tmp_path/'ledger.json';ledger.write_text('[1, 2, 3]')
    report=summarize(events=tmp_path/'absent.jsonl',ledger=ledger)
    assert report['status'] == 'NOT_RUN'
    assert report['timings']['frontend']['status'] == 'NOT_RUN'


def test_headless_columns_keep_stage_and_numeric_pose_separate(tmp_path):
    p=tmp_path/'frames.csv'
    p.write_text('timestamp_ns,scheduled_ns,start_ns,end_ns,stage,quality,pose_emitted,imu_added,queue_pending_frames\n'
                 '100,1000,1000,1010,1,0,0,10,0\n'
                 '110,1020,1020,1030,2,2,1,10,0\n'
                 '120,1040,1040,1050,2,2,1,10,0\n'
                 '130,1060,1060,1070,3,0,0,10,0\n')
    report=timeline_summary(p)
    assert report['native_stage_observations']['tracking_exit_count'] == 1
    assert report['published_pose_frequency']['sensor_clock']['hz'] == 100_000_000
    assert report['effective_valid_output_frequency']['status'] == 'NOT_RUN'


def test_process_group_descendant_is_stopped_before_evidence_seal(tmp_path):
    code='import subprocess,sys; subprocess.Popen([sys.executable,"-c","import time; time.sleep(20)"])'
    result=run(tmp_path/'descendant', command(code),rate=1.,interval_s=.02)
    assert result['exit_code'] == 0
    assert result['cleanup']['non_zombie_process_group_absence_verified']
    assert result['cleanup']['remaining_pids'] == []


def test_directory_artifact_detects_added_membership(tmp_path):
    source=tmp_path/'inputs';source.mkdir();(source/'one').write_text('input')
    code=f'from pathlib import Path; Path({str(source / "two")!r}).write_text("new input")'
    result=run(tmp_path/'directory',command(code),rate=1.,artifacts=[source],interval_s=.02)
    assert result['status'] == 'FAIL' and result['failure'] == 'INPUT_ARTIFACT_MUTATED'


def test_native_nested_output_is_sealed(tmp_path):
    code='from pathlib import Path; import os; p=Path(os.environ["RM27_BENCHMARK_OUTPUT"])/"native"; p.mkdir(); (p/"pose").write_text("raw")'
    p=tmp_path/'nested';result=run(p,command(code),rate=1.,interval_s=.02)
    assert result['status'] == 'PASS' and 'native/pose' in result['evidence_sha256']
    (p/'native/pose').write_text('changed')
    with pytest.raises(ValueError,match='hash mismatch'):
        summarize(benchmark=p/'benchmark.json')
