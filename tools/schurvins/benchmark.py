"""Native process benchmark; measurement PASS never qualifies a VIO estimator.

Run this module on the execution machine, inside the target userspace when
applicable. Measuring a docker/ssh client does not measure its remote estimator.
The command is executed without a shell. A literal {output} in any command
argument expands to the new benchmark output directory.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import time

from rm27.perception.vision.localization.experiments.common import file_hash, read_json, write_json
from rm27.perception.vision.localization.experiments.native_events import analyze_native_events, _stats, _frequency
from tools.schurvins.summarize import native_resources


def _read_optional(path):
    try:
        return Path(path).read_text().strip().replace('\x00', ' ')
    except OSError:
        return None


def execution_identity():
    """Read the executing machine, never accept a caller-supplied hostname."""
    return dict(hostname=platform.node(), machine=platform.machine(), system=platform.system(),
                kernel=platform.release(), os_release=_read_optional('/etc/os-release'),
                device_tree_model=_read_optional('/proc/device-tree/model'),
                device_tree_compatible=_read_optional('/proc/device-tree/compatible'),
                cpuinfo_sha256=(file_hash('/proc/cpuinfo') if Path('/proc/cpuinfo').is_file() else None),
                cpu_count=os.cpu_count(), python=platform.python_version(),
                clock='time.monotonic_ns on the execution machine')


def _validate_target(target, identity):
    if target == 'm3c-native':
        model = (identity.get('device_tree_model') or '').lower()
        compatible = (identity.get('device_tree_compatible') or '').lower()
        if identity['machine'] not in ('aarch64', 'arm64') or not any(
                name in model + ' ' + compatible for name in ('maixcam2', 'm3c')):
            raise ValueError('m3c-native requires observed AArch64 and M3C/MaixCAM2 device-tree identity')
    if target == 'qemu-reference':
        return 'EMULATED_REFERENCE_ONLY'
    return 'M3C_NATIVE_MEASUREMENT_ONLY' if target == 'm3c-native' else 'PC_NATIVE_MEASUREMENT_ONLY'


def _process_tree(root_pid):
    """Sample Linux /proc descendants. Linux GNU time remains CPU/RSS authority."""
    processes = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry/'stat').read_text()
            tail = raw[raw.rfind(')')+2:].split()
            # fields after comm begin with state (field 3).
            processes[int(entry.name)] = dict(ppid=int(tail[1]), ticks=int(tail[11])+int(tail[12]),
                start_ticks=int(tail[19]), rss_kib=max(0, int(tail[21])) * os.sysconf('SC_PAGE_SIZE') // 1024)
        except (OSError, ValueError, IndexError):
            continue
    selected = {root_pid}
    while True:
        added = {pid for pid, row in processes.items() if row['ppid'] in selected} - selected
        if not added:
            break
        selected.update(added)
    rows = {str(pid): processes[pid] for pid in sorted(selected) if pid in processes}
    return dict(monotonic_ns=time.monotonic_ns(), process_count=len(rows),
                rss_sum_kib=sum(row['rss_kib'] for row in rows.values()), processes=rows,
                system_observations=_system_observations())



def _system_observations():
    observed = {}
    patterns = (('/sys/class/thermal', 'thermal_zone*/temp', 'temperature_millicelsius'),
                ('/sys/devices/system/cpu', 'cpu*/cpufreq/scaling_cur_freq', 'frequency_khz'))
    for root, pattern, name in patterns:
        values = {}
        for path in list(Path(root).glob(pattern))[:128]:
            try:
                values[str(path)] = int(path.read_text().strip())
            except (OSError, ValueError):
                continue
        observed[name] = values
    return observed

def _stop_owned(process):
    # A child may outlive the launched command. Terminate the owned session's
    # process group even when GNU time has already exited before sealing logs.
    running = process.poll() is None
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    if running:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    def live_members():
        members = []
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                raw = path.read_text()
                fields = raw[raw.rfind(')')+2:].split()
                if int(fields[2]) == process.pid and fields[0] != 'Z':
                    members.append(int(path.parent.name))
            except (OSError, ValueError, IndexError):
                continue
        return members
    remaining = live_members()
    if remaining:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        deadline = time.monotonic()+5
        while remaining and time.monotonic() < deadline:
            time.sleep(.02)
            remaining = live_members()
    return dict(non_zombie_process_group_absence_verified=not remaining, remaining_pids=remaining,
                scope='owned native process group; detached children are outside this runner contract')


def artifact_hashes(paths):
    """Freeze regular-file membership, contents, and symbolic-link targets."""
    frozen = {}
    for value in paths:
        path = Path(value).absolute()
        if not path.exists():
            raise ValueError('artifact is missing: '+str(path))
        selected = [path] if not path.is_dir() else sorted(path.rglob('*'))
        for p in selected:
            if p.is_symlink():
                frozen[str(p)] = dict(kind='symlink', target=os.readlink(p),
                                     target_file_sha256=file_hash(p) if p.is_file() else None)
            elif p.is_file():
                frozen[str(p)] = dict(kind='file', sha256=file_hash(p))
    return frozen


def run(output, command, *, rate, target='pc-native', timeout_s=900, interval_s=.1, artifacts=()):
    """Execute one owned native process group and seal measured resource evidence."""
    if rate not in (0.5, 1.0) or not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('rate must be 0.5 or 1.0 and timeout positive')
    if not math.isfinite(interval_s) or not .01 <= interval_s <= 10:
        raise ValueError('sample interval must be between .01 and 10 seconds')
    if not command or Path(command[0]).name in ('docker', 'ssh', 'podman'):
        raise ValueError('run benchmark beside the native binary, not around a docker/ssh/podman client')
    identity = execution_identity()
    evidence_class = _validate_target(target, identity)
    binary = shutil.which(command[0])
    if not binary:
        raise ValueError('native command executable missing')
    time_binary = Path('/usr/bin/time')
    if not time_binary.is_file():
        raise ValueError('GNU /usr/bin/time is required for authoritative process resources')
    frozen = artifact_hashes(artifacts)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    command_template = [str(arg) for arg in command]
    command = [arg.replace('{output}', str(output)) for arg in command_template]
    manifest = dict(schema_version=1, status='NOT_RUN', scope='NATIVE_PROCESS_MEASUREMENT_ONLY',
        estimator_acceptance='NOT_RUN', evidence_class=evidence_class, target=target, rate=rate,
        identity=identity, command=command, command_template=command_template, binary=str(Path(binary).resolve()), binary_sha256=file_hash(binary),
        artifacts_before_sha256=frozen, sampler_interval_s=interval_s, timeout_s=timeout_s,
        sampler_sha256=file_hash(__file__))
    write_json(output/'benchmark.json', manifest)
    samples = []
    started = time.monotonic_ns()
    process, failure, cleanup = None, None, dict(non_zombie_process_group_absence_verified=True, remaining_pids=[])
    try:
        with (output/'process.log').open('wb') as log, (output/'resources.jsonl').open('w') as resource_log:
            env = dict(os.environ, LC_ALL='C', RM27_BENCHMARK_OUTPUT=str(output))
            process = subprocess.Popen([str(time_binary), '-v', '-o', str(output/'native-resource.txt'), '--', *command],
                stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            while process.poll() is None:
                row = _process_tree(process.pid)
                samples.append(row)
                resource_log.write(json.dumps(row, allow_nan=False)+'\n')
                resource_log.flush()
                if (time.monotonic_ns()-started)/1e9 > timeout_s:
                    failure = 'TIMEOUT'
                    break
                time.sleep(interval_s)
    except (OSError, KeyboardInterrupt) as exc:
        failure = type(exc).__name__ + ': ' + str(exc)
    finally:
        if process is not None:
            cleanup = _stop_owned(process)
    elapsed_s = (time.monotonic_ns()-started)/1e9
    try:
        after = artifact_hashes(artifacts)
    except (OSError, ValueError):
        after = {}
    if after != frozen:
        failure = 'INPUT_ARTIFACT_MUTATED'
    binary_after = file_hash(binary) if Path(binary).is_file() else None
    if binary_after != manifest['binary_sha256']:
        failure = 'EXECUTABLE_MUTATED'
    if not cleanup['non_zombie_process_group_absence_verified']:
        failure = 'PROCESS_GROUP_CLEANUP_UNVERIFIED'
    resources = native_resources((output/'native-resource.txt').read_text()) if (output/'native-resource.txt').is_file() else dict(status='NOT_RUN')
    if resources['status'] == 'OBSERVED':
        resources['status'] = 'PASS'
        resources['scope'] = 'launched native command process tree; GNU time max RSS is not simultaneous summed RSS'
    if samples:
        sampled = dict(status='PASS', samples=len(samples), peak_sampled_process_tree_rss_kib=max(s['rss_sum_kib'] for s in samples),
            scope='sampled simultaneous RSS sum, includes GNU time wrapper, can miss between-sample peaks',
            cpu_percent_of_one_core=(resources.get('average_cpu_cores')*100 if resources.get('average_cpu_cores') is not None else None),
            system_observations={name:_stats([value for sample in samples
                for value in sample['system_observations'][name].values()], unit=unit)
                for name,unit in (('temperature_millicelsius','millidegrees_C'), ('frequency_khz','kHz'))})
    else:
        sampled = dict(status='NOT_RUN')
    code = process.returncode if process is not None else None
    manifest.update(status='PASS' if code == 0 and failure is None and resources['status'] == 'PASS' else 'FAIL',
        failure=failure, exit_code=code, elapsed_monotonic_s=elapsed_s, resources=resources, sampled_resources=sampled,
        artifacts_after_sha256=after, binary_after_sha256=binary_after, cleanup=cleanup,
        evidence_sha256={str(p.relative_to(output)):file_hash(p) for p in output.rglob('*')
                         if p.is_file() and p != output/'benchmark.json'})
    write_json(output/'benchmark.json', manifest)
    return manifest


def read_ledger(path):
    """Read existing frozen expected.jsonl, or a JSON array of exact camera ns."""
    path = Path(path)
    if path.suffix == '.jsonl':
        return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    return read_json(path)


def timeline_summary(path):
    """Exact same-clock scheduling/start/end observations from headless runtime.

    Schedule lag is a playback pacing measurement, not an observed sensor FIFO.
    A synchronous runner has no separate producer queue; it must not claim an
    actual queue high-water mark merely from this timeline.
    """
    import bisect
    with Path(path).open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if rows and 'timestamp_ns' in rows[0]:
        aliases = dict(sensor_ns='timestamp_ns', scheduled_monotonic_ns='scheduled_ns',
                       start_monotonic_ns='start_ns', end_monotonic_ns='end_ns')
        if not all(source in rows[0] for source in aliases.values()):
            raise ValueError('headless timeline requires timestamp/scheduled/start/end ns columns')
        rows = [dict(row, **{name:row[source] for name,source in aliases.items()}) for row in rows]
    required = ('sensor_ns', 'scheduled_monotonic_ns', 'start_monotonic_ns', 'end_monotonic_ns')
    if not rows:
        return dict(status='NOT_RUN', reason='no completed frames')
    if not all(all(name in row for name in required) for row in rows):
        raise ValueError('timeline requires exact sensor/scheduled/start/end ns columns')
    converted = [{name:int(row[name]) for name in required} for row in rows]
    for i, row in enumerate(converted):
        if min(row.values()) < 0 or row['end_monotonic_ns'] < row['start_monotonic_ns']:
            raise ValueError('invalid native clock interval')
        if i and (row['sensor_ns'] <= converted[i-1]['sensor_ns'] or row['start_monotonic_ns'] < converted[i-1]['end_monotonic_ns']
                  or row['scheduled_monotonic_ns'] <= converted[i-1]['scheduled_monotonic_ns']):
            raise ValueError('timeline clocks must increase within one source/reset epoch')
    starts = [row['start_monotonic_ns'] for row in converted]
    ends = [row['end_monotonic_ns'] for row in converted]
    schedule = [row['scheduled_monotonic_ns'] for row in converted]
    lag = [max(0, row['start_monotonic_ns']-row['scheduled_monotonic_ns']) for row in converted]
    due_not_started = [max(0, bisect.bisect_right(schedule, start)-i-1) for i, start in enumerate(starts)]
    backlog_scope = 'lower bound from completed-frame prefix schedule; unprocessed source tail is unobserved'
    if all('queue_pending_frames' in row for row in rows):
        due_not_started = [int(row['queue_pending_frames']) for row in rows]
        if min(due_not_started) < 0:
            raise ValueError('negative playback schedule backlog')
        backlog_scope = 'runtime overdue unprocessed timestamps in complete source schedule; simulated playback arrival, not hardware queue'
    output = dict(status='PASS', count=len(rows),
        callback_processing_ns=_stats([end-start for start,end in zip(starts,ends)]),
        playback_start_lateness_ns=_stats(lag),
        playback_schedule_backlog_frames=_stats(due_not_started, unit='frames'),
        playback_schedule_backlog_scope=backlog_scope,
        lateness_change_ns=lag[-1]-lag[0],
        processing_completion_frequency=_frequency(ends, 'NATIVE_STEADY'),
        actual_input_queue_depth=dict(status='NOT_RUN', reason='synchronous schedule lag does not observe a producer queue'),
        effective_valid_output_frequency=dict(status='NOT_RUN', reason='requires explicit accepted native visual/state events'),
        clock='ONE_NATIVE_STEADY_CLOCK', scope='complete-image callback timings; not exposure-to-output latency')
    if all('stage' in row and 'pose_emitted' in row for row in rows):
        stages, emitted = [int(row['stage']) for row in rows], [int(row['pose_emitted']) for row in rows]
        if any(stage not in (0,1,2,3) for stage in stages) or any(value not in (0,1) for value in emitted):
            raise ValueError('unexpected native stage or pose-emission flag')
        selected = [i for i,value in enumerate(emitted) if value]
        output['published_pose_frequency'] = dict(status='PASS' if len(selected) >= 2 else 'NOT_RUN',
            sensor_clock=_frequency([converted[i]['sensor_ns'] for i in selected], 'CAMERA_SENSOR'),
            native_clock=_frequency([ends[i] for i in selected], 'NATIVE_STEADY'),
            qualification='numeric native tracking-stage poses; accepted corrections and metric validity unknown')
        output['native_stage_observations'] = dict(status='PASS',
            stage_counts={str(stage):stages.count(stage) for stage in sorted(set(stages))},
            tracking_exit_count=sum(a == 2 and b != 2 for a,b in zip(stages,stages[1:])),
            qualification='raw native stage transitions; no operational-validity qualification')
    return output


def summarize(*, events=None, ledger=None, timeline=None, benchmark=None):
    result = dict(schema_version=1, status='NOT_RUN', scope='BENCHMARK_OBSERVATIONS_ONLY', estimator_acceptance='NOT_RUN',
                  timings=dict(status='NOT_RUN'), resources=dict(status='NOT_RUN'), input_backlog=dict(status='NOT_RUN'),
                  visual_corrections=dict(status='NOT_RUN'), effective_valid_output_frequency=dict(status='NOT_RUN'),
                  tracking_loss=dict(status='NOT_RUN'), artifacts_sha256={})
    if benchmark:
        record = read_json(benchmark)
        for name, expected in record.get('evidence_sha256', {}).items():
            p = Path(benchmark).parent/name
            if not p.is_file() or file_hash(p) != expected:
                raise ValueError('benchmark measurement artifact hash mismatch: '+name)
        result.update(execution=record, resources=record.get('resources', dict(status='NOT_RUN')),
                      status=record['status'])
    if events:
        if ledger is None:
            raise ValueError('native event analysis requires exact source ledger')
        native = analyze_native_events(events, read_ledger(ledger))
        result.update(native=native, timings=native.get('timings'), visual_corrections=native.get('visual_corrections'),
            tracking_loss=dict(status='PASS' if native.get('lifecycle',{}).get('frontend_failures') else 'NOT_RUN',
                observed_frontend_failures=native.get('lifecycle',{}).get('frontend_failures'),
                qualification='frontend failure observations; absence does not certify continuous tracking'))
        if native.get('visual_corrections',{}).get('events'):
            result['native_accepted_correction_frequency'] = dict(status='PASS', epochs=native['visual_corrections']['epochs'],
                scope='explicit native correction frequency; operational validity remains unqualified')
        if native['status'] == 'FAIL':
            result['status'] = 'FAIL'
        elif native['status'] == 'PASS' and result['status'] != 'FAIL':
            result['status'] = 'PASS'
    if timeline:
        result['input_backlog'] = timeline_summary(timeline)
        if result['status'] == 'NOT_RUN':
            result['status'] = result['input_backlog']['status']
    for name, path in (('events',events),('ledger',ledger),('timeline',timeline),('benchmark',benchmark)):
        if path:
            result['artifacts_sha256'][name] = dict(path=str(Path(path).resolve()),
                status='PASS' if Path(path).is_file() else 'NOT_RUN',
                sha256=file_hash(path) if Path(path).is_file() else None)
    return result


def repeat_summary(records):
    """Keep all declared cohort members; reject mixed rate/target/binary groups."""
    if not records:
        return dict(status='NOT_RUN', planned=0, passed=0)
    reports = [read_json(path) for path in records]
    keys = ('rate', 'target', 'binary_sha256', 'artifacts_before_sha256', 'command_template')
    for path in records:
        summarize(benchmark=path)  # check original measured artifact membership and hashes
    expected = {key: reports[0].get(key) for key in keys}
    if any({key:row.get(key) for key in keys} != expected for row in reports):
        raise ValueError('repeat cohort configuration, executable, rate or target differs')
    identities = [{key:row.get('identity',{}).get(key) for key in ('hostname','machine','kernel','device_tree_model')}
                  for row in reports]
    if any(row != identities[0] for row in identities):
        raise ValueError('repeat cohort execution machines differ')
    passed = sum(row['status'] == 'PASS' for row in reports)
    return dict(status='PASS' if passed == len(reports) else 'FAIL', planned=len(reports), passed=passed,
                configuration=expected, members=[dict(path=str(path),status=row['status'],failure=row.get('failure'))
                                                for path,row in zip(records,reports)],
                cpu_s=_stats([row['resources']['user_cpu_s']+row['resources']['system_cpu_s'] for row in reports
                              if row.get('resources',{}).get('status') == 'PASS'], unit='s'),
                max_rss_kib=_stats([row['resources']['maximum_rss_kib'] for row in reports
                                   if row.get('resources',{}).get('status') == 'PASS'], unit='KiB'),
                scope='process execution/resource cohort only; estimator acceptance remains separate')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    launch = sub.add_parser('run')
    launch.add_argument('--output', required=True)
    launch.add_argument('--rate', type=float, choices=(.5,1.), required=True)
    launch.add_argument('--target', choices=('pc-native','m3c-native','qemu-reference'), default='pc-native')
    launch.add_argument('--timeout-s', type=float, default=900)
    launch.add_argument('--sample-interval-s', type=float, default=.1)
    launch.add_argument('--artifact', action='append', default=[])
    launch.add_argument('command', nargs=argparse.REMAINDER)
    report = sub.add_parser('summarize')
    for name in ('events','ledger','timeline','benchmark'):
        report.add_argument('--'+name)
    report.add_argument('--output', required=True)
    cohort = sub.add_parser('repeat')
    cohort.add_argument('--output',required=True)
    cohort.add_argument('records',nargs='+')
    args = parser.parse_args(argv)
    if args.action == 'run':
        command = args.command[1:] if args.command and args.command[0] == '--' else args.command
        result = run(args.output, command, rate=args.rate, target=args.target, timeout_s=args.timeout_s,
                     interval_s=args.sample_interval_s, artifacts=args.artifact)
    elif args.action == 'summarize':
        if Path(args.output).exists():
            raise ValueError('summary output already exists')
        result = summarize(events=args.events, ledger=args.ledger, timeline=args.timeline, benchmark=args.benchmark)
        write_json(args.output,result)
    else:
        if Path(args.output).exists():
            raise ValueError('repeat output already exists')
        result = repeat_summary(args.records)
        write_json(args.output,result)
    print(json.dumps(dict(status=result['status'],output=args.output),allow_nan=False))
    return 1 if result['status'] == 'FAIL' else 0


if __name__ == '__main__':
    raise SystemExit(main())
