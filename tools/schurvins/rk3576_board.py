#!/usr/bin/env python3
"""Standalone native RK3576 SchurVINS diagnostic launcher (Python stdlib only).

doctor is read-only. run uses an already built, immutable local Docker image;
it never builds/pulls/installs anything. A completed process is not VIO acceptance.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import subprocess
import sys
import threading
import time
import uuid


PROFILE = dict(memory_mib=2560, cpus=4, timeout_s=900, mode='mono', rate=0.5)
MEMORY_RESERVE_MIB = 512
OUTPUT_RESERVE_BYTES = 1024**3
PAYLOAD_LABEL = 'org.rm27.schur.payload'
SOURCE_ENV = ('source /opt/ros/melodic/setup.bash && '
              'source /ws/devel/setup.bash && ')
NATIVE_NODE = '/ws/devel/.private/svo_ros/lib/svo_ros/svo_node'


def read_json(path):
    def invalid(value):
        raise ValueError('invalid JSON number: ' + value)
    return json.loads(Path(path).read_text(), parse_constant=invalid)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def validate_bundle(bundle):
    """Validate every regular payload file, including unexpected membership."""
    bundle = Path(bundle).resolve(strict=True)
    if not bundle.is_dir() or any(c in str(bundle) for c in ',\n\r'):
        raise ValueError('invalid bundle directory/mount path')
    manifest = read_json(bundle/'candidate.json')
    if (manifest.get('schema_version') != 1 or
            manifest.get('status') != 'PREPARED_NOT_RUN' or
            manifest.get('platform') != 'linux/arm64' or
            manifest.get('runtime') != 'native-docker'):
        raise ValueError('unsupported candidate schema/status/platform/runtime')
    if manifest.get('profile') != PROFILE:
        raise ValueError('candidate must use the reviewed initial mono diagnostic profile')
    inventory = manifest.get('files_sha256')
    if not isinstance(inventory, dict) or not inventory:
        raise ValueError('candidate requires a complete files_sha256 inventory')
    for name, digest in inventory.items():
        path = PurePosixPath(name)
        if (not name or path.is_absolute() or '..' in path.parts or
                str(path) != name or name == 'candidate.json' or
                not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest)):
            raise ValueError('invalid candidate inventory path/hash: '+str(name))
    found = set()
    for path in bundle.rglob('*'):
        if path.is_symlink() or not (path.is_file() or path.is_dir()):
            raise ValueError('bundle must contain regular files/directories only: '+str(path))
        if path.is_file() and path != bundle/'candidate.json':
            found.add(path.relative_to(bundle).as_posix())
    if found != set(inventory):
        raise ValueError('candidate file membership changed: '+str(sorted(found ^ set(inventory))))
    required = {'Dockerfile', 'runtime-rootfs.tar.gz', 'tools/run_inner.py',
                'tools/ros_transport.py', 'input/input.json', 'input/sensors.zip',
                'input/expected.jsonl'}
    if not required <= found:
        raise ValueError('candidate missing required payload files')
    for name, digest in inventory.items():
        if file_hash(bundle/name) != digest:
            raise ValueError('candidate artifact hash changed: '+name)
    if manifest.get('runtime_archive_sha256') != inventory['runtime-rootfs.tar.gz']:
        raise ValueError('runtime archive binding differs from inventory')
    return bundle, manifest


def read_text(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, UnicodeError):
        return None


def board_sample(root=Path('/')):
    """Read raw thermal/frequency observations; no inferred thermal acceptance."""
    root = Path(root)
    temperatures, frequencies = {}, {}
    for zone in sorted((root/'sys/class/thermal').glob('thermal_zone*')):
        temperatures[zone.name] = dict(type=read_text(zone/'type'),
                                       temperature_millicelsius=read_text(zone/'temp'))
    policies = sorted((root/'sys/devices/system/cpu/cpufreq').glob('policy*'))
    for policy in policies:
        frequencies[policy.name] = {name: read_text(policy/name) for name in
                                   ('scaling_cur_freq', 'cpuinfo_cur_freq', 'scaling_governor',
                                    'scaling_min_freq', 'scaling_max_freq', 'affected_cpus')}
    return dict(monotonic_ns=time.monotonic_ns(), temperatures=temperatures,
                cpu_frequency_khz=frequencies)


def host_facts(root=Path('/')):
    root = Path(root)
    compatible = (read_text(root/'sys/firmware/devicetree/base/compatible') or
                  read_text(root/'proc/device-tree/compatible'))
    memory = {}
    for line in (read_text(root/'proc/meminfo') or '').splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[2] == 'kB':
            memory[fields[0].rstrip(':')] = int(fields[1])*1024
    return dict(machine=platform.machine(), kernel=platform.release(),
                os_release=read_text(root/'etc/os-release'),
                device_tree_compatible=compatible, memory_bytes=memory,
                cgroup_controllers=read_text(root/'sys/fs/cgroup/cgroup.controllers'),
                proc_cgroups=read_text(root/'proc/cgroups'), sample=board_sample(root))


def command_result(command, timeout=15):
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, timeout=timeout)
    return dict(command=command, exit_code=result.returncode,
                stdout=result.stdout, stderr=result.stderr)


class Docker:
    """Bind all commands to one local socket, never an implicit remote daemon."""
    def __init__(self, endpoint):
        if not isinstance(endpoint, str) or not endpoint.startswith('unix:///'):
            raise ValueError('a local unix Docker socket is required; remote/helper daemons rejected')
        self.prefix = ['docker', '--host', endpoint]

    @classmethod
    def local(cls):
        endpoint = os.environ.get('DOCKER_HOST')
        if not endpoint:
            result = command_result(['docker', 'context', 'inspect', '--format',
                                     '{{json .Endpoints.docker.Host}}'])
            if result['exit_code'] != 0:
                raise ValueError('cannot identify local Docker endpoint: '+result['stderr'])
            endpoint = json.loads(result['stdout'])
        return cls(endpoint)

    def call(self, args, timeout=15):
        return command_result(self.prefix+args, timeout)

    def json(self, args):
        result = self.call(args)
        if result['exit_code'] != 0:
            raise ValueError('Docker inspection failed: '+result['stderr'])
        return json.loads(result['stdout'])


def existing_parent(path):
    path = Path(path).resolve()
    while not path.exists():
        path = path.parent
    return path


def doctor(bundle, image, output_parent=None, *, docker=None, root=Path('/')):
    bundle, manifest = validate_bundle(bundle)
    if not re.fullmatch(r'sha256:[0-9a-f]{64}', image):
        raise ValueError('an exact immutable sha256 image ID is required')
    facts = host_facts(root)
    errors = []
    if facts['machine'].lower() not in ('aarch64', 'arm64'):
        errors.append('host machine is not native ARM64; x86/QEMU/helper hosts rejected')
    if 'rk3576' not in (facts['device_tree_compatible'] or '').lower():
        errors.append('RK3576 device-tree identity unavailable or mismatched')
    required_memory = (PROFILE['memory_mib']+MEMORY_RESERVE_MIB)*1024**2
    for key in ('MemTotal', 'MemAvailable'):
        if facts['memory_bytes'].get(key, 0) < required_memory:
            errors.append(key+' below 2560 MiB container cap + 512 MiB admission reserve')
    disk_path = existing_parent(output_parent or bundle.parent)
    disk_free = shutil.disk_usage(disk_path).free
    if disk_free < OUTPUT_RESERVE_BYTES:
        errors.append('output filesystem has less than 1 GiB diagnostic admission reserve')
    docker = docker or Docker.local()
    server = docker.json(['info', '--format', '{{json .}}'])
    if (server.get('Architecture', '').lower() not in ('aarch64', 'arm64') or
            server.get('OSType') != 'linux'):
        errors.append('Docker server is not native Linux ARM64')
    for capability in ('MemoryLimit', 'SwapLimit', 'CPUCfsPeriod', 'CPUCfsQuota', 'PidsLimit'):
        if server.get(capability) is not True:
            errors.append('Docker resource admission capability unavailable: '+capability)
    inspected = docker.json(['image', 'inspect', image])[0]
    if inspected.get('Id') != image or inspected.get('Architecture') != 'arm64' or inspected.get('Os') != 'linux':
        errors.append('local image ID/OS/architecture mismatch')
    if (inspected.get('Config', {}).get('Labels') or {}).get(PAYLOAD_LABEL) != manifest['runtime_archive_sha256']:
        errors.append('local image payload label does not match candidate runtime archive')
    docker_disk = dict(path=server.get('DockerRootDir'), free_bytes=None, status='UNAVAILABLE')
    if server.get('DockerRootDir'):
        try:
            docker_disk.update(free_bytes=shutil.disk_usage(server['DockerRootDir']).free,
                               status='OBSERVED')
        except OSError:
            pass
    return dict(status='FAIL' if errors else 'PASS', errors=errors, host=facts,
                docker_endpoint=docker.prefix[-1], docker_server=server,
                image=dict(id=image, architecture=inspected.get('Architecture'),
                           payload_sha256=(inspected.get('Config', {}).get('Labels') or {}).get(PAYLOAD_LABEL)),
                candidate_sha256=file_hash(bundle/'candidate.json'),
                output_disk=dict(path=str(disk_path), free_bytes=disk_free,
                                 admission_reserve_bytes=OUTPUT_RESERVE_BYTES),
                docker_disk=docker_disk, native_dependency_probe='NOT_RUN',
                algorithm_status='NOT_QUALIFIED',
                scope='read-only host/image admission; reserves are diagnostic bounds, not performance acceptance')


def cgroup_paths(membership, mountinfo):
    """Resolve the container PID's actual v1/v2 membership through host mounts."""
    groups = {}
    for line in membership.splitlines():
        _, controllers, path = line.split(':', 2)
        for controller in controllers.split(',') if controllers else ['unified']:
            groups[controller] = path
    paths = {}
    for line in mountinfo.splitlines():
        before, after = line.split(' - ', 1)
        fields, mount = before.split(), after.split()
        if mount[0] not in ('cgroup', 'cgroup2'):
            continue
        controllers = ['unified'] if mount[0] == 'cgroup2' else mount[2].split(',')
        for controller in controllers:
            member = groups.get(controller)
            if member is None:
                continue
            mount_root = PurePosixPath(fields[3])
            member_path = PurePosixPath(member)
            try:
                relative = member_path.relative_to(mount_root)
            except ValueError:
                continue
            if '..' in relative.parts:
                continue
            mount_point = re.sub(r'\\([0-7]{3})', lambda m: chr(int(m[1], 8)), fields[4])
            paths[controller] = str(Path(mount_point)/relative)
    return paths


def cgroup_sample(paths):
    names = {'unified': ('cpu.max', 'cpu.stat', 'memory.max', 'memory.current',
                         'memory.peak', 'memory.events'),
             'cpu': ('cpu.cfs_period_us', 'cpu.cfs_quota_us', 'cpu.stat'),
             'cpuacct': ('cpuacct.usage', 'cpuacct.stat'),
             'memory': ('memory.limit_in_bytes', 'memory.usage_in_bytes',
                        'memory.max_usage_in_bytes', 'memory.failcnt', 'memory.oom_control')}
    counters = {}
    for controller, directory in paths.items():
        for name in names.get(controller, ()):
            counters[name] = read_text(Path(directory)/name)
    observed = {key: value for key, value in counters.items() if value is not None}
    return dict(status='OBSERVED' if observed else 'UNAVAILABLE', counters=observed,
                unavailable=[key for key, value in counters.items() if value is None],
                scope='whole owned container, including ROS, transport and estimator; memory usage is not RSS')


def sample_container(docker, name, output, stopped):
    """Bounded observations; missing counters never imply a resource PASS."""
    paths, last_board, membership_observed = {}, 0, False
    with (output/'board-samples.jsonl').open('w') as stream:
        while not stopped.is_set():
            row = dict(monotonic_ns=time.monotonic_ns())
            if not membership_observed:
                try:
                    state = docker.json(['container', 'inspect', '--format', '{{json .State}}', name])
                    pid = state.get('Pid', 0)
                    if pid:
                        paths = cgroup_paths(read_text('/proc/'+str(pid)+'/cgroup') or '',
                                             read_text('/proc/self/mountinfo') or '')
                        membership_observed = True
                except (OSError, ValueError, subprocess.SubprocessError) as exc:
                    row['inspection_error'] = str(exc)
            row['cgroup'] = cgroup_sample(paths)
            if time.monotonic()-last_board >= 1:
                row['board'] = board_sample()
                last_board = time.monotonic()
            if stopped.is_set():
                break
            stream.write(json.dumps(row, allow_nan=False)+'\n')
            stream.flush()
            stopped.wait(.25)


def container_command(docker, bundle, output, image, name, probe=False):
    command = docker.prefix+['run', '--name', name, '--pull', 'never',
        '--network', 'none', '--read-only', '--cap-drop', 'ALL',
        '--security-opt', 'no-new-privileges', '--memory', '2560m',
        '--memory-swap', '2560m', '--cpus', '4', '--pids-limit', '512',
        '--tmpfs', '/tmp:rw,size=128m', '--shm-size', '64m',
        '--user', str(os.getuid())+':'+str(os.getgid())]
    for key, value in dict(PATH='/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
                           HOME='/out/home', ROS_HOME='/out/ros', ROS_LOG_DIR='/out/ros/log',
                           LC_ALL='C', PYTHONDONTWRITEBYTECODE='1', TMPDIR='/tmp').items():
        command += ['--env', key+'='+value]
    for source, destination, readonly in [(bundle/'tools', '/tools', True),
                                          (bundle/'input', '/input', True),
                                          (output, '/out', False)]:
        command += ['--mount', 'type=bind,src='+str(source)+',dst='+destination+(',readonly' if readonly else '')]
    if probe:
        shell = SOURCE_ENV+('/lib/ld-linux-aarch64.so.1 --list '+NATIVE_NODE+
                            ' && python -c "import rospy, cv2, numpy"')
        target = ['/bin/bash', '-c', shell]
    else:
        target = ['/bin/bash', '-c', SOURCE_ENV+'exec python /tools/run_inner.py "$@"',
                  'rm27-schur-native', '--mode', 'mono', '--rate', '0.5', '--timeout', '900']
    return command+[image]+target


def cleanup_container(docker, name):
    result = dict(container=name, absence_verified=False)
    for operation, args in [('remove', ['container', 'rm', '--force', name]),
                            ('inspect', ['container', 'ls', '--all', '--filter',
                                         'name=^/'+name+'$', '--format', '{{.ID}}'])]:
        try:
            observation = docker.call(args, timeout=30)
            result[operation] = observation
            if operation == 'inspect':
                result['absence_verified'] = observation['exit_code'] == 0 and not observation['stdout'].strip()
        except (OSError, subprocess.SubprocessError) as exc:
            result[operation+'_error'] = str(exc)
    return result


def launch(docker, bundle, output, image, *, probe=False):
    name = 'rm27-schur-'+('probe-' if probe else 'replay-')+uuid.uuid4().hex
    command = container_command(docker, bundle, output, image, name, probe)
    evidence = dict(command=command, container=name, exit_code=None, timed_out=False)
    process, sampler, stopped = None, None, threading.Event()
    reaped = False
    try:
        with (output/'launch.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            if not probe:
                sampler = threading.Thread(target=sample_container,
                    args=(docker, name, output, stopped), daemon=True)
                sampler.start()
            try:
                evidence['exit_code'] = process.wait(timeout=60 if probe else PROFILE['timeout_s']+90)
                reaped = True
            except subprocess.TimeoutExpired:
                evidence['timed_out'] = True
                process.kill()
                evidence['exit_code'] = process.wait(timeout=10)
                reaped = True
    except (OSError, subprocess.SubprocessError) as exc:
        evidence['error'] = str(exc)
    except KeyboardInterrupt:
        evidence['error'] = 'interrupted'
    finally:
        stopped.set()
        if process is not None and not reaped:
            try:
                process.kill()
                evidence['exit_code'] = process.wait(timeout=10)
                reaped = True
            except (OSError, subprocess.SubprocessError) as exc:
                evidence['cli_cleanup_error'] = str(exc)
        evidence['cli_reaped'] = reaped if process is not None else 'NOT_STARTED'
        if sampler:
            sampler.join(timeout=16)
            evidence['sampler_stopped'] = not sampler.is_alive()
        evidence['cleanup'] = cleanup_container(docker, name)
    evidence['status'] = ('PASS' if evidence['exit_code'] == 0 and not evidence['timed_out']
                          and not evidence.get('error') and evidence.get('sampler_stopped', True)
                          and evidence['cli_reaped'] is True
                          and evidence['cleanup']['absence_verified'] else 'FAIL')
    write_json(output/'launch.json', evidence)
    return evidence


def pose_evidence(output):
    path = output/'raw/pose.jsonl'
    count, malformed = 0, 0
    if path.is_file():
        with path.open() as stream:
            for line in stream:
                if line.strip():
                    try:
                        row = json.loads(line)
                        if not isinstance(row, dict) or 'position' not in row:
                            raise ValueError('not a raw pose')
                        count += 1
                    except ValueError:
                        malformed += 1
    return dict(raw_pose_count=count, malformed_rows=malformed,
                status='OBSERVED' if count and not malformed else 'FAIL',
                algorithm_status='NOT_QUALIFIED')


def run(bundle, image, output, *, docker=None):
    bundle, manifest = validate_bundle(bundle)
    output_arg = Path(output)
    if output_arg.exists() or output_arg.is_symlink():
        raise FileExistsError('fresh output directory required')
    output = output_arg.resolve()
    if bundle == output or bundle in output.parents or any(c in str(output) for c in ',\n\r'):
        raise ValueError('output must be outside the immutable bundle and use a valid mount path')
    docker = docker or Docker.local()
    admission = doctor(bundle, image, output.parent, docker=docker)
    if admission['status'] != 'PASS':
        raise ValueError('board doctor rejected run: '+'; '.join(admission['errors']))
    output.mkdir(parents=False, exist_ok=False)
    summary = dict(status='FAIL', algorithm_status='NOT_QUALIFIED',
                   candidate_sha256=admission['candidate_sha256'], profile=manifest['profile'],
                   native_imu_ingestion='UNOBSERVED')
    write_json(output/'doctor.json', admission)
    write_json(output/'board-before.json', host_facts())
    try:
        probe_out = output/'dependency-probe'
        probe_out.mkdir()
        for folder in ('home', 'ros/log'):
            (probe_out/folder).mkdir(parents=True)
        summary['dependency_probe'] = launch(docker, bundle, probe_out, image, probe=True)
        if summary['dependency_probe']['status'] != 'PASS':
            raise ValueError('native loader/Python dependency probe failed')
        for folder in ('home', 'ros/log'):
            (output/folder).mkdir(parents=True)
        summary['process'] = launch(docker, bundle, output, image)
        summary['poses'] = pose_evidence(output)
        execution_path = output/'execution.json'
        summary['inner_execution'] = read_json(execution_path) if execution_path.is_file() else None
        inner = summary['inner_execution'] or {}
        inner_ok = (not inner.get('failure') and not inner.get('cleanup_errors') and
                    all(inner.get('exits', {}).get(name) == 0 for name in ('master', 'observer', 'native', 'player')))
        if summary['process']['status'] == 'PASS' and summary['poses']['status'] == 'OBSERVED' and inner_ok:
            summary['status'] = 'PROCESS_COMPLETED'
        else:
            summary['failure'] = 'process, raw pose availability or inner execution failed; see separate evidence'
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        summary['failure'] = str(exc)
    finally:
        try:
            validate_bundle(bundle)
            summary['bundle_after'] = 'UNCHANGED'
            if file_hash(bundle/'candidate.json') != admission['candidate_sha256']:
                raise ValueError('candidate.json changed during run')
        except (OSError, ValueError) as exc:
            summary.update(status='FAIL', bundle_after='CHANGED_OR_UNREADABLE', failure=str(exc))
        write_json(output/'board-after.json', host_facts())
        write_json(output/'result.json', summary)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('doctor', 'run'))
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--image', required=True, help='exact local sha256 image ID; never pulled')
    parser.add_argument('--output', type=Path, help='fresh run directory, outside the bundle')
    args = parser.parse_args(argv)
    if args.action == 'run' and args.output is None:
        parser.error('run requires --output')
    if args.action == 'doctor' and args.output is not None:
        parser.error('doctor is read-only and does not accept --output')
    try:
        result = (doctor(args.bundle, args.image) if args.action == 'doctor' else
                  run(args.bundle, args.image, args.output))
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        result = dict(status='FAIL', error=str(exc), algorithm_status='NOT_QUALIFIED')
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result['status'] in ('PASS', 'PROCESS_COMPLETED') else 1


if __name__ == '__main__':
    sys.exit(main())
