"""Native board admission/launch tests; no Docker, estimator or hardware runs."""
import json
import subprocess
from types import SimpleNamespace

import pytest

from tools.schurvins import rk3576_board as board


IMAGE = 'sha256:'+'a'*64


@pytest.fixture
def bundle(tmp_path):
    root = tmp_path/'bundle'
    for name in ('Dockerfile', 'runtime-rootfs.tar.gz', 'tools/run_inner.py',
                 'tools/ros_transport.py', 'input/input.json', 'input/sensors.zip',
                 'input/expected.jsonl', 'rk3576_board.py'):
        path = root/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('fixed payload '+name)
    inventory = {path.relative_to(root).as_posix(): board.file_hash(path)
                 for path in root.rglob('*') if path.is_file()}
    manifest = dict(schema_version=1, status='PREPARED_NOT_RUN', platform='linux/arm64',
                    runtime='native-docker', profile=dict(board.PROFILE),
                    files_sha256=inventory, runtime_archive_sha256=inventory['runtime-rootfs.tar.gz'])
    board.write_json(root/'candidate.json', manifest)
    return root


class FakeDocker:
    prefix = ['docker', '--host', 'unix:///var/run/docker.sock']

    def __init__(self, payload, architecture='aarch64', image_arch='arm64'):
        self.payload = payload
        self.architecture = architecture
        self.image_arch = image_arch
        self.calls = []

    def json(self, args):
        self.calls.append(args)
        if args[0] == 'info':
            return dict(Architecture=self.architecture, OSType='linux', DockerRootDir='/docker-test',
                        MemoryLimit=True, SwapLimit=True, CPUCfsPeriod=True,
                        CPUCfsQuota=True, PidsLimit=True)
        if args[:2] == ['image', 'inspect']:
            return [dict(Id=IMAGE, Architecture=self.image_arch, Os='linux',
                         Config=dict(Labels={board.PAYLOAD_LABEL: self.payload}))]
        return dict(Pid=0)

    def call(self, args, timeout=15):
        self.calls.append(args)
        return dict(command=self.prefix+args, exit_code=0, stdout='', stderr='')


@pytest.fixture
def native(monkeypatch, bundle):
    facts = dict(machine='aarch64', kernel='test', os_release='test',
                 device_tree_compatible='vendor,board\x00rockchip,rk3576\x00',
                 memory_bytes=dict(MemTotal=4*1024**3, MemAvailable=3500*1024**2), sample={})
    monkeypatch.setattr(board, 'host_facts', lambda *args: facts)
    monkeypatch.setattr(board.shutil, 'disk_usage', lambda path: SimpleNamespace(free=4*1024**3))
    payload = board.read_json(bundle/'candidate.json')['runtime_archive_sha256']
    return facts, FakeDocker(payload)


def test_inventory_preserves_all_inputs_and_rejects_extra_or_changed_payload(bundle):
    before = {p: p.read_bytes() for p in bundle.rglob('*') if p.is_file()}
    board.validate_bundle(bundle)
    assert all(path.read_bytes() == data for path, data in before.items())
    (bundle/'tools/run_inner.py').write_text('changed')
    with pytest.raises(ValueError, match='hash changed'):
        board.validate_bundle(bundle)
    (bundle/'tools/run_inner.py').write_bytes(before[bundle/'tools/run_inner.py'])
    (bundle/'tools/candidate.json').write_text('{}')
    with pytest.raises(ValueError, match='membership changed'):
        board.validate_bundle(bundle)


@pytest.mark.parametrize('name', ['../escape', '/absolute', 'input/../escape', './same', 'candidate.json'])
def test_escaping_or_ambiguous_inventory_paths_rejected(bundle, name):
    manifest = board.read_json(bundle/'candidate.json')
    manifest['files_sha256'][name] = 'b'*64
    board.write_json(bundle/'candidate.json', manifest)
    with pytest.raises(ValueError, match='inventory path'):
        board.validate_bundle(bundle)


def test_symlink_payload_is_rejected(bundle, tmp_path):
    target = tmp_path/'outside'
    target.write_text('outside')
    (bundle/'link').symlink_to(target)
    with pytest.raises(ValueError, match='regular files'):
        board.validate_bundle(bundle)


def test_native_doctor_is_read_only_and_separates_loader_from_metadata(bundle, native):
    _, docker = native
    before = sorted(str(path) for path in bundle.rglob('*'))
    result = board.doctor(bundle, IMAGE, docker=docker)
    assert result['status'] == 'PASS'
    assert result['native_dependency_probe'] == 'NOT_RUN'
    assert result['algorithm_status'] == 'NOT_QUALIFIED'
    assert before == sorted(str(path) for path in bundle.rglob('*'))
    assert all(args[0] in ('info', 'image') for args in docker.calls)


@pytest.mark.parametrize('host_machine,server_arch', [('x86_64', 'aarch64'), ('aarch64', 'x86_64')])
def test_x86_host_or_helper_daemon_cannot_qualify_as_native(bundle, native, host_machine, server_arch):
    facts, docker = native
    facts['machine'], docker.architecture = host_machine, server_arch
    result = board.doctor(bundle, IMAGE, docker=docker)
    assert result['status'] == 'FAIL'
    assert any('native' in error for error in result['errors'])


@pytest.mark.parametrize('key', ['MemTotal', 'MemAvailable'])
def test_ram_total_and_available_each_need_cap_plus_reserve(bundle, native, key):
    facts, docker = native
    facts['memory_bytes'][key] = 3072*1024**2-1
    result = board.doctor(bundle, IMAGE, docker=docker)
    assert result['status'] == 'FAIL'
    assert any(key in error for error in result['errors'])


def test_wrong_board_image_and_missing_disk_reserve_block_admission(bundle, native, monkeypatch):
    facts, docker = native
    facts['device_tree_compatible'] = 'rockchip,rk3588'
    docker.payload = 'wrong'
    docker.image_arch = 'amd64'
    monkeypatch.setattr(board.shutil, 'disk_usage', lambda path: SimpleNamespace(free=1024**3-1))
    result = board.doctor(bundle, IMAGE, docker=docker)
    assert result['status'] == 'FAIL'
    assert len(result['errors']) == 4


@pytest.mark.parametrize('capability', ['MemoryLimit', 'SwapLimit', 'CPUCfsPeriod', 'CPUCfsQuota', 'PidsLimit'])
@pytest.mark.parametrize('value', [False, None])
def test_missing_or_false_resource_capabilities_reject_admission(bundle, native, monkeypatch, capability, value):
    docker = native[1]
    original = docker.json
    def inspect(args):
        result = original(args)
        if args[0] == 'info':
            if value is None:
                result.pop(capability)
            else:
                result[capability] = value
        return result
    monkeypatch.setattr(docker, 'json', inspect)
    result = board.doctor(bundle, IMAGE, docker=docker)
    assert result['status'] == 'FAIL'
    assert any(capability in error for error in result['errors'])


@pytest.mark.parametrize('endpoint', ['tcp://other:2375', 'ssh://helper', 'unix://relative'])
def test_remote_docker_endpoint_is_rejected(endpoint):
    with pytest.raises(ValueError, match='local unix'):
        board.Docker(endpoint)


def test_mutable_image_tag_is_rejected_before_docker(bundle, native):
    with pytest.raises(ValueError, match='immutable'):
        board.doctor(bundle, 'schur:latest', docker=native[1])
    assert native[1].calls == []


def test_command_uses_existing_native_runtime_and_restricts_mounts(bundle, native, tmp_path):
    command = board.container_command(native[1], bundle, tmp_path/'output', IMAGE, 'owned')
    for flag in ('--read-only', '--cap-drop', '--security-opt', '--memory-swap', '--pids-limit'):
        assert flag in command
    assert command[command.index('--pull')+1] == 'never'
    assert command[command.index('--network')+1] == 'none'
    assert command[command.index('--memory')+1] == '2560m'
    mounts = [command[i+1] for i, value in enumerate(command) if value == '--mount']
    assert len(mounts) == 3
    assert all('readonly' in mount for mount in mounts[:2])
    assert not any(value in command for value in ('--device', '--privileged', 'build', 'pull', 'ssh'))
    assert not any('qemu' in value.lower() for value in command)
    assert 'source /ws/devel/setup.bash' in command[-8]
    assert command[-7:] == ['rm27-schur-native', '--mode', 'mono', '--rate', '0.5', '--timeout', '900']


def test_cgroup_v1_membership_maps_actual_mounts_and_absence_stays_unavailable(tmp_path):
    cpu, memory = tmp_path/'cpus', tmp_path/'memory'
    cpu.mkdir()
    memory.mkdir()
    member = '4:cpu,cpuacct:/docker/owned\n5:memory:/docker/owned\n'
    mounts = ('20 10 0:20 /docker/owned '+str(cpu)+' rw - cgroup cgroup rw,cpu,cpuacct\n'
              '21 10 0:21 /docker '+str(memory)+' rw - cgroup cgroup rw,memory\n')
    paths = board.cgroup_paths(member, mounts)
    assert paths == dict(cpu=str(cpu), cpuacct=str(cpu), memory=str(memory/'owned'))
    assert board.cgroup_sample(paths)['status'] == 'UNAVAILABLE'
    (cpu/'cpuacct.usage').write_text('321')
    result = board.cgroup_sample(paths)
    assert result['counters'] == {'cpuacct.usage': '321'}
    assert 'memory.usage_in_bytes' in result['unavailable']


def test_cgroup_v2_resolves_nested_container_and_samples_real_files(tmp_path):
    container = tmp_path/'docker/owned'
    container.mkdir(parents=True)
    (container/'memory.current').write_text('456')
    paths = board.cgroup_paths('0::/docker/owned\n',
                               '10 1 0:1 / '+str(tmp_path)+' rw - cgroup2 cgroup rw\n')
    assert paths == {'unified': str(container)}
    assert board.cgroup_sample(paths)['counters'] == {'memory.current': '456'}


def test_host_evidence_uses_raw_device_tree_thermal_and_memory(tmp_path, monkeypatch):
    files = {'proc/meminfo': 'MemTotal: 4000000 kB\nMemAvailable: 3500000 kB\n',
             'sys/firmware/devicetree/base/compatible': 'vendor\x00rockchip,rk3576\x00',
             'sys/class/thermal/thermal_zone0/type': 'soc-thermal',
             'sys/class/thermal/thermal_zone0/temp': '45321',
             'sys/devices/system/cpu/cpufreq/policy0/scaling_governor': 'ondemand',
             'sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq': '1800000'}
    for name, value in files.items():
        path = tmp_path/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(value)
    result = board.host_facts(tmp_path)
    assert result['memory_bytes']['MemAvailable'] == 3500000*1024
    assert 'rk3576' in result['device_tree_compatible']
    assert result['sample']['temperatures']['thermal_zone0']['temperature_millicelsius'] == '45321'
    assert result['sample']['cpu_frequency_khz']['policy0']['scaling_governor'] == 'ondemand'


class TimeoutProcess:
    def __init__(self, *args, **kwargs):
        self.waits = []
        self.killed = False

    def wait(self, timeout):
        self.waits.append(timeout)
        if len(self.waits) == 1:
            raise subprocess.TimeoutExpired('docker', timeout)
        return -9

    def kill(self):
        self.killed = True


def test_outer_timeout_reaps_cli_and_always_verifies_container_absence(bundle, native, tmp_path, monkeypatch):
    output = tmp_path/'run'
    output.mkdir()
    process = TimeoutProcess()
    monkeypatch.setattr(board.subprocess, 'Popen', lambda *args, **kwargs: process)
    monkeypatch.setattr(board, 'sample_container', lambda *args: None)
    result = board.launch(native[1], bundle, output, IMAGE)
    assert result['status'] == 'FAIL' and result['timed_out']
    assert process.waits == [990, 10] and process.killed
    assert result['cleanup']['absence_verified']
    assert any(args[:3] == ['container', 'rm', '--force'] for args in native[1].calls)
    assert (output/'launch.json').is_file()


def test_cleanup_failure_is_not_hidden_as_process_success(bundle, native, tmp_path, monkeypatch):
    output = tmp_path/'run'
    output.mkdir()
    monkeypatch.setattr(board.subprocess, 'Popen', lambda *args, **kwargs: SimpleNamespace(wait=lambda timeout: 0))
    monkeypatch.setattr(board, 'sample_container', lambda *args: None)
    monkeypatch.setattr(native[1], 'call', lambda args, timeout=15:
                        dict(command=args, exit_code=0, stdout='still-owned\n', stderr=''))
    result = board.launch(native[1], bundle, output, IMAGE)
    assert result['exit_code'] == 0
    assert result['status'] == 'FAIL' and not result['cleanup']['absence_verified']


def test_interruption_reaps_owned_client_removes_container_and_retains_evidence(bundle, native, tmp_path, monkeypatch):
    output = tmp_path/'run'
    output.mkdir()
    class InterruptedProcess(TimeoutProcess):
        def wait(self, timeout):
            self.waits.append(timeout)
            if len(self.waits) == 1:
                raise KeyboardInterrupt()
            return -9
    process = InterruptedProcess()
    monkeypatch.setattr(board.subprocess, 'Popen', lambda *args, **kwargs: process)
    result = board.launch(native[1], bundle, output, IMAGE, probe=True)
    assert result['status'] == 'FAIL' and result['error'] == 'interrupted'
    assert process.waits == [60, 10] and process.killed
    assert result['cli_reaped'] and result['cleanup']['absence_verified']
    assert board.read_json(output/'launch.json')['error'] == 'interrupted'


def test_native_dependency_probe_only_lists_loader_and_imports_python(bundle, native, tmp_path):
    command = board.container_command(native[1], bundle, tmp_path/'output', IMAGE, 'owned', probe=True)
    shell = command[-1]
    assert '--list '+board.NATIVE_NODE in shell
    assert 'import rospy, cv2, numpy' in shell
    assert 'run_inner.py' not in shell and 'rosrun' not in shell
    assert 'HOME=/out/home' in command and 'ROS_LOG_DIR=/out/ros/log' in command


def test_local_socket_is_explicit_for_subsequent_commands(monkeypatch):
    calls = []
    monkeypatch.setenv('DOCKER_HOST', 'unix:///run/user/1000/docker.sock')
    monkeypatch.setattr(board, 'command_result', lambda command, timeout=15:
                        calls.append(command) or dict(exit_code=0, stdout='{}', stderr=''))
    docker = board.Docker.local()
    docker.call(['info'])
    assert calls == [['docker', '--host', 'unix:///run/user/1000/docker.sock', 'info']]


def test_cleanup_inspection_errors_never_establish_absence(native, monkeypatch):
    def fail(args, timeout=15):
        raise subprocess.TimeoutExpired('docker', timeout)
    monkeypatch.setattr(native[1], 'call', fail)
    result = board.cleanup_container(native[1], 'owned')
    assert not result['absence_verified']
    assert 'remove_error' in result and 'inspect_error' in result


def fake_launch(docker, bundle, output, image, probe=False, pose=True, failure=False):
    if not probe:
        (output/'raw').mkdir()
        (output/'raw/pose.jsonl').write_text(json.dumps(dict(position=[0, 0, 0]))+'\n' if pose else '')
        board.write_json(output/'execution.json', dict(exits={name: 0 for name in
                         ('master', 'observer', 'native', 'player')}, failure='native failure' if failure else None,
                         cleanup_errors={}))
    return dict(status='PASS', exit_code=0, cleanup=dict(absence_verified=True))


@pytest.mark.parametrize('pose,inner_failure,expected', [(False, False, 'FAIL'), (True, True, 'FAIL'),
                                                       (True, False, 'PROCESS_COMPLETED')])
def test_process_and_empty_pose_or_inner_failure_remain_separate_from_algorithm_acceptance(
        bundle, native, tmp_path, monkeypatch, pose, inner_failure, expected):
    before = {p: p.read_bytes() for p in bundle.rglob('*') if p.is_file()}
    monkeypatch.setattr(board, 'launch', lambda docker, bundle, output, image, probe=False:
                        fake_launch(docker, bundle, output, image, probe, pose, inner_failure))
    result = board.run(bundle, IMAGE, tmp_path/'run', docker=native[1])
    assert result['status'] == expected
    assert result['algorithm_status'] == 'NOT_QUALIFIED'
    assert result['poses']['raw_pose_count'] == int(pose)
    assert result['bundle_after'] == 'UNCHANGED'
    assert all(path.read_bytes() == data for path, data in before.items())


def test_probe_failure_preserves_evidence_and_does_not_launch_estimator(bundle, native, tmp_path, monkeypatch):
    calls = []
    def failed_probe(*args, **kwargs):
        calls.append(kwargs)
        return dict(status='FAIL', exit_code=127, cleanup=dict(absence_verified=True))
    monkeypatch.setattr(board, 'launch', failed_probe)
    result = board.run(bundle, IMAGE, tmp_path/'run', docker=native[1])
    assert result['status'] == 'FAIL' and len(calls) == 1 and calls[0]['probe']
    assert result['dependency_probe']['exit_code'] == 127
    assert (tmp_path/'run/result.json').is_file()
    assert not (tmp_path/'run/raw').exists()


def test_existing_output_and_bundle_nested_output_are_rejected_without_mutation(bundle, native, tmp_path):
    output = tmp_path/'existing'
    output.mkdir()
    marker = output/'keep.txt'
    marker.write_text('original')
    with pytest.raises(FileExistsError, match='fresh'):
        board.run(bundle, IMAGE, output, docker=native[1])
    assert marker.read_text() == 'original'
    with pytest.raises(ValueError, match='outside'):
        board.run(bundle, IMAGE, bundle/'run', docker=native[1])
    assert native[1].calls == []


def test_rejected_doctor_never_creates_output_or_launches(bundle, native, tmp_path, monkeypatch):
    native[0]['machine'] = 'x86_64'
    monkeypatch.setattr(board, 'launch', lambda *args, **kwargs: pytest.fail('must not launch'))
    with pytest.raises(ValueError, match='doctor rejected'):
        board.run(bundle, IMAGE, tmp_path/'rejected', docker=native[1])
    assert not (tmp_path/'rejected').exists()


def test_candidate_mutation_is_retained_and_fails_run(bundle, native, tmp_path, monkeypatch):
    def mutate(*args, **kwargs):
        result = fake_launch(*args, **kwargs)
        if not kwargs.get('probe'):
            (bundle/'input/sensors.zip').write_text('changed during replay')
        return result
    monkeypatch.setattr(board, 'launch', mutate)
    result = board.run(bundle, IMAGE, tmp_path/'run', docker=native[1])
    assert result['status'] == 'FAIL' and result['bundle_after'] == 'CHANGED_OR_UNREADABLE'
    assert (bundle/'input/sensors.zip').read_text() == 'changed during replay'
    assert 'hash changed' in result['failure']
