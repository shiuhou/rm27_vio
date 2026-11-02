"""Prepare/build an isolated Bionic/Melodic ARM64 reference, without host installs.

PRoot and explicit QEMU are local tools; no binfmt registration is made. This
reference is not the unknown RK3576 BSP/ABI and emulation is not performance data.
"""
import argparse
import gzip
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import signal
import struct
import subprocess
import tarfile
import tempfile
import time
import uuid

from rm27.perception.vision.localization.experiments.common import cleanup_container, file_hash, write_json
from tools.schurvins.build import snapshot, symlinks
from tools.schurvins.prepare import PIN, prepare

BASE_IMAGE = 'ros@sha256:85d442cdcc1b530d401b56f5e80aba6582fdc0764e36e72cc5dab6dcf6c5642a'
HELPER_IMAGE_ID = 'sha256:33ceb71981b602c1a7443a53469e4dba065f7503eab3078a2d7a57a2ab987517'
MIN_FREE = 20 * 1024**3
ROOTFS_EXCLUDE = {'proc', 'sys', 'dev', 'tmp', 'ws', 'probe', 'host-rootfs',
                  'out', 'tools', 'input', 'var/cache/apt', 'var/lib/apt/lists'}
RUNTIME_HELPERS = ('proot/usr/bin/proot',
                   'proot/usr/lib/x86_64-linux-gnu/libtalloc.so.2.4.2',
                   'qemu-bionic/usr/bin/qemu-aarch64-static',
                   'environment.json', 'rootfs-source.json')
# QEMU 8.2 + this PRoot fails faccessat: readable files appear absent. The
# Bionic QEMU below actually passes apt signature/key readability and child exec.
HOST_TOOLS = {
    'proot': ('pool/universe/p/proot/proot_5.1.0-1.3_amd64.deb',
              '01a5d27c4ac16e184bdb356c9e69fa7d494325ac653c4cd64fae4c3fc63cdbbb'),
    'libtalloc': ('pool/main/t/talloc/libtalloc2_2.4.2-1build2_amd64.deb',
                 'c9728388084bdccdbcfb3e97358535431855f4ce57992c2a775534dc1bd4935f'),
    'qemu': ('pool/universe/q/qemu/qemu-user-static_2.11+dfsg-1ubuntu7.42_amd64.deb',
             '6f9cb1236cfa22e34f25fb165b710629f6bd59172a1248da7138b74ede4e2a63'),
}

def external(path):
    path = Path(path).resolve()
    repo = Path(__file__).resolve().parents[2]
    if path == repo or repo in path.parents:
        raise ValueError('ARM64 workspace/output must be outside repository')
    return path


class DiskSpaceError(OSError):
    """A measured operation cannot retain the required free-space reserve."""


def disk_guard(path, expected_bytes=0):
    path = Path(path)
    while not path.exists():
        path = path.parent
    free = shutil.disk_usage(path).free
    if expected_bytes < 0 or free - expected_bytes < MIN_FREE:
        raise DiskSpaceError('ARM64 operation would leave less than 20 GiB free')
    return free


def elf_header(path):
    """Read the actual ELF header; filenames and successful builds prove no ISA."""
    with Path(path).open('rb') as stream:
        data = stream.read(64)
    if len(data) < 64 or data[:4] != b'\x7fELF':
        raise ValueError('not an ELF file: ' + str(path))
    if data[4] != 2 or data[5] != 1 or data[6] != 1:
        raise ValueError('expected ELF64 little-endian current format: ' + str(path))
    machine = struct.unpack_from('<H', data, 18)[0]
    if machine != 183:
        raise ValueError('not AArch64 ELF (machine {}): {}'.format(machine, path))
    return dict(elf_class=64, byte_order='little', machine='AArch64', machine_id=machine)


def guest_file(rootfs, name, prefix='/'):
    """Resolve target absolute symlinks within the target, never host /lib."""
    rootfs = Path(rootfs).resolve()
    if prefix != '/':
        if not name.startswith(prefix+'/'):
            raise ValueError('target path is outside binding')
        name = name[len(prefix):]
    todo = list(PurePosixPath(name).parts)
    if not todo or todo.pop(0) != '/':
        raise ValueError('target path must be absolute')
    parts, links = [], 0
    while todo:
        part = todo.pop(0)
        if part in ('', '.'):
            continue
        if part == '..':
            if not parts:
                raise ValueError('target path escapes rootfs')
            parts.pop()
            continue
        candidate = rootfs.joinpath(*parts, part)
        if candidate.is_symlink():
            links += 1
            if links > 40:
                raise ValueError('target symlink cycle')
            target = PurePosixPath(os.readlink(candidate))
            if target.is_absolute():
                if prefix != '/':
                    if not str(target).startswith(prefix+'/'):
                        raise ValueError('absolute target symlink leaves binding')
                    target = PurePosixPath(str(target)[len(prefix):])
                parts = []
                todo = list(target.parts[1:]) + todo
            else:
                todo = list(target.parts) + todo
        else:
            parts.append(part)
    return rootfs.joinpath(*parts)


def command(root, argv, binds=(), memory_gib=None, network=False):
    """Explicit emulator + PRoot intercept child exec; no host-wide binfmt."""
    root = external(root)
    cmd = ['/lib64/ld-linux-x86-64.so.2', '--library-path',
           str(root/'proot/usr/lib/x86_64-linux-gnu'),
           str(root/'proot/usr/bin/proot'), '-0', '-r', str(root/'rootfs'),
           '-q', str(root/'qemu-bionic/usr/bin/qemu-aarch64-static'),
           '-b', '/proc', '-b', '/dev', '-b', '/sys', '-b', '/etc/resolv.conf']
    for host, guest in binds:
        host = external(host)
        if ':' in str(host) or ':' in guest or not guest.startswith('/'):
            raise ValueError('invalid target binding')
        cmd += ['-b', str(host)+':'+guest]
    # A sourced host ROS2/CMake/Python environment must not enter ROS1 target.
    target_env = ['HOME=/root', 'USER=root', 'LOGNAME=root', 'SHELL=/bin/bash',
                  'PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
                  'LC_ALL=C', 'DEBIAN_FRONTEND=noninteractive']
    if network:
        for name in ['http_proxy', 'https_proxy', 'HTTP_PROXY', 'HTTPS_PROXY', 'NO_PROXY', 'no_proxy']:
            if name in os.environ:
                target_env.append(name+'='+os.environ[name])
    cmd += ['-w', '/', '/usr/bin/env', '-i', *target_env, *argv]
    if memory_gib is not None:
        if memory_gib not in (4, 12):
            raise ValueError('memory profile must be 4 or 12 GiB')
        cmd = ['systemd-run', '--user', '--scope', '--quiet', '-p',
               'MemoryMax={}G'.format(memory_gib), '-p', 'CPUQuota=400%', *cmd]
    return cmd


def rootfs_tree(rootfs):
    """Hash target regular files without ever following absolute target links."""
    rootfs = Path(rootfs)
    files, links = {}, {}
    def unreadable(error):
        raise error
    for directory, directories, names in os.walk(rootfs, followlinks=False, onerror=unreadable):
        here = Path(directory)
        for name in list(directories) + names:
            path = here/name
            relative = str(path.relative_to(rootfs))
            if any(relative == p or relative.startswith(p+'/') for p in ROOTFS_EXCLUDE):
                if name in directories:
                    directories.remove(name)
                continue
            if path.is_symlink():
                links[relative] = os.readlink(path)
            elif path.is_file():
                files[relative] = file_hash(path)
    return dict(files_sha256=files, symlinks=links)


def checked_runtime(installation):
    """Verify the immutable emulation environment; dataset truth is not needed."""
    if (installation.get('architecture') != 'arm64'
            or installation.get('platform') != 'linux/arm64'
            or installation.get('runtime') != 'proot-qemu'
            or installation.get('target') != 'aarch64-bionic-melodic-reference'
            or installation.get('image_id') != HELPER_IMAGE_ID):
        raise ValueError('unsupported ARM64 reference runtime')
    root = external(installation['root'])
    expected = installation.get('runtime_files_sha256')
    if not isinstance(expected, dict) or set(expected) != set(RUNTIME_HELPERS):
        raise ValueError('incomplete ARM64 runtime helper manifest')
    for name, digest in expected.items():
        path = root/name
        if not path.is_file() or path.is_symlink() or file_hash(path) != digest:
            raise ValueError('ARM64 runtime helper changed: '+name)
    if (rootfs_tree(root/'rootfs') != installation.get('rootfs')
            or symlinks(root/'proot') != installation.get('helper_symlinks')):
        raise ValueError('ARM64 runtime tree changed')
    environment = json.loads((root/'environment.json').read_text())
    if environment != installation.get('environment'):
        raise ValueError('ARM64 environment provenance changed')
    lock = Path(environment['package_lock'])
    if not lock.is_file() or file_hash(lock) != environment['packages_sha256']:
        raise ValueError('ARM64 package lock changed')
    return root


def replay_command(installation, transport, inputs, output, mode, rate, timeout_s,
                   start_offset_s=0, *, container_name, memory_gib=12):
    """Build argv for the existing ROS replay scripts; never start an estimator.

    The caller owns exclusive output creation, runtime-tmp creation, installation
    preflight, timeout handling and named-container cleanup. Sensor validation and
    scoring remain in the existing runner. This command permits no network or
    host device access beyond the container's ordinary virtual /dev.
    """
    if (installation.get('runtime') != 'proot-qemu'
            or installation.get('architecture') != 'arm64'
            or installation.get('image_id') != HELPER_IMAGE_ID):
        raise ValueError('unsupported ARM64 replay installation')
    if mode not in ('mono', 'stereo') or memory_gib not in (4, 12):
        raise ValueError('invalid ARM64 replay mode/resource profile')
    if (type(rate) not in (int, float) or not math.isfinite(rate) or rate <= 0
            or type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or timeout_s <= 0
            or type(start_offset_s) not in (int, float)
            or not math.isfinite(start_offset_s) or start_offset_s < 0):
        raise ValueError('invalid ARM64 replay timing')
    if not isinstance(container_name, str) or not re.fullmatch(r'rm27-schur-[a-zA-Z0-9_-]+', container_name):
        raise ValueError('explicit owned container name required')
    root = external(installation['root'])
    workspace, transport, inputs, output = map(external, (
        installation['workspace'], transport, inputs, output))
    temporary = output/'runtime-tmp'
    if (not output.is_dir() or not temporary.is_dir() or temporary.is_symlink()
            or any(temporary.iterdir())):
        raise ValueError('fresh output/runtime-tmp directory required')
    if (workspace == inputs or workspace == output or inputs == output
            or workspace in output.parents or inputs in output.parents
            or transport == temporary or transport in temporary.parents):
        raise ValueError('independent replay inputs/workspace/output required')
    if set(p.name for p in transport.iterdir()) != {'run_inner.py', 'ros_transport.py'}:
        raise ValueError('exact frozen ROS transport scripts required')
    for name in ('run_inner.py', 'ros_transport.py'):
        if not (transport/name).is_file() or (transport/name).is_symlink():
            raise ValueError('frozen transport must contain regular script files')
    paths = [root, workspace, transport, inputs, output, temporary]
    if any(':' in str(p) or '\n' in str(p) or '\r' in str(p) for p in paths):
        raise ValueError('invalid ARM64 mount path')
    mounts = [(root/'rootfs', 'ro'), (root/'proot', 'ro'),
              (root/'qemu-bionic', 'ro'), (workspace, 'ro'),
              (transport, 'ro'), (inputs, 'ro'), (output, 'rw')]
    cmd = ['docker', 'run', '--rm', '--name', container_name,
           '--network', 'none', '--cpus', '4', '--memory', str(memory_gib)+'g',
           '--cap-add', 'SYS_PTRACE', '--security-opt', 'seccomp=unconfined',
           '--user', str(os.getuid())+':'+str(os.getgid()),
           '--tmpfs', '/tmp:rw,exec,nosuid,size=512m']
    for source, access in mounts:
        cmd += ['-v', str(source)+':'+str(source)+':'+access]
    shell = ('source /opt/ros/melodic/setup.bash && source /ws/devel/setup.bash && '
             'export ROS_HOME=/out/ros ROS_LOG_DIR=/out/ros/log '
             'PYTHONDONTWRITEBYTECODE=1 TMPDIR=/tmp && '
             'exec python /tools/run_inner.py "$@"')
    target = ['/bin/bash', '-c', shell, 'rm27-schur-inner', '--mode', mode,
              '--rate', str(rate), '--timeout', str(timeout_s),
              '--start-offset', str(start_offset_s)]
    binds = [(workspace, '/ws'), (transport, '/tools'), (inputs, '/input'),
             (output, '/out'), (temporary, '/tmp')]
    return cmd+[HELPER_IMAGE_ID]+command(root, target, binds)


def run_logged(args, log, timeout_s, env=None, *, disk_path=None, disk_margin=0):
    """Reap the whole local process group when a command stops or times out."""
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                               env=env, start_new_session=True)
    try:
        if disk_path is None:
            return process.wait(timeout=timeout_s)
        deadline = time.monotonic()+timeout_s
        while True:
            disk_guard(disk_path, disk_margin)
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise subprocess.TimeoutExpired(args, timeout_s)
            try:
                return process.wait(timeout=min(1, remaining))
            except subprocess.TimeoutExpired:
                if time.monotonic() >= deadline:
                    raise subprocess.TimeoutExpired(args, timeout_s)
    except (subprocess.TimeoutExpired, KeyboardInterrupt, OSError):
        def stop(signum):
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                pass  # Group exited between the timeout/interrupt and signal.
        stop(signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            stop(signal.SIGKILL)
            process.wait()
        raise


def isolated_build_command(root, workspace, output, shell, memory_gib, container):
    """Compile in a copied workspace; the shared target userspace stays read-only."""
    root, workspace, output = map(external, (root, workspace, output))
    if memory_gib not in (4, 12) or not re.fullmatch(r'rm27-schur-[A-Za-z0-9_-]+', container):
        raise ValueError('explicit build resource profile and owned container required')
    for writable in (workspace, output):
        for immutable in (root/'rootfs', root/'proot', root/'qemu-bionic'):
            if writable == immutable or writable in immutable.parents or immutable in writable.parents:
                raise ValueError('writable build paths overlap the immutable target runtime')
    if workspace == output or workspace in output.parents or output in workspace.parents:
        raise ValueError('independent build workspace/output required')
    for path in (root, workspace, output):
        if ':' in str(path) or '\n' in str(path) or '\r' in str(path):
            raise ValueError('invalid ARM64 build mount path')
    cmd = ['docker', 'run', '--rm', '--name', container, '--network', 'none',
           '--cpus', '4', '--memory', str(memory_gib)+'g', '--cap-add', 'SYS_PTRACE',
           '--security-opt', 'seccomp=unconfined', '--user', str(os.getuid())+':'+str(os.getgid())]
    for source, access in [(root/'rootfs', 'ro'), (root/'proot', 'ro'),
                           (root/'qemu-bionic', 'ro'), (workspace, 'rw'), (output, 'rw')]:
        cmd += ['-v', str(source)+':'+str(source)+':'+access]
    return cmd+[HELPER_IMAGE_ID]+command(root, ['/bin/bash', '-c', shell],
        [(workspace, '/ws'), (output/'build-tmp', '/tmp'), (output/'build-home', '/root')])


def incremental_build(root, workspace, output, base_installation, extra_patches, memory_gib):
    """Rebuild a hash-verified reference copy without mutating its evidence."""
    root, workspace, output = map(external, (root, workspace, output))
    if workspace.exists() or output.exists():
        raise FileExistsError(workspace if workspace.exists() else output)
    base_path = Path(base_installation).resolve()
    base = json.loads(base_path.read_text())
    if checked_runtime(base) != root:
        raise ValueError('base installation belongs to another target userspace')
    origin = external(base['workspace'])
    if any(a == b or a in b.parents or b in a.parents
           for a, b in ((workspace, origin), (output, origin), (workspace, output),
                        (workspace, root/'rootfs'), (output, root/'rootfs'))):
        raise ValueError('independent ARM64 derivative workspace/output required')
    def unchanged():
        if (snapshot(origin/'src') != base['source_files_sha256']
                or snapshot(origin/'devel') != base['devel_files_sha256']
                or symlinks(origin/'src') != base['source_symlinks']
                or symlinks(origin/'devel') != base['devel_symlinks']
                or file_hash(origin/'source-lock.json') != base['source_lock_sha256']):
            raise ValueError('ARM64 base installation files changed')
    unchanged()
    if not (origin/'build').is_dir() or not any((origin/'build').rglob('*.o')):
        raise ValueError('measured incremental build requires the complete reference build cache')
    patches = [Path(p).resolve() for p in extra_patches]
    names = [p.name for p in patches]
    if not patches or len(set(names)) != len(names) or any(name in base['patches'] for name in names):
        raise ValueError('explicit unique derivative patches required')
    digests = {p.name:file_hash(p) for p in patches}
    def allocated(path):
        return sum(p.stat().st_blocks*512 for p in path.rglob('*')
                   if p.is_file() and not p.is_symlink())
    copy_bytes = allocated(origin)
    compile_bytes = 2*(allocated(origin/'build')+allocated(origin/'devel'))
    largest = max(p.stat().st_size for p in origin.rglob('*') if p.is_file() and not p.is_symlink())
    observations = []
    def guard(stage, expected=0):
        target = workspace
        while not target.exists():
            target = target.parent
        free = shutil.disk_usage(target).free
        observations.append(dict(stage=stage, free_bytes=free, expected_additional_bytes=expected))
        disk_guard(workspace, expected)
    guard('before-copy', copy_bytes+compile_bytes+largest)
    shutil.copytree(origin, workspace, symlinks=True, copy_function=shutil.copy2)
    guard('after-copy', compile_bytes+largest)
    if (snapshot(workspace/'src') != base['source_files_sha256']
            or snapshot(workspace/'devel') != base['devel_files_sha256']
            or symlinks(workspace/'src') != base['source_symlinks']
            or symlinks(workspace/'devel') != base['devel_symlinks']):
        raise ValueError('independent ARM64 workspace copy differs from base')
    for patch in patches:
        for dry in (True, False):
            args = ['patch', '-p1', '-i', str(patch)]
            if dry:
                args.insert(1, '--dry-run')
            subprocess.run(args, cwd=workspace/'src/SchurVINS', check=True)
        (workspace/(patch.name+'.sha256')).write_text(digests[patch.name]+'\n')
        guard('patched-'+patch.name, compile_bytes+largest)
    if any(file_hash(p) != digests[p.name] for p in patches):
        raise ValueError('derivative patch changed while preparing workspace')
    output.mkdir(parents=True)
    for name in ('patches', 'build-tmp', 'build-home'):
        (output/name).mkdir()
    for patch in patches:
        shutil.copyfile(patch, output/'patches'/patch.name)
    shell = ('source /opt/ros/melodic/setup.bash && cd /ws && '
             'export PYTHONDONTWRITEBYTECODE=1 TMPDIR=/tmp && '
             'catkin config --extend /opt/ros/melodic --cmake-args '
             '-DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON '
             '-DUSE_SYSTEM_EIGEN=ON -DUSE_SYSTEM_GLOG=ON && '
             'catkin build svo_ros --no-status -j2 -p1')
    container = 'rm27-schur-arm64-build-'+uuid.uuid4().hex
    args = isolated_build_command(root, workspace, output, shell, memory_gib, container)
    write_json(output/'command.json', args)
    write_json(output/'space.json', dict(copy_estimate_bytes=copy_bytes,
        compile_peak_growth_estimate_bytes=compile_bytes, polling_margin_bytes=largest,
        observations=observations, scope='incremental allocation estimate; other host jobs may run'))
    code = None
    failure = None
    storage_errors = []
    integrity_errors = []
    try:
        guard('before-compile', compile_bytes+largest)
        with (output/'build.log').open('x') as log:
            code = run_logged(args, log, 8*3600, disk_path=workspace, disk_margin=largest)
    except (subprocess.TimeoutExpired, KeyboardInterrupt, OSError) as exc:
        code = 130 if isinstance(exc, KeyboardInterrupt) else 124 if isinstance(exc, subprocess.TimeoutExpired) else 125
        failure = str(exc)
        if isinstance(exc, DiskSpaceError):
            storage_errors.append(str(exc))
    finally:
        with (output/'cleanup.log').open('x') as log:
            cleanup = cleanup_container(container, log)
    absent = cleanup['absence_verified']
    try:
        guard('after-compile')
    except OSError as exc:
        storage_errors.append(str(exc))
        if not code:
            code, failure = 125, str(exc)
    try:
        checked_runtime(base)
        unchanged()
    except (OSError, ValueError) as exc:
        integrity_errors.append(str(exc))
        if not code:
            code, failure = 126, str(exc)
    write_json(output/'space.json', dict(copy_estimate_bytes=copy_bytes,
        compile_peak_growth_estimate_bytes=compile_bytes, polling_margin_bytes=largest,
        workspace_allocated_bytes=allocated(workspace), observations=observations,
        scope='concurrent host jobs permitted; no exclusive performance claim'))
    write_json(output/'result.json', dict(status='BLOCKED' if storage_errors else 'FAIL' if code or not absent else 'NOT_RUN',
        exit_code=code, failure=failure, container_absence_verified=absent,
        cleanup=cleanup, storage_errors=storage_errors, integrity_errors=integrity_errors,
        memory_gib=memory_gib, scope='RM27 derivative build only; target ELF verification follows'))
    if code or not absent:
        return code or 1
    try:
        guard('before-ELF-verification')
        elf = verify(root, workspace, output/'elf', isolated_output=output)
        checked_runtime(base)
        unchanged()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        write_json(output/'result.json', dict(status='BLOCKED' if isinstance(exc, DiskSpaceError) else 'FAIL',
            exit_code=125 if isinstance(exc, DiskSpaceError) else 1, failure=str(exc),
            cleanup=cleanup, container_absence_verified=absent, memory_gib=memory_gib,
            scope='post-build verification did not complete; no installation sealed'))
        return 125 if isinstance(exc, DiskSpaceError) else 1
    manifest = installation_manifest(root, workspace, output, base['environment'], elf)
    manifest.update(variant='RM27 derivative', base_installation=str(base_path), base_installation_sha256=file_hash(base_path),
        patches={**base['patches'], **digests},
        patch_order=['platform.patch', 'aarch64-fast.patch', 'aarch64-fast-neon.patch', 'aarch64-neon.patch', *names],
        scope='ARM64 RM27 derivative build; emulated replay and RK3576 BSP/ABI/performance NOT_RUN')
    write_json(output/'installation.json', manifest)
    write_json(output/'result.json', dict(status='PASS', exit_code=0,
        evidence_class='ARM64 BUILD VERIFIED', container_absence_verified=absent,
        cleanup=cleanup,
        memory_gib=memory_gib, base_reference_unchanged=True,
        scope='RM27 derivative build and target ELF closure; no replay or board qualification'))
    return 0


def packages(root):
    result = subprocess.check_output(command(root, ['dpkg-query', '-W']), text=True)
    return result


def provision(root, output, package_lock=None):
    root, output = external(root), external(output)
    if output.exists():
        raise FileExistsError(output)
    disk_guard(root, 12*1024**3)
    root.mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True)
    cache = root/'downloads'
    cache.mkdir(exist_ok=True)
    for name, (relative, sha) in HOST_TOOLS.items():
        archive = cache/Path(relative).name
        if not archive.exists():
            subprocess.run(['curl', '-fL', '--max-time', '180', '-o', str(archive),
                            'http://cn.archive.ubuntu.com/ubuntu/'+relative], check=True)
        if file_hash(archive) != sha:
            raise ValueError('host tool checksum mismatch: '+name)
        destination = root/('qemu-bionic' if name == 'qemu' else 'proot')
        subprocess.run(['dpkg-deb', '-x', str(archive), str(destination)], check=True)
    marker = root/'rootfs-source.json'
    if not (root/'rootfs').exists():
        subprocess.run(['docker', 'pull', '--platform', 'linux/arm64', BASE_IMAGE], check=True)
        cid = subprocess.check_output(['docker', 'create', '--platform', 'linux/arm64', BASE_IMAGE], text=True).strip()
        (root/'rootfs').mkdir()
        try:
            exporter = subprocess.Popen(['docker', 'export', cid], stdout=subprocess.PIPE)
            extracted = subprocess.run(['tar', '--no-same-owner', '-x', '-C', str(root/'rootfs')], stdin=exporter.stdout)
            exporter.stdout.close()
            if exporter.wait() or extracted.returncode:
                raise RuntimeError('rootfs export failed; preserve partial evidence, use fresh root')
        finally:
            subprocess.run(['docker', 'rm', cid], check=True)
        write_json(marker, dict(base_image=BASE_IMAGE))
    if not marker.is_file() or json.loads(marker.read_text()).get('base_image') != BASE_IMAGE:
        raise ValueError('unowned/different ARM64 rootfs')
    elf_header(guest_file(root/'rootfs', '/bin/bash'))
    architecture = subprocess.check_output(command(root, ['dpkg', '--print-architecture']), text=True).strip()
    if architecture != 'arm64':
        raise ValueError('target dpkg architecture is not arm64')
    # Signature verification stays enabled. Fake root cannot drop into _apt.
    apt = ['apt-get', '-o', 'APT::Sandbox::User=root', '-o', 'Acquire::Retries=1',
           '-o', 'Acquire::http::Timeout=30']
    package_lock = Path(package_lock) if package_lock else Path(__file__).with_name('packages.arm64.lock')
    lines = package_lock.read_text().splitlines()
    requested = [line.replace('\t', '=') for line in lines if line]
    if not requested or any(not re.fullmatch(r'[a-z0-9][a-z0-9+.:~-]*=[A-Za-z0-9+.:~_-]+', x) for x in requested):
        raise ValueError('invalid ARM64 package lock')
    with (output/'provision.log').open('x') as log:
        for args in [apt+['update'], apt+['install', '-y', '--no-install-recommends']+requested]:
            code = run_logged(command(root, args, network=True), log, 3600,
                              env={**os.environ, 'DEBIAN_FRONTEND':'noninteractive'})
            if code:
                write_json(output/'result.json', dict(status='BLOCKED', exit_code=code, stage='target-apt', scope='ARM64 reference provisioning'))
                return code
    actual = packages(root)
    (output/'packages.arm64.lock').write_text(actual)
    if actual != package_lock.read_text():
        raise ValueError('ARM64 installed package closure drift')
    record = dict(schema_version=1, base_image=BASE_IMAGE, architecture='arm64',
                  packages_sha256=file_hash(output/'packages.arm64.lock'),
                  package_lock=str(output/'packages.arm64.lock'),
                  host_tools={k:dict(archive=v[0],sha256=v[1]) for k,v in HOST_TOOLS.items()},
                  cached_package_debs_sha256={p.name:file_hash(p) for p in sorted((root/'rootfs/var/cache/apt/archives').glob('*.deb'))},
                  scope='Bionic/Melodic reference; RK3576 BSP/ABI/performance unverified')
    write_json(root/'environment.json', record)
    write_json(output/'environment.json', record)
    return 0


def apply_patch(workspace, package, name):
    patch = Path(__file__).with_name(name)
    stamp = workspace/(name+'.sha256')
    if stamp.exists():
        if stamp.read_text().strip() != file_hash(patch):
            raise ValueError('different ARM64 source patch: '+name)
    else:
        for dry in (True, False):
            args = ['patch', '-p1', '-i', str(patch)]
            if dry:
                args.insert(1, '--dry-run')
            subprocess.run(args, cwd=workspace/'src'/package, check=True)
        stamp.write_text(file_hash(patch)+'\n')


def verify(root, workspace, output, *, isolated_output=None):
    """Ask target loader for the complete closure, check every actual ELF."""
    root, workspace, output = external(root), external(workspace), external(output)
    if output.exists():
        raise FileExistsError(output)
    binary_name = '/ws/devel/.private/svo_ros/lib/svo_ros/svo_node'
    binary = workspace/binary_name.removeprefix('/ws/')
    header = elf_header(binary)
    program = subprocess.check_output(['readelf', '-lW', str(binary)], text=True)
    match = re.search(r'Requesting program interpreter: ([^\]]+)', program)
    if not match:
        raise ValueError('target executable has no dynamic interpreter')
    loader = match.group(1)
    elf_header(guest_file(root/'rootfs', loader))
    shell = 'source /opt/ros/melodic/setup.bash && source /ws/devel/setup.bash && exec '+loader+' --list '+binary_name
    args = ['/bin/bash', '-c', shell]
    if isolated_output is None:
        result = subprocess.run(command(root, args, [(workspace, '/ws')]), capture_output=True, text=True)
    else:
        container = 'rm27-schur-arm64-verify-'+uuid.uuid4().hex
        try:
            result = subprocess.run(isolated_build_command(root, workspace, isolated_output, shell, 4, container),
                                    capture_output=True, text=True, timeout=120)
        finally:
            with (Path(isolated_output)/(output.name+'-cleanup.log')).open('x') as log:
                cleanup = cleanup_container(container, log)
        if not cleanup['absence_verified']:
            raise RuntimeError('owned ARM64 verification container absence was not verified')
    if result.returncode or 'not found' in result.stdout+result.stderr:
        raise ValueError('target dependency resolution failed: '+result.stdout+result.stderr)
    libraries = {}
    for line in result.stdout.splitlines():
        match = re.search(r'(?:=>\s+)?(/[^\s]+)\s+\(', line)
        if match:
            guest = match.group(1)
            host = guest_file(workspace, guest, '/ws') if guest.startswith('/ws/') else guest_file(root/'rootfs', guest)
            elf_header(host)
            libraries[guest] = file_hash(host)
    if not libraries:
        raise ValueError('target loader reported no dependencies')
    output.mkdir(parents=True)
    (output/'loader.txt').write_text(result.stdout+result.stderr)
    report = dict(status='PASS', evidence_class='ARM64 BUILD VERIFIED', binary_sha256=file_hash(binary),
                  elf=header, loader=loader, libraries_sha256=libraries,
                  scope='ELF/target-loader closure only; no estimator replay, RK3576 or timing qualification')
    if isolated_output is not None:
        report['cleanup'] = cleanup
    write_json(output/'elf.json', report)
    return report


def neon_probe(root, workspace, output):
    root, workspace, output = external(root), external(workspace), external(output)
    if output.exists():
        raise FileExistsError(output)
    disk_guard(output, 1024**3)
    source = workspace/'src/fast_neon'
    if not source.is_dir():
        raise ValueError('prepare pinned sources before NEON probe')
    apply_patch(workspace, 'fast_neon', 'aarch64-fast-neon.patch')
    apply_patch(workspace, 'SchurVINS', 'aarch64-neon.patch')
    output.mkdir(parents=True)
    probe = Path(__file__).with_name('neon_probe.cpp')
    shutil.copyfile(probe, output/'neon_probe.cpp')
    args = command(root, ['g++', '-O2', '-std=c++11', '-march=armv8-a+simd',
                         '-I/ws/src/fast_neon/include',
                         '-I/ws/src/SchurVINS/svo_direct/include',
                         '-I/ws/src/SchurVINS/svo_common/include',
                         '-I/usr/include/eigen3', '-I/usr/include/opencv',
                         '/probe/neon_probe.cpp',
                         '/ws/src/fast_neon/src/fast_9.cpp',
                         '/ws/src/fast_neon/src/faster_corner_9_neon.cpp',
                         '-o', '/probe/neon_probe'], [(workspace, '/ws'), (output, '/probe')])
    write_json(output/'command.json', args)
    with (output/'neon.log').open('x') as log:
        result = subprocess.run(args, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode == 0:
            elf_header(output/'neon_probe')
            result = subprocess.run(command(root, ['/probe/neon_probe'], [(output, '/probe')]),
                                    stdout=log, stderr=subprocess.STDOUT)
    write_json(output/'result.json', dict(status='PASS' if result.returncode == 0 else 'FAIL',
               exit_code=result.returncode, evidence_class='EMULATED / MOCK VERIFIED',
               probe_sha256=file_hash(probe),
               source_sha256={name:file_hash(workspace/'src'/name) for name in
                   ['fast_neon/src/fast_9.cpp', 'fast_neon/src/faster_corner_9_neon.cpp',
                    'SchurVINS/svo_direct/include/svo/direct/patch_score.h']},
               scope='scalar/NEON correctness only; no target timing inference'))
    return result.returncode


def installation_manifest(root, workspace, output, environment, elf):
    """Freeze build artifacts and the independently provisioned target runtime."""
    root, workspace, output = map(external, (root, workspace, output))
    return dict(schema_version=1, backend='SchurVINS', commit=PIN,
                target='aarch64-bionic-melodic-reference', architecture='arm64',
                platform='linux/arm64', runtime='proot-qemu', image_id=HELPER_IMAGE_ID,
                workspace=str(workspace), root=str(root),
                binary='devel/.private/svo_ros/lib/svo_ros/svo_node',
                binary_sha256=elf['binary_sha256'], elf=elf,
                source_files_sha256=snapshot(workspace/'src'),
                source_symlinks=symlinks(workspace/'src'),
                devel_files_sha256=snapshot(workspace/'devel'),
                devel_symlinks=symlinks(workspace/'devel'),
                rootfs=rootfs_tree(root/'rootfs'),
                runtime_files_sha256={name:file_hash(root/name) for name in RUNTIME_HELPERS},
                helper_symlinks=symlinks(root/'proot'), environment=environment,
                patches={name:file_hash(Path(__file__).with_name(name)) for name in
                         ['platform.patch', 'aarch64-fast.patch',
                          'aarch64-fast-neon.patch', 'aarch64-neon.patch']},
                build_command_sha256=file_hash(output/'command.json'),
                source_lock_sha256=file_hash(workspace/'source-lock.json'),
                variant='upstream with ARM64 portability patches',
                scope='ARM64 reference build; RK3576 BSP/ABI and performance NOT_RUN')


def build(root, workspace, output, source_lock=None, memory_gib=12, *, base_installation=None, extra_patches=()):
    root, workspace, output = external(root), external(workspace), external(output)
    if output.exists():
        raise FileExistsError(output)
    if base_installation is not None:
        if source_lock is not None:
            raise ValueError('base installation already pins the complete source lock')
        return incremental_build(root, workspace, output, base_installation, extra_patches, memory_gib)
    if extra_patches:
        raise ValueError('derivative patches require an independently copied base installation')
    if not (root/'environment.json').is_file():
        raise ValueError('provision a locked ARM64 environment first')
    env = json.loads((root/'environment.json').read_text())
    if file_hash(Path(env['package_lock'])) != env['packages_sha256']:
        raise ValueError('ARM64 package lock changed')
    if packages(root) != Path(env['package_lock']).read_text():
        raise ValueError('ARM64 installed package closure changed')
    disk_guard(workspace, 16*1024**3)
    prepare(Path(source_lock) if source_lock else Path(__file__).with_name('sources.lock.json'), workspace)
    apply_patch(workspace, 'SchurVINS', 'platform.patch')
    apply_patch(workspace, 'fast_neon', 'aarch64-fast.patch')
    apply_patch(workspace, 'fast_neon', 'aarch64-fast-neon.patch')
    apply_patch(workspace, 'SchurVINS', 'aarch64-neon.patch')
    for name in ['svo_global_map','svo_online_loopclosing','svo_pgo','rqt_svo','svo_benchmarking']:
        (workspace/'src/SchurVINS'/name/'CATKIN_IGNORE').touch()
    output.mkdir(parents=True)
    (output/'patches').mkdir()
    for name in ['platform.patch', 'aarch64-fast.patch',
                 'aarch64-fast-neon.patch', 'aarch64-neon.patch']:
        shutil.copyfile(Path(__file__).with_name(name), output/'patches'/name)
    shell = ('source /opt/ros/melodic/setup.bash && cd /ws && '
             'catkin config --extend /opt/ros/melodic --cmake-args '
             '-DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON '
             '-DUSE_SYSTEM_EIGEN=ON -DUSE_SYSTEM_GLOG=ON && '
             'catkin build svo_ros --no-status -j4 -p2')
    args = command(root, ['/bin/bash', '-c', shell], [(workspace, '/ws')], memory_gib)
    write_json(output/'command.json', args)
    with (output/'build.log').open('x') as log:
        try:
            code = run_logged(args, log, 8*3600)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            code = 124 if isinstance(exc, subprocess.TimeoutExpired) else 130
            write_json(output/'result.json', dict(status='FAIL', exit_code=code,
                       stage='build-timeout' if code == 124 else 'build-interrupted',
                       reason='owned process group terminated and reaped', memory_gib=memory_gib))
            return code
    write_json(output/'result.json', dict(status='NOT_RUN' if code == 0 else 'FAIL', exit_code=code,
                                         evidence_class=None,
                                         memory_gib=memory_gib, scope='build only; ELF verification is separate'))
    if code:
        return code
    elf = verify(root, workspace, output/'elf')
    write_json(output/'result.json', dict(status='PASS', exit_code=0, evidence_class='ARM64 BUILD VERIFIED', memory_gib=memory_gib, scope='reference build and target ELF closure; no replay or board qualification'))
    write_json(output/'installation.json', installation_manifest(root, workspace, output, env, elf))
    return 0


def package_output_isolation(output, *inputs):
    """Reject a destination that could mutate or recursively include its inputs."""
    for path in inputs:
        path = Path(path).resolve()
        if output == path or output in path.parents or path in output.parents:
            raise ValueError('independent package output required: '+str(path))


def package_overlay(root, workspace, installation, output, base_installation, base_archive):
    """Package an exact source/devel delta against the existing userspace bundle."""
    root, workspace, output = map(external, (root, workspace, output))
    if output.exists():
        raise FileExistsError(output)
    installation, base_installation, base_archive = map(Path, (installation, base_installation, base_archive))
    target = json.loads(installation.read_text())
    base = json.loads(base_installation.read_text())
    base_package_path = base_archive.with_name('package.json')
    base_package = json.loads(base_package_path.read_text())
    origin = external(base['workspace'])
    package_output_isolation(output, workspace, origin, root/'rootfs', root/'proot',
        root/'qemu-bionic', installation.parent, base_installation.parent, base_archive.parent)
    if (Path(target['workspace']).resolve() != workspace or checked_runtime(target) != root
            or target['rootfs'] != base['rootfs'] or target['environment'] != base['environment']
            or file_hash(base_archive) != base_package['archive_sha256']
            or file_hash(base_installation) != base_package['installation_sha256']):
        raise ValueError('overlay base archive/userspace does not match the locked installations')
    payload = {}
    removed = []
    for tree, files_key, links_key in [('src','source_files_sha256','source_symlinks'),
                                        ('devel','devel_files_sha256','devel_symlinks')]:
        for directory, manifest in [(origin,base), (workspace,target)]:
            if (snapshot(directory/tree) != manifest[files_key]
                    or symlinks(directory/tree) != manifest[links_key]):
                raise ValueError('overlay source/devel installation changed')
        old_files, new_files = base[files_key], target[files_key]
        old_links, new_links = base[links_key], target[links_key]
        removed += ['ws/'+tree+'/'+name for name in (set(old_files)|set(old_links))-(set(new_files)|set(new_links))]
        for name in set(new_files)|set(new_links):
            changed = (new_links.get(name) != old_links.get(name) if name in new_links
                       else new_files[name] != old_files.get(name) or name in old_links)
            if changed:
                payload['ws/'+tree+'/'+name] = workspace/tree/name
    metadata = {'metadata/installation.json':installation, 'metadata/base-installation.json':base_installation,
                'metadata/base-package.json':base_package_path, 'metadata/source-lock.json':workspace/'source-lock.json'}
    for name, digest in target['patches'].items():
        candidates = [installation.parent/'patches'/name, Path(__file__).with_name(name)]
        patch = next((p for p in candidates if p.is_file() and file_hash(p)==digest), None)
        if patch is None:
            raise ValueError('frozen deployment patch is missing: '+name)
        metadata['metadata/patches/'+name] = patch
    expected = sum(p.stat().st_size for p in [*payload.values(),*metadata.values()] if not p.is_symlink())
    expected += 2048*(len(payload)+len(metadata)+4)+65536
    disk_guard(output, expected)
    output.mkdir(parents=True)
    archive = output/'overlay.tar.gz'
    with archive.open('xb') as stream, gzip.GzipFile(fileobj=stream, mode='wb', filename='', mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode='w') as bundle:
            for name, path in sorted({**payload,**metadata}.items()):
                info = bundle.gettarinfo(str(path), arcname=name)
                info.uid = info.gid = 0
                info.uname = info.gname = ''
                info.mtime = 0
                if info.isfile():
                    with path.open('rb') as source:
                        bundle.addfile(info, source)
                else:
                    bundle.addfile(info)
    # Use bounded task-owned RAM staging. The existing multi-GiB userspace is
    # never copied or extracted merely to check a small workspace overlay.
    stage_bytes = sum(p.stat().st_size for tree in ('src','devel') for p in (origin/tree).rglob('*')
                      if p.is_file() and not p.is_symlink())+expected
    temporary_root = Path('/dev/shm')
    if not temporary_root.is_dir() or shutil.disk_usage(temporary_root).free < stage_bytes:
        disk_guard(output, stage_bytes)
        temporary_root = output
    with tempfile.TemporaryDirectory(prefix='rm27-arm-overlay-', dir=temporary_root) as temporary:
        stage = Path(temporary)
        def restore(member, source):
            relative = PurePosixPath(member.name)
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('unsafe deployment member')
            destination = stage/member.name
            if any(parent.is_symlink() for parent in destination.parents if parent != stage.parent):
                raise ValueError('deployment member parent is a symlink')
            destination.parent.mkdir(parents=True, exist_ok=True)
            if member.isdir():
                destination.mkdir(exist_ok=True)
                return
            if destination.is_symlink() or destination.is_file():
                destination.unlink()
            if member.issym():
                destination.symlink_to(member.linkname)
            elif member.isfile():
                with destination.open('wb') as sink:
                    shutil.copyfileobj(source.extractfile(member), sink)
                destination.chmod(member.mode)
            else:
                raise ValueError('unsupported deployment member kind')
        with tarfile.open(base_archive, 'r:gz') as bundle:
            for member in bundle:
                if any(member.name == name or member.name.startswith(name+'/') for name in ('ws/src','ws/devel')):
                    restore(member, bundle)
        if (snapshot(stage/'ws/src') != base['source_files_sha256']
                or snapshot(stage/'ws/devel') != base['devel_files_sha256']
                or symlinks(stage/'ws/src') != base['source_symlinks']
                or symlinks(stage/'ws/devel') != base['devel_symlinks']):
            raise ValueError('base archive workspace does not match its installation manifest')
        for name in sorted(removed, reverse=True):
            path = stage/name
            if path.is_symlink() or path.is_file():
                path.unlink()
        with tarfile.open(archive, 'r:gz') as bundle:
            seen = set()
            for member in bundle:
                name = PurePosixPath(member.name)
                if name.is_absolute() or '..' in name.parts:
                    raise ValueError('unsafe overlay member')
                if member.name in seen:
                    raise ValueError('duplicate overlay member')
                seen.add(member.name)
                if member.name not in payload:
                    if member.name not in metadata:
                        raise ValueError('unexpected overlay member')
                    if not member.isfile() or bundle.extractfile(member).read() != metadata[member.name].read_bytes():
                        raise ValueError('overlay metadata changed')
                    continue
                restore(member, bundle)
            if seen != set(payload)|set(metadata):
                raise ValueError('overlay members are incomplete')
        if (snapshot(stage/'ws/src') != target['source_files_sha256']
                or snapshot(stage/'ws/devel') != target['devel_files_sha256']
                or symlinks(stage/'ws/src') != target['source_symlinks']
                or symlinks(stage/'ws/devel') != target['devel_symlinks']):
            raise ValueError('base plus overlay does not reproduce the complete derivative manifest')
        elf_header(stage/'ws'/target['binary'])
    disk_guard(output)
    write_json(output/'package.json', dict(schema_version=1, status='PASS',
        evidence_class='ARM64 BUILD VERIFIED', archive_sha256=file_hash(archive),
        base_archive=str(base_archive.resolve()), base_archive_sha256=file_hash(base_archive),
        base_installation_sha256=file_hash(base_installation), installation_sha256=file_hash(installation),
        replaced_or_added_paths=sorted(payload), removed_paths=sorted(removed),
        permanent_upper_estimate_bytes=expected, actual_archive_bytes=archive.stat().st_size,
        roundtrip_stage_bytes=stage_bytes, roundtrip_verified=True, staging_removed=True,
        layout=dict(requires_base_archive=True, rootfs_included=False, guest_workspace='/ws'),
        scope='reference base plus exact RM27 workspace overlay only; RK3576 BSP/ABI/startup/performance NOT_RUN'))
    return 0


def package(root, workspace, installation, output, *, base_installation=None, base_archive=None):
    """Bundle verified reference userspace and workspace without dataset/builds."""
    root, workspace, output = external(root), external(workspace), external(output)
    if output.exists():
        raise FileExistsError(output)
    package_output_isolation(output, workspace, root/'rootfs', root/'proot',
                             root/'qemu-bionic', Path(installation).resolve().parent)
    if base_installation is not None or base_archive is not None:
        if base_installation is None or base_archive is None:
            raise ValueError('overlay requires both base installation and base archive')
        return package_overlay(root, workspace, installation, output, base_installation, base_archive)
    manifest = json.loads(Path(installation).read_text())
    if (manifest.get('target') != 'aarch64-bionic-melodic-reference'
            or Path(manifest.get('workspace', '')).resolve() != workspace
            or Path(manifest.get('root', '')).resolve() != root):
        raise ValueError('installation is not this ARM64 reference')
    checked_runtime(manifest)
    if (snapshot(workspace/'src') != manifest['source_files_sha256']
            or snapshot(workspace/'devel') != manifest['devel_files_sha256']
            or symlinks(workspace/'src') != manifest['source_symlinks']
            or symlinks(workspace/'devel') != manifest['devel_symlinks']):
        raise ValueError('ARM64 installation files changed')
    paths = [(root/'rootfs', 'rootfs'), (workspace/'src', 'ws/src'),
             (workspace/'devel', 'ws/devel')]
    raw_size = sum(p.stat().st_size for source, _ in paths for p in source.rglob('*')
                   if p.is_file() and not p.is_symlink())
    disk_guard(output, raw_size)
    output.mkdir(parents=True)
    verified = verify(root, workspace, output/'elf')
    if verified['binary_sha256'] != manifest['binary_sha256']:
        raise ValueError('ARM64 executable changed')
    def runtime_files(member):
        for name in ['rootfs/tmp', 'rootfs/proc', 'rootfs/sys', 'rootfs/dev',
                     'rootfs/ws', 'rootfs/probe', 'rootfs/host-rootfs',
                     'rootfs/out', 'rootfs/tools', 'rootfs/input']:
            if member.name == name:
                # PRoot's host bind stubs may be mode 000. Ship an empty mount
                # point, never those host/virtual contents or inaccessible mode.
                if not member.isdir():
                    return None
                member.mode = 0o1777 if name == 'rootfs/tmp' else 0o755
                return member
            if member.name.startswith(name+'/'):
                return None
        if any(member.name == name or member.name.startswith(name+'/') for name in
               ['rootfs/var/cache/apt', 'rootfs/var/lib/apt/lists']):
            return None
        return member
    archive = output/'reference.tar.gz'
    with tarfile.open(archive, 'w:gz', dereference=False) as target:
        for source, name in paths:
            target.add(source, arcname=name, filter=runtime_files)
    shutil.copyfile(installation, output/'installation.json')
    write_json(output/'package.json', dict(status='PASS',
               evidence_class='ARM64 BUILD VERIFIED', archive_sha256=file_hash(archive),
               installation_sha256=file_hash(output/'installation.json'),
               layout=dict(target_userspace='rootfs', workspace='ws', guest_workspace='/ws',
                           dataset_included=False, emulation_helpers_included=False),
               scope='reference userspace archive, requires /ws layout; RK3576 BSP/ABI and startup on actual board NOT_RUN'))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['provision','build','verify','neon','package'])
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--workspace', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--package-lock', type=Path)
    parser.add_argument('--source-lock', type=Path)
    parser.add_argument('--installation', type=Path)
    parser.add_argument('--base-installation', type=Path,
                        help='hash-verified ARM64 reference to copy before an incremental derivative build')
    parser.add_argument('--patch', type=Path, action='append', default=[],
                        help='ordered frozen derivative patch; requires --base-installation')
    parser.add_argument('--isolation-directory', type=Path,
                        help='verify using read-only target runtime and existing build-tmp/build-home bindings')
    parser.add_argument('--base-archive', type=Path,
                        help='existing reference archive to retain as the base of a small package overlay')
    parser.add_argument('--memory-gib', type=int, choices=[4,12], default=12)
    args = parser.parse_args()
    if args.action == 'provision':
        return provision(args.root, args.output, args.package_lock)
    if args.workspace is None:
        parser.error('--workspace is required for build/verify')
    if args.action == 'build':
        return build(args.root, args.workspace, args.output, args.source_lock, args.memory_gib,
                     base_installation=args.base_installation, extra_patches=args.patch)
    if args.action == 'neon':
        return neon_probe(args.root, args.workspace, args.output)
    if args.action == 'package':
        if args.installation is None:
            parser.error('--installation is required for package')
        return package(args.root, args.workspace, args.installation, args.output,
                       base_installation=args.base_installation, base_archive=args.base_archive)
    verify(args.root, args.workspace, args.output, isolated_output=args.isolation_directory)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
