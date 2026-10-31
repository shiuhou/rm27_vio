"""ARM64 artifact/target isolation gates; ELF fixtures do not prove a build."""
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
from types import SimpleNamespace

import pytest

from tools.schurvins import arm64


def elf(path, machine=183, bits=2, endian=1):
    data = bytearray(64)
    data[:7] = b'\x7fELF'+bytes([bits, endian, 1])
    struct.pack_into('<H', data, 18, machine)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


@pytest.mark.parametrize('machine,bits,endian', [(62,2,1),(40,1,1),(183,2,2)])
def test_wrong_isa_or_elf_encoding_is_rejected(tmp_path, machine, bits, endian):
    with pytest.raises(ValueError):
        arm64.elf_header(elf(tmp_path/'artifact',machine,bits,endian))


def test_aarch64_header_and_truncated_non_elf(tmp_path):
    assert arm64.elf_header(elf(tmp_path/'target'))['machine'] == 'AArch64'
    path=tmp_path/'truncated'
    path.write_bytes(b'\x7fELF')
    with pytest.raises(ValueError, match='not an ELF'):
        arm64.elf_header(path)


def test_absolute_target_symlinks_stay_in_target(tmp_path):
    root=tmp_path/'root'
    elf(root/'usr/lib/aarch64-linux-gnu/libtarget.so')
    (root/'lib').symlink_to('/usr/lib')
    assert arm64.guest_file(root,'/lib/aarch64-linux-gnu/libtarget.so') == root/'usr/lib/aarch64-linux-gnu/libtarget.so'
    (root/'usr/lib/relative').symlink_to('aarch64-linux-gnu/libtarget.so')
    assert arm64.guest_file(root,'/lib/relative') == root/'usr/lib/aarch64-linux-gnu/libtarget.so'


def test_target_escape_and_symlink_cycle_rejected(tmp_path):
    (tmp_path/'cycle').symlink_to('/cycle')
    with pytest.raises(ValueError,match='cycle'):
        arm64.guest_file(tmp_path,'/cycle')
    with pytest.raises(ValueError,match='escapes'):
        arm64.guest_file(tmp_path,'/../../etc/shadow')
    with pytest.raises(ValueError,match='absolute'):
        arm64.guest_file(tmp_path,'relative')


def test_catkin_absolute_alias_is_resolved_inside_workspace(tmp_path):
    real=elf(tmp_path/'devel/.private/pkg/lib/libpkg.so')
    public=tmp_path/'devel/lib'
    public.mkdir()
    (public/'libpkg.so').symlink_to('/ws/devel/.private/pkg/lib/libpkg.so')
    assert arm64.guest_file(tmp_path,'/ws/devel/lib/libpkg.so','/ws') == real
    (public/'host.so').symlink_to('/lib/host.so')
    with pytest.raises(ValueError,match='leaves binding'):
        arm64.guest_file(tmp_path,'/ws/devel/lib/host.so','/ws')


def test_space_guard_retains_20gib(monkeypatch,tmp_path):
    monkeypatch.setattr(arm64.shutil,'disk_usage',lambda _:SimpleNamespace(free=21*1024**3))
    assert arm64.disk_guard(tmp_path,1024**3) == 21*1024**3
    with pytest.raises(OSError,match='20 GiB'):
        arm64.disk_guard(tmp_path,1024**3+1)


def test_build_cannot_mutate_repository_and_output_cannot_overwrite(tmp_path):
    repo=Path(arm64.__file__).resolve().parents[2]
    with pytest.raises(ValueError,match='outside repository'):
        arm64.external(repo/'new-arm64-workspace')
    existing=tmp_path/'existing'
    existing.mkdir()
    with pytest.raises(FileExistsError):
        arm64.build(tmp_path/'root',tmp_path/'ws',existing)
    assert list(existing.iterdir()) == []


def test_explicit_emulator_and_resource_scope_command(tmp_path,monkeypatch):
    monkeypatch.setenv('CMAKE_PREFIX_PATH','/host/ros2')
    monkeypatch.setenv('LD_LIBRARY_PATH','/host/x86-libs')
    cmd=arm64.command(tmp_path/'env',['/bin/true'],[(tmp_path/'ws','/ws')],4)
    assert cmd[:4] == ['systemd-run','--user','--scope','--quiet']
    assert 'MemoryMax=4G' in cmd
    assert 'CPUQuota=400%' in cmd
    assert cmd[cmd.index('-q')+1].endswith('/qemu-bionic/usr/bin/qemu-aarch64-static')
    assert str(tmp_path/'ws')+':/ws' in cmd
    assert '/usr/bin/env' in cmd and '-i' in cmd
    assert not any('/host/' in part for part in cmd)
    with pytest.raises(ValueError,match='binding'):
        arm64.command(tmp_path/'env',['/bin/true'],[(tmp_path/'ws','relative')])


def test_timeout_reaps_child_process_group(tmp_path):
    child_file=tmp_path/'child.pid'
    script=('import subprocess,sys,time; '
            'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(30)"]); '
            'open(sys.argv[1],"w").write(str(p.pid)); time.sleep(30)')
    with (tmp_path/'command.log').open('w') as log:
        with pytest.raises(subprocess.TimeoutExpired):
            arm64.run_logged([sys.executable,'-c',script,str(child_file)],log,.3)
    child=int(child_file.read_text())
    status=Path('/proc')/str(child)/'status'
    # A short-lived zombie awaiting PID1 reaping cannot execute or spawn work.
    if status.exists():
        assert 'State:\tZ' in status.read_text()


def test_timeout_exit_race_still_reaps_and_preserves_timeout(monkeypatch):
    waits=iter([subprocess.TimeoutExpired(['finished-command'],.1),0])
    def wait(**kwargs):
        result=next(waits)
        if isinstance(result,Exception):raise result
        return result
    monkeypatch.setattr(arm64.subprocess,'Popen',lambda *a,**k:SimpleNamespace(pid=27,wait=wait))
    def gone(*args):raise ProcessLookupError('already exited')
    monkeypatch.setattr(arm64.os,'killpg',gone)
    with pytest.raises(subprocess.TimeoutExpired):arm64.run_logged(['finished-command'],None,.1)


def verification_fixture(tmp_path):
    root=tmp_path/'env'
    ws=tmp_path/'ws'
    elf(ws/'devel/.private/svo_ros/lib/svo_ros/svo_node')
    elf(root/'rootfs/lib/ld-linux-aarch64.so.1')
    elf(root/'rootfs/lib/aarch64-linux-gnu/libc.so.6')
    return root,ws


@pytest.mark.parametrize('failure', ['unresolved','wrong-library','loader-failure','no-libraries'])
def test_dependency_gate_never_passes_partial_or_host_resolution(tmp_path,monkeypatch,failure):
    root,ws=verification_fixture(tmp_path)
    monkeypatch.setattr(arm64.subprocess,'check_output',lambda *a,**k:'[Requesting program interpreter: /lib/ld-linux-aarch64.so.1]')
    output='libc.so.6 => /lib/aarch64-linux-gnu/libc.so.6 (0x1)\n'
    code=0
    if failure=='unresolved':output='libc.so.6 => not found\n'
    if failure=='wrong-library':elf(root/'rootfs/lib/aarch64-linux-gnu/libc.so.6',62)
    if failure=='loader-failure':code=1
    if failure=='no-libraries':output=''
    monkeypatch.setattr(arm64.subprocess,'run',lambda *a,**k:SimpleNamespace(returncode=code,stdout=output,stderr=''))
    with pytest.raises(ValueError):
        arm64.verify(root,ws,tmp_path/'verify')
    assert not (tmp_path/'verify/elf.json').exists()


def test_dependency_gate_records_actual_target_library_hashes(tmp_path,monkeypatch):
    root,ws=verification_fixture(tmp_path)
    monkeypatch.setattr(arm64.subprocess,'check_output',lambda *a,**k:'[Requesting program interpreter: /lib/ld-linux-aarch64.so.1]')
    monkeypatch.setattr(arm64.subprocess,'run',lambda *a,**k:SimpleNamespace(returncode=0,stdout='libc.so.6 => /lib/aarch64-linux-gnu/libc.so.6 (0x1)\n',stderr=''))
    report=arm64.verify(root,ws,tmp_path/'verify')
    assert report['status']=='PASS'
    assert report['evidence_class']=='ARM64 BUILD VERIFIED'
    assert report['libraries_sha256']['/lib/aarch64-linux-gnu/libc.so.6'] == arm64.file_hash(root/'rootfs/lib/aarch64-linux-gnu/libc.so.6')
    assert json.loads((tmp_path/'verify/elf.json').read_text()) == report


def test_rootfs_manifest_never_hashes_host_symlink_targets_or_virtual_mounts(tmp_path):
    (tmp_path/'etc').mkdir()
    (tmp_path/'etc/target.conf').write_text('target configuration')
    (tmp_path/'etc/host-link').symlink_to('/etc/passwd')
    (tmp_path/'tmp').mkdir()
    (tmp_path/'tmp/transient').write_text('not deployment input')
    tree=arm64.rootfs_tree(tmp_path)
    assert tree == {'files_sha256':{'etc/target.conf':arm64.file_hash(tmp_path/'etc/target.conf')},
                    'symlinks':{'etc/host-link':'/etc/passwd'}}


@pytest.mark.skipif(os.geteuid()==0,reason='root bypasses directory permissions')
def test_unreadable_target_directory_cannot_silently_disappear_from_manifest(tmp_path):
    hidden=tmp_path/'etc/hidden'
    hidden.mkdir(parents=True)
    (hidden/'target.conf').write_text('must not silently skip')
    hidden.chmod(0)
    try:
        with pytest.raises(PermissionError):arm64.rootfs_tree(tmp_path)
    finally:hidden.chmod(0o755)


def runtime_fixture(tmp_path):
    root=tmp_path/'arm64'
    for name in arm64.RUNTIME_HELPERS:
        path=root/name
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(name)
    (root/'rootfs/etc').mkdir(parents=True)
    (root/'rootfs/etc/target.conf').write_text('immutable target')
    lock=root/'packages.lock'
    lock.write_text('target-package\t1.0\n')
    environment={'package_lock':str(lock),'packages_sha256':arm64.file_hash(lock)}
    (root/'environment.json').write_text(json.dumps(environment))
    (root/'proot/usr/lib/x86_64-linux-gnu/libtalloc.so.2').symlink_to('libtalloc.so.2.4.2')
    manifest=dict(root=str(root),workspace=str(root/'ws'),target='aarch64-bionic-melodic-reference',
                  architecture='arm64',platform='linux/arm64',runtime='proot-qemu',
                  image_id=arm64.HELPER_IMAGE_ID,rootfs=arm64.rootfs_tree(root/'rootfs'),
                  runtime_files_sha256={name:arm64.file_hash(root/name) for name in arm64.RUNTIME_HELPERS},
                  helper_symlinks=arm64.symlinks(root/'proot'),environment=environment)
    return root,manifest


@pytest.mark.parametrize('changed',['rootfs-file','rootfs-new','helper','symlink','lock'])
def test_runtime_integrity_rejects_changes_without_target_execution(tmp_path,changed):
    root,manifest=runtime_fixture(tmp_path)
    assert arm64.checked_runtime(manifest)==root
    if changed=='rootfs-file':(root/'rootfs/etc/target.conf').write_text('changed')
    if changed=='rootfs-new':(root/'rootfs/etc/new.conf').write_text('new')
    if changed=='helper':(root/'proot/usr/bin/proot').write_text('changed')
    if changed=='symlink':
        link=root/'proot/usr/lib/x86_64-linux-gnu/libtalloc.so.2'
        link.unlink()
        link.symlink_to('/host/libtalloc.so.2')
    if changed=='lock':(root/'packages.lock').write_text('target-package\t2.0\n')
    with pytest.raises(ValueError):arm64.checked_runtime(manifest)


def replay_fixture(tmp_path):
    root,manifest=runtime_fixture(tmp_path)
    inputs=tmp_path/'sensor-only'
    inputs.mkdir()
    output=tmp_path/'fresh-run'
    (output/'runtime-tmp').mkdir(parents=True)
    transport=output/'runtime-tools'
    transport.mkdir()
    for name in ('run_inner.py','ros_transport.py'):(transport/name).write_text('# frozen')
    return manifest,transport,inputs,output


def test_replay_uses_existing_frozen_scripts_readonly_and_explicit_emulator(tmp_path,monkeypatch):
    args=replay_fixture(tmp_path)
    monkeypatch.setenv('PYTHONPATH','/host/ros2')
    cmd=arm64.replay_command(*args,'stereo',.5,900,container_name='rm27-schur-arm64-test')
    assert cmd[:3]==['docker','run','--rm']
    assert '--network' in cmd and cmd[cmd.index('--network')+1]=='none'
    assert cmd[cmd.index('--memory')+1]=='12g'
    assert cmd[cmd.index('--cpus')+1]=='4'
    assert cmd[cmd.index('-q')+1].endswith('qemu-bionic/usr/bin/qemu-aarch64-static')
    assert str(args[1])+':'+str(args[1])+':ro' in cmd
    assert str(args[2])+':'+str(args[2])+':ro' in cmd
    assert str(args[3])+':'+str(args[3])+':rw' in cmd
    assert str(args[3]/'runtime-tmp')+':/tmp' in cmd
    assert 'exec python /tools/run_inner.py' in cmd[-10]
    assert '/host/ros2' not in ' '.join(cmd)
    assert cmd[-8:]==['--mode','stereo','--rate','0.5','--timeout','900','--start-offset','0']
    assert list((args[3]/'runtime-tmp').iterdir())==[]


@pytest.mark.parametrize('field,value', [('rate',0),('rate',float('nan')),
    ('timeout_s',float('inf')),('start_offset_s',-1),('memory_gib',8),
    ('container_name','unowned-name')])
def test_replay_refuses_invalid_timing_or_ownership(tmp_path,field,value):
    args=replay_fixture(tmp_path)
    kwargs=dict(mode='mono',rate=.5,timeout_s=900,start_offset_s=0,
                container_name='rm27-schur-test',memory_gib=12)
    kwargs[field]=value
    with pytest.raises(ValueError):arm64.replay_command(*args,**kwargs)


def test_replay_never_overwrites_temporary_files_or_follows_transport_links(tmp_path):
    args=replay_fixture(tmp_path)
    temporary=args[3]/'runtime-tmp/previous-evidence'
    temporary.write_text('retain')
    with pytest.raises(ValueError,match='fresh'):
        arm64.replay_command(*args,'mono',.5,900,container_name='rm27-schur-test')
    assert temporary.read_text()=='retain'
    temporary.unlink()
    script=args[1]/'run_inner.py'
    script.unlink()
    script.symlink_to('/host/run_inner.py')
    with pytest.raises(ValueError,match='regular'):
        arm64.replay_command(*args,'mono',.5,900,container_name='rm27-schur-test')
