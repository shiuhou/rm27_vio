"""Native candidate assembly from tiny archives; no Docker or board execution."""
import copy
import hashlib
import io
import json
import struct
import tarfile
from types import SimpleNamespace
import zipfile
import zlib

import pytest

from tools.schurvins import rk3576_prepare as prep


def sha(content):
    return hashlib.sha256(content).hexdigest()


def elf(machine=183, bits=2, endian=1):
    content = bytearray(64)
    content[:7] = b'\x7fELF' + bytes([bits, endian, 1])
    struct.pack_into('<H', content, 18, machine)
    return bytes(content)


def archive(path, entries):
    """Entries are (name, kind, bytes/target); order matters for alias attacks."""
    with tarfile.open(path, 'w:gz') as target:
        for name, kind, value in entries:
            member = tarfile.TarInfo(name)
            member.mode = 0o755 if name.endswith('svo_node') else 0o644
            if kind == 'file':
                member.size = len(value)
                target.addfile(member, io.BytesIO(value))
            else:
                member.type = {'symlink': tarfile.SYMTYPE,
                               'hardlink': tarfile.LNKTYPE}[kind]
                member.linkname = value
                target.addfile(member)


@pytest.fixture
def candidate(tmp_path):
    binary = 'devel/.private/svo_ros/lib/svo_ros/svo_node'
    public = 'devel/lib/svo_ros/svo_node'
    root_files = {'lib/ld-linux-aarch64.so.1': elf(),
                  'usr/lib/libdependency.so': elf() + b'library',
                  'etc/reference': b'pinned userspace',
                  'bin/bzip2': b'fixture utility',
                  'bin/bzcat': b'fixture utility'}
    root_links = {'lib/libdependency.so': '/usr/lib/libdependency.so',
                  'lib/library-directory': '../usr/lib'}
    old_binary, new_binary = elf() + b'old', elf() + b'derivative'
    old_src = {'SchurVINS/node.cpp': b'old source',
               'SchurVINS/retired.cpp': b'removed source'}
    new_src = {'SchurVINS/node.cpp': b'derivative source',
               'SchurVINS/new.h': b'new source'}
    rootfs = dict(files_sha256={k: sha(v) for k, v in root_files.items()},
                  symlinks=root_links)

    def installation(src, content):
        return dict(architecture='arm64', platform='linux/arm64', runtime='proot-qemu',
                    commit='d8ab6dff20860694d8d879d716d7fd6cb2c2530e',
                    rootfs=copy.deepcopy(rootfs), binary=binary, binary_sha256=sha(content),
                    source_files_sha256={k: sha(v) for k, v in src.items()},
                    source_symlinks={}, devel_files_sha256={
                        binary.removeprefix('devel/'): sha(content),
                        public.removeprefix('devel/'): sha(content)},
                    devel_symlinks={public.removeprefix('devel/'): '/ws/' + binary},
                    elf=dict(libraries_sha256={
                        '/lib/ld-linux-aarch64.so.1': sha(root_files['lib/ld-linux-aarch64.so.1']),
                        '/lib/libdependency.so': sha(root_files['usr/lib/libdependency.so'])}),
                    source_lock_sha256=sha(b'locked sources'),
                    patches={'fixture.patch': sha(b'locked patch')})

    base = installation(old_src, old_binary)
    final = installation(new_src, new_binary)
    base_entries = [('rootfs/' + name, 'file', value)
                    for name, value in root_files.items() if name != 'bin/bzcat']
    base_entries += [('rootfs/bin/bzcat', 'hardlink', 'rootfs/bin/bzip2')]
    base_entries += [('rootfs/' + name, 'symlink', value + '/' if name == 'lib/library-directory' else value)
                     for name, value in root_links.items()]
    base_entries += [('ws/src/' + name, 'file', value) for name, value in old_src.items()]
    base_entries += [('ws/' + binary, 'file', old_binary),
                     ('ws/' + public, 'symlink', '/ws/' + binary)]
    overlay_entries = [('ws/src/' + name, 'file', value) for name, value in new_src.items()]
    overlay_entries += [('ws/' + binary, 'file', new_binary)]
    metadata = {'metadata/installation.json': json.dumps(final).encode(),
                'metadata/base-installation.json': json.dumps(base).encode(),
                'metadata/source-lock.json': b'locked sources',
                'metadata/patches/fixture.patch': b'locked patch'}
    overlay_entries += [(name, 'file', value) for name, value in metadata.items()]
    package = dict(replaced_or_added_paths=['ws/src/' + name for name in new_src] + ['ws/' + binary],
                   removed_paths=['ws/src/SchurVINS/retired.cpp'])
    base_dir, overlay_dir = tmp_path/'base', tmp_path/'overlay'
    base_dir.mkdir()
    overlay_dir.mkdir()
    base_archive, overlay_archive = base_dir/'reference.tar.gz', overlay_dir/'overlay.tar.gz'
    archive(base_archive, base_entries)
    archive(overlay_archive, overlay_entries)
    return SimpleNamespace(base=base, final=final, package=package, metadata=metadata,
                           base_entries=base_entries, overlay_entries=overlay_entries,
                           base_archive=base_archive, overlay_archive=overlay_archive,
                           binary=binary, public=public, old_binary=old_binary,
                           new_binary=new_binary, output=tmp_path/'flattened.tar.gz')


def flatten(fixture):
    return prep.flatten(fixture.base_archive, fixture.overlay_archive, fixture.package,
                        fixture.base, fixture.final, fixture.output)


def test_flatten_reconstructs_derivative_and_preserves_target_aliases(candidate):
    result, metadata = flatten(candidate)
    with tarfile.open(candidate.output) as image:
        names = image.getnames()
        assert all(not name.startswith(('rootfs/', 'metadata/')) for name in names)
        assert 'ws/src/SchurVINS/retired.cpp' not in names
        assert image.extractfile('ws/src/SchurVINS/node.cpp').read() == b'derivative source'
        assert image.extractfile('ws/src/SchurVINS/new.h').read() == b'new source'
        assert image.extractfile('ws/' + candidate.binary).read() == candidate.new_binary
        assert image.getmember('ws/' + candidate.binary).mode == 0o755
        alias = image.getmember('ws/' + candidate.public)
        assert alias.issym() and alias.linkname == '/ws/' + candidate.binary
        assert image.getmember('lib/library-directory').linkname == '../usr/lib'
        hardlink = image.getmember('bin/bzcat')
        assert hardlink.islnk() and hardlink.linkname == 'bin/bzip2'
        assert image.extractfile(hardlink).read() == b'fixture utility'
    assert metadata == candidate.metadata
    assert result['workspace_verified'] and result['userspace_verified']
    assert result['aarch64_executable_verified'] and result['loader_libraries'] == 2


def test_pax_long_paths_and_hardlinks_reopen_with_normalized_image_targets(candidate):
    long_dir = 'opt/ros/melodic/lib/python2.7/dist-packages/' + 'module_'*20
    regular, alias = long_dir + '/source.pyc', long_dir + '/alias.pyc'
    content = b'long path fixture bytecode'
    candidate.base_entries += [('rootfs/' + regular, 'file', content),
                               ('rootfs/' + alias, 'hardlink', 'rootfs/' + regular)]
    for installation in (candidate.base, candidate.final):
        installation['rootfs']['files_sha256'].update({regular: sha(content), alias: sha(content)})
    archive(candidate.base_archive, candidate.base_entries)
    with tarfile.open(candidate.base_archive) as source:
        assert source.getmember('rootfs/' + regular).pax_headers['path'] == 'rootfs/' + regular
        assert source.getmember('rootfs/' + alias).pax_headers['linkpath'] == 'rootfs/' + regular
    flatten(candidate)
    # Reopen the actual serialized archive: in-memory member.name/linkname can
    # look correct while retained PAX path/linkpath records override them.
    with tarfile.open(candidate.output) as image:
        assert not any(name.startswith('rootfs/') for name in image.getnames())
        hardlink = image.getmember(alias)
        assert hardlink.islnk() and hardlink.linkname == regular
        assert image.extractfile(hardlink).read() == content


@pytest.mark.parametrize('kind', ['missing', 'unexpected', 'changed', 'symlink'])
def test_entire_rootfs_membership_and_contents_are_required(candidate, kind):
    entries = candidate.base_entries
    if kind == 'missing':
        entries = [e for e in entries if e[0] != 'rootfs/etc/reference']
    elif kind == 'unexpected':
        entries += [('rootfs/etc/unlisted', 'file', b'unlisted')]
    elif kind == 'changed':
        entries = [(n, k, b'changed' if n == 'rootfs/etc/reference' else v) for n, k, v in entries]
    else:
        entries = [(n, k, '/host/library' if n == 'rootfs/lib/libdependency.so' else v)
                   for n, k, v in entries]
    archive(candidate.base_archive, entries)
    with pytest.raises(ValueError, match='userspace membership/hash mismatch'):
        flatten(candidate)


def test_loader_closure_cannot_be_satisfied_by_unverified_library(candidate):
    candidate.final['elf']['libraries_sha256']['/lib/libdependency.so'] = '0'*64
    with pytest.raises(ValueError, match='loader library closure mismatch'):
        flatten(candidate)


@pytest.mark.parametrize('content', [b'not ELF', elf(62), elf(bits=1), elf(endian=2), elf()[:20]])
def test_native_executable_must_be_complete_aarch64_elf(candidate, content):
    candidate.final['binary_sha256'] = sha(content)
    for name in candidate.final['devel_files_sha256']:
        candidate.final['devel_files_sha256'][name] = sha(content)
    entries = [(n, k, content if n == 'ws/' + candidate.binary else v)
               for n, k, v in candidate.overlay_entries]
    archive(candidate.overlay_archive, entries)
    with pytest.raises(ValueError, match='ELF64 AArch64'):
        flatten(candidate)


@pytest.mark.parametrize('attack', ['traversal', 'duplicate', 'symlink-parent'])
def test_archive_structure_cannot_escape_or_alias_image_members(candidate, attack):
    if attack == 'traversal':
        entries = [('rootfs/../../escape', 'file', b'escape')] + candidate.base_entries
    elif attack == 'duplicate':
        entries = candidate.base_entries + [candidate.base_entries[0]]
    else:
        entries = [('rootfs/etc/alias', 'symlink', '/outside'),
                   ('rootfs/etc/alias/child', 'file', b'escape')] + candidate.base_entries
    archive(candidate.base_archive, entries)
    with pytest.raises(ValueError, match='unsafe|duplicate|beneath a symlink'):
        flatten(candidate)
    assert not (candidate.output.parent/'escape').exists()


@pytest.fixture
def preparation(candidate, tmp_path, monkeypatch):
    """Complete local metadata; fake transport files are copied, never executed."""
    base_install = candidate.base_archive.with_name('installation.json')
    base_install.write_bytes(candidate.metadata['metadata/base-installation.json'])
    bp = dict(status='PASS', archive_sha256=sha(candidate.base_archive.read_bytes()),
              installation_sha256=sha(base_install.read_bytes()))
    op = dict(candidate.package, status='PASS', roundtrip_verified=True,
              archive_sha256=sha(candidate.overlay_archive.read_bytes()),
              installation_sha256=sha(candidate.metadata['metadata/installation.json']),
              base_archive_sha256=bp['archive_sha256'],
              base_installation_sha256=bp['installation_sha256'])
    base_package = candidate.base_archive.with_name('package.json')
    overlay_package = candidate.overlay_archive.with_name('package.json')
    base_package.write_text(json.dumps(bp))
    overlay_package.write_text(json.dumps(op))
    inputs = tmp_path/'input'
    inputs.mkdir()
    pixel = b'\x25'
    camera_yaml = (b'sensor_type: camera\nrate_hz: 20\nresolution: [1, 1]\n'
                   b'camera_model: pinhole\nintrinsics: [1.0, 1.0, 0.0, 0.0]\n'
                   b'distortion_model: radial-tangential\ndistortion_coefficients: [0.0, 0.0, 0.0, 0.0]\n')
    imu_yaml = (b'sensor_type: imu\nrate_hz: 200\ngyroscope_noise_density: 0.001\n'
                b'gyroscope_random_walk: 0.0001\naccelerometer_noise_density: 0.01\n'
                b'accelerometer_random_walk: 0.001\n')
    ledger = [dict(stream='imu0', state_ns=1, gyro=[0.1, 0.2, 0.3], accel=[1.0, 2.0, 3.0]),
              dict(stream='cam0', state_ns=1, pixels_sha256=sha(pixel), height=1,
                   width=1, encoding='mono8', step=1)]
    (inputs/'expected.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in ledger))
    with zipfile.ZipFile(inputs/'sensors.zip', 'w') as sensors:
        sensors.writestr('mav0/cam0/data/1.png', mono_png(pixel))
        sensors.writestr('mav0/cam0/data.csv', '#timestamp [ns],filename\n1,1.png\n')
        sensors.writestr('mav0/cam0/sensor.yaml', camera_yaml)
        sensors.writestr('mav0/imu0/data.csv', '#timestamp [ns],gyro_xyz,accel_xyz\n1,0.1,0.2,0.3,1,2,3\n')
        sensors.writestr('mav0/imu0/sensor.yaml', imu_yaml)
    data = dict(purpose='EUROC_PUBLIC_REPLAY', mode='mono', truth_included=False,
                counts={'cam0': 1, 'imu0': 1}, first_ns={'cam0': 1, 'imu0': 1},
                last_ns={'cam0': 1, 'imu0': 1},
                calibration_sha256={'cam0': sha(camera_yaml), 'imu0': sha(imu_yaml)},
                artifacts_sha256={
                    name: sha((inputs/name).read_bytes()) for name in ('expected.jsonl', 'sensors.zip')})
    (inputs/'input.json').write_text(json.dumps(data))
    tools = tmp_path/'fixture-tools'
    tools.mkdir()
    for name in ('rk3576_prepare.py', 'run_inner.py', 'ros_transport.py', 'rk3576_board.py'):
        (tools/name).write_bytes(b'# inert fixture, never execute\n')
    monkeypatch.setattr(prep, '__file__', str(tools/'rk3576_prepare.py'))
    monkeypatch.setattr(prep, 'disk_guard', lambda *args: None)
    return SimpleNamespace(base_package=base_package, overlay_package=overlay_package,
                           inputs=inputs, output=tmp_path/'prepared', data=data)


def prepare(fixture):
    return prep.prepare(fixture.base_package, fixture.overlay_package, fixture.inputs, fixture.output)


def mono_png(pixel):
    """Encode one grayscale pixel independently of the decoder under test."""
    def chunk(kind, payload):
        return (struct.pack('>I', len(payload)) + kind + payload
                + struct.pack('>I', zlib.crc32(kind + payload)))
    return (b'\x89PNG\r\n\x1a\n'
            + chunk(b'IHDR', struct.pack('>IIBBBBB', 1, 1, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(b'\0' + pixel)) + chunk(b'IEND', b''))


def test_preparation_is_local_only_and_keeps_qualification_not_run(preparation):
    output = prepare(preparation)
    result = json.loads((output/'candidate.json').read_text())
    assert result['status'] == 'PREPARED_NOT_RUN' and result['runtime'] == 'native-docker'
    assert set(result['evidence'].values()) == {'NOT_RUN'}
    assert result['runtime_archive_sha256'] == sha((output/'runtime-rootfs.tar.gz').read_bytes())
    assert all(sha((output/name).read_bytes()) == digest for name, digest in result['files_sha256'].items())
    assert (output/'input/sensors.zip').read_bytes() == (preparation.inputs/'sensors.zip').read_bytes()
    dockerfile = (output/'Dockerfile').read_text().splitlines()
    assert dockerfile[:2] == ['FROM scratch', 'ADD runtime-rootfs.tar.gz /']
    assert not any(line.startswith('RUN ') for line in dockerfile)
    with pytest.raises(FileExistsError):
        prepare(preparation)


@pytest.mark.parametrize('artifact', ['base', 'overlay', 'expected.jsonl', 'sensors.zip'])
def test_changed_sealed_files_are_rejected_before_output(preparation, artifact):
    path = {'base': preparation.base_package.with_name('reference.tar.gz'),
            'overlay': preparation.overlay_package.with_name('overlay.tar.gz')}.get(artifact)
    path = path if path is not None else preparation.inputs/artifact
    with path.open('ab') as target:
        target.write(b'tampered')
    with pytest.raises(ValueError, match='hash mismatch|artifact changed'):
        prepare(preparation)
    assert not preparation.output.exists()


@pytest.mark.parametrize('member', ['mav0/state_groundtruth_estimate0/data.csv',
                                     'mav0/cam0/../../truth.csv'])
def test_truth_or_traversal_sensor_members_are_rejected(preparation, member):
    with zipfile.ZipFile(preparation.inputs/'sensors.zip', 'a') as sensors:
        sensors.writestr(member, b'not permitted')
    preparation.data['artifacts_sha256']['sensors.zip'] = sha((preparation.inputs/'sensors.zip').read_bytes())
    (preparation.inputs/'input.json').write_text(json.dumps(preparation.data))
    with pytest.raises(ValueError, match='sensor archive|archive member|unsafe'):
        prepare(preparation)
    assert not preparation.output.exists()


def test_duplicate_sensor_member_is_rejected(preparation):
    with zipfile.ZipFile(preparation.inputs/'sensors.zip', 'a') as sensors:
        with pytest.warns(UserWarning, match='Duplicate name'):
            sensors.writestr('mav0/cam0/data/1.png', b'replacement image')
    preparation.data['artifacts_sha256']['sensors.zip'] = sha((preparation.inputs/'sensors.zip').read_bytes())
    (preparation.inputs/'input.json').write_text(json.dumps(preparation.data))
    with pytest.raises(ValueError, match='duplicate|archive member'):
        prepare(preparation)
    assert not preparation.output.exists()


@pytest.mark.parametrize('change', ['count', 'endpoint', 'missing-csv', 'calibration',
                                  'ledger-truth', 'pixels'])
def test_semantically_inconsistent_sensor_inputs_are_rejected_even_with_current_hashes(preparation, change):
    if change == 'count':
        preparation.data['counts']['cam0'] = 2
    elif change == 'endpoint':
        preparation.data['last_ns']['imu0'] = 2
    elif change == 'calibration':
        preparation.data['calibration_sha256']['cam0'] = '0'*64
    elif change == 'ledger-truth':
        ledger_path = preparation.inputs/'expected.jsonl'
        rows = [json.loads(line) for line in ledger_path.read_text().splitlines()]
        rows[0]['groundtruth'] = [0.0, 0.0, 0.0]
        ledger_path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    else:
        archive_path = preparation.inputs/'sensors.zip'
        with zipfile.ZipFile(archive_path) as sensors:
            members = {name: sensors.read(name) for name in sensors.namelist()}
        if change == 'missing-csv':
            del members['mav0/cam0/data.csv']
        else:
            members['mav0/cam0/data/1.png'] = mono_png(b'\x26')
        with zipfile.ZipFile(archive_path, 'w') as sensors:
            for name, payload in members.items():
                sensors.writestr(name, payload)
    preparation.data['artifacts_sha256'] = {
        name: sha((preparation.inputs/name).read_bytes()) for name in ('expected.jsonl', 'sensors.zip')}
    (preparation.inputs/'input.json').write_text(json.dumps(preparation.data))
    with pytest.raises(ValueError, match='count|endpoint|missing sensor|calibration hash|source ledger'):
        prepare(preparation)
    assert not preparation.output.exists()
