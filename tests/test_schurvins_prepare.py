"""Host-side pinned-source preparation; generated inputs are test fixtures only."""
import hashlib
import io
import json
import tarfile

import pytest

from tools.schurvins.prepare import PIN, extract, prepare
from tools.schurvins.build import build


def source_tar(name='root/file.txt', link=None):
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w:gz') as source:
        member = tarfile.TarInfo(name)
        if link is None:
            member.size = 4
            source.addfile(member, io.BytesIO(b'test'))
        else:
            member.type = tarfile.SYMTYPE
            member.linkname = link
            source.addfile(member)
    return buffer.getvalue()


@pytest.mark.parametrize('name,link', [('root/../../escape', None),
                                      ('root/link', '/tmp/escape'),
                                      ('root/link', '../../escape')])
def test_source_tar_cannot_escape_destination(tmp_path, name, link):
    with pytest.raises(ValueError):
        extract(source_tar(name, link), tmp_path/'source')
    assert not (tmp_path/'escape').exists()


def test_source_tar_internal_symlink_is_supported(tmp_path):
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode='w:gz') as source:
        member = tarfile.TarInfo('root/file.txt')
        member.size = 4
        source.addfile(member, io.BytesIO(b'test'))
        link = tarfile.TarInfo('root/dir/link')
        link.type = tarfile.SYMTYPE
        link.linkname = '../file.txt'
        source.addfile(link)
    extract(buffer.getvalue(), tmp_path/'source')
    assert (tmp_path/'source/dir/link').read_bytes() == b'test'


def source_lock(tmp_path, archive):
    lock = dict(schema_version=1, upstream_commit=PIN, sources={
        'SchurVINS': dict(commit=PIN, sha256=hashlib.sha256(archive).hexdigest(),
                         archive_url='https://invalid.example/test-fixture')})
    file = tmp_path/'lock.json'
    file.write_text(json.dumps(lock))
    workspace = tmp_path/'ws'
    cache = workspace/'downloads'
    cache.mkdir(parents=True)
    (cache/('SchurVINS-'+PIN+'.tar.gz')).write_bytes(archive)
    return file, workspace


def test_cached_source_is_verified_before_extraction(tmp_path):
    lock, workspace = source_lock(tmp_path, source_tar())
    archive = next((workspace/'downloads').iterdir())
    archive.write_bytes(b'tampered fixture')
    with pytest.raises(ValueError, match='cached checksum mismatch'):
        prepare(lock, workspace)
    assert not (workspace/'src').exists()


def test_preparation_is_idempotent_and_refuses_unowned_source(tmp_path):
    lock, workspace = source_lock(tmp_path, source_tar())
    first = prepare(lock, workspace)
    assert prepare(lock, workspace) == first
    assert (workspace/'src/SchurVINS/file.txt').read_bytes() == b'test'
    (workspace/'extracted-SchurVINS.json').unlink()
    with pytest.raises(ValueError, match='unowned source'):
        prepare(lock, workspace)


@pytest.mark.parametrize('group', ['sources', 'external_projects'])
def test_lock_names_cannot_write_outside_workspace(tmp_path, group):
    lock, workspace = source_lock(tmp_path, source_tar())
    value = json.loads(lock.read_text())
    value.setdefault(group, {})['../../escape'] = dict(
        commit=PIN, sha256='0'*64, archive_url='https://invalid.example/test-fixture')
    lock.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='unsafe source lock name'):
        prepare(lock, workspace)
    assert not (workspace/'src').exists()
    assert not (tmp_path/'escape').exists()


@pytest.mark.parametrize('group,name', [('sources', 'catkin_simple'),
                                        ('external_projects', 'ceres_solver')])
@pytest.mark.parametrize('field,value', [('commit', '../../escape'),
                                        ('commit', None),
                                        ('sha256', 'not-a-sha256'),
                                        ('sha256', None)])
def test_all_lock_entries_are_validated_before_materialization(tmp_path, monkeypatch,
                                                              group, name, field, value):
    lock, workspace = source_lock(tmp_path, source_tar())
    contents = json.loads(lock.read_text())
    item = dict(commit=PIN, sha256='0'*64,
                archive_url='https://invalid.example/test-fixture')
    item[field] = value
    contents.setdefault(group, {})[name] = item
    lock.write_text(json.dumps(contents))
    def unexpected_download(url):
        pytest.fail('invalid source lock must be rejected before downloading')
    monkeypatch.setattr('tools.schurvins.prepare.download', unexpected_download)
    with pytest.raises(ValueError, match='invalid lock entry '+name):
        prepare(lock, workspace)
    assert not (workspace/'src').exists()
    assert not (workspace/'extracted-SchurVINS.json').exists()


@pytest.mark.parametrize('source_lock_value', [None, {'schema_version': 1,
                          'upstream_commit': '0'*40, 'sources': {}}])
def test_build_rejects_unprepared_sources_before_mutation(tmp_path, source_lock_value):
    workspace = tmp_path/'workspace'
    (workspace/'src/SchurVINS').mkdir(parents=True)
    if source_lock_value is not None:
        (workspace/'source-lock.json').write_text(json.dumps(source_lock_value))
    with pytest.raises(ValueError, match='source.lock|source lock'):
        build(workspace, 'unused-test-image', tmp_path/'output')
    assert not (workspace/'platform-patch.sha256').exists()
    assert not (tmp_path/'output').exists()


def test_build_refuses_repository_as_workspace(tmp_path):
    from pathlib import Path
    repo = Path(__file__).resolve().parents[1]
    with pytest.raises(ValueError, match='outside the repository'):
        build(repo/'uncreated-workspace', 'unused-test-image', tmp_path/'output')


@pytest.mark.parametrize('architecture,selected,required,forbidden', [
    ('aarch64', 'faster_corner_9_neon.cpp', '-march=armv8-a+simd', '-mfpu='),
    ('arm64', 'faster_corner_9_neon.cpp', '-march=armv8-a+simd', '-msse'),
    ('armv7l', 'faster_corner_9_neon.cpp', '-mfpu=neon', '-msse'),
    ('x86_64', 'faster_corner_10_sse.cpp', '-mssse3', '-mfpu='),
    ('riscv64', None, None, None),
])
def test_fast_patch_selects_target_isa_and_implementation(tmp_path, architecture,
                                                        selected, required, forbidden):
    """Configure actual patched CMake; no target compiler or C++ build claimed."""
    import os
    from pathlib import Path
    import subprocess
    repo = Path(__file__).resolve().parents[1]
    fixture = repo/'tests/fixtures/schurvins/fast_neon.CMakeLists.txt'
    assert hashlib.sha256(fixture.read_bytes()).hexdigest() == (
        '623f4f8d77df0089fae6ff8f9b132da291fc57f38c0b1f421ece955f32d0dbd4')
    source = tmp_path/'source'
    source.mkdir()
    (source/'CMakeLists.txt').write_bytes(fixture.read_bytes())
    subprocess.run(['patch', '-p1', '-i', str(repo/'tools/schurvins/aarch64-fast.patch')],
                   cwd=source, check=True, capture_output=True, text=True)
    # Disable project compiler probes: this checks real CMake selection only.
    cmake = source/'CMakeLists.txt'
    cmake.write_text(cmake.read_text().replace('project(fast)', 'project(fast NONE)', 1))
    stub = tmp_path/'catkin_simple'
    stub.mkdir()
    (stub/'catkin_simpleConfig.cmake').write_text('''
function(catkin_simple)
endfunction()
function(cs_add_library)
  file(WRITE "${CMAKE_BINARY_DIR}/selection.txt" "${CMAKE_CXX_FLAGS}\n${ARGN}")
endfunction()
function(cs_install)
endfunction()
function(cs_export)
endfunction()
''')
    toolchain = tmp_path/'toolchain.cmake'
    toolchain.write_text('set(CMAKE_SYSTEM_NAME Linux)\n'
                         'set(CMAKE_SYSTEM_PROCESSOR '+architecture+')\n')
    build_dir = tmp_path/'configured'
    environment = dict(os.environ, ARM_ARCHITECTURE='legacy-host-setting')
    result = subprocess.run(['cmake', '-S', str(source), '-B', str(build_dir),
                             '-DCMAKE_TOOLCHAIN_FILE='+str(toolchain),
                             '-Dcatkin_simple_DIR='+str(stub)],
                            env=environment, capture_output=True, text=True)
    if selected is None:
        assert result.returncode != 0
        assert 'Unsupported FAST target processor: '+architecture in result.stderr
        assert not (build_dir/'selection.txt').exists()
        return
    assert result.returncode == 0, result.stderr
    selection = (build_dir/'selection.txt').read_text()
    assert selected in selection and required in selection and forbidden not in selection
    other = 'faster_corner_10_sse.cpp' if 'neon' in selected else 'faster_corner_9_neon.cpp'
    assert other not in selection
