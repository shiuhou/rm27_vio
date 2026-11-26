"""Bundle refusal paths preserve evidence and target library identity."""
import struct

import pytest

from tools.schurvins.headless.package import closure, package, verify_needed


def elf(path, marker=b''):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = bytearray(64)
    data[:7] = b'\x7fELF\x02\x01\x01'
    struct.pack_into('<H', data, 18, 183)
    path.write_bytes(data + marker)
    return path


def test_unresolved_dependency_refused(tmp_path):
    with pytest.raises(ValueError, match='unresolved dependency'):
        closure(' libfoo.so => not found\n', tmp_path, tmp_path)


def test_recorded_dependency_file_must_exist(tmp_path):
    with pytest.raises(ValueError, match='missing dependency file'):
        closure(' libfoo.so => /sysroot/lib/missing.so (0x10)\n', tmp_path, tmp_path)


def test_soname_collision_refused(tmp_path):
    root = tmp_path / 'target'
    elf(root / 'lib/one.so', b'one')
    elf(root / 'lib/two.so', b'two')
    report = (' libfoo.so => /sysroot/lib/one.so (0x10)\n'
              ' libfoo.so => /sysroot/lib/two.so (0x20)\n')
    with pytest.raises(ValueError, match='soname collision'):
        closure(report, root, tmp_path)


def test_target_absolute_symlink_never_reads_host(tmp_path):
    root = tmp_path / 'target'
    real = elf(root / 'lib/target-loader.so')
    (root / 'lib/loader.so').symlink_to('/lib/target-loader.so')
    result = closure(' /lib/ld-linux-aarch64.so.1 => /sysroot/lib/loader.so (0x10)\n', root, tmp_path)
    assert result['ld-linux-aarch64.so.1']['source'] == str(real)


def test_wrong_architecture_refused(tmp_path):
    path = elf(tmp_path / 'target/lib/loader.so')
    data = bytearray(path.read_bytes())
    struct.pack_into('<H', data, 18, 62)
    path.write_bytes(data)
    with pytest.raises(ValueError, match='not AArch64 ELF'):
        closure(' /lib/ld-linux-aarch64.so.1 => /sysroot/lib/loader.so (0x10)\n', tmp_path / 'target', tmp_path)


def test_transitive_needed_absence_refused(monkeypatch, tmp_path):
    monkeypatch.setattr('tools.schurvins.headless.package.subprocess.check_output',
                        lambda *args, **kwargs: ' 0x00000001 (NEEDED) Shared library: [absent.so]\n')
    with pytest.raises(ValueError, match='missing DT_NEEDED absent.so'):
        verify_needed(tmp_path / 'binary', {})


def test_existing_output_preserved_before_reading_inputs(tmp_path):
    output = tmp_path / 'bundle'
    output.mkdir()
    sentinel = output / 'manifest.json'
    sentinel.write_text('original evidence\n')
    with pytest.raises(ValueError, match='output must be new'):
        package(binary=tmp_path/'missing', ldd=tmp_path/'missing', sysroot=tmp_path,
                opengv_dir=tmp_path, source=tmp_path, build_manifest=tmp_path/'missing', output=output)
    assert sentinel.read_text() == 'original evidence\n'
