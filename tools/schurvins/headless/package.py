"""Package an audited AArch64 dependency closure, without touching the target.

The resulting local research bundle uses a Bionic reference loader. It is not
evidence of compatibility with the unidentified M3C or permission to redistribute
the upstream dependency set.
"""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess

from tools.schurvins.arm64 import disk_guard, elf_header, guest_file
from tools.schurvins.headless.build import digest


def closure(report, sysroot, opengv_dir):
    """Resolve the recorded target ldd output; never resolve a guest link on host."""
    libraries = {}
    for line in report.splitlines():
        if not line.strip():
            continue
        if 'not found' in line:
            raise ValueError('unresolved dependency: ' + line.strip())
        match = re.fullmatch(r'\s*(\S+)\s+=>\s+(\S+)\s+\(0x[0-9a-fA-F]+\)\s*', line)
        if not match:
            raise ValueError('unrecognized target dependency: ' + line.strip())
        name, guest = match.groups()
        name = Path(name).name
        if name in ('', '.', '..') or '/' in name:
            raise ValueError('invalid dependency name')
        if guest.startswith('/sysroot/'):
            path = guest_file(sysroot, guest[len('/sysroot'):])
        elif guest.startswith('/armws/devel/.private/opengv/lib/'):
            path = Path(opengv_dir).resolve() / Path(guest).name
        else:
            raise ValueError('dependency outside declared target roots: ' + guest)
        if not path.is_file():
            raise ValueError('missing dependency file: ' + str(path))
        elf_header(path)
        entry = dict(source=str(path), target_ldd_path=guest, sha256=digest(path))
        if name in libraries and libraries[name]['sha256'] != entry['sha256']:
            raise ValueError('dependency soname collision: ' + name)
        libraries[name] = entry
    if 'ld-linux-aarch64.so.1' not in libraries:
        raise ValueError('AArch64 loader absent from dependency closure')
    return libraries


def verify_needed(binary, libraries):
    """Check every DT_NEEDED, including transitive dependencies, against the set."""
    for path in [Path(binary)] + [Path(item['source']) for item in libraries.values()]:
        output = subprocess.check_output(['readelf', '-d', str(path)], text=True)
        for name in re.findall(r'\(NEEDED\).*?\[([^\]]+)\]', output):
            if name not in libraries:
                raise ValueError('dependency closure missing DT_NEEDED ' + name + ' for ' + str(path))


def notices(sysroot, libraries, source):
    """Preserve package copyright files and source license/package declarations."""
    sysroot, source = Path(sysroot).resolve(), Path(source).resolve()
    paths = {Path(row['source']).as_posix(): name for name, row in libraries.items()}
    records, owners = {}, {}
    for listing in sorted((sysroot / 'var/lib/dpkg/info').glob('*.list')):
        package = listing.name[:-5]
        for line in listing.read_text(errors='replace').splitlines():
            if not line.startswith('/'):
                continue
            candidate = sysroot / line.lstrip('/')
            # Lexical file entries and resolved entries may both name the same ELF.
            if candidate.as_posix() in paths:
                owners[paths[candidate.as_posix()]] = package
    for name, row in libraries.items():
        path = Path(row['source'])
        if sysroot in path.parents:
            package = owners.get(name)
            if package is None:
                raise ValueError('target package owner not found for ' + name)
            copyright_path = guest_file(sysroot, '/usr/share/doc/' + package.split(':')[0] + '/copyright')
            if not copyright_path.is_file():
                raise ValueError('target copyright notice absent for ' + package)
            records['licenses/debian/' + package.replace(':', '_') + '/copyright'] = copyright_path
    for item in sorted((sysroot / 'usr/share/common-licenses').iterdir()):
        if item.is_file():
            records['licenses/common/' + item.name] = item
    # Some pinned packages provide only package.xml declarations, not LICENSE.
    # Preserve these verbatim; this does not certify the declared license terms.
    for item in sorted(source.rglob('*')):
        if item.is_file() and (item.name.lower() in ('license', 'license.txt', 'copying', 'copyright')
                               or item.name == 'package.xml'):
            records['licenses/source/' + item.relative_to(source).as_posix()] = item
    eigen_notice = source / 'eigen_checks/include/eigen-checks/gtest.h'
    if eigen_notice.is_file():
        records['licenses/source/eigen_checks/gtest-header-notice.h'] = eigen_notice
    return records, owners


def package(*, binary, ldd, sysroot, opengv_dir, source, build_manifest, output):
    output = Path(output).absolute()
    if output.exists() or output.is_symlink():
        raise ValueError('output must be new')
    binary, ldd, source, build_manifest = map(Path, (binary, ldd, source, build_manifest))
    elf_header(binary)
    libraries = closure(ldd.read_text(), sysroot, opengv_dir)
    verify_needed(binary, libraries)
    license_files, owners = notices(sysroot, libraries, source)
    inputs = {'bin/schurvins_headless': binary, 'provenance/ldd-arm64.txt': ldd,
              'provenance/build-manifest.json': build_manifest, **license_files}
    params = source / 'SchurVINS/svo_ros/param'
    for mode in ('mono', 'stereo'):
        inputs['params/vio_' + mode + '.yaml'] = params / ('vio_' + mode + '.yaml')
        inputs['params/calib/euroc_' + mode + '.yaml'] = params / 'calib' / ('euroc_' + mode + '.yaml')
    for name, row in libraries.items():
        inputs['lib/' + name] = Path(row['source'])
    # Preflight every input before creating output; failures never look complete.
    expected_bytes = 0
    for item in inputs.values():
        if not item.is_file():
            raise ValueError('missing bundle input: ' + str(item))
        expected_bytes += item.stat().st_size
    disk_guard(output, expected_bytes)
    output.mkdir(parents=True)
    file_records = {}
    for name, item in inputs.items():
        before = digest(item)
        target = output / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(item, target)
        if digest(item) != before or digest(target) != before:
            raise RuntimeError('input changed during packaging: ' + str(item))
        file_records[name] = dict(source=str(item.resolve()), sha256=before, bytes=target.stat().st_size)
    (output / 'bin/schurvins_headless').chmod(0o755)
    (output / 'lib/ld-linux-aarch64.so.1').chmod(0o755)
    wrapper = output / 'run'
    wrapper.write_text('#!/bin/sh\nset -eu\nbundle_root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)\n'
                       'exec "$bundle_root/lib/ld-linux-aarch64.so.1" --inhibit-cache '
                       '--library-path "$bundle_root/lib" "$bundle_root/bin/schurvins_headless" "$@"\n')
    wrapper.chmod(0o755)
    (output / 'README.txt').write_text(
        'LOCAL ARM64 REFERENCE RESEARCH BUNDLE\n'
        'Bionic glibc reference ABI; M3C identity/ABI/OS and actual execution remain BLOCKED.\n'
        'No ROS runtime, device driver, camera SDK, UART, system installation or firmware changes.\n'
        './run --help\n'
        './run --dataset ASL_ROOT --mode stereo --calib params/calib/euroc_stereo.yaml '
        '--params params/vio_stereo.yaml --output NEW_OUTPUT --rate 0.5\n'
        'The dataset/output and relative parameter paths are relative to the current directory.\n'
        'QEMU smoke results are not board performance or estimator qualification.\n'
        'The frozen build source tree and build instructions are referenced in manifest.json.\n'
        'Copyright notices and package declarations are preserved verbatim. Distribution rights, '
        'including fast_neon RPG declarations, have not been assessed. This is a local test bundle.\n')
    for name in ('run', 'README.txt'):
        item = output / name
        file_records[name] = dict(sha256=digest(item), bytes=item.stat().st_size)
    result = dict(schema_version=1, status='PASS', scope='LOCAL_ARM64_REFERENCE_BUNDLE_ONLY',
                  abi='Bionic reference glibc; actual M3C ABI UNKNOWN',
                  target_execution='NOT_RUN', board_qualification='BLOCKED',
                  source_tree=str(source.resolve()), binary_sha256=digest(binary),
                  generator_sha256=digest(Path(__file__)),
                  ldd_sha256=digest(ldd), dependency_count=len(libraries),
                  dependency_package_owners=owners, files=file_records,
                  total_bytes=sum(item['bytes'] for item in file_records.values()),
                  redistribution_assessment='NOT_RUN')
    (output / 'manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('binary', 'ldd', 'sysroot', 'opengv-dir', 'source', 'build-manifest', 'output'):
        parser.add_argument('--' + name, required=True)
    result = package(**vars(parser.parse_args()))
    print(json.dumps({key: result[key] for key in ('status', 'scope', 'dependency_count', 'total_bytes')}))


if __name__ == '__main__':
    main()
