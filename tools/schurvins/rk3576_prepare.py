"""Assemble a native ARM64 diagnostic candidate without Docker or board access."""
import argparse
import copy
import gzip
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import zipfile

import cv2
import numpy as np

from tools.schurvins.arm64 import external, disk_guard
from tools.schurvins.ros_transport import inputs as sensor_rows
from rm27.perception.vision.localization.experiments.common import file_hash, write_json


def safe_name(name):
    path = PurePosixPath(name)
    if path.is_absolute() or '..' in path.parts or str(path) != name or name == '.':
        raise ValueError('unsafe archive member: ' + name)
    return name


def resolve_file(name, files, links):
    """Resolve target absolute/relative symlinks in the image, never on the host."""
    for _ in range(40):
        parts = PurePosixPath(name).parts
        for i in range(1, len(parts) + 1):
            prefix = '/'.join(parts[:i])
            if prefix in links:
                target = PurePosixPath(links[prefix])
                combined = (target if target.is_absolute() else PurePosixPath(prefix).parent/target)
                normalized = []
                for part in (*combined.parts, *parts[i:]):
                    if part in ('/', '.', ''):
                        continue
                    if part == '..':
                        if not normalized:
                            raise ValueError('symlink escapes image')
                        normalized.pop()
                    else:
                        normalized.append(part)
                name = '/'.join(normalized)
                break
        else:
            return files.get(name)
    raise ValueError('symlink cycle or excessive depth: ' + name)


def verify_workspace(files, links, installation):
    for tree, key in [('src', 'source'), ('devel', 'devel')]:
        prefix = 'ws/' + tree + '/'
        wanted_links = {prefix+k: v for k, v in installation[key+'_symlinks'].items()}
        actual_links = {k: v for k, v in links.items() if k.startswith(prefix)}
        if actual_links != wanted_links:
            raise ValueError('workspace symlink membership/target mismatch: ' + tree)
        wanted_files = {prefix+k: v for k, v in installation[key+'_files_sha256'].items()}
        actual_files = {k: v for k, v in files.items() if k.startswith(prefix)}
        # Catkin manifests include hashes of resolvable file symlink aliases.
        if actual_files != {k: v for k, v in wanted_files.items() if k not in wanted_links}:
            raise ValueError('workspace regular file membership/hash mismatch: ' + tree)
        if any(resolve_file(k, files, links) != v for k, v in wanted_files.items()):
            raise ValueError('workspace file alias hash mismatch: ' + tree)


def verify_image_archive(archive, expected_files, expected_links):
    """Reopen the emitted tar: PAX headers can override in-memory TarInfo paths."""
    files, links, seen = {}, {}, set()
    with tarfile.open(archive, 'r|gz') as source:
        for member in source:
            name = safe_name(member.name)
            if name in seen:
                raise ValueError('duplicate emitted archive member')
            seen.add(name)
            if member.isfile():
                digest = hashlib.sha256()
                payload = source.extractfile(member)
                for chunk in iter(lambda: payload.read(1024*1024), b''):
                    digest.update(chunk)
                files[name] = digest.hexdigest()
            elif member.islnk():
                if member.linkname not in files:
                    raise ValueError('emitted archive hardlink target missing')
                files[name] = files[member.linkname]
            elif member.issym():
                links[name] = str(PurePosixPath(member.linkname))
            elif not member.isdir():
                raise ValueError('unsupported emitted archive member kind')
    if files != expected_files or links != expected_links:
        raise ValueError('emitted archive membership/hash/link mismatch')


def flatten(base_archive, overlay_archive, overlay_package, base, installation, output):
    """Stream the sealed archives into one image root, checking the whole closure."""
    replacements = set(overlay_package['replaced_or_added_paths'])
    removed = set(overlay_package['removed_paths'])
    if replacements & removed:
        raise ValueError('overlay both removes and replaces a member')
    for name in replacements | removed:
        safe_name(name)
        if not name.startswith(('ws/src/', 'ws/devel/')):
            raise ValueError('overlay outside workspace')
    files, links, seen, base_files, base_links = {}, {}, set(), {}, {}
    elf_headers = {}
    metadata = {}
    raw_bytes = 0

    with output.open('xb') as raw, gzip.GzipFile(fileobj=raw, mode='wb', filename='', mtime=0, compresslevel=1) as compressed:
        with tarfile.open(fileobj=compressed, mode='w|') as target:
            def emit(source, member, name):
                nonlocal raw_bytes
                safe_name(name)
                if name in seen:
                    raise ValueError('duplicate image member: ' + name)
                if any(str(p) in links for p in PurePosixPath(name).parents):
                    raise ValueError('archive member beneath a symlink: ' + name)
                seen.add(name)
                info = copy.copy(member)
                # Old PAX path/linkpath values override renamed TarInfo fields
                # when the written archive is reopened. Regenerate them.
                info.pax_headers = {}
                info.name = name
                info.uid = info.gid = 0
                info.uname = info.gname = ''
                info.mtime = 0
                if member.isfile():
                    # The tar stream copies chunks, with only this member in memory.
                    # Individual native libraries are small; rootfs is never extracted.
                    content = source.extractfile(member).read()
                    files[name] = hashlib.sha256(content).hexdigest()
                    if content.startswith(b'\x7fELF'):
                        elf_headers[name] = content[:64].hex()
                    raw_bytes += len(content)
                    import io
                    target.addfile(info, io.BytesIO(content))
                elif member.issym():
                    # Existing installation manifests use str(Path.readlink()),
                    # which canonicalizes a directory link's trailing slash.
                    info.linkname = str(PurePosixPath(member.linkname))
                    links[name] = info.linkname
                    target.addfile(info)
                elif member.islnk():
                    link = safe_name(member.linkname)
                    if link.startswith('rootfs/'):
                        link = link[len('rootfs/'):]
                    if link not in files:
                        raise ValueError('hardlink must target an earlier regular file: ' + link)
                    info.linkname = link
                    files[name] = files[link]
                    if link in elf_headers:
                        elf_headers[name] = elf_headers[link]
                    target.addfile(info)
                elif member.isdir():
                    target.addfile(info)
                else:
                    raise ValueError('unsupported archive member kind: ' + name)

            with tarfile.open(base_archive, 'r|gz') as source:
                base_seen = set()
                for member in source:
                    safe_name(member.name)
                    if member.name in base_seen:
                        raise ValueError('duplicate reference member')
                    base_seen.add(member.name)
                    if member.name == 'rootfs':
                        continue
                    if member.name.startswith('rootfs/'):
                        name = member.name[len('rootfs/'):]
                    elif member.name == 'ws' or member.name.startswith('ws/'):
                        name = member.name
                    else:
                        raise ValueError('unexpected reference member: ' + member.name)
                    if name == 'ws':
                        continue  # rootfs ships an empty /ws mountpoint.
                    if name in replacements or name in removed:
                        if member.isfile():
                            base_files[name] = hashlib.sha256(source.extractfile(member).read()).hexdigest()
                        elif member.issym():
                            base_links[name] = member.linkname
                        else:
                            raise ValueError('overlay replaces non-leaf')
                        continue
                    emit(source, member, name)
            base_files.update(files)
            base_links.update(links)
            verify_workspace(base_files, base_links, base)
            with tarfile.open(overlay_archive, 'r|gz') as source:
                overlay_seen = set()
                for member in source:
                    safe_name(member.name)
                    if member.name in overlay_seen:
                        raise ValueError('duplicate overlay member')
                    overlay_seen.add(member.name)
                    if member.name.startswith('metadata/'):
                        if not member.isfile():
                            raise ValueError('metadata must be regular files')
                        metadata[member.name] = source.extractfile(member).read()
                    elif member.name in replacements:
                        emit(source, member, member.name)
                    else:
                        raise ValueError('undeclared overlay member')
                if overlay_seen - set(metadata) != replacements:
                    raise ValueError('overlay membership incomplete')
    verify_workspace(files, links, installation)
    root_files = {k: v for k, v in files.items() if not k.startswith('ws/')}
    root_links = {k: v for k, v in links.items() if not k.startswith('ws/')}
    if root_files != installation['rootfs']['files_sha256'] or root_links != installation['rootfs']['symlinks']:
        raise ValueError('reference userspace membership/hash mismatch')
    binary = 'ws/' + installation['binary']
    if files.get(binary) != installation['binary_sha256']:
        raise ValueError('native executable identity mismatch')
    header = bytes.fromhex(elf_headers.get(binary, ''))
    if len(header) != 64 or header[:6] != b'\x7fELF\x02\x01' or int.from_bytes(header[18:20], 'little') != 183:
        raise ValueError('native executable is not ELF64 AArch64')
    libraries = installation['elf']['libraries_sha256']
    if any(resolve_file(k.lstrip('/'), files, links) != v for k, v in libraries.items()):
        raise ValueError('target loader library closure mismatch')
    verify_image_archive(output, files, links)
    return dict(regular_files=len(files), symlinks=len(links), uncompressed_file_bytes=raw_bytes,
                loader_libraries=len(libraries), workspace_verified=True, userspace_verified=True,
                aarch64_executable_verified=True, reopened_archive_verified=True), metadata


def validate_sensor_input(inputs, data):
    """Read back the source-only ZIP through the existing CSV reader and ledger."""
    with zipfile.ZipFile(inputs/'sensors.zip') as sensors:
        names = sensors.namelist()
        required = {'mav0/cam0/data.csv', 'mav0/cam0/sensor.yaml',
                    'mav0/imu0/data.csv', 'mav0/imu0/sensor.yaml'}
        if len(set(names)) != len(names):
            raise ValueError('duplicate sensor archive member')
        for name in names:
            safe_name(name)
            path = PurePosixPath(name)
            if name not in required and not (str(path.parent) == 'mav0/cam0/data'
                    and path.suffix == '.png' and path.stem.isdecimal()):
                raise ValueError('unexpected or truth-bearing sensor archive member')
        if not required <= set(names):
            raise ValueError('missing sensor CSV/calibration member')
        streams = sensor_rows(sensors, 'mono')
        images = {'mav0/cam0/data/'+filename for _, filename in streams['cam0']}
        if set(names) != required | images:
            raise ValueError('camera CSV/image membership mismatch')
        ledger = {'cam0': [], 'imu0': []}
        for line in (inputs/'expected.jsonl').read_text().splitlines():
            row = json.loads(line)
            if row.get('stream') not in ledger:
                raise ValueError('unexpected source ledger stream')
            ledger[row['stream']].append(row)
        for sensor, rows in streams.items():
            if (data['counts'].get(sensor) != len(rows) or len(ledger[sensor]) != len(rows)
                    or data['first_ns'].get(sensor) != rows[0][0]
                    or data['last_ns'].get(sensor) != rows[-1][0]):
                raise ValueError('sensor CSV/ledger/manifest count or endpoint mismatch')
            calibration = sensors.read('mav0/'+sensor+'/sensor.yaml')
            if hashlib.sha256(calibration).hexdigest() != data['calibration_sha256'].get(sensor):
                raise ValueError('sensor calibration hash mismatch')
            for (ns, payload), actual in zip(rows, ledger[sensor]):
                expected = dict(stream=sensor, state_ns=ns)
                if sensor == 'imu0':
                    expected.update(gyro=payload[:3], accel=payload[3:])
                else:
                    encoded = sensors.read('mav0/cam0/data/'+payload)
                    pixels = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_UNCHANGED)
                    if pixels is None or pixels.ndim != 2 or pixels.dtype != np.uint8:
                        raise ValueError('mono8 image required')
                    expected.update(pixels_sha256=hashlib.sha256(pixels.tobytes()).hexdigest(),
                        height=pixels.shape[0], width=pixels.shape[1], encoding='mono8', step=pixels.shape[1])
                if actual != expected:
                    raise ValueError('decoded sensor payload/source ledger mismatch')


def prepare(base_package, overlay_package, inputs, output):
    base_package, overlay_package, inputs = map(lambda p: Path(p).resolve(), (base_package, overlay_package, inputs))
    output = external(output)
    if output.exists():
        raise FileExistsError(output)
    if any(output == p or p in output.parents or output in p.parents for p in
           (base_package.parent, overlay_package.parent, inputs)):
        raise ValueError('output overlaps inputs')
    bp, op = [json.loads(p.read_text()) for p in (base_package, overlay_package)]
    ba, oa = base_package.with_name('reference.tar.gz'), overlay_package.with_name('overlay.tar.gz')
    bi = base_package.with_name('installation.json')
    # The derivative manifest and exact patches are inside the checked overlay.
    if bp.get('status') != 'PASS' or op.get('status') != 'PASS' or op.get('roundtrip_verified') is not True:
        raise ValueError('verified ARM64 base and overlay packages required')
    for path, digest in [(ba, bp['archive_sha256']), (bi, bp['installation_sha256']), (oa, op['archive_sha256'])]:
        if file_hash(path) != digest:
            raise ValueError('source package hash mismatch: ' + str(path))
    if op['base_archive_sha256'] != bp['archive_sha256'] or op['base_installation_sha256'] != bp['installation_sha256']:
        raise ValueError('overlay is bound to a different base')
    with tarfile.open(oa) as archive:
        installation_bytes = archive.extractfile('metadata/installation.json').read()
    if hashlib.sha256(installation_bytes).hexdigest() != op['installation_sha256']:
        raise ValueError('derivative installation hash mismatch')
    installation = json.loads(installation_bytes)
    base = json.loads(bi.read_text())
    if (installation.get('architecture') != 'arm64' or installation.get('platform') != 'linux/arm64'
            or installation.get('runtime') != 'proot-qemu' or installation['rootfs'] != base['rootfs']):
        raise ValueError('unsupported source installation')
    input_manifest_sha256 = file_hash(inputs/'input.json')
    data = json.loads((inputs/'input.json').read_text())
    if (data.get('purpose') != 'EUROC_PUBLIC_REPLAY' or data.get('mode') != 'mono'
            or data.get('truth_included') is not False or set(data['counts']) != {'cam0', 'imu0'}
            or set(data['artifacts_sha256']) != {'expected.jsonl', 'sensors.zip'}):
        raise ValueError('sensor-only EuRoC cam0/imu0 input required')
    for name, digest in data['artifacts_sha256'].items():
        if file_hash(inputs/name) != digest:
            raise ValueError('input artifact changed: ' + name)
    validate_sensor_input(inputs, data)
    disk_guard(output, ba.stat().st_size + oa.stat().st_size + (inputs/'sensors.zip').stat().st_size + 128*1024**2)
    output.mkdir(parents=True)
    closure, metadata = flatten(ba, oa, op, base, installation, output/'runtime-rootfs.tar.gz')
    provenance = output/'provenance'
    provenance.mkdir()
    shutil.copyfile(base_package, provenance/'base-package.json')
    shutil.copyfile(overlay_package, provenance/'overlay-package.json')
    shutil.copyfile(bi, provenance/'base-installation.json')
    for name, content in metadata.items():
        rel = safe_name(name[len('metadata/'):])
        destination = provenance/rel
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
    if metadata.get('metadata/base-installation.json') != bi.read_bytes():
        raise ValueError('overlay base metadata mismatch')
    if hashlib.sha256(metadata['metadata/source-lock.json']).hexdigest() != installation['source_lock_sha256']:
        raise ValueError('source lock mismatch')
    for name, digest in installation['patches'].items():
        if hashlib.sha256(metadata['metadata/patches/'+name]).hexdigest() != digest:
            raise ValueError('patch identity mismatch: ' + name)
    (output/'input').mkdir()
    for name in ['input.json', *data['artifacts_sha256']]:
        shutil.copyfile(inputs/name, output/'input'/name)
    for name, expected in {'input.json': input_manifest_sha256, **data['artifacts_sha256']}.items():
        if file_hash(output/'input'/name) != expected or file_hash(inputs/name) != expected:
            raise ValueError('input changed during candidate assembly: ' + name)
    (output/'tools').mkdir()
    for name in ['run_inner.py', 'ros_transport.py']:
        shutil.copyfile(Path(__file__).with_name(name), output/'tools'/name)
    shutil.copyfile(Path(__file__).with_name('rk3576_board.py'), output/'rk3576_board.py')
    shutil.copyfile(Path(__file__), provenance/'rk3576_prepare.py')
    digest = file_hash(output/'runtime-rootfs.tar.gz')
    (output/'Dockerfile').write_text('FROM scratch\nADD runtime-rootfs.tar.gz /\n'
        'ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8\n'
        'LABEL org.rm27.schur.payload="'+digest+'"\n')
    (output/'.dockerignore').write_text('*\n!Dockerfile\n!runtime-rootfs.tar.gz\n')
    write_json(output/'candidate.json', dict(schema_version=1, status='PREPARED_NOT_RUN',
        platform='linux/arm64', runtime='native-docker', backend='SchurVINS',
        runtime_archive_sha256=digest, source_commit=installation['commit'],
        installation_sha256=op['installation_sha256'], source_package_hashes=dict(base=file_hash(base_package),overlay=file_hash(overlay_package)),
        profile=dict(memory_mib=2560,cpus=4,timeout_s=900,mode='mono',rate=0.5),
        input=data, archive_validation=closure,
        files_sha256={str(p.relative_to(output)): file_hash(p) for p in sorted(output.rglob('*')) if p.is_file()},
        evidence=dict(native_image_build='NOT_RUN',board_startup='NOT_RUN',mono_trajectory='NOT_RUN',
            physical_camera_imu='NOT_RUN',accuracy='NOT_RUN',real_time='NOT_RUN'),
        limitations=['Native image uses pinned Bionic/Melodic userspace; host BSP/kernel compatibility remains untested.',
            'PC mono 0.5x produced zero poses; 1x produced one. ARM64 packaging is not a mono fix.',
            'Raw diagnostic only; existing cohort/scoring/1cm gates remain unchanged.']))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['base-package', 'overlay-package', 'input', 'output']:
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.base_package,args.overlay_package,args.input,args.output))


if __name__ == '__main__':
    main()
