"""Prepare immutable pinned OpenVINS source with observation-only counters.

No historical source or research directory is modified. Compilation is separate
so the identical prepared source can serve native and cross builds.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

PIN = '69488123ed9362dd44b6f28e7f4680abbff1442b'
ARCHIVE_SHA256 = '283dde4096d9380956f6a9c8ed7e54e2d1303ee64af002d9a97f9b2196e677f4'


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def verify_source(archive: Path, source_hashes: dict[str, str]) -> dict:
    archive_hash = digest(archive)
    if archive_hash != ARCHIVE_SHA256:
        raise ValueError('archive SHA-256 does not match pinned OpenVINS archive')
    remaining = set(source_hashes)
    found = set()
    with tarfile.open(archive, mode='r|gz') as members:
        for member in members:
            relative = '/'.join(Path(member.name).parts[1:])
            if relative not in remaining:
                continue
            if not member.isfile():
                raise ValueError(f'pinned source member is not a regular file: {relative}')
            actual = hashlib.sha256(members.extractfile(member).read()).hexdigest()
            if actual != source_hashes[relative]:
                raise ValueError(f'source differs from pinned archive: {relative}')
            remaining.remove(relative)
            found.add(relative)
    if remaining:
        raise ValueError(f'source paths absent from pinned archive: {sorted(remaining)}')
    return dict(archive_sha256=archive_hash, matching_files=len(found))


def prepare(source: Path, output: Path, archive: Path) -> dict:
    if output.exists():
        raise FileExistsError(f'output already exists: {output}')
    paths = [p for name in ('ov_core', 'ov_init', 'ov_msckf', 'config')
             for p in (source / name).rglob('*') if p.is_file()]
    for name in ('LICENSE',):
        paths.append(source / name)
    before = {str(p.relative_to(source)): digest(p) for p in paths}
    verification = verify_source(archive, before)
    output.mkdir(parents=True)
    for p in paths:
        destination = output / p.relative_to(source)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, destination)
    changes = []
    for updater, label in [('UpdaterMSCKF', 'msckf'), ('UpdaterSLAM', 'slam')]:
        p = output / f'ov_msckf/src/update/{updater}.cpp'
        text = p.read_text()
        anchor = '  StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);'
        if text.count(anchor) != 1:
            raise ValueError(f'unexpected upstream update boundary: {p}')
        text = '#include "audit.h"\n' + text.replace(anchor, anchor + f'''
  ++rm27_openvins::audit.{label}_updates;
  rm27_openvins::audit.{label}_features += feature_vec.size();
  rm27_openvins::audit.{label}_rows += Hx_big.rows();''')
        p.write_text(text)
        changes.append(str(p.relative_to(output)))
    config = output / 'config/euroc_mav/estimator_config.yaml'
    text = config.read_text()
    for old, new in [('use_stereo: true', 'use_stereo: false'), ('max_cameras: 2', 'max_cameras: 1')]:
        if text.count(old) != 1:
            raise ValueError(f'unexpected EuRoC config: {old}')
        text = text.replace(old, new)
    # Native headless delivery is synchronous; static upstream initialization
    # remains unchanged and uses the existing IMU/camera calibration and noise.
    text += '\nmulti_threading_subs: false\nmulti_threading_pubs: false\n'
    config.write_text(text)
    changes.append(str(config.relative_to(output)))
    after = {rel: digest(output / rel) for rel in before}
    if {rel: digest(source / rel) for rel in before} != before:
        raise RuntimeError('source changed during preparation')
    manifest = dict(schema_version=1, upstream_commit=PIN,
                    upstream_archive_sha256=ARCHIVE_SHA256,
                    input_source=str(source.resolve()), source_files_sha256=before,
                    archive_verification=verification,
                    prepared_files_sha256=after, explicit_changed_files=changes,
                    variant='pinned OpenVINS + observation-only accepted-update counters + mono config')
    (output / 'preparation.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = prepare(args.source, args.output, args.archive)
    print(json.dumps(dict(upstream_commit=PIN, files=len(result['source_files_sha256']),
                          changed_files=result['explicit_changed_files'])))


if __name__ == '__main__':
    main()
