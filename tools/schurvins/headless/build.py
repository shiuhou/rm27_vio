"""Copy pinned source into an independent build; never change a source workspace."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(workspace, output):
    workspace, output = Path(workspace).resolve(), Path(output).resolve()
    commit = subprocess.check_output(['git', '--no-optional-locks', 'rev-parse', 'HEAD'],
                                     cwd=workspace / 'src' / 'SchurVINS', text=True).strip()
    if commit != 'd8ab6dff20860694d8d879d716d7fd6cb2c2530e':
        raise ValueError('workspace is not the pinned SchurVINS commit')
    if output.exists():
        raise ValueError('output must be new')
    output.mkdir(parents=True)
    source = output / 'src'
    source.mkdir()
    for name in ('SchurVINS', 'minkindr', 'eigen_checks', 'opengv', 'fast_neon'):
        shutil.copytree(workspace / 'src' / name, source / name, symlinks=True,
                        ignore=shutil.ignore_patterns('.git', '__pycache__'))
    patches = []
    # Diagnostics do not modify filter state. ARM64 patches only affect ISA handling.
    patch_root = Path(__file__).resolve().parents[1]
    for name, cwd in [('native-diagnostics.patch', source / 'SchurVINS'),
                      ('aarch64-neon.patch', source / 'SchurVINS'),
                      ('aarch64-fast-neon.patch', source / 'fast_neon')]:
        patch = patch_root / name
        check = subprocess.run(['git', 'apply', '--check', str(patch)], cwd=cwd,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if check.returncode == 0:
            subprocess.run(['git', 'apply', str(patch)], cwd=cwd, check=True)
            patches.append({'name': name, 'sha256': digest(patch)})
        else:
            reverse = subprocess.run(['git', 'apply', '--reverse', '--check', str(patch)], cwd=cwd,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if reverse.returncode:
                raise RuntimeError('patch is neither applicable nor already applied: ' + name)
    header = source / 'SchurVINS/svo/include/svo/frame_handler_base.h'
    text = header.read_text(encoding='utf-8-sig')
    if '#include <vikit/params_helper.h>' not in text:
        raise RuntimeError('expected unused ROS include not found')
    header.write_text(text.replace('#include <vikit/params_helper.h>', ''))
    manifest = {'source_workspace': str(workspace), 'commit': commit,
                'scope': 'ROS-free boundary; pinned estimator with native diagnostics',
                'patches': patches, 'boundary_change': 'remove unused vikit/params_helper.h include',
                'source_files_sha256': {str(p.relative_to(source)): digest(p) for p in source.rglob('*') if p.is_file() and not p.is_symlink()}}
    (output / 'source.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return source


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--workspace', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    print(prepare(args.workspace, args.output))


if __name__ == '__main__':
    main()
