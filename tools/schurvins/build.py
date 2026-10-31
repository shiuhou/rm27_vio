"""Build a prepared external workspace and freeze executable/source provenance."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from rm27.perception.vision.localization.experiments.common import file_hash, write_json
from tools.schurvins.prepare import PIN


def snapshot(root):
    root = Path(root)
    return {str(p.relative_to(root)):file_hash(p) for p in sorted(root.rglob('*'))
            if p.is_file() and '.git' not in p.parts and '__pycache__' not in p.parts}


def symlinks(root):
    root = Path(root)
    return {str(p.relative_to(root)):str(p.readlink()) for p in sorted(root.rglob('*'))
            if p.is_symlink() and '.git' not in p.parts and '__pycache__' not in p.parts}


def build(workspace, image, output, extra_patches=()):
    workspace, output = Path(workspace).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    repo = Path(__file__).resolve().parents[2]
    if workspace == repo or repo in workspace.parents:
        raise ValueError('workspace must be outside the repository')
    if not (workspace/'src/SchurVINS').is_dir():
        raise ValueError('prepare.py must materialize workspace first')
    provenance = workspace/'source-lock.json'
    if not provenance.is_file():
        raise ValueError('prepare.py source-lock.json is required before building')
    lock = json.loads(provenance.read_text())
    if (lock.get('schema_version') != 1 or lock.get('upstream_commit') != PIN
            or lock.get('sources', {}).get('SchurVINS', {}).get('commit') != PIN):
        raise ValueError('unsupported source lock')
    source = workspace/'src/SchurVINS'
    patch = Path(__file__).with_name('platform.patch').resolve()
    stamp = workspace/'platform-patch.sha256'
    if stamp.exists():
        if stamp.read_text().strip() != file_hash(patch):
            raise ValueError('workspace has a different platform patch')
    else:
        subprocess.run(['patch','--dry-run','-p1','-i',str(patch)],cwd=source,check=True)
        subprocess.run(['patch','-p1','-i',str(patch)],cwd=source,check=True)
        stamp.write_text(file_hash(patch)+'\n')
    applied_patches = {patch.name: file_hash(patch)}
    for extra in extra_patches:
        extra = Path(extra).resolve()
        if not extra.is_file() or extra.name in applied_patches:
            raise ValueError('extra patches must be existing, uniquely named files')
        extra_stamp = workspace/(extra.name+'.sha256')
        if extra_stamp.exists():
            if extra_stamp.read_text().strip() != file_hash(extra):
                raise ValueError('workspace has a different '+extra.name)
        else:
            subprocess.run(['patch','--dry-run','-p1','-i',str(extra)],cwd=source,check=True)
            subprocess.run(['patch','-p1','-i',str(extra)],cwd=source,check=True)
            extra_stamp.write_text(file_hash(extra)+'\n')
        applied_patches[extra.name] = file_hash(extra)
    for name in ['svo_global_map','svo_online_loopclosing','svo_pgo','rqt_svo','svo_benchmarking']:
        (source/name/'CATKIN_IGNORE').touch()
    image_id = subprocess.check_output(['docker','image','inspect',image,'--format','{{.Id}}'],text=True).strip()
    output.mkdir(parents=True)
    # Keep the exact applied bytes beside each installation. Later development
    # of the repository patch must not destroy reproduction of an older run.
    (output/'patches').mkdir()
    for applied in [patch, *(Path(p).resolve() for p in extra_patches)]:
        shutil.copyfile(applied, output/'patches'/applied.name)
        if file_hash(output/'patches'/applied.name) != applied_patches[applied.name]:
            raise ValueError('patch changed while freezing build provenance')
    command = ['docker','run','--rm','--network','none','--cpus','4','--memory','12g',
               '--user',str(__import__('os').getuid())+':'+str(__import__('os').getgid()),
               '-v',str(workspace)+':/ws', image_id, 'bash','-c',
               'source /opt/ros/melodic/setup.bash && cd /ws && '
               'catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release '
               '-DCATKIN_ENABLE_TESTING=ON -DUSE_SYSTEM_EIGEN=ON -DUSE_SYSTEM_GLOG=ON '
               '&& catkin build svo_ros --no-status -j4 -p2']
    write_json(output/'command.json',command)
    with (output/'build.log').open('x') as log:
        result = subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
    write_json(output/'result.json',dict(exit_code=result.returncode,image_id=image_id))
    if result.returncode:
        return result.returncode
    # Linked catkin devel aliases target /ws inside the container. Hash the real
    # artifact so host verification does not follow that container-only path.
    binary_relative = 'devel/.private/svo_ros/lib/svo_ros/svo_node'
    binary = workspace/binary_relative
    if not binary.is_file():
        raise ValueError('svo_node missing after build')
    packages = subprocess.check_output(['docker','run','--rm','--network','none',image_id,
                                        'cat','/opt/schurvins-packages.txt'],text=True)
    (output/'packages.txt').write_text(packages)
    manifest = dict(schema_version=1, backend='SchurVINS',
                    commit=PIN,image_id=image_id,
                    workspace=str(workspace),binary=binary_relative,
                    binary_sha256=file_hash(binary),source_files_sha256=snapshot(workspace/'src'),
                    devel_files_sha256=snapshot(workspace/'devel'),
                    source_symlinks=symlinks(workspace/'src'),
                    devel_symlinks=symlinks(workspace/'devel'),
                    system_packages_sha256=file_hash(output/'packages.txt'),
                    build_command_sha256=file_hash(output/'command.json'),
                    source_lock_sha256=file_hash(provenance),
                    dockerfile_sha256=file_hash(Path(__file__).with_name('Dockerfile')),
                    patches=applied_patches,
                    variant='RM27 derivative' if extra_patches else 'upstream with platform patch',
                    scope='x86_64 PC build; not ARM64/RK3576 qualification')
    write_json(output/'installation.json',manifest)
    return 0


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',required=True)
    parser.add_argument('--image',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--patch', action='append', default=[],
                        help='explicit derivative/observation patch; frozen in installation')
    args=parser.parse_args()
    raise SystemExit(build(args.workspace,args.image,args.output,args.patch))
