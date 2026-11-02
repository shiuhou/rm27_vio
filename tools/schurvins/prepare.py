#!/usr/bin/env python3
"""Fetch hash-locked SchurVINS sources into an external catkin workspace.

Normal preparation never resolves a branch. --resolve-lock is the explicit
maintainer operation that freezes current dependency refs into a new lock.
No build, rosdep install, or estimator execution occurs here.
"""
import argparse
import concurrent.futures
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess
import tarfile

PIN = 'd8ab6dff20860694d8d879d716d7fd6cb2c2530e'
DEPS = {
    'catkin_simple': ('catkin/catkin_simple', 'master'),
    'cmake_external_project_catkin': ('zurich-eye/cmake_external_project_catkin', 'master'),
    'eigen_catkin': ('ethz-asl/eigen_catkin', 'master'),
    'eigen_checks': ('ethz-asl/eigen_checks', 'master'),
    'fast_neon': ('uzh-rpg/fast_neon', 'master'),
    'gflags_catkin': ('ethz-asl/gflags_catkin', 'master'),
    'glog_catkin': ('ethz-asl/glog_catkin', 'master'),
    'minkindr': ('ethz-asl/minkindr', 'master'),
    'opengv': ('ethz-asl/opengv', 'master'),
    'minkindr_ros': ('ethz-asl/minkindr_ros', 'master'),
    'ceres_catkin': ('ethz-asl/ceres_catkin', 'master'),
}
EXTERNAL = {
    'ceres_solver': ('ceres-solver/ceres-solver', 'facb199f3eda902360f9e1d5271372b7e54febe1', '1.14.0'),
    'gflags': ('gflags/gflags', '46f73f88b18aee341538c0dfc22b1710a6abedef', 'v2.2.1'),
}
SYSTEM = {'eigen': 'USE_SYSTEM_EIGEN=ON; version recorded in image dpkg manifest',
          'glog': 'USE_SYSTEM_GLOG=ON; version recorded in image dpkg manifest'}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def download(url):
    return subprocess.run(['curl', '--fail', '--location', '--silent', '--show-error',
                           '--connect-timeout', '15', '--max-time', '60',
                           '--retry', '1', '--', url], check=True, capture_output=True,
                          timeout=130).stdout


def archive_url(repo, commit):
    return 'https://codeload.github.com/{}/tar.gz/{}'.format(repo, commit)


def extract(data, destination):
    """Extract source files, permitting only symlinks resolving within the tree."""
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as source:
        members = source.getmembers()
        roots = {m.name.split('/')[0] for m in members}
        if len(roots) != 1:
            raise ValueError('source archive must contain one root')
        for member in members:
            relative = Path(*Path(member.name).parts[1:])
            if member.islnk() or member.isdev() or relative.is_absolute() or '..' in relative.parts:
                raise ValueError('unsafe source archive member: ' + member.name)
            if member.issym() and Path(member.linkname).is_absolute():
                raise ValueError('absolute source archive symlink: ' + member.name)
        destination.mkdir(parents=True, exist_ok=False)
        for member in members:
            relative = Path(*Path(member.name).parts[1:])
            target = destination / relative
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(source.extractfile(member).read())
                target.chmod(member.mode & 0o777)
        # Create links after all regular writes, so archive members can never
        # redirect file extraction through a symlink.
        links = []
        for member in members:
            if member.issym():
                link = destination / Path(*Path(member.name).parts[1:])
                link.parent.mkdir(parents=True, exist_ok=True)
                link.symlink_to(member.linkname)
                links.append(link)
        for link in links:
            if not link.resolve().is_relative_to(destination.resolve()):
                link.unlink()
                raise ValueError('source archive symlink escapes tree: ' + str(link))


def resolve_lock(output, cache):
    if output.exists():
        raise FileExistsError(output)
    cache.mkdir(parents=True, exist_ok=True)
    def resolve(item):
        name, (repo, ref) = item
        saved = cache / (name + '.json')
        if saved.exists():
            return name, json.loads(saved.read_text())
        remote = subprocess.run(['git', 'ls-remote', 'https://github.com/' + repo + '.git',
                                 'refs/heads/' + ref], check=True, capture_output=True,
                                text=True, timeout=30).stdout.split()
        if len(remote) != 2 or not re.fullmatch('[a-f0-9]{40}', remote[0]):
            raise ValueError('unable to resolve dependency ref: ' + repo + ':' + ref)
        commit = remote[0]
        data = download(archive_url(repo, commit))
        entry = dict(repository='https://github.com/' + repo, commit=commit,
                     requested_ref=ref, archive_url=archive_url(repo, commit), sha256=digest(data))
        (cache / (name + '-' + commit + '.tar.gz')).write_bytes(data)
        saved.write_text(json.dumps(entry, indent=2) + '\n')
        print('Locked ' + name, flush=True)
        return name, entry
    sources = dict(concurrent.futures.ThreadPoolExecutor(max_workers=4).map(resolve, DEPS.items()))
    data = download(archive_url('bytedance/SchurVINS', PIN))
    sources['SchurVINS'] = dict(repository='https://github.com/bytedance/SchurVINS', commit=PIN,
                               archive_url=archive_url('bytedance/SchurVINS', PIN), sha256=digest(data))
    external = {}
    for name, (repo, commit, tag) in EXTERNAL.items():
        url = archive_url(repo, commit)
        data = download(url)
        (cache / (name + '-' + commit + '.tar.gz')).write_bytes(data)
        external[name] = dict(repository='https://github.com/' + repo, commit=commit,
                              tag=tag, archive_url=url, sha256=digest(data))
    result = dict(schema_version=1, upstream_commit=PIN, sources=sources,
                  omitted_dependencies={'dbow2_catkin': 'loop closure disabled',
                                        'rpg_trajectory_evaluation': 'reuse RM27 fixed-scale evaluator'},
                  external_projects=external, external_projects_status='LOCKED_SOURCE_ARCHIVES',
                  required_system_dependencies=SYSTEM)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    return result


def prepare(lock_path, workspace):
    lock = json.loads(lock_path.read_text())
    if lock.get('schema_version') != 1 or lock.get('upstream_commit') != PIN:
        raise ValueError('unsupported source lock')
    sources = lock['sources']
    if sources.get('SchurVINS', {}).get('commit') != PIN:
        raise ValueError('SchurVINS commit mismatch')
    # Names and commits become local cache paths. Validate the complete lock
    # before materializing any member, including nested ExternalProject inputs.
    for entries in (sources, lock.get('external_projects', {})):
        for name, item in entries.items():
            if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', name):
                raise ValueError('unsafe source lock name: ' + name)
            if (not isinstance(item, dict)
                    or not isinstance(item.get('commit'), str)
                    or not re.fullmatch('[a-f0-9]{40}', item['commit'])
                    or not isinstance(item.get('sha256'), str)
                    or not re.fullmatch('[a-f0-9]{64}', item['sha256'])):
                raise ValueError('invalid lock entry ' + name)
    workspace = workspace.resolve()
    repo = Path(__file__).resolve().parents[2]
    if workspace == repo or repo in workspace.parents:
        raise ValueError('workspace must be outside the repository')
    workspace.mkdir(parents=True, exist_ok=True)
    provenance = workspace / 'source-lock.json'
    if provenance.exists() and json.loads(provenance.read_text()) != lock:
        raise ValueError('workspace belongs to another source lock')
    archive_dir = workspace / 'downloads'
    archive_dir.mkdir(exist_ok=True)
    result = {}
    for name, item in sources.items():
        commit, expected = item['commit'], item['sha256']
        archive = archive_dir / (name + '-' + commit + '.tar.gz')
        if archive.exists():
            data = archive.read_bytes()
        else:
            data = download(item['archive_url'])
            if digest(data) != expected:
                raise ValueError('download checksum mismatch: ' + name)
            archive.write_bytes(data)
        if digest(data) != expected:
            raise ValueError('cached checksum mismatch: ' + name)
        destination = workspace / 'src' / name
        stamp = workspace / ('extracted-' + name + '.json')
        if destination.exists():
            if not stamp.exists() or json.loads(stamp.read_text()) != item:
                raise ValueError('refusing unowned source directory: ' + str(destination))
        else:
            destination.parent.mkdir(parents=True, exist_ok=True)
            extract(data, destination)
            stamp.write_text(json.dumps(item, indent=2, sort_keys=True) + '\n')
        result[name] = str(destination)
    # Eliminate unverified nested Git/tag downloads from ExternalProject. Both
    # archives retain the exact upstream versions; this changes transport only.
    for name, item in lock.get('external_projects', {}).items():
        archive = archive_dir / (name + '-' + item['commit'] + '.tar.gz')
        data = archive.read_bytes() if archive.exists() else download(item['archive_url'])
        if digest(data) != item['sha256']:
            raise ValueError('external source checksum mismatch: ' + name)
        if not archive.exists():
            archive.write_bytes(data)
        cmake = workspace / 'src' / ('ceres_catkin' if name == 'ceres_solver' else 'gflags_catkin') / 'CMakeLists.txt'
        original = cmake.read_text()
        if name == 'ceres_solver':
            old = 'GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git\n  GIT_TAG ${VERSION}'
        elif name == 'gflags':
            old = 'URL https://github.com/gflags/gflags/archive/v2.2.1.zip\n  URL_MD5 2d988ef0b50939fb50ada965dafce96b'
        else:
            raise ValueError('unsupported external project: ' + name)
        new = 'URL file://${CMAKE_CURRENT_SOURCE_DIR}/../../downloads/' + archive.name + '\n  URL_HASH SHA256=' + item['sha256']
        if old in original:
            cmake.write_text(original.replace(old, new))
        elif new not in original:
            raise ValueError('external download recipe changed: ' + name)
    provenance.write_text(json.dumps(lock, indent=2, sort_keys=True) + '\n')
    # The documented upstream Python bindings are irrelevant to this C++ build.
    python_bindings = workspace / 'src' / 'minkindr' / 'minkindr_python'
    if python_bindings.is_dir():
        (python_bindings / 'CATKIN_IGNORE').touch()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lock', type=Path, default=Path(__file__).with_name('sources.lock.json'))
    parser.add_argument('--workspace', type=Path)
    parser.add_argument('--resolve-lock', action='store_true')
    args = parser.parse_args()
    if args.resolve_lock:
        if args.workspace is None:
            parser.error('--workspace required for lock-resolution downloads')
        print(json.dumps(resolve_lock(args.lock, args.workspace / 'downloads'), indent=2))
    else:
        if args.workspace is None:
            parser.error('--workspace required')
        print(json.dumps(prepare(args.lock, args.workspace), indent=2))


if __name__ == '__main__':
    main()
