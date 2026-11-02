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


def test_mono_alignment_production_header_known_motion_and_degeneracies(tmp_path):
    """Compile the shipped patch, with only unused SVO frame dependencies stubbed."""
    from pathlib import Path
    import shutil
    import subprocess
    repo = Path(__file__).resolve().parents[1]
    compiler = shutil.which('g++')
    eigen = Path('/usr/include/eigen3')
    if compiler is None or not (eigen/'Eigen/Core').exists():
        pytest.skip('production alignment fixture needs g++ and Eigen3 headers')
    patch = (repo/'tools/schurvins/mono-initialization.patch').read_text()
    marker = '+++ b/svo/include/svo/rm27_mono_alignment.h\n'
    section = patch.split(marker, 1)[1].split('\n--- a/', 1)[0]
    assert section.startswith('@@ -0,0 '), 'alignment must be the shipped new header'
    header = ''.join(line[1:] for line in section.splitlines(keepends=True)
                     if line.startswith('+'))
    include = tmp_path/'include'
    (include/'svo/common').mkdir(parents=True)
    (include/'svo/rm27_mono_alignment.h').write_text(header)
    # The fixture fills production MonoPose/IMU structures directly. No solver,
    # integration, observation, acceptance or motion equation is stubbed.
    (include/'svo/common/frame.h').write_text('''#pragma once
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace svo {
struct ImuMeasurement {
 ImuMeasurement(double t,const Eigen::Vector3d& w,const Eigen::Vector3d& a)
 :timestamp_(t),angular_velocity_(w),linear_acceleration_(a){}
 double timestamp_; Eigen::Vector3d angular_velocity_,linear_acceleration_;
};
using ImuMeasurements=std::vector<ImuMeasurement>;
struct Transform {
 Eigen::Vector3d getPosition()const{return Eigen::Vector3d::Zero();}
 Eigen::Quaterniond getEigenQuaternion()const{return Eigen::Quaterniond::Identity();}
};
struct Frame {Transform T_world_cam()const{return Transform();}};
struct FrameBundle {
 using Ptr=std::shared_ptr<FrameBundle>;
 double getMinTimestampSeconds()const{return 0;}
 std::shared_ptr<Frame> at(int)const{return std::make_shared<Frame>();}
 Transform get_T_W_B()const{return Transform();}
 ImuMeasurements imu_datas_;
};
}
''')
    (include/'svo/imu_handler.h').write_text('#include <svo/common/frame.h>\n')
    executable = tmp_path/'alignment'
    build_result = subprocess.run([
        compiler, '-std=c++14', '-O1', '-I'+str(include), '-I'+str(eigen),
        str(repo/'tests/fixtures/schur_mono_alignment.cpp'), '-o', str(executable)],
        capture_output=True, text=True, timeout=90)
    assert build_result.returncode == 0, build_result.stderr
    result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout+'\n'+result.stderr


def test_schur_tracking_quality_uses_production_inlier_history_and_current_evidence(tmp_path):
    """Compile the shipped quality helper; all decisions use its real implementation."""
    from pathlib import Path
    import shutil
    import subprocess
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('production quality fixture needs g++')
    repo = Path(__file__).resolve().parents[1]
    patch = (repo/'tools/schurvins/tracking-quality.patch').read_text()
    section = patch.split('+++ b/svo/include/svo/rm27_tracking_quality.h\n', 1)[1].split('\n--- a/', 1)[0]
    assert section.startswith('@@ -0,0 '), 'quality helper must be the shipped new header'
    header = ''.join(line[1:] for line in section.splitlines(keepends=True) if line.startswith('+'))
    (tmp_path/'rm27_tracking_quality.h').write_text(header)
    source = tmp_path/'quality.cpp'
    source.write_text('''#include "rm27_tracking_quality.h"
#include <cassert>
#include <limits>
int main() {
  rm27::SchurInlierQualityHistory quality;
  // Stereo bootstrap has metric geometry, but no accepted visual correction.
  auto first=quality.decide(240,40,40,false,false);
  assert(!first.good && !first.has_previous);
  auto previous=quality.decide(224,40,40,true,true);
  assert(previous.good && !previous.has_previous);
  // Recorded V101 frontend num_matches was 279 (seeds included). It must
  // not replace the actual previous Schur inlier count of 224.
  auto current=quality.decide(223,40,40,false,true);
  assert(current.good && current.previous_observations==224 && !current.drop_failed);
  // Recorded V103 early-return case: a previous Good never survives 7 inliers.
  auto low=quality.decide(7,40,40,false,true);
  assert(!low.good && low.minimum_failed && low.drop_failed);
  auto still_low=quality.decide(6,40,40,true,true);
  assert(!still_low.good && still_low.previous_observations==7);
  assert(!still_low.drop_failed);
  // Preserve the original absolute and drop thresholds, including strict >.
  quality.reset();
  assert(quality.decide(81,40,40,false,true).good);
  assert(quality.decide(41,40,40,false,true).good);
  assert(!quality.decide(39,40,40,false,true).good);
  quality.reset();
  assert(quality.decide(81,40,40,false,true).good);
  auto dropped=quality.decide(40,40,40,false,true);
  assert(!dropped.good && dropped.drop_failed && !dropped.minimum_failed);
  // Low quality still contributes its measured same-population inlier count.
  assert(quality.decide(40,40,40,false,true).good);
  quality.reset();
  quality.decide(81,40,40,false,true);
  assert(quality.decide(40,40,40,true,true).good);
  // Initialization keeps its original absolute gate without a comparison to
  // arbitrary-scale frontend populations. No missing update becomes Good.
  quality.reset();
  quality.decide(100,40,40,false,true);
  assert(quality.decide(40,40,40,false,true,false).good);
  assert(!quality.decide(39,40,40,true,true,false).good);
  assert(!quality.decide(300,40,40,true,false).good);
  auto fresh=quality.decide(40,40,40,false,true);
  assert(fresh.good && !fresh.has_previous);
  quality.reset();
  quality.decide(std::numeric_limits<std::size_t>::max(),40,40,false,true);
  assert(quality.decide(40,40,40,false,true).drop_failed);
  quality.reset();
  assert(!quality.decide(0,40,40,false,false).has_previous);
}
''')
    executable = tmp_path/'quality'
    compiled = subprocess.run([compiler, '-std=c++14', '-Wall', '-Wextra', '-Werror',
                               str(source), '-o', str(executable)],
                              capture_output=True, text=True, timeout=30)
    assert compiled.returncode == 0, compiled.stderr
    tested = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
    assert tested.returncode == 0, tested.stdout+'\n'+tested.stderr


def test_schur_input_health_production_header_preserves_sensor_order_and_strict_coverage(tmp_path):
    """Compile the shipped guard; no input acceptance algorithm is reimplemented."""
    from pathlib import Path
    import shutil
    import subprocess
    compiler, patch_command = shutil.which('g++'), shutil.which('patch')
    eigen = Path('/usr/include/eigen3')
    if compiler is None or patch_command is None or not (eigen/'Eigen/Core').exists():
        pytest.skip('production input-health fixture needs g++, patch and Eigen3 headers')
    repo = Path(__file__).resolve().parents[1]
    patch = (repo/'tools/schurvins/input-health.patch').read_text()
    section = patch.split('+++ b/svo/include/svo/rm27_input_health.h\n', 1)[1].split('\n--- a/', 1)[0]
    assert section.startswith('@@ -0,0 '), 'input guard must be the shipped new header'
    header = ''.join(line[1:] for line in section.splitlines(keepends=True) if line.startswith('+'))
    (tmp_path/'rm27_input_health.h').write_text(header.rstrip('\n')+'\n')
    for patch_name in ('native-time-resolution.patch', 'input-restart-coverage.patch'):
        candidate = (repo/'tools/schurvins'/patch_name).read_text()
        delta = candidate.split('\n--- a/', 1)[0].rstrip('\n')+'\n'
        patched = subprocess.run([patch_command, '-p4'], input=delta, cwd=tmp_path,
                                 capture_output=True, text=True, timeout=10)
        assert patched.returncode == 0, patched.stderr+patched.stdout
    source = tmp_path/'input_health.cpp'
    source.write_text('''#include <cstddef>
#include "rm27_input_health.h"
#include <Eigen/Core>
#include <cassert>
#include <cstring>
#include <limits>
#include <vector>
struct Sample {
  double timestamp_;
  Eigen::Vector3d angular_velocity_=Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration_=Eigen::Vector3d::Zero();
  explicit Sample(double t):timestamp_(t){}
};
int main() {
  using F=rm27::InputFault;
  const int64_t t=1578489382112345678LL;
  rm27::CameraInputOrder camera;
  assert(camera.accept(0)==F::CameraTimestamp);
  assert(camera.accept(-1)==F::CameraTimestamp);
  assert(camera.accept(t)==F::None && camera.latest==t);
  assert(camera.accept(t)==F::CameraOrder && camera.latest==t);
  assert(camera.accept(t-1)==F::CameraOrder && camera.latest==t);
  assert(camera.accept(t+1)==F::None && camera.latest==t+1);
  // Preserve distinct integer times even when the native double-seconds
  // representation cannot advance. This is a rejection, never a retimestamp.
  const double previous=static_cast<double>(t)*1e-9;
  const double one_ns_later=static_cast<double>(t+1)*1e-9;
  assert(previous==one_ns_later);
  assert(rm27::cameraNativeTimeHealth(one_ns_later,previous)==F::CameraNativeResolution);
  assert(camera.latest==t+1);
  assert(std::strcmp(rm27::inputFaultName(F::CameraNativeResolution),
      "input_rejected_camera_native_resolution")==0);
  assert(rm27::cameraNativeTimeHealth(previous,previous)==F::CameraNativeResolution);
  assert(rm27::cameraNativeTimeHealth(previous-1.,previous)==F::CameraNativeResolution);
  assert(rm27::cameraNativeTimeHealth(std::numeric_limits<double>::quiet_NaN(),previous)
      ==F::CameraNativeResolution);
  assert(rm27::cameraNativeTimeHealth(previous,std::numeric_limits<double>::infinity())
      ==F::CameraNativeResolution);
  // A subsequent normal 20 Hz frame passes both raw and native boundaries.
  assert(camera.accept(t+50000000)==F::None && camera.latest==t+50000000);
  assert(rm27::cameraNativeTimeHealth(static_cast<double>(camera.latest)*1e-9,previous)==F::None);
  // The actual representable successor also passes: no epsilon is imposed.
  assert(rm27::cameraNativeTimeHealth(std::nextafter(previous,
      std::numeric_limits<double>::infinity()),previous)==F::None);
  rm27::ImuInputOrder imu;
  assert(imu.accept(t,false)==F::ImuNonfinite && imu.latest==-1);
  assert(imu.accept(0,true)==F::ImuNonfinite && imu.latest==-1);
  assert(imu.accept(t,true)==F::None);
  assert(imu.accept(t,true)==F::ImuOrder && imu.latest==t);
  assert(imu.accept(t-1,true)==F::ImuOrder && imu.latest==t);
  assert(imu.accept(t+1,true)==F::None && imu.latest==t+1);
  assert(imu.accept(t+10000001,true)==F::None);
  assert(imu.accept(t+25000001,true)==F::ImuGap);
  assert(imu.latest==t+25000001);
  imu.reset();
  assert(imu.latest==-1 && imu.accept(123,true)==F::None);
  std::vector<Sample> samples{Sample(0),Sample(.01),Sample(.02),Sample(.03)};
  assert(rm27::imuIntervalHealth(samples,0,.03)==F::None);
  assert(rm27::imuIntervalHealth(samples,.005,.025)==F::None);
  assert(rm27::imuIntervalHealth(samples,-.001,.02)==F::ImuCoverage);
  assert(rm27::imuIntervalHealth(samples,.01,.031)==F::ImuCoverage);
  assert(rm27::imuIntervalHealth(samples,.02,.01)==F::ImuCoverage);
  assert(rm27::imuIntervalHealth(samples,0,std::numeric_limits<double>::quiet_NaN())==F::ImuCoverage);
  assert(rm27::imuIntervalHealth(std::vector<Sample>{},0,.01)==F::ImuCoverage);
  // A restart clears old-epoch samples after the ROS input boundary check.
  // No unprocessed frame becomes valid; fresh coverage must arrive first.
  const auto empty_epoch=rm27::imuIntervalHealth(std::vector<Sample>{},0,.01);
  assert(rm27::waitForInitialImuCoverage(empty_epoch,false));
  assert(!rm27::waitForInitialImuCoverage(empty_epoch,true));
  const auto future_only=rm27::imuIntervalHealth(
      std::vector<Sample>{Sample(.02),Sample(.03)},.01,.02);
  assert(future_only==F::ImuCoverage);
  assert(rm27::waitForInitialImuCoverage(future_only,false));
  assert(!rm27::waitForInitialImuCoverage(future_only,true));
  const auto fresh_epoch=rm27::imuIntervalHealth(samples,.01,.03);
  assert(fresh_epoch==F::None && !rm27::waitForInitialImuCoverage(fresh_epoch,false));
  for(auto fault:{F::ImuNonfinite,F::ImuOrder,F::ImuGap}) {
    assert(!rm27::waitForInitialImuCoverage(fault,false));
    assert(!rm27::waitForInitialImuCoverage(fault,true));
  }
  assert(rm27::imuIntervalHealth(std::vector<Sample>{Sample(0)},0,0)==F::ImuCoverage);
  std::vector<Sample> boundary{Sample(0),Sample(.015)};
  assert(rm27::imuIntervalHealth(boundary,0,.015)==F::ImuGap);
  boundary[1].timestamp_=.014999999;
  assert(rm27::imuIntervalHealth(boundary,0,.014999999)==F::None);
  // Startup history outside the consumed interval is not a current gap.
  std::vector<Sample> history{Sample(-1),Sample(0),Sample(.01),Sample(.02),Sample(1)};
  assert(rm27::imuIntervalHealth(history,0,.02)==F::None);
  assert(rm27::imuIntervalHealth(history,-.001,.02)==F::ImuGap);
  assert(rm27::imuIntervalHealth(history,0,.021)==F::ImuGap);
  auto invalid=samples;
  invalid[2].timestamp_=invalid[1].timestamp_;
  assert(rm27::imuIntervalHealth(invalid,0,.03)==F::ImuOrder);
  invalid=samples;invalid[2].timestamp_=.005;
  assert(rm27::imuIntervalHealth(invalid,0,.03)==F::ImuOrder);
  invalid=samples;invalid[1].timestamp_=std::numeric_limits<double>::quiet_NaN();
  assert(rm27::imuIntervalHealth(invalid,0,.03)==F::ImuNonfinite);
  invalid=samples;invalid.front().timestamp_=std::numeric_limits<double>::infinity();
  assert(rm27::imuIntervalHealth(invalid,0,.03)==F::ImuNonfinite);
  invalid=samples;invalid.back().timestamp_=std::numeric_limits<double>::infinity();
  assert(rm27::imuIntervalHealth(invalid,0,.03)==F::ImuNonfinite);
  invalid=samples;invalid[1].angular_velocity_.x()=std::numeric_limits<double>::quiet_NaN();
  assert(rm27::imuIntervalHealth(invalid,0,.03)==F::ImuNonfinite);
  invalid=samples;invalid[1].linear_acceleration_.z()=std::numeric_limits<double>::infinity();
  assert(rm27::imuIntervalHealth(invalid,0,.03)==F::ImuNonfinite);
}
''')
    executable = tmp_path/'input_health'
    compiled = subprocess.run([compiler, '-std=c++14', '-Wall', '-Wextra', '-Werror',
                               '-I'+str(eigen), str(source), '-o', str(executable)],
                              capture_output=True, text=True, timeout=30)
    assert compiled.returncode == 0, compiled.stderr
    tested = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
    assert tested.returncode == 0, tested.stdout+'\n'+tested.stderr


@pytest.mark.parametrize('production_flags', [False, True])
def test_native_imu_callback_production_header_requires_explicit_join_and_retains_bits(tmp_path, production_flags):
    """Run the shipped C++ callback boundary, including unclean/drop evidence."""
    import json
    from pathlib import Path
    import shutil
    import struct
    import subprocess
    compiler = shutil.which('g++')
    if compiler is None:
        pytest.skip('native callback boundary fixture needs g++')
    repo = Path(__file__).resolve().parents[1]
    patch = (repo/'tools/schurvins/native-imu-callback-boundaries.patch').read_text()
    section = patch.split('+++ b/svo/include/svo/rm27_imu_observation.h\n', 1)[1].split('\n--- a/', 1)[0]
    header = ''.join(line[1:] for line in section.splitlines(keepends=True) if line.startswith('+'))
    (tmp_path/'rm27_imu_observation.h').write_text(header)
    (tmp_path/'main.cpp').write_text(r'''
#include "rm27_imu_observation.h"
int main(int argc,char** argv) {
  if(argc!=3)return 2;
  if(std::string(argv[1])=="capacity") {
    rm27_imu::Sink sink(argv[2],1);
    for(int i=0;i<2;++i) {
      rm27_imu::Record row;row.receipt_sequence=sink.begin();
      row.values={{1,2,3,4,5,6}};sink.append(row);
    }
    sink.close("svo_interface_callback_threads_joined");return 0;
  }
  setenv("RM27_SCHUR_IMU_LEDGER",argv[2],1);
  rm27_imu::start();
  if(std::string(argv[1])=="nonfinite") {
    {
      rm27_imu::Callback callback(1403715273,262142976,18,"imu",
          std::numeric_limits<double>::infinity(),
          {{std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),-0.,0.,-.25}});
      callback.finish(rm27_imu::Disposition::HealthRejected);
    }
    rm27_imu::stop();return 0;
  }
  {
    const std::string frame("imu\0id",6);
    rm27_imu::Callback callback(1403715273,262142976,17,frame,
        1403715273.+262142976.*1e-9,{{.1,-.2,.3,-0.,.5,9.81}});
    callback.finish(rm27_imu::Disposition::BufferAccepted);
  }
  if(std::string(argv[1])=="joined")rm27_imu::stop();
  return 0;
}
''')
    binary = tmp_path/'callback'
    flags = (['-O3','-fsee','-fomit-frame-pointer','-fno-signed-zeros','-fno-math-errno',
              '-funroll-loops','-ffast-math','-fno-finite-math-only'] if production_flags else [])
    result = subprocess.run([compiler,*flags,'-std=c++14','-Wall','-Wextra','-Werror','-pthread',
        str(tmp_path/'main.cpp'),'-o',str(binary)],capture_output=True,text=True)
    assert result.returncode == 0, result.stderr
    for mode in ('joined','abrupt','capacity','nonfinite'):
        path = tmp_path/(mode+'.jsonl')
        run = subprocess.run([str(binary),mode,str(path)],capture_output=True,text=True)
        assert run.returncode == 0, run.stderr
        records = [json.loads(line) for line in path.read_text().splitlines()]
        if mode == 'abrupt':
            assert records == [] and 'UNFINALIZED' in run.stderr
        elif mode == 'capacity':
            assert len(records) == 2
            assert records[-1]['attempted_callbacks'] == 2
            assert records[-1]['written_records'] == 1 and records[-1]['dropped_records'] == 1
        elif mode == 'nonfinite':
            row,footer = records
            assert row['values'][:3] == [None,None,None]
            assert row['native_seconds'] is None
            assert row['native_seconds_bits'] == 0x7ff0000000000000
            assert row['value_bits'][0] & 0x7fffffffffffffff > 0x7ff0000000000000
            assert row['value_bits'][1:3] == [0x7ff0000000000000,0xfff0000000000000]
            assert [struct.pack('=d',v) for v in row['values'][3:]] == [
                struct.pack('=d',v) for v in (-0.,0.,-.25)]
            assert row['disposition'] == 'health_rejected'
            assert footer['written_records'] == 1 and not footer['io_failed']
        else:
            row,footer = records
            assert row['header_sequence'] == 17 and row['sensor_ns'] == 1403715273262142976
            assert row['frame_id_hex'] == b'imu\0id'.hex() and row['frame_id_size'] == 6
            assert row['disposition'] == 'buffer_accepted'
            assert struct.pack('=d',row['values'][3]) == struct.pack('=d',-0.)
            assert row['value_bits'] == [struct.unpack('=Q',struct.pack('=d',v))[0] for v in row['values']]
            assert footer['close_boundary'] == 'svo_interface_callback_threads_joined'
            assert footer['attempted_callbacks'] == footer['written_records'] == 1
            assert footer['dropped_records'] == 0 and not footer['io_failed']


def test_native_event_timestamp_production_header_keeps_exact_source_nanoseconds(tmp_path):
    """Execute the actual patched logger, stubbing only unused frame access."""
    from pathlib import Path
    import os
    import shutil
    import subprocess
    compiler, patch_command = shutil.which('g++'), shutil.which('patch')
    if compiler is None or patch_command is None:
        pytest.skip('production event fixture needs g++ and patch')
    repo = Path(__file__).resolve().parents[1]
    base = (repo/'tools/schurvins/native-diagnostics.patch').read_text()
    marker = '+++ b/svo/include/svo/rm27_diagnostics.h\n'
    section = base.split(marker, 1)[1].split('\n--- a/', 1)[0]
    header = ''.join(line[1:] for line in section.splitlines(keepends=True)
                     if line.startswith('+'))
    target = tmp_path/'svo/include/svo/rm27_diagnostics.h'
    target.parent.mkdir(parents=True)
    target.write_text(header.rstrip('\n')+'\n')
    guard = (repo/'tools/schurvins/input-health.patch').read_text()
    section = guard.split('--- a/svo/include/svo/rm27_diagnostics.h\n', 1)[1]
    section = section.split('\n--- a/', 1)[0]
    delta = '--- a/svo/include/svo/rm27_diagnostics.h\n'+section+'\n'
    patched = subprocess.run([patch_command, '-p1'], input=delta, cwd=tmp_path,
                             capture_output=True, text=True, timeout=10)
    assert patched.returncode == 0, patched.stderr+patched.stdout
    include = tmp_path/'include'
    (include/'svo/common').mkdir(parents=True)
    (include/'svo/common/frame.h').write_text('''#pragma once
#include <cstdint>
#include <memory>
namespace svo {
struct FrameBundle {
 using Ptr=std::shared_ptr<FrameBundle>;
 int64_t getMinTimestampNanoseconds()const{return 1578489382112345679LL;}
 int64_t getBundleId()const{return 17;}
};
}
''')
    source = tmp_path/'events.cpp'
    source.write_text('''#include <svo/rm27_diagnostics.h>
#include <limits>
int main(){
 rm27::eventAtTimestamp("input_invalid_imu_nonfinite",1578489382112345678LL,-1,
     {3.,0.},1578489382112345677LL,"imu0");
 rm27::event("reset",std::make_shared<svo::FrameBundle>());
 rm27::eventAtTimestamp("diagnostic_missing",1578489382112345680LL,-1,
     {std::numeric_limits<double>::infinity()});
}
''')
    executable, events = tmp_path/'events', tmp_path/'events.jsonl'
    compiled = subprocess.run([compiler, '-std=c++14', '-Wall', '-Wextra', '-Werror',
                               '-I'+str(include), '-I'+str(target.parents[1]),
                               str(source), '-o', str(executable)],
                              capture_output=True, text=True, timeout=30)
    assert compiled.returncode == 0, compiled.stderr
    tested = subprocess.run([str(executable)], env=dict(os.environ, RM27_NATIVE_EVENTS=str(events)),
                            capture_output=True, text=True, timeout=10)
    assert tested.returncode == 0, tested.stderr
    rows = [json.loads(line) for line in events.read_text().splitlines()]
    assert len(rows) == 3
    assert [row['sensor_ns'] for row in rows] == [1578489382112345678,
                                                1578489382112345679,
                                                1578489382112345680]
    assert rows[0]['source_sensor_ns'] == 1578489382112345677
    assert rows[0]['source_stream'] == 'imu0'
    assert rows[0]['bundle_id'] == -1 and rows[1]['bundle_id'] == 17
    assert rows[0]['values'] == [3, 0] and rows[2]['values'] == [None]
    assert all(type(row['sensor_ns']) is int for row in rows)
    assert type(rows[0]['source_sensor_ns']) is int
    assert all(a['monotonic_ns'] <= b['monotonic_ns'] for a, b in zip(rows, rows[1:]))
