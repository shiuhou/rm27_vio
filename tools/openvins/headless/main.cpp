// Sensor-only EuRoC transport for the pinned upstream OpenVINS estimator.
// No ROS, ground-truth loader, synthetic features, or external initialization.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <boost/filesystem.hpp>
#include <opencv2/imgcodecs.hpp>
#include "audit.h"
#include "core/VioManager.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

namespace fs = boost::filesystem;
using Clock = std::chrono::steady_clock;
using Ns = std::chrono::nanoseconds;
struct Image { int64_t stamp; std::string filename; };
struct Imu { int64_t stamp; ov_core::ImuData data; };
struct Args { fs::path dataset, config, output; double rate = 0; size_t limit = 0; };

int64_t now_ns() { return std::chrono::duration_cast<Ns>(Clock::now().time_since_epoch()).count(); }
int64_t stamp(const std::string &s) {
  if (s.empty() || s.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("timestamp must be original unsigned integer nanoseconds");
  size_t pos = 0;
  const auto n = std::stoll(s, &pos);
  if (n <= 0 || pos != s.size()) throw std::runtime_error("invalid timestamp");
  return n;
}
double number(const std::string &s) {
  size_t pos = 0; const double v = std::stod(s, &pos);
  if (pos != s.size() || !std::isfinite(v)) throw std::runtime_error("nonfinite or malformed numeric input");
  return v;
}
std::vector<std::vector<std::string>> csv(const fs::path &path, size_t columns) {
  std::ifstream in(path.string());
  if (!in) throw std::runtime_error("cannot read input: " + path.string());
  std::vector<std::vector<std::string>> result;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> row; std::stringstream ss(line); std::string field;
    while (std::getline(ss, field, ',')) row.push_back(field);
    if (row.size() != columns) throw std::runtime_error("unexpected CSV column count: " + path.string());
    result.push_back(row);
  }
  if (result.empty()) throw std::runtime_error("empty sensor input: " + path.string());
  return result;
}
Args parse(int argc, char **argv) {
  Args a;
  for (int i=1; i<argc; ++i) {
    const std::string key(argv[i]);
    if (key == "--help") {
      std::cout << "openvins_headless --dataset ASL_ROOT --config MONO_YAML --output NEW_DIR"
                   " [--rate 0|0.5|1] [--max-frames N]\n";
      std::exit(0);
    }
    if (i+1 >= argc) throw std::runtime_error("missing option value: " + key);
    const std::string value(argv[++i]);
    if (key=="--dataset") a.dataset=value;
    else if (key=="--config") a.config=value;
    else if (key=="--output") a.output=value;
    else if (key=="--rate") a.rate=number(value);
    else if (key=="--max-frames") {
      if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("max-frames must be a positive integer");
      a.limit=std::stoull(value);
      if (!a.limit) throw std::runtime_error("max-frames must be positive");
    } else throw std::runtime_error("unknown option: " + key);
  }
  if (a.dataset.empty() || a.config.empty() || a.output.empty()) throw std::runtime_error("dataset, config and output required");
  if (a.rate!=0 && a.rate!=0.5 && a.rate!=1) throw std::runtime_error("rate must be 0, 0.5 or 1");
  return a;
}

class ObservedManager : public ov_msckf::VioManager {
public:
  explicit ObservedManager(ov_msckf::VioManagerOptions opts) : VioManager(opts) {}
  double frontend_ms() const {
    return (rT2-rT1).total_microseconds()*1e-3;
  }
  bool completed(double timestamp) const { return timelastupdate==timestamp; }
  double propagation_ms() const { return (rT3-rT2).total_microseconds()*1e-3; }
  double msckf_ms() const { return (rT4-rT3).total_microseconds()*1e-3; }
  double slam_ms() const { return (rT5-rT4).total_microseconds()*1e-3; }
  double backend_ms() const { return (rT7-rT2).total_microseconds()*1e-3; }
};

int main(int argc, char **argv) {
  try {
    const Args a=parse(argc, argv);
    if (fs::exists(a.output)) throw std::runtime_error("output already exists: " + a.output.string());
    std::vector<Image> images;
    int64_t previous=0;
    for (const auto &row:csv(a.dataset/"mav0/cam0/data.csv",2)) {
      const auto n=stamp(row[0]);
      if (n<=previous) throw std::runtime_error("camera timestamps must strictly increase");
      const fs::path name(row[1]);
      if (name.is_absolute() || name.filename()!=name || name.extension()!=".png")
        throw std::runtime_error("camera filename must be a plain PNG filename");
      images.push_back({n,row[1]}); previous=n;
    }
    std::vector<Imu> imu;
    previous=0;
    for (const auto &row:csv(a.dataset/"mav0/imu0/data.csv",7)) {
      const auto n=stamp(row[0]);
      if (n<=previous) throw std::runtime_error("IMU timestamps must strictly increase");
      Imu m; m.stamp=n; m.data.timestamp=static_cast<double>(n)*1e-9;
      for (size_t j=0;j<3;++j) { m.data.wm(j)=number(row[j+1]); m.data.am(j)=number(row[j+4]); }
      imu.push_back(m); previous=n;
    }
    const size_t camera_input=images.size();
    if (a.limit && a.limit<images.size()) images.resize(a.limit);
    ov_core::Printer::setPrintLevel("WARNING");
    auto parser=std::make_shared<ov_core::YamlParser>(fs::absolute(a.config).string());
    ov_msckf::VioManagerOptions opts;
    opts.print_and_load(parser);
    if (!parser->successful()) throw std::runtime_error("incomplete OpenVINS configuration");
    if (opts.state_options.num_cameras!=1 || opts.use_stereo)
      throw std::runtime_error("this runtime requires max_cameras=1 and use_stereo=false");
    if (opts.use_multi_threading_subs || opts.use_multi_threading_pubs)
      throw std::runtime_error("headless replay requires explicit synchronous subscriber/publisher configuration");
    if (opts.record_timing_information)
      throw std::runtime_error("external native output paths must be disabled; runtime owns immutable output directory");
    if (opts.use_aruco) throw std::runtime_error("runtime was built without ArUco support");
    if (opts.use_mask) throw std::runtime_error("external image masks are not supported by EuRoC transport");
    if (imu.front().stamp>images.front().stamp || imu.back().stamp<=images.back().stamp)
      throw std::runtime_error("IMU must bracket selected camera sequence");
    if (!fs::create_directories(a.output)) throw std::runtime_error("cannot create fresh output");
    std::ofstream frames((a.output/"frames.csv").string()), poses((a.output/"poses.csv").string());
    if (!frames || !poses) throw std::runtime_error("cannot create output files");
    frames << "timestamp_ns,scheduled_ns,start_ns,end_ns,initialized,state_matches_camera,finite_posterior,pose_emitted,msckf_updates,msckf_features,msckf_rows,slam_updates,slam_features,slam_rows,imu_delivered,queue_pending_frames,frontend_ms,propagation_ms,msckf_ms,slam_ms,backend_ms\n";
    poses << "timestamp_ns,tx,ty,tz,qw,qx,qy,qz,vx,vy,vz,camera_to_imu_offset_s,timestamp_imu_seconds,visual_update\n";
    frames << std::setprecision(17); poses << std::setprecision(17);
    ObservedManager manager(opts);
    const auto begin=Clock::now();
    const int64_t begin_ns=std::chrono::duration_cast<Ns>(begin.time_since_epoch()).count();
    size_t imu_cursor=0, pose_count=0, finite_failures=0, state_failures=0, accepted_frames=0;
    int64_t first_init=0, last_accepted=0, max_update_gap=0;
    for (size_t i=0;i<images.size();++i) {
      const auto &image=images[i];
      const int64_t due=a.rate ? begin_ns+static_cast<int64_t>((image.stamp-images.front().stamp)/a.rate) : begin_ns;
      if (a.rate) std::this_thread::sleep_until(Clock::time_point(Ns(due)));
      const int64_t start=now_ns();
      // Upstream propagation interpolates the camera endpoint. Feed one real
      // IMU sample strictly beyond the current estimated endpoint; no synthetic
      // padding or discarded samples. Exact original stamps remain in ledgers.
      const double endpoint=static_cast<double>(image.stamp)*1e-9 + manager.get_state()->_calib_dt_CAMtoIMU->value()(0);
      while ((!imu_cursor || imu[imu_cursor-1].data.timestamp<=endpoint) && imu_cursor<imu.size())
        manager.feed_measurement_imu(imu[imu_cursor++].data);
      if (!imu_cursor || imu[imu_cursor-1].data.timestamp<=endpoint)
        throw std::runtime_error("IMU does not bracket current calibrated camera endpoint");
      cv::Mat pixels=cv::imread((a.dataset/"mav0/cam0/data"/image.filename).string(),cv::IMREAD_GRAYSCALE);
      if (pixels.empty()) throw std::runtime_error("camera decode failed: " + image.filename);
      const auto camera=opts.camera_intrinsics.at(0);
      const int scale=opts.downsample_cameras ? 2 : 1;
      if (pixels.cols!=camera->w()*scale || pixels.rows!=camera->h()*scale)
        throw std::runtime_error("camera pixels do not match calibration resolution");
      ov_core::CameraData message;
      message.timestamp=static_cast<double>(image.stamp)*1e-9;
      message.sensor_ids.push_back(0); message.images.push_back(pixels);
      message.masks.push_back(cv::Mat::zeros(pixels.size(),CV_8UC1));
      const auto before=rm27_openvins::audit;
      manager.feed_measurement_camera(message);
      const int64_t end=now_ns();
      const auto after=rm27_openvins::audit;
      const auto state=manager.get_state();
      const bool initialized=manager.initialized();
      const bool matches=state->_timestamp==message.timestamp;
      const bool complete=manager.completed(message.timestamp);
      const auto covariance=ov_msckf::StateHelper::get_full_covariance(state);
      const bool finite=state->_imu->value().allFinite() && covariance.allFinite() &&
                        (covariance.diagonal().array()>=0).all() &&
                        std::abs(state->_imu->quat().norm()-1)<1e-6;
      const auto msckf=after.msckf_updates-before.msckf_updates;
      const auto slam=after.slam_updates-before.slam_updates;
      const bool accepted=(msckf+slam)>0 && initialized && matches && finite;
      if (accepted) {
        ++accepted_frames;
        if (last_accepted) max_update_gap=std::max(max_update_gap,image.stamp-last_accepted);
        last_accepted=image.stamp;
      }
      if (initialized && !first_init) first_init=image.stamp;
      if (!finite) ++finite_failures;
      if (initialized && !matches) ++state_failures;
      const bool emit=initialized && matches && finite;
      if (emit) {
        const auto p=state->_imu->pos();
        const auto q=state->_imu->quat();
        const auto v=state->_imu->vel();
        // Native JPL q_GtoI components, reordered to qw,qx,qy,qz, equal the
        // Hamilton q_ItoG components. No conjugation is needed. Native Rot()
        // itself maps G vectors to I; the Hamilton matrix is its transpose.
        poses << image.stamp << ',' << p(0) << ',' << p(1) << ',' << p(2) << ','
              << q(3) << ',' << q(0) << ',' << q(1) << ',' << q(2) << ','
              << v(0) << ',' << v(1) << ',' << v(2) << ',' << state->_calib_dt_CAMtoIMU->value()(0)
              << ',' << state->_timestamp+state->_calib_dt_CAMtoIMU->value()(0) << ',' << accepted << '\n';
        ++pose_count;
      }
      size_t pending=0;
      if (a.rate) for (size_t j=i+1;j<images.size();++j) {
        const int64_t scheduled=begin_ns+static_cast<int64_t>((images[j].stamp-images.front().stamp)/a.rate);
        if (scheduled>end) break;
        ++pending;
      }
      frames << image.stamp << ',' << due << ',' << start << ',' << end << ',' << initialized << ',' << matches << ',' << finite << ',' << emit
             << ',' << msckf << ',' << after.msckf_features-before.msckf_features << ',' << after.msckf_rows-before.msckf_rows
             << ',' << slam << ',' << after.slam_features-before.slam_features << ',' << after.slam_rows-before.slam_rows
             << ',' << imu_cursor << ',' << pending << ',' << manager.frontend_ms() << ',';
      if (complete) frames << manager.propagation_ms() << ',' << manager.msckf_ms() << ',' << manager.slam_ms() << ',' << manager.backend_ms();
      else frames << ",,,";
      frames << '\n';
      if (!finite) throw std::runtime_error("nonfinite/invalid native posterior");
    }
    if (images.size()==camera_input) while (imu_cursor<imu.size()) manager.feed_measurement_imu(imu[imu_cursor++].data);
    frames.close(); poses.close();
    std::ofstream summary((a.output/"summary.json").string());
    summary << std::setprecision(17)
      << "{\n  \"schema_version\": 1,\n  \"backend\": \"OpenVINS\",\n"
      << "  \"upstream_commit\": \"69488123ed9362dd44b6f28e7f4680abbff1442b\",\n"
      << "  \"status\": \"EXECUTION_COMPLETED\",\n  \"operational_validity\": \"NOT_RUN\",\n"
      << "  \"mode\": \"mono_imu\",\n  \"rate\": " << a.rate << ",\n"
      << "  \"camera_input\": " << camera_input << ",\n"
      << "  \"selected_frames\": " << images.size() << ",\n  \"poses\": " << pose_count
      << ",\n  \"imu_input\": " << imu.size() << ",\n  \"imu_delivered\": " << imu_cursor
      << ",\n  \"first_initialized_timestamp_ns\": " << first_init << ",\n"
      << "  \"accepted_visual_update_frames\": " << accepted_frames << ",\n"
      << "  \"msckf_updates\": " << rm27_openvins::audit.msckf_updates << ",\n"
      << "  \"slam_updates\": " << rm27_openvins::audit.slam_updates << ",\n"
      << "  \"last_visual_update_timestamp_ns\": " << last_accepted << ",\n"
      << "  \"maximum_visual_update_gap_ns\": " << max_update_gap << ",\n"
      << "  \"nonfinite_posteriors\": " << finite_failures << ",\n"
      << "  \"state_timestamp_failures\": " << state_failures << ",\n"
      << "  \"quaternion_convention\": \"native_OpenVINS_JPL_q_GtoI_reordered_wxyz\",\n"
      << "  \"position_convention\": \"p_IinG_m\",\n"
      << "  \"timestamp_contract\": \"original_camera_integer_ns; estimator_API_double_seconds; IMU_clock=CAM_clock+estimated_offset\",\n"
      << "  \"timing_contract\": \"steady_clock callback/decode plus upstream boost wallclock stage timers; replay pending is scheduled lateness, not hardware queue\"\n}\n";
    if (!summary) throw std::runtime_error("summary write failed");
    return 0;
  } catch(const std::exception &e) {
    std::cerr << "openvins_headless: " << e.what() << '\n'; return 2;
  }
}
