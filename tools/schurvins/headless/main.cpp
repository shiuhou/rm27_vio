// Platform-independent SchurVINS boundary. No ROS runtime or camera/UART access.
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/imu_handler.h>
#include <opencv2/imgcodecs.hpp>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <sys/resource.h>

namespace svo { namespace headless {
std::shared_ptr<FrameHandlerMono> makeMono(const YAML::Node&, const CameraBundlePtr&);
std::shared_ptr<FrameHandlerStereo> makeStereo(const YAML::Node&, const CameraBundlePtr&);
std::shared_ptr<ImuHandler> getImuHandler(const YAML::Node&);
}}

namespace {
using Clock = std::chrono::steady_clock;
int64_t monotonic() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
std::ofstream outputFile(const std::string& path) {
  std::ofstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path);
  return stream;
}
struct Image { int64_t ns; std::string path; };
struct Imu { int64_t ns; Eigen::Vector3d gyro, accel; };
std::vector<std::string> fields(std::string line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::stringstream stream(line); std::vector<std::string> out; std::string field;
  while (std::getline(stream, field, ',')) out.push_back(field);
  return out;
}
int64_t integer(const std::string& text) {
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("timestamp must be positive integer nanoseconds");
  size_t used; auto value = std::stoll(text, &used);
  if (used != text.size() || value <= 0) throw std::runtime_error("invalid timestamp");
  return value;
}
double number(const std::string& text) {
  size_t used; double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) throw std::runtime_error("invalid finite measurement");
  return value;
}
std::vector<Image> images(const std::string& folder) {
  std::ifstream input(folder + "/data.csv");
  if (!input) throw std::runtime_error("missing camera data.csv: " + folder);
  std::string line; std::vector<Image> out;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto parts = fields(line);
    if (parts.size() != 2 || parts[1].empty() || parts[1].find('/') != std::string::npos || parts[1].find("..") != std::string::npos)
      throw std::runtime_error("invalid image row");
    int64_t ns = integer(parts[0]);
    if (!out.empty() && ns <= out.back().ns) throw std::runtime_error("camera timestamps are not strictly increasing");
    out.push_back({ns, folder + "/data/" + parts[1]});
  }
  if (out.empty()) throw std::runtime_error("empty camera input");
  return out;
}
std::vector<Imu> inertial(const std::string& folder) {
  std::ifstream input(folder + "/data.csv");
  if (!input) throw std::runtime_error("missing IMU data.csv");
  std::string line; std::vector<Imu> out;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto p = fields(line);
    if (p.size() != 7) throw std::runtime_error("invalid IMU row");
    Imu m{integer(p[0]), {number(p[1]), number(p[2]), number(p[3])}, {number(p[4]), number(p[5]), number(p[6])}};
    if (!out.empty() && m.ns <= out.back().ns) throw std::runtime_error("IMU timestamps are not strictly increasing");
    out.push_back(m);
  }
  if (out.empty()) throw std::runtime_error("empty IMU input");
  return out;
}
bool prior(svo::FrameHandlerBase& vo, svo::ImuHandler& imu, int64_t ns) {
  if (!vo.hasStarted()) {
    svo::Quaternion q;
    if (!imu.getInitialAttitude(ns * 1e-9, q)) return false;
    vo.setRotationPrior(q);
  } else if (vo.getLastFrames()) {
    svo::Quaternion q;
    if (imu.getRelativeRotationPrior(vo.getLastFrames()->getMinTimestampNanoseconds() * 1e-9,
                                    ns * 1e-9, false, q)) vo.setRotationIncrementPrior(q);
  }
  return true;
}
std::string help() {
  return "schurvins_headless --dataset ASL_ROOT --mode mono|stereo --calib YAML --params YAML [--params YAML] --output NEWDIR --rate 0|0.5|1 [--max-frames N]\n";
}
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]); FLAGS_logtostderr = true; FLAGS_minloglevel = 1;
  std::string dataset, mode, calibration, output;
  std::vector<std::string> params;
  bool output_owned = false;
  double rate = -1; size_t max_frames = std::numeric_limits<size_t>::max();
  try {
    for (int i = 1; i < argc; ++i) {
      std::string key(argv[i]);
      if (key == "--help") { std::cout << help(); return 0; }
      if (i + 1 == argc) throw std::runtime_error("missing option value");
      std::string value(argv[++i]);
      if (key == "--dataset") dataset = value;
      else if (key == "--mode") mode = value;
      else if (key == "--calib") calibration = value;
      else if (key == "--output") output = value;
      else if (key == "--params") params.push_back(value);
      else if (key == "--rate") rate = number(value);
      else if (key == "--max-frames") max_frames = static_cast<size_t>(integer(value));
      else throw std::runtime_error("unknown option: " + key);
    }
    if (dataset.empty() || output.empty() || calibration.empty() || params.empty() ||
        (mode != "mono" && mode != "stereo") || (rate != 0 && rate != 0.5 && rate != 1))
      throw std::runtime_error(help());
    struct stat metadata;
    if (::stat((dataset + "/mav0").c_str(), &metadata) == 0 && S_ISDIR(metadata.st_mode)) dataset += "/mav0";
    auto cam0 = images(dataset + "/cam0");
    std::vector<Image> cam1;
    if (mode == "stereo") {
      cam1 = images(dataset + "/cam1");
      if (cam1.size() != cam0.size()) throw std::runtime_error("stereo image counts differ");
      for (size_t i = 0; i < cam0.size(); ++i)
        if (cam0[i].ns != cam1[i].ns) throw std::runtime_error("stereo timestamps differ; exact pairing required");
    }
    auto imu_samples = inertial(dataset + "/imu0");
    YAML::Node config(YAML::NodeType::Map);
    for (const auto& path : params) {
      auto overlay = YAML::LoadFile(path);
      if (!overlay.IsMap()) throw std::runtime_error("parameter file must be a map");
      for (auto value : overlay) config[value.first.as<std::string>()] = value.second;
    }
    if (::mkdir(output.c_str(), 0755) != 0) throw std::runtime_error("output must not exist and its parent must exist");
    output_owned = true;
    config["calib_file"] = calibration; config["trace_dir"] = output;
    config["runlc"] = false; config["use_global_map"] = false;
    config["use_ceres_backend"] = false; config["automatic_reinitialization"] = false;
    config["use_imu"] = true;
    { auto effective = outputFile(output + "/effective-params.yaml"); effective << config; effective.close(); }
    setenv("RM27_NATIVE_EVENTS", (output + "/native-events.jsonl").c_str(), 1);
    std::shared_ptr<svo::FrameHandlerBase> vo;
    if (mode == "mono") vo = svo::headless::makeMono(config, nullptr);
    else vo = svo::headless::makeStereo(config, nullptr);
    auto imu = svo::headless::getImuHandler(config); vo->setImuHandler(imu); vo->start();
    auto poses = outputFile(output + "/poses.csv");
    auto frames = outputFile(output + "/frames.csv");
    poses << "timestamp_ns,tx,ty,tz,qw,qx,qy,qz\n" << std::setprecision(17);
    frames << "timestamp_ns,scheduled_ns,start_ns,end_ns,stage,quality,pose_emitted,imu_added,queue_pending_frames\n";
    const size_t selected = std::min(cam0.size(), max_frames);
    size_t imu_index = 0, pose_count = 0, prior_skipped = 0, tracking_losses = 0, imu_coverage_skipped = 0;
    bool was_tracking = false;
    const int64_t anchor = monotonic();
    for (size_t i = 0; i < selected; ++i) {
      const int64_t ns = cam0[i].ns;
      const int64_t scheduled = rate ? anchor + static_cast<int64_t>((ns - cam0[0].ns) / rate) : monotonic();
      if (rate) std::this_thread::sleep_until(Clock::time_point(std::chrono::nanoseconds(scheduled)));
      const int64_t begin = monotonic();
      // Add original samples through the first sample after this camera time,
      // preserving the edge-containing IMU horizon used by the native wrapper.
      while (imu_index < imu_samples.size() && (imu_index == 0 || imu_samples[imu_index - 1].ns <= ns)) {
        const auto& m = imu_samples[imu_index++];
        imu->addImuMeasurement(svo::ImuMeasurement(m.ns * 1e-9, m.gyro, m.accel));
      }
      std::vector<cv::Mat> bundle{cv::imread(cam0[i].path, cv::IMREAD_GRAYSCALE)};
      if (mode == "stereo") bundle.push_back(cv::imread(cam1[i].path, cv::IMREAD_GRAYSCALE));
      for (const auto& image : bundle) if (image.empty()) throw std::runtime_error("image decode failed");
      // The native edge extractor requires a strictly older sample. Reject a
      // camera lacking that history instead of inventing a pre-roll sample.
      const bool covered = imu_index > 1 && imu_samples.front().ns < ns && imu_samples[imu_index - 1].ns >= ns;
      if (!covered) ++imu_coverage_skipped;
      const bool processed = covered && prior(*vo, *imu, ns);
      if (processed) vo->addImageBundle(bundle, ns);
      else ++prior_skipped;
      bool emitted = false;
      const bool tracking = vo->stage() == svo::Stage::kTracking;
      if (was_tracking && !tracking) ++tracking_losses;
      was_tracking = tracking;
      if (processed && tracking && vo->getLastFrames() && vo->getLastFrames()->getMinTimestampNanoseconds() == ns) {
        const auto t = vo->getLastFrames()->get_T_W_B();
        const auto p = t.getPosition(); const auto q = t.getRotation().toImplementation();
        if (!p.allFinite() || !q.coeffs().allFinite()) throw std::runtime_error("native pose is nonfinite");
        poses << ns << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
              << q.w() << ',' << q.x() << ',' << q.y() << ',' << q.z() << '\n';
        ++pose_count; emitted = true;
      }
      const int64_t end = monotonic();
      size_t pending = 0;
      if (rate) {
        const int64_t horizon = cam0[0].ns + static_cast<int64_t>((end - anchor) * rate);
        auto due = std::upper_bound(cam0.begin() + i + 1, cam0.begin() + selected, horizon,
                                   [](int64_t value, const Image& frame) { return value < frame.ns; });
        pending = static_cast<size_t>(due - (cam0.begin() + i + 1));
      }
      frames << ns << ',' << scheduled << ',' << begin << ',' << end << ',' << static_cast<int>(vo->stage())
             << ',' << static_cast<int>(vo->trackingQuality()) << ',' << emitted << ',' << imu_index << ',' << pending << '\n';
      frames.flush(); poses.flush();
    }
    // Tail samples are admitted after the last selected camera, never fabricated.
    if (selected == cam0.size()) while (imu_index < imu_samples.size()) {
      const auto& m = imu_samples[imu_index++]; imu->addImuMeasurement(svo::ImuMeasurement(m.ns * 1e-9, m.gyro, m.accel));
    }
    const auto wall_ns = monotonic() - anchor;
    vo.reset(); imu.reset(); poses.close(); frames.close();
    struct rusage usage; getrusage(RUSAGE_SELF, &usage);
    auto summary = outputFile(output + "/summary.json");
    summary << "{\"schema_version\":1,\"status\":\"EXECUTION_COMPLETED\",\"runtime\":\"ROS_FREE_CPP\","
            << "\"mode\":\"" << mode << "\",\"rate\":" << rate << ",\"camera_input\":" << cam0.size()
            << ",\"camera_selected\":" << selected << ",\"imu_input\":" << imu_samples.size()
            << ",\"imu_added\":" << imu_index << ",\"pose_count\":" << pose_count
            << ",\"prior_skipped\":" << prior_skipped << ",\"tracking_losses\":" << tracking_losses
            << ",\"imu_coverage_skipped\":" << imu_coverage_skipped
            << ",\"wall_ns\":" << wall_ns << ",\"peak_rss_kib\":" << usage.ru_maxrss
            << ",\"cpu_user_s\":" << usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6
            << ",\"cpu_system_s\":" << usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6
            << ",\"metric_initialization_verified\":false,\"operational_validity\":false,"
            << "\"queue_semantics\":\"overdue scheduled replay frames; not a hardware queue\","
            << "\"pose_semantics\":\"native stage kTracking only; no accepted visual correction certificate\"}\n";
    summary.close();
    return 0;
  } catch (const std::exception& error) {
    if (output_owned) {
      // Best effort only: a full or unavailable filesystem must not mask the
      // original exception or turn a failed run into successful completion.
      std::ofstream failure(output + "/summary.json");
      failure << "{\"schema_version\":1,\"status\":\"FAIL\",\"runtime\":\"ROS_FREE_CPP\",\"failure_code\":\"RUNTIME_EXCEPTION\",\"operational_validity\":false}\n";
    }
    std::cerr << "SchurVINS headless: " << error.what() << '\n';
    return 2;
  }
}
