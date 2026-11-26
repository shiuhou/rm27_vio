#pragma once
#include "joint_window.h"
#include <svo/common/frame.h>
#include <svo/common/imu_calibration.h>
#include <svo/rm27_mono_alignment.h>
#include <memory>
#include <map>
namespace schur_vins {
// Raw factors, not output means/covariance, cross the metric commit boundary.
class JointRuntime {
 public:
  JointRuntime(const svo::ImuCalibration&, const svo::ImuInitialization&,
               double focal, double sigma, int tracking_frames);
  struct SeedReport { size_t expired_points=0; double max_similarity_error=0; };
  const SeedReport& seedReport() const { return seed_report_; }
  bool bootstrap(const svo::FrameBundle::Ptr&, const rm27::MonoAlignment&);
  void forward(const svo::FrameBundle::Ptr&);
  int backward(const svo::FrameBundle::Ptr&);
  void manage();
  void commitGeometry() { exportGeometry(); }
  double bootstrapScale() const { return bootstrap_scale_; }
  const Eigen::Vector3d& bootstrapOrigin() const { return origin_; }
  bool qualified() const { return ready_ && !failed_; }
  bool ready() const { return ready_; }
  bool corrected() const { return corrected_; }
  const JointWindow& window() const { return window_; }
 private:
  bool append(const svo::FrameBundle::Ptr&, bool metric);
  void exportGeometry();
  void pruneOwners();
  bool seedMetric(const rm27::MonoAlignmentResult&);
  bool qualify(const rm27::MonoAlignment&, std::string*);
  JointWindow window_;
  SeedReport seed_report_;
  svo::headless::joint_imu::Calibration calibration_;
  svo::ImuInitialization initial_;
  double focal_, sigma_;
  double bootstrap_scale_=1.;
  int tracking_frames_;
  bool ready_=false, corrected_=false, seeded_=false, failed_=false;
  Eigen::Vector3d origin_=Eigen::Vector3d::Zero();
  std::map<int64_t,svo::FrameBundle::Ptr> frames_;
  std::map<int64_t,std::weak_ptr<svo::Point>> points_;
};
}
