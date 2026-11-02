// Synthetic boundary setup shared by native Mono quality/window regressions.
// Production classification, propagation and filtering remain in schurvins_core.
#pragma once
#include <svo/frame_handler_mono.h>
#include <svo/schur_vins.h>
#include <svo/initialization.h>
#include <svo/common/point.h>
#include <svo/common/camera.h>
#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection.h>
#include <svo/reprojector.h>
#include <svo/tracker/feature_tracking_types.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdexcept>

inline void require(bool condition, const char* behavior) {
  if (!condition) throw std::runtime_error(behavior);
}

class MonoProbe : public svo::FrameHandlerMono {
 public:
  explicit MonoProbe(const svo::CameraBundlePtr& cameras)
      : FrameHandlerMono(baseOptions(), depthOptions(), detectorOptions(),
                         svo::InitializationOptions(), reprojectorOptions(),
                         svo::FeatureTrackerOptions(), cameras) {}

  void setPreviousImagePopulation(size_t count, bool keyframe = false) {
    num_obs_last_ = count;
    last_frames_ = makeFrame(4);
    if (keyframe) last_frames_->setKeyframe();
  }
  bool evaluateQuality(size_t count) {
    setTrackingQuality(count);
    return tracking_quality_ == svo::TrackingQuality::kGood;
  }
  bool evaluateBaseQuality(size_t count) {
    FrameHandlerBase::setTrackingQuality(count);
    return tracking_quality_ == svo::TrackingQuality::kGood;
  }
  size_t metricObservationHistory() const { return last_metric_observations_; }
  bool clonesPending() const { return schur_vins_->MonoClonesPending(); }
  void resetEpoch() { resetAll(); }

  void beginMetricEpoch() {
    points_.clear();
    const auto T_B_C = cams_->get_T_C_B(0).inverse();
    for (int i = 0; i < 100; ++i) {
      const Eigen::Vector3d p((i % 10 - 4.5) * .12,
                              (i / 10 - 4.5) * .1, 3. + .003 * i);
      points_.emplace_back(new svo::Point(T_B_C * p));
    }
    auto first = makeFrame(0);
    schur_vins_->InitializeMetric(first, Eigen::Vector3d(1, 0, 0),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0, 0, -9.81007));
    schur_vins_->Backward(first);
    last_frames_ = first;
  }
  void matureMetricWindow() {
    for (int step = 1; step <= 2; ++step) {
      auto bundle = makeFrame(step);
      schur_vins_->Forward(bundle);
      schur_vins_->Backward(bundle);
      last_frames_ = bundle;
    }
    require(!clonesPending(), "real mono cloned states did not mature");
  }

 protected:
  svo::FrameBundlePtr makeFrame(int step) {
    const auto T_B_C = cams_->get_T_C_B(0).inverse();
    const svo::Transformation body(svo::Quaternion(),
                                    Eigen::Vector3d(.05 * step, 0, 0));
    svo::FramePtr frame(new svo::Frame(step,
        1000000000LL + 50000000LL * step, cams_->getCameraShared(0), body * T_B_C));
    frame->set_T_cam_imu(cams_->get_T_C_B(0));
    frame->setNFrameIndex(0);
    frame->resizeFeatureStorage(points_.size());
    frame->num_features_ = points_.size();
    for (size_t i = 0; i < points_.size(); ++i) {
      const auto p = frame->T_f_w_ * points_[i]->pos();
      frame->f_vec_.col(i) = p.normalized();
      frame->level_vec_(i) = 0;
      frame->landmark_vec_[i] = points_[i];
      frame->track_id_vec_(i) = points_[i]->id();
      frame->type_vec_[i] = svo::FeatureType::kCorner;
      points_[i]->addObservation(frame, i);
    }
    svo::FrameBundlePtr bundle(new svo::FrameBundle({frame}));
    for (int i = -10; i <= 1; ++i) {
      bundle->imu_datas_.emplace_back(bundle->getMinTimestampSeconds() + .005 * i,
          Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, 9.81007));
    }
    bundle->imu_ready_ = true;
    return bundle;
  }

 private:
  static svo::BaseOptions baseOptions() {
    svo::BaseOptions options;
    options.quality_min_fts = 40;
    options.quality_max_fts_drop = 40;
    options.window_size_ = 5;
    return options;
  }
  static svo::DepthFilterOptions depthOptions() {
    svo::DepthFilterOptions options;
    options.use_threaded_depthfilter = false;
    return options;
  }
  static svo::DetectorOptions detectorOptions() {
    svo::DetectorOptions options;
    options.cell_size = 30;
    return options;
  }
  static svo::ReprojectorOptions reprojectorOptions() {
    svo::ReprojectorOptions options;
    options.cell_size = 30;
    return options;
  }
  std::vector<svo::PointPtr> points_;
};
