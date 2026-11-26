#include <svo/common/frame.h>
#include <svo/common/local_feature.h>
#include <svo/common/point.h>
#include <svo/schur_vins.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

// An image association rejected by actual Backward must not survive as a
// measurement in the next visual update. No private filter state is injected.
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_minloglevel = 2;
  schur_vins::SchurVINS filter;
  filter.InitFocalLength(400.);
  filter.InitObsStddev(1.);
  filter.InitMaxState(6);
  filter.InitImuModel(.01, .001, .001, .0001);
  std::vector<svo::PointPtr> points;
  std::vector<Eigen::Vector3d> world;
  for (int y = 0; y < 5; ++y)
    for (int x = 0; x < 8; ++x) {
      const Eigen::Vector3d p((x - 3.5) * .25, (y - 2.) * .2,
                              3. + .1 * ((x + y) % 3));
      world.push_back(p);
      points.emplace_back(new svo::Point(p));
    }
  svo::FramePtr rejected_frame;
  bool actually_rejected = false, retained_at_rejection = false;
  size_t rejected_state_features = 0;
  size_t rejected_factors_next = 0, preserved_good_points = 0;
  size_t good_observations = 0, good_factors = 0;
  std::vector<svo::FramePtr> frames;
  bool point_updated_next = false;
  std::vector<int> valid;
  for (int k = 0; k < 4; ++k) {
    const double t = 1. + k * .05;
    const Eigen::Vector3d center(.5 * (t - 1), 0, 0);
    auto f = std::make_shared<svo::Frame>();
    f->timestamp_ = int64_t(std::llround(t * 1e9));
    f->id_ = 100 + k;
    f->nframe_index_ = 0;
    f->T_f_w_ = svo::Transformation(Eigen::Quaterniond::Identity(), -center);
    f->resizeFeatureStorage(points.size());
    f->num_features_ = points.size();
    for (size_t i = 0; i < points.size(); ++i) {
      Eigen::Vector3d bearing = world[i] - center;
      if (k == 2 && i == 0) bearing.y() += 8. / 400. * bearing.z();
      f->type_vec_[i] = svo::FeatureType::kCorner;
      f->f_vec_.col(i) = bearing.normalized();
      f->landmark_vec_[i] = points[i];
      f->level_vec_(i) = 0;
    }
    frames.push_back(f);
    auto b = std::make_shared<svo::FrameBundle>(std::vector<svo::FramePtr>{f});
    for (int j = -11; j <= 1; ++j)
      b->imu_datas_.emplace_back(t + j * .005, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d(0, 0, 9.81007));
    if (k == 0)
      filter.InitializeMetric(b, Eigen::Vector3d(.5, 0, 0),
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
          Eigen::Vector3d(0, 0, -9.81007));
    else filter.Forward(b);
    valid.push_back(filter.Backward(b));
    if (k == 2) {
      rejected_frame = f;
      actually_rejected = f->type_vec_[0] == svo::FeatureType::kOutlier
                       && !f->landmark_vec_[0];
      for (const auto& obs : points[0]->local_obs_)
        retained_at_rejection |= obs.second->frame == f;
      for (const auto& obs : points[1]->local_obs_)
        if (obs.second->frame == f)
          if (const auto state = obs.second->state.lock())
            for (const auto& feature : state->features)
              rejected_state_features += feature->frame == f &&
                                         feature->point.lock() == points[0];
    }
    if (k == 3) {
      point_updated_next = points[0]->last_structure_optim_ == f->id_;
      for (const auto& sf : points[0]->state_factors)
        if (const auto state = sf.second.state.lock())
          rejected_factors_next += state->frame_bundle->at(0) == rejected_frame;
      // Every uncorrupted point must retain one real observation and factor
      // per clone, plus its unchanged frontend association in every image.
      for (size_t i = 1; i < points.size(); ++i) {
        const auto& point = points[i];
        bool preserved = point->local_obs_.size() == 4 &&
                         point->state_factors.size() == 4 &&
                         point->last_structure_optim_ == f->id_;
        for (const auto& expected_frame : frames) {
          size_t observations = 0, factors = 0, state_features = 0;
          for (const auto& obs : point->local_obs_) {
            if (obs.second->frame != expected_frame) continue;
            ++observations;
            preserved &= obs.second->point.lock() == point;
            if (const auto state = obs.second->state.lock())
              for (const auto& feature : state->features)
                state_features += feature->frame == expected_frame &&
                                  feature->point.lock() == point;
          }
          for (const auto& sf : point->state_factors)
            if (const auto state = sf.second.state.lock())
              factors += state->frame_bundle->at(0) == expected_frame;
          preserved &= observations == 1 && factors == 1 && state_features == 1 &&
              expected_frame->type_vec_[i] == svo::FeatureType::kCorner &&
              expected_frame->landmark_vec_[i] == point;
          good_observations += observations;
          good_factors += factors;
        }
        preserved_good_points += preserved;
      }
    }
  }
  const bool ok = actually_rejected && !retained_at_rejection &&
      rejected_state_features == 0 && point_updated_next && rejected_factors_next == 0 &&
      preserved_good_points == 39 && good_observations == 156 &&
      good_factors == 156 && valid.back() == 40;
  std::cout << std::setprecision(17)
      << "{\"actually_rejected\":" << actually_rejected
      << ",\"retained_at_rejection\":" << retained_at_rejection
      << ",\"rejected_state_features\":" << rejected_state_features
      << ",\"point_updated_next\":" << point_updated_next
      << ",\"rejected_factors_next\":" << rejected_factors_next
      << ",\"preserved_good_points\":" << preserved_good_points
      << ",\"good_observations\":" << good_observations
      << ",\"good_factors\":" << good_factors
      << ",\"valid_counts\":[" << valid[0] << "," << valid[1] << ","
      << valid[2] << "," << valid[3] << "],\"PASS\":" << ok << "}\n";
  return !ok;
}
