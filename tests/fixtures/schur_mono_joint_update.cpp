#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/schur_vins.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

// Public production API only: three physically consistent mono clones,
// followed by a real visual Schur correction. The points have small initial
// depth errors. No private filter containers, cached Jacobians or state are
// injected. No FrameHandler/Base optimizer is called after Backward.
int main() {
  schur_vins::SchurVINS filter;
  filter.InitFocalLength(400.0);
  filter.InitObsStddev(1.0);
  filter.InitMaxState(5);
  filter.InitImuModel(.01, .001, .001, .0001);
  std::vector<svo::PointPtr> points;
  std::vector<Eigen::Vector3d> world;
  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 4; ++x) {
      Eigen::Vector3d p((x - 1.5) * .5, (y - 1.) * .5, 3. + .2 * ((x + y) % 3));
      world.push_back(p);
      points.push_back(std::make_shared<svo::Point>(p * 1.03));
    }
  }
  std::vector<int> valid;
  bool premature_update = false;
  for (int k = 0; k < 3; ++k) {
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
      f->type_vec_[i] = svo::FeatureType::kCorner;
      f->f_vec_.col(i) = (world[i] - center).normalized();
      f->landmark_vec_[i] = points[i];
    }
    auto b = std::make_shared<svo::FrameBundle>(std::vector<svo::FramePtr>{f});
    for (int j = -11; j <= 1; ++j) {
      b->imu_datas_.emplace_back(t + j * .005, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d(0, 0, 9.81007));
    }
    if (k == 0) {
      filter.InitializeMetric(b, Eigen::Vector3d(.5, 0, 0), Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, -9.81007));
    } else {
      filter.Forward(b);
    }
    valid.push_back(filter.Backward(b));
    if (k < 2) {
      for (const auto& p : points) premature_update |= p->ekf_init;
    }
  }
  size_t updated = 0;
  double max_cov_error = 0, max_point_shift = 0, min_determinant = 1e100;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i];
    min_determinant = std::min(min_determinant, p->V.determinant());
    if (!p->ekf_init) continue;
    ++updated;
    // Information identity for exactly one conditional point update from
    // the production .01*I point prior, no outside-window keyframe terms.
    const Eigen::Matrix3d expected = (100. * Eigen::Matrix3d::Identity() + p->V).inverse();
    max_cov_error = std::max(max_cov_error, (p->cov - expected).norm());
    max_point_shift = std::max(max_point_shift, (p->pos() - world[i] * 1.03).norm());
  }
  const bool ok = !premature_update && updated == points.size() &&
      max_cov_error < 1e-7 && max_point_shift > 1e-8;
  std::cout << std::setprecision(17)
      << "{\"no_update_before_three_real_clones\":" << !premature_update
      << ",\"points\":" << points.size() << ",\"updated_inside_Backward\":" << updated
      << ",\"max_exactly_once_covariance_error\":" << max_cov_error
      << ",\"max_point_shift_m\":" << max_point_shift
      << ",\"min_V_determinant\":" << min_determinant
      << ",\"valid_counts\":[" << valid[0] << "," << valid[1] << "," << valid[2]
      << "],\"PASS\":" << ok << "}\n";
  return !ok;
}
