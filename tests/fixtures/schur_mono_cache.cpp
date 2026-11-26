// Regression for overlap keypoints after a visual map geometry update.
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <vikit/cameras.h>
#include <vikit/cameras/camera_geometry.h>
#include <vikit/cameras/pinhole_projection.h>
#include <vikit/cameras/no_distortion.h>
#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

int main() {
  const auto camera = vk::cameras::PinholeGeometry::createTestCamera();
  svo::Frame frame(1, 1000000000LL, camera, svo::Transformation());
  const double cu = camera->imageWidth() / 2.;
  const double cv = camera->imageHeight() / 2.;
  const double left = std::min(cu, cv) / 4.;
  const std::vector<Eigen::Vector2d> pixels = {
      {cu, cv}, {cu * 1.75, cv * 1.75}, {cu * 1.75, cv * .25},
      {left, cv * .25}, {left, cv * 1.75}};
  std::vector<svo::PointPtr> points;
  frame.resizeFeatureStorage(pixels.size());
  frame.num_features_ = pixels.size();
  for (size_t i = 0; i < pixels.size(); ++i) {
    points.emplace_back(new svo::Point(Eigen::Vector3d(i * .1, i * .2, 3.)));
    frame.type_vec_[i] = svo::FeatureType::kCorner;
    frame.px_vec_.col(i) = pixels[i];
    frame.landmark_vec_[i] = points[i];
  }
  frame.setKeyPoints();
  std::set<int> selected;
  for (const auto& entry : frame.key_pts_) selected.insert(entry.first);
  const bool populated = selected.size() == 5 && *selected.begin() >= 0;

  // Bundle refinement changes map coordinates without changing image pixels.
  for (size_t i = 0; i < points.size(); ++i)
    points[i]->pos_ += Eigen::Vector3d(.2 + i, .4 - i, .6 + .5 * i);
  frame.resetKeyPoints();
  frame.setKeyPoints();
  bool refreshed = populated;
  for (const auto& entry : frame.key_pts_) {
    const int index = entry.first;
    refreshed = refreshed && index >= 0 && size_t(index) < points.size();
    if (index >= 0 && size_t(index) < points.size())
      refreshed = refreshed && (entry.second - points[index]->pos()).norm() < 1e-12;
  }

  // No removed feature index or old world coordinate may survive an empty frame.
  frame.clearFeatureStorage();
  for (auto& entry : frame.key_pts_) {
    entry.first = 1000;
    entry.second = Eigen::Vector3d(1., 2., 3.);
  }
  frame.resetKeyPoints();
  frame.setKeyPoints();
  bool empty_reset = frame.key_pts_.size() == 5;
  for (const auto& entry : frame.key_pts_)
    empty_reset = empty_reset && entry.first == -1 && entry.second.isZero(0.);

  bool explicit_ids = true;
  for (const int id : {777, 1234, 9876}) {
    svo::FramePtr child(new svo::Frame(id, 2000000000LL, camera, svo::Transformation()));
    svo::FrameBundle wrapper({child}, id);
    explicit_ids = explicit_ids && wrapper.getBundleId() == id && child->bundleId() == id;
  }
  const bool pass = populated && refreshed && empty_reset && explicit_ids;
  std::cout << "{\"populated\":" << populated << ",\"refreshed\":" << refreshed
            << ",\"explicit_ids\":" << explicit_ids << ",\"empty_reset\":" << empty_reset << ",\"PASS\":" << pass << "}\n";
  return !pass;
}
