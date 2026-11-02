// Real production Reset/destructor must break frame-landmark ownership cycles.
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/schur_vins.h>
#include <iostream>
#include <memory>
#include <stdexcept>

struct Lifetimes {
  std::weak_ptr<svo::Frame> frame;
  std::weak_ptr<svo::Point> point;
  bool released() const { return frame.expired() && point.expired(); }
};

// Register a real production Schur observation through public methods. No
// private-container injection or copied Reset operations are used.
Lifetimes register_observation(schur_vins::SchurVINS& filter) {
  auto frame = std::make_shared<svo::Frame>();
  frame->timestamp_ = 1000000000;
  frame->id_ = 101;
  frame->nframe_index_ = 0;
  frame->resizeFeatureStorage(1);
  frame->num_features_ = 1;
  frame->type_vec_[0] = svo::FeatureType::kCorner;
  frame->f_vec_.col(0) = Eigen::Vector3d(0, 0, 1);
  auto point = std::make_shared<svo::Point>(Eigen::Vector3d(0, 0, 2));
  frame->landmark_vec_[0] = point;
  auto bundle = std::make_shared<svo::FrameBundle>(std::vector<svo::FramePtr>{frame});
  bundle->imu_datas_.emplace_back(.995, Eigen::Vector3d::Zero(), Eigen::Vector3d(0,0,9.81007));
  bundle->imu_datas_.emplace_back(1.005, Eigen::Vector3d::Zero(), Eigen::Vector3d(0,0,9.81007));
  filter.InitializeMetric(bundle, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0,0,-9.81007));
  if (filter.Backward(bundle) != 1) throw std::runtime_error("registration did not occur");
  return {frame, point};
}

int main() {
  bool reset_released, destructor_released;
  {
    schur_vins::SchurVINS filter;
    auto refs = register_observation(filter);
    filter.Reset();
    reset_released = refs.released();
  }
  {
    auto filter = std::make_unique<schur_vins::SchurVINS>();
    auto refs = register_observation(*filter);
    filter.reset();
    destructor_released = refs.released();
  }
  std::cout << "{\"production_Reset_released\":" << reset_released
      << ",\"production_destructor_released\":" << destructor_released << "}\n";
  return !(reset_released && destructor_released);
}
