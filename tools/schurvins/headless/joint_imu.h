#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace svo { namespace headless { namespace joint_imu {

using Vector15 = Eigen::Matrix<double, 15, 1>;
using Matrix15 = Eigen::Matrix<double, 15, 15>;
using BiasJacobian = Eigen::Matrix<double, 9, 6>;

// SI units. Angular velocity and specific force are expressed in the IMU body.
struct Sample {
  double timestamp = 0.0;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
};
using Measurements = std::vector<Sample>;

template<class Range> Measurements fromSvoMeasurements(const Range& raw) {
  Measurements result;
  result.reserve(raw.size());
  for (const auto& measurement : raw) {
    Sample sample;
    sample.timestamp = measurement.timestamp_;
    sample.angular_velocity = measurement.angular_velocity_;
    sample.linear_acceleration = measurement.linear_acceleration_;
    result.push_back(sample);
  }
  return result;
}

struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Preserve the original double camera timestamp; never round through ns here.
  double timestamp = 0.0;
  Eigen::Quaterniond q_WB = Eigen::Quaterniond::Identity();
  Eigen::Vector3d p_WB = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_W = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
};

struct Calibration {
  // Required continuous-time densities from the active calibration, NOT
  // per-sample standard deviations. No estimator information defaults.
  double sigma_acc = 0.0;
  double sigma_gyro = 0.0;
  double sigma_ba = 0.0;
  double sigma_bg = 0.0;
  double max_gap = 0.015;
};

struct Preintegration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double ti = 0.0, tj = 0.0, dt = 0.0;
  Eigen::Quaterniond dR = Eigen::Quaterniond::Identity();
  Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
  Eigen::Vector3d beta = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg0 = Eigen::Vector3d::Zero();
  // Error order: right rotation at j, position/velocity in B_i, body ba/bg.
  Matrix15 covariance = Matrix15::Zero();
  Matrix15 sqrt_information = Matrix15::Zero();
  // Rows dR (right tangent), alpha, beta. Columns ba, bg.
  BiasJacobian bias_jacobian = BiasJacobian::Zero();
};

// Sorts raw samples (SVO supplies newest first), coalesces identical edge
// duplicates, and linearly interpolates both exact camera endpoints. Conflicting
// duplicates, missing coverage, nonfinite data and raw gaps > max_gap fail.
bool clipMeasurements(const Measurements& raw, double ti, double tj,
                      double max_gap, Measurements* clipped,
                      std::string* error = nullptr);

// Covariance comes only from the four continuous noise densities. Singular
// covariance fails explicitly; no jitter, eigenvalue floor or fake diagonal.
// The caller caches this at each bias linearization. Small perturbations in
// residual() use bias_jacobian; a new nonlinear iteration can re-preintegrate.
bool preintegrate(const Measurements& raw, double ti, double tj,
                  const Eigen::Vector3d& ba, const Eigen::Vector3d& bg,
                  const Calibration& calibration, Preintegration* result,
                  std::string* error = nullptr);

Eigen::Quaterniond expSO3(const Eigen::Vector3d& tangent);
Eigen::Vector3d logSO3(const Eigen::Quaterniond& rotation);

// q_WB maps body to world, with right rotation perturbations. Gravity is a
// global, world-frame variable supplied by the joint estimator.
Vector15 residual(const Preintegration& preintegration, const State& i,
                  const State& j, const Eigen::Vector3d& gravity_W);
Vector15 whitenedResidual(const Preintegration& preintegration, const State& i,
                          const State& j, const Eigen::Vector3d& gravity_W);

} } }  // namespace svo::headless::joint_imu
