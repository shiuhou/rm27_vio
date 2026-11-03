#include "joint_imu.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>

namespace svo { namespace headless { namespace joint_imu {
namespace {
using Matrix3 = Eigen::Matrix3d;
using Matrix15x12 = Eigen::Matrix<double, 15, 12>;
using Matrix12 = Eigen::Matrix<double, 12, 12>;

bool fail(const char* reason, std::string* error) {
  if (error) *error = reason;
  return false;
}

Matrix3 skew(const Eigen::Vector3d& vector) {
  Matrix3 result;
  result << 0.0, -vector.z(), vector.y(), vector.z(), 0.0,
      -vector.x(), -vector.y(), vector.x(), 0.0;
  return result;
}

bool sameReading(const Sample& a, const Sample& b) {
  return (a.angular_velocity.array() == b.angular_velocity.array()).all() &&
      (a.linear_acceleration.array() == b.linear_acceleration.array()).all();
}

Sample interpolate(const Sample& a, const Sample& b, double time) {
  const double fraction = (time - a.timestamp) / (b.timestamp - a.timestamp);
  Sample result;
  result.timestamp = time;
  result.angular_velocity = a.angular_velocity + fraction *
      (b.angular_velocity - a.angular_velocity);
  result.linear_acceleration = a.linear_acceleration + fraction *
      (b.linear_acceleration - a.linear_acceleration);
  return result;
}

// Exact discretization of a frozen midpoint linear model. The exponential and
// the Lyapunov integral are convergent power series, NOT Q*dt diagonal noise.
// Qd = integral_0^dt exp(F*s) Q exp(F^T*s) ds. Keeping the full integral retains
// acceleration->position, gyro->motion and bias-walk cross correlations even
// for an interval containing just one clipped segment.
bool discretize(const Matrix15& F, const Matrix15& Q, double dt,
                Matrix15* Phi, Matrix15* Qd) {
  const Matrix15 A = F * dt;
  // Scale the continuous interval for stable series evaluation under motion.
  const double norm = A.cwiseAbs().rowwise().sum().maxCoeff();
  if (!std::isfinite(norm)) return false;
  int squarings = 0;
  double scale = 1.0;
  while (norm / scale > 0.5) {
    scale *= 2.0;
    if (++squarings > 30) return false;
  }
  const Matrix15 B = A / scale;
  Matrix15 transition = Matrix15::Identity();
  Matrix15 transition_term = Matrix15::Identity();
  Matrix15 integral = Q;
  Matrix15 integral_term = Q;
  for (int order = 1; order <= 32; ++order) {
    transition_term = (transition_term * B / double(order)).eval();
    transition += transition_term;
    integral_term = ((B * integral_term + integral_term * B.transpose()) /
                     double(order + 1)).eval();
    integral += integral_term;
    const double epsilon = std::numeric_limits<double>::epsilon();
    if (transition_term.cwiseAbs().maxCoeff() <= epsilon * transition.cwiseAbs().maxCoeff() &&
        integral_term.cwiseAbs().maxCoeff() <= epsilon * integral.cwiseAbs().maxCoeff())
      break;
  }
  integral *= dt / scale;
  for (int k = 0; k < squarings; ++k) {
    integral = (transition * integral * transition.transpose() + integral).eval();
    transition = (transition * transition).eval();
  }
  *Phi = transition;
  *Qd = 0.5 * (integral + integral.transpose()).eval();
  return Phi->allFinite() && Qd->allFinite();
}

bool integrateClipped(const Measurements& samples, const Eigen::Vector3d& ba,
                      const Eigen::Vector3d& bg, const Calibration& calibration,
                      bool covariance, Preintegration* result) {
  result->dR = Eigen::Quaterniond::Identity();
  result->alpha.setZero();
  result->beta.setZero();
  result->covariance.setZero();
  Matrix12 noise = Matrix12::Zero();
  noise.diagonal().segment<3>(0).setConstant(calibration.sigma_gyro * calibration.sigma_gyro);
  noise.diagonal().segment<3>(3).setConstant(calibration.sigma_acc * calibration.sigma_acc);
  noise.diagonal().segment<3>(6).setConstant(calibration.sigma_ba * calibration.sigma_ba);
  noise.diagonal().segment<3>(9).setConstant(calibration.sigma_bg * calibration.sigma_bg);
  for (size_t k = 1; k < samples.size(); ++k) {
    const Sample& previous = samples[k - 1];
    const Sample& next = samples[k];
    const double dt = next.timestamp - previous.timestamp;
    const Eigen::Vector3d omega = 0.5 * (previous.angular_velocity + next.angular_velocity) - bg;
    const Eigen::Vector3d acceleration = 0.5 *
        (previous.linear_acceleration + next.linear_acceleration) - ba;
    const Eigen::Quaterniond midpoint = result->dR * expSO3(omega * (0.5 * dt));
    const Matrix3 R = midpoint.toRotationMatrix();
    const Eigen::Vector3d acceleration_i = R * acceleration;
    result->alpha += result->beta * dt + 0.5 * acceleration_i * dt * dt;
    result->beta += acceleration_i * dt;
    result->dR = (result->dR * expSO3(omega * dt)).normalized();
    if (covariance) {
      // x=[theta_j(right), alpha_i, beta_i, delta_ba, delta_bg].
      // theta_dot=-[omega]x theta-dbg-ng;
      // alpha_dot=beta;
      // beta_dot=-R[a]x theta-R*dba-R*na; ba_dot=nba; bg_dot=nbg.
      Matrix15 F = Matrix15::Zero();
      F.block<3, 3>(0, 0) = -skew(omega);
      F.block<3, 3>(0, 12) = -Matrix3::Identity();
      F.block<3, 3>(3, 6) = Matrix3::Identity();
      F.block<3, 3>(6, 0) = -R * skew(acceleration);
      F.block<3, 3>(6, 9) = -R;
      Matrix15x12 G = Matrix15x12::Zero();
      G.block<3, 3>(0, 0) = -Matrix3::Identity();
      G.block<3, 3>(6, 3) = -R;
      G.block<3, 3>(9, 6) = Matrix3::Identity();
      G.block<3, 3>(12, 9) = Matrix3::Identity();
      Matrix15 Phi, Qd;
      if (!discretize(F, G * noise * G.transpose(), dt, &Phi, &Qd)) return false;
      const Matrix15 propagated = Phi * result->covariance * Phi.transpose() + Qd;
      result->covariance = 0.5 * (propagated + propagated.transpose()).eval();
    }
  }
  return result->dR.coeffs().allFinite() && result->alpha.allFinite() &&
      result->beta.allFinite() && result->covariance.allFinite();
}
}  // namespace

Eigen::Quaterniond expSO3(const Eigen::Vector3d& tangent) {
  const double angle = tangent.norm();
  if (angle < 1e-10) {
    // Second-order scalar term and sinc expansion avoid normalizing a zero axis.
    const double angle2 = tangent.squaredNorm();
    const double scale = 0.5 - angle2 / 48.0;
    return Eigen::Quaterniond(1.0 - angle2 / 8.0, scale * tangent.x(),
                              scale * tangent.y(), scale * tangent.z()).normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, tangent / angle));
}

Eigen::Vector3d logSO3(const Eigen::Quaterniond& rotation) {
  Eigen::Quaterniond q = rotation.normalized();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const double sine = q.vec().norm();
  if (sine < 1e-10) return 2.0 * q.vec();
  return (2.0 * std::atan2(sine, q.w()) / sine) * q.vec();
}

bool clipMeasurements(const Measurements& raw, double ti, double tj,
                      double max_gap, Measurements* clipped, std::string* error) {
  if (error) error->clear();
  if (!clipped) return fail("missing clipped-measurement output", error);
  clipped->clear();
  if (!(std::isfinite(ti) && std::isfinite(tj) && tj > ti &&
        std::isfinite(max_gap) && max_gap > 0.0))
    return fail("invalid IMU integration interval or gap limit", error);
  Measurements ordered = raw;
  for (const auto& sample : ordered) {
    if (!(std::isfinite(sample.timestamp) && sample.angular_velocity.allFinite() &&
          sample.linear_acceleration.allFinite()))
      return fail("nonfinite raw IMU sample", error);
  }
  std::sort(ordered.begin(), ordered.end(), [](const Sample& a, const Sample& b) {
    return a.timestamp < b.timestamp;
  });
  Measurements unique;
  unique.reserve(ordered.size());
  for (const auto& sample : ordered) {
    if (!unique.empty() && sample.timestamp == unique.back().timestamp) {
      if (!sameReading(sample, unique.back()))
        return fail("conflicting IMU samples share an exact timestamp", error);
    } else {
      unique.push_back(sample);
    }
  }
  if (unique.size() < 2 || unique.front().timestamp > ti || unique.back().timestamp < tj)
    return fail("raw IMU samples do not cover both camera endpoints", error);
  for (size_t k = 1; k < unique.size(); ++k) {
    const Sample& a = unique[k - 1];
    const Sample& b = unique[k];
    if (b.timestamp <= ti || a.timestamp >= tj) continue;
    // Check the raw gap, including a gap straddling an interpolated endpoint.
    if (b.timestamp - a.timestamp > max_gap)
      return fail("raw IMU coverage gap exceeds calibration limit", error);
    const double left = std::max(a.timestamp, ti);
    const double right = std::min(b.timestamp, tj);
    if (clipped->empty())
      clipped->push_back(left == a.timestamp ? a : interpolate(a, b, left));
    clipped->push_back(right == b.timestamp ? b : interpolate(a, b, right));
  }
  if (clipped->size() < 2 || clipped->front().timestamp != ti || clipped->back().timestamp != tj)
    return fail("failed exact camera-endpoint IMU clipping", error);
  return true;
}

bool preintegrate(const Measurements& raw, double ti, double tj,
                  const Eigen::Vector3d& ba, const Eigen::Vector3d& bg,
                  const Calibration& calibration, Preintegration* result,
                  std::string* error) {
  if (error) error->clear();
  if (!result) return fail("missing preintegration output", error);
  // Clear output on all failure paths so an earlier successful factor cannot leak.
  *result = Preintegration();
  const double densities[] = {calibration.sigma_acc, calibration.sigma_gyro,
                              calibration.sigma_ba, calibration.sigma_bg};
  for (double density : densities)
    if (!std::isfinite(density) || density < 0.0)
      return fail("invalid continuous IMU noise density", error);
  if (!ba.allFinite() || !bg.allFinite()) return fail("nonfinite IMU bias", error);
  Measurements samples;
  if (!clipMeasurements(raw, ti, tj, calibration.max_gap, &samples, error)) return false;
  Preintegration integrated;
  integrated.ti = ti;
  integrated.tj = tj;
  integrated.dt = tj - ti;
  integrated.ba0 = ba;
  integrated.bg0 = bg;
  if (!integrateClipped(samples, ba, bg, calibration, true, &integrated))
    return fail("nonfinite IMU integration or covariance propagation", error);
  Eigen::LLT<Matrix15> llt(integrated.covariance);
  if (llt.info() != Eigen::Success)
    return fail("rank-deficient preintegrated IMU covariance", error);
  integrated.sqrt_information = llt.matrixL().solve(Matrix15::Identity());
  if (!integrated.sqrt_information.allFinite())
    return fail("nonfinite preintegrated IMU covariance whitening", error);
  for (int column = 0; column < 6; ++column) {
    const double step = column < 3 ? 1e-5 : 1e-6;
    Eigen::Vector3d ba_plus = ba, ba_minus = ba, bg_plus = bg, bg_minus = bg;
    if (column < 3) { ba_plus[column] += step; ba_minus[column] -= step; }
    else { bg_plus[column - 3] += step; bg_minus[column - 3] -= step; }
    Preintegration plus, minus;
    if (!integrateClipped(samples, ba_plus, bg_plus, calibration, false, &plus) ||
        !integrateClipped(samples, ba_minus, bg_minus, calibration, false, &minus))
      return fail("nonfinite IMU bias Jacobian", error);
    integrated.bias_jacobian.block<3, 1>(0, column) =
        (logSO3(integrated.dR.conjugate() * plus.dR) -
         logSO3(integrated.dR.conjugate() * minus.dR)) / (2.0 * step);
    integrated.bias_jacobian.block<3, 1>(3, column) = (plus.alpha - minus.alpha) / (2.0 * step);
    integrated.bias_jacobian.block<3, 1>(6, column) = (plus.beta - minus.beta) / (2.0 * step);
  }
  if (!integrated.bias_jacobian.allFinite()) return fail("nonfinite IMU bias Jacobian", error);
  *result = integrated;
  return true;
}

Vector15 residual(const Preintegration& integrated, const State& i,
                  const State& j, const Eigen::Vector3d& gravity_W) {
  Eigen::Matrix<double, 6, 1> db;
  db << i.ba - integrated.ba0, i.bg - integrated.bg0;
  const Eigen::Matrix<double, 9, 1> correction = integrated.bias_jacobian * db;
  const Eigen::Quaterniond corrected_rotation = integrated.dR * expSO3(correction.head<3>());
  Vector15 result;
  result.segment<3>(0) = logSO3(corrected_rotation.conjugate() * i.q_WB.conjugate() * j.q_WB);
  result.segment<3>(3) = i.q_WB.conjugate() *
      (j.p_WB - i.p_WB - i.v_W * integrated.dt -
       0.5 * gravity_W * integrated.dt * integrated.dt) - integrated.alpha - correction.segment<3>(3);
  result.segment<3>(6) = i.q_WB.conjugate() *
      (j.v_W - i.v_W - gravity_W * integrated.dt) - integrated.beta - correction.tail<3>();
  result.segment<3>(9) = j.ba - i.ba;
  result.segment<3>(12) = j.bg - i.bg;
  return result;
}

Vector15 whitenedResidual(const Preintegration& integrated, const State& i,
                          const State& j, const Eigen::Vector3d& gravity_W) {
  return integrated.sqrt_information * residual(integrated, i, j, gravity_W);
}

} } }  // namespace svo::headless::joint_imu
