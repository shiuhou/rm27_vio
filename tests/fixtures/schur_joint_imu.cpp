#include "joint_imu.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace imu = svo::headless::joint_imu;
using Eigen::Vector3d;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const char* message) {
  if (!(std::abs(actual - expected) <= tolerance)) {
    std::cerr << message << ": actual=" << actual << " expected=" << expected
              << " tolerance=" << tolerance << '\n';
    throw std::runtime_error(message);
  }
}
imu::Calibration calibration() {
  // Euroc active calibration YAML continuous densities, not sample variance.
  imu::Calibration c;
  c.sigma_acc = 8.0e-3;
  c.sigma_gyro = 12.0e-4;
  c.sigma_ba = 0.1;
  c.sigma_bg = 0.03;
  return c;
}
imu::Measurements samples(const std::vector<double>& times, const Vector3d& a,
                          const Vector3d& omega, const Vector3d& ba = Vector3d::Zero(),
                          const Vector3d& bg = Vector3d::Zero()) {
  imu::Measurements result;
  for (double time : times) {
    imu::Sample sample;
    sample.timestamp = time;
    sample.linear_acceleration = a + ba;
    sample.angular_velocity = omega + bg;
    result.push_back(sample);
  }
  return result;
}
imu::Preintegration integrate(const imu::Measurements& raw, double ti, double tj,
                             const Vector3d& ba = Vector3d::Zero(),
                             const Vector3d& bg = Vector3d::Zero()) {
  imu::Preintegration result;
  std::string error;
  if (!imu::preintegrate(raw, ti, tj, ba, bg, calibration(), &result, &error))
    throw std::runtime_error(error);
  return result;
}
imu::State predicted(const imu::State& i, const imu::Preintegration& p, const Vector3d& g) {
  imu::State j = i;
  j.timestamp = p.tj;
  j.q_WB = (i.q_WB * p.dR).normalized();
  j.p_WB = i.p_WB + i.v_W * p.dt + 0.5 * g * p.dt * p.dt + i.q_WB * p.alpha;
  j.v_W = i.v_W + g * p.dt + i.q_WB * p.beta;
  return j;
}

void constantAccelerationAndFrames() {
  const double ti = 3.002, tj = 3.049, T = tj - ti;
  const Vector3d a(2.3, -0.7, 9.6), ba(.04, -.03, .02), bg(.005, .003, -.004);
  const auto raw = samples({3.0, 3.004, 3.011, 3.016, 3.024, 3.035, 3.042, 3.05},
                           a, Vector3d::Zero(), ba, bg);
  const auto p = integrate(raw, ti, tj, ba, bg);
  near((p.alpha - .5 * a * T * T).norm(), 0.0, 1e-14, "constant SI force position");
  near((p.beta - a * T).norm(), 0.0, 1e-14, "constant SI force velocity");
  near(imu::logSO3(p.dR).norm(), 0.0, 1e-14, "bias-subtracted gyro identity");
  near(p.dt, T, 0.0, "exact double camera interval");
  near((p.bias_jacobian.block<3,3>(3,0) + .5*T*T*Eigen::Matrix3d::Identity()).norm(),
       0.0, 2e-10, "accelerometer bias position Jacobian");
  near((p.bias_jacobian.block<3,3>(6,0) + T*Eigen::Matrix3d::Identity()).norm(),
       0.0, 2e-10, "accelerometer bias velocity Jacobian");
  near((p.bias_jacobian.block<3,3>(0,3) + T*Eigen::Matrix3d::Identity()).norm(),
       0.0, 2e-10, "gyro bias rotation Jacobian");
  imu::State i;
  i.timestamp = ti;
  i.q_WB = imu::expSO3(Vector3d(.3, -.2, .1));
  i.p_WB = Vector3d(1., -2., .4);
  i.v_W = Vector3d(.2, -.4, .1);
  i.ba = ba; i.bg = bg;
  const Vector3d g(0., 0., -9.8082);
  const auto j = predicted(i, p, g);
  near(imu::residual(p, i, j, g).norm(), 0., 2e-15, "world/body residual convention");
  near(imu::whitenedResidual(p, i, j, g).norm(), 0., 1e-9, "whitened predicted residual");
  auto drifted = j;
  drifted.ba += Vector3d(.01, -.02, .03);
  drifted.bg += Vector3d(.003, -.004, .005);
  near((imu::residual(p, i, drifted, g).segment<3>(9) - (drifted.ba - i.ba)).norm(),
       0., 1e-16, "body accelerometer bias random walk residual");
  near((imu::residual(p, i, drifted, g).tail<3>() - (drifted.bg - i.bg)).norm(),
       0., 1e-16, "body gyro bias random walk residual");
}

void constantRotationVaryingDt() {
  std::vector<double> times{0.};
  int k = 0;
  while (times.back() < .08) times.push_back(times.back() + .001 + (k++ % 4)*.0004);
  const double ti = .0007, tj = .0793, T = tj-ti, w = 2.4, a = 3.;
  const auto raw = samples(times, Vector3d(a,0.,0.), Vector3d(0.,0.,w));
  const auto p = integrate(raw, ti, tj);
  near(imu::logSO3(p.dR.conjugate() * imu::expSO3(Vector3d(0.,0.,w*T))).norm(),
       0., 1e-14, "constant body gyro with variable exact dt");
  const Vector3d beta(a*std::sin(w*T)/w, a*(1.-std::cos(w*T))/w, 0.);
  const Vector3d alpha(a*(1.-std::cos(w*T))/(w*w),
                       a*(T/w-std::sin(w*T)/(w*w)), 0.);
  near((p.beta-beta).norm(), 0., 4e-7, "rotating SI body force velocity integral");
  near((p.alpha-alpha).norm(), 0., 3e-7, "rotating SI body force position integral");
  imu::State i;
  i.timestamp=ti;
  i.q_WB=imu::expSO3(Vector3d(.5,-.2,.1));
  const auto j=predicted(i,p,Vector3d::Zero());
  near(imu::residual(p,i,j,Vector3d::Zero()).norm(), 0., 1e-15,
       "q_WB rotation composes on right");
  auto wrong=j;
  wrong.q_WB=p.dR*i.q_WB;
  require(imu::residual(p,i,wrong,Vector3d::Zero()).head<3>().norm() > .02,
          "gyro frame test must reject left composition");
}

void continuousCovariance() {
  const auto c=calibration();
  const double T=.06;
  const auto p=integrate(samples({0., .005, .013, .021, .032, .041, .051, T},
                                 Vector3d::Zero(), Vector3d::Zero()), 0., T);
  const double qa=c.sigma_acc*c.sigma_acc, qg=c.sigma_gyro*c.sigma_gyro;
  const double qba=c.sigma_ba*c.sigma_ba, qbg=c.sigma_bg*c.sigma_bg;
  for (int axis=0; axis<3; ++axis) {
    near(p.covariance(axis,axis), qg*T+qbg*std::pow(T,3)/3., 1e-18,
         "continuous gyro variance incl bias random walk");
    near(p.covariance(3+axis,3+axis), qa*std::pow(T,3)/3.+qba*std::pow(T,5)/20.,
         1e-20, "continuous position variance incl bias random walk");
    near(p.covariance(6+axis,6+axis), qa*T+qba*std::pow(T,3)/3., 1e-18,
         "continuous velocity variance incl bias random walk");
    near(p.covariance(3+axis,6+axis), qa*T*T/2.+qba*std::pow(T,4)/8.,
         1e-19, "position-velocity covariance");
    near(p.covariance(3+axis,9+axis), -qba*std::pow(T,3)/6., 1e-18,
         "position-accelerometer-bias covariance");
    near(p.covariance(6+axis,9+axis), -qba*T*T/2., 1e-17,
         "velocity-accelerometer-bias covariance");
    near(p.covariance(axis,12+axis), -qbg*T*T/2., 1e-18,
         "rotation-gyro-bias covariance");
    near(p.covariance(9+axis,9+axis), qba*T, 1e-17, "accelerometer bias random walk density");
    near(p.covariance(12+axis,12+axis), qbg*T, 1e-18, "gyro bias random walk density");
  }
  near((p.covariance-p.covariance.transpose()).norm(), 0., 1e-20, "covariance symmetry");
  Eigen::SelfAdjointEigenSolver<imu::Matrix15> eig(p.covariance);
  require(eig.info()==Eigen::Success && eig.eigenvalues().minCoeff()>0., "full15 covariance PSD/rank");
  near((p.sqrt_information*p.covariance*p.sqrt_information.transpose()-imu::Matrix15::Identity()).norm(),
       0., 2e-12, "LLT whitens full correlated covariance");
  const auto one=integrate(samples({0.,.01}, Vector3d::Zero(), Vector3d::Zero()), 0., .01);
  require(one.covariance.block<3,3>(3,6).norm()>0. && one.covariance.block<3,3>(0,12).norm()>0.,
          "one clipped segment retains continuous cross covariance");
  const auto dense=integrate(samples({0.,.003,.01}, Vector3d::Zero(), Vector3d::Zero()),0.,.01);
  near((one.covariance-dense.covariance).norm(),0.,1e-18,
       "continuous density covariance independent of sample splitting");
  const auto dynamic=integrate(samples({0.,.007,.014,.022,.032,.043,.05},
                                      Vector3d(2.,-1.,9.),Vector3d(.4,-.6,.8)),0.,.05);
  require(dynamic.covariance.block<3,3>(0,6).norm()>1e-10 &&
          dynamic.covariance.block<3,3>(3,12).norm()>1e-10,
          "gyro-to-motion full covariance correlations");
  Eigen::SelfAdjointEigenSolver<imu::Matrix15> dynamic_eig(dynamic.covariance);
  require(dynamic_eig.info()==Eigen::Success && dynamic_eig.eigenvalues().minCoeff()>0.,
          "dynamic full15 covariance positive definite");
  imu::Calibration zero=c;
  zero.sigma_acc=zero.sigma_gyro=zero.sigma_ba=zero.sigma_bg=0.;
  imu::Preintegration invalid;
  std::string error;
  require(!imu::preintegrate(samples({0.,.01},Vector3d::Zero(),Vector3d::Zero()),0.,.01,
                             Vector3d::Zero(),Vector3d::Zero(),zero,&invalid,&error) &&
          error.find("rank-deficient")!=std::string::npos,
          "singular covariance fails explicitly without fake jitter");
}

void biasJacobianAndCorrection() {
  const auto raw=samples({0.,.005,.012,.019,.028,.04,.05},Vector3d(2.,-1.,9.),Vector3d(.4,-.6,.8));
  const Vector3d ba(.03,-.02,.01), bg(.003,.002,-.001);
  const auto base=integrate(raw,.001,.049,ba,bg);
  for (int column=0;column<6;++column) {
    const double step=3e-7;
    Vector3d bap=ba,bam=ba,bgp=bg,bgm=bg;
    if(column<3) {bap[column]+=step; bam[column]-=step;}
    else {bgp[column-3]+=step; bgm[column-3]-=step;}
    const auto plus=integrate(raw,.001,.049,bap,bgp);
    const auto minus=integrate(raw,.001,.049,bam,bgm);
    Eigen::Matrix<double,9,1> numerical;
    numerical << (imu::logSO3(base.dR.conjugate()*plus.dR)-
                  imu::logSO3(base.dR.conjugate()*minus.dR))/(2.*step),
                 (plus.alpha-minus.alpha)/(2.*step), (plus.beta-minus.beta)/(2.*step);
    near((base.bias_jacobian.col(column)-numerical).norm(),0.,1e-8,
         "cached bias Jacobian matches independent finite-difference step");
  }
  imu::State i;
  i.timestamp=.001;
  i.q_WB=imu::expSO3(Vector3d(.4,-.2,.1));
  i.ba=ba+Vector3d(1e-4,-2e-4,3e-4);
  i.bg=bg+Vector3d(1e-4,2e-4,-1e-4);
  const auto fresh=integrate(raw,.001,.049,i.ba,i.bg);
  const Vector3d g(0.,0.,-9.8082);
  const auto j=predicted(i,fresh,g);
  near(imu::residual(base,i,j,g).norm(),0.,3e-10,
       "bias-corrected cached factor matches reintegration to second order");
}

void endpointCoverageAndDuplicateEdges() {
  auto raw=samples({0.,.005,.012,.02,.03,.04,.05},Vector3d(1.,2.,9.),Vector3d(.1,.2,.3));
  for (auto& s:raw) {
    s.linear_acceleration += s.timestamp*Vector3d(.3,-.4,.5);
    s.angular_velocity += s.timestamp*Vector3d(.1,.2,-.1);
  }
  const auto p=integrate(raw,.002,.048);
  imu::Measurements clipped;
  std::string error;
  require(imu::clipMeasurements(raw,.002,.048,.015,&clipped,&error), "exact camera endpoint clipping");
  require(clipped.front().timestamp==.002 && clipped.back().timestamp==.048,
          "original double endpoint stamps retained");
  const auto explicit_clip=integrate(clipped,.002,.048);
  near((p.alpha-explicit_clip.alpha).norm()+(p.beta-explicit_clip.beta).norm()+
       (p.covariance-explicit_clip.covariance).norm(),0.,0.,"edge-containing and explicitly clipped equivalence");
  auto duplicated=raw;
  duplicated.insert(duplicated.end(),raw.begin(),raw.end());
  std::reverse(duplicated.begin(),duplicated.end());
  const auto duplicate_p=integrate(duplicated,.002,.048);
  near((p.covariance-duplicate_p.covariance).norm()+(p.alpha-duplicate_p.alpha).norm()+
       (p.beta-duplicate_p.beta).norm()+imu::logSO3(p.dR.conjugate()*duplicate_p.dR).norm(),
       0.,1e-16,"newest-first duplicate edge samples contribute exactly once");
  auto conflict=raw;
  conflict.push_back(raw[2]); conflict.back().angular_velocity.x()+=.1;
  require(!imu::clipMeasurements(conflict,.002,.048,.015,&clipped,&error), "conflicting duplicate sample rejected");
  require(!imu::clipMeasurements(raw,-.001,.048,.015,&clipped,&error), "missing start coverage rejected");
  require(!imu::clipMeasurements(raw,.002,.051,.015,&clipped,&error), "missing end coverage rejected");
  require(!imu::clipMeasurements(samples({0.,.02},Vector3d::Zero(),Vector3d::Zero()),
                                 .004,.006,.015,&clipped,&error),
          "long raw gap straddling tiny clipped interval rejected");
  auto nonfinite=raw; nonfinite[2].linear_acceleration.z()=std::numeric_limits<double>::quiet_NaN();
  require(!imu::clipMeasurements(nonfinite,.002,.048,.015,&clipped,&error), "nonfinite raw sample rejected");
  require(!imu::clipMeasurements(raw,.02,.02,.015,&clipped,&error),"zero duration rejected");
  const double epoch=1.40363658e9, ti=epoch+.0013, tj=epoch+.0271;
  const Vector3d force(1.2,-.7,9.8);
  const auto epoch_p=integrate(samples({epoch,epoch+.005,epoch+.011,epoch+.02,epoch+.03},
                                      force,Vector3d::Zero()),ti,tj);
  require(epoch_p.ti==ti && epoch_p.tj==tj && epoch_p.dt==tj-ti,
          "large epoch preserves original double camera stamps");
  near((epoch_p.beta-force*(tj-ti)).norm(),0.,1e-15,
       "large epoch uses exact clipped SI interval");
}

int main() {
  try {
    constantAccelerationAndFrames();
    constantRotationVaryingDt();
    continuousCovariance();
    biasJacobianAndCorrection();
    endpointCoverageAndDuplicateEdges();
    std::cout << "PASS schur_joint_imu: SI midpoint motion, exact endpoints, full15 continuous covariance, "
                 "bias Jacobians, right body rotation, whitening and coverage/rank refusal\n";
    return EXIT_SUCCESS;
  } catch(const std::exception& error) {
    std::cerr << "FAIL schur_joint_imu: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
