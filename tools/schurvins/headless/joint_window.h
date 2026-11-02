#pragma once
// One raw-factor Schur objective from bootstrap through fixed-lag tracking.
// Tangent: right SO(3), world position/velocity, body accelerometer/gyro biases.
#include "joint_imu.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace schur_vins {
class JointWindow {
 public:
  using Measurements = svo::headless::joint_imu::Measurements;
  struct Frame {
    int64_t ns = 0;
    double timestamp = 0;  // Original camera seconds; ns is measurement identity.
    Eigen::Quaterniond q_WB = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p_WB = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_W = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  };
  struct Observation {
    int64_t frame_ns = 0, point_id = 0;
    int camera_id = 0;
    uint64_t feature_id = 0;
    Eigen::Vector3d bearing = Eigen::Vector3d::Zero();
    // p_C = q_CB * p_B + t_CB. No extrinsic uncertainty is estimated.
    Eigen::Quaterniond q_CB = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_CB = Eigen::Vector3d::Zero();
    Eigen::Vector2d focal = Eigen::Vector2d::Zero();
    double sigma_pixels = 0;
    int level = 0;  // Identity metadata; existing Schur weighting uses level_scale=1.
  };
  struct Options {
    svo::headless::joint_imu::Calibration imu;
    double huber_pixels = 1.5;  // Existing SchurVINS Huber radius.
    int max_iterations = 15;
  };
  struct PointBlock {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    Eigen::MatrixXd W;
    int rank = 0;
    double numeric_scale = 1;  // Ephemeral LM scale; inverse is H^+ / scale.
  };
  struct Linearization {
    Eigen::MatrixXd H;  // State/gravity block before point elimination.
    Eigen::VectorXd b;  // -J' r throughout this API.
    std::map<int64_t, PointBlock> points;
    std::map<int64_t, int> frame_offsets;  // 15 slots; first gauge has 6 omitted.
    int gravity_offset = 0;
    double cost = 0;
    bool finite = false;
    std::string reason;
  };
  struct SolveReport {
    bool success = false, finite = false;
    int iterations = 0, state_dimension = 0, state_rank = 0;
    double initial_cost = 0, final_cost = 0, max_step = 0;
    size_t active_pixels = 0, absorbed_pixels = 0, dropped_pixels = 0;
    // Active registry includes pending rows, which carry no likelihood until
    // >=3 distinct frame views and the inherited world-point norm <=1000 m.
    size_t pending_pixels = 0, pending_points = 0, spatial_ineligible_points = 0;
    size_t active_imu_edges = 0, absorbed_imu_edges = 0, active_points = 0;
    std::string reason;
  };
  explicit JointWindow(const Options& options);
  bool addFrame(const Frame&, const Measurements& from_previous = Measurements(),
                std::string* error = nullptr);
  // Existing active IDs are no-ops; retired IDs create a new free segment.
  bool addLandmark(int64_t physical_id, const Eigen::Vector3d& seed,
                   std::string* error = nullptr);
  bool addObservation(const Observation&, std::string* error = nullptr);
  // Conservative rejection discards this row's information and keeps its global
  // measurement identity retired. It is never classified as absorbed evidence.
  bool rejectObservation(int64_t ns, int camera_id, uint64_t feature_id,
                         std::string* error = nullptr);
  // Current rows with at least three distinct active frame views (MIN_OBS=3),
  // admitted world-point norm <=1000 m, raw rank-three point information and
  // nonzero reduced current-camera visual information.
  size_t currentMultiViewObservations(int64_t ns) const;
  bool setInitialPrior(const Eigen::Vector3d& v, const Eigen::Vector3d& ba,
                       const Eigen::Vector3d& bg, double sigma_v,
                       double sigma_ba, double sigma_bg,
                       std::string* error = nullptr);
  bool setFrameSeed(const Frame&, std::string* error = nullptr);
  bool setLandmarkSeed(int64_t, const Eigen::Vector3d&, std::string* error = nullptr);
  bool setGravity(const Eigen::Vector3d&, std::string* error = nullptr);
  const Eigen::Vector3d& gravity() const { return gravity_; }
  const std::vector<Frame>& frames() const { return frames_; }
  const std::map<int64_t, Eigen::Vector3d>& landmarks() const { return points_; }
  const std::vector<Observation>& observations() const { return observations_; }
  std::set<int64_t> admittedPoints() const { return eligiblePoints(); }
  SolveReport optimize();
  bool marginalizeTo(size_t keep_frames, std::string* error = nullptr);
  Linearization linearize(bool include_priors = true) const;
  // Moore-Penrose covariance, with rank explicitly reported. No LM information.
  Eigen::MatrixXd covariance(int* rank = nullptr) const;
  SolveReport status() const;
  // Bounded first-failure evidence: complete raw graph, numerical prior and
  // measurement ownership. Writing refuses to overwrite existing evidence.
  bool writeSnapshot(const std::string& path, std::string* error = nullptr) const;
  static std::unique_ptr<JointWindow> readSnapshot(
      const std::string& path, std::string* error = nullptr);
  // Same undamped point blocks used by camera Schur and point recovery.
  static bool schur(const Linearization&, Eigen::MatrixXd*, Eigen::VectorXd*,
                    std::string* error = nullptr);
  static std::map<int64_t, Eigen::Vector3d> backSubstitute(
      const Linearization&, const Eigen::VectorXd& state_step);
 private:
  using PixelKey = std::tuple<int64_t, int, uint64_t>;
  using EdgeKey = std::pair<int64_t, int64_t>;
  struct Edge { EdgeKey key; Measurements raw; };
  struct InitialPrior {
    bool active = false;
    Eigen::Vector3d v, ba, bg;
    double sigma_v = 0, sigma_ba = 0, sigma_bg = 0;
  };
  struct Prior {
    Eigen::MatrixXd A;
    Eigen::VectorXd rhs;
    std::vector<Frame> anchors;
    Eigen::Vector3d gravity;
  };
  std::set<int64_t> eligiblePoints() const;
  Linearization assemble(const std::set<int64_t>* eliminated_frames,
                         const std::set<int64_t>* absorbed_points, bool include_priors) const;
  int column(const Linearization&, int64_t ns, int component) const;
  void applyStep(const Linearization&, const Eigen::VectorXd&,
                 const std::map<int64_t, Eigen::Vector3d>&);
  Options options_;
  std::vector<Frame> frames_;
  std::map<int64_t, Eigen::Vector3d> points_;
  std::vector<Observation> observations_;
  std::vector<Edge> edges_;
  std::set<PixelKey> active_pixels_, absorbed_pixels_, dropped_pixels_;
  std::set<EdgeKey> active_edges_, absorbed_edges_;
  std::map<int64_t, int64_t> retired_after_;
  int64_t gauge_ns_ = 0;
  Frame gauge_;
  Eigen::Vector3d gravity_ = Eigen::Vector3d(0, 0, -9.81007);
  InitialPrior initial_;
  Prior prior_;
  SolveReport report_;
};
}  // namespace schur_vins
