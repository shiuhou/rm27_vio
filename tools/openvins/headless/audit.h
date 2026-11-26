#pragma once
#include <cstdint>

// Observation-only counters, incremented strictly after the upstream EKF update
// returns. No updater selection, residual, Jacobian, gain or state is modified.
namespace rm27_openvins {
struct UpdateAudit {
  uint64_t msckf_updates = 0, msckf_features = 0, msckf_rows = 0;
  uint64_t slam_updates = 0, slam_features = 0, slam_rows = 0;
};
extern UpdateAudit audit;
}
