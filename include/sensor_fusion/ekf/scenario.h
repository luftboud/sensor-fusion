#ifndef SENSOR_FUSION_EKF_SCENARIO_H_
#define SENSOR_FUSION_EKF_SCENARIO_H_

#include <cstddef>
#include <vector>

#include "sensor_fusion/ekf/state.h"

namespace sensor_fusion {

// One planar IMU step, the input the filter integrates.
//
// Frame: the body frame of the car, the same axes as ImuSample. x points
// forward, y to the left and z up. ax is specific force along x, so a pitch
// leaves a piece of gravity in the reading. wz is the angular rate around z.
// A positive wz turns left.
//
// The 2D model keeps this ax and this wz. The levelled pair af and wu from
// an OXTS record is a different reading, and this sample does not carry it.
// See docs/kitti-oxts-survey.md.
//
// t is seconds on the drive clock. Zero is the first reference sample.
//
// imu[i] is the reading at the moment t[i]. The prediction holds it over
// [t[i], t[i+1]) and integrates with dt = t[i+1] - t[i], so imu[0] moves the
// state from t[0] to t[1]. The last sample has no next step and moves
// nothing.
struct PlanarImu {
  double t = 0.0;   // Seconds since the first reference sample.
  double ax = 0.0;  // Forward specific force (m/s^2), body x.
  double wz = 0.0;  // Yaw rate about body z (rad/s). Positive turns left.
};

// One GNSS fix in the drive's east-north frame.
//
// east and north are metres from the first reference sample, the same origin
// as State2d. The first fix sits at (0, 0) only when it is that sample. A
// fix that arrives later has non-zero coordinates.
//
// t is on the drive clock and may fall between IMU steps. A fix does not
// have to share its t with a PlanarImu. The filter chooses which IMU step
// consumes the fix and records that choice in EkfEstimate::updated.
struct GnssFix {
  double t = 0.0;      // Seconds since the first reference sample.
  double east = 0.0;   // Metres east of the drive origin.
  double north = 0.0;  // Metres north of the drive origin.
};

// Ground truth at one IMU step, kept for comparison with EkfEstimate.
// A filter step reads PlanarImu and, when it accepts a fix, GnssFix.
struct ReferenceSample {
  double t = 0.0;  // Seconds since the first reference sample.
  State2d state = {};
};

// One drive: the IMU steps, the GNSS fixes that exist, and the reference.
//
// The three series share one clock, in seconds. Zero is the first reference
// sample. An empty Scenario is a valid aggregate. When reference is
// non-empty, a filled drive has these properties:
//
// - reference and imu have the same length, and reference[i].t == imu[i].t.
// - Time in each series is strictly increasing.
// - reference.front() has t == 0, state.east == 0 and state.north == 0.
//   yaw and v there are the drive's own values.
// - gnss times use the same clock and need not equal any imu time. A stretch
//   with no gnss samples is a GNSS outage.
//
// EkfEstimate is not a member. The filter returns one estimate per IMU step
// beside this log. reference stays on the log for comparison.
struct Scenario {
  std::vector<PlanarImu> imu;              // Propagation steps.
  std::vector<GnssFix> gnss;               // Fixes that exist. A gap is an outage.
  std::vector<ReferenceSample> reference;  // Comparison series, one per IMU step.
};

// Returns true for an empty scenario, and for a filled one that has every
// property listed above Scenario. A NaN time fails the order checks. The
// values themselves are not checked.
inline bool IsWellFormed(const Scenario& scenario) {
  const std::vector<PlanarImu>& imu = scenario.imu;
  const std::vector<GnssFix>& gnss = scenario.gnss;
  const std::vector<ReferenceSample>& reference = scenario.reference;
  if (reference.empty()) {
    return imu.empty() && gnss.empty();
  }
  if (imu.size() != reference.size()) {
    return false;
  }
  const ReferenceSample& origin = reference.front();
  if (origin.t != 0.0 || origin.state.east != 0.0 ||
      origin.state.north != 0.0) {
    return false;
  }
  for (std::size_t i = 0; i < reference.size(); ++i) {
    if (imu[i].t != reference[i].t) {
      return false;
    }
    if (i > 0 && !(reference[i - 1].t < reference[i].t)) {
      return false;
    }
  }
  for (std::size_t i = 1; i < gnss.size(); ++i) {
    if (!(gnss[i - 1].t < gnss[i].t)) {
      return false;
    }
  }
  return true;
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_EKF_SCENARIO_H_
