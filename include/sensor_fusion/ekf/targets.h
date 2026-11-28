#ifndef SENSOR_FUSION_EKF_TARGETS_H_
#define SENSOR_FUSION_EKF_TARGETS_H_

// The targets of the success criteria in docs/project.md. Every report reads
// them from here, so a target is changed in one place.
//
// tools/report.py reads this file with a regular expression, so keep every
// target on one line in the form
//   inline constexpr double kName = <number literal>;

namespace sensor_fusion {

// Criterion 1, on the KITTI drive: the largest absolute difference between
// the C++ filter and filterpy, over east, north, yaw, speed and the two
// position standard deviations of both runs (with every fix and with the
// outage). Metres, radians and metres per second; the same bound for each.
inline constexpr double kCriterion1MaxDifference = 1e-9;

// Criterion 2, on the synthetic drive: the filter RMSE over the whole route
// is at least this fraction below the RMSE of the raw GNSS fixes. 0.3 is 30%.
inline constexpr double kCriterion2MinReduction = 0.3;

// Criterion 3, on the synthetic drive: after the outage, the error without
// the correction is at least this many times the error with it.
inline constexpr double kCriterion3MinRatio = 3.0;

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_EKF_TARGETS_H_
