#ifndef SENSOR_FUSION_EKF_TARGETS_H_
#define SENSOR_FUSION_EKF_TARGETS_H_

// The targets of the success criteria in docs/project.md. Every report reads
// them from here, so a target is changed in one place.

namespace sensor_fusion {

// Criterion 2, on the synthetic drive: the filter RMSE over the whole route
// is at least this fraction below the RMSE of the raw GNSS fixes. 0.3 is 30%.
inline constexpr double kCriterion2MinReduction = 0.3;

// Criterion 3, on the synthetic drive: after the outage, the error without
// the correction is at least this many times the error with it.
inline constexpr double kCriterion3MinRatio = 3.0;

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_EKF_TARGETS_H_
