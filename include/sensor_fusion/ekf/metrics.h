#ifndef SENSOR_FUSION_EKF_METRICS_H_
#define SENSOR_FUSION_EKF_METRICS_H_

#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/ekf/state.h"

namespace sensor_fusion {

// Accuracy of a filter run against the reference trajectory. The building
// blocks are separate functions, so a new metric is one more function and
// one more field in FusionMetrics. ComputeFusionMetrics() computes the set
// that foreign_impl/ref_ekf.py prints, the same way, so the two can be
// compared. See docs/metrics.md.

// A planar position in metres, east and north of the drive origin.
struct PlanarPosition {
  double east = 0.0;
  double north = 0.0;
};

// The half-open time interval [start, end) in seconds.
struct TimeWindow {
  double start = 0.0;
  double end = 0.0;

  [[nodiscard]] bool Contains(double t) const { return t >= start && t < end; }
};

// Root mean square of the values. std::nullopt when there are none.
inline std::optional<double> Rmse(std::span<const double> values) {
  if (values.empty()) {
    return std::nullopt;
  }
  double sum = 0.0;
  for (const double value : values) {
    sum += value * value;
  }
  return std::sqrt(sum / static_cast<double>(values.size()));
}

// Distance between an estimate and the reference sample (m).
inline double PositionError(const EkfEstimate& estimate,
                            const ReferenceSample& reference) {
  return std::hypot(estimate.state.east - reference.state.east,
                    estimate.state.north - reference.state.north);
}

// PositionError() of each estimate against the reference sample at the same
// index. std::nullopt when the two series differ in length or in time.
inline std::optional<std::vector<double>> PositionErrors(
    std::span<const EkfEstimate> estimates,
    std::span<const ReferenceSample> reference) {
  if (estimates.size() != reference.size()) {
    return std::nullopt;
  }
  std::vector<double> errors;
  errors.reserve(estimates.size());
  for (std::size_t i = 0; i < estimates.size(); ++i) {
    if (estimates[i].t != reference[i].t) {
      return std::nullopt;
    }
    errors.push_back(PositionError(estimates[i], reference[i]));
  }
  return errors;
}

// The reference position at time t, linearly interpolated between samples
// as numpy.interp() does: before the first sample it is the first position,
// after the last it is the last, and on a sample it is that sample. The
// reference must not be empty and its times must increase.
inline PlanarPosition ReferencePositionAt(
    std::span<const ReferenceSample> reference, double t) {
  if (!(t > reference.front().t)) {
    return {.east = reference.front().state.east,
            .north = reference.front().state.north};
  }
  if (!(t < reference.back().t)) {
    return {.east = reference.back().state.east,
            .north = reference.back().state.north};
  }
  // The last sample at or before t.
  std::size_t low = 0;
  std::size_t high = reference.size() - 1;
  while (high - low > 1) {
    const std::size_t middle = low + ((high - low) / 2);
    if (reference[middle].t <= t) {
      low = middle;
    } else {
      high = middle;
    }
  }
  const ReferenceSample& before = reference[low];
  if (before.t == t) {
    return {.east = before.state.east, .north = before.state.north};
  }
  const ReferenceSample& after = reference[low + 1];
  // numpy computes slope * (t - t0) + y0. Its arm64 build fuses the product
  // and the sum into one rounding; std::fma() does the same on every
  // platform, so the result does not depend on the compiler. An x86-64
  // numpy that rounds twice can differ in the last bit.
  const double span = after.t - before.t;
  const double east_slope = (after.state.east - before.state.east) / span;
  const double north_slope = (after.state.north - before.state.north) / span;
  return {.east = std::fma(east_slope, t - before.t, before.state.east),
          .north = std::fma(north_slope, t - before.t, before.state.north)};
}

// Distance of each fix from the reference position at its time (m). Empty
// when the reference is empty.
inline std::vector<double> GnssErrors(
    std::span<const GnssFix> gnss, std::span<const ReferenceSample> reference) {
  std::vector<double> errors;
  if (reference.empty()) {
    return errors;
  }
  errors.reserve(gnss.size());
  for (const GnssFix& fix : gnss) {
    const PlanarPosition truth = ReferencePositionAt(reference, fix.t);
    errors.push_back(
        std::hypot(fix.east - truth.east, fix.north - truth.north));
  }
  return errors;
}

// The errors whose estimate time lies in the window. errors[i] belongs to
// estimates[i]; the extra entries of the longer series are ignored.
inline std::vector<double> ErrorsInWindow(
    std::span<const double> errors, std::span<const EkfEstimate> estimates,
    const TimeWindow& window) {
  std::vector<double> inside;
  const std::size_t count =
      errors.size() < estimates.size() ? errors.size() : estimates.size();
  for (std::size_t i = 0; i < count; ++i) {
    if (window.Contains(estimates[i].t)) {
      inside.push_back(errors[i]);
    }
  }
  return inside;
}

// The moment GNSS returns after an outage, judged on the step before the
// first fix: that step has drifted the longest without a correction.
struct Reacquisition {
  std::size_t index = 0;  // The step before the first update after the end.
  double t = 0.0;
  double filter_error = 0.0;    // Filter with the outage (m).
  double imu_only_error = 0.0;  // Filter without any fix (m).
  // imu_only_error / filter_error, with the denominator at least 1e-9 m as
  // in ref_ekf.py. Above 1 the filter beat dead reckoning.
  double ratio = 0.0;
};

// Finds the first estimate at or after window.end that consumed a fix and
// compares the step before it. std::nullopt when no fix follows the window
// or the first such fix is on step 0.
inline std::optional<Reacquisition> FindReacquisition(
    std::span<const EkfEstimate> outage_run,
    std::span<const double> outage_errors,
    std::span<const double> imu_only_errors, const TimeWindow& window) {
  for (std::size_t i = 0; i < outage_run.size(); ++i) {
    if (outage_run[i].t < window.end || !outage_run[i].updated) {
      continue;
    }
    if (i == 0) {
      return std::nullopt;
    }
    const std::size_t before = i - 1;
    if (before >= outage_errors.size() || before >= imu_only_errors.size()) {
      return std::nullopt;
    }
    constexpr double kMinError = 1e-9;
    const double filter_error = outage_errors[before];
    return Reacquisition{
        .index = before,
        .t = outage_run[before].t,
        .filter_error = filter_error,
        .imu_only_error = imu_only_errors[before],
        .ratio = imu_only_errors[before] /
                 (filter_error > kMinError ? filter_error : kMinError),
    };
  }
  return std::nullopt;
}

// The runs to compare. Every run has one estimate per reference sample, at
// the same times.
struct FusionRuns {
  std::span<const ReferenceSample> reference;
  std::span<const GnssFix> gnss;            // Every fix, for the raw error.
  std::span<const EkfEstimate> full;        // Filter with every fix.
  std::span<const EkfEstimate> outage;      // Filter with the outage cut.
  std::span<const EkfEstimate> imu_only;    // Filter without fixes.
  TimeWindow outage_window;
};

// A value is std::nullopt when it has nothing to average: no fixes, no step
// inside the window, or no fix after it.
struct FusionMetrics {
  std::optional<double> raw_gnss_rmse;
  double filter_rmse = 0.0;
  // 1 - filter_rmse / raw_gnss_rmse. 0.3 means 30% below raw GNSS.
  std::optional<double> filter_rmse_reduction;
  double outage_rmse = 0.0;  // Filter with the outage, whole drive.
  std::optional<double> outage_window_rmse;
  std::optional<double> imu_only_window_rmse;
  std::optional<Reacquisition> reacquisition;
};

// std::nullopt when the reference is empty or a run does not match it.
inline std::optional<FusionMetrics> ComputeFusionMetrics(
    const FusionRuns& runs) {
  if (runs.reference.empty()) {
    return std::nullopt;
  }
  const std::optional<std::vector<double>> full_errors =
      PositionErrors(runs.full, runs.reference);
  const std::optional<std::vector<double>> outage_errors =
      PositionErrors(runs.outage, runs.reference);
  const std::optional<std::vector<double>> imu_only_errors =
      PositionErrors(runs.imu_only, runs.reference);
  if (!full_errors.has_value() || !outage_errors.has_value() ||
      !imu_only_errors.has_value()) {
    return std::nullopt;
  }

  // Each run has one error per reference sample and the reference is not
  // empty, so the two whole-drive RMSE values always exist.
  FusionMetrics metrics;
  metrics.raw_gnss_rmse = Rmse(GnssErrors(runs.gnss, runs.reference));
  metrics.filter_rmse = Rmse(*full_errors).value_or(0.0);
  if (metrics.raw_gnss_rmse.has_value() && *metrics.raw_gnss_rmse > 0.0) {
    metrics.filter_rmse_reduction =
        1.0 - (metrics.filter_rmse / *metrics.raw_gnss_rmse);
  }
  metrics.outage_rmse = Rmse(*outage_errors).value_or(0.0);
  metrics.outage_window_rmse = Rmse(
      ErrorsInWindow(*outage_errors, runs.outage, runs.outage_window));
  metrics.imu_only_window_rmse = Rmse(
      ErrorsInWindow(*imu_only_errors, runs.imu_only, runs.outage_window));
  metrics.reacquisition = FindReacquisition(
      runs.outage, *outage_errors, *imu_only_errors, runs.outage_window);
  return metrics;
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_EKF_METRICS_H_
