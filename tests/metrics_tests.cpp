#include <cmath>
#include <cstddef>
#include <iostream>
#include <optional>
#include <vector>

#include "sensor_fusion/ekf/metrics.h"
#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/ekf/state.h"

namespace {

using sensor_fusion::ComputeFusionMetrics;
using sensor_fusion::EkfEstimate;
using sensor_fusion::ErrorsInWindow;
using sensor_fusion::FindReacquisition;
using sensor_fusion::FusionMetrics;
using sensor_fusion::FusionRuns;
using sensor_fusion::GnssErrors;
using sensor_fusion::GnssFix;
using sensor_fusion::PlanarPosition;
using sensor_fusion::PositionError;
using sensor_fusion::PositionErrors;
using sensor_fusion::ReferencePositionAt;
using sensor_fusion::ReferenceSample;
using sensor_fusion::Rmse;
using sensor_fusion::TimeWindow;

int failure_count = 0;

void Check(bool ok, const char* text, int line) {
  if (!ok) {
    std::cerr << "line " << line << ": " << text << '\n';
    ++failure_count;
  }
}

#define CHECK(condition) \
  Check(static_cast<bool>(condition), #condition, __LINE__)

bool Near(double actual, double expected, double tolerance = 1e-12) {
  return std::abs(actual - expected) <= tolerance;
}

ReferenceSample Sample(double t, double east, double north) {
  return {.t = t,
          .state = {.east = east, .north = north, .yaw = 0.0, .v = 0.0}};
}

EkfEstimate Estimate(double t, double east, double north,
                     bool updated = false) {
  return {.t = t,
          .state = {.east = east, .north = north, .yaw = 0.0, .v = 0.0},
          .std_east = 1.0,
          .std_north = 1.0,
          .updated = updated};
}

void TestRmse() {
  CHECK(!Rmse({}).has_value());
  const std::vector<double> one = {-2.5};
  CHECK(Rmse(one) == 2.5);
  // sqrt((9 + 16 + 144) / 3), as numpy.sqrt(numpy.mean(numpy.square(x))).
  const std::vector<double> three = {3.0, 4.0, 12.0};
  CHECK(Rmse(three) == 0x1.e05afd10a530ep+2);
}

void TestPositionErrors() {
  const std::vector<ReferenceSample> reference = {Sample(0.0, 0.0, 0.0),
                                                  Sample(1.0, 10.0, 0.0)};
  const std::vector<EkfEstimate> estimates = {Estimate(0.0, 3.0, 4.0),
                                              Estimate(1.0, 10.0, -2.0)};
  CHECK(PositionError(estimates[0], reference[0]) == 5.0);
  const std::optional<std::vector<double>> errors =
      PositionErrors(estimates, reference);
  CHECK(errors.has_value() && errors->size() == 2 && (*errors)[0] == 5.0 &&
        (*errors)[1] == 2.0);

  // Different length or different times do not compare.
  CHECK(!PositionErrors(std::vector<EkfEstimate>{estimates[0]}, reference)
             .has_value());
  std::vector<EkfEstimate> shifted = estimates;
  shifted[1].t = 1.5;
  CHECK(!PositionErrors(shifted, reference).has_value());
  CHECK(PositionErrors({}, {}).has_value());
}

// Expected values printed by numpy.interp() with float.hex().
void TestReferencePositionAt() {
  const std::vector<ReferenceSample> reference = {
      Sample(0.0, 0.0, 5.0), Sample(0.1, 1.1, 4.4),
      Sample(0.30000000000000004, 2.7, 4.1), Sample(0.7, -3.3, 9.9)};
  struct Case {
    double t;
    double east;
    double north;
  };
  const std::vector<Case> cases = {
      {.t = -1.0, .east = 0.0, .north = 0x1.4p+2},
      {.t = 0.0, .east = 0.0, .north = 0x1.4p+2},
      {.t = 0.05, .east = 0x1.199999999999ap-1, .north = 0x1.2cccccccccccdp+2},
      {.t = 0.1, .east = 0x1.199999999999ap+0, .north = 0x1.199999999999ap+2},
      {.t = 0.2, .east = 0x1.e666666666667p+0, .north = 0x1.1p+2},
      {.t = 0.30000000000000004,
       .east = 0x1.599999999999ap+1,
       .north = 0x1.0666666666666p+2},
      {.t = 0.31, .east = 0x1.4666666666668p+1, .north = 0x1.0fae147ae147ap+2},
      {.t = 0.7, .east = -0x1.a666666666666p+1, .north = 0x1.3cccccccccccdp+3},
      {.t = 2.0, .east = -0x1.a666666666666p+1, .north = 0x1.3cccccccccccdp+3},
  };
  for (const Case& c : cases) {
    const PlanarPosition position = ReferencePositionAt(reference, c.t);
    CHECK(position.east == c.east);
    CHECK(position.north == c.north);
  }

  // A single sample is the position at every time.
  const std::vector<ReferenceSample> single = {Sample(1.0, 2.0, 3.0)};
  const PlanarPosition position = ReferencePositionAt(single, 0.5);
  CHECK(position.east == 2.0 && position.north == 3.0);
}

void TestGnssErrors() {
  const std::vector<ReferenceSample> reference = {Sample(0.0, 0.0, 0.0),
                                                  Sample(2.0, 20.0, 0.0)};
  const std::vector<GnssFix> fixes = {{.t = 0.5, .east = 8.0, .north = 3.0},
                                      {.t = 1.0, .east = 10.0, .north = -1.0}};
  const std::vector<double> errors = GnssErrors(fixes, reference);
  // The truth at 0.5 s is (5, 0), at 1 s (10, 0).
  CHECK(errors.size() == 2 && Near(errors[0], std::hypot(3.0, 3.0)) &&
        errors[1] == 1.0);
  CHECK(GnssErrors(fixes, {}).empty());
  CHECK(GnssErrors({}, reference).empty());
}

void TestWindow() {
  const TimeWindow window = {.start = 1.0, .end = 2.0};
  CHECK(window.Contains(1.0));
  CHECK(window.Contains(1.999));
  CHECK(!window.Contains(2.0));
  CHECK(!window.Contains(0.999));
  CHECK(!window.Contains(std::nan("")));
  const TimeWindow empty = {.start = 1.0, .end = 1.0};
  CHECK(!empty.Contains(1.0));

  const std::vector<EkfEstimate> estimates = {
      Estimate(0.5, 0, 0), Estimate(1.0, 0, 0), Estimate(1.5, 0, 0),
      Estimate(2.0, 0, 0)};
  const std::vector<double> errors = {1.0, 2.0, 3.0, 4.0};
  const std::vector<double> inside = ErrorsInWindow(errors, estimates, window);
  CHECK(inside == std::vector<double>({2.0, 3.0}));
}

void TestReacquisition() {
  const TimeWindow window = {.start = 1.0, .end = 2.0};
  const std::vector<EkfEstimate> run = {
      Estimate(0.0, 0, 0), Estimate(1.0, 0, 0, true),  // Before the end.
      Estimate(2.0, 0, 0), Estimate(3.0, 0, 0, true),
      Estimate(4.0, 0, 0, true)};
  const std::vector<double> filter = {0.1, 0.2, 2.0, 0.5, 0.4};
  const std::vector<double> imu_only = {0.1, 0.3, 9.0, 12.0, 15.0};
  const std::optional<sensor_fusion::Reacquisition> found =
      FindReacquisition(run, filter, imu_only, window);
  CHECK(found.has_value());
  if (found.has_value()) {
    CHECK(found->index == 2 && found->t == 2.0);
    CHECK(found->filter_error == 2.0 && found->imu_only_error == 9.0);
    CHECK(found->ratio == 4.5);
  }

  // The denominator is at least 1e-9 m.
  const std::vector<double> exact = {0.0, 0.0, 0.0, 0.0, 0.0};
  const auto perfect = FindReacquisition(run, exact, imu_only, window);
  CHECK(perfect.has_value() && Near(perfect->ratio, 9.0e9, 1.0));

  // No fix after the window.
  CHECK(!FindReacquisition(run, filter, imu_only,
                           TimeWindow{.start = 4.5, .end = 5.0})
             .has_value());
  // The first fix after the window is on step 0: no step before it.
  const std::vector<EkfEstimate> first_updated = {Estimate(0.0, 0, 0, true)};
  CHECK(!FindReacquisition(first_updated, filter, imu_only,
                           TimeWindow{.start = -2.0, .end = -1.0})
             .has_value());
  // Error series shorter than the run.
  CHECK(!FindReacquisition(run, std::vector<double>{0.1}, imu_only, window)
             .has_value());
}

void TestComputeFusionMetrics() {
  // The car stands at the origin for 4 s.
  std::vector<ReferenceSample> reference;
  std::vector<EkfEstimate> full;
  std::vector<EkfEstimate> outage;
  std::vector<EkfEstimate> imu_only;
  for (int i = 0; i <= 4; ++i) {
    const auto t = static_cast<double>(i);
    reference.push_back(Sample(t, 0.0, 0.0));
    full.push_back(Estimate(t, 0.5, 0.0, i == 3));
    // Drifts north by 1 m a second until the fix at 3 s.
    outage.push_back(Estimate(t, 0.0, i < 3 ? t : 0.5, i == 3));
    imu_only.push_back(Estimate(t, 0.0, 2.0 * t));
  }
  const std::vector<GnssFix> gnss = {{.t = 0.5, .east = 2.0, .north = 0.0},
                                     {.t = 3.0, .east = 0.0, .north = -2.0}};
  const FusionRuns runs = {.reference = reference,
                           .gnss = gnss,
                           .full = full,
                           .outage = outage,
                           .imu_only = imu_only,
                           .outage_window = {.start = 1.0, .end = 3.0}};
  const std::optional<FusionMetrics> metrics = ComputeFusionMetrics(runs);
  CHECK(metrics.has_value());
  if (!metrics.has_value()) {
    return;
  }
  CHECK(metrics->raw_gnss_rmse == 2.0);
  CHECK(metrics->filter_rmse == 0.5);
  CHECK(metrics->filter_rmse_reduction == 0.75);
  // Outage errors 0, 1, 2, 0.5, 0.5.
  CHECK(Near(metrics->outage_rmse, std::sqrt(5.5 / 5.0)));
  // Inside [1, 3): the steps at 1 s and 2 s.
  CHECK(Near(metrics->outage_window_rmse.value_or(-1.0), std::sqrt(2.5)));
  CHECK(Near(metrics->imu_only_window_rmse.value_or(-1.0), std::sqrt(10.0)));
  CHECK(metrics->reacquisition.has_value());
  if (metrics->reacquisition.has_value()) {
    CHECK(metrics->reacquisition->index == 2);
    CHECK(metrics->reacquisition->filter_error == 2.0);
    CHECK(metrics->reacquisition->imu_only_error == 4.0);
    CHECK(metrics->reacquisition->ratio == 2.0);
  }

  // Without fixes and with a window that holds no step the values are
  // missing, not zero.
  FusionRuns empty_window = runs;
  empty_window.gnss = {};
  empty_window.outage_window = {.start = 1.25, .end = 1.5};
  const std::optional<FusionMetrics> sparse =
      ComputeFusionMetrics(empty_window);
  CHECK(sparse.has_value());
  if (sparse.has_value()) {
    CHECK(!sparse->raw_gnss_rmse.has_value());
    CHECK(!sparse->filter_rmse_reduction.has_value());
    CHECK(!sparse->outage_window_rmse.has_value());
    CHECK(!sparse->imu_only_window_rmse.has_value());
    CHECK(sparse->reacquisition.has_value());
  }

  // A run that does not match the reference.
  FusionRuns short_run = runs;
  short_run.imu_only = std::span<const EkfEstimate>(imu_only).first(4);
  CHECK(!ComputeFusionMetrics(short_run).has_value());
  FusionRuns no_reference = runs;
  no_reference.reference = {};
  CHECK(!ComputeFusionMetrics(no_reference).has_value());
}

}  // namespace

int main() {
  TestRmse();
  TestPositionErrors();
  TestReferencePositionAt();
  TestGnssErrors();
  TestWindow();
  TestReacquisition();
  TestComputeFusionMetrics();

  if (failure_count != 0) {
    std::cerr << failure_count << " checks failed\n";
    return 1;
  }

  std::cout << "All tests passed\n";
  return 0;
}
