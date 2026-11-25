#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/ekf/state.h"
#include "sensor_fusion/sim/synthetic_drive.h"

namespace {

using sensor_fusion::DemoDrive;
using sensor_fusion::DriveSegment;
using sensor_fusion::GenerateScenario;
using sensor_fusion::GnssFix;
using sensor_fusion::IsWellFormed;
using sensor_fusion::kMaxDriveSteps;
using sensor_fusion::PlanarImu;
using sensor_fusion::ReferenceSample;
using sensor_fusion::Scenario;
using sensor_fusion::State2d;
using sensor_fusion::SyntheticConfig;

int failure_count = 0;

void Check(bool ok, const char* text, int line) {
  if (!ok) {
    std::cerr << "line " << line << ": " << text << '\n';
    ++failure_count;
  }
}

#define CHECK(condition) \
  Check(static_cast<bool>(condition), #condition, __LINE__)

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

bool Near(double actual, double expected, double tolerance = 1e-9) {
  return std::abs(actual - expected) <= tolerance;
}

// No noise, no bias and no outage: every sensor reads the truth.
SyntheticConfig Exact() {
  SyntheticConfig config;
  config.accel_noise = 0.0;
  config.accel_bias = 0.0;
  config.gyro_noise = 0.0;
  config.gyro_bias = 0.0;
  config.gnss_noise = 0.0;
  config.outage_duration = 0.0;
  return config;
}

bool SameImu(const std::vector<PlanarImu>& a, const std::vector<PlanarImu>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].t != b[i].t || a[i].ax != b[i].ax || a[i].wz != b[i].wz) {
      return false;
    }
  }
  return true;
}

bool SameReference(const std::vector<ReferenceSample>& a,
                   const std::vector<ReferenceSample>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    const State2d& x = a[i].state;
    const State2d& y = b[i].state;
    if (a[i].t != b[i].t || x.east != y.east || x.north != y.north ||
        x.yaw != y.yaw || x.v != y.v) {
      return false;
    }
  }
  return true;
}

bool SameFix(const GnssFix& a, const GnssFix& b) {
  return a.t == b.t && a.east == b.east && a.north == b.north;
}

bool SameGnss(const std::vector<GnssFix>& a, const std::vector<GnssFix>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!SameFix(a[i], b[i])) {
      return false;
    }
  }
  return true;
}

// Mean and standard deviation of a sample.
struct Stats {
  double mean = 0.0;
  double deviation = 0.0;
};

Stats Measure(const std::vector<double>& values) {
  if (values.size() < 2) {
    return {};
  }
  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }
  const double mean = sum / static_cast<double>(values.size());
  double squares = 0.0;
  for (const double value : values) {
    squares += (value - mean) * (value - mean);
  }
  return {
      .mean = mean,
      .deviation =
          std::sqrt(squares / static_cast<double>(values.size() - 1)),
  };
}

// Changes one setting of Exact() and reports whether the generator refuses.
template <typename Change>
bool RejectsConfig(Change change) {
  SyntheticConfig config = Exact();
  change(config);
  const std::vector<DriveSegment> drive = {{.duration = 1.0, .accel = 1.0}};
  return !GenerateScenario(drive, config).has_value();
}

bool RejectsSegment(const DriveSegment& segment) {
  return !GenerateScenario({segment}, Exact()).has_value();
}

void TestRejectsBadConfig() {
  CHECK(!RejectsConfig([](SyntheticConfig&) {}));

  CHECK(RejectsConfig([](SyntheticConfig& c) { c.imu_dt = 0.0; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.imu_dt = -0.01; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.imu_dt = kNan; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.imu_dt = kInf; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.start_yaw = kNan; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.start_speed = kInf; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.gnss_period = 0.0; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.gnss_period = -1.0; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.gnss_offset = -0.5; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.outage_start = kNan; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.outage_duration = -1.0; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.accel_noise = -0.1; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.accel_bias = kNan; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.gyro_noise = kInf; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.gyro_bias = kInf; }));
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.gnss_noise = -2.0; }));

  // A negative bias and an outage before the drive are allowed.
  CHECK(!RejectsConfig([](SyntheticConfig& c) { c.accel_bias = -0.1; }));
  CHECK(!RejectsConfig([](SyntheticConfig& c) { c.outage_start = -5.0; }));

  // Far too many fixes for a one-second drive.
  CHECK(RejectsConfig([](SyntheticConfig& c) { c.gnss_period = 1e-9; }));
}

// 1e-17 s is below the resolution of a double near 1, so twelve fixes would
// share a few times. Such a drive is not well formed.
void TestRejectsRepeatedFixTimes() {
  SyntheticConfig config = Exact();
  config.gnss_offset = std::nextafter(1.0, 0.0);
  config.gnss_period = 1e-17;
  CHECK(!GenerateScenario({{.duration = 1.0}}, config).has_value());

  // The same offset with a usable period gives one fix.
  config.gnss_period = 1.0;
  const std::optional<Scenario> scenario =
      GenerateScenario({{.duration = 1.0}}, config);
  CHECK(scenario.has_value() && scenario->gnss.size() == 1);
}

void TestRejectsBadSegment() {
  CHECK(!RejectsSegment({.duration = 0.0}));
  CHECK(RejectsSegment({.duration = -1.0}));
  CHECK(RejectsSegment({.duration = kNan}));
  CHECK(RejectsSegment({.duration = kInf}));
  CHECK(RejectsSegment({.duration = 1.0, .accel = kNan}));
  CHECK(RejectsSegment({.duration = 1.0, .yaw_rate = kInf}));

  // Not a whole number of 0.01 s steps.
  CHECK(RejectsSegment({.duration = 0.015}));
  CHECK(RejectsSegment({.duration = 1.0001}));

  // Whole numbers of steps, up to rounding.
  CHECK(!RejectsSegment({.duration = 0.07}));
  CHECK(!RejectsSegment({.duration = 62.83}));

  // Too long, alone or together.
  CHECK(RejectsSegment({.duration = 1e9}));
  const std::size_t half_steps = (kMaxDriveSteps / 2) + 1;
  const double half = 0.01 * static_cast<double>(half_steps);
  const std::vector<DriveSegment> two_halves = {{.duration = half},
                                                {.duration = half}};
  CHECK(!GenerateScenario(two_halves, Exact()).has_value());
}

void TestEmptyDrive() {
  SyntheticConfig config = Exact();
  config.start_yaw = 1.0;
  config.start_speed = 3.0;
  const std::optional<Scenario> scenario = GenerateScenario({}, config);
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  CHECK(IsWellFormed(*scenario));
  CHECK(scenario->imu.size() == 1);
  CHECK(scenario->reference.size() == 1);
  CHECK(scenario->gnss.empty());  // The first fix would be at 0.5 s.
  const State2d& start = scenario->reference[0].state;
  CHECK(start.east == 0.0 && start.north == 0.0);
  CHECK(start.yaw == 1.0 && start.v == 3.0);

  config.gnss_offset = 0.0;
  const std::optional<Scenario> with_fix = GenerateScenario({}, config);
  CHECK(with_fix.has_value() && with_fix->gnss.size() == 1);
  if (with_fix.has_value() && with_fix->gnss.size() == 1) {
    CHECK(SameFix(with_fix->gnss[0], {.t = 0.0, .east = 0.0, .north = 0.0}));
  }
}

// One IMU step more than the segments hold, on the grid i * dt.
void TestTimeline() {
  const std::optional<Scenario> scenario =
      GenerateScenario(DemoDrive(), SyntheticConfig{});
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  CHECK(IsWellFormed(*scenario));
  CHECK(scenario->imu.size() == 6501);
  CHECK(scenario->reference.size() == 6501);
  bool on_grid = true;
  for (std::size_t i = 0; i < scenario->imu.size(); ++i) {
    const double t = static_cast<double>(i) * 0.01;
    on_grid = on_grid && scenario->imu[i].t == t &&
              scenario->reference[i].t == t;
  }
  CHECK(on_grid);
  CHECK(Near(scenario->reference.back().t, 65.0));
}

// imu[i] holds over [t[i], t[i+1]): it moves the state from step i to i + 1.
void TestInputHoldsUntilNextStep() {
  const std::vector<DriveSegment> drive = {
      {.duration = 1.0, .accel = 2.0, .yaw_rate = 0.0},
      {.duration = 1.0, .accel = 0.0, .yaw_rate = 0.5},
  };
  const std::optional<Scenario> scenario = GenerateScenario(drive, Exact());
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  const std::vector<PlanarImu>& imu = scenario->imu;
  const std::vector<ReferenceSample>& reference = scenario->reference;
  CHECK(imu.size() == 201);

  CHECK(imu[0].ax == 2.0 && imu[0].wz == 0.0);
  CHECK(imu[99].ax == 2.0 && imu[99].wz == 0.0);
  CHECK(imu[100].ax == 0.0 && imu[100].wz == 0.5);
  CHECK(imu[199].ax == 0.0 && imu[199].wz == 0.5);
  CHECK(imu[200].ax == 0.0 && imu[200].wz == 0.5);  // Last segment repeats.

  CHECK(reference[0].state.v == 0.0);
  CHECK(Near(reference[1].state.v, 0.02));  // imu[0] moved it.
  CHECK(Near(reference[100].state.v, 2.0));
  CHECK(reference[100].state.yaw == 0.0);  // The turn starts after t = 1.
  CHECK(Near(reference[101].state.yaw, 0.005));
  CHECK(Near(reference[200].state.yaw, 0.5));
  CHECK(Near(reference[200].state.v, 2.0));
}

void TestStraightLine() {
  SyntheticConfig config = Exact();
  config.start_speed = 10.0;
  const std::optional<Scenario> scenario =
      GenerateScenario({{.duration = 20.0}}, config);
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  bool matches = true;
  for (const ReferenceSample& sample : scenario->reference) {
    matches = matches && Near(sample.state.east, 10.0 * sample.t) &&
              sample.state.north == 0.0 && sample.state.yaw == 0.0 &&
              sample.state.v == 10.0;
  }
  CHECK(matches);
}

// x = a t^2 / 2 and v = a t, which the midpoint rule meets exactly.
void TestConstantAcceleration() {
  const std::optional<Scenario> scenario =
      GenerateScenario({{.duration = 10.0, .accel = 1.5}}, Exact());
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  bool matches = true;
  for (const ReferenceSample& sample : scenario->reference) {
    const double t = sample.t;
    matches = matches && Near(sample.state.east, 0.75 * t * t) &&
              Near(sample.state.v, 1.5 * t) && sample.state.north == 0.0;
  }
  CHECK(matches);
  CHECK(Near(scenario->reference.back().state.east, 75.0));
}

// 10 m/s at 0.1 rad/s to the left is a circle of radius 100 m around
// (0, 100). One lap takes 2 * pi / 0.1 = 62.83 s.
void TestCircle() {
  SyntheticConfig config = Exact();
  config.start_speed = 10.0;
  const std::optional<Scenario> scenario =
      GenerateScenario({{.duration = 62.83, .yaw_rate = 0.1}}, config);
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  bool on_circle = true;
  bool heading = true;
  for (const ReferenceSample& sample : scenario->reference) {
    const double radius =
        std::hypot(sample.state.east, sample.state.north - 100.0);
    on_circle = on_circle && Near(radius, 100.0, 1e-4);
    heading = heading && Near(sample.state.yaw, 0.1 * sample.t);
  }
  CHECK(on_circle);
  CHECK(heading);
  const State2d& end = scenario->reference.back().state;
  CHECK(std::hypot(end.east, end.north) < 0.05);  // Back at the start.
}

// From rest at 2 m/s^2, east = t^2 at any time, also between IMU steps.
void TestFixBetweenImuSteps() {
  SyntheticConfig config = Exact();
  config.gnss_offset = 0.505;
  const std::optional<Scenario> scenario =
      GenerateScenario({{.duration = 10.0, .accel = 2.0}}, config);
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  CHECK(IsWellFormed(*scenario));
  CHECK(scenario->gnss.size() == 10);  // 0.505, 1.505, ..., 9.505.
  for (std::size_t k = 0; k < scenario->gnss.size(); ++k) {
    const GnssFix& fix = scenario->gnss[k];
    CHECK(Near(fix.t, 0.505 + static_cast<double>(k)));
    const double steps = fix.t / config.imu_dt;
    CHECK(std::abs(steps - std::round(steps)) > 0.1);  // Off the IMU grid.
    CHECK(Near(fix.east, fix.t * fix.t));
    CHECK(fix.north == 0.0);
  }
}

void TestFixTimes() {
  const std::optional<Scenario> demo = GenerateScenario(DemoDrive(), Exact());
  CHECK(demo.has_value());
  if (demo.has_value()) {
    CHECK(demo->gnss.size() == 65);  // 0.5, 1.5, ..., 64.5.
    bool times = true;
    for (std::size_t k = 0; k < demo->gnss.size(); ++k) {
      times = times && Near(demo->gnss[k].t, 0.5 + static_cast<double>(k));
    }
    CHECK(times);
  }

  // A fix exactly at the end of the drive reads the last reference sample.
  SyntheticConfig config = Exact();
  config.start_speed = 1.0;
  config.gnss_offset = 0.0;
  config.gnss_period = 0.5;
  const std::optional<Scenario> scenario =
      GenerateScenario({{.duration = 2.0, .yaw_rate = 0.3}}, config);
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  CHECK(scenario->gnss.size() == 5);  // 0, 0.5, 1, 1.5, 2.
  if (scenario->gnss.size() == 5) {
    const State2d& end = scenario->reference.back().state;
    CHECK(scenario->gnss[4].t == 2.0);
    CHECK(Near(scenario->gnss[4].east, end.east));
    CHECK(Near(scenario->gnss[4].north, end.north));
  }
}

// The outage drops fixes in [start, start + duration) and nothing else.
void TestOutageWindow() {
  SyntheticConfig config = Exact();
  config.gnss_offset = 0.0;
  config.outage_start = 2.0;
  config.outage_duration = 3.0;
  const std::optional<Scenario> scenario =
      GenerateScenario({{.duration = 6.0}}, config);
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  const std::vector<double> expected = {0.0, 1.0, 5.0, 6.0};
  CHECK(scenario->gnss.size() == expected.size());
  if (scenario->gnss.size() == expected.size()) {
    for (std::size_t k = 0; k < expected.size(); ++k) {
      CHECK(Near(scenario->gnss[k].t, expected[k]));
    }
  }
}

// The default outage: no fix in [25, 40). The fixes outside it and the IMU
// read the same as in the same drive without an outage.
void TestDemoOutage() {
  const SyntheticConfig with_outage;
  SyntheticConfig without_outage;
  without_outage.outage_duration = 0.0;
  const std::optional<Scenario> cut =
      GenerateScenario(DemoDrive(), with_outage);
  const std::optional<Scenario> full =
      GenerateScenario(DemoDrive(), without_outage);
  CHECK(cut.has_value() && full.has_value());
  if (!cut.has_value() || !full.has_value()) {
    return;
  }
  CHECK(IsWellFormed(*cut));
  CHECK(full->gnss.size() == 65);
  CHECK(cut->gnss.size() == 50);

  std::vector<GnssFix> kept;
  for (const GnssFix& fix : full->gnss) {
    if (fix.t < 25.0 || fix.t >= 40.0) {
      kept.push_back(fix);
    }
  }
  CHECK(SameGnss(cut->gnss, kept));
  CHECK(SameImu(cut->imu, full->imu));

  bool has_before = false;
  bool has_after = false;
  for (const GnssFix& fix : cut->gnss) {
    CHECK(fix.t < 25.0 || fix.t >= 40.0);
    has_before = has_before || fix.t == 24.5;
    has_after = has_after || fix.t == 40.5;
  }
  CHECK(has_before && has_after);
}

// On a straight drive the errors are the readings minus the truth.
void TestNoiseStatistics() {
  SyntheticConfig config;
  config.start_speed = 10.0;
  config.outage_duration = 0.0;
  const std::optional<Scenario> scenario =
      GenerateScenario({{.duration = 200.0}}, config);
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  std::vector<double> accel_errors;
  std::vector<double> gyro_errors;
  for (const PlanarImu& sample : scenario->imu) {
    accel_errors.push_back(sample.ax);
    gyro_errors.push_back(sample.wz);
  }
  std::vector<double> gnss_errors;
  for (const GnssFix& fix : scenario->gnss) {
    gnss_errors.push_back(fix.east - (10.0 * fix.t));
    gnss_errors.push_back(fix.north);
  }
  CHECK(accel_errors.size() == 20001);
  CHECK(gnss_errors.size() == 400);

  // The tolerances are five or more standard errors wide.
  const Stats accel = Measure(accel_errors);
  CHECK(Near(accel.mean, config.accel_bias, 0.002));
  CHECK(Near(accel.deviation, config.accel_noise, 0.0025));
  const Stats gyro = Measure(gyro_errors);
  CHECK(Near(gyro.mean, config.gyro_bias, 0.0002));
  CHECK(Near(gyro.deviation, config.gyro_noise, 0.00025));
  const Stats gnss = Measure(gnss_errors);
  CHECK(Near(gnss.mean, 0.0, 0.5));
  CHECK(Near(gnss.deviation, config.gnss_noise, 0.4));
}

void TestSeeds() {
  SyntheticConfig config;
  const std::optional<Scenario> first = GenerateScenario(DemoDrive(), config);
  const std::optional<Scenario> again = GenerateScenario(DemoDrive(), config);
  config.seed = 2;
  const std::optional<Scenario> other = GenerateScenario(DemoDrive(), config);
  // The upper 32 bits of the seed count too.
  config.seed = 1 + (std::uint64_t{1} << 32U);
  const std::optional<Scenario> high = GenerateScenario(DemoDrive(), config);
  CHECK(first.has_value() && again.has_value() && other.has_value() &&
        high.has_value());
  if (!first.has_value() || !again.has_value() || !other.has_value() ||
      !high.has_value()) {
    return;
  }
  CHECK(SameImu(first->imu, again->imu));
  CHECK(SameGnss(first->gnss, again->gnss));
  CHECK(!SameImu(first->imu, other->imu));
  CHECK(!SameGnss(first->gnss, other->gnss));
  CHECK(!SameImu(first->imu, high->imu));
  CHECK(SameReference(first->reference, other->reference));
}

// The IMU and the GNSS draw from separate engines. Their errors do not
// repeat each other, and the GNSS settings leave the IMU noise alone.
void TestSensorsDrawSeparately() {
  SyntheticConfig config = Exact();
  config.accel_noise = 1.0;
  config.gnss_noise = 1.0;
  config.gnss_offset = 0.0;
  const std::optional<Scenario> scenario =
      GenerateScenario({{.duration = 1.0}}, config);
  config.gnss_noise = 5.0;
  config.gnss_period = 0.25;
  const std::optional<Scenario> other_gnss =
      GenerateScenario({{.duration = 1.0}}, config);
  CHECK(scenario.has_value() && other_gnss.has_value());
  if (!scenario.has_value() || !other_gnss.has_value() ||
      scenario->gnss.empty()) {
    return;
  }
  // At t = 0 the car stands at the origin, so the readings are the errors.
  CHECK(scenario->gnss[0].east != scenario->imu[0].ax);
  CHECK(SameImu(scenario->imu, other_gnss->imu));
}

// Noise changes what the sensors read, never where the car is.
void TestNoiseLeavesReferenceAlone() {
  const std::optional<Scenario> noisy =
      GenerateScenario(DemoDrive(), SyntheticConfig{});
  const std::optional<Scenario> exact = GenerateScenario(DemoDrive(), Exact());
  CHECK(noisy.has_value() && exact.has_value());
  if (noisy.has_value() && exact.has_value()) {
    CHECK(SameReference(noisy->reference, exact->reference));
    CHECK(!SameImu(noisy->imu, exact->imu));
  }
}

void TestDemoDrive() {
  const std::vector<DriveSegment> drive = DemoDrive();
  double duration = 0.0;
  for (const DriveSegment& segment : drive) {
    duration += segment.duration;
  }
  CHECK(Near(duration, 65.0));

  const std::optional<Scenario> scenario = GenerateScenario(drive, Exact());
  CHECK(scenario.has_value());
  if (!scenario.has_value()) {
    return;
  }
  const std::vector<ReferenceSample>& reference = scenario->reference;
  CHECK(reference.size() == 6501);
  CHECK(Near(reference[1000].state.v, 10.0));                     // 10 s.
  CHECK(Near(reference[3000].state.yaw, std::numbers::pi / 2.0));  // 30 s.
  CHECK(Near(reference[4500].state.yaw, std::numbers::pi / 2.0));  // 45 s.
  CHECK(Near(reference.back().state.yaw, 0.0));
  CHECK(Near(reference.back().state.v, 0.0));

  // The car only turns while it moves.
  bool moving = true;
  for (std::size_t i = 2000; i < 3000; ++i) {
    moving = moving && reference[i].state.v > 9.0;
  }
  for (std::size_t i = 4500; i < 5500; ++i) {
    moving = moving && reference[i].state.v > 9.0;
  }
  CHECK(moving);
}

}  // namespace

int main() {
  TestRejectsBadConfig();
  TestRejectsRepeatedFixTimes();
  TestRejectsBadSegment();
  TestEmptyDrive();
  TestTimeline();
  TestInputHoldsUntilNextStep();
  TestStraightLine();
  TestConstantAcceleration();
  TestCircle();
  TestFixBetweenImuSteps();
  TestFixTimes();
  TestOutageWindow();
  TestDemoOutage();
  TestNoiseStatistics();
  TestSeeds();
  TestSensorsDrawSeparately();
  TestNoiseLeavesReferenceAlone();
  TestDemoDrive();

  if (failure_count != 0) {
    std::cerr << failure_count << " checks failed\n";
    return 1;
  }

  std::cout << "All tests passed\n";
  return 0;
}
