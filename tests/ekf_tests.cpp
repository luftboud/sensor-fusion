#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "sensor_fusion/ekf/ekf.h"
#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/ekf/state.h"
#include "sensor_fusion/sim/synthetic_drive.h"

namespace {

using sensor_fusion::DemoDrive;
using sensor_fusion::EkfConfig;
using sensor_fusion::EkfEstimate;
using sensor_fusion::GenerateScenario;
using sensor_fusion::GnssFix;
using sensor_fusion::PlanarImu;
using sensor_fusion::RunEkf;
using sensor_fusion::State2d;
using sensor_fusion::SyntheticConfig;

int failure_count = 0;

bool Check(bool ok, const char* text, int line) {
  if (!ok) {
    std::cerr << "line " << line << ": " << text << '\n';
    ++failure_count;
  }
  return ok;
}

#define CHECK(condition) \
  Check(static_cast<bool>(condition), #condition, __LINE__)

bool Near(double actual, double expected) {
  return std::abs(actual - expected) < 1e-9;
}

template <typename Exception, typename Function>
bool Throws(Function run) {
  try {
    run();
  } catch (const Exception&) {
    return true;
  } catch (const std::exception&) {
    return false;
  }
  return false;
}

void TestPrediction() {
  const std::array imu{
      PlanarImu{.t = 0.0, .ax = 2.0},
      PlanarImu{.t = 1.0, .ax = -1.0},
      PlanarImu{.t = 3.0, .ax = 99.0},
  };
  const auto estimates = RunEkf(imu, {});
  if (!CHECK(estimates.size() == imu.size())) {
    return;
  }
  CHECK(Near(estimates[0].state.east, 0.0));
  CHECK(Near(estimates[1].state.east, 1.0));
  CHECK(Near(estimates[2].state.east, 3.0));
  CHECK(Near(estimates[2].state.north, 0.0));
  CHECK(Near(estimates[2].state.v, 0.0));
  CHECK(estimates.back().std_east > estimates.front().std_east);
  for (std::size_t i = 0; i < imu.size(); ++i) {
    CHECK(estimates[i].t == imu[i].t);
    CHECK(!estimates[i].updated);
  }
}

void TestOffGridFixes() {
  const std::array imu{PlanarImu{.t = 0.0}, PlanarImu{.t = 1.0},
                       PlanarImu{.t = 2.0}};
  const std::array gnss{
      GnssFix{.t = -1.0, .east = 100.0},
      GnssFix{.t = 0.25, .east = 1.5},
      GnssFix{.t = 0.75, .east = 2.5},
      GnssFix{.t = 3.0, .east = 100.0},
  };
  const EkfConfig config{
      .initial_state = {.v = 2.0},
      .initial_position_std = 1.0,
      .initial_yaw_std = 0.0,
      .initial_speed_std = 0.0,
      .accel_noise_density = 0.0,
      .gyro_noise_density = 0.0,
      .gnss_position_std = 1.0,
  };
  const auto estimates = RunEkf(imu, gnss, config);
  if (!CHECK(estimates.size() == imu.size())) {
    return;
  }
  CHECK(!estimates[0].updated);
  CHECK(estimates[1].updated);
  CHECK(!estimates[2].updated);
  CHECK(Near(estimates[1].state.east, 2.0 + 2.0 / 3.0));
  CHECK(Near(estimates[1].std_east, std::sqrt(1.0 / 3.0)));
  CHECK(Near(estimates[1].std_north, std::sqrt(1.0 / 3.0)));
  CHECK(Near(estimates[2].state.east, 4.0 + 2.0 / 3.0));
}

void TestTurningPrediction() {
  constexpr double kTurn = std::numbers::pi / 2.0;
  const std::array imu{PlanarImu{.t = 0.0, .wz = kTurn},
                       PlanarImu{.t = 1.0}};
  const EkfConfig config{.initial_state = {.v = 2.0}};
  const auto estimates = RunEkf(imu, {}, config);
  if (!CHECK(estimates.size() == imu.size())) {
    return;
  }
  CHECK(Near(estimates.back().state.east, std::sqrt(2.0)));
  CHECK(Near(estimates.back().state.north, std::sqrt(2.0)));
  CHECK(Near(estimates.back().state.yaw, kTurn));
  CHECK(Near(estimates.back().state.v, 2.0));
}

void TestFixOnFirstStep() {
  const std::array imu{PlanarImu{.t = 0.0}};
  const std::array gnss{GnssFix{.t = 0.0, .east = 4.0, .north = -2.0}};
  const auto estimates = RunEkf(imu, gnss);
  if (!CHECK(estimates.size() == imu.size())) {
    return;
  }
  CHECK(estimates.front().updated);
  CHECK(Near(estimates.front().state.east, 2.0));
  CHECK(Near(estimates.front().state.north, -1.0));
  CHECK(Near(estimates.front().std_east, std::sqrt(2.0)));
  CHECK(RunEkf({}, {}).empty());
}

void TestAccelerationNoiseCovariance() {
  const std::array imu{PlanarImu{.t = 0.0}, PlanarImu{.t = 2.0}};
  const EkfConfig config{
      .initial_position_std = 0.0,
      .initial_yaw_std = 0.0,
      .initial_speed_std = 0.0,
      .accel_noise_density = 0.5,
      .gyro_noise_density = 0.0,
  };
  const auto estimates = RunEkf(imu, {}, config);
  if (!CHECK(estimates.size() == imu.size())) {
    return;
  }
  // The speed increment has variance 0.5^2 * 2. Its effect on east is
  // scaled by dt / 2 = 1, and it has no effect on north.
  CHECK(Near(estimates.back().std_east, std::sqrt(0.5)));
  CHECK(Near(estimates.back().std_north, 0.0));
}

void TestFixCorrectsYawAndSpeed() {
  const std::array imu{PlanarImu{.t = 0.0}, PlanarImu{.t = 1.0}};
  const std::array gnss{GnssFix{.t = 1.0, .east = 4.0, .north = 2.0}};
  const EkfConfig config{
      .initial_state = {.v = 2.0},
      .initial_position_std = 0.0,
      .initial_yaw_std = 0.5,
      .initial_speed_std = 1.0,
      .accel_noise_density = 0.0,
      .gyro_noise_density = 0.0,
      .gnss_position_std = 1.0,
  };
  const auto estimates = RunEkf(imu, gnss, config);
  if (!CHECK(estimates.size() == imu.size())) {
    return;
  }
  const auto& corrected = estimates.back();
  CHECK(corrected.updated);
  CHECK(Near(corrected.state.east, 3.0));
  CHECK(Near(corrected.state.north, 1.0));
  CHECK(Near(corrected.state.yaw, 0.5));
  CHECK(Near(corrected.state.v, 3.0));
  CHECK(Near(corrected.std_east, std::sqrt(0.5)));
  CHECK(Near(corrected.std_north, std::sqrt(0.5)));
}

void TestSubnormalInnovation() {
  constexpr double kStd = 1e-155;
  const std::array imu{PlanarImu{}};
  const std::array gnss{GnssFix{.east = 4.0, .north = -2.0}};
  const EkfConfig config{.initial_position_std = kStd,
                         .gnss_position_std = kStd};
  const auto estimates = RunEkf(imu, gnss, config);
  if (!CHECK(estimates.size() == imu.size())) {
    return;
  }
  const auto& corrected = estimates.front();
  CHECK(corrected.updated);
  CHECK(Near(corrected.state.east, 2.0));
  CHECK(Near(corrected.state.north, -1.0));
  // Compare at the scale of the input rather than using absolute tolerance.
  CHECK(Near(corrected.std_east / kStd, std::sqrt(0.5)));
  CHECK(Near(corrected.std_north / kStd, std::sqrt(0.5)));
}

void TestGainOverflowWithFiniteCorrection() {
  constexpr double kTime = 1e-316;
  constexpr double kSpeedStd = 1e154;
  constexpr double kGnssStd = 3e-162;
  const std::array imu{PlanarImu{}, PlanarImu{.t = kTime}};
  const EkfConfig config{
      .initial_position_std = 0.0,
      .initial_yaw_std = 0.0,
      .initial_speed_std = kSpeedStd,
      .accel_noise_density = 0.0,
      .gyro_noise_density = 0.0,
      .gnss_position_std = kGnssStd,
  };
  const double position_std = kTime * kSpeedStd;
  const double position_to_gnss = position_std / kGnssStd;
  for (const double east : std::array{0.0, position_std}) {
    const std::array gnss{GnssFix{.t = kTime, .east = east}};
    const auto estimates = RunEkf(imu, gnss, config);
    if (!CHECK(estimates.size() == imu.size())) {
      return;
    }
    const auto& corrected = estimates.back();
    const double expected_speed_ratio =
        (east / kGnssStd) * position_to_gnss /
        (1.0 + position_to_gnss * position_to_gnss);
    CHECK(corrected.updated);
    CHECK(Near(corrected.state.v / kSpeedStd, expected_speed_ratio));
  }
}

void TestGainUnderflowWithFiniteCorrection() {
  const std::array imu{PlanarImu{}};
  const std::array gnss{GnssFix{.east = 1e200}};
  const EkfConfig config{.initial_position_std = 1e-100,
                         .gnss_position_std = 1e100};
  const auto estimates = RunEkf(imu, gnss, config);
  if (!CHECK(estimates.size() == imu.size())) {
    return;
  }
  CHECK(estimates.front().updated);
  CHECK(Near(estimates.front().state.east / 1e-200, 1.0));
}

void TestNearlySingularInnovation() {
  const std::array imu{PlanarImu{.t = 0.0}, PlanarImu{.t = 1.0}};
  const std::array gnss{GnssFix{.t = 1.0, .east = 1.0, .north = 1.0}};
  // Speed uncertainty projects onto the diagonal heading. GNSS variance
  // is too small to change either positional variance when added directly.
  const EkfConfig config{
      .initial_state = {.yaw = std::numbers::pi / 4.0},
      .initial_position_std = 0.0,
      .initial_yaw_std = 0.0,
      .initial_speed_std = 2.0,
      .accel_noise_density = 0.0,
      .gyro_noise_density = 0.0,
      .gnss_position_std = 1e-9,
  };
  const auto estimates = RunEkf(imu, gnss, config);
  if (!CHECK(estimates.size() == imu.size())) {
    return;
  }
  const auto& corrected = estimates.back();
  CHECK(corrected.updated);
  CHECK(Near(corrected.state.east, 1.0));
  CHECK(Near(corrected.state.north, 1.0));
  CHECK(Near(corrected.state.v, std::sqrt(2.0)));
  CHECK(corrected.std_east >= 0.0 && corrected.std_east < 1e-8);
  CHECK(corrected.std_north >= 0.0 && corrected.std_north < 1e-8);
}

void TestCrossTrackFixesWithRankOneCovariance() {
  const std::array imu{PlanarImu{}, PlanarImu{.t = 1.0},
                       PlanarImu{.t = 2.0}, PlanarImu{.t = 3.0}};
  constexpr std::array headings{std::numbers::pi / 180.0,
                                89.0 * std::numbers::pi / 180.0,
                                std::numbers::pi / 2.0};
  for (const double yaw : headings) {
    const double east = -std::sin(yaw);
    const double north = std::cos(yaw);
    const std::array gnss{GnssFix{.t = 1.0, .east = east, .north = north},
                          GnssFix{.t = 2.0, .east = east, .north = north},
                          GnssFix{.t = 3.0, .east = east, .north = north}};
    const EkfConfig config{
        .initial_state = {.yaw = yaw},
        .initial_position_std = 0.0,
        .initial_yaw_std = 0.0,
        .initial_speed_std = 2.0,
        .accel_noise_density = 0.0,
        .gyro_noise_density = 0.0,
        .gnss_position_std = 1e-9,
    };
    const auto estimates = RunEkf(imu, gnss, config);
    if (!CHECK(estimates.size() == imu.size())) {
      return;
    }
    for (std::size_t i = 1; i < estimates.size(); ++i) {
      const auto& estimate = estimates[i];
      CHECK(estimate.updated);
      CHECK(Near(estimate.state.east, 0.0));
      CHECK(Near(estimate.state.north, 0.0));
      CHECK(Near(estimate.state.yaw, yaw));
      CHECK(Near(estimate.state.v, 0.0));
      // Only speed is uncertain. Fits at times 1, 2 and 3 add information
      // proportional to 1, 4 and 9, and position at time t is t times speed.
      constexpr std::array information{1.0, 5.0, 14.0};
      const double position_std =
          config.gnss_position_std * imu[i].t / std::sqrt(information[i - 1]);
      CHECK(Near(estimate.std_east / position_std, std::abs(north)));
      CHECK(Near(estimate.std_north / position_std, std::abs(east)));
    }
  }
}

void TestInvalidInputs() {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  const std::array imu{PlanarImu{}};
  const std::array duplicate_imu{PlanarImu{}, PlanarImu{}};
  const std::array duplicate_gnss{GnssFix{}, GnssFix{}};
  const std::array nan_time{PlanarImu{.t = kNan}};
  const std::array nan_imu{PlanarImu{.ax = kNan}};
  const std::array nan_gnss{GnssFix{.north = kNan}};
  CHECK(Throws<std::invalid_argument>([&] { RunEkf(duplicate_imu, {}); }));
  CHECK(Throws<std::invalid_argument>([&] { RunEkf(imu, duplicate_gnss); }));
  CHECK(Throws<std::invalid_argument>([&] { RunEkf(nan_time, {}); }));
  CHECK(Throws<std::invalid_argument>([&] { RunEkf(nan_imu, {}); }));
  CHECK(Throws<std::invalid_argument>([&] { RunEkf(imu, nan_gnss); }));
  CHECK(Throws<std::invalid_argument>(
      [&] { RunEkf(imu, {}, {.accel_noise_density = -1.0}); }));
  CHECK(Throws<std::invalid_argument>(
      [&] { RunEkf(imu, {}, {.gnss_position_std = 0.0}); }));
  CHECK(Throws<std::invalid_argument>([&] {
    RunEkf(imu, {}, {.gnss_position_std = 1e-200});
  }));
  CHECK(Throws<std::invalid_argument>(
      [&] { RunEkf(imu, {}, {.initial_state = {.v = kNan}}); }));
}

void TestNumericalFailures() {
  constexpr double kMax = std::numeric_limits<double>::max();
  const std::array huge_interval{PlanarImu{.t = -kMax}, PlanarImu{.t = kMax}};
  const std::array huge_acceleration{PlanarImu{.ax = kMax},
                                     PlanarImu{.t = 2.0}};
  const std::array imu{PlanarImu{}};
  const std::array gnss{GnssFix{.east = kMax}};
  const EkfConfig far_west{.initial_state = {.east = -kMax}};
  CHECK(Throws<std::overflow_error>([&] { RunEkf(huge_interval, {}); }));
  CHECK(Throws<std::overflow_error>([&] { RunEkf(huge_acceleration, {}); }));
  CHECK(Throws<std::overflow_error>([&] { RunEkf(imu, gnss, far_west); }));

  const std::array long_step{PlanarImu{}, PlanarImu{.t = 1e5}};
  CHECK(Throws<std::runtime_error>(
      [&] { RunEkf(long_step, {}, {.accel_noise_density = 1e150}); }));
}

double PositionSquaredError(const State2d& estimate, const State2d& truth) {
  const double east = estimate.east - truth.east;
  const double north = estimate.north - truth.north;
  return east * east + north * north;
}

void TestSyntheticOutage() {
  const SyntheticConfig config;
  const auto drive = GenerateScenario(DemoDrive(), config);
  if (!drive) {
    throw std::runtime_error("Could not generate the synthetic drive");
  }

  const auto estimates = RunEkf(drive->imu, drive->gnss);
  const auto imu_only = RunEkf(drive->imu, {});
  const double outage_end = config.outage_start + config.outage_duration;
  std::vector<GnssFix> before_outage;
  for (const GnssFix& fix : drive->gnss) {
    if (fix.t < config.outage_start) {
      before_outage.push_back(fix);
    }
  }
  const auto without_reacquisition = RunEkf(drive->imu, before_outage);
  const std::size_t sample_count = drive->imu.size();
  if (!CHECK(sample_count > 0 && drive->reference.size() == sample_count &&
             estimates.size() == sample_count &&
             imu_only.size() == sample_count &&
             without_reacquisition.size() == sample_count)) {
    return;
  }

  double ekf_squares = 0.0;
  double imu_squares = 0.0;
  double ekf_at_fixes_squares = 0.0;
  double gnss_squares = 0.0;
  double reacquisition_error = 0.0;
  double uncorrected_error = 0.0;
  std::size_t fix_index = 0;
  bool reacquired = false;
  for (std::size_t i = 0; i < estimates.size(); ++i) {
    const EkfEstimate& estimate = estimates[i];
    const State2d& truth = drive->reference[i].state;
    const double ekf_square_error = PositionSquaredError(estimate.state, truth);
    CHECK(estimate.t == drive->reference[i].t);
    CHECK(std::isfinite(estimate.std_east) &&
          std::isfinite(estimate.std_north));
    ekf_squares += ekf_square_error;
    imu_squares += PositionSquaredError(imu_only[i].state, truth);
    if (estimate.t >= config.outage_start && estimate.t < outage_end) {
      CHECK(!estimate.updated);
    }
    if (estimate.updated) {
      if (!CHECK(fix_index < drive->gnss.size())) {
        return;
      }
      const GnssFix& fix = drive->gnss[fix_index++];
      CHECK(Near(fix.t, estimate.t));  // The demo fixes share an IMU timestamp.
      gnss_squares += PositionSquaredError(
          State2d{.east = fix.east, .north = fix.north}, truth);
      ekf_at_fixes_squares += ekf_square_error;
      if (!reacquired && estimate.t >= outage_end) {
        reacquired = true;
        reacquisition_error = std::sqrt(ekf_square_error);
        uncorrected_error = std::sqrt(
            PositionSquaredError(without_reacquisition[i].state, truth));
      }
    }
  }

  CHECK(reacquired);
  if (!CHECK(fix_index > 0 && fix_index == drive->gnss.size())) {
    return;
  }
  CHECK(ekf_squares < imu_squares);
  CHECK(ekf_at_fixes_squares <= 0.7 * 0.7 * gnss_squares);
  CHECK(3.0 * reacquisition_error <= uncorrected_error);
  const auto imu_count = static_cast<double>(estimates.size());
  const auto fix_count = static_cast<double>(fix_index);
  std::cout << "Position RMSE over the drive (m): EKF "
            << std::sqrt(ekf_squares / imu_count) << ", IMU-only "
            << std::sqrt(imu_squares / imu_count) << '\n'
            << "Position RMSE at GNSS fixes (m): EKF "
            << std::sqrt(ekf_at_fixes_squares / fix_count) << ", raw GNSS "
            << std::sqrt(gnss_squares / fix_count) << '\n'
            << "Position error at reacquisition (m): with correction "
            << reacquisition_error << ", without correction "
            << uncorrected_error << '\n';
}

}  // namespace

int main() {
  TestPrediction();
  TestOffGridFixes();
  TestTurningPrediction();
  TestFixOnFirstStep();
  TestAccelerationNoiseCovariance();
  TestFixCorrectsYawAndSpeed();
  TestSubnormalInnovation();
  TestGainOverflowWithFiniteCorrection();
  TestGainUnderflowWithFiniteCorrection();
  TestNearlySingularInnovation();
  TestCrossTrackFixesWithRankOneCovariance();
  TestInvalidInputs();
  TestNumericalFailures();
  TestSyntheticOutage();
  return failure_count == 0 ? 0 : 1;
}
