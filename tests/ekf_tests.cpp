#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
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

void Check(bool ok, const char* text, int line) {
  if (!ok) {
    std::cerr << "line " << line << ": " << text << '\n';
    ++failure_count;
  }
}

#define CHECK(condition) \
  Check(static_cast<bool>(condition), #condition, __LINE__)

bool Near(double actual, double expected) {
  return std::abs(actual - expected) < 1e-9;
}

void TestPrediction() {
  const std::array imu{
      PlanarImu{.t = 0.0, .ax = 2.0},
      PlanarImu{.t = 1.0, .ax = -1.0},
      PlanarImu{.t = 3.0, .ax = 99.0},
  };
  const auto estimates = RunEkf(imu, {});
  CHECK(estimates.size() == imu.size());
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
  CHECK(Near(estimates.back().state.east, std::sqrt(2.0)));
  CHECK(Near(estimates.back().state.north, std::sqrt(2.0)));
  CHECK(Near(estimates.back().state.yaw, kTurn));
  CHECK(Near(estimates.back().state.v, 2.0));
}

void TestFixOnFirstStep() {
  const std::array imu{PlanarImu{.t = 0.0}};
  const std::array gnss{GnssFix{.t = 0.0, .east = 4.0, .north = -2.0}};
  const auto estimates = RunEkf(imu, gnss);
  CHECK(estimates.size() == 1);
  CHECK(estimates.front().updated);
  CHECK(Near(estimates.front().state.east, 2.0));
  CHECK(Near(estimates.front().state.north, -1.0));
  CHECK(Near(estimates.front().std_east, std::sqrt(2.0)));
  CHECK(RunEkf({}, {}).empty());
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
  CHECK(estimates.size() == drive->imu.size());
  const double outage_end = config.outage_start + config.outage_duration;
  std::vector<GnssFix> before_outage;
  for (const GnssFix& fix : drive->gnss) {
    if (fix.t < config.outage_start) {
      before_outage.push_back(fix);
    }
  }
  const auto without_reacquisition = RunEkf(drive->imu, before_outage);

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
      const GnssFix& fix = drive->gnss.at(fix_index++);
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
  CHECK(fix_index == drive->gnss.size());
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
  TestSyntheticOutage();
  return failure_count == 0 ? 0 : 1;
}
