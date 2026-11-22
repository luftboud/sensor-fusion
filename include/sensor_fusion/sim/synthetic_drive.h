#ifndef SENSOR_FUSION_SIM_SYNTHETIC_DRIVE_H_
#define SENSOR_FUSION_SIM_SYNTHETIC_DRIVE_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <optional>
#include <random>
#include <vector>

#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/ekf/state.h"

namespace sensor_fusion {

// Synthetic 2D drive for the filter. The car follows a planned list of
// segments, and the generator writes three series into a Scenario:
//
//   reference  the exact state at every IMU step,
//   imu        the exact ax and wz plus a bias and white noise,
//   gnss       the exact position at each fix time plus white noise, with
//              no fixes inside the outage window.
//
// Input timing: imu[i] holds over [t[i], t[i+1]). The state at t[i+1] is the
// state at t[i] moved by imu[i] for dt = t[i+1] - t[i]. The last sample has
// no next step, and it repeats the inputs of the last segment.
//
//   const std::optional<Scenario> drive =
//       GenerateScenario(DemoDrive(), SyntheticConfig{});

// Most IMU steps, and most GNSS fixes, in one drive. 1'000'000 steps at
// 100 Hz is about 2.8 hours and some 64 MB of samples. The limit keeps a bad
// config from allocating without bound.
inline constexpr std::size_t kMaxDriveSteps = 1'000'000;

// One piece of a drive. The forward acceleration and the yaw rate stay
// constant for the whole piece.
struct DriveSegment {
  double duration = 0.0;  // Seconds, a whole number of IMU steps.
  double accel = 0.0;     // Forward acceleration (m/s^2).
  double yaw_rate = 0.0;  // Rad/s. Positive turns left.
};

// Timing and sensor settings. A noise value is the standard deviation of
// white noise with one draw per sample. A bias is a constant offset added to
// every sample of that sensor. The defaults are a cheap IMU, such as the one
// in a phone, and a consumer GNSS receiver.
struct SyntheticConfig {
  double imu_dt = 0.01;           // Seconds between IMU steps.
  double start_yaw = 0.0;         // Heading at t = 0 (rad). 0 is east.
  double start_speed = 0.0;       // Forward speed at t = 0 (m/s).
  double gnss_period = 1.0;       // Seconds between GNSS fixes.
  double gnss_offset = 0.5;       // Time of the first fix (s).
  double outage_start = 25.0;     // First second without fixes.
  double outage_duration = 15.0;  // Seconds without fixes. 0 turns it off.
  double accel_noise = 0.05;      // m/s^2.
  double accel_bias = 0.02;       // m/s^2.
  double gyro_noise = 0.005;      // Rad/s.
  double gyro_bias = 0.002;       // Rad/s.
  double gnss_noise = 2.0;        // Metres, on east and on north.
  std::uint64_t seed = 1;         // Same seed, same noise.
};

namespace internal {

// Normal noise from std::mt19937_64.
//
// The C++ standard fixes the output of std::mt19937_64 and std::seed_seq,
// but not the algorithm of std::normal_distribution. libstdc++ and libc++
// would give different noise for one seed. The Box-Muller transform on the
// raw engine output gives the same draws with GCC and Clang.
class GaussianNoise {
 public:
  // stream separates the sensors: the IMU and the GNSS draw from different
  // engines, so the GNSS settings do not change the IMU noise.
  GaussianNoise(std::uint64_t seed, std::uint32_t stream) {
    // std::seed_seq keeps only 32 bits of each value, so the 64-bit seed goes
    // in as two halves.
    std::seed_seq seeds{static_cast<std::uint32_t>(seed),
                        static_cast<std::uint32_t>(seed >> 32U), stream};
    engine_.seed(seeds);
  }

  // Returns a draw from N(0, sigma^2). A call with sigma 0 still uses up a
  // draw, so turning one noise off keeps the draws of the others.
  double Next(double sigma) { return sigma * NextStandard(); }

 private:
  // Uniform in (0, 1]: the top 53 bits of the engine output, plus one, times
  // 2^-53. Zero is excluded because Box-Muller takes its logarithm.
  double Uniform() {
    constexpr double kTwoToMinus53 = 0x1.0p-53;
    return (static_cast<double>(engine_() >> 11U) + 1.0) * kTwoToMinus53;
  }

  // Box-Muller makes two independent draws at once. The second waits in
  // spare_ for the next call.
  double NextStandard() {
    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    const double radius = std::sqrt(-2.0 * std::log(Uniform()));
    const double angle = 2.0 * std::numbers::pi * Uniform();
    spare_ = radius * std::sin(angle);
    has_spare_ = true;
    return radius * std::cos(angle);
  }

  std::mt19937_64 engine_;
  double spare_ = 0.0;
  bool has_spare_ = false;
};

// Moves a state by dt with the segment's acceleration and yaw rate. The
// midpoint rule is exact for a straight line at constant acceleration. On a
// turn, the step comes out shorter than the arc by about
// (yaw_rate * dt)^2 / 24 of its length.
inline State2d StepState(const State2d& state, const DriveSegment& segment,
                         double dt) {
  const double v_mid = state.v + (0.5 * segment.accel * dt);
  const double yaw_mid = state.yaw + (0.5 * segment.yaw_rate * dt);
  return {
      .east = state.east + (v_mid * std::cos(yaw_mid) * dt),
      .north = state.north + (v_mid * std::sin(yaw_mid) * dt),
      .yaw = state.yaw + (segment.yaw_rate * dt),
      .v = state.v + (segment.accel * dt),
  };
}

// Returns the number of steps of dt in duration. Returns nullopt for a
// negative or non-finite duration, for one that is not a whole number of
// steps up to rounding, and for more than kMaxDriveSteps.
inline std::optional<std::size_t> StepCount(double duration, double dt) {
  if (!std::isfinite(duration) || duration < 0.0) {
    return std::nullopt;
  }
  const double steps = std::round(duration / dt);
  if (steps > static_cast<double>(kMaxDriveSteps)) {
    return std::nullopt;
  }
  constexpr double kRelativeTolerance = 1e-9;
  if (std::abs((steps * dt) - duration) >
      kRelativeTolerance * std::fmax(1.0, duration)) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(steps);
}

inline bool IsFiniteNonNegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

inline bool IsValidConfig(const SyntheticConfig& config) {
  return std::isfinite(config.imu_dt) && config.imu_dt > 0.0 &&
         std::isfinite(config.start_yaw) && std::isfinite(config.start_speed) &&
         std::isfinite(config.gnss_period) && config.gnss_period > 0.0 &&
         IsFiniteNonNegative(config.gnss_offset) &&
         std::isfinite(config.outage_start) &&
         IsFiniteNonNegative(config.outage_duration) &&
         IsFiniteNonNegative(config.accel_noise) &&
         std::isfinite(config.accel_bias) &&
         IsFiniteNonNegative(config.gyro_noise) &&
         std::isfinite(config.gyro_bias) &&
         IsFiniteNonNegative(config.gnss_noise);
}

// Expands the segments into one input per IMU step: inputs[i] holds over
// [t[i], t[i+1]). Returns nullopt for a bad segment or a drive that is too
// long. Every segment is checked before anything is allocated.
inline std::optional<std::vector<DriveSegment>> StepInputs(
    const std::vector<DriveSegment>& drive, double dt) {
  std::vector<std::size_t> counts;
  counts.reserve(drive.size());
  std::size_t total = 0;
  for (const DriveSegment& segment : drive) {
    if (!std::isfinite(segment.accel) || !std::isfinite(segment.yaw_rate)) {
      return std::nullopt;
    }
    const std::optional<std::size_t> steps = StepCount(segment.duration, dt);
    if (!steps.has_value() || *steps > kMaxDriveSteps - total) {
      return std::nullopt;
    }
    counts.push_back(*steps);
    total += *steps;
  }
  std::vector<DriveSegment> inputs;
  inputs.reserve(total);
  for (std::size_t k = 0; k < drive.size(); ++k) {
    inputs.insert(inputs.end(), counts[k], drive[k]);
  }
  return inputs;
}

constexpr std::uint32_t kImuStream = 1;
constexpr std::uint32_t kGnssStream = 2;

}  // namespace internal

// Builds a drive from the segments and the settings. The drive starts at
// t = 0 at the origin with config.start_yaw and config.start_speed, and it
// has one IMU step more than the segments hold, so the last step lands on
// the end of the drive. Fixes fall at gnss_offset + k * gnss_period up to the
// end of the drive, except in [outage_start, outage_start + outage_duration).
//
// The result passes IsWellFormed(). Returns nullopt when a setting or a
// segment is invalid: see internal::IsValidConfig() and
// internal::StepInputs().
//
// The generator does not stop the car from turning at zero speed. The
// segments decide how the car moves.
inline std::optional<Scenario> GenerateScenario(
    const std::vector<DriveSegment>& drive, const SyntheticConfig& config) {
  if (!internal::IsValidConfig(config)) {
    return std::nullopt;
  }
  const double dt = config.imu_dt;
  const std::optional<std::vector<DriveSegment>> inputs =
      internal::StepInputs(drive, dt);
  if (!inputs.has_value()) {
    return std::nullopt;
  }
  const std::size_t step_count = inputs->size();
  const double end_time = static_cast<double>(step_count) * dt;

  std::size_t fix_count = 0;
  if (config.gnss_offset <= end_time) {
    const double count =
        std::floor((end_time - config.gnss_offset) / config.gnss_period) + 1.0;
    if (count > static_cast<double>(kMaxDriveSteps)) {
      return std::nullopt;
    }
    fix_count = static_cast<std::size_t>(count);
  }

  Scenario scenario;
  scenario.imu.reserve(step_count + 1);
  scenario.reference.reserve(step_count + 1);

  // After the last segment its inputs hold. Without segments the car keeps
  // its start speed and heading.
  const DriveSegment last = inputs->empty() ? DriveSegment{} : inputs->back();
  internal::GaussianNoise imu_noise(config.seed, internal::kImuStream);
  State2d state{
      .east = 0.0,
      .north = 0.0,
      .yaw = config.start_yaw,
      .v = config.start_speed,
  };
  for (std::size_t i = 0; i <= step_count; ++i) {
    // i * dt, not a running sum, so that time does not collect rounding.
    const double t = static_cast<double>(i) * dt;
    const DriveSegment& input = i < step_count ? (*inputs)[i] : last;
    const double accel_error = imu_noise.Next(config.accel_noise);
    const double gyro_error = imu_noise.Next(config.gyro_noise);
    scenario.reference.push_back({.t = t, .state = state});
    scenario.imu.push_back({
        .t = t,
        .ax = input.accel + config.accel_bias + accel_error,
        .wz = input.yaw_rate + config.gyro_bias + gyro_error,
    });
    if (i < step_count) {
      state = internal::StepState(state, input, dt);
    }
  }

  // Every fix draws its noise, also inside the outage. A fix outside the
  // outage then reads the same with and without it.
  internal::GaussianNoise gnss_noise(config.seed, internal::kGnssStream);
  const double outage_end = config.outage_start + config.outage_duration;
  std::size_t i = 0;  // Last IMU step at or before the fix.
  for (std::size_t k = 0; k < fix_count; ++k) {
    const double t =
        config.gnss_offset + (static_cast<double>(k) * config.gnss_period);
    if (t > end_time) {
      break;
    }
    while (i < step_count && scenario.reference[i + 1].t <= t) {
      ++i;
    }
    const ReferenceSample& before = scenario.reference[i];
    const State2d truth =
        i < step_count
            ? internal::StepState(before.state, (*inputs)[i], t - before.t)
            : before.state;
    const double east_error = gnss_noise.Next(config.gnss_noise);
    const double north_error = gnss_noise.Next(config.gnss_noise);
    if (t >= config.outage_start && t < outage_end) {
      continue;
    }
    scenario.gnss.push_back({
        .t = t,
        .east = truth.east + east_error,
        .north = truth.north + north_error,
    });
  }
  return scenario;
}

// The demo drive of issue #12, 65 seconds. With the default config the
// outage covers 25 to 40 s: the second half of the left turn and the
// straight after it.
//
//    0-10 s  speed up from rest to 10 m/s
//   10-20 s  straight
//   20-30 s  quarter turn left, radius about 64 m
//   30-45 s  straight, heading north
//   45-55 s  quarter turn right
//   55-65 s  brake to a stop, heading east
inline std::vector<DriveSegment> DemoDrive() {
  constexpr double kQuarterTurnRate = std::numbers::pi / 2.0 / 10.0;
  return {
      {.duration = 10.0, .accel = 1.0, .yaw_rate = 0.0},
      {.duration = 10.0, .accel = 0.0, .yaw_rate = 0.0},
      {.duration = 10.0, .accel = 0.0, .yaw_rate = kQuarterTurnRate},
      {.duration = 15.0, .accel = 0.0, .yaw_rate = 0.0},
      {.duration = 10.0, .accel = 0.0, .yaw_rate = -kQuarterTurnRate},
      {.duration = 10.0, .accel = -1.0, .yaw_rate = 0.0},
  };
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_SIM_SYNTHETIC_DRIVE_H_
