#ifndef SENSOR_FUSION_EKF_EKF_H_
#define SENSOR_FUSION_EKF_EKF_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

#include "matrix/matrix.h"
#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/ekf/state.h"

namespace sensor_fusion {

struct EkfConfig {
  State2d initial_state = {};
  double initial_position_std = 2.0;  // Metres, per axis.
  double initial_yaw_std = 0.2;       // Radians.
  double initial_speed_std = 1.0;     // Metres per second.

  // Process noise densities. Their squares times dt give the variances of
  // the velocity and yaw increments over dt.
  double accel_noise_density = 0.3;  // (m/s^2) / sqrt(Hz).
  double gyro_noise_density = 0.03;  // (rad/s) / sqrt(Hz).
  double gnss_position_std = 2.0;    // Metres, per axis. Must be positive.
};

namespace internal {

class PlanarEkf {
 public:
  explicit PlanarEkf(const EkfConfig& config)
      : state_(config.initial_state),
        accel_noise_power_(config.accel_noise_density *
                           config.accel_noise_density),
        gyro_noise_power_(config.gyro_noise_density *
                          config.gyro_noise_density) {
    const std::array deviations{
        config.initial_position_std, config.initial_yaw_std,
        config.initial_speed_std, config.accel_noise_density,
        config.gyro_noise_density, config.gnss_position_std,
    };
    for (const double deviation : deviations) {
      if (deviation < 0.0 || !std::isfinite(deviation * deviation)) {
        throw std::invalid_argument("EKF noise must be nonnegative and finite");
      }
    }
    const double gnss_variance =
        config.gnss_position_std * config.gnss_position_std;
    if (gnss_variance == 0.0 || !std::isfinite(state_.east) ||
        !std::isfinite(state_.north) || !std::isfinite(state_.yaw) ||
        !std::isfinite(state_.v)) {
      throw std::invalid_argument(
          "EKF requires a finite initial state and positive GNSS variance");
    }

    covariance_(kEast, kEast) =
        config.initial_position_std * config.initial_position_std;
    covariance_(kNorth, kNorth) = covariance_(kEast, kEast);
    covariance_(kYaw, kYaw) = config.initial_yaw_std * config.initial_yaw_std;
    covariance_(kSpeed, kSpeed) =
        config.initial_speed_std * config.initial_speed_std;

    observation_(0, kEast) = 1.0;
    observation_(1, kNorth) = 1.0;
    measurement_noise_(0, 0) = gnss_variance;
    measurement_noise_(1, 1) = gnss_variance;
  }

  void Predict(const PlanarImu& imu, double dt) {
    const double delta_speed = imu.ax * dt;
    const double delta_yaw = imu.wz * dt;
    const double yaw_mid = state_.yaw + 0.5 * delta_yaw;
    const double distance = (state_.v + 0.5 * delta_speed) * dt;
    const double cos_yaw = std::cos(yaw_mid);
    const double sin_yaw = std::sin(yaw_mid);

    auto transition = identity<double, kStateSize>();
    transition(kEast, kYaw) = -distance * sin_yaw;
    transition(kNorth, kYaw) = distance * cos_yaw;
    transition(kEast, kSpeed) = dt * cos_yaw;
    transition(kNorth, kSpeed) = dt * sin_yaw;

    // Differentiate the midpoint step with respect to the noisy increments
    // in speed and yaw. Each increment also affects the midpoint position.
    Matrix<double, kStateSize, 2> noise_jacobian;
    noise_jacobian(kEast, 0) = 0.5 * transition(kEast, kSpeed);
    noise_jacobian(kNorth, 0) = 0.5 * transition(kNorth, kSpeed);
    noise_jacobian(kSpeed, 0) = 1.0;
    noise_jacobian(kEast, 1) = 0.5 * transition(kEast, kYaw);
    noise_jacobian(kNorth, 1) = 0.5 * transition(kNorth, kYaw);
    noise_jacobian(kYaw, 1) = 1.0;
    SquareMatrix<double, 2> increment_noise;
    increment_noise(0, 0) = accel_noise_power_ * dt;
    increment_noise(1, 1) = gyro_noise_power_ * dt;

    covariance_ = transition * covariance_ * transpose(transition) +
                  noise_jacobian * increment_noise * transpose(noise_jacobian);

    state_.east += distance * cos_yaw;
    state_.north += distance * sin_yaw;
    state_.yaw += delta_yaw;
    state_.v += delta_speed;
  }

  void Update(const GnssFix& fix) {
    Vector<double, 2> residual;
    residual(0, 0) = fix.east - state_.east;
    residual(1, 0) = fix.north - state_.north;

    const auto cross_covariance = covariance_ * transpose(observation_);
    const auto innovation_covariance =
        observation_ * cross_covariance + measurement_noise_;
    const auto gain = cross_covariance * inverse(innovation_covariance);
    const auto correction = gain * residual;

    state_.east += correction(kEast, 0);
    state_.north += correction(kNorth, 0);
    state_.yaw += correction(kYaw, 0);
    state_.v += correction(kSpeed, 0);

    // Joseph form avoids subtracting nearly equal covariance matrices.
    const auto residual_transform =
        identity<double, kStateSize>() - gain * observation_;
    covariance_ =
        residual_transform * covariance_ * transpose(residual_transform) +
        gain * measurement_noise_ * transpose(gain);
    covariance_ = (covariance_ + transpose(covariance_)) * 0.5;
  }

  EkfEstimate Estimate(double t, bool updated) const {
    return {
        .t = t,
        .state = state_,
        .std_east = std::sqrt(covariance_(kEast, kEast)),
        .std_north = std::sqrt(covariance_(kNorth, kNorth)),
        .updated = updated,
    };
  }

 private:
  static constexpr std::size_t kEast = 0;
  static constexpr std::size_t kNorth = 1;
  static constexpr std::size_t kYaw = 2;
  static constexpr std::size_t kSpeed = 3;
  static constexpr std::size_t kStateSize = 4;

  State2d state_;
  double accel_noise_power_;
  double gyro_noise_power_;
  SquareMatrix<double, kStateSize> covariance_;
  Matrix<double, 2, kStateSize> observation_;
  SquareMatrix<double, 2> measurement_noise_;
};

}  // namespace internal

// Returns one estimate per IMU timestamp. The initial state is at imu[0].t.
// Each fix in the IMU time range is applied at its own timestamp and marks
// the first estimate at or after that timestamp as updated. Fixes outside
// the IMU time range are ignored. Empty GNSS gives an IMU-only trajectory.
// Both streams must contain finite values and have strictly increasing times.
// Throws std::invalid_argument for invalid inputs or configuration.
inline std::vector<EkfEstimate> RunEkf(std::span<const PlanarImu> imu,
                                     std::span<const GnssFix> gnss,
                                     const EkfConfig& config = {}) {
  internal::PlanarEkf filter(config);
  for (std::size_t i = 0; i < imu.size(); ++i) {
    const PlanarImu& sample = imu[i];
    if (!std::isfinite(sample.t) || !std::isfinite(sample.ax) ||
        !std::isfinite(sample.wz) ||
        (i > 0 && sample.t <= imu[i - 1].t)) {
      throw std::invalid_argument("Invalid EKF IMU series");
    }
  }
  for (std::size_t i = 0; i < gnss.size(); ++i) {
    const GnssFix& fix = gnss[i];
    if (!std::isfinite(fix.t) || !std::isfinite(fix.east) ||
        !std::isfinite(fix.north) || (i > 0 && fix.t <= gnss[i - 1].t)) {
      throw std::invalid_argument("Invalid EKF GNSS series");
    }
  }
  if (imu.empty()) {
    return {};
  }

  std::vector<EkfEstimate> estimates;
  estimates.reserve(imu.size());
  double time = imu.front().t;
  std::size_t next_fix = 0;
  while (next_fix < gnss.size() && gnss[next_fix].t < time) {
    ++next_fix;
  }
  const bool initial_fix = next_fix < gnss.size() && gnss[next_fix].t == time;
  if (initial_fix) {
    filter.Update(gnss[next_fix++]);
  }
  estimates.push_back(filter.Estimate(time, initial_fix));

  for (std::size_t i = 1; i < imu.size(); ++i) {
    const double step_end = imu[i].t;
    const std::size_t first_fix_on_step = next_fix;
    while (next_fix < gnss.size() && gnss[next_fix].t <= step_end) {
      const GnssFix& fix = gnss[next_fix++];
      filter.Predict(imu[i - 1], fix.t - time);
      filter.Update(fix);
      time = fix.t;
    }
    if (time < step_end) {
      filter.Predict(imu[i - 1], step_end - time);
      time = step_end;
    }
    estimates.push_back(filter.Estimate(time, next_fix != first_fix_on_step));
  }
  return estimates;
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_EKF_EKF_H_
