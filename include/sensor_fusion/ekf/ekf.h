#ifndef SENSOR_FUSION_EKF_EKF_H_
#define SENSOR_FUSION_EKF_EKF_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
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
  // Metres per axis. Its square must be positive and finite in double.
  double gnss_position_std = 2.0;
};

namespace internal {

class PlanarEkf {
 public:
  explicit PlanarEkf(const EkfConfig& config)
      : state_(config.initial_state),
        accel_noise_density_(config.accel_noise_density),
        gyro_noise_density_(config.gyro_noise_density),
        gnss_position_std_(config.gnss_position_std) {
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
    if (gnss_position_std_ * gnss_position_std_ == 0.0 ||
        !std::isfinite(state_.east) ||
        !std::isfinite(state_.north) || !std::isfinite(state_.yaw) ||
        !std::isfinite(state_.v)) {
      throw std::invalid_argument(
          "EKF requires a finite initial state and positive GNSS variance");
    }

    covariance_root_(kEast, kEast) = config.initial_position_std;
    covariance_root_(kNorth, kNorth) = config.initial_position_std;
    covariance_root_(kYaw, kYaw) = config.initial_yaw_std;
    covariance_root_(kSpeed, kSpeed) = config.initial_speed_std;
  }

  void Predict(const PlanarImu& imu, double dt) {
    if (!std::isfinite(dt)) {
      throw std::overflow_error("EKF IMU time interval overflowed");
    }
    const double delta_speed = imu.ax * dt;
    const double delta_yaw = imu.wz * dt;
    const double yaw_mid = state_.yaw + 0.5 * delta_yaw;
    const double distance = (state_.v + 0.5 * delta_speed) * dt;
    if (!std::isfinite(yaw_mid) || !std::isfinite(distance)) {
      throw std::overflow_error("EKF motion prediction overflowed");
    }
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
    const double root_dt = std::sqrt(dt);
    const double speed_std = accel_noise_density_ * root_dt;
    const double yaw_std = gyro_noise_density_ * root_dt;
    const auto propagated_root = transition * covariance_root_;
    Matrix<double, kStateSize, kStateSize + 2> prediction;
    for (std::size_t row = 0; row < kStateSize; ++row) {
      for (std::size_t col = 0; col < kStateSize; ++col) {
        prediction(row, col) = propagated_root(row, col);
      }
      prediction(row, kStateSize) = noise_jacobian(row, 0) * speed_std;
      prediction(row, kStateSize + 1) = noise_jacobian(row, 1) * yaw_std;
    }
    // Rotate columns of [F L | G sqrt(Q)] to keep a square factor of P.
    for (std::size_t row = 0; row < kStateSize; ++row) {
      ReduceRow(prediction, row, row);
      for (std::size_t col = 0; col <= row; ++col) {
        covariance_root_(row, col) = prediction(row, col);
      }
      for (std::size_t col = row + 1; col < kStateSize; ++col) {
        covariance_root_(row, col) = 0.0;
      }
    }

    state_.east += distance * cos_yaw;
    state_.north += distance * sin_yaw;
    state_.yaw += delta_yaw;
    state_.v += delta_speed;
    CheckNumerics();
  }

  void Update(const GnssFix& fix) {
    // Each coordinate is an independent observation with the same noise.
    // Start with the larger variance to keep the first correction well scaled.
    if (StateStd(kEast) >= StateStd(kNorth)) {
      CorrectPosition(kEast, fix.east - state_.east);
      CorrectPosition(kNorth, fix.north - state_.north);
    } else {
      CorrectPosition(kNorth, fix.north - state_.north);
      CorrectPosition(kEast, fix.east - state_.east);
    }
    CheckNumerics();
  }

  EkfEstimate Estimate(double t, bool updated) const {
    return {
        .t = t,
        .state = state_,
        .std_east = StateStd(kEast),
        .std_north = StateStd(kNorth),
        .updated = updated,
    };
  }

 private:
  static constexpr std::size_t kEast = 0;
  static constexpr std::size_t kNorth = 1;
  static constexpr std::size_t kYaw = 2;
  static constexpr std::size_t kSpeed = 3;
  static constexpr std::size_t kStateSize = 4;

  // Apply Givens rotations on the right to zero the entries after pivot_col.
  // An orthogonal rotation preserves the factor times its transpose.
  template <std::size_t Rows, std::size_t Cols>
  static void ReduceRow(Matrix<double, Rows, Cols>& factor, std::size_t row,
                        std::size_t pivot_col) {
    for (std::size_t col = pivot_col + 1; col < Cols; ++col) {
      const double entry = factor(row, col);
      if (entry == 0.0) {
        continue;
      }
      const double pivot = factor(row, pivot_col);
      // Scale before computing the rotation so subnormal pivots still yield
      // normalized coefficients.
      const double scale = std::max(std::abs(pivot), std::abs(entry));
      if (!std::isfinite(scale) || !std::isfinite(entry)) {
        throw std::overflow_error("EKF covariance became non-finite");
      }
      const double x = pivot / scale;
      const double y = entry / scale;
      const double norm = std::hypot(x, y);
      const double cosine = x / norm;
      const double sine = y / norm;
      factor(row, pivot_col) = scale * norm;
      factor(row, col) = 0.0;
      for (std::size_t next = row + 1; next < Rows; ++next) {
        const double left = factor(next, pivot_col);
        const double right = factor(next, col);
        factor(next, pivot_col) = cosine * left + sine * right;
        factor(next, col) = cosine * right - sine * left;
      }
    }
  }

  void CorrectPosition(std::size_t axis, double residual) {
    if (!std::isfinite(residual)) {
      throw std::overflow_error("EKF GNSS residual overflowed");
    }
    // The first row describes uncertainty in the measured coordinate. After
    // rotation, its first entry is the innovation standard deviation. The
    // lower rows contain the scaled gain and the posterior covariance factor.
    SquareMatrix<double, kStateSize + 1> correction;
    correction(0, 0) = gnss_position_std_;
    for (std::size_t col = 0; col < kStateSize; ++col) {
      correction(0, col + 1) = covariance_root_(axis, col);
      for (std::size_t row = 0; row < kStateSize; ++row) {
        correction(row + 1, col + 1) = covariance_root_(row, col);
      }
    }
    ReduceRow(correction, 0, 0);
    const double innovation_std = correction(0, 0);
    if (!std::isfinite(innovation_std)) {
      throw std::overflow_error("EKF GNSS innovation overflowed");
    }
    const std::array components{&state_.east, &state_.north, &state_.yaw,
                                &state_.v};
    for (std::size_t row = 0; row < kStateSize; ++row) {
      const double scaled_gain = correction(row + 1, 0);
      const double gain = scaled_gain / innovation_std;
      if (!std::isfinite(gain) || (gain == 0.0 && scaled_gain != 0.0)) {
        // The correction can be representable even if the gain is outside
        // the range of double. Scale the residual before multiplying.
        *components[row] =
            std::fma(scaled_gain, residual / innovation_std, *components[row]);
      } else {
        *components[row] = std::fma(gain, residual, *components[row]);
      }
      for (std::size_t col = 0; col < kStateSize; ++col) {
        covariance_root_(row, col) = correction(row + 1, col + 1);
      }
    }
  }

  double StateStd(std::size_t row) const {
    return std::hypot(std::hypot(covariance_root_(row, 0),
                                covariance_root_(row, 1)),
                      std::hypot(covariance_root_(row, 2),
                                 covariance_root_(row, 3)));
  }

  void CheckNumerics() const {
    if (!std::isfinite(state_.east) || !std::isfinite(state_.north) ||
        !std::isfinite(state_.yaw) || !std::isfinite(state_.v)) {
      throw std::overflow_error("EKF state became non-finite");
    }
    const double max_std = std::sqrt(std::numeric_limits<double>::max());
    for (std::size_t row = 0; row < kStateSize; ++row) {
      if (!(StateStd(row) <= max_std)) {
        throw std::overflow_error("EKF covariance variance overflowed");
      }
    }
  }

  State2d state_;
  double accel_noise_density_;
  double gyro_noise_density_;
  double gnss_position_std_;
  // P = L L^T. Keeping L avoids cancellation in covariance updates when
  // a very precise fix observes a direction with an almost singular prior.
  SquareMatrix<double, kStateSize> covariance_root_;
};

}  // namespace internal

// Returns one estimate per IMU timestamp. The initial state is at imu[0].t.
// Each fix in the IMU time range is applied at its own timestamp and marks
// the first estimate at or after that timestamp as updated. Fixes outside
// the IMU time range are ignored. Empty GNSS gives an IMU-only trajectory.
// Both streams must contain finite values and have strictly increasing times.
// Throws std::invalid_argument for invalid inputs or configuration.
// Throws std::runtime_error (including std::overflow_error) if the state or
// covariance cannot be computed at working precision.
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
