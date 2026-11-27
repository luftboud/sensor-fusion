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

// State order: east, north, yaw, speed. The 2D model has four states.
inline constexpr std::size_t kEast = 0;
inline constexpr std::size_t kNorth = 1;
inline constexpr std::size_t kYaw = 2;
inline constexpr std::size_t kSpeed = 3;
inline constexpr std::size_t kPlanarStateSize = 4;
// Noise inputs in the prediction: speed increment and yaw increment.
inline constexpr std::size_t kPlanarNoiseInputs = 2;

// Matrix storage for the shared filter. Every shape the filter needs comes
// from a policy, so the same algorithm runs on fixed or runtime sizes.
// StaticMatrixPolicy is in this header. DynamicMatrixPolicy is in
// ekf_dynamic.h. Neither policy may mix the two kinds of storage.
struct StaticMatrixPolicy {
  using Square = SquareMatrix<double, kPlanarStateSize>;
  using Tall = Matrix<double, kPlanarStateSize, kPlanarNoiseInputs>;

  // The size arguments are ignored: the sizes above are fixed at compile time.
  static Square MakeSquare(std::size_t /*size*/) { return {}; }
  static Tall MakeTall(std::size_t /*size*/) { return {}; }
  static Matrix<double, 1, 1> MakeScalar() { return {}; }
  static Matrix<double, 1, kPlanarStateSize> MakeRow(std::size_t /*size*/) { return {}; }
  static Matrix<double, kPlanarStateSize, 1> MakeColumn(std::size_t /*size*/) {
    return {};
  }
  static Square Identity(std::size_t /*size*/) {
    return identity<double, kPlanarStateSize>();
  }
};

// Euclidean norm of entries [begin, end) of one row, as a balanced tree of
// hypot calls. For four entries this is hypot(hypot(a, b), hypot(c, d)).
template <typename M>
double RowNorm(const M& factor, std::size_t row, std::size_t begin,
               std::size_t end) {
  if (end - begin == 1) {
    return std::abs(factor(row, begin));
  }
  const std::size_t middle = begin + ((end - begin) / 2);
  return std::hypot(RowNorm(factor, row, begin, middle),
                    RowNorm(factor, row, middle, end));
}

// The planar extended Kalman filter, written once for any matrix policy.
// The covariance is kept as a square-root factor L with P = L L^T.
template <typename Policy>
class PlanarEkfCore {
 public:
  // state_size is the dimension of the covariance factor. The 2D model has
  // kPlanarStateSize states; any other value is rejected before allocation.
  explicit PlanarEkfCore(const EkfConfig& config,
                         std::size_t state_size = kPlanarStateSize)
      : covariance_root_(Policy::MakeSquare(RequireStateSize(state_size))),
        state_(config.initial_state),
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
    const double yaw_mid = state_.yaw + (0.5 * delta_yaw);
    const double distance = (state_.v + (0.5 * delta_speed)) * dt;
    if (!std::isfinite(yaw_mid) || !std::isfinite(distance)) {
      throw std::overflow_error("EKF motion prediction overflowed");
    }
    const double cos_yaw = std::cos(yaw_mid);
    const double sin_yaw = std::sin(yaw_mid);
    const std::size_t n = covariance_root_.rows();

    auto transition = Policy::Identity(n);
    transition(kEast, kYaw) = -distance * sin_yaw;
    transition(kNorth, kYaw) = distance * cos_yaw;
    transition(kEast, kSpeed) = dt * cos_yaw;
    transition(kNorth, kSpeed) = dt * sin_yaw;

    // Differentiate the midpoint step with respect to the noisy increments
    // in speed and yaw. Each increment also affects the midpoint position.
    auto noise_jacobian = Policy::MakeTall(n);
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
    for (std::size_t row = 0; row < n; ++row) {
      noise_jacobian(row, 0) *= speed_std;
      noise_jacobian(row, 1) *= yaw_std;
    }
    // [F L | G sqrt(Q)]: the noise columns are scaled by their standard
    // deviations, then rotated to keep a square factor of P.
    auto prediction = hconcat(propagated_root, noise_jacobian);
    for (std::size_t row = 0; row < n; ++row) {
      givens_reduce_row(prediction, row, row);
      for (std::size_t col = 0; col <= row; ++col) {
        covariance_root_(row, col) = prediction(row, col);
      }
      for (std::size_t col = row + 1; col < n; ++col) {
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

  [[nodiscard]] EkfEstimate Estimate(double t, bool updated) const {
    return {
        .t = t,
        .state = state_,
        .std_east = StateStd(kEast),
        .std_north = StateStd(kNorth),
        .updated = updated,
    };
  }

 private:
  static std::size_t RequireStateSize(std::size_t state_size) {
    if (state_size != kPlanarStateSize) {
      throw std::invalid_argument("EKF state size must be 4 for the 2D model");
    }
    return state_size;
  }

  // axis is kEast or kNorth, a row index, and residual is in metres. Every
  // call passes one of the named constants, so the two cannot be swapped
  // unnoticed.
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  void CorrectPosition(std::size_t axis, double residual) {
    if (!std::isfinite(residual)) {
      throw std::overflow_error("EKF GNSS residual overflowed");
    }
    const std::size_t n = covariance_root_.rows();
    // The first row describes uncertainty in the measured coordinate. After
    // rotation, its first entry is the innovation standard deviation. The
    // lower rows contain the scaled gain and the posterior covariance factor.
    // [[sigma_g, L[axis, :]], [0, L]] as two block rows.
    auto scalar = Policy::MakeScalar();
    scalar(0, 0) = gnss_position_std_;
    auto axis_row = Policy::MakeRow(n);
    for (std::size_t col = 0; col < n; ++col) {
      axis_row(0, col) = covariance_root_(axis, col);
    }
    auto top = hconcat(scalar, axis_row);
    auto bottom = hconcat(Policy::MakeColumn(n), covariance_root_);
    auto correction = vconcat(top, bottom);
    givens_reduce_row(correction, 0, 0);
    const double innovation_std = correction(0, 0);
    if (!std::isfinite(innovation_std)) {
      throw std::overflow_error("EKF GNSS innovation overflowed");
    }
    const std::array components{&state_.east, &state_.north, &state_.yaw,
                                &state_.v};
    for (std::size_t row = 0; row < n; ++row) {
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
      for (std::size_t col = 0; col < n; ++col) {
        covariance_root_(row, col) = correction(row + 1, col + 1);
      }
    }
  }

  [[nodiscard]] double StateStd(std::size_t row) const {
    return RowNorm(covariance_root_, row, 0, covariance_root_.rows());
  }

  void CheckNumerics() const {
    if (!std::isfinite(state_.east) || !std::isfinite(state_.north) ||
        !std::isfinite(state_.yaw) || !std::isfinite(state_.v)) {
      throw std::overflow_error("EKF state became non-finite");
    }
    const double max_std = std::sqrt(std::numeric_limits<double>::max());
    for (std::size_t row = 0; row < covariance_root_.rows(); ++row) {
      if (!(StateStd(row) <= max_std)) {
        throw std::overflow_error("EKF covariance variance overflowed");
      }
    }
  }

  // P = L L^T. Keeping L avoids cancellation in covariance updates when
  // a very precise fix observes a direction with an almost singular prior.
  typename Policy::Square covariance_root_;
  State2d state_;
  double accel_noise_density_;
  double gyro_noise_density_;
  double gnss_position_std_;
};

// Static-size filter: every matrix has its dimensions in the type, so a step
// never allocates.
class PlanarEkf : public PlanarEkfCore<StaticMatrixPolicy> {
 public:
  using Base = PlanarEkfCore<StaticMatrixPolicy>;
  explicit PlanarEkf(const EkfConfig& config) : Base(config) {}
};

// Shared driver for RunEkf and RunEkfDynamic. The caller constructs the
// filter, which validates the configuration. This validates both series,
// then runs the filter over them.
template <typename Filter>
std::vector<EkfEstimate> RunSeries(Filter& filter, std::span<const PlanarImu> imu,
                                   std::span<const GnssFix> gnss) {
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
  return internal::RunSeries(filter, imu, gnss);
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_EKF_EKF_H_
