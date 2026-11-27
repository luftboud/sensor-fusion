#ifndef SENSOR_FUSION_EKF_EKF_DYNAMIC_H_
#define SENSOR_FUSION_EKF_EKF_DYNAMIC_H_

#include <cstddef>
#include <span>
#include <vector>

#include "matrix/matrix.h"
#include "sensor_fusion/ekf/ekf.h"
#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/ekf/state.h"

namespace sensor_fusion {

// Configuration for the dynamic-size filter. state_size is the dimension of
// the state. The 2D model has four states, so any other value is rejected
// with std::invalid_argument.
struct DynamicEkfConfig {
  EkfConfig filter = {};
  std::size_t state_size = internal::kPlanarStateSize;
};

namespace internal {

// Runtime-sized storage for PlanarEkfCore. Each matrix is allocated when the
// filter step needs it, so a step allocates on the heap.
struct DynamicMatrixPolicy {
  using Square = Matrix<double, 0, 0>;
  using Tall = Matrix<double, 0, 0>;

  static Square MakeSquare(std::size_t n) { return Square(n, n); }
  static Tall MakeTall(std::size_t n) { return Tall(n, kPlanarNoiseInputs); }
  static Matrix<double, 0, 0> MakeScalar() { return Matrix<double, 0, 0>(1, 1); }
  static Matrix<double, 0, 0> MakeRow(std::size_t n) {
    return Matrix<double, 0, 0>(1, n);
  }
  static Matrix<double, 0, 0> MakeColumn(std::size_t n) {
    return Matrix<double, 0, 0>(n, 1);
  }
  static Square Identity(std::size_t n) { return identity<double>(n); }
};

// Dynamic-size filter. It runs the same algorithm as PlanarEkf, on matrices
// whose sizes are set by the configuration at construction time.
class PlanarEkfDynamic : public PlanarEkfCore<DynamicMatrixPolicy> {
 public:
  using Base = PlanarEkfCore<DynamicMatrixPolicy>;
  explicit PlanarEkfDynamic(const DynamicEkfConfig& config)
      : Base(config.filter, config.state_size) {}
};

}  // namespace internal

// Runs the dynamic-size filter with the same input and output as RunEkf. The
// estimates are equal to RunEkf's up to rounding. The validation rules are
// the same. Throws std::invalid_argument if config.state_size is not 4.
inline std::vector<EkfEstimate> RunEkfDynamic(
    std::span<const PlanarImu> imu, std::span<const GnssFix> gnss,
    const DynamicEkfConfig& config = {}) {
  internal::PlanarEkfDynamic filter(config);
  return internal::RunSeries(filter, imu, gnss);
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_EKF_EKF_DYNAMIC_H_
