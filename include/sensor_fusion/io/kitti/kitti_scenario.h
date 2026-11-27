#ifndef SENSOR_FUSION_IO_KITTI_KITTI_SCENARIO_H_
#define SENSOR_FUSION_IO_KITTI_KITTI_SCENARIO_H_

#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/io/imu/imu.h"

namespace sensor_fusion {

// Turns a KITTI oxts drive into the IMU steps and the reference of a
// Scenario, the same way load_scenario() in foreign_impl/ref_ekf.py does, so
// that the C++ filter and the filterpy reference start from the same numbers.
//
// Each step follows the Python expression, operation by operation, so the
// results match numpy in every bit where the math library does too. See
// docs/kitti-scenario.md.

// Equatorial radius of WGS-84 (m), the radius ref_ekf.py uses.
inline constexpr double kEarthRadiusM = 6378137.0;

// One planar IMU step from an OXTS record: the body-frame pair ax and wz, see
// PlanarImu and docs/kitti-oxts-survey.md.
inline PlanarImu ToPlanarImu(double t, const OxtsRecord& record) {
  return {.t = t, .ax = record.ax, .wz = record.wz};
}

namespace internal {

// numpy.radians() multiplies by pi / 180 as one constant. degrees * pi / 180
// rounds twice and can differ in the last bit.
inline double Radians(double degrees) {
  constexpr double kRadiansPerDegree = std::numbers::pi / 180.0;
  return degrees * kRadiansPerDegree;
}

// Python's a % b for b > 0, as numpy.mod() computes it: the result has the
// sign of b.
inline double PositiveMod(double a, double b) {
  double mod = std::fmod(a, b);
  if (mod == 0.0) {
    return std::copysign(0.0, b);
  }
  if ((mod < 0.0) != (b < 0.0)) {
    mod += b;
  }
  return mod;
}

// numpy.unwrap() with the default period 2 pi: removes the jumps of more than
// pi between neighbours by adding whole turns. A jump of exactly pi stays.
inline std::vector<double> Unwrap(const std::vector<double>& angles) {
  constexpr double kPeriod = 2.0 * std::numbers::pi;
  constexpr double kHalfPeriod = kPeriod / 2.0;
  std::vector<double> unwrapped = angles;
  double correction = 0.0;  // numpy's cumulative sum of ph_correct.
  for (std::size_t i = 1; i < angles.size(); ++i) {
    const double jump = angles[i] - angles[i - 1];
    double wrapped = PositiveMod(jump + kHalfPeriod, kPeriod) - kHalfPeriod;
    if (wrapped == -kHalfPeriod && jump > 0.0) {
      wrapped = kHalfPeriod;
    }
    const double step = std::abs(jump) < kHalfPeriod ? 0.0 : wrapped - jump;
    correction += step;
    unwrapped[i] = angles[i] + correction;
  }
  return unwrapped;
}

}  // namespace internal

// Builds imu and reference from a drive that ReadKittiOxts() read. gnss
// stays empty: the fixes come from a separate file, see gnss_csv.h.
//
// - imu[i] is ToPlanarImu() of record i at samples[i].t.
// - reference[i].state.east and north are metres from record 0 on a local
//   flat earth: east = R * cos(lat0) * (lon - lon0), north = R * (lat - lat0),
//   with the angles in radians and R = kEarthRadiusM.
// - reference[i].state.yaw is the OXTS yaw, unwrapped like numpy.unwrap(), so
//   it has no 2 pi jumps.
// - reference[i].state.v is vf, the forward speed along the ground.
//
// The result passes IsWellFormed() for a drive that read without error. An
// empty drive gives an empty Scenario.
inline Scenario KittiScenario(const KittiReadResult& drive) {
  Scenario scenario;
  const std::size_t count =
      drive.records.size() < drive.samples.size() ? drive.records.size()
                                                  : drive.samples.size();
  if (count == 0) {
    return scenario;
  }

  // The angles in radians come first, as whole arrays the way numpy makes
  // them. Each one is rounded and stored before it is subtracted, so a
  // compiler that fuses a multiply and an add (gcc at -O2 does) cannot turn
  // Radians(lon) - lon0 into one fused operation. That would round once
  // instead of twice, give record 0 a position of 1e-11 m instead of 0 and
  // stop matching numpy.
  std::vector<double> yaws;
  std::vector<double> lats;
  std::vector<double> lons;
  yaws.reserve(count);
  lats.reserve(count);
  lons.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    yaws.push_back(drive.records[i].yaw);
    lats.push_back(internal::Radians(drive.records[i].lat));
    lons.push_back(internal::Radians(drive.records[i].lon));
  }
  const std::vector<double> unwrapped = internal::Unwrap(yaws);

  const double lat0 = lats[0];
  const double lon0 = lons[0];
  // ref_ekf.py: EARTH_RADIUS_M * np.cos(lat[0]) * (lon - lon[0]).
  const double east_scale = kEarthRadiusM * std::cos(lat0);

  scenario.imu.reserve(count);
  scenario.reference.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const OxtsRecord& record = drive.records[i];
    const double t = drive.samples[i].t;
    scenario.imu.push_back(ToPlanarImu(t, record));
    scenario.reference.push_back({
        .t = t,
        .state =
            {
                .east = east_scale * (lons[i] - lon0),
                .north = kEarthRadiusM * (lats[i] - lat0),
                .yaw = unwrapped[i],
                .v = record.vf,
            },
    });
  }
  return scenario;
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_IO_KITTI_KITTI_SCENARIO_H_
