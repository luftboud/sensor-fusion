#include <cmath>
#include <cstddef>
#include <iostream>
#include <numbers>
#include <vector>

#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/io/imu/imu.h"
#include "sensor_fusion/io/kitti/kitti_scenario.h"

namespace {

using sensor_fusion::ImuSample;
using sensor_fusion::IsWellFormed;
using sensor_fusion::kEarthRadiusM;
using sensor_fusion::KittiReadResult;
using sensor_fusion::KittiScenario;
using sensor_fusion::OxtsRecord;
using sensor_fusion::PlanarImu;
using sensor_fusion::Scenario;
using sensor_fusion::ToPlanarImu;
using sensor_fusion::internal::PositiveMod;
using sensor_fusion::internal::Radians;
using sensor_fusion::internal::Unwrap;

int failure_count = 0;

void Check(bool ok, const char* text, int line) {
  if (!ok) {
    std::cerr << "line " << line << ": " << text << '\n';
    ++failure_count;
  }
}

#define CHECK(condition) \
  Check(static_cast<bool>(condition), #condition, __LINE__)

constexpr double kPi = std::numbers::pi;

bool Near(double actual, double expected, double tolerance = 1e-9) {
  return std::abs(actual - expected) <= tolerance;
}

bool Same(const std::vector<double>& a, const std::vector<double>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

// The expected values below were printed by numpy 2.2.6 with float.hex().

void TestRadians() {
  CHECK(Radians(180.0) == kPi);
  CHECK(Radians(-90.0) == -kPi / 2.0);
  CHECK(Radians(0.0) == 0.0);
  // The first latitude of drive 0009. numpy.radians() gives this double;
  // 49.009347760599 * pi / 180 gives a different one.
  CHECK(Radians(49.009347760599) == 0x1.b5f3a4e9dd47ep-1);
  CHECK(Radians(8.4371344308375) == 0x1.2d946f96d17adp-3);
}

// numpy.mod(): the result has the sign of the divisor, and 0 is +0.
void TestPositiveMod() {
  const double period = 2.0 * kPi;
  CHECK(PositiveMod(-1.0, period) == 0x1.521fb54442d18p+2);
  CHECK(PositiveMod(1.0, period) == 1.0);
  CHECK(PositiveMod(7.0, period) == 0x1.6f0255dde9740p-1);
  for (const double zero_result : {0.0, -0.0, -period}) {
    const double mod = PositiveMod(zero_result, period);
    CHECK(mod == 0.0 && !std::signbit(mod));
  }
}

void TestUnwrap() {
  CHECK(Unwrap({}).empty());
  CHECK(Same(Unwrap({-3.0}), {-3.0}));

  // Jumps across +-pi get a whole turn.
  CHECK(Same(Unwrap({3.0, -3.0, 3.1, -3.1, 0.0}),
             {0x1.8p+1, 0x1.a43f6a8885a30p+1, 0x1.8cccccccccccdp+1,
              0x1.97729dbbb8d63p+1, 0x1.921fb54442d18p+2}));

  // A jump of exactly pi, up or down, stays.
  CHECK(Same(Unwrap({0.0, kPi, 0.0, -kPi, 0.0}), {0.0, kPi, 0.0, -kPi, 0.0}));

  // Jumps below pi stay; the corrections add up over several turns.
  CHECK(Same(Unwrap({0.0, 2.0, 4.0, 6.0, 1.7, -2.0, -4.5}),
             {0.0, 2.0, 4.0, 6.0, 0x1.feec82110f9e5p+2, 0x1.521fb54442d18p+3,
              0x1.021fb54442d18p+3}));
}

// A small drive: three records 0.1 s apart, moving north-east and turning
// across the +-pi edge of yaw.
KittiReadResult SmallDrive() {
  KittiReadResult drive;
  const std::vector<double> lats = {49.0, 49.00001, 49.00002};
  const std::vector<double> lons = {8.4, 8.40001, 8.40003};
  const std::vector<double> yaws = {3.1, -3.1, -3.0};
  for (std::size_t i = 0; i < lats.size(); ++i) {
    OxtsRecord record;
    record.lat = lats[i];
    record.lon = lons[i];
    record.yaw = yaws[i];
    record.vf = 10.0 + static_cast<double>(i);
    record.ax = 0.5 * static_cast<double>(i);
    record.wz = -0.1 * static_cast<double>(i);
    record.az = 9.81;
    drive.records.push_back(record);
    drive.samples.push_back(
        ImuSample{.t = 0.1 * static_cast<double>(i), .accel = {}, .gyro = {}});
  }
  return drive;
}

void TestToPlanarImu() {
  OxtsRecord record;
  record.ax = 1.5;
  record.af = 99.0;  // The levelled values are not used.
  record.wz = -0.25;
  record.wu = 99.0;
  const PlanarImu imu = ToPlanarImu(2.0, record);
  CHECK(imu.t == 2.0 && imu.ax == 1.5 && imu.wz == -0.25);
}

void TestScenarioFromDrive() {
  const Scenario scenario = KittiScenario(SmallDrive());
  CHECK(IsWellFormed(scenario));
  CHECK(scenario.gnss.empty());
  CHECK(scenario.imu.size() == 3);
  CHECK(scenario.reference.size() == 3);
  if (scenario.reference.size() != 3) {
    return;
  }

  // Record 0 is the origin.
  CHECK(scenario.reference[0].t == 0.0);
  CHECK(scenario.reference[0].state.east == 0.0);
  CHECK(scenario.reference[0].state.north == 0.0);
  CHECK(scenario.reference[0].state.yaw == 3.1);
  CHECK(scenario.reference[0].state.v == 10.0);

  // 1e-5 degrees of latitude is R * pi / 180 * 1e-5, about 1.113 m. A degree
  // of longitude is shorter by cos(lat0).
  const double metres_per_degree = kEarthRadiusM * kPi / 180.0;
  const double cos_lat0 = std::cos(Radians(49.0));
  CHECK(Near(scenario.reference[1].state.north, 1e-5 * metres_per_degree,
             1e-6));
  CHECK(Near(scenario.reference[2].state.north, 2e-5 * metres_per_degree,
             1e-6));
  CHECK(Near(scenario.reference[1].state.east,
             1e-5 * metres_per_degree * cos_lat0, 1e-6));
  CHECK(Near(scenario.reference[2].state.east,
             3e-5 * metres_per_degree * cos_lat0, 1e-6));

  // The same expressions as ref_ekf.py, bit for bit.
  const double east_scale = kEarthRadiusM * std::cos(Radians(49.0));
  CHECK(scenario.reference[2].state.east ==
        east_scale * (Radians(8.40003) - Radians(8.4)));
  CHECK(scenario.reference[2].state.north ==
        kEarthRadiusM * (Radians(49.00002) - Radians(49.0)));

  // Yaw is unwrapped: -3.1 after 3.1 is the same heading, one turn up.
  CHECK(Near(scenario.reference[1].state.yaw,
             (2.0 * kPi) - 3.1));
  CHECK(Near(scenario.reference[2].state.yaw,
             (2.0 * kPi) - 3.0));

  for (std::size_t i = 0; i < 3; ++i) {
    const double t = 0.1 * static_cast<double>(i);
    CHECK(scenario.imu[i].t == t && scenario.reference[i].t == t);
    CHECK(scenario.imu[i].ax == 0.5 * static_cast<double>(i));
    CHECK(scenario.imu[i].wz == -0.1 * static_cast<double>(i));
    CHECK(scenario.reference[i].state.v == 10.0 + static_cast<double>(i));
  }
}

// Records 0, 2, 263 and 446 of drive 0009. On rows 2, 263 and 446,
// R * (lon - lon0) * cos(lat0) rounds to a different east than ref_ekf.py's
// R * cos(lat0) * (lon - lon0), so the order of the products is pinned here.
void TestRealCoordinatesMatchNumpy() {
  const std::vector<double> lats = {49.009347760599, 49.009342262652,
                                    49.00914174419, 49.010013293278};
  const std::vector<double> lons = {8.4371344308375, 8.4371640536639,
                                    8.439842890797, 8.4404680725925};
  KittiReadResult drive;
  for (std::size_t i = 0; i < lats.size(); ++i) {
    OxtsRecord record;
    record.lat = lats[i];
    record.lon = lons[i];
    drive.records.push_back(record);
    drive.samples.push_back(ImuSample{
        .t = static_cast<double>(i), .accel = {}, .gyro = {}});
  }
  const Scenario scenario = KittiScenario(drive);
  CHECK(scenario.reference.size() == 4);
  if (scenario.reference.size() != 4) {
    return;
  }
  const std::vector<double> east = {0.0, 0x1.14dd9ade65fa6p+1,
                                    0x1.8b88fbcef058ep+7,
                                    0x1.e6d5af13ed5b6p+7};
  const std::vector<double> north = {0.0, -0x1.395bd210dcfacp-1,
                                     -0x1.6ef0325344a5fp+4,
                                     0x1.2858d7556a915p+6};
  for (std::size_t i = 0; i < 4; ++i) {
    CHECK(scenario.reference[i].state.east == east[i]);
    CHECK(scenario.reference[i].state.north == north[i]);
  }
}

void TestEmptyAndUnevenDrives() {
  CHECK(IsWellFormed(KittiScenario(KittiReadResult{})));
  CHECK(KittiScenario(KittiReadResult{}).imu.empty());

  // A failed read has no records; a hand-made result with fewer records
  // than samples uses the records it has.
  KittiReadResult uneven = SmallDrive();
  uneven.records.pop_back();
  const Scenario scenario = KittiScenario(uneven);
  CHECK(scenario.imu.size() == 2 && scenario.reference.size() == 2);
  CHECK(IsWellFormed(scenario));
}

}  // namespace

int main() {
  TestRadians();
  TestPositiveMod();
  TestUnwrap();
  TestToPlanarImu();
  TestScenarioFromDrive();
  TestRealCoordinatesMatchNumpy();
  TestEmptyAndUnevenDrives();

  if (failure_count != 0) {
    std::cerr << failure_count << " checks failed\n";
    return 1;
  }

  std::cout << "All tests passed\n";
  return 0;
}
