#include <iostream>

#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/ekf/state.h"

namespace {

using sensor_fusion::EkfEstimate;
using sensor_fusion::GnssFix;
using sensor_fusion::PlanarImu;
using sensor_fusion::ReferenceSample;
using sensor_fusion::Scenario;
using sensor_fusion::State2d;

int failure_count = 0;

void Check(bool ok, const char* text, int line) {
  if (!ok) {
    std::cerr << "line " << line << ": " << text << '\n';
    ++failure_count;
  }
}

#define CHECK(condition) \
  Check(static_cast<bool>(condition), #condition, __LINE__)

// `const T value;` compiles only when every member of T has an initializer.
void TestDefaultsAreZero() {
  const State2d state;
  CHECK(state.east == 0.0 && state.north == 0.0);
  CHECK(state.yaw == 0.0 && state.v == 0.0);

  const EkfEstimate estimate;
  CHECK(estimate.t == 0.0);
  CHECK(estimate.state.east == 0.0 && estimate.state.north == 0.0);
  CHECK(estimate.state.yaw == 0.0 && estimate.state.v == 0.0);
  CHECK(estimate.std_east == 0.0 && estimate.std_north == 0.0);
  CHECK(estimate.updated == false);

  const PlanarImu imu;
  CHECK(imu.t == 0.0 && imu.ax == 0.0 && imu.wz == 0.0);

  const GnssFix fix;
  CHECK(fix.t == 0.0 && fix.east == 0.0 && fix.north == 0.0);

  const ReferenceSample reference;
  CHECK(reference.t == 0.0);
  CHECK(reference.state.east == 0.0 && reference.state.north == 0.0);
  CHECK(reference.state.yaw == 0.0 && reference.state.v == 0.0);

  const Scenario scenario;
  CHECK(scenario.imu.empty() && scenario.gnss.empty() &&
        scenario.reference.empty());
}

// A GNSS time between two IMU steps is stored as given. The log does not
// snap it onto an IMU timestamp.
void TestGnssTimeStaysBetweenImuSteps() {
  const PlanarImu first{.t = 0.0, .ax = 0.2, .wz = 0.0};
  const PlanarImu second{.t = 0.2, .ax = 0.2, .wz = 0.01};
  const GnssFix fix{.t = 0.1, .east = 1.5, .north = -0.25};
  const ReferenceSample origin{
      .t = 0.0,
      .state = {.east = 0.0, .north = 0.0, .yaw = 0.4, .v = 2.0},
  };
  const ReferenceSample later{
      .t = 0.2,
      .state = {.east = 0.37, .north = 0.15, .yaw = 0.4, .v = 2.0},
  };
  const EkfEstimate estimate{
      .t = 0.2,
      .state = {.east = 0.4, .north = 0.1, .yaw = 0.4, .v = 2.0},
      .std_east = 1.2,
      .std_north = 0.8,
      .updated = true,
  };
  const State2d state{.east = 3.0, .north = 4.0, .yaw = 0.5, .v = -1.0};

  Scenario scenario;
  scenario.imu = {first, second};
  scenario.gnss = {fix};
  scenario.reference = {origin, later};

  CHECK(scenario.imu.size() == scenario.reference.size());
  CHECK(scenario.gnss.size() == 1);
  CHECK(scenario.gnss[0].t > scenario.imu[0].t);
  CHECK(scenario.gnss[0].t < scenario.imu[1].t);
  CHECK(scenario.gnss[0].east == 1.5 && scenario.gnss[0].north == -0.25);
  CHECK(scenario.reference[0].state.east == 0.0);
  CHECK(scenario.reference[0].state.north == 0.0);
  CHECK(scenario.reference[0].state.yaw == 0.4);
  CHECK(scenario.reference[0].state.v == 2.0);
  CHECK(estimate.updated);
  CHECK(estimate.std_east == 1.2 && estimate.std_north == 0.8);
  CHECK(estimate.state.east == 0.4);
  CHECK(state.v == -1.0);
}

}  // namespace

int main() {
  TestDefaultsAreZero();
  TestGnssTimeStaysBetweenImuSteps();

  if (failure_count != 0) {
    std::cerr << failure_count << " checks failed\n";
    return 1;
  }

  std::cout << "All tests passed\n";
  return 0;
}
