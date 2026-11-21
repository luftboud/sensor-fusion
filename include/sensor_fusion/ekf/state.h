#ifndef SENSOR_FUSION_EKF_STATE_H_
#define SENSOR_FUSION_EKF_STATE_H_

namespace sensor_fusion {

// Planar state of the car in the local east-north frame of one drive.
//
// east and north are metres from the first reference sample of that drive.
// That sample is the origin, so its east and north are 0. yaw and v there
// are whatever the drive has.
//
// yaw is the heading in radians: 0 means east, and positive is
// counter-clockwise, the same convention as OxtsRecord::yaw. With the signed
// forward speed v, the position rates are
//
//   east rate  = v * cos(yaw)
//   north rate = v * sin(yaw)
//
// A negative v is reverse, and the heading stays continuous through it.
// Time is not a component of the state. A sample that carries a state keeps
// its t beside the state.
struct State2d {
  double east = 0.0;   // Metres east of the drive origin.
  double north = 0.0;  // Metres north of the drive origin.
  // Heading (rad): 0 = east, positive = counter-clockwise.
  double yaw = 0.0;
  double v = 0.0;  // Signed forward speed (m/s). Reverse is negative.
};

// One filter output, written once per IMU step.
//
// state is the estimate at time t. std_east and std_north are metres, the
// square roots of the east and north diagonal entries of the filter
// covariance. This record has a standard deviation for position only.
//
// updated is the filter's own report that it consumed a GNSS fix on this
// IMU step. Scenario does not choose which step a fix belongs to. Several
// fixes consumed on one step still leave updated true.
struct EkfEstimate {
  double t = 0.0;  // Seconds since the first reference sample.
  State2d state = {};
  double std_east = 0.0;   // Metres, sqrt of the east covariance diagonal.
  double std_north = 0.0;  // Metres, sqrt of the north covariance diagonal.
  // The filter consumed a GNSS fix on this IMU step.
  bool updated = false;
};

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_EKF_STATE_H_
