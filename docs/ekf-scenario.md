# 2D scenario types

The 2D filter, the synthetic drive, and the comparison all share one set of records. The records are `State2d` and `EkfEstimate` in `include/sensor_fusion/ekf/state.h`, and `PlanarImu`, `GnssFix`, `ReferenceSample`, and `Scenario` in `include/sensor_fusion/ekf/scenario.h`.

This note is the contract those headers spell out. The predict/update step is a later task. So is the generator that fills a `Scenario`, and so is the 15-second cut in the GNSS series.

## State

`State2d` is the planar state. Time is not a member.

| Field | Meaning |
| --- | --- |
| `east`, `north` | Metres from the first reference sample of the drive |
| `yaw` | Heading in radians. 0 is east, and positive is counter-clockwise |
| `v` | Signed forward speed, m/s. Reverse is negative |

With that heading, the position rates are `v * cos(yaw)` to the east and `v * sin(yaw)` to the north. `yaw` uses the same zero as KITTI `OxtsRecord::yaw`.

## What the filter reads

`PlanarImu` is one propagation step: `t`, `ax`, `wz`.

`ax` and `wz` are body-frame axes, the pair the [KITTI survey](kitti-oxts-survey.md#two-axis-frames) keeps for the flat model. `ax` is specific force along the car's forward axis, so pitch leaves a piece of gravity in it. `wz` is the yaw rate about the car's up axis; positive turns left. The levelled OXTS pair `af` and `wu` is a different reading.

`GnssFix` is one satellite fix: `t`, `east`, `north`. The coordinates use the same origin as `State2d`. A fix that is later than the start of the drive is not at `(0, 0)`.

## Clock and origin

Every `t` is seconds on one drive clock. Zero is the first reference sample.

GNSS time is free on that clock. A fix may fall between two IMU steps, and it does not have to share a timestamp with a `PlanarImu`. `Scenario` does not choose which IMU step consumes a fix. The filter does, and reports the choice in `EkfEstimate::updated`.

A stretch of the drive with no `GnssFix` samples is an outage. The outage is the absence of those samples.

## What the filter leaves aside

`ReferenceSample` is `t` plus a `State2d`. It is the ground truth at one IMU step, and the comparison reads it. A filter step reads `PlanarImu` and, when it accepts a fix, `GnssFix`.

## What the filter writes

`EkfEstimate` is one output row per IMU step.

| Field | Meaning |
| --- | --- |
| `t`, `state` | Time and the estimated `State2d` |
| `std_east`, `std_north` | Metres, square roots of the east and north covariance diagonals |
| `updated` | True when the filter consumed a GNSS fix on this IMU step |

Position is the only part of the state with a standard deviation here. Several fixes consumed on one step still leave `updated` true.

The estimates are a sequence beside the log, one entry per IMU step. `Scenario` has no estimate member.

## Scenario

`Scenario` holds three series: `imu`, `gnss`, and `reference`. An empty scenario is a valid value. The headers do not check the shape of a filled drive. A filled drive has this shape:

- `reference` and `imu` have the same length, and `reference[i].t == imu[i].t`.
- Time in each series is strictly increasing.
- `reference.front()` has `t == 0`, `east == 0`, and `north == 0`. `yaw` and `v` there are the drive's own values.
- `gnss` uses the same clock. Its timestamps need not equal an IMU timestamp.
