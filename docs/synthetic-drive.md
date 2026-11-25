# Synthetic drive

`include/sensor_fusion/sim/synthetic_drive.h` makes up a drive and fills a `Scenario` with it. The true path is known exactly, so the error of the filter can be measured against it. The records and their rules are in [2D scenario types](ekf-scenario.md).

```cpp
const std::optional<Scenario> drive =
    GenerateScenario(DemoDrive(), SyntheticConfig{});
```

## Segments

A drive is a list of `DriveSegment`. The acceleration and the yaw rate are constant inside a segment.

| Field | Meaning |
| --- | --- |
| `duration` | Seconds, a whole number of IMU steps |
| `accel` | Forward acceleration, m/s² |
| `yaw_rate` | Rad/s. Positive turns left |

The generator does not check that the motion is realistic. A segment may turn the car at zero speed.

## Settings

`SyntheticConfig` holds the timing, the sensor errors, and the seed.

| Field | Default | Meaning |
| --- | --- | --- |
| `imu_dt` | 0.01 | Seconds between IMU steps |
| `start_yaw`, `start_speed` | 0, 0 | Heading (rad) and speed (m/s) at `t = 0` |
| `gnss_period` | 1.0 | Seconds between fixes |
| `gnss_offset` | 0.5 | Time of the first fix, s |
| `outage_start` | 25.0 | Start of the outage, s |
| `outage_duration` | 15.0 | Length of the outage, s. 0 turns it off |
| `accel_noise`, `accel_bias` | 0.05, 0.02 | m/s² |
| `gyro_noise`, `gyro_bias` | 0.005, 0.002 | Rad/s |
| `gnss_noise` | 2.0 | Metres, on east and on north |
| `seed` | 1 | Seed of the noise |

A noise value is the standard deviation of one sample. A bias is the same for every sample of the sensor. The noise values do not scale with `imu_dt`.

## The true path

The car starts at `(0, 0)` with `start_yaw` and `start_speed`. One step uses the midpoint rule:

```text
v_mid   = v   + accel * dt / 2
yaw_mid = yaw + yaw_rate * dt / 2
east   += v_mid * cos(yaw_mid) * dt
north  += v_mid * sin(yaw_mid) * dt
yaw    += yaw_rate * dt
v      += accel * dt
```

Apart from rounding, the rule is exact on a straight line at constant acceleration. At constant speed on a turn, the step is longer than the true chord by about `(yaw_rate * dt)² / 24` of its length. `yaw` is not wrapped to a range.

`N` steps give `N + 1` samples in `imu` and in `reference`. Time is `i * dt`, so it does not collect rounding.

## IMU

`imu[i]` is the segment input at `t[i]` plus the bias and the noise. It moves the state from `t[i]` to `t[i+1]`. When the segment changes at `t[i]`, the reading belongs to the new segment. The last sample has no next step and repeats the last segment.

The road is flat, so `ax` carries no gravity. Noise never changes `reference`.

## GNSS and the outage

Fixes fall at `gnss_offset + k * gnss_period`, up to the end of the drive. The position at a fix is the true one: the generator takes the last IMU step at or before the fix and moves it by the remaining time. The noise is added on top.

With the default settings every fix of the demo drive lands on an IMU step. Set `gnss_offset` to 0.505 to put the fixes between the steps.

Fixes with `outage_start <= t < outage_start + outage_duration` are left out. They still draw their noise, so the other fixes read the same with and without the outage.

## Noise

Normal numbers come from `std::mt19937_64` with the Box-Muller transform. The algorithm of `std::normal_distribution` depends on the standard library, so it is not used. `std::log`, `std::sin`, and `std::cos` can still differ in the last bit between platforms, so tests do not compare noisy values with fixed numbers.

The IMU and the GNSS have separate engines. Changing the GNSS settings leaves the IMU readings as they were.

## When there is no result

`GenerateScenario` throws no exceptions of its own. It returns `std::nullopt` when:

- a value is NaN or infinite;
- `imu_dt` or `gnss_period` is not positive;
- a duration, a noise value, or `gnss_offset` is negative;
- a segment is not a whole number of IMU steps;
- the drive has more than `kMaxDriveSteps` (1 000 000) steps or fixes;
- the drive would not pass `IsWellFormed()`, for example when `gnss_period` is so small that two fixes get the same time.

A returned scenario always passes `IsWellFormed()`.

## Demo drive

`DemoDrive()` lasts 65 seconds and ends near `(327, 277)`.

| Time, s | Motion |
| --- | --- |
| 0–10 | Speeds up from rest to 10 m/s |
| 10–20 | Straight east |
| 20–30 | Quarter turn left, radius about 64 m |
| 30–45 | Straight north |
| 45–55 | Quarter turn right |
| 55–65 | Brakes to a stop, heading east |

The default outage, 25–40 s, covers the second half of the left turn and the straight after it. It leaves 50 of the 65 fixes.
