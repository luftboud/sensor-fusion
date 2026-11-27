# EKF model

The 2D extended Kalman filter fuses GNSS position with the IMU's forward acceleration and yaw rate. This page states the model once. Both implementations, the static `PlanarEkf` and the dynamic `PlanarEkfDynamic`, run this model. The types of the state and the scenario are in [2D scenario types](ekf-scenario.md). The Ukrainian version is [docs/ua/ekf-model.md](ua/ekf-model.md), and its formulas match this page.

## State

The state is a column vector with four entries, in this order:

```text
x = [east, north, yaw, v]^T
```

`east` and `north` are metres from the drive origin. `yaw` is the heading in radians, with 0 pointing east and positive counter-clockwise. `v` is the signed forward speed in m/s. The position rates are `v cos(yaw)` and `v sin(yaw)`. The state is `State2d` in `state.h`.

The covariance is `P`, a 4×4 symmetric matrix. The filter stores a square-root factor `L` with `P = L L^T`, so `P` is never formed explicitly. The matrix `L` is the one the code keeps.

## Inputs and noise

One IMU sample `(ax, wz)` is held over `[t_k, t_{k+1})`, and the step length is `dt = t_{k+1} - t_k`:

- `ax` is the forward specific force in body x (m/s²).
- `wz` is the yaw rate about body z (rad/s). Positive turns left.

Two process noise densities, `sigma_a` (`accel_noise_density`) and `sigma_w` (`gyro_noise_density`), give the increment noise over one step:

```text
speed increment noise:  dv ~ N(0, sigma_a^2 dt)
yaw increment noise:    dpsi ~ N(0, sigma_w^2 dt)
Q = diag(sigma_a^2 dt, sigma_w^2 dt)
```

A GNSS fix gives east and north with independent noise of standard deviation `sigma_g` (`gnss_position_std`):

```text
z = [east_fix, north_fix]^T,   R = sigma_g^2 I_2
```

The prior is diagonal, with `sigma_p` (`initial_position_std`) for east and north, `sigma_psi` (`initial_yaw_std`) for yaw, and `sigma_v` (`initial_speed_std`) for speed:

```text
P0 = diag(sigma_p^2, sigma_p^2, sigma_psi^2, sigma_v^2)
```

## Prediction

Let `psi_m = yaw + wz dt / 2` be the midpoint heading, and `d = (v + ax dt / 2) dt` the distance over the step. The nonlinear step is:

```text
east'  = east + d cos(psi_m)
north' = north + d sin(psi_m)
yaw'   = yaw + wz dt
v'     = v + ax dt
```

The Jacobian `F = df/dx` is the identity, except for these four entries:

```text
F[east, yaw]   = -d sin(psi_m)
F[north, yaw]  =  d cos(psi_m)
F[east, v]     =  dt cos(psi_m)
F[north, v]    =  dt sin(psi_m)
```

The noise enters through the speed and yaw increments, and each increment also moves the midpoint position by half. The noise Jacobian `G`, with columns for the speed increment and the yaw increment, is:

```text
G = [ 0.5 F[east, v]    0.5 F[east, yaw]
      0.5 F[north, v]   0.5 F[north, yaw]
      0                 1
      1                 0 ]
```

The covariance update is:

```text
P' = F P F^T + G Q G^T
```

In square-root form, the filter builds the 4×6 array `A = [F L | G diag(sigma_a sqrt(dt), sigma_w sqrt(dt))]`. For each row it applies Givens rotations to the columns, which zero the entries after that row's diagonal. The first four columns then form the new lower-triangular factor `L'`, and `L' L'^T = A A^T = P'`. The rotations preserve `A A^T`, so the factor stays square and the covariance is never formed.

## Update

A GNSS fix updates one axis at a time, east and north in turn. The axis with the larger standard deviation goes first. For an axis `a`, the observation row is `H = e_a^T`, the innovation is `nu = z_a - x_a`, and the innovation variance is `S = P[a, a] + sigma_g^2`. The gain and the update are:

```text
K  = P H^T / S
x' = x + K nu
P' = P - K H P
```

In square-root form, the filter builds the 5×5 array

```text
[ sigma_g   L[a, :]
  0         L      ]
```

where `L[a, :]` is row `a` of `L`. It rotates the first row until its tail is zero. The first entry of that row is then `sqrt(S)`, and the first column below it is `sqrt(S) K`. The lower-right 4×4 block is the factor of `P'`, with `P' = L' L'^T`. The filter keeps that block as returned, without zeroing the upper part.

The gain is `K = c / sqrt(S)`, where `c` is the first column below the pivot. When `K` itself would overflow or underflow, the filter multiplies `c` by `nu / sqrt(S)` instead, so the correction is finite. The reported standard deviations are `std_east = ||L[east, :]||` and `std_north = ||L[north, :]||`, and these equal `sqrt(P[east, east])` and `sqrt(P[north, north])`.

## Time stepping

`RunEkf` and `RunEkfDynamic` run the same loop. The initial state is at `imu[0].t`. A fix whose time equals `imu[0].t` is applied at the start, and the estimate at that time is marked `updated`. For each later IMU step `[t, t_end)`, every fix with `fix.t <= t_end` is processed in time order. Before each fix, the filter predicts from the current time to `fix.t` and then applies the update. After the last fix, the filter predicts to `t_end`. The estimate at `t_end` is marked `updated` when at least one fix was processed in that step. Fixes earlier than `imu[0].t` or later than the last IMU sample are ignored.

## Interface

```cpp
std::vector<EkfEstimate> RunEkf(std::span<const PlanarImu> imu,
                                std::span<const GnssFix> gnss,
                                const EkfConfig& config = {});

std::vector<EkfEstimate> RunEkfDynamic(std::span<const PlanarImu> imu,
                                       std::span<const GnssFix> gnss,
                                       const DynamicEkfConfig& config = {});
```

`EkfConfig` holds the initial state, the standard deviations listed above, and the noise densities. `DynamicEkfConfig` holds an `EkfConfig` as `filter`, and `state_size`, which must be 4 for this 2D model. Both functions return one `EkfEstimate` per IMU sample. Each estimate has `t`, `state`, `std_east`, `std_north`, and `updated`.

Both series must contain finite values and have strictly increasing times. Empty GNSS gives an IMU-only trajectory. Both functions throw `std::invalid_argument` for invalid inputs or configuration. They throw `std::overflow_error`, or `std::runtime_error`, if the state or covariance overflows at working precision.

## Implementations

Both implementations run the same algorithm: the same prediction, the same Givens reduction, the same update, and the same input checks. A shared template, `PlanarEkfCore<Policy>` in `ekf.h`, holds this code once. The matrix type comes from the policy.

| | Static `PlanarEkf` | Dynamic `PlanarEkfDynamic` |
| --- | --- | --- |
| Header | `include/sensor_fusion/ekf/ekf.h` | `include/sensor_fusion/ekf/ekf_dynamic.h` |
| Matrix type | `Matrix<double, N, M>` with sizes in the type | `Matrix<double, 0, 0>` with sizes set at construction |
| Allocation | None on the filter path | Each step allocates its matrices on the heap |
| Interface | `RunEkf(imu, gnss, EkfConfig)` | `RunEkfDynamic(imu, gnss, DynamicEkfConfig)` |
| State size | 4, fixed by the type | From `DynamicEkfConfig::state_size`, which must be 4 |

What is the same: the model, the formulas, the numbers, the validation, and the exceptions. The tests run the same scenarios through both. They also check that, on the synthetic drive and on several seeds, the largest difference in state, standard deviation, time, and `updated` is at most `1e-9`.

## Known limitations

- The model is planar. It has no pitch, roll, or height. The forward `ax` includes a piece of gravity when the car pitches.
- The yaw is not observed directly. There is no GNSS heading, so yaw and speed are observable only through the position track.
- There are no sensor bias states. The noise is white, with fixed densities.
- There is no innovation gating. A GNSS outlier is accepted in full.
- The filter treats the east and north fixes as independent with equal standard deviation. Correlated or unequal errors are not modelled.
- The state size is 4 in both implementations. A dynamic filter with another size is rejected, because the model does not define one.
- The dynamic implementation allocates on each step. Its speed is not the goal of this implementation.
- The Givens reduction handles the square-root factor directly. The posterior factor after an update is kept as returned, so its upper triangle is not zeroed.
