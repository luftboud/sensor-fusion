# Metrics

`include/sensor_fusion/ekf/metrics.h` measures a filter run against the reference trajectory. Each metric is a small function, so a new metric is one more function and one more field in `FusionMetrics`. `ComputeFusionMetrics()` computes the set that `foreign_impl/ref_ekf.py` prints, in the same way, so the C++ and Python numbers can be compared.

## Inputs

`FusionRuns` holds the reference, every GNSS fix, and three runs of the filter over the same IMU steps:

- `full`: with every fix;
- `outage`: with the fixes of the outage window cut out;
- `imu_only`: with no fix, which is dead reckoning.

Each run must have one estimate per reference sample, at the same times. Otherwise `ComputeFusionMetrics()` returns `std::nullopt`.

`TimeWindow` is the half-open interval `[start, end)`, as `cut_outage()` in `ref_ekf.py` uses it.

## The set

| Field | Meaning | `ref_ekf.py` |
| --- | --- | --- |
| `raw_gnss_rmse` | RMSE of each fix against the reference at its time | `Raw GNSS RMSE` |
| `filter_rmse` | RMSE of `full` over every step | `EKF full RMSE (route)` |
| `filter_rmse_reduction` | `1 - filter_rmse / raw_gnss_rmse`; 0.3 is 30% below raw GNSS, success criterion 2 | `% below raw` |
| `outage_rmse` | RMSE of `outage` over every step | `EKF outage RMSE (route)` |
| `outage_window_rmse` | RMSE of `outage` on the steps in the window | `EKF outage RMSE (window)` |
| `imu_only_window_rmse` | RMSE of `imu_only` on the same steps | `Dead reckoning RMSE (window)` |
| `reacquisition` | see below; success criterion 3 | `Error just before reacquisition` |

`reacquisition` is the step before the first estimate at or after the window end that consumed a fix. That step has drifted the longest without a correction. It holds:

- `t`;
- `filter_error`, the error of `outage` on that step;
- `imu_only_error`, the error of `imu_only` on that step;
- `ratio = imu_only_error / max(filter_error, 1e-9)`.

A value with nothing to average is `std::nullopt` (`null` in JSON), not zero: no fixes, no step inside the window, or no fix after it.

## Interpolation

A fix rarely falls on an IMU step, so `raw_gnss_rmse` needs the reference between samples. `ReferencePositionAt()` repeats `numpy.interp()`:

- the first position before the first sample;
- the last position after the last sample;
- the sample itself on a sample;
- `slope * (t - t0) + y0` in between.

The arm64 build of numpy evaluates that last expression with one rounding (a fused multiply-add). `std::fma()` does the same on every platform, so the C++ result does not depend on the compiler.

## Agreement with ref_ekf.py

On drive 0009, with the fixes from the current `ref_ekf.py`, every value of `ComputeFusionMetrics()` matches the Python computation to 1e-10 m or better. The spread comes from the trajectories, not from the metrics. `RunEkf()` built with clang matches filterpy to 6e-14 m. Built with gcc, it matches to 3e-10 m, because clang on arm64 fuses multiply-adds the way numpy does and gcc in ISO mode does not. Both are well within numerical error.
