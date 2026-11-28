# KITTI scenario

`KittiScenario()` in `include/sensor_fusion/io/kitti/kitti_scenario.h` turns a drive that `ReadKittiOxts()` read into the `imu` and `reference` series of a [`Scenario`](ekf-scenario.md). It repeats `load_scenario()` from `foreign_impl/ref_ekf.py`, so the C++ filter and the filterpy reference start from the same numbers. The GNSS fixes come from a CSV file that Python writes; see `include/sensor_fusion/io/gnss/gnss_csv.h`.

## What each series holds

| Field | Source |
| --- | --- |
| `imu[i]` | `t`, `ax`, `wz` of record `i`, the body-frame pair from the [KITTI survey](kitti-oxts-survey.md#two-axis-frames) |
| `reference[i].state.east` | `R * cos(lat0) * (lon - lon0)` |
| `reference[i].state.north` | `R * (lat - lat0)` |
| `reference[i].state.yaw` | OXTS `yaw`, unwrapped like `numpy.unwrap()` |
| `reference[i].state.v` | OXTS `vf`, forward speed along the ground |

`R` is 6 378 137 m, the equatorial radius of WGS-84. The angles are in radians. Record 0 is the origin, so `reference[0]` is at `(0, 0)`, and its `yaw` and `v` are the initial state of both filters.

East and north come from a flat earth around the first record. Over a drive of a few hundred metres the error of this approximation is far below the GNSS noise. It is kept because the reference uses it, not because it is the best projection.

## Matching numpy bit for bit

Each expression follows the Python one operation by operation. On drive 0009 all 447 rows of `t`, `east`, `north`, `yaw`, `v`, `ax`, and `wz` are bit-identical to `load_scenario()`, checked with numpy 2.2.6.

The order of operations matters. `numpy.radians(x)` is `x * (pi / 180)`, with `pi / 180` as one constant. `x * pi / 180` rounds twice and gives a different double:

| Formula | Equal to `numpy.radians` | Bit-identical rows on drive 0009 | Largest difference |
| --- | --- | --- | --- |
| `x * (pi / 180)` | always | 447/447 east, 447/447 north | 0 |
| `x * pi / 180` | 72% of random angles; 277/447 latitudes, 377/447 longitudes | 86/447 east, 47/447 north | 1.2e-10 m east, 1.4e-9 m north |

The difference is far below a millimetre, so the trajectory does not change. It still breaks a bit-for-bit comparison with filterpy, and a later step can grow it. `internal::Radians()` uses the first form.

The same care applies to the rest:

- `east` multiplies `R * cos(lat0)` first and then `(lon - lon0)`, as Python evaluates `EARTH_RADIUS_M * np.cos(lat[0]) * (lon - lon[0])` from left to right.
- `internal::Unwrap()` repeats `numpy.unwrap()`: the wrapped jump is `mod(jump + pi, 2 pi) - pi` with the sign rule of `numpy.mod()`, a jump of exactly `+pi` stays `+pi`, and the corrections are summed in order, as `cumsum` does.
- Time is `(ns - ns0) * 1e-9`, the same product in both languages.
- All latitudes and longitudes are converted to radians and stored before any subtraction, as numpy makes `np.radians(...)` arrays first. gcc at `-O2` and above fuses a multiply and a following add or subtract into one operation with a single rounding, even with `-std=c++20`. Fused, `Radians(lon) - lon0` rounds once instead of twice: record 0 lands about 1e-11 m from the origin instead of on it, and no row matches numpy. A stored value cannot be fused with a later subtraction.

`cos` comes from the C++ math library and from numpy, which may differ in the last bit on another platform. On this machine they agree.

## Running the comparison

`apps/report/kitti_run.cpp` runs the C++ filter on a drive, twice: with every GNSS fix and with the outage cut out. It writes both trajectories for `ref_ekf.py --own`. The fixes come from Python, because the numpy random generator cannot be repeated in C++. First run the reference, which writes the GNSS files:

```bash
python foreign_impl/ref_ekf.py data/kitti/2011_09_26/2011_09_26_drive_0009_sync/oxts --out build/report/kitti
```

Then the C++ filter, and the reference again to compare the two:

```bash
cmake --build build --target kitti_run
build/kitti_run --oxts data/kitti/2011_09_26/2011_09_26_drive_0009_sync/oxts \
    --gnss build/report/kitti/gnss_full.csv \
    --gnss-outage build/report/kitti/gnss_outage.csv
python foreign_impl/ref_ekf.py data/kitti/2011_09_26/2011_09_26_drive_0009_sync/oxts \
    --out build/report/kitti \
    --own build/report/kitti/kitti_estimates.csv \
    --own-outage build/report/kitti/kitti_estimates_outage.csv
```

Use the GNSS files that the current `ref_ekf.py` writes. Files from an older version hold other fixes, and the two filters then differ by metres.

| Flag | Default | Meaning |
| --- | --- | --- |
| `--oxts DIR` | required | KITTI oxts directory |
| `--gnss FILE` | required | fixes for the whole drive, `t,east,north` |
| `--gnss-outage FILE` | required | the same fixes with the outage window cut out: every fix of `--gnss` outside the window, in the same order, and none inside |
| `--out DIR` | `build/report/kitti` | output directory, created if missing |
| `--outage-start S`, `--outage-length S` | 15, 15 | the outage window `[start, start + length)` for the metrics; it must be the window that `ref_ekf.py` cut from the fixes |
| `--initial-position-std`, `--initial-yaw-std`, `--initial-speed-std`, `--accel-noise-density`, `--gyro-noise-density`, `--gnss-position-std` | `EkfConfig` | filter parameters, the same names as the `EkfConfig` fields |

The filter starts at `reference[0]`, as `ref_ekf.py` does. The defaults of `EkfConfig` equal `PARAMS` in `ref_ekf.py`. Pass a flag only if the Python side changes the same value.

Outputs, one row per IMU step:

- `kitti_estimates.csv` and `kitti_estimates_outage.csv`, columns `t,east,north,yaw,v,std_east,std_north,updated`;
- each number is the shortest text that reads back as the same double, so two runs give identical files;
- `updated` is 1 on the step that consumed a fix;
- `kitti_metrics.json` holds the run and its accuracy figures, see below.

`kitti_metrics.json` has six sections, in this order:

| Key | Holds |
| --- | --- |
| `schema` | the layout version, now 1. It goes up when a key is renamed, removed or changes meaning; a new key keeps it |
| `producer` | `"kitti_run"` |
| `dataset` | `oxts` as passed, `steps`, and `duration_s` from the first step to the last |
| `gnss` | `full_fixes`, `outage_fixes`, `outage_start_s`, `outage_length_s` |
| `config` | the six filter parameters, named like the flags: `initial_position_std` for `--initial-position-std` |
| `metrics` | `raw_gnss_rmse_m`, `filter_rmse_m`, `filter_rmse_reduction`, `filter_outage_rmse_route_m`, `filter_outage_rmse_window_m`, `imu_only_window_rmse_m`, and `reacquisition` with `t_s`, `filter_error_m`, `imu_only_error_m`, `ratio` |

The `_m` and `_s` suffixes are metres and seconds. The metrics are those of [`FusionMetrics`](metrics.md#the-set). A value with nothing to average is `null`, and so is `reacquisition` when no fix follows the window.

Each file is written as `<name>.tmp` and renamed when all three are complete, so a failed run leaves earlier outputs as they were.

stdout stays empty. A one-line status or the reason for a failure goes to stderr. Exit status:

- 0: done;
- 1: an input does not read, the outage file is not the full file without the outage window, the filter fails, or an output cannot be written;
- 2: bad arguments, including filter parameters that `EkfConfig` rejects.

On drive 0009, both C++ trajectories match filterpy 2.0.0 to 8e-14 m in position and 2e-14 m in standard deviation, built with clang or with gcc (see [metrics](metrics.md#agreement-with-ref_ekfpy)). The metrics are:

| Metric | Value |
| --- | --- |
| raw GNSS RMSE | 2.108 m |
| filter RMSE | 1.328 m, 37% below raw GNSS |
| outage window RMSE, filter / IMU only | 6.942 m / 22.980 m |
| error at reacquisition (30.43 s), filter / IMU only | 13.71 m / 32.09 m, ratio 2.3 |
