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
