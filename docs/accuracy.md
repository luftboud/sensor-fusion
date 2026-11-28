# Accuracy on the synthetic drive

`apps/report/accuracy.cpp` measures success criteria 2 and 3 of [the project](project.md) on the synthetic drive of [sim/synthetic_drive.h](../include/sensor_fusion/sim/synthetic_drive.h). The KITTI drive is for criterion 1 and is measured by [kitti_run](kitti-scenario.md).

- **Criterion 2:** the position RMSE of the filter over the whole route is at least 30% below the RMSE of the raw GNSS fixes. The drive has no outage.
- **Criterion 3:** after the GNSS outage, the error at reacquisition without the correction is at least 3 times the error with it.

## Method

Each seed gives two drives with the same noise. The only difference is the outage:

- **Without outage** (`outage_duration = 0`): the filter runs with every fix, and the IMU-only run gives the dead-reckoning baseline. Criterion 2 is read from this drive.
- **With outage** (default: 25 s to 40 s): the filter runs with the fixes that remain, and the IMU-only run is the same as above. Criterion 3 is read from this drive.

The synthetic generator draws the noise of every fix, also inside the outage, so a fix outside the outage is the same in both drives. The IMU data comes from its own noise stream and is the same too. The program checks that the IMU data and the reference match before it compares anything.

The filter starts at the first reference sample, as in `kitti_run`. The EKF uses the defaults of `EkfConfig`.

## Metrics

The numbers come from `ComputeFusionMetrics()` in [metrics.h](../include/sensor_fusion/ekf/metrics.h), the same functions as the KITTI report. See [metrics](metrics.md). The program only chooses the runs.

## Verdict

The seeds are `1..N`, with `N = 20` by default (`--seeds N`, 1 to 1000). For each criterion the program writes the value of every seed, the median, the minimum and the maximum. A criterion is met only when **every** seed meets it. That is stricter than the median and is deliberate: one bad seed should not hide behind a good average.

The targets are in [targets.h](../include/sensor_fusion/ekf/targets.h): `kCriterion2MinReduction = 0.3` and `kCriterion3MinRatio = 3.0`.

## Running

```bash
cmake --build build --target accuracy
build/accuracy --seeds 20
```

- Stdout stays empty.
- Stderr gets one verdict line per criterion and the path of the output.
- The output is `build/report/accuracy/accuracy.json` (`--out DIR` changes the directory).
- The exit status is 0 when the numbers were computed, whether or not the criteria are met. A criterion that is not met is in the verdict, not in the exit status.
- 1: an internal error or a write error; 2: bad arguments.

## Output

`accuracy.json` has these fields:

| Field | Meaning |
| --- | --- |
| `schema`, `producer`, `seeds` | version, producer name, number of seeds |
| `scenario` | outage start and duration, GNSS period and noise of the drive |
| `ekf` | the `EkfConfig` values used |
| `runs[]` | per seed: `criterion2` (raw GNSS RMSE, filter RMSE, reduction) and `criterion3` (time of reacquisition, filter error, IMU-only error, ratio) |
| `criteria` | per criterion: `target`, `median`, `min`, `max`, `seeds_met`, `verdict` |

A value that could not be computed is `null`. A `null` counts as not meeting the target.

## Result at the time of writing

Twenty seeds, 2026-11-28, with the default filter parameters:

| Criterion | Target | Median | Min | Max | Seeds that meet it | Verdict |
| --- | --- | --- | --- | --- | --- | --- |
| 2: reduction below raw GNSS | 30% | 36.5% | 26.9% | 44.9% | 19 of 20 | not met |
| 3: ratio at reacquisition | 3x | 2.23x | 1.13x | 4.03x | 4 of 20 | not met |

These numbers are measured, not tuned. Neither criterion is met as the filter stands. The parameters were not changed to fit the targets.

## Limits

- One drive shape: `DemoDrive()`, 65 s, with one outage. Other drives are not measured.
- One noise setting: the defaults of `SyntheticConfig`.
- The seeds vary the noise, not the drive.
