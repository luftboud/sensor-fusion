# Sensor Fusion Core

A header-only C++20 library that fuses GNSS and IMU measurements with an extended Kalman filter.

Cheap GNSS receivers are off by several meters and drop out in tunnels, dense streets, and under interference. An IMU keeps producing data, but its estimate drifts. Between satellite fixes the filter integrates the IMU; each GNSS update removes the drift that has built up. The aim is to hold position through outages of tens of seconds.

Matrix sizes in the filter core are known at compile time, so the library is also a place to compare modern C++ approaches: a hand-written static-size matrix library against dynamic matrices, SIMD, and a batched parallel run. Eigen and filterpy are correctness and speed references. They are not build dependencies.

The full project specification, week plan, and success criteria are in [docs/project.md](docs/project.md) (Ukrainian).

## Goals

- A 2D EKF that runs on a full dataset and reports a trajectory with covariances, matching filterpy within numerical error.
- Position RMSE at least 30% below raw GNSS.
- After a 15-second GNSS outage, error at reacquisition at least 3× smaller than dead reckoning without the filter.
- Three optimization approaches measured; static matrices speed up a filter step, with no allocations on the hot path.
- A batched run that scales across cores at 70% efficiency or better, compared with Eigen.
- An NMEA parser that rejects corrupt records, covered by tests.
- A documented library, a CLI from log to trajectory and metrics, one figure with three trajectories, and a report of accuracy and performance.

## Repository layout

```text
sensor-fusion/
  CMakeLists.txt
  README.md
  AGENTS.md
  CONTRIBUTING.md
  .clang-tidy
  .github/workflows/   CI build, sanitizers, and lint
  include/sensor_fusion/
    io/          NMEA, IMU, and KITTI readers
    matrix/      compile-time-sized matrices
    ekf/         dynamic-matrix baseline, then the static 2D model (3D if time allows)
  apps/cli/      future CLI entry point (apps/cli/main.cpp)
  tests/
  docs/project.md
  data/README.md
```

`benchmarks/` (weeks 7–8) and a Python tree for the filterpy check and trajectory plots are not in the tree yet. They will be added with that work.

## Build

The library is an interface target. Configuration succeeds; there is nothing to compile until the first headers and the CLI exist.

```bash
cmake -S . -B build
```

Requires CMake 3.20 or newer and a C++20 compiler.

Datasets stay on disk and are not committed. See [data/README.md](data/README.md).

## Contributing

Branch names, commit messages, and rebase merges are described in [CONTRIBUTING.md](CONTRIBUTING.md). GitHub Actions runs the build, sanitizers, and lint checks on every push and pull request.
