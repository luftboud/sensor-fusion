# Agent notes

Guidance for coding agents working in this repository. The product specification is [docs/project.md](docs/project.md). The Ukrainian version is [docs/ua/project.md](docs/ua/project.md). This file records only the decisions already made.

## What this is

Sensor Fusion Core is a header-only C++20 library that fuses GNSS and IMU with an extended Kalman filter, plus a comparison of static matrices, dynamic matrices, SIMD, and batched parallelism. Eigen and filterpy are external references for correctness and speed. They are not dependencies of this build.

## Decisions

- Language standard is C++20. Keep `CMAKE_CXX_EXTENSIONS` off.
- The core lives under `include/sensor_fusion/` and is header-only. The `.cpp` files are programs: `apps/report/kitti_run.cpp` (issue #33) and, later, the CLI at `apps/cli/main.cpp`. Do not add the CLI until a CLI task asks for it.
- Future code uses the namespace `sensor_fusion`.
- CMake is a single root [`CMakeLists.txt`](CMakeLists.txt). The current target is the interface library `sensor_fusion` (`sensor_fusion::sensor_fusion`). Do not import the course lab template, PVS Studio, or sanitizer option blocks. AddressSanitizer and UndefinedBehaviorSanitizer are CI flags in [`.github/workflows/ci.yml`](.github/workflows/ci.yml) only.
- Commits, branch names, and rebase merges follow [CONTRIBUTING.md](CONTRIBUTING.md). Do not add a clang-format config or a format check.
- CI is [`.github/workflows/ci.yml`](.github/workflows/ci.yml) (GCC, Clang, sanitizers) and [`.github/workflows/lint.yml`](.github/workflows/lint.yml) (clang-tidy, commit subjects, pull-request branch names, actionlint). clang-tidy settings are in [`.clang-tidy`](.clang-tidy). It skips until a `.cpp` file exists.
- Test, benchmark, and CLI targets are added with the first code that needs them. `tests/` is reserved and empty. Do not FetchContent a test framework, and do not call `enable_testing()`, until the first real test.
- Do not add `benchmarks/` or a Python `tools/` tree until that work is requested. Weeks 7–8 cover benchmarks; Python is for the filterpy check and the three-trajectory figure.
- Raw datasets are local only. See [data/README.md](data/README.md). Do not commit files under `data/` other than that README.

## Layout

| Path | Role |
| --- | --- |
| `include/sensor_fusion/io/` | NMEA parser, IMU reader, KITTI loader and scenario, GNSS CSV reader |
| `include/matrix` | Compile-time-sized matrix library |
| `include/sensor_fusion/ekf/` | 2D scenario types (`state.h`, `scenario.h`), the EKF (`ekf.h`), accuracy metrics (`metrics.h`). The dynamic-matrix baseline and an optional 3D model are later tasks |
| `include/sensor_fusion/sim/` | Synthetic 2D drive (`synthetic_drive.h`) that fills a `Scenario` |
| `apps/report/` | `kitti_run.cpp`: EKF on a KITTI drive, estimates CSV for the filterpy comparison |
| `apps/cli/` | Future `main.cpp`: log to trajectory and metrics |
| `tests/` | Tests, mirroring the headers once a framework is chosen |
| `docs/project.md` | Full specification, schedule, and success criteria |
| `data/` | Local KITTI dump and the recorded track; gitignored except the README |

## Build

```bash
cmake -S . -B build
```

CMake 3.20 or newer. Configuration produces no binaries yet.

## Do not implement yet

Do not add the NMEA parser, IMU or KITTI readers, the matrix library, the EKF predict/update step, benchmarks, or the CLI unless a task asks for that work. The 2D scenario types in `include/sensor_fusion/ekf/` are already in place. Identifiers, comments stay in English.
