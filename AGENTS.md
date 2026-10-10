# Agent notes

Guidance for coding agents working in this repository. The product specification is [docs/project.md](docs/project.md) (Ukrainian). This file records only the decisions already made.

## What this is

Sensor Fusion Core is a header-only C++20 library that fuses GNSS and IMU with an extended Kalman filter, plus a comparison of static matrices, dynamic matrices, SIMD, and batched parallelism. Eigen and filterpy are external references for correctness and speed. They are not dependencies of this build.

## Decisions

- Language standard is C++20. Keep `CMAKE_CXX_EXTENSIONS` off.
- The core lives under `include/sensor_fusion/` and is header-only. The only planned `.cpp` is the CLI at `apps/cli/main.cpp`. Do not add that file until a CLI task asks for it.
- Future code uses the namespace `sensor_fusion`.
- CMake is a single root [`CMakeLists.txt`](CMakeLists.txt). The current target is the interface library `sensor_fusion` (`sensor_fusion::sensor_fusion`). Do not import the course lab template, PVS Studio, or sanitizer option blocks.
- Test, benchmark, and CLI targets are added with the first code that needs them. `tests/` is reserved and empty. Do not FetchContent a test framework, and do not call `enable_testing()`, until the first real test.
- Do not add `benchmarks/` or a Python `tools/` tree until that work is requested. Weeks 7–8 cover benchmarks; Python is for the filterpy check and the three-trajectory figure.
- Raw datasets are local only. See [data/README.md](data/README.md). Do not commit files under `data/` other than that README.

## Layout

| Path | Role |
| --- | --- |
| `include/sensor_fusion/io/` | NMEA parser, IMU reader, KITTI loader |
| `include/sensor_fusion/matrix/` | Compile-time-sized matrix library |
| `include/sensor_fusion/ekf/` | Dynamic-matrix EKF baseline, then the static 2D model, optionally 3D |
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

Do not add the NMEA parser, IMU or KITTI readers, the matrix library, the EKF, benchmarks, CI, or the CLI unless a task asks for that work. Identifiers, comments, and new docs other than `docs/project.md` stay in English.
