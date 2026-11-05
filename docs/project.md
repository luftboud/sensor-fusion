# Sensor Fusion Core: a GNSS and IMU fusion library in modern C++

## Short description

Cheap GNSS receivers are off by several meters and lose the signal in tunnels, dense streets, and under radio interference. Inertial sensors keep working, but they drift quickly. We are going to build a library that fuses these sources with an extended Kalman filter: the IMU carries the trajectory between satellite measurements, and GNSS removes the drift that has built up. The result should hold accuracy through outages of tens of seconds.

## A proving ground for modern C++

The filter core is matrix operations whose sizes are known at compile time, so the project is a place to compare modern C++ approaches. We write our own template matrix library and measure three axes: static matrices against dynamic ones, SIMD vectorization, and a batched parallel run. Libraries such as Eigen and filterpy are a correctness reference and a speed baseline. They are not dependencies.

## Validation

We will validate the result on public datasets that have a reference trajectory (for example KITTI), plus our own recorded track.

## Project plan

- Weeks 1–2: repository layout, CMake, CI; an NMEA parser and an IMU reader with tests; downloading KITTI.
- Weeks 3–4: our own compile-time-sized matrix library; a simple EKF on dynamic matrices as a baseline; a check against public libraries.
- Weeks 5–6: the core, a 2D EKF on static matrices; a check on the dataset, noise tuning, and GNSS-outage scenarios.
- Weeks 7–8: benchmarks, three optimization approaches, profiling, and a comparison with Eigen.
- Week 9: a test on our own track; an extra goal, if time allows, is a 3D model with 15 states.
- Week 10: the interface, documentation, the report, and a demo.

## Success criteria

1. The 2D EKF runs on a full dataset and reports a trajectory with covariance estimates. The results match filterpy within numerical error.
2. Position RMSE is at least 30% below the synthetic raw GNSS.
3. After an artificial 15-second GNSS outage, the error at reacquisition is at least 3 times smaller than without the correction.
4. Three optimization approaches are implemented and measured. Static matrices speed up a filter step. A test confirms that the hot path does not allocate.
5. A batched run scales across cores at 70% efficiency or better. The speed is compared with Eigen and analyzed.
6. The NMEA parser rejects corrupt records. The tests pass, and the parser is covered by tests.
7. There is a documented library, a CLI from a log to a trajectory and metrics, one figure with three trajectories, and a report with accuracy and performance tables.
