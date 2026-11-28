"""
    Reference EKF (filterpy) for the 2D scenario, plus metrics and plots.
        - State x = [east, north, yaw, v];
        - inputs ax, wz;
        - measurement = GNSS (east, north).
        - Model and parameters mirror include/sensor_fusion/ekf/ekf.h

    Usage:
        python foreign_impl/ref_ekf.py data/kitti/2011_09_26/2011_09_26_drive_0009_sync/oxts
        python foreign_impl/ref_ekf.py OXTS_DIR [--own own_full.csv --own-outage own_outage.csv]
            [--json compare.json]

    --json writes max|own - filterpy| per column for each --own run, for
    tools/report.py.
"""

import argparse
import json
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import filterpy
from filterpy.kalman import ExtendedKalmanFilter

EARTH_RADIUS_M = 6378137.0

PARAMS = dict(

    init_std_position=2.0,                  # m, per axis
    init_std_yaw=0.2,                       # rad
    init_std_speed=1.0,                     # m/s
    accel_noise_density=0.3,                # (m/s^2)/sqrt(Hz); increment variance = density^2 * dt
    gyro_noise_density=0.03,                # (rad/s)/sqrt(Hz)
    gnss_position_std=2.0,                  # m, filter measurement noise R (per axis)

    gnss_noise=2.0,                         # m, noise added to the synthetic fixes
    gnss_period=1.0,                        # s between fixes
    gnss_offset=0.5,                        # s, first fix time
    outage_start=15.0,                      # s
    outage_length=15.0,                     # s
    seed=42,
)


def load_scenario(oxts_dir):
    """Read KITTI OXTS into times, reference states and IMU inputs."""
    oxts_dir = pathlib.Path(oxts_dir)
    timestamps = [
        np.datetime64(line.strip().replace(" ", "T"))
        for line in (oxts_dir / "timestamps.txt").read_text().splitlines()
        if line.strip()
    ]
    times = np.array(
        [(stamp - timestamps[0]) / np.timedelta64(1, "ns") for stamp in timestamps]
    ) * 1e-9
    oxts_rows = np.array(
        [np.loadtxt(oxts_dir / "data" / f"{i:010d}.txt") for i in range(len(times))]
    )
    lat = np.radians(oxts_rows[:, 0])
    lon = np.radians(oxts_rows[:, 1])
    east = EARTH_RADIUS_M * np.cos(lat[0]) * (lon - lon[0])
    north = EARTH_RADIUS_M * (lat - lat[0])
    yaw = np.unwrap(oxts_rows[:, 5])
    speed = oxts_rows[:, 8]
    return dict(
        times=times,
        reference=np.c_[east, north, yaw, speed],
        accel_x=oxts_rows[:, 11],
        yaw_rate=oxts_rows[:, 19],
    )


def make_gnss(scenario, params, rng):

    fix_times = np.arange(
        params["gnss_offset"], scenario["times"][-1], params["gnss_period"]
    )
    true_east = np.interp(fix_times, scenario["times"], scenario["reference"][:, 0])
    true_north = np.interp(fix_times, scenario["times"], scenario["reference"][:, 1])
    noise = rng.normal(0.0, params["gnss_noise"], (fix_times.size, 2))
    fix_positions = np.c_[true_east, true_north] + noise
    return fix_times, fix_positions


def cut_outage(fix_times, fix_positions, outage_start, outage_length):
    """
    Drop the fixes inside the outage window
    """
    in_outage = (fix_times >= outage_start) & (fix_times < outage_start + outage_length)
    return fix_times[~in_outage], fix_positions[~in_outage]


def predict(ekf, accel_x, yaw_rate, dt, params):
    """
    Midpoint step and covariance propagation, as in PlanarEkf::Predict
    """
    east, north, yaw, speed = ekf.x
    delta_speed = accel_x * dt
    delta_yaw = yaw_rate * dt
    yaw_mid = yaw + 0.5 * delta_yaw
    distance = (speed + 0.5 * delta_speed) * dt
    cos_yaw, sin_yaw = np.cos(yaw_mid), np.sin(yaw_mid)

    transition = np.eye(4)
    transition[0, 2] = -distance * sin_yaw
    transition[1, 2] = distance * cos_yaw
    transition[0, 3] = dt * cos_yaw
    transition[1, 3] = dt * sin_yaw

    noise_jacobian = np.zeros((4, 2))
    noise_jacobian[0, 0] = 0.5 * transition[0, 3]
    noise_jacobian[1, 0] = 0.5 * transition[1, 3]
    noise_jacobian[3, 0] = 1.0
    noise_jacobian[0, 1] = 0.5 * transition[0, 2]
    noise_jacobian[1, 1] = 0.5 * transition[1, 2]
    noise_jacobian[2, 1] = 1.0
    increment_noise = np.diag([
        params["accel_noise_density"] ** 2 * dt,
        params["gyro_noise_density"] ** 2 * dt,
    ])

    ekf.F = transition
    ekf.Q = noise_jacobian @ increment_noise @ noise_jacobian.T
    ekf.x = np.array([
        east + distance * cos_yaw,
        north + distance * sin_yaw,
        yaw + delta_yaw,
        speed + delta_speed,
    ])
    ekf.predict_x = lambda u=0: None
    ekf.predict()


def run_ekf(scenario, fix_times, fix_positions, params):
    """
    Mirrors RunEkf in ekf.h.

    Each fix is applied at its own timestamp: predict to the fix time with
    the IMU sample that starts the step, update, then predict the rest of
    the step.
    
    Row 0 is the initial state at times[0].
    """
    times = scenario["times"]
    ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
    ekf.x = scenario["reference"][0].copy()
    ekf.P = np.diag([
        params["init_std_position"] ** 2,
        params["init_std_position"] ** 2,
        params["init_std_yaw"] ** 2,
        params["init_std_speed"] ** 2,
    ])
    gnss_noise_cov = np.eye(2) * params["gnss_position_std"] ** 2
    measurement_jacobian = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    def apply_fix(fix_index):
        ekf.update(
            fix_positions[fix_index],
            lambda x: measurement_jacobian,
            lambda x: x[:2],
            R=gnss_noise_cov,
        )

    states = np.zeros((len(times), 4))
    position_stds = np.zeros((len(times), 2))
    updated = np.zeros(len(times), dtype=bool)

    time = times[0]
    next_fix = 0
    while next_fix < len(fix_times) and fix_times[next_fix] < time:
        next_fix += 1
    if next_fix < len(fix_times) and fix_times[next_fix] == time:
        apply_fix(next_fix)
        next_fix += 1
        updated[0] = True
    states[0] = ekf.x
    position_stds[0] = np.sqrt(np.diag(ekf.P)[:2])

    for step in range(1, len(times)):
        step_end = times[step]
        imu_ax = scenario["accel_x"][step - 1]
        imu_wz = scenario["yaw_rate"][step - 1]
        while next_fix < len(fix_times) and fix_times[next_fix] <= step_end:
            predict(ekf, imu_ax, imu_wz, fix_times[next_fix] - time, params)
            apply_fix(next_fix)
            time = fix_times[next_fix]
            next_fix += 1
            updated[step] = True
        if time < step_end:
            predict(ekf, imu_ax, imu_wz, step_end - time, params)
            time = step_end
        states[step] = ekf.x
        position_stds[step] = np.sqrt(np.diag(ekf.P)[:2])
    return states, position_stds, updated


def position_error(estimate, reference):
    return np.hypot(estimate[:, 0] - reference[:, 0], estimate[:, 1] - reference[:, 1])


def rmse(errors):
    return float(np.sqrt(np.mean(np.square(errors))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("oxts_dir")
    parser.add_argument("--own", help="C++ estimates CSV, full GNSS")
    parser.add_argument("--own-outage", help="C++ estimates CSV, with outage")
    parser.add_argument("--out", default="docs/ekf-compare")
    parser.add_argument("--json", help="write the --own comparison as JSON")
    args = parser.parse_args()
    params = PARAMS
    out_dir = pathlib.Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    scenario = load_scenario(args.oxts_dir)
    times, reference = scenario["times"], scenario["reference"]

    gnss_times, gnss_positions = make_gnss(
        scenario, params, np.random.default_rng(params["seed"]))
    gnss_times_outage, gnss_positions_outage = cut_outage(
        gnss_times, gnss_positions, params["outage_start"], params["outage_length"])
    np.savetxt(out_dir / "gnss_full.csv", np.c_[gnss_times, gnss_positions],
               delimiter=",", header="t,east,north", comments="")
    np.savetxt(out_dir / "gnss_outage.csv",
               np.c_[gnss_times_outage, gnss_positions_outage],
               delimiter=",", header="t,east,north", comments="")

    full_states, full_stds, full_updated = run_ekf(
        scenario, gnss_times, gnss_positions, params)
    outage_states, outage_stds, outage_updated = run_ekf(
        scenario, gnss_times_outage, gnss_positions_outage, params)
    dead_states, _, _ = run_ekf(scenario, np.array([]), np.zeros((0, 2)), params)

    outage_begin = params["outage_start"]
    outage_end = outage_begin + params["outage_length"]
    in_outage = (times >= outage_begin) & (times < outage_end)
    steps_after_outage = np.where(times >= outage_end)[0]
    first_update_after = next(i for i in steps_after_outage if outage_updated[i])

    raw_gnss_error = np.hypot(
        gnss_positions[:, 0] - np.interp(gnss_times, times, reference[:, 0]),
        gnss_positions[:, 1] - np.interp(gnss_times, times, reference[:, 1]))
    error_full = position_error(full_states, reference)
    error_outage = position_error(outage_states, reference)
    error_dead = position_error(dead_states, reference)

    print(f"Raw GNSS RMSE:            {rmse(raw_gnss_error):.3f} m")
    print(f"EKF full   RMSE (route):  {rmse(error_full):.3f} m   "
          f"({100 * (1 - rmse(error_full) / rmse(raw_gnss_error)):.0f}% below raw; "
          f"target >=30%)")
    print(f"EKF outage RMSE (route):  {rmse(error_outage):.3f} m")
    print(f"EKF outage RMSE (window): {rmse(error_outage[in_outage]):.3f} m   "
          f"[{outage_begin:.0f}, {outage_end:.0f}) s")
    print(f"Dead reckoning RMSE (window): {rmse(error_dead[in_outage]):.3f} m")
    before = first_update_after - 1
    print(f"Error just before reacquisition (t={times[before]:.1f}s): "
          f"EKF {error_outage[before]:.2f} m, dead-reckoning {error_dead[before]:.2f} m "
          f"(ratio {error_dead[before] / max(error_outage[before], 1e-9):.1f}x; "
          f"target >=3x)")

    comparison_runs = {}
    for run_name, csv_path, filterpy_states, filterpy_stds in (
            ("full", args.own, full_states, full_stds),
            ("outage", args.own_outage, outage_states, outage_stds)):
        if csv_path:
            own = np.loadtxt(csv_path, delimiter=",", skiprows=1, ndmin=2)
            run = {"own": csv_path, "rows": int(own.shape[0]),
                   "filterpy_rows": int(filterpy_states.shape[0])}
            comparison_runs[run_name] = run
            if own.shape[0] != filterpy_states.shape[0] or own.shape[1] < 7:
                run["error"] = "row count mismatch"
                print(f"{csv_path}: {own.shape[0]} rows, filterpy has "
                      f"{filterpy_states.shape[0]}")
                continue
            state_diff = np.abs(own[:, 1:5] - filterpy_states).max(axis=0)
            std_diff = np.abs(own[:, 5:7] - filterpy_stds).max(axis=0)
            run["max_abs_diff"] = dict(zip(
                ("east", "north", "yaw", "v", "std_east", "std_north"),
                (float(x) for x in np.r_[state_diff, std_diff])))
            print(f"{csv_path}: max|own-filterpy| east,north,yaw,v = {state_diff}; "
                  f"std_east,std_north = {std_diff}")
    if args.json:
        json_path = pathlib.Path(args.json)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps({
            "schema": 1,
            "producer": "ref_ekf.py",
            "filterpy_version": filterpy.__version__,
            "oxts": args.oxts_dir,
            "runs": comparison_runs,
        }, indent=2) + "\n")

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(reference[:, 0], reference[:, 1], "k", label="reference")
    ax.plot(outage_states[:, 0], outage_states[:, 1], label="EKF with outage")
    ax.plot(dead_states[:, 0], dead_states[:, 1], "--", label="IMU only")
    ax.plot(gnss_positions[:, 0], gnss_positions[:, 1], ".", ms=4,
            label="raw GNSS (synthetic)")
    ax.axis("equal"); ax.legend()
    ax.set_xlabel("east, m"); ax.set_ylabel("north, m")
    fig.savefig(out_dir / "trajectories.png", dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, error_full, label="EKF full GNSS")
    ax.plot(times, error_outage, label="EKF with outage")
    ax.plot(times, error_dead, label="IMU only")
    ax.axvspan(outage_begin, outage_end, color="grey", alpha=0.2, label="outage")
    ax.set_xlabel("t, s"); ax.set_ylabel("position error, m"); ax.legend()
    fig.savefig(out_dir / "errors.png", dpi=150); plt.close(fig)

    header = "t,east,north,yaw,v,std_east,std_north,updated"
    for name, states, stds, updated in (
            ("filterpy_full", full_states, full_stds, full_updated),
            ("filterpy_outage", outage_states, outage_stds, outage_updated)):
        np.savetxt(out_dir / f"{name}.csv",
                   np.c_[times, states, stds, updated.astype(int)],
                   delimiter=",", header=header, comments="")


if __name__ == "__main__":
    main()
