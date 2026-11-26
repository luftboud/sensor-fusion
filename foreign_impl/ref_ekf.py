"""
    Reference EKF (filterpy) for the 2D scenario, plus metrics and plots.
        - State x = [east, north, yaw, v];
        - inputs ax, wz;
        - measurement = GNSS (east, north).
        - Model and parameters are the "trait" that the C++ EKF must mirror

    Usage:
        // REF ekf results
        python foreign_impl/ref_ekf.py data/kitti/2011_09_26/2011_09_26_drive_0009_sync/oxts
        // In comparison to a custom implementation
        python foreign_impl/ref_ekf.py OXTS_DIR [--own own_full.csv --own-outage own_outage.csv]
"""

import argparse
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

EARTH_RADIUS_M = 6378137.0

PARAMS = dict(
    sigma_accel=0.5,                # m/s^2, process noise on ax
    sigma_yaw_rate=0.02,            # rad/s, process noise on wz
    sigma_gnss=3.0,                 # m, noise of synthetic GNSS AND filter R
    init_std_position=3.0,          # m
    init_std_yaw=0.05,              # rad
    init_std_speed=0.5,             # m/s
    gnss_period=1.0,                # s between fixes
    gnss_offset=0.5,                # s, first fix time (so fixes fall between IMU steps)
    outage_start=15.0,              # s
    outage_length=15.0,             # s
    seed=42,
)


def load_scenario(oxts_dir):
    """
    Read KITTI OXTS into times, reference states and IMU inputs
    """
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
    noise = rng.normal(0.0, params["sigma_gnss"], (fix_times.size, 2))
    fix_positions = np.c_[true_east, true_north] + noise
    return fix_times, fix_positions


def cut_outage(fix_times, fix_positions, outage_start, outage_length):
    """Drop the fixes inside the outage window."""
    in_outage = (fix_times >= outage_start) & (fix_times < outage_start + outage_length)
    return fix_times[~in_outage], fix_positions[~in_outage]


def run_ekf(scenario, fix_times, fix_positions, params):

    times = scenario["times"]
    ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)
    ekf.x = scenario["reference"][0].copy()
    ekf.P = np.diag([
        params["init_std_position"] ** 2,
        params["init_std_position"] ** 2,
        params["init_std_yaw"] ** 2,
        params["init_std_speed"] ** 2,
    ])
    gnss_noise_cov = np.eye(2) * params["sigma_gnss"] ** 2
    measurement_jacobian = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    states = np.zeros((len(times), 4))
    position_stds = np.zeros((len(times), 2))
    updated = np.zeros(len(times), dtype=bool)
    states[0] = ekf.x
    position_stds[0] = np.sqrt(np.diag(ekf.P)[:2])

    for step in range(1, len(times)):
        dt = times[step] - times[step - 1]
        east, north, yaw, speed = ekf.x
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)

        predicted_state = np.array([
            east + speed * cos_yaw * dt,
            north + speed * sin_yaw * dt,
            yaw + scenario["yaw_rate"][step - 1] * dt,
            speed + scenario["accel_x"][step - 1] * dt,
        ])
        ekf.F = np.array([
            [1, 0, -speed * sin_yaw * dt, cos_yaw * dt],
            [0, 1,  speed * cos_yaw * dt, sin_yaw * dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        ekf.Q = np.diag([
            0, 0,
            (params["sigma_yaw_rate"] * dt) ** 2,
            (params["sigma_accel"] * dt) ** 2,
        ])
        ekf.x = predicted_state
        ekf.predict_x = lambda u=0: None
        ekf.predict()

        fixes_this_step = np.where(
            (fix_times > times[step - 1]) & (fix_times <= times[step])
        )[0]
        for fix_index in fixes_this_step:
            ekf.update(
                fix_positions[fix_index],
                lambda x: measurement_jacobian,
                lambda x: x[:2],
                R=gnss_noise_cov,
            )
            updated[step] = True

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

    for csv_path, filterpy_states, filterpy_stds in (
            (args.own, full_states, full_stds),
            (args.own_outage, outage_states, outage_stds)):
        if csv_path:
            own = np.loadtxt(csv_path, delimiter=",", skiprows=1)
            state_diff = np.abs(own[:, 1:5] - filterpy_states).max(axis=0)
            std_diff = np.abs(own[:, 5:7] - filterpy_stds).max(axis=0)
            print(f"{csv_path}: max|own-filterpy| east,north,yaw,v = {state_diff}; "
                  f"std_east,std_north = {std_diff}")

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
