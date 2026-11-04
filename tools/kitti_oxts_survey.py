#!/usr/bin/env python3
"""Survey one KITTI Raw OXTS folder.

Recreate the environment from the repository root:

    python3 -m venv .venv
    .venv/bin/pip install -r tools/requirements.txt

Run:

    .venv/bin/python tools/kitti_oxts_survey.py

A plain run finds the default OXTS folder and writes figures from the
repository root, so the working directory does not matter. A path passed as
an argument is resolved from the current directory.
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OXTS = ROOT / "data/kitti/2011_09_26/2011_09_26_drive_0009_sync/oxts"
FIGURES = ROOT / "docs/kitti-oxts"
EARTH_RADIUS_M = 6_378_137.0
GRAVITY_MPS2 = 9.80665
LONG_STEP_FACTOR = 1.5
FIELD_COUNT = 30

# dataformat.txt labels field 13 as "ay" and calls pos_accuracy a velocity.
# Field 13 is az, specific force toward the vehicle top (m/s^2).
# pos_accuracy is position accuracy (m).
LAT = 0
LON = 1
PITCH = 4
YAW = 5
VN = 6
VE = 7
VF = 8
AX = 11
AY = 12
AZ = 13
AF = 14
AL = 15
AU = 16
WZ = 19
WU = 22
POS_ACCURACY = 23
NUMSATS = 26
POSMODE = 27
VELMODE = 28
ORIMODE = 29


def parse_timestamp_ns(line: str) -> int:
    """Nanoseconds on a naive clock. Differences do not depend on timezone."""
    date_part, time_part = line.split()
    year_s, month_s, day_s = date_part.split("-")
    clock, fraction = time_part.split(".")
    hour_s, minute_s, second_s = clock.split(":")
    days = (datetime(int(year_s), int(month_s), int(day_s)) - datetime(1970, 1, 1)).days
    seconds = days * 86_400 + int(hour_s) * 3_600 + int(minute_s) * 60 + int(second_s)
    nanos = int(fraction.ljust(9, "0")[:9])
    return seconds * 1_000_000_000 + nanos


def load_oxts(oxts: Path) -> tuple[np.ndarray, np.ndarray]:
    data_dir = oxts / "data"
    timestamps_path = oxts / "timestamps.txt"
    if not data_dir.is_dir() or not timestamps_path.is_file():
        raise SystemExit(f"OXTS folder is missing data/ or timestamps.txt: {oxts}")

    files = sorted(data_dir.glob("*.txt"))
    timestamps = [line.strip() for line in timestamps_path.read_text().splitlines() if line.strip()]
    if len(files) != len(timestamps):
        raise SystemExit(
            f"measurement count {len(files)} != timestamp count {len(timestamps)}"
        )

    expected = [f"{index:010d}.txt" for index in range(len(files))]
    actual = [path.name for path in files]
    if actual != expected:
        raise SystemExit("measurement filenames are not contiguous from 0000000000.txt")

    rows: list[list[float]] = []
    for path in files:
        parts = path.read_text().split()
        if len(parts) != FIELD_COUNT:
            raise SystemExit(f"{path.name} has {len(parts)} fields, expected {FIELD_COUNT}")
        rows.append([float(part) for part in parts])

    times_ns = np.asarray([parse_timestamp_ns(line) for line in timestamps], dtype=np.int64)
    if times_ns.size < 2 or np.any(np.diff(times_ns) <= 0):
        raise SystemExit("timestamps are not strictly increasing")
    return times_ns, np.asarray(rows, dtype=float)


def local_metres(lat_deg: np.ndarray, lon_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    lat0 = np.deg2rad(lat_deg[0])
    lon0 = np.deg2rad(lon_deg[0])
    east = EARTH_RADIUS_M * np.cos(lat0) * (np.deg2rad(lon_deg) - lon0)
    north = EARTH_RADIUS_M * (np.deg2rad(lat_deg) - lat0)
    return east, north


def save_figures(
    time_s: np.ndarray,
    east: np.ndarray,
    north: np.ndarray,
    speed: np.ndarray,
    yaw: np.ndarray,
    wz: np.ndarray,
    ax: np.ndarray,
) -> list[Path]:
    FIGURES.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    fig, axes = plt.subplots()
    axes.plot(east, north)
    axes.set_aspect("equal", adjustable="box")
    axes.set_xlabel("East (m)")
    axes.set_ylabel("North (m)")
    axes.set_title("OXTS path relative to the first sample")
    written.append(_save(fig, "path.png"))

    fig, axes = plt.subplots()
    axes.plot(time_s, speed)
    axes.set_xlabel("Time (s)")
    axes.set_ylabel("Speed (m/s)")
    axes.set_title("Horizontal speed")
    written.append(_save(fig, "speed.png"))

    fig, axes = plt.subplots()
    axes.plot(time_s, yaw)
    axes.set_xlabel("Time (s)")
    axes.set_ylabel("Yaw (rad)")
    axes.set_title("Heading (0 = east, counter-clockwise)")
    written.append(_save(fig, "yaw.png"))

    fig, axes_w = plt.subplots()
    axes_a = axes_w.twinx()
    line_w, = axes_w.plot(time_s, wz, color="C0", label="wz")
    line_a, = axes_a.plot(time_s, ax, color="C1", label="ax")
    axes_w.set_xlabel("Time (s)")
    axes_w.set_ylabel("wz (rad/s)")
    axes_a.set_ylabel("ax (m/s^2)")
    axes_w.set_title("Yaw rate and forward acceleration")
    axes_w.legend(handles=[line_w, line_a], loc="best")
    written.append(_save(fig, "wz_ax.png"))
    return written


def _save(fig: matplotlib.figure.Figure, name: str) -> Path:
    path = FIGURES / name
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


def _format_triplet(values: np.ndarray) -> str:
    low, mid, high = np.min(values), np.median(values), np.max(values)
    return f"{low:.6g} / {mid:.6g} / {high:.6g}"


def _print_codes(name: str, values: np.ndarray) -> None:
    modes, counts = np.unique(values.astype(int), return_counts=True)
    print(f"{name}:")
    for mode, count in zip(modes.tolist(), counts.tolist()):
        print(f"  {mode} {count}")


def _rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values * values)))


def report(times_ns: np.ndarray, values: np.ndarray, figures: list[Path]) -> None:
    time_s = (times_ns - times_ns[0]) / 1e9
    dt_s = np.diff(time_s)
    median = float(np.median(dt_s))
    limit = LONG_STEP_FACTOR * median
    long_steps = [
        (index, float(time_s[index]), float(dt), float(dt) / median)
        for index, dt in enumerate(dt_s, start=1)
        if dt > limit
    ]

    print(f"samples: {values.shape[0]}")
    print(f"duration_s: {time_s[-1]:.9f}")
    print(f"dt_min_s: {np.min(dt_s):.9f}")
    print(f"dt_median_s: {median:.9f}")
    print(f"dt_max_s: {np.max(dt_s):.9f}")
    print(f"long_steps (dt > {LONG_STEP_FACTOR:g} * median): {len(long_steps)}")
    for index, stamp_s, dt, ratio in long_steps:
        print(f"  index {index} t_s {stamp_s:.9f} dt_s {dt:.9f} ratio {ratio:.3f}")

    print(f"pos_accuracy_m min/median/max: {_format_triplet(values[:, POS_ACCURACY])}")
    print(f"numsats min/median/max: {_format_triplet(values[:, NUMSATS])}")
    _print_codes("posmode", values[:, POSMODE])
    _print_codes("velmode", values[:, VELMODE])
    _print_codes("orimode", values[:, ORIMODE])

    gravity_leak = GRAVITY_MPS2 * np.sin(values[:, PITCH])
    print(f"az_mps2 min/median/max: {_format_triplet(values[:, AZ])}")
    print(f"ax_mps2 min/median/max: {_format_triplet(values[:, AX])}")
    print(f"ax_mps2 rms: {_rms(values[:, AX]):.6g}")
    print(f"pitch_rad min/median/max: {_format_triplet(values[:, PITCH])}")
    print(f"g_sin_pitch_mps2 rms: {_rms(gravity_leak):.6g}")
    print(f"ax_minus_af_mps2 rms: {_rms(values[:, AX] - values[:, AF]):.6g}")
    print(f"ay_minus_al_mps2 rms: {_rms(values[:, AY] - values[:, AL]):.6g}")
    print(f"az_minus_au_mps2 rms: {_rms(values[:, AZ] - values[:, AU]):.6g}")
    horizontal_speed = np.hypot(values[:, VN], values[:, VE])
    print(f"vf_minus_horizontal_mps rms: {_rms(values[:, VF] - horizontal_speed):.6g}")
    print(f"wz_minus_wu_radps rms: {_rms(values[:, WZ] - values[:, WU]):.6g}")
    odd = np.flatnonzero(
        (values[:, POSMODE] < 0) | (values[:, VELMODE] < 0) | (values[:, ORIMODE] < 0)
    )
    print(f"negative_mode_rows: {odd.size}")
    for index in odd.tolist():
        print(
            f"  index {index} pos_accuracy_m {values[index, POS_ACCURACY]:.6g}"
            f" numsats {values[index, NUMSATS]:.6g}"
        )

    print("figures:")
    for path in figures:
        print(f"  {path.relative_to(ROOT)}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Survey a KITTI Raw OXTS folder.")
    parser.add_argument(
        "oxts",
        nargs="?",
        type=Path,
        default=DEFAULT_OXTS,
        help="path to the oxts directory (default: drive 0009 synced)",
    )
    args = parser.parse_args(argv)
    oxts = args.oxts.expanduser().resolve()

    times_ns, values = load_oxts(oxts)
    time_s = (times_ns - times_ns[0]) / 1e9
    east, north = local_metres(values[:, LAT], values[:, LON])
    speed = np.hypot(values[:, VN], values[:, VE])
    figures = save_figures(
        time_s,
        east,
        north,
        speed,
        values[:, YAW],
        values[:, WZ],
        values[:, AX],
    )
    print(f"oxts: {oxts}")
    print(f"east_m min/max: {np.min(east):.6g} / {np.max(east):.6g}")
    print(f"north_m min/max: {np.min(north):.6g} / {np.max(north):.6g}")
    report(times_ns, values, figures)


if __name__ == "__main__":
    main(sys.argv[1:])
