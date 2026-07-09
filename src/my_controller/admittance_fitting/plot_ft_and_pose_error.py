#!/usr/bin/env python3
"""Plot F/T force and TCP pose error from a controller CSV log.

Run from the project root, for example:

  python3 src/my_controller/admittance_fitting/plot_ft_and_pose_error.py pose_error_20260708_150000.csv

The output image is saved to this script's folder by default:

  src/my_controller/admittance_fitting/ft_sensor_and_pose_error.png
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Column-name configuration.
# Change these lists if your CSV uses different names. The script tries each
# name from left to right and uses the first one found in the CSV header.
# ---------------------------------------------------------------------------
TIME_COLUMNS = ["t", "time", "timestamp", "time_s"]

FORCE_COLUMNS = {
    "Fx": ["filtered_fx", "force_x", "fx", "ft_fx"],
    "Fy": ["filtered_fy", "force_y", "fy", "ft_fy"],
    "Fz": ["filtered_fz", "force_z", "fz", "ft_fz"],
}

POSITION_ERROR_COLUMNS = {
    "x error": ["position_error_x", "error_x", "x_error", "tcp_error_x"],
    "y error": ["position_error_y", "error_y", "y_error", "tcp_error_y"],
    "z error": ["position_error_z", "error_z", "z_error", "tcp_error_z"],
}

# Default controller log stores these in degrees:
#   error_roll_deg,error_pitch_deg,error_yaw_deg
# If your CSV stores radians, use names like error_roll/error_pitch/error_yaw
# or set ORIENTATION_UNIT below to "rad".
ORIENTATION_ERROR_COLUMNS = {
    "yaw error": ["error_yaw_deg", "yaw_error_deg", "error_yaw", "yaw_error"],
    "pitch error": ["error_pitch_deg", "pitch_error_deg", "error_pitch", "pitch_error"],
    "roll error": ["error_roll_deg", "roll_error_deg", "error_roll", "roll_error"],
}
ORIENTATION_UNIT = "deg"


def find_column(header: Iterable[str], candidates: list[str], label: str) -> str:
    header_set = set(header)
    for name in candidates:
        if name in header_set:
            return name
    raise KeyError(
        f"Could not find a CSV column for '{label}'. Tried: {', '.join(candidates)}"
    )


def read_numeric_columns(csv_path: Path, requested: dict[str, list[str]]) -> dict[str, list[float]]:
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} does not appear to have a CSV header.")

        selected = {
            label: find_column(reader.fieldnames, candidates, label)
            for label, candidates in requested.items()
        }
        data = {label: [] for label in requested}

        for row_number, row in enumerate(reader, start=2):
            for label, column in selected.items():
                value = row.get(column, "").strip()
                if value == "":
                    raise ValueError(
                        f"Missing value for column '{column}' at CSV row {row_number}."
                    )
                data[label].append(float(value))

    return data


def make_plot(csv_path: Path, output_path: Path) -> None:
    requested_columns = {"time": TIME_COLUMNS}
    requested_columns.update(FORCE_COLUMNS)
    requested_columns.update(POSITION_ERROR_COLUMNS)
    requested_columns.update(ORIENTATION_ERROR_COLUMNS)

    data = read_numeric_columns(csv_path, requested_columns)
    time = data["time"]

    fig, (ax_force, ax_pos) = plt.subplots(
        2,
        1,
        figsize=(12, 9),
        sharex=True,
        constrained_layout=True,
    )

    for label in FORCE_COLUMNS:
        ax_force.plot(time, data[label], linewidth=1.6, label=label)
    ax_force.set_title("F/T Sensor Force vs Time")
    ax_force.set_ylabel("Force [N]")
    ax_force.grid(True, alpha=0.35)
    ax_force.legend(loc="best")

    for label in POSITION_ERROR_COLUMNS:
        ax_pos.plot(time, data[label], linewidth=1.6, label=f"{label} [m]")
    ax_pos.set_title("TCP Pose Error vs Time")
    ax_pos.set_xlabel("Time [s]")
    ax_pos.set_ylabel("Position error [m]")
    ax_pos.grid(True, alpha=0.35)

    ax_ori = ax_pos.twinx()
    for label in ORIENTATION_ERROR_COLUMNS:
        ax_ori.plot(
            time,
            data[label],
            linewidth=1.4,
            linestyle="--",
            label=f"{label} [{ORIENTATION_UNIT}]",
        )
    ax_ori.set_ylabel(f"Orientation error [{ORIENTATION_UNIT}]")

    pos_lines, pos_labels = ax_pos.get_legend_handles_labels()
    ori_lines, ori_labels = ax_ori.get_legend_handles_labels()
    ax_pos.legend(pos_lines + ori_lines, pos_labels + ori_labels, loc="best")

    fig.suptitle(csv_path.name, fontsize=13)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Plot F/T sensor force and TCP pose error from a controller CSV log."
    )
    parser.add_argument("csv", type=Path, help="Path to the controller CSV log file.")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=script_dir / "ft_sensor_and_pose_error.png",
        help="Output image path. Defaults to admittance_fitting/ft_sensor_and_pose_error.png.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    make_plot(args.csv, args.output)
    print(f"Saved plot to {args.output}")


if __name__ == "__main__":
    main()
