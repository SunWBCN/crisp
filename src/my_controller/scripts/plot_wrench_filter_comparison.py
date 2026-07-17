#!/usr/bin/env python3
"""Plot unfiltered vs low-pass-filtered F/T force from a comparison CSV log.

This script is invoked automatically by the Cartesian admittance controller after control
stops, when logging.plot_filter_comparison (YAML) or --plot-filter-comparison true (CLI) is
enabled. It produces four vertically stacked subplots comparing the external force recorded
immediately before the first-order low-pass filter (unfiltered) against the filter output
(filtered):

  1. Fx: unfiltered and filtered force
  2. Fy: unfiltered and filtered force
  3. Fz: unfiltered and filtered force
  4. |F|: unfiltered and filtered force magnitude

The magnitudes are computed as sqrt(fx^2 + fy^2 + fz^2) for each signal.

Run manually, for example:

  python3 src/my_controller/scripts/plot_wrench_filter_comparison.py \
      src/my_controller/scripts/wrench_filter_comparison/wrench_filter_comparison_20260710_153000.csv \
      -o src/my_controller/scripts/wrench_filter_comparison/wrench_filter_comparison_20260710_153000.png \
      --alpha 0.05

The controller passes the same timestamped basename for the CSV and PNG and the configured
wrench_filter_alpha via --alpha so it can be shown in the figure title.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import matplotlib

# Use a non-interactive backend so the plot can be generated headless (no display needed).
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (must follow matplotlib.use)


# ---------------------------------------------------------------------------
# Column-name configuration. The script tries each candidate from left to right
# and uses the first one present in the CSV header, so it tolerates small header
# variations while defaulting to the names the controller writes.
# ---------------------------------------------------------------------------
TIME_COLUMNS = ["time", "t", "timestamp", "time_s"]

UNFILTERED_COLUMNS = {
    "Fx": ["unfiltered_fx", "raw_fx", "pre_filter_fx"],
    "Fy": ["unfiltered_fy", "raw_fy", "pre_filter_fy"],
    "Fz": ["unfiltered_fz", "raw_fz", "pre_filter_fz"],
}

FILTERED_COLUMNS = {
    "Fx": ["filtered_fx", "lp_fx"],
    "Fy": ["filtered_fy", "lp_fy"],
    "Fz": ["filtered_fz", "lp_fz"],
}

AXES = ["Fx", "Fy", "Fz"]


def find_column(header: Iterable[str], candidates: list[str], label: str) -> str:
    header_set = set(header)
    for name in candidates:
        if name in header_set:
            return name
    raise KeyError(
        f"Could not find a CSV column for '{label}'. Tried: {', '.join(candidates)}"
    )


def read_numeric_columns(
    csv_path: Path, requested: dict[str, list[str]]
) -> dict[str, list[float]]:
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} does not appear to have a CSV header.")

        selected = {
            label: find_column(reader.fieldnames, candidates, label)
            for label, candidates in requested.items()
        }
        data: dict[str, list[float]] = {label: [] for label in requested}

        for row_number, row in enumerate(reader, start=2):
            for label, column in selected.items():
                value = row.get(column, "").strip()
                if value == "":
                    raise ValueError(
                        f"Missing value for column '{column}' at CSV row {row_number}."
                    )
                data[label].append(float(value))

    return data


def magnitude(x: list[float], y: list[float], z: list[float]) -> list[float]:
    return [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(x, y, z)]


def make_plot(csv_path: Path, output_path: Path, alpha: float | None) -> None:
    requested: dict[str, list[str]] = {"time": TIME_COLUMNS}
    for axis in AXES:
        requested[f"unfiltered {axis}"] = UNFILTERED_COLUMNS[axis]
        requested[f"filtered {axis}"] = FILTERED_COLUMNS[axis]

    data = read_numeric_columns(csv_path, requested)
    time = data["time"]

    unfiltered = {axis: data[f"unfiltered {axis}"] for axis in AXES}
    filtered = {axis: data[f"filtered {axis}"] for axis in AXES}

    unfiltered_mag = magnitude(unfiltered["Fx"], unfiltered["Fy"], unfiltered["Fz"])
    filtered_mag = magnitude(filtered["Fx"], filtered["Fy"], filtered["Fz"])

    panels = [
        ("Fx", unfiltered["Fx"], filtered["Fx"]),
        ("Fy", unfiltered["Fy"], filtered["Fy"]),
        ("Fz", unfiltered["Fz"], filtered["Fz"]),
        ("|F|", unfiltered_mag, filtered_mag),
    ]

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    for ax, (name, unfiltered_signal, filtered_signal) in zip(axes, panels):
        # Unfiltered: thin, semi-transparent line. Filtered: thicker solid line.
        ax.plot(
            time,
            unfiltered_signal,
            linewidth=0.8,
            alpha=0.4,
            color="tab:gray",
            label=f"{name} unfiltered",
        )
        ax.plot(
            time,
            filtered_signal,
            linewidth=2.0,
            color="tab:blue",
            label=f"{name} filtered",
        )
        ax.set_ylabel("Force [N]")
        ax.set_title(name)
        ax.grid(True, alpha=0.35)
        ax.legend(loc="best")

    axes[-1].set_xlabel("Time [s]")

    if alpha is not None:
        title = (
            "Unfiltered vs low-pass-filtered F/T force "
            f"(wrench_filter_alpha = {alpha:g})"
        )
    else:
        title = "Unfiltered vs low-pass-filtered F/T force"
    fig.suptitle(title, fontsize=15)

    # tight_layout, leaving headroom for the suptitle so no labels are clipped.
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.98))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot unfiltered vs low-pass-filtered F/T force from a comparison CSV."
    )
    parser.add_argument("csv", type=Path, help="Path to the comparison CSV log file.")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output PNG path. Defaults to the CSV path with a .png extension.",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=None,
        help="Configured wrench_filter_alpha, shown in the figure title.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output = args.output if args.output is not None else args.csv.with_suffix(".png")
    make_plot(args.csv, output, args.alpha)
    print(f"Saved plot to {output}")


if __name__ == "__main__":
    main()
