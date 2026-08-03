#!/usr/bin/env python3
"""Plot Tx command-direction error against rover ground truth over time."""

import argparse
import bisect
import csv
import math
import os
import statistics
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RESULT_DIR = PROJECT_ROOT / "result/dynamic_tracking/3m02"
DEFAULT_TX_CSV = DEFAULT_RESULT_DIR / "tx.csv"
DEFAULT_ROVER_CSV = (
    DEFAULT_RESULT_DIR / "arc_chrony_test_vel_log_20260729_125443.csv"
)
DEFAULT_OUTPUT_DIR = DEFAULT_RESULT_DIR / "alignment_accuracy"
OUTPUT_STEM = "05_tx_alignment_error_timeseries"


def wrap_angle(angle_deg):
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def read_rover_rows(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = [
            {
                "elapsed_sec": float(row["elapsed_sec"]),
                "tx_to_rx_deg": float(row["tx_to_rx_deg"]),
            }
            for row in csv.DictReader(stream)
        ]
    if not rows:
        raise ValueError(f"Rover log is empty: {path}")
    return rows


def read_latest_tx_run(path, max_elapsed_sec):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        all_rows = list(csv.DictReader(stream))

    run_starts = [
        index
        for index, row in enumerate(all_rows)
        if row["sample_index"] == "0"
    ]
    if not run_starts:
        raise ValueError(f"No Tx run boundary (sample_index=0) found: {path}")

    rows = []
    for row in all_rows[run_starts[-1] :]:
        if (
            not row["sample_index"]
            or not row["command_elapsed_s"]
            or not row["gimbal_command_ros_deg"]
        ):
            continue
        elapsed_sec = float(row["command_elapsed_s"])
        if int(row["sample_index"]) < 1 or elapsed_sec > max_elapsed_sec:
            continue
        rows.append(
            {
                "sample_index": int(row["sample_index"]),
                "elapsed_sec": elapsed_sec,
                "gimbal_command_ros_deg": float(
                    row["gimbal_command_ros_deg"]
                ),
            }
        )
    if not rows:
        raise ValueError(f"No analyzable Tx commands found: {path}")
    return rows


def interpolate_angle(rows, elapsed_sec, field):
    times = [row["elapsed_sec"] for row in rows]
    if elapsed_sec <= times[0]:
        return rows[0][field]
    if elapsed_sec >= times[-1]:
        return rows[-1][field]

    high = bisect.bisect_right(times, elapsed_sec)
    before = rows[high - 1]
    after = rows[high]
    ratio = (
        (elapsed_sec - before["elapsed_sec"])
        / (after["elapsed_sec"] - before["elapsed_sec"])
    )
    delta = wrap_angle(after[field] - before[field])
    return wrap_angle(before[field] + ratio * delta)


def build_points(rover_rows, tx_rows):
    points = []
    for tx_row in tx_rows:
        reference_deg = interpolate_angle(
            rover_rows,
            tx_row["elapsed_sec"],
            "tx_to_rx_deg",
        )
        error_deg = wrap_angle(
            tx_row["gimbal_command_ros_deg"] - reference_deg
        )
        points.append(
            {
                **tx_row,
                "reference_tx_to_rx_deg": reference_deg,
                "error_deg": error_deg,
                "absolute_error_deg": abs(error_deg),
            }
        )
    return points


def configure_matplotlib():
    os.environ.setdefault(
        "MPLCONFIGDIR",
        str(Path(tempfile.gettempdir()) / "uwb_alignment_matplotlib"),
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 11,
            "axes.unicode_minus": False,
            "pdf.fonttype": 42,
        }
    )
    return plt


def plot_error(plt, points, output_dir):
    times = [point["elapsed_sec"] for point in points]
    errors = [point["error_deg"] for point in points]
    bias = statistics.mean(errors)
    mae = statistics.mean(abs(error) for error in errors)
    rmse = math.sqrt(statistics.mean(error**2 for error in errors))
    worst = max(points, key=lambda point: point["absolute_error_deg"])

    figure, axis = plt.subplots(figsize=(11.0, 5.8))
    axis.axhspan(-5, 5, color="#009E73", alpha=0.10, label="Within ±5°")
    axis.axhspan(-3, 3, color="#56B4E9", alpha=0.15, label="Within ±3°")
    axis.axhline(0.0, color="#222222", linewidth=1.1)
    axis.axhline(
        bias,
        color="#CC79A7",
        linestyle="--",
        linewidth=1.7,
        label=f"Mean bias: {bias:.2f}°",
    )
    axis.plot(
        times,
        errors,
        color="#D55E00",
        linewidth=1.5,
        marker="o",
        markersize=3.0,
        label="Tx command error",
    )
    axis.scatter(
        [worst["elapsed_sec"]],
        [worst["error_deg"]],
        color="#B00020",
        edgecolor="white",
        linewidth=0.7,
        s=65,
        zorder=4,
    )
    axis.annotate(
        f"Maximum |error|: {worst['absolute_error_deg']:.2f}°",
        (worst["elapsed_sec"], worst["error_deg"]),
        xytext=(12, 13),
        textcoords="offset points",
        fontsize=10,
        color="#7A0014",
    )
    axis.text(
        0.99,
        0.03,
        f"N={len(points)}   MAE={mae:.2f}°   RMSE={rmse:.2f}°",
        transform=axis.transAxes,
        ha="right",
        va="bottom",
        bbox={"facecolor": "white", "edgecolor": "#BBBBBB", "alpha": 0.9},
    )
    axis.set(
        title="Tx Alignment Error over Time (3 m Dynamic Tracking)",
        xlabel="Elapsed time (s)",
        ylabel="Tx command error (°)",
        xlim=(0.0, times[-1] + 0.25),
        ylim=(-5.5, 5.5),
    )
    axis.grid(alpha=0.25)
    axis.legend(loc="upper right", ncol=2)
    figure.tight_layout()

    paths = []
    for suffix, options in (
        (".png", {"dpi": 300}),
        (".pdf", {}),
    ):
        path = output_dir / f"{OUTPUT_STEM}{suffix}"
        figure.savefig(path, bbox_inches="tight", facecolor="white", **options)
        paths.append(path)
    plt.close(figure)
    return paths


def write_points(points, output_dir):
    path = output_dir / f"{OUTPUT_STEM}_samples.csv"
    fields = (
        "sample_index",
        "elapsed_sec",
        "reference_tx_to_rx_deg",
        "gimbal_command_ros_deg",
        "error_deg",
        "absolute_error_deg",
    )
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: point[field] for field in fields} for point in points)
    return path


def main():
    parser = argparse.ArgumentParser(
        description="Plot Tx alignment command error over elapsed time."
    )
    parser.add_argument("--tx-csv", type=Path, default=DEFAULT_TX_CSV)
    parser.add_argument("--rover-csv", type=Path, default=DEFAULT_ROVER_CSV)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    rover_rows = read_rover_rows(args.rover_csv)
    tx_rows = read_latest_tx_run(
        args.tx_csv,
        rover_rows[-1]["elapsed_sec"],
    )
    points = build_points(rover_rows, tx_rows)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    generated = [
        *plot_error(configure_matplotlib(), points, args.output_dir),
        write_points(points, args.output_dir),
    ]
    for path in generated:
        print(path)


if __name__ == "__main__":
    main()
