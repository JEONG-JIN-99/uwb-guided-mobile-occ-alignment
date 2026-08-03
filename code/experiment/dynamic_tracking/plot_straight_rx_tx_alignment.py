#!/usr/bin/env python3
"""직선 주행에서 로버 기준 Rx/Tx 짐벌 명령 오차를 함께 분석한다."""

import argparse
import bisect
import csv
import math
import os
import statistics
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RESULT_DIR = PROJECT_ROOT / "result/dynamic_tracking/1111"
DEFAULT_ROVER_CSV = (
    DEFAULT_RESULT_DIR / "navigation_line_log_20260729_140938.csv"
)
DEFAULT_RX_CSV = DEFAULT_RESULT_DIR / "rx.csv"
DEFAULT_TX_CSV = DEFAULT_RESULT_DIR / "tx.csv"
DEFAULT_OUTPUT_DIR = DEFAULT_RESULT_DIR / "alignment_accuracy"
RX_PLOT_STEM = "02_rx_command_tracking_corrected"
TX_PLOT_STEM = "03_tx_command_tracking_corrected"
ERROR_PLOT_STEM = "04_rx_tx_directional_error_corrected"
DATA_STEM = "05_straight_line_rx_tx_alignment_corrected"


def wrap_angle(angle_deg):
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def read_rover_rows(path):
    fields = ("elapsed_sec", "heading_deg", "rx_to_tx_deg", "tx_to_rx_deg")
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = set(fields) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"Rover log is missing fields: {sorted(missing)}")
        rows = [
            {field: float(row[field]) for field in fields}
            for row in reader
        ]
    if not rows:
        raise ValueError(f"Rover log is empty: {path}")
    return rows


def read_latest_commands(path, max_elapsed_sec):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        all_rows = list(csv.DictReader(stream))
    starts = [
        index
        for index, row in enumerate(all_rows)
        if row.get("sample_index") == "0"
    ]
    if not starts:
        raise ValueError(f"No run boundary (sample_index=0): {path}")

    rows = []
    for row in all_rows[starts[-1] :]:
        if not row.get("sample_index") or not row.get("command_elapsed_s"):
            continue
        if not row.get("gimbal_command_ros_deg"):
            continue
        sample_index = int(row["sample_index"])
        elapsed_sec = float(row["command_elapsed_s"])
        if sample_index < 1 or elapsed_sec > max_elapsed_sec:
            continue
        rows.append(
            {
                "sample_index": sample_index,
                "elapsed_sec": elapsed_sec,
                "gimbal_command_ros_deg": float(
                    row["gimbal_command_ros_deg"]
                ),
            }
        )
    if not rows:
        raise ValueError(f"No analyzable commands: {path}")
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


def build_rx_points(rover_rows, command_rows, reference_mode):
    initial_reference_deg = rover_rows[0]["rx_to_tx_deg"]
    initial_body_relative_deg = wrap_angle(
        initial_reference_deg - rover_rows[0]["heading_deg"]
    )
    points = []
    for command in command_rows:
        elapsed_sec = command["elapsed_sec"]
        heading_deg = interpolate_angle(
            rover_rows, elapsed_sec, "heading_deg"
        )
        reference_deg = interpolate_angle(
            rover_rows, elapsed_sec, "rx_to_tx_deg"
        )
        if reference_mode == "actual-heading":
            required_command_deg = wrap_angle(
                reference_deg - heading_deg - initial_body_relative_deg
            )
            observed_direction_deg = wrap_angle(
                heading_deg
                + initial_body_relative_deg
                + command["gimbal_command_ros_deg"]
            )
        else:
            required_command_deg = wrap_angle(
                reference_deg - initial_reference_deg
            )
            observed_direction_deg = wrap_angle(
                initial_reference_deg + command["gimbal_command_ros_deg"]
            )
        error_deg = wrap_angle(
            command["gimbal_command_ros_deg"] - required_command_deg
        )
        points.append(
            {
                **command,
                "heading_deg": heading_deg,
                "reference_direction_deg": reference_deg,
                "required_command_deg": required_command_deg,
                "observed_direction_deg": observed_direction_deg,
                "error_deg": error_deg,
                "absolute_error_deg": abs(error_deg),
            }
        )
    return initial_reference_deg, initial_body_relative_deg, points


def build_tx_points(rover_rows, command_rows):
    initial_reference_deg = rover_rows[0]["tx_to_rx_deg"]
    points = []
    for command in command_rows:
        elapsed_sec = command["elapsed_sec"]
        reference_deg = interpolate_angle(
            rover_rows, elapsed_sec, "tx_to_rx_deg"
        )
        required_command_deg = wrap_angle(
            reference_deg - initial_reference_deg
        )
        observed_direction_deg = wrap_angle(
            initial_reference_deg + command["gimbal_command_ros_deg"]
        )
        error_deg = wrap_angle(
            command["gimbal_command_ros_deg"] - required_command_deg
        )
        points.append(
            {
                **command,
                "reference_direction_deg": reference_deg,
                "required_command_deg": required_command_deg,
                "observed_direction_deg": observed_direction_deg,
                "error_deg": error_deg,
                "absolute_error_deg": abs(error_deg),
            }
        )
    return initial_reference_deg, points


def percentile(values, probability):
    ordered = sorted(values)
    index = max(0, math.ceil(probability * len(ordered)) - 1)
    return ordered[index]


def calculate_metrics(node, points):
    errors = [point["error_deg"] for point in points]
    absolute = [abs(error) for error in errors]
    count = len(points)
    return {
        "node": node,
        "sample_count": count,
        "bias_deg": statistics.mean(errors),
        "mae_deg": statistics.mean(absolute),
        "rmse_deg": math.sqrt(statistics.mean(error**2 for error in errors)),
        "median_absolute_error_deg": statistics.median(absolute),
        "p90_absolute_error_deg": percentile(absolute, 0.90),
        "p95_absolute_error_deg": percentile(absolute, 0.95),
        "max_absolute_error_deg": max(absolute),
        "within_3_deg_percent": 100.0
        * sum(error <= 3.0 for error in absolute)
        / count,
        "within_5_deg_percent": 100.0
        * sum(error <= 5.0 for error in absolute)
        / count,
        "within_10_deg_percent": 100.0
        * sum(error <= 10.0 for error in absolute)
        / count,
    }


def choose_output_stem(output_dir, base_stem, extensions):
    stem = base_stem
    suffix = 2
    while any((output_dir / f"{stem}{extension}").exists() for extension in extensions):
        stem = f"{base_stem}_{suffix:02d}"
        suffix += 1
    return stem


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
            "font.size": 10.5,
            "axes.unicode_minus": False,
            "pdf.fonttype": 42,
        }
    )
    return plt


def save_figure(figure, output_dir, stem):
    paths = []
    for extension, options in ((".png", {"dpi": 300}), (".pdf", {})):
        path = output_dir / f"{stem}{extension}"
        figure.savefig(
            path,
            bbox_inches="tight",
            facecolor="white",
            **options,
        )
        paths.append(path)
    return paths


def plot_command_tracking(
    plt,
    node,
    points,
    rover_end_sec,
    output_dir,
    stem,
    color,
):
    figure, axis = plt.subplots(figsize=(11.0, 5.8))
    times = [point["elapsed_sec"] for point in points]
    required = [point["required_command_deg"] for point in points]
    commands = [point["gimbal_command_ros_deg"] for point in points]
    axis.plot(
        times,
        required,
        color="#222222",
        linewidth=2.2,
        label=f"Rover-reference {node} command",
    )
    axis.plot(
        times,
        commands,
        color=color,
        linewidth=1.6,
        marker="o",
        markersize=3.0,
        markevery=3,
        label=f"Applied {node} gimbal command",
    )
    axis.axhline(0.0, color="#888888", linewidth=0.8)
    axis.set(
        title=f"{node} Gimbal Command Tracking during Straight-Line Motion",
        xlabel="Elapsed time (s)",
        ylabel="Gimbal command from initial direction (°)",
        xlim=(0.0, rover_end_sec + 0.25),
    )
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()
    paths = save_figure(figure, output_dir, stem)
    plt.close(figure)
    return paths


def plot_errors(
    plt,
    rx_points,
    tx_points,
    rover_end_sec,
    output_dir,
    stem,
):
    figure, error_axis = plt.subplots(figsize=(11.0, 5.8))
    error_axis.axhspan(-5, 5, color="#009E73", alpha=0.12, label="Within ±5°")
    error_axis.axhspan(
        -10, 10, color="#F0E442", alpha=0.10, label="Within ±10°"
    )
    error_axis.axhline(0.0, color="#222222", linewidth=1.0)
    for node, points, color in (
        ("Rx", rx_points, "#0072B2"),
        ("Tx", tx_points, "#D55E00"),
    ):
        error_axis.plot(
            [point["elapsed_sec"] for point in points],
            [point["error_deg"] for point in points],
            color=color,
            linewidth=1.5,
            marker="o",
            markersize=2.7,
            markevery=3,
            label=f"{node} directional error",
        )
    error_axis.set(
        xlabel="Elapsed time (s)",
        ylabel="Directional error (°)",
        xlim=(0.0, rover_end_sec + 0.25),
    )
    error_axis.grid(alpha=0.25)
    error_axis.legend(loc="best", ncol=2)
    error_axis.set_title(
        "Rx/Tx Directional Error during Straight-Line Motion"
    )
    figure.tight_layout()
    paths = save_figure(figure, output_dir, stem)
    plt.close(figure)
    return paths


def write_metrics(
    path,
    metrics,
    rx_initial_reference_deg,
    rx_initial_body_relative_deg,
    tx_initial_reference_deg,
    rover_end_sec,
    rx_reference_mode,
):
    metric_fields = tuple(metrics[0])
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("analysis_parameter", "value"))
        writer.writerow(("rover_end_sec", f"{rover_end_sec:.6f}"))
        writer.writerow(
            (
                "rx_initial_rover_reference_deg",
                f"{rx_initial_reference_deg:.6f}",
            )
        )
        writer.writerow(
            (
                "tx_initial_rover_reference_deg",
                f"{tx_initial_reference_deg:.6f}",
            )
        )
        writer.writerow(
            (
                "rx_initial_body_relative_deg",
                f"{rx_initial_body_relative_deg:.6f}",
            )
        )
        writer.writerow(("rx_reference_mode", rx_reference_mode))
        writer.writerow(("nominal_rover_heading_deg", "-90.000000"))
        writer.writerow(("rx_mounting_offset_deg", "90.000000"))
        writer.writerow(())
        writer.writerow(metric_fields)
        for row in metrics:
            writer.writerow(
                f"{row[field]:.6f}" if isinstance(row[field], float)
                else row[field]
                for field in metric_fields
            )


def write_samples(path, rx_points, tx_points):
    fields = (
        "node",
        "sample_index",
        "elapsed_sec",
        "heading_deg",
        "reference_direction_deg",
        "required_command_deg",
        "gimbal_command_ros_deg",
        "observed_direction_deg",
        "error_deg",
        "absolute_error_deg",
    )
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for node, points in (("Rx", rx_points), ("Tx", tx_points)):
            for point in points:
                row = {field: point.get(field, "") for field in fields}
                row["node"] = node
                writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze straight-line Rx/Tx gimbal alignment."
    )
    parser.add_argument("--rover-csv", type=Path, default=DEFAULT_ROVER_CSV)
    parser.add_argument("--rx-csv", type=Path, default=DEFAULT_RX_CSV)
    parser.add_argument("--tx-csv", type=Path, default=DEFAULT_TX_CSV)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--rx-reference-mode",
        choices=("actual-heading", "nominal-heading"),
        default="actual-heading",
        help=(
            "Use time-varying rover heading or assume the nominal straight "
            "heading remains fixed."
        ),
    )
    args = parser.parse_args()

    rover_rows = read_rover_rows(args.rover_csv)
    rover_end_sec = rover_rows[-1]["elapsed_sec"]
    rx_commands = read_latest_commands(args.rx_csv, rover_end_sec)
    tx_commands = read_latest_commands(args.tx_csv, rover_end_sec)
    (
        rx_initial_reference_deg,
        rx_initial_body_relative_deg,
        rx_points,
    ) = build_rx_points(
        rover_rows,
        rx_commands,
        args.rx_reference_mode,
    )
    tx_initial_reference_deg, tx_points = build_tx_points(
        rover_rows, tx_commands
    )
    metrics = [
        calculate_metrics("Rx", rx_points),
        calculate_metrics("Tx", tx_points),
    ]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    rx_stem = choose_output_stem(
        args.output_dir, RX_PLOT_STEM, (".png", ".pdf")
    )
    tx_stem = choose_output_stem(
        args.output_dir, TX_PLOT_STEM, (".png", ".pdf")
    )
    error_stem = choose_output_stem(
        args.output_dir, ERROR_PLOT_STEM, (".png", ".pdf")
    )
    data_stem = choose_output_stem(
        args.output_dir, DATA_STEM, ("_metrics.csv", "_samples.csv")
    )
    plt = configure_matplotlib()
    generated = [
        *plot_command_tracking(
            plt,
            "Rx",
            rx_points,
            rover_end_sec,
            args.output_dir,
            rx_stem,
            "#0072B2",
        ),
        *plot_command_tracking(
            plt,
            "Tx",
            tx_points,
            rover_end_sec,
            args.output_dir,
            tx_stem,
            "#D55E00",
        ),
        *plot_errors(
            plt,
            rx_points,
            tx_points,
            rover_end_sec,
            args.output_dir,
            error_stem,
        ),
    ]
    metrics_path = args.output_dir / f"{data_stem}_metrics.csv"
    samples_path = args.output_dir / f"{data_stem}_samples.csv"
    write_metrics(
        metrics_path,
        metrics,
        rx_initial_reference_deg,
        rx_initial_body_relative_deg,
        tx_initial_reference_deg,
        rover_end_sec,
        args.rx_reference_mode,
    )
    write_samples(samples_path, rx_points, tx_points)
    generated.extend((metrics_path, samples_path))

    for path in generated:
        print(path)
    for row in metrics:
        print(
            f"{row['node']}: N={row['sample_count']}, "
            f"bias={row['bias_deg']:.3f} deg, "
            f"MAE={row['mae_deg']:.3f} deg, "
            f"RMSE={row['rmse_deg']:.3f} deg, "
            f"max={row['max_absolute_error_deg']:.3f} deg"
        )


if __name__ == "__main__":
    main()
