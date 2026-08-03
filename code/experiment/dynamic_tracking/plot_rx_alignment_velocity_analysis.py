#!/usr/bin/env python3
"""Rx 방향 추종 오차와 로버 속도를 같은 시간축에서 분석한다."""

import argparse
import csv
import math
import os
import statistics
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ROVER_CSV = (
    PROJECT_ROOT / "arc_chrony_test_vel_log_20260729_125443.csv"
)
DEFAULT_RX_CSV = PROJECT_ROOT / "result/dynamic_tracking/3m02/rx.csv"
DEFAULT_OUTPUT_DIR = (
    PROJECT_ROOT / "result/dynamic_tracking/3m02/alignment_accuracy"
)
DEFAULT_STEM = "04_direction_error_velocity_analysis"


def wrap_angle(angle_deg):
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def read_rover_rows(path):
    required = {
        "elapsed_sec",
        "heading_deg",
        "rx_to_tx_deg",
        "linear_velocity_mps",
        "angular_velocity_rps",
    }
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                f"로버 로그에 필요한 열이 없습니다: {sorted(missing)}"
            )
        rows = [
            {
                "elapsed_sec": float(row["elapsed_sec"]),
                "heading_deg": float(row["heading_deg"]),
                "rx_to_tx_deg": float(row["rx_to_tx_deg"]),
                "linear_velocity_mps": float(row["linear_velocity_mps"]),
                "angular_velocity_deg_s": math.degrees(
                    float(row["angular_velocity_rps"])
                ),
            }
            for row in reader
        ]
    if not rows:
        raise ValueError(f"로버 로그가 비어 있습니다: {path}")
    return rows


def read_rx_rows(path, max_elapsed_sec):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for row in csv.DictReader(stream):
            if not row["sample_index"] or not row["command_elapsed_s"]:
                continue
            elapsed_sec = float(row["command_elapsed_s"])
            if (
                int(row["sample_index"]) < 1
                or elapsed_sec > max_elapsed_sec
                or not row["gimbal_command_ros_deg"]
            ):
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
        raise ValueError(f"분석 가능한 Rx 명령이 없습니다: {path}")
    return rows


def interpolation_indices(rows, elapsed_sec):
    if elapsed_sec <= rows[0]["elapsed_sec"]:
        return 0, 0, 0.0
    if elapsed_sec >= rows[-1]["elapsed_sec"]:
        last = len(rows) - 1
        return last, last, 0.0

    low = 0
    high = len(rows) - 1
    while high - low > 1:
        middle = (low + high) // 2
        if rows[middle]["elapsed_sec"] <= elapsed_sec:
            low = middle
        else:
            high = middle
    before = rows[low]
    after = rows[high]
    ratio = (
        (elapsed_sec - before["elapsed_sec"])
        / (after["elapsed_sec"] - before["elapsed_sec"])
    )
    return low, high, ratio


def interpolate_scalar(rows, elapsed_sec, field):
    low, high, ratio = interpolation_indices(rows, elapsed_sec)
    return rows[low][field] + ratio * (rows[high][field] - rows[low][field])


def interpolate_angle(rows, elapsed_sec, field):
    low, high, ratio = interpolation_indices(rows, elapsed_sec)
    delta = wrap_angle(rows[high][field] - rows[low][field])
    return wrap_angle(rows[low][field] + ratio * delta)


def build_points(rover_rows, rx_rows):
    points = []
    for rx_row in rx_rows:
        elapsed_sec = rx_row["elapsed_sec"]
        heading_deg = interpolate_angle(
            rover_rows,
            elapsed_sec,
            "heading_deg",
        )
        reference_deg = interpolate_angle(
            rover_rows,
            elapsed_sec,
            "rx_to_tx_deg",
        )
        observation_deg = wrap_angle(
            heading_deg
            + 90.0
            + rx_row["gimbal_command_ros_deg"]
        )
        error_deg = wrap_angle(observation_deg - reference_deg)
        points.append(
            {
                **rx_row,
                "heading_deg": heading_deg,
                "reference_deg": reference_deg,
                "observation_deg": observation_deg,
                "error_deg": error_deg,
                "absolute_error_deg": abs(error_deg),
                "linear_velocity_mps": interpolate_scalar(
                    rover_rows,
                    elapsed_sec,
                    "linear_velocity_mps",
                ),
                "angular_velocity_deg_s": interpolate_scalar(
                    rover_rows,
                    elapsed_sec,
                    "angular_velocity_deg_s",
                ),
            }
        )
    return points


def pearson_correlation(left, right):
    if len(left) != len(right) or len(left) < 2:
        return math.nan
    left_mean = statistics.mean(left)
    right_mean = statistics.mean(right)
    numerator = sum(
        (x - left_mean) * (y - right_mean)
        for x, y in zip(left, right)
    )
    left_energy = sum((value - left_mean) ** 2 for value in left)
    right_energy = sum((value - right_mean) ** 2 for value in right)
    denominator = math.sqrt(left_energy * right_energy)
    if denominator == 0.0:
        return math.nan
    return numerator / denominator


def calculate_metrics(points):
    errors = [point["error_deg"] for point in points]
    absolute_errors = [point["absolute_error_deg"] for point in points]
    linear = [point["linear_velocity_mps"] for point in points]
    angular_abs = [
        abs(point["angular_velocity_deg_s"])
        for point in points
    ]
    return [
        ("Number of aligned commands", len(points), "count"),
        ("Mean signed error (Bias)", statistics.mean(errors), "deg"),
        ("Mean absolute error (MAE)", statistics.mean(absolute_errors), "deg"),
        (
            "Root mean square error (RMSE)",
            math.sqrt(statistics.mean(value**2 for value in errors)),
            "deg",
        ),
        ("Mean linear velocity", statistics.mean(linear), "m/s"),
        ("Maximum linear velocity", max(linear), "m/s"),
        (
            "Mean absolute angular velocity",
            statistics.mean(angular_abs),
            "deg/s",
        ),
        ("Maximum absolute angular velocity", max(angular_abs), "deg/s"),
        (
            "Pearson r: absolute error vs linear velocity",
            pearson_correlation(absolute_errors, linear),
            "correlation",
        ),
        (
            "Pearson r: absolute error vs absolute angular velocity",
            pearson_correlation(absolute_errors, angular_abs),
            "correlation",
        ),
    ]


def configure_matplotlib(font_path=None):
    os.environ.setdefault(
        "MPLCONFIGDIR",
        str(Path(tempfile.gettempdir()) / "uwb_alignment_matplotlib"),
    )
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import font_manager
    import matplotlib.pyplot as plt

    candidates = (
        Path(font_path).expanduser() if font_path else None,
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    )
    selected = next(
        (path for path in candidates if path is not None and path.exists()),
        None,
    )
    if selected is None:
        raise RuntimeError("그림 글꼴을 찾지 못했습니다.")
    font_manager.fontManager.addfont(selected)
    font_name = font_manager.FontProperties(fname=selected).get_name()
    plt.rcParams.update(
        {
            "font.family": font_name,
            "font.size": 11,
            "axes.labelsize": 12,
            "axes.titlesize": 13,
            "legend.fontsize": 9.5,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            "axes.unicode_minus": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    return plt


def choose_unused_stem(output_dir, base_stem):
    candidate = base_stem
    sequence = 2
    while any(
        (output_dir / f"{candidate}.{suffix}").exists()
        for suffix in ("png", "pdf", "csv")
    ) or (output_dir / f"{candidate}_samples.csv").exists():
        candidate = f"{base_stem}_{sequence}"
        sequence += 1
    return candidate


def plot_analysis(plt, points, rover_end_sec, output_dir, stem):
    times = [point["elapsed_sec"] for point in points]
    references = [point["reference_deg"] for point in points]
    observations = [point["observation_deg"] for point in points]
    errors = [point["error_deg"] for point in points]
    linear = [point["linear_velocity_mps"] for point in points]
    angular = [point["angular_velocity_deg_s"] for point in points]
    bias = statistics.mean(errors)
    worst = max(points, key=lambda point: point["absolute_error_deg"])

    figure, axes = plt.subplots(
        3,
        1,
        figsize=(12.0, 11.0),
        sharex=True,
        gridspec_kw={"height_ratios": (1.05, 1.0, 0.9)},
    )
    direction_axis, error_axis, velocity_axis = axes

    direction_axis.plot(
        times,
        references,
        color="#222222",
        linewidth=2.4,
        label="Reference direction (Rx-to-Tx)",
    )
    direction_axis.plot(
        times,
        observations,
        color="#0072B2",
        linewidth=1.8,
        marker="o",
        markersize=2.8,
        markevery=4,
        label="Commanded absolute viewing direction",
    )
    direction_axis.set_ylabel("Absolute direction (°)")
    direction_axis.set_title("Rx Direction Tracking, Error, and Rover Velocity")
    direction_axis.legend(loc="upper left")
    direction_axis.grid(alpha=0.25)

    error_axis.axhspan(-10, 10, color="#F0E442", alpha=0.14)
    error_axis.axhspan(-5, 5, color="#009E73", alpha=0.16)
    error_axis.axhline(0.0, color="#222222", linewidth=1.0)
    error_axis.axhline(
        bias,
        color="#CC79A7",
        linestyle="--",
        linewidth=1.7,
        label=f"Mean bias ({bias:.2f}°)",
    )
    error_axis.plot(
        times,
        errors,
        color="#D55E00",
        linewidth=1.7,
        marker="o",
        markersize=2.8,
        label="Directional error",
    )
    error_axis.axvline(
        worst["elapsed_sec"],
        color="#D55E00",
        linestyle=":",
        linewidth=1.2,
        label=(
            f"Maximum |error| ({worst['absolute_error_deg']:.2f}° "
            f"at {worst['elapsed_sec']:.2f}s)"
        ),
    )
    error_axis.set_ylabel("Directional error (°)")
    error_axis.legend(loc="lower right")
    error_axis.grid(alpha=0.25)

    linear_line = velocity_axis.plot(
        times,
        linear,
        color="#0072B2",
        linewidth=1.8,
        label="Linear velocity",
    )[0]
    velocity_axis.fill_between(
        times,
        linear,
        color="#0072B2",
        alpha=0.12,
    )
    velocity_axis.set_ylabel("Linear velocity (m/s)", color="#0072B2")
    velocity_axis.tick_params(axis="y", labelcolor="#0072B2")
    velocity_axis.set_xlabel("Elapsed time (s)")
    velocity_axis.grid(alpha=0.25)

    angular_axis = velocity_axis.twinx()
    angular_line = angular_axis.plot(
        times,
        angular,
        color="#CC79A7",
        linewidth=1.4,
        label="Angular velocity",
    )[0]
    angular_axis.axhline(0.0, color="#777777", linewidth=0.8)
    angular_axis.set_ylabel("Angular velocity (deg/s)", color="#CC79A7")
    angular_axis.tick_params(axis="y", labelcolor="#CC79A7")
    velocity_axis.legend(
        (linear_line, angular_line),
        ("Linear velocity", "Angular velocity"),
        loc="upper right",
    )

    for axis in axes:
        axis.axvline(
            rover_end_sec,
            color="#777777",
            linestyle=":",
            linewidth=1.2,
        )
        axis.set_xlim(0.0, rover_end_sec + 0.25)

    figure.tight_layout()
    png_path = output_dir / f"{stem}.png"
    pdf_path = output_dir / f"{stem}.pdf"
    figure.savefig(png_path, dpi=300, bbox_inches="tight", facecolor="white")
    figure.savefig(pdf_path, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    return png_path, pdf_path


def write_metrics(metrics, output_dir, stem):
    path = output_dir / f"{stem}.csv"
    with path.open("x", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("metric", "value", "unit"))
        writer.writerows(metrics)
    return path


def write_aligned_samples(points, output_dir, stem):
    path = output_dir / f"{stem}_samples.csv"
    fields = (
        "sample_index",
        "elapsed_sec",
        "heading_deg",
        "reference_deg",
        "gimbal_command_ros_deg",
        "observation_deg",
        "error_deg",
        "absolute_error_deg",
        "linear_velocity_mps",
        "angular_velocity_deg_s",
    )
    with path.open("x", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for point in points:
            writer.writerow({field: point[field] for field in fields})
    return path


def build_parser():
    parser = argparse.ArgumentParser(
        description="Rx 방향 오차와 로버 선속도·각속도를 함께 분석합니다."
    )
    parser.add_argument("--rover-csv", type=Path, default=DEFAULT_ROVER_CSV)
    parser.add_argument("--rx-csv", type=Path, default=DEFAULT_RX_CSV)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--font-path", type=Path)
    return parser


def main():
    args = build_parser().parse_args()
    rover_rows = read_rover_rows(args.rover_csv)
    rover_end_sec = rover_rows[-1]["elapsed_sec"]
    rx_rows = read_rx_rows(args.rx_csv, rover_end_sec)
    points = build_points(rover_rows, rx_rows)
    metrics = calculate_metrics(points)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = choose_unused_stem(args.output_dir, DEFAULT_STEM)
    plt = configure_matplotlib(args.font_path)
    generated = [
        *plot_analysis(plt, points, rover_end_sec, args.output_dir, stem),
        write_metrics(metrics, args.output_dir, stem),
        write_aligned_samples(points, args.output_dir, stem),
    ]
    for path in generated:
        print(path)


if __name__ == "__main__":
    main()
