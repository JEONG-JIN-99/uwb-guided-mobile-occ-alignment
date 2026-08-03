#!/usr/bin/env python3
"""로버 기준 방향과 Rx 짐벌 명령의 동적 정렬 정확도를 시각화한다."""

import argparse
import csv
import math
import os
import statistics
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ROVER_CSV = PROJECT_ROOT / "rover_log_20260728_152232.csv"
DEFAULT_RX_CSV = PROJECT_ROOT / "result/dynamic_tracking/3m001/rx.csv"
DEFAULT_OUTPUT_DIR = (
    PROJECT_ROOT / "result/dynamic_tracking/3m001/alignment_accuracy"
)


def wrap_angle(angle_deg):
    """각도를 [-180, 180) 범위로 정규화한다."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def read_rover_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for row in csv.DictReader(stream):
            rows.append(
                {
                    "elapsed_sec": float(row["elapsed_sec"]),
                    "heading_deg": float(row["heading_deg"]),
                    "rx_to_tx_deg": float(row["rx_to_tx_deg"]),
                }
            )
    if not rows:
        raise ValueError(f"로버 로그가 비어 있습니다: {path}")
    return rows


def read_rx_rows(path, min_elapsed_sec, max_elapsed_sec):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for row in csv.DictReader(stream):
            if not row["sample_index"] or not row["command_elapsed_s"]:
                continue
            sample_index = int(row["sample_index"])
            elapsed_sec = float(row["command_elapsed_s"])
            if (
                sample_index < 1
                or elapsed_sec < min_elapsed_sec
                or elapsed_sec > max_elapsed_sec
            ):
                continue
            if not row["gimbal_command_ros_deg"]:
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
        raise ValueError(f"분석 가능한 Rx 명령이 없습니다: {path}")
    return rows


def interpolate_angle(rows, elapsed_sec, field):
    """인접 로버 표본 사이를 최단 각도 방향으로 선형 보간한다."""
    if elapsed_sec <= rows[0]["elapsed_sec"]:
        return rows[0][field]
    if elapsed_sec >= rows[-1]["elapsed_sec"]:
        return rows[-1][field]

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
    delta = wrap_angle(after[field] - before[field])
    return wrap_angle(before[field] + ratio * delta)


def build_alignment_points(rover_rows, rx_rows):
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
            }
        )
    return points


def percentile_nearest_rank(values, probability):
    ordered = sorted(values)
    index = math.ceil(probability * len(ordered)) - 1
    return ordered[index]


def calculate_metrics(points):
    signed_errors = [point["error_deg"] for point in points]
    absolute_errors = [abs(value) for value in signed_errors]
    count = len(points)
    return [
        ("Number of alignment commands", f"{count}"),
        ("Mean signed error (Bias)", f"{statistics.mean(signed_errors):.2f}°"),
        ("Mean absolute error (MAE)", f"{statistics.mean(absolute_errors):.2f}°"),
        (
            "Root mean square error (RMSE)",
            f"{math.sqrt(statistics.mean(v * v for v in signed_errors)):.2f}°",
        ),
        (
            "Median absolute error",
            f"{statistics.median(absolute_errors):.2f}°",
        ),
        (
            "90th percentile absolute error",
            f"{percentile_nearest_rank(absolute_errors, 0.90):.2f}°",
        ),
        (
            "95th percentile absolute error",
            f"{percentile_nearest_rank(absolute_errors, 0.95):.2f}°",
        ),
        ("Maximum absolute error", f"{max(absolute_errors):.2f}°"),
        (
            "Absolute error within 5°",
            (
                f"{sum(value <= 5.0 for value in absolute_errors)}/{count} "
                f"({100.0 * sum(value <= 5.0 for value in absolute_errors) / count:.2f}%)"
            ),
        ),
        (
            "Absolute error within 10°",
            (
                f"{sum(value <= 10.0 for value in absolute_errors)}/{count} "
                f"({100.0 * sum(value <= 10.0 for value in absolute_errors) / count:.2f}%)"
            ),
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
        raise RuntimeError(
            "그림 글꼴을 찾지 못했습니다. --font-path로 지정하십시오."
        )

    font_manager.fontManager.addfont(selected)
    font_name = font_manager.FontProperties(fname=selected).get_name()
    plt.rcParams.update(
        {
            "font.family": font_name,
            "font.size": 12,
            "axes.labelsize": 13,
            "axes.titlesize": 14,
            "legend.fontsize": 11,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "axes.unicode_minus": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    return plt


def save_figure(figure, output_dir, stem):
    png_path = output_dir / f"{stem}.png"
    pdf_path = output_dir / f"{stem}.pdf"
    figure.savefig(png_path, dpi=300, bbox_inches="tight", facecolor="white")
    figure.savefig(pdf_path, bbox_inches="tight", facecolor="white")
    return png_path, pdf_path


def plot_absolute_directions(plt, points, rover_end_sec, output_dir):
    times = [point["elapsed_sec"] for point in points]
    references = [point["reference_deg"] for point in points]
    observations = [point["observation_deg"] for point in points]

    figure, axis = plt.subplots(figsize=(10.5, 5.8))
    axis.plot(
        times,
        references,
        color="#222222",
        linewidth=2.4,
        label="Reference direction (Rx-to-Tx)",
    )
    axis.plot(
        times,
        observations,
        color="#0072B2",
        linewidth=1.8,
        marker="o",
        markersize=2.6,
        markevery=4,
        label="Commanded absolute viewing direction",
    )
    axis.axvline(
        rover_end_sec,
        color="#777777",
        linestyle=":",
        linewidth=1.4,
        label=f"End of rover log ({rover_end_sec:.3f} s)",
    )
    axis.set(
        title="Absolute Direction Tracking of the Rx Gimbal",
        xlabel="Elapsed time (s)",
        ylabel="Absolute ROS direction (°)",
        xlim=(0.0, rover_end_sec + 0.25),
    )
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()
    paths = save_figure(figure, output_dir, "01_absolute_direction_tracking")
    plt.close(figure)
    return paths


def plot_alignment_errors(plt, points, rover_end_sec, output_dir):
    times = [point["elapsed_sec"] for point in points]
    errors = [point["error_deg"] for point in points]
    bias = statistics.mean(errors)

    figure, axis = plt.subplots(figsize=(10.5, 5.8))
    axis.axhspan(-10, 10, color="#F0E442", alpha=0.16, label="±10° band")
    axis.axhspan(-5, 5, color="#009E73", alpha=0.18, label="±5° band")
    axis.axhline(0, color="#222222", linewidth=1.2)
    axis.axhline(
        bias,
        color="#CC79A7",
        linestyle="--",
        linewidth=1.8,
        label=f"Mean bias ({bias:.2f}°)",
    )
    axis.plot(
        times,
        errors,
        color="#D55E00",
        linewidth=1.7,
        marker="o",
        markersize=3.0,
        label="Directional error",
    )
    axis.axvline(
        rover_end_sec,
        color="#777777",
        linestyle=":",
        linewidth=1.4,
        label=f"End of rover log ({rover_end_sec:.3f} s)",
    )
    limit = max(12.0, math.ceil(max(abs(value) for value in errors) / 5) * 5)
    axis.set(
        title="Directional Alignment Error of the Rx Gimbal Command",
        xlabel="Elapsed time (s)",
        ylabel="Directional error (°)",
        xlim=(0.0, rover_end_sec + 0.25),
        ylim=(-limit, limit),
    )
    axis.grid(alpha=0.25)
    axis.legend(loc="best", ncol=2)
    figure.tight_layout()
    paths = save_figure(figure, output_dir, "02_alignment_error_timeseries")
    plt.close(figure)
    return paths


def plot_metrics_table(plt, metrics, output_dir):
    figure, axis = plt.subplots(figsize=(8.2, 5.8))
    axis.axis("off")
    table = axis.table(
        cellText=metrics,
        colLabels=("Alignment accuracy metric", "Result"),
        cellLoc="left",
        colLoc="center",
        colWidths=(0.66, 0.34),
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(11.5)
    table.scale(1.0, 1.55)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("#A0A0A0")
        cell.set_linewidth(0.6)
        if row == 0:
            cell.set_facecolor("#D9EAF7")
            cell.set_text_props(weight="bold", ha="center")
        elif row % 2 == 0:
            cell.set_facecolor("#F5F7F9")
        if column == 1 and row > 0:
            cell.set_text_props(ha="right")
    axis.set_title("Dynamic Rx Gimbal Alignment Accuracy", pad=12, weight="bold")
    figure.tight_layout()
    paths = save_figure(figure, output_dir, "03_alignment_metrics_table")
    plt.close(figure)
    return paths


def write_metrics_csv(metrics, output_dir):
    path = output_dir / "03_alignment_metrics_table.csv"
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("Alignment accuracy metric", "Result"))
        writer.writerows(metrics)
    return path


def build_parser():
    parser = argparse.ArgumentParser(
        description="Rx 동적 짐벌 정렬 정확도 그래프와 표를 생성합니다."
    )
    parser.add_argument("--rover-csv", type=Path, default=DEFAULT_ROVER_CSV)
    parser.add_argument("--rx-csv", type=Path, default=DEFAULT_RX_CSV)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--font-path", type=Path)
    return parser


def main():
    args = build_parser().parse_args()
    rover_rows = read_rover_rows(args.rover_csv)
    rover_start_sec = rover_rows[0]["elapsed_sec"]
    rover_end_sec = rover_rows[-1]["elapsed_sec"]
    rx_rows = read_rx_rows(args.rx_csv, rover_start_sec, rover_end_sec)
    points = build_alignment_points(rover_rows, rx_rows)
    metrics = calculate_metrics(points)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    plt = configure_matplotlib(args.font_path)
    generated = [
        *plot_absolute_directions(
            plt,
            points,
            rover_end_sec,
            args.output_dir,
        ),
        *plot_alignment_errors(
            plt,
            points,
            rover_end_sec,
            args.output_dir,
        ),
        *plot_metrics_table(plt, metrics, args.output_dir),
        write_metrics_csv(metrics, args.output_dir),
    ]
    for path in generated:
        print(path)


if __name__ == "__main__":
    main()
