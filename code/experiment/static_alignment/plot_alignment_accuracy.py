#!/usr/bin/env python3
"""1m·2m·3m 정적 정렬 명령의 절대 잔여오차 비교 그림을 생성한다."""

import argparse
import csv
import math
import os
import statistics
import tempfile
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
REFERENCE_ANGLE_DEG = 0.0
INITIAL_BIN_EDGES = (0, 10, 20, 30, 40, 50, 60)

SCATTER_NAME = "alignment_accuracy_initial_angle_scatter.png"
MEAN_BY_INITIAL_NAME = "alignment_accuracy_initial_angle_mean.png"
VIOLIN_NAME = "alignment_accuracy_distance_violin.png"

CONDITION_LABELS = {
    "no_settle": "No stabilization",
    "settle_0.5s": "0.5 s stabilization",
}
CONDITION_COLORS = {
    "no_settle": "#e68613",
    "settle_0.5s": "#2474b5",
}

DEFAULT_DATASETS = (
    (
        1,
        "no_settle",
        "result/static_alignment/run_20260727_220119/"
        "static_alignment_results.csv",
    ),
    (
        1,
        "settle_0.5s",
        "result/static_alignment_after_settle/run_20260727_220727/"
        "static_alignment_results.csv",
    ),
    (
        2,
        "no_settle",
        "result/static_alignment/run_20260727_213725/"
        "static_alignment_results.csv",
    ),
    (
        2,
        "settle_0.5s",
        "result/static_alignment_after_settle/run_20260727_214708/"
        "static_alignment_results.csv",
    ),
    (
        3,
        "no_settle",
        "result/static_alignment/run_20260727_210016/"
        "static_alignment_results.csv",
    ),
    (
        3,
        "settle_0.5s",
        "result/static_alignment_after_settle/run_20260727_205444/"
        "static_alignment_results.csv",
    ),
)


@dataclass(frozen=True)
class AlignmentPoint:
    distance_m: int
    condition: str
    attempt: int
    initial_angle_deg: float
    command_angle_deg: float

    @property
    def absolute_error_deg(self):
        return abs(self.command_angle_deg - REFERENCE_ANGLE_DEG)


def read_dataset(csv_path, distance_m, condition):
    with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {
            "attempt",
            "distance_m",
            "initial_gimbal_ros_deg",
            "gimbal_command_ros_deg",
        }
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                f"{csv_path}에 필요한 열이 없습니다: {sorted(missing)}"
            )

        points = []
        for row in reader:
            raw_command = row["gimbal_command_ros_deg"].strip()
            if not raw_command:
                continue
            recorded_distance = float(row["distance_m"])
            if not math.isclose(recorded_distance, float(distance_m)):
                raise ValueError(
                    f"{csv_path}의 distance_m={recorded_distance:g}가 "
                    f"예상값 {distance_m:g}와 다릅니다."
                )
            point = AlignmentPoint(
                distance_m=distance_m,
                condition=condition,
                attempt=int(row["attempt"]),
                initial_angle_deg=float(row["initial_gimbal_ros_deg"]),
                command_angle_deg=float(raw_command),
            )
            if not all(
                math.isfinite(value)
                for value in (
                    point.initial_angle_deg,
                    point.command_angle_deg,
                )
            ):
                continue
            points.append(point)
    if not points:
        raise ValueError(f"{csv_path}에서 유효한 정렬 명령을 찾지 못했습니다.")
    return points


def load_default_points(project_root):
    points = []
    for distance_m, condition, relative_path in DEFAULT_DATASETS:
        points.extend(
            read_dataset(
                project_root / relative_path,
                distance_m,
                condition,
            )
        )
    return points


def select(points, distance_m, condition):
    return [
        point
        for point in points
        if point.distance_m == distance_m and point.condition == condition
    ]


def values_by_initial_bin(points):
    groups = []
    for lower, upper in zip(INITIAL_BIN_EDGES, INITIAL_BIN_EDGES[1:]):
        groups.append(
            [
                point.absolute_error_deg
                for point in points
                if lower <= abs(point.initial_angle_deg) < upper
            ]
        )
    return groups


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
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 11,
            "legend.fontsize": 9,
            "figure.titlesize": 14,
        }
    )
    return plt


def add_figure_note(figure):
    figure.text(
        0.5,
        0.01,
        "Error definition: |gimbal command angle - ideal 0 deg|. "
        "Conditions are shown separately and are not pooled.",
        ha="center",
        va="bottom",
        fontsize=9,
        color="#444444",
    )


def plot_initial_angle_scatter(plt, points, output_path):
    figure, axes = plt.subplots(
        1,
        3,
        figsize=(15, 5.2),
        sharex=True,
        sharey=True,
        constrained_layout=False,
    )
    marker_by_condition = {
        "no_settle": "o",
        "settle_0.5s": "x",
    }
    offset_by_condition = {
        "no_settle": -0.22,
        "settle_0.5s": 0.22,
    }

    for axis, distance_m in zip(axes, (1, 2, 3)):
        for condition in CONDITION_LABELS:
            group = select(points, distance_m, condition)
            axis.scatter(
                [
                    point.initial_angle_deg + offset_by_condition[condition]
                    for point in group
                ],
                [point.absolute_error_deg for point in group],
                s=28,
                alpha=0.7,
                color=CONDITION_COLORS[condition],
                marker=marker_by_condition[condition],
                linewidths=1.0,
                label=CONDITION_LABELS[condition],
            )
        axis.set_title(f"Distance = {distance_m} m")
        axis.set_xlabel("Initial angle (deg)")
        axis.set_xlim(-53, 52)
        axis.set_xticks((-50, -25, 0, 25, 50))
        axis.grid(alpha=0.25)
    axes[0].set_ylabel("Absolute residual command error (deg)")
    axes[-1].legend(loc="upper right")
    figure.suptitle("Initial Angle vs. Absolute Residual Command Error")
    add_figure_note(figure)
    figure.subplots_adjust(left=0.065, right=0.985, top=0.86, bottom=0.17, wspace=0.08)
    figure.savefig(output_path, dpi=200)
    plt.close(figure)


def mean_and_ci95(values):
    mean = statistics.fmean(values)
    if len(values) < 2:
        return mean, 0.0
    standard_error = statistics.stdev(values) / math.sqrt(len(values))
    return mean, 1.96 * standard_error


def plot_initial_angle_mean(plt, points, output_path):
    figure, axes = plt.subplots(
        1,
        3,
        figsize=(16, 5.4),
        sharey=True,
        constrained_layout=False,
    )
    centers = list(range(1, len(INITIAL_BIN_EDGES)))
    offsets = {
        "no_settle": -0.09,
        "settle_0.5s": 0.09,
    }
    markers = {
        "no_settle": "o",
        "settle_0.5s": "s",
    }

    for axis, distance_m in zip(axes, (1, 2, 3)):
        for condition in CONDITION_LABELS:
            group = select(points, distance_m, condition)
            grouped_values = values_by_initial_bin(group)
            means_and_intervals = [
                mean_and_ci95(values) for values in grouped_values
            ]
            means = [value[0] for value in means_and_intervals]
            intervals = [value[1] for value in means_and_intervals]
            positions = [
                center + offsets[condition] for center in centers
            ]
            axis.errorbar(
                positions,
                means,
                yerr=intervals,
                color=CONDITION_COLORS[condition],
                marker=markers[condition],
                markersize=5.5,
                linewidth=1.8,
                capsize=3,
                label=CONDITION_LABELS[condition],
            )
        axis.set_title(f"Distance = {distance_m} m")
        axis.set_xlabel("Absolute initial-angle bin (deg)")
        axis.set_xticks(
            centers,
            [
                f"{lower}-{upper}"
                for lower, upper in zip(
                    INITIAL_BIN_EDGES,
                    INITIAL_BIN_EDGES[1:],
                )
            ],
            rotation=25,
        )
        axis.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel("Mean absolute residual command error (deg)")
    axes[-1].legend(loc="upper left")
    figure.suptitle(
        "Mean Absolute Residual Command Error by Initial-Angle Bin"
    )
    add_figure_note(figure)
    figure.text(
        0.5,
        0.055,
        "Points show arithmetic means; error bars show approximate 95% "
        "confidence intervals.",
        ha="center",
        va="bottom",
        fontsize=9,
        color="#444444",
    )
    figure.subplots_adjust(left=0.06, right=0.985, top=0.86, bottom=0.21, wspace=0.08)
    figure.savefig(output_path, dpi=200)
    plt.close(figure)


def style_violin(parts, color):
    for body in parts["bodies"]:
        body.set_facecolor(color)
        body.set_edgecolor(color)
        body.set_alpha(0.45)
    for key in ("cbars", "cmins", "cmaxes"):
        parts[key].set_color(color)
        parts[key].set_linewidth(1.0)


def plot_distance_violin(plt, points, output_path):
    figure, axis = plt.subplots(figsize=(9.5, 6), constrained_layout=False)
    base_positions = (1, 2, 3)
    offsets = {
        "no_settle": -0.16,
        "settle_0.5s": 0.16,
    }

    for condition in CONDITION_LABELS:
        datasets = [
            [
                point.absolute_error_deg
                for point in select(points, distance_m, condition)
            ]
            for distance_m in base_positions
        ]
        positions = [
            distance_m + offsets[condition] for distance_m in base_positions
        ]
        parts = axis.violinplot(
            datasets,
            positions=positions,
            widths=0.28,
            showmeans=False,
            showmedians=False,
            showextrema=True,
        )
        style_violin(parts, CONDITION_COLORS[condition])
        mean_absolute_errors = [
            statistics.fmean(values) for values in datasets
        ]
        axis.scatter(
            positions,
            mean_absolute_errors,
            color=CONDITION_COLORS[condition],
            edgecolors="white",
            linewidths=0.8,
            s=42,
            zorder=3,
            label=CONDITION_LABELS[condition],
        )

    axis.set_title(
        "Absolute Residual Command Error by Distance (Dots: Mean)"
    )
    axis.set_xlabel("Distance (m)")
    axis.set_ylabel("Absolute residual command error (deg)")
    axis.set_xticks(base_positions, [str(value) for value in base_positions])
    axis.set_xlim(0.55, 3.45)
    axis.grid(axis="y", alpha=0.25)
    axis.legend(loc="upper right")
    add_figure_note(figure)
    figure.subplots_adjust(left=0.11, right=0.97, top=0.89, bottom=0.16)
    figure.savefig(output_path, dpi=200)
    plt.close(figure)


def print_summary(points):
    print(
        "distance_m,condition,count,mae_deg,median_absolute_error_deg,"
        "p95_absolute_error_deg,max_absolute_error_deg"
    )
    for distance_m in (1, 2, 3):
        for condition in CONDITION_LABELS:
            errors = sorted(
                point.absolute_error_deg
                for point in select(points, distance_m, condition)
            )
            p95_position = (len(errors) - 1) * 0.95
            lower = math.floor(p95_position)
            upper = math.ceil(p95_position)
            p95 = (
                errors[lower]
                if lower == upper
                else errors[lower]
                + (errors[upper] - errors[lower])
                * (p95_position - lower)
            )
            print(
                f"{distance_m},{condition},{len(errors)},"
                f"{statistics.fmean(errors):.6f},"
                f"{statistics.median(errors):.6f},"
                f"{p95:.6f},{max(errors):.6f}"
            )


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "1m·2m·3m의 안정화 유무별 절대 잔여 명령오차 비교 그림을 "
            "생성합니다."
        )
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        default=PROJECT_ROOT,
        help="프로젝트 루트 경로",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=PROJECT_ROOT / "docs" / "paper" / "figures",
        help="PNG 출력 폴더",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    project_root = args.project_root.resolve()
    output_dir = args.output_dir.resolve()
    points = load_default_points(project_root)
    expected_count = len(DEFAULT_DATASETS) * 100
    if len(points) != expected_count:
        raise ValueError(
            f"유효 명령 수가 {len(points)}개입니다. 예상값은 "
            f"{expected_count}개입니다."
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    plt = configure_matplotlib()
    plot_initial_angle_scatter(plt, points, output_dir / SCATTER_NAME)
    plot_initial_angle_mean(plt, points, output_dir / MEAN_BY_INITIAL_NAME)
    plot_distance_violin(plt, points, output_dir / VIOLIN_NAME)
    print_summary(points)
    print(f"Saved figures to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
