#!/usr/bin/env python3
"""랜덤 경로 5개 실행 중 상위 3개를 선정해 논문용 결과를 합산한다."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import analyze_individual_experiments as common
import analyze_random_path_experiments as random_analysis


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RESULT_ROOT = PROJECT_ROOT / "result" / "dynamic_tracking"
EXPERIMENTS = tuple(f"random{index:02d}" for index in range(1, 6))
STATUS_KEYS = (
    "success",
    "color_not_detected",
    "camera_frame_timeout",
    "uwb_unavailable",
    "other_error",
)


@dataclass
class RunResult:
    experiment: str
    rover_start_s: float
    rover_end_s: float
    recognition: dict
    recognition_source: str
    rx_points: list[dict]
    tx_points: list[dict]

    @property
    def valid_rx_points(self):
        return [point for point in self.rx_points if point["alignable"]]

    @property
    def valid_tx_points(self):
        return [point for point in self.tx_points if point["alignable"]]

    @property
    def rx_mae(self):
        return statistics.fmean(
            abs(point["error_deg"]) for point in self.valid_rx_points
        )

    @property
    def tx_mae(self):
        return statistics.fmean(
            abs(point["error_deg"]) for point in self.valid_tx_points
        )

    @property
    def mean_device_mae(self):
        return (self.rx_mae + self.tx_mae) / 2.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-root", type=Path, default=DEFAULT_RESULT_ROOT)
    return parser.parse_args()


def leading_count(value):
    text = str(value).strip()
    if not text:
        raise ValueError("빈 인식 건수")
    return int(text.split()[0])


def final_recognition_counts(directory, automatic_groups):
    """수동 검수 표가 있으면 논문 확정값을 우선 사용한다."""
    path = directory / "red_recognition_analysis_table.csv"
    if not path.is_file():
        return dict(automatic_groups["all"]), "rx.csv automatic status"
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    row = next(
        (
            item
            for item in rows
            if str(item.get("Analysis Scope", "")).strip() == "Entire Interval"
        ),
        None,
    )
    if row is None:
        raise ValueError(f"{path}: Entire Interval 행 없음")
    counts = {
        "success": leading_count(row["Detection Success"]),
        "color_not_detected": leading_count(row["Color Not Detected"]),
        "camera_frame_timeout": leading_count(row["Camera Frame Timeout"]),
        "uwb_unavailable": leading_count(row["UWB Unavailable"]),
        "other_error": leading_count(row["Other Errors"]),
    }
    counts["total"] = sum(counts[key] for key in STATUS_KEYS)
    return counts, path.name


def load_run(paths):
    directory, rover_path, rx_path, tx_path = paths
    rover_rows = common.read_rover(rover_path)
    tx_mode = random_analysis.detect_tx_coordinate_mode(rover_rows)
    rx_required = {
        "sample_index",
        "trajectory_mode",
        "distance_m",
        "scheduled_elapsed_s",
        "command_elapsed_s",
        "gimbal_command_ros_deg",
        "servo_clipped",
        "target_color",
        "color_success",
        "status",
    }
    tx_required = {
        "sample_index",
        "trajectory_mode",
        "distance_m",
        "scheduled_elapsed_s",
        "command_elapsed_s",
        "gimbal_command_ros_deg",
        "servo_clipped",
        "status",
    }
    rx_run = random_analysis.read_latest_device_run(rx_path, rx_required)
    tx_run = random_analysis.read_latest_device_run(tx_path, tx_required)
    automatic_recognition, _ = random_analysis.recognition_metrics(
        rx_run, rover_rows
    )
    recognition, recognition_source = final_recognition_counts(
        directory, automatic_recognition
    )
    rx_points = random_analysis.direction_commands(rx_run, rover_rows, "rx")
    tx_points = random_analysis.direction_commands(
        tx_run, rover_rows, "tx", tx_mode
    )
    if not rx_points or not tx_points:
        raise ValueError(f"{directory.name}: 유효 정렬 표본 없음")
    return RunResult(
        experiment=directory.name,
        rover_start_s=rover_rows[0]["elapsed_sec"],
        rover_end_s=rover_rows[-1]["elapsed_sec"],
        recognition=recognition,
        recognition_source=recognition_source,
        rx_points=rx_points,
        tx_points=tx_points,
    )


def select_best_three(runs):
    """거리별 상위 3회 코드와 동일하게 평균 Rx·Tx MAE만 사용한다."""
    ranked = sorted(
        runs,
        key=lambda run: (run.mean_device_mae, run.experiment),
    )
    return ranked[:3], ranked


def interpolate_value(times, values, target):
    high = bisect.bisect_right(times, target)
    if high == 0:
        return values[0]
    if high == len(times):
        return values[-1]
    low = high - 1
    width = times[high] - times[low]
    if width <= 0:
        return values[low]
    ratio = (target - times[low]) / width
    return values[low] + ratio * (values[high] - values[low])


def align_unwrapped(reference, observed):
    return common.align_unwrapped(reference, observed)


def run_series(run):
    rx_ideal, rx_actual = align_unwrapped(
        [point["ideal_deg"] for point in run.rx_points],
        [point["actual_deg"] for point in run.rx_points],
    )

    # Tx→Rx는 Rx→Tx와 180° 반대 방향이다. 한 그래프에 겹치기 위해
    # Tx ideal/command 모두 180° 회전해 Rx→Tx 공통 방향 좌표계로 바꾼다.
    tx_ideal_common = [
        common.wrap_angle(point["ideal_deg"] + 180.0)
        for point in run.tx_points
    ]
    tx_actual_common = [
        common.wrap_angle(point["actual_deg"] + 180.0)
        for point in run.tx_points
    ]
    _, tx_actual = align_unwrapped(tx_ideal_common, tx_actual_common)

    return {
        "rx_times": [
            point["elapsed_sec"] - run.rover_start_s
            for point in run.rx_points
        ],
        "tx_times": [
            point["elapsed_sec"] - run.rover_start_s
            for point in run.tx_points
        ],
        "ideal": rx_ideal,
        "rx": rx_actual,
        "tx": tx_actual,
    }


def mean_time_series(selected, step_s=0.2):
    series = [run_series(run) for run in selected]
    common_start = max(
        max(item["rx_times"][0], item["tx_times"][0]) for item in series
    )
    common_end = min(
        min(item["rx_times"][-1], item["tx_times"][-1]) for item in series
    )
    if common_end <= common_start:
        raise ValueError("상위 세 랜덤 실행의 공통 시간 구간이 없습니다")
    count = math.floor((common_end - common_start) / step_s) + 1
    times = [common_start + index * step_s for index in range(count)]

    def averaged(time_key, value_key, elapsed):
        return statistics.fmean(
            interpolate_value(item[time_key], item[value_key], elapsed)
            for item in series
        )

    ideal = [averaged("rx_times", "ideal", elapsed) for elapsed in times]
    rx = [averaged("rx_times", "rx", elapsed) for elapsed in times]
    tx = [averaged("tx_times", "tx", elapsed) for elapsed in times]
    return {
        "elapsed_s": [elapsed - common_start for elapsed in times],
        "ideal_deg": ideal,
        "rx_deg": rx,
        "tx_deg": tx,
        "common_start_s": common_start,
        "common_end_s": common_end,
        "step_s": step_s,
        "mean_trajectory_rx_mae_deg": statistics.fmean(
            abs(common.wrap_angle(actual - target))
            for actual, target in zip(rx, ideal)
        ),
        "mean_trajectory_tx_mae_deg": statistics.fmean(
            abs(common.wrap_angle(actual - target))
            for actual, target in zip(tx, ideal)
        ),
    }


def pooled_summary(selected, mean_series):
    counts = Counter()
    for run in selected:
        for key in STATUS_KEYS:
            counts[key] += run.recognition[key]
    total = sum(counts[key] for key in STATUS_KEYS)
    frames = counts["success"] + counts["color_not_detected"]
    rx_errors = [
        abs(point["error_deg"])
        for run in selected
        for point in run.valid_rx_points
    ]
    tx_errors = [
        abs(point["error_deg"])
        for run in selected
        for point in run.valid_tx_points
    ]
    return {
        "Selected experiments": ";".join(
            run.experiment for run in sorted(selected, key=lambda run: run.experiment)
        ),
        "Total cycles": total,
        "Successful detections": counts["success"],
        "Color not detected": counts["color_not_detected"],
        "Camera timeouts": counts["camera_frame_timeout"],
        "UWB unavailable": counts["uwb_unavailable"],
        "Other errors": counts["other_error"],
        "Overall detection rate (%)": 100.0 * counts["success"] / total,
        "Conditional detection rate (%)": (
            100.0 * counts["success"] / frames if frames else None
        ),
        "Pooled Rx samples": len(rx_errors),
        "Pooled Rx MAE (deg)": statistics.fmean(rx_errors),
        "Pooled Tx samples": len(tx_errors),
        "Pooled Tx MAE (deg)": statistics.fmean(tx_errors),
        "Mean trajectory samples": len(mean_series["elapsed_s"]),
        "Common duration (s)": mean_series["elapsed_s"][-1],
        "Time step (s)": mean_series["step_s"],
        "Mean trajectory Rx MAE (deg)": mean_series[
            "mean_trajectory_rx_mae_deg"
        ],
        "Mean trajectory Tx MAE (deg)": mean_series[
            "mean_trajectory_tx_mae_deg"
        ],
    }


def write_csv(path, rows, fieldnames):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_manifest(path, ranked):
    selected_names = {run.experiment for run in ranked[:3]}
    rows = []
    for rank, run in enumerate(ranked, start=1):
        recognition = run.recognition
        rows.append(
            {
                "experiment": run.experiment,
                "selected": int(run.experiment in selected_names),
                "rank": rank,
                "overall_detection_rate_pct": (
                    100.0 * recognition["success"] / recognition["total"]
                ),
                "rx_mae_deg": run.rx_mae,
                "tx_mae_deg": run.tx_mae,
                "mean_rx_tx_mae_deg": run.mean_device_mae,
                "selection_rule": "mean_rx_tx_mae_asc_only",
                "recognition_source": run.recognition_source,
            }
        )
    write_csv(path, rows, tuple(rows[0]))


def write_mean_trajectory(path, series):
    rows = []
    for elapsed, ideal, rx, tx in zip(
        series["elapsed_s"],
        series["ideal_deg"],
        series["rx_deg"],
        series["tx_deg"],
    ):
        rows.append(
            {
                "elapsed_s": elapsed,
                "mean_ideal_direction_deg": ideal,
                "mean_rx_alignment_direction_deg": rx,
                "mean_tx_alignment_direction_deg": tx,
                "mean_rx_error_deg": common.wrap_angle(rx - ideal),
                "mean_tx_error_deg": common.wrap_angle(tx - ideal),
                "averaged_run_count": 3,
            }
        )
    write_csv(path, rows, tuple(rows[0]))


def plot_pooled(plt, summary, series, output_root):
    figure, axis = plt.subplots(
        figsize=(10.5, 1940.0 / 300.0),
        constrained_layout=True,
    )
    times = series["elapsed_s"]
    axis.plot(
        times,
        series["ideal_deg"],
        color="#000000",
        linestyle="--",
        linewidth=2.8,
        label="Mean ideal direction",
        zorder=2,
    )
    axis.plot(
        times,
        series["rx_deg"],
        color="#FF0000",
        linewidth=2.0,
        marker="o",
        markersize=3.2,
        markevery=8,
        label=(
            "Mean Rx direction "
            f"(MAE = {series['mean_trajectory_rx_mae_deg']:.2f}°)"
        ),
        zorder=4,
    )
    axis.plot(
        times,
        series["tx_deg"],
        color="#0000FF",
        linestyle="-.",
        linewidth=2.0,
        marker="s",
        markersize=3.0,
        markevery=8,
        label=(
            "Mean Tx direction "
            f"(MAE = {series['mean_trajectory_tx_mae_deg']:.2f}°)"
        ),
        zorder=5,
    )
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("Direction (deg)")
    axis.grid(True, alpha=0.28)
    axis.legend(
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        borderaxespad=0,
        ncol=1,
    )
    png = output_root / "alignment_tracking_random_best3.png"
    pdf = output_root / "alignment_tracking_random_best3.pdf"
    figure.savefig(png, dpi=300)
    figure.savefig(pdf)
    plt.close(figure)
    return png, pdf


def write_report(path, ranked, summary, series):
    selected = ranked[:3]
    excluded = ranked[3:]
    lines = [
        "# Random Dynamic Tracking 상위 3회 합산 분석",
        "",
        "## 선정 및 합산 방법",
        "",
        "random01–05를 같은 평가법으로 재계산하고, 거리별 상위 3회 분석과 "
        "동일하게 인식률은 선정에 사용하지 않았다. `(Rx MAE + Tx MAE) / 2`가 "
        "낮은 순서로 세 실행을 선정했다.",
        "",
        "선정된 세 실행은 로버 시작시각 기준 상대시간으로 맞추고, 세 실행 모두 "
        "데이터가 존재하는 공통 구간을 0.2초 간격으로 보간했다. Tx→Rx 방향은 "
        "180° 회전하여 Rx→Tx와 동일한 공통 방향 좌표계로 변환한 뒤 평균했다.",
        "",
        "## 선정 결과",
        "",
        f"- Selected: {', '.join(run.experiment for run in selected)}",
        f"- Excluded: {', '.join(run.experiment for run in excluded)}",
        "",
        "| Rank | Experiment | Overall rate | Rx MAE | Tx MAE | Mean Rx/Tx MAE |",
        "|---:|---|---:|---:|---:|---:|",
    ]
    for rank, run in enumerate(ranked, start=1):
        recognition = run.recognition
        rate = 100.0 * recognition["success"] / recognition["total"]
        lines.append(
            f"| {rank} | {run.experiment} | {rate:.2f}% | "
            f"{run.rx_mae:.2f}° | {run.tx_mae:.2f}° | "
            f"{run.mean_device_mae:.2f}° |"
        )
    lines += [
        "",
        "## 합산 지표",
        "",
        "| Runs | N | Success | Color misses | Camera timeouts | Overall rate | Conditional rate | Pooled Rx n/MAE | Pooled Tx n/MAE | Mean time n | Mean-trajectory Rx MAE | Mean-trajectory Tx MAE |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| 3 | {summary['Total cycles']} | {summary['Successful detections']} | "
        f"{summary['Color not detected']} | {summary['Camera timeouts']} | "
        f"{summary['Overall detection rate (%)']:.2f}% | "
        f"{summary['Conditional detection rate (%)']:.2f}% | "
        f"{summary['Pooled Rx samples']}/{summary['Pooled Rx MAE (deg)']:.2f}° | "
        f"{summary['Pooled Tx samples']}/{summary['Pooled Tx MAE (deg)']:.2f}° | "
        f"{summary['Mean trajectory samples']} | "
        f"{summary['Mean trajectory Rx MAE (deg)']:.2f}° | "
        f"{summary['Mean trajectory Tx MAE (deg)']:.2f}° |",
        "",
        "## Rx·Tx 통합 그래프",
        "",
        "![Random best-three pooled tracking](alignment_tracking_random_best3.png)",
        "",
        "## 해석 시 주의사항",
        "",
        "- 상위 실행 선택은 선택 편향을 유발할 수 있으므로 선정 규칙과 제외 실행을 함께 공개한다.",
        "- 인식률은 선정 기준이 아니다. 수동 검수 확정표가 있는 실행은 해당 표를 우선하고, 나머지는 Rx CSV 상태의 원시 건수를 합산했다.",
        "- 그래프 MAE는 평균 방향 시계열과 평균 Ideal 시계열 사이의 MAE다. 원시 오차를 합친 pooled MAE는 표에 별도로 기록했다.",
        "- Rx·Tx 방향은 엔코더 실측 자세가 아니라 실제 적용된 짐벌 명령 기반 방향이다.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main():
    args = parse_args()
    result_root = args.result_root.resolve()
    complete, skipped = random_analysis.discover_experiments(
        result_root, list(EXPERIMENTS)
    )
    if skipped:
        details = "; ".join(f"{name}: {reason}" for name, reason in skipped)
        raise ValueError(f"랜덤 실험 검증 실패: {details}")
    lookup = {paths[0].name: paths for paths in complete}
    missing = [name for name in EXPERIMENTS if name not in lookup]
    if missing:
        raise ValueError(f"랜덤 실험 누락: {missing}")

    runs = [load_run(lookup[name]) for name in EXPERIMENTS]
    selected, ranked = select_best_three(runs)
    series = mean_time_series(selected)
    summary = pooled_summary(selected, series)

    write_manifest(
        result_root / "random_tracking_best3_selection_manifest.csv", ranked
    )
    write_csv(
        result_root / "random_tracking_summary_best3.csv",
        [summary],
        tuple(summary),
    )
    write_mean_trajectory(
        result_root / "random_tracking_mean_trajectory_best3.csv", series
    )

    plt = common.configure_matplotlib()
    plt.rcParams.update(
        {
            # The random best-three plot uses 1.5x the 22 pt text used by
            # the distance-based best-three plots: 11 pt * 2 * 1.5 = 33 pt.
            key: value * 3.0
            for key, value in {
                "font.size": 11.0,
                "axes.labelsize": 11.0,
                "xtick.labelsize": 11.0,
                "ytick.labelsize": 11.0,
                "legend.fontsize": 11.0,
            }.items()
        }
    )
    plot_pooled(plt, summary, series, result_root)
    write_report(
        result_root / "random_tracking_best3_analysis.md",
        ranked,
        summary,
        series,
    )
    print(
        f"selected={summary['Selected experiments']} "
        f"N={summary['Total cycles']} "
        f"overall={summary['Overall detection rate (%)']:.2f}% "
        f"pooled_Rx_MAE={summary['Pooled Rx MAE (deg)']:.2f} "
        f"pooled_Tx_MAE={summary['Pooled Tx MAE (deg)']:.2f} "
        f"mean_Rx_MAE={summary['Mean trajectory Rx MAE (deg)']:.2f} "
        f"mean_Tx_MAE={summary['Mean trajectory Tx MAE (deg)']:.2f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
