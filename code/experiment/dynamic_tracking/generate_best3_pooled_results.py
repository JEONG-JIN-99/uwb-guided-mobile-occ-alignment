#!/usr/bin/env python3
"""거리별 5개 원호 실행 중 상위 3개를 선정해 논문용 결과를 합산한다."""

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

import analyze_individual_experiments as analysis


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RESULT_ROOT = PROJECT_ROOT / "result" / "dynamic_tracking"
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
    distance_m: int
    rover_path: Path
    rover_start_s: float
    rover_end_s: float
    recognition: dict
    rx_points: list[dict]
    tx_points: list[dict]

    @property
    def rx_mae(self):
        return statistics.fmean(abs(point["error_deg"]) for point in self.rx_points)

    @property
    def tx_mae(self):
        return statistics.fmean(abs(point["error_deg"]) for point in self.tx_points)

    @property
    def mean_device_mae(self):
        return (self.rx_mae + self.tx_mae) / 2.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-root", type=Path, default=DEFAULT_RESULT_ROOT)
    return parser.parse_args()


def load_run(paths, expected_distance):
    directory, rover_path, rx_path, tx_path = paths
    rover_rows = analysis.read_rover(rover_path)
    t_start = rover_rows[0]["elapsed_sec"]
    t_end = rover_rows[-1]["elapsed_sec"]
    rx_required = {
        "sample_index",
        "trajectory_mode",
        "distance_m",
        "scheduled_elapsed_s",
        "command_elapsed_s",
        "gimbal_command_ros_deg",
        "target_color",
        "color_success",
        "status",
    }
    tx_required = {
        "sample_index",
        "trajectory_mode",
        "distance_m",
        "command_elapsed_s",
        "gimbal_command_ros_deg",
        "status",
    }
    rx_run = analysis.read_device_run(rx_path, rx_required)
    tx_run = analysis.read_device_run(tx_path, tx_required)
    for node, rows in (("Rx", rx_run), ("Tx", tx_run)):
        modes = {
            str(row.get("trajectory_mode", "")).strip()
            for row in rows
            if str(row.get("trajectory_mode", "")).strip()
        }
        distances = {
            analysis.to_float(row.get("distance_m"))
            for row in rows
            if analysis.to_float(row.get("distance_m")) is not None
        }
        if modes != {"fixed_radius_arc"} or distances != {float(expected_distance)}:
            raise ValueError(
                f"{directory.name}: {node} 실행 조건 불일치 "
                f"modes={modes}, distances={distances}"
            )
    common_coordinate_error = max(
        abs(analysis.wrap_angle(row["rx_to_tx_deg"] - row["tx_to_rx_deg"]))
        for row in rover_rows
    )
    if common_coordinate_error > 45.0:
        raise ValueError(f"{directory.name}: Rx/Tx 공통 방향 좌표계 불일치")
    recognition = analysis.recognition_metrics(rx_run, t_start, t_end)
    rx_points = analysis.build_rx_points(
        rover_rows, analysis.command_rows(rx_run, t_start, t_end)
    )
    tx_points = analysis.build_tx_points(
        rover_rows, analysis.command_rows(tx_run, t_start, t_end)
    )
    if not rx_points or not tx_points:
        raise ValueError(f"{directory.name}: 유효 정렬 표본 없음")
    return RunResult(
        directory.name,
        expected_distance,
        rover_path,
        t_start,
        t_end,
        recognition,
        rx_points,
        tx_points,
    )


def select_best_three(runs):
    """Rx·Tx MAE 산술평균 오름차순만으로 선정한다."""
    ranked = sorted(
        runs,
        key=lambda run: (
            run.mean_device_mae,
            run.experiment,
        ),
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


def run_series(run):
    rx_ideal, rx_actual = aligned_series(run.rx_points)
    _, tx_actual = aligned_series(run.tx_points)
    return {
        "rx_times": [
            point["elapsed_sec"] - run.rover_start_s for point in run.rx_points
        ],
        "tx_times": [
            point["elapsed_sec"] - run.rover_start_s for point in run.tx_points
        ],
        "ideal": rx_ideal,
        "rx": rx_actual,
        "tx": tx_actual,
    }


def mean_time_series(selected, step_s=0.2):
    """세 실행의 공통 상대시간에서 방향을 보간한 뒤 산술평균한다."""
    series = [run_series(run) for run in selected]
    common_start = max(
        max(item["rx_times"][0], item["tx_times"][0]) for item in series
    )
    common_end = min(
        min(item["rx_times"][-1], item["tx_times"][-1]) for item in series
    )
    if common_end <= common_start:
        raise ValueError("세 실행의 공통 시간 구간이 없습니다")
    sample_count = math.floor((common_end - common_start) / step_s) + 1
    times = [common_start + index * step_s for index in range(sample_count)]
    ideal_values = []
    rx_values = []
    tx_values = []
    for elapsed in times:
        ideal_values.append(
            statistics.fmean(
                interpolate_value(item["rx_times"], item["ideal"], elapsed)
                for item in series
            )
        )
        rx_values.append(
            statistics.fmean(
                interpolate_value(item["rx_times"], item["rx"], elapsed)
                for item in series
            )
        )
        tx_values.append(
            statistics.fmean(
                interpolate_value(item["tx_times"], item["tx"], elapsed)
                for item in series
            )
        )
    display_times = [elapsed - common_start for elapsed in times]
    rx_errors = [
        abs(analysis.wrap_angle(actual - ideal))
        for actual, ideal in zip(rx_values, ideal_values)
    ]
    tx_errors = [
        abs(analysis.wrap_angle(actual - ideal))
        for actual, ideal in zip(tx_values, ideal_values)
    ]
    return {
        "elapsed_s": display_times,
        "ideal_deg": ideal_values,
        "rx_deg": rx_values,
        "tx_deg": tx_values,
        "common_start_s": common_start,
        "common_end_s": common_end,
        "step_s": step_s,
        "rx_mae_deg": statistics.fmean(rx_errors),
        "tx_mae_deg": statistics.fmean(tx_errors),
    }


def pooled_summary(distance, selected, mean_series):
    counts = Counter()
    for run in selected:
        for key in STATUS_KEYS:
            counts[key] += run.recognition[key]
    total = sum(counts[key] for key in STATUS_KEYS)
    frames = counts["success"] + counts["color_not_detected"]
    return {
        "Distance (m)": distance,
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
        "Mean trajectory samples": len(mean_series["elapsed_s"]),
        "Common duration (s)": mean_series["elapsed_s"][-1],
        "Time step (s)": mean_series["step_s"],
        "Mean Rx MAE (deg)": mean_series["rx_mae_deg"],
        "Mean Tx MAE (deg)": mean_series["tx_mae_deg"],
    }


def write_csv(path, rows, fieldnames):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_selection_manifest(path, ranked_by_distance):
    rows = []
    for distance in (1, 2, 3):
        ranked = ranked_by_distance[distance]
        selected_names = {run.experiment for run in ranked[:3]}
        for rank, run in enumerate(ranked, start=1):
            rows.append(
                {
                    "distance_m": distance,
                    "experiment": run.experiment,
                    "selected": int(run.experiment in selected_names),
                    "rank": rank,
                    "overall_detection_rate_pct": run.recognition["system_rate"],
                    "rx_mae_deg": run.rx_mae,
                    "tx_mae_deg": run.tx_mae,
                    "mean_rx_tx_mae_deg": run.mean_device_mae,
                    "selection_rule": "mean_rx_tx_mae_asc_only",
                }
            )
    write_csv(path, rows, tuple(rows[0]))


def aligned_series(points):
    ideal, actual = analysis.align_unwrapped(
        [point["ideal_deg"] for point in points],
        [point["aligned_deg"] for point in points],
    )
    return ideal, actual


def plot_pooled(plt, distance, selected, summary, mean_series, output_root):
    figure, axis = plt.subplots(
        figsize=(10.5, 1940.0 / 300.0),
        constrained_layout=True,
    )
    times = mean_series["elapsed_s"]
    axis.plot(
        times,
        mean_series["ideal_deg"],
        color="#000000",
        linestyle="--",
        linewidth=2.8,
        label="Mean ideal direction",
        zorder=2,
    )
    axis.plot(
        times,
        mean_series["rx_deg"],
        color="#FF0000",
        linestyle="-",
        linewidth=2.0,
        marker="o",
        markersize=3.2,
        markevery=8,
        label=(
            f"Mean Rx direction (MAE = "
            f"{summary['Mean Rx MAE (deg)']:.2f}°)"
        ),
        zorder=4,
    )
    axis.plot(
        times,
        mean_series["tx_deg"],
        color="#0000FF",
        linestyle="-.",
        linewidth=2.0,
        marker="s",
        markersize=3.0,
        markevery=8,
        label=(
            f"Mean Tx direction (MAE = "
            f"{summary['Mean Tx MAE (deg)']:.2f}°)"
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
    png = output_root / f"alignment_tracking_{distance}m_best3.png"
    pdf = output_root / f"alignment_tracking_{distance}m_best3.pdf"
    figure.savefig(png, dpi=300)
    figure.savefig(pdf)
    plt.close(figure)
    return png, pdf


def write_report(path, summaries, ranked_by_distance):
    lines = [
        "# Dynamic Tracking 거리별 상위 3회 합산 분석",
        "",
        "## 선정 및 합산 방법",
        "",
        "거리별 01–05 실행을 모두 같은 평가법으로 재계산한 뒤, 인식률은 선정에 "
        "사용하지 않고 `(Rx MAE + Tx MAE) / 2`가 낮은 순서로 세 실행을 선정했다.",
        "",
        "선정된 세 실행은 각 로버 시작시각을 기준으로 상대시간을 맞췄다. 세 실행 모두 "
        "데이터가 존재하는 공통 시간 교집합에서 0.2초 간격으로 각 방향을 선형 보간하고, "
        "각 시각의 Ideal·Rx·Tx 방향을 각각 더한 뒤 3으로 나눈 산술평균 시계열을 그렸다.",
        "",
        "## 선정 결과",
        "",
        "| Distance | Selected | Excluded |",
        "|---:|---|---|",
    ]
    for distance in (1, 2, 3):
        ranked = ranked_by_distance[distance]
        lines.append(
            f"| {distance} m | "
            f"{', '.join(run.experiment for run in sorted(ranked[:3], key=lambda r: r.experiment))} | "
            f"{', '.join(run.experiment for run in sorted(ranked[3:], key=lambda r: r.experiment))} |"
        )
    lines += [
        "",
        "## 합산 지표",
        "",
        "| Distance | Runs | N | Success | Color misses | Camera timeouts | Overall rate | Conditional rate | Mean time n | Mean Rx MAE | Mean Tx MAE |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for summary in summaries:
        lines.append(
            f"| {summary['Distance (m)']} m | 3 | {summary['Total cycles']} "
            f"| {summary['Successful detections']} | {summary['Color not detected']} "
            f"| {summary['Camera timeouts']} "
            f"| {summary['Overall detection rate (%)']:.2f}% "
            f"| {summary['Conditional detection rate (%)']:.2f}% "
            f"| {summary['Mean trajectory samples']} "
            f"| {summary['Mean Rx MAE (deg)']:.2f}° "
            f"| {summary['Mean Tx MAE (deg)']:.2f}° |"
        )
    lines += ["", "## 거리별 Rx·Tx 통합 그래프", ""]
    for summary in summaries:
        distance = summary["Distance (m)"]
        lines += [
            f"### {distance} m",
            "",
            f"![{distance} m best-three pooled tracking](alignment_tracking_{distance}m_best3.png)",
            "",
        ]
    lines += [
        "## 해석 시 주의사항",
        "",
        "- 상위 실행 선택은 논문 결과를 유리하게 만들 수 있는 선택 편향을 유발하므로 선정 규칙과 제외 실행을 함께 공개해야 한다.",
        "- 평균 그래프는 세 실행의 공통 시간 구간만 사용하므로 더 길게 지속된 실행의 후반부는 포함하지 않는다.",
        "- 그래프 MAE는 시간별 평균 방향과 시간별 평균 Ideal 방향의 차이이며, 실행별 MAE의 단순평균이나 전체 원시 오차의 pooled MAE가 아니다.",
        "- Rx·Tx 방향은 엔코더 실측 자세가 아니라 실제 적용된 짐벌 명령 기반 방향이다.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def write_mean_trajectories(path, series_by_distance):
    rows = []
    for distance in (1, 2, 3):
        series = series_by_distance[distance]
        for elapsed, ideal, rx, tx in zip(
            series["elapsed_s"],
            series["ideal_deg"],
            series["rx_deg"],
            series["tx_deg"],
        ):
            rows.append(
                {
                    "distance_m": distance,
                    "elapsed_s": elapsed,
                    "mean_ideal_direction_deg": ideal,
                    "mean_rx_alignment_direction_deg": rx,
                    "mean_tx_alignment_direction_deg": tx,
                    "mean_rx_error_deg": analysis.wrap_angle(rx - ideal),
                    "mean_tx_error_deg": analysis.wrap_angle(tx - ideal),
                    "averaged_run_count": 3,
                }
            )
    write_csv(path, rows, tuple(rows[0]))


def main():
    args = parse_args()
    result_root = args.result_root.resolve()
    complete, _ = analysis.discover_experiments(result_root)
    path_lookup = {paths[0].name: paths for paths in complete}
    ranked_by_distance = {}
    summaries = []
    mean_series_by_distance = {}
    for distance in (1, 2, 3):
        names = [f"{distance}m{index:02d}" for index in range(1, 6)]
        missing = [name for name in names if name not in path_lookup]
        if missing:
            raise ValueError(f"{distance} m 실험 누락: {missing}")
        runs = [load_run(path_lookup[name], distance) for name in names]
        selected, ranked = select_best_three(runs)
        ranked_by_distance[distance] = ranked
        mean_series = mean_time_series(selected)
        mean_series_by_distance[distance] = mean_series
        summaries.append(pooled_summary(distance, selected, mean_series))

    manifest = result_root / "dynamic_tracking_best3_selection_manifest.csv"
    write_selection_manifest(manifest, ranked_by_distance)
    summary_path = result_root / "dynamic_tracking_summary_best3.csv"
    write_csv(summary_path, summaries, tuple(summaries[0]))
    write_mean_trajectories(
        result_root / "dynamic_tracking_mean_trajectories_best3.csv",
        mean_series_by_distance,
    )

    plt = analysis.configure_matplotlib()
    plt.rcParams.update(
        {
            # Use 1.5x the previous 22 pt text: 11 pt * 2 * 1.5 = 33 pt.
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
    for summary in summaries:
        distance = summary["Distance (m)"]
        selected = ranked_by_distance[distance][:3]
        plot_pooled(
            plt,
            distance,
            selected,
            summary,
            mean_series_by_distance[distance],
            result_root,
        )
    report = result_root / "dynamic_tracking_best3_analysis.md"
    write_report(report, summaries, ranked_by_distance)
    for summary in summaries:
        print(
            f"{summary['Distance (m)']}m: "
            f"selected={summary['Selected experiments']} "
            f"N={summary['Total cycles']} "
            f"overall={summary['Overall detection rate (%)']:.2f}% "
            f"mean_Rx_MAE={summary['Mean Rx MAE (deg)']:.2f} "
            f"mean_Tx_MAE={summary['Mean Tx MAE (deg)']:.2f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
