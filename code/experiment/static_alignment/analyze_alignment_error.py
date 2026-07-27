#!/usr/bin/env python3
"""Tx 기준각과 짐벌 명령각을 비교해 소프트웨어 정렬오차를 분석한다."""

import argparse
import csv
import json
import math
import os
import statistics
import tempfile
from pathlib import Path


DEFAULT_INPUT_NAME = "static_alignment_results.csv"
DETAIL_OUTPUT_NAME = "alignment_error_results.csv"
SUMMARY_OUTPUT_NAME = "alignment_error_summary.json"
PLOT_OUTPUT_NAME = "alignment_error_plot.png"


def angular_difference_deg(angle_deg, reference_deg):
    """reference에서 angle까지의 최소 부호 각도차를 [-180, 180)로 반환한다."""
    return (float(angle_deg) - float(reference_deg) + 180.0) % 360.0 - 180.0


def percentile(values, percent):
    """선형 보간 백분위수를 계산한다."""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("percentile requires at least one value")
    position = (len(ordered) - 1) * float(percent) / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize(errors):
    """부호 오차 목록으로 정렬오차 요약 통계를 만든다."""
    errors = [float(value) for value in errors]
    if not errors:
        return None
    absolute = [abs(value) for value in errors]
    return {
        "count": len(errors),
        "mean_signed_error_deg": statistics.fmean(errors),
        "std_signed_error_deg": (
            statistics.stdev(errors) if len(errors) > 1 else 0.0
        ),
        "mean_absolute_error_deg": statistics.fmean(absolute),
        "rmse_deg": math.sqrt(statistics.fmean(value * value for value in errors)),
        "median_absolute_error_deg": statistics.median(absolute),
        "p95_absolute_error_deg": percentile(absolute, 95.0),
        "max_absolute_error_deg": max(absolute),
        "within_1_deg_percent": 100.0
        * sum(value <= 1.0 for value in absolute)
        / len(absolute),
        "within_3_deg_percent": 100.0
        * sum(value <= 3.0 for value in absolute)
        / len(absolute),
        "within_5_deg_percent": 100.0
        * sum(value <= 5.0 for value in absolute)
        / len(absolute),
    }


def read_error_rows(csv_path, reference_deg):
    """원본 CSV에서 유효 짐벌 명령을 읽고 오차 필드를 덧붙인다."""
    with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames or "gimbal_command_ros_deg" not in reader.fieldnames:
            raise ValueError(
                f"{csv_path}에 gimbal_command_ros_deg 열이 없습니다."
            )

        rows = []
        skipped = 0
        for source in reader:
            raw_command = source.get("gimbal_command_ros_deg", "").strip()
            if not raw_command:
                skipped += 1
                continue
            try:
                command = float(raw_command)
            except ValueError:
                skipped += 1
                continue
            if not math.isfinite(command):
                skipped += 1
                continue

            error = angular_difference_deg(command, reference_deg)
            rows.append(
                {
                    "attempt": source.get("attempt", ""),
                    "status": source.get("status", ""),
                    "color_success": source.get("color_success", ""),
                    "initial_gimbal_ros_deg": source.get(
                        "initial_gimbal_ros_deg", ""
                    ),
                    "uwb_raw_azimuth_deg": source.get(
                        "uwb_raw_azimuth_deg", ""
                    ),
                    "gimbal_command_ros_deg": command,
                    "reference_tx_ros_deg": float(reference_deg),
                    "signed_alignment_error_deg": error,
                    "absolute_alignment_error_deg": abs(error),
                }
            )
    return rows, skipped


def write_details(rows, output_path):
    fieldnames = tuple(rows[0])
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            output_row = dict(row)
            for field in (
                "gimbal_command_ros_deg",
                "reference_tx_ros_deg",
                "signed_alignment_error_deg",
                "absolute_alignment_error_deg",
            ):
                output_row[field] = f"{row[field]:.6f}"
            writer.writerow(output_row)


def build_summary(rows, source_path, reference_deg, skipped):
    groups = {"all": rows}
    for row in rows:
        status = row["status"] or "unknown"
        groups.setdefault(f"status:{status}", []).append(row)

    statistics_by_group = {}
    for name, group_rows in groups.items():
        group_summary = summarize(
            row["signed_alignment_error_deg"] for row in group_rows
        )
        statistics_by_group[name] = {
            key: value if key == "count" else round(value, 6)
            for key, value in group_summary.items()
        }

    return {
        "definition": (
            "signed_alignment_error_deg = shortest angular difference from "
            "reference_tx_ros_deg to gimbal_command_ros_deg; "
            "absolute_alignment_error_deg = abs(signed_alignment_error_deg)"
        ),
        "source_csv": source_path.name,
        "reference_tx_ros_deg": float(reference_deg),
        "valid_command_count": len(rows),
        "skipped_row_count": skipped,
        "statistics": statistics_by_group,
    }


def plot_errors(rows, summary, output_path):
    # 홈의 matplotlib 캐시가 쓰기 불가능한 장비에서도 실행되게 한다.
    os.environ.setdefault(
        "MPLCONFIGDIR",
        str(Path(tempfile.gettempdir()) / "uwb_alignment_matplotlib"),
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    attempts = [int(row["attempt"]) for row in rows]
    errors = [row["signed_alignment_error_deg"] for row in rows]
    statuses = [row["status"] or "unknown" for row in rows]
    colors = {
        "success": "#2ca02c",
        "color_not_detected": "#d62728",
        "camera_frame_timeout": "#ff7f0e",
        "uwb_timeout": "#9467bd",
        "error": "#7f7f7f",
        "unknown": "#1f77b4",
    }

    figure, (axis_series, axis_hist) = plt.subplots(
        2, 1, figsize=(12, 8), constrained_layout=True
    )
    axis_series.axhline(0.0, color="black", linewidth=1.2, label="Reference (0 deg)")
    for status in dict.fromkeys(statuses):
        indices = [index for index, value in enumerate(statuses) if value == status]
        axis_series.scatter(
            [attempts[index] for index in indices],
            [errors[index] for index in indices],
            s=28,
            alpha=0.85,
            color=colors.get(status, "#1f77b4"),
            label=status,
        )
    axis_series.set_title("Software Alignment Error by Attempt")
    axis_series.set_xlabel("Attempt")
    axis_series.set_ylabel("Signed error (deg)")
    axis_series.grid(alpha=0.25)
    axis_series.legend(loc="best", ncols=2)

    absolute = [abs(value) for value in errors]
    bin_count = min(20, max(5, math.ceil(math.sqrt(len(absolute)))))
    axis_hist.hist(
        absolute,
        bins=bin_count,
        color="#4c78a8",
        edgecolor="white",
        alpha=0.9,
    )
    all_stats = summary["statistics"]["all"]
    axis_hist.axvline(
        all_stats["mean_absolute_error_deg"],
        color="#d62728",
        linestyle="--",
        linewidth=1.8,
        label=f"MAE = {all_stats['mean_absolute_error_deg']:.3f} deg",
    )
    axis_hist.axvline(
        all_stats["median_absolute_error_deg"],
        color="#2ca02c",
        linestyle=":",
        linewidth=1.8,
        label=f"Median = {all_stats['median_absolute_error_deg']:.3f} deg",
    )
    axis_hist.set_title("Absolute Alignment Error Distribution")
    axis_hist.set_xlabel("Absolute error (deg)")
    axis_hist.set_ylabel("Count")
    axis_hist.grid(axis="y", alpha=0.25)
    axis_hist.legend(loc="best")

    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "정적 정렬 CSV의 짐벌 명령각을 Tx 기준각과 비교해 오차 CSV, "
            "요약 JSON, PNG 그래프를 생성합니다."
        )
    )
    parser.add_argument(
        "run_dir",
        type=Path,
        help="static_alignment_results.csv가 들어 있는 실행 결과 폴더",
    )
    parser.add_argument(
        "--reference-angle",
        type=float,
        default=0.0,
        help="Tx의 실제 ROS yaw 기준각(기본값: 0도)",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if not math.isfinite(args.reference_angle):
        raise SystemExit("--reference-angle은 유한한 숫자여야 합니다.")

    run_dir = args.run_dir.expanduser().resolve()
    source_path = run_dir / DEFAULT_INPUT_NAME
    if not source_path.is_file():
        raise SystemExit(f"입력 CSV를 찾을 수 없습니다: {source_path}")

    rows, skipped = read_error_rows(source_path, args.reference_angle)
    if not rows:
        raise SystemExit("분석 가능한 gimbal_command_ros_deg 값이 없습니다.")

    detail_path = run_dir / DETAIL_OUTPUT_NAME
    summary_path = run_dir / SUMMARY_OUTPUT_NAME
    plot_path = run_dir / PLOT_OUTPUT_NAME
    write_details(rows, detail_path)
    summary = build_summary(rows, source_path, args.reference_angle, skipped)
    with summary_path.open("w", encoding="utf-8") as json_file:
        json.dump(summary, json_file, ensure_ascii=False, indent=2)
        json_file.write("\n")
    plot_errors(rows, summary, plot_path)

    stats = summary["statistics"]["all"]
    print(f"[DONE] valid={len(rows)}, skipped={skipped}")
    print(
        f"[SUMMARY] mean_signed={stats['mean_signed_error_deg']:.3f} deg, "
        f"MAE={stats['mean_absolute_error_deg']:.3f} deg, "
        f"RMSE={stats['rmse_deg']:.3f} deg"
    )
    print(f"[SAVED] {detail_path}")
    print(f"[SAVED] {summary_path}")
    print(f"[SAVED] {plot_path}")


if __name__ == "__main__":
    main()
