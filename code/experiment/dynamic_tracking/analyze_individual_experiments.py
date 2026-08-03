#!/usr/bin/env python3
"""Generate the three prescribed outputs for each dynamic-tracking run."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import statistics
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RESULT_ROOT = PROJECT_ROOT / "result" / "dynamic_tracking"
ROVER_MARKER = "arc_chrony_test_vel"
RX_PLOT_NAME = "rx_alignment_tracking.png"
TX_PLOT_NAME = "tx_alignment_tracking.png"
RX_TX_OVERLAY_PLOT_NAME = "rx_tx_alignment_tracking_overlay.png"
REPORT_NAME_TEMPLATE = "dynamic_tracking_analysis_{experiment}.md"


def parse_args():
    parser = argparse.ArgumentParser(
        description="동적 추적 실험별 표, 그래프, 설명 문서를 생성한다."
    )
    parser.add_argument(
        "--result-root",
        type=Path,
        default=DEFAULT_RESULT_ROOT,
        help="실험 폴더가 들어 있는 dynamic_tracking 결과 루트",
    )
    parser.add_argument(
        "--experiment",
        action="append",
        default=[],
        help="분석할 실험 폴더명. 여러 번 지정할 수 있음",
    )
    parser.add_argument(
        "--font-scale",
        type=float,
        default=1.0,
        help="그래프 전체 글자 크기 배율",
    )
    parser.add_argument(
        "--no-title",
        action="store_true",
        help="그래프 제목을 표시하지 않음",
    )
    parser.add_argument(
        "--append-experiment-name",
        action="store_true",
        help="그래프 파일명 끝에 실험 폴더명을 추가",
    )
    return parser.parse_args()


def wrap_angle(angle_deg):
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def to_float(value):
    try:
        stripped = str(value).strip()
        return float(stripped) if stripped else None
    except (TypeError, ValueError):
        return None


def to_int(value):
    try:
        stripped = str(value).strip()
        return int(stripped) if stripped else None
    except (TypeError, ValueError):
        return None


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fieldnames = set(reader.fieldnames or ())
        rows = list(reader)
    return fieldnames, rows


def require_fields(path, fieldnames, required):
    missing = set(required) - fieldnames
    if missing:
        raise ValueError(f"{path.name}: 필수 열 누락 {sorted(missing)}")


def latest_run(rows):
    """Return the last run delimited by a sample_index=0 row."""
    starts = [
        index
        for index, row in enumerate(rows)
        if to_int(row.get("sample_index")) == 0
    ]
    if not starts:
        raise ValueError("sample_index=0 실행 경계를 찾을 수 없음")
    return rows[starts[-1] :]


def discover_experiments(result_root):
    complete = []
    skipped = []
    for directory in sorted(path for path in result_root.iterdir() if path.is_dir()):
        rover_files = sorted(
            path
            for path in directory.glob("*.csv")
            if ROVER_MARKER in path.name
        )
        rx_path = directory / "rx.csv"
        tx_files = sorted(
            path
            for path in directory.glob("*tx.csv")
            if path.name != "rx.csv"
        )
        if len(rover_files) == 1 and rx_path.is_file() and len(tx_files) == 1:
            complete.append((directory, rover_files[0], rx_path, tx_files[0]))
            continue
        reasons = []
        if len(rover_files) != 1:
            reasons.append(f"로버 로그 {len(rover_files)}개")
        if not rx_path.is_file():
            reasons.append("rx.csv 없음")
        if len(tx_files) != 1:
            reasons.append(f"Tx 로그 {len(tx_files)}개")
        skipped.append((directory.name, ", ".join(reasons)))
    return complete, skipped


def read_rover(path):
    required = {
        "elapsed_sec",
        "heading_deg",
        "rx_to_tx_deg",
        "tx_to_rx_deg",
    }
    fieldnames, raw_rows = read_csv(path)
    require_fields(path, fieldnames, required)
    rows = []
    for raw in raw_rows:
        values = {field: to_float(raw.get(field)) for field in required}
        if any(value is None for value in values.values()):
            continue
        rows.append(values)
    rows.sort(key=lambda row: row["elapsed_sec"])
    if len(rows) < 2:
        raise ValueError(f"{path.name}: 유효 로버 행이 2개 미만")
    return rows


def read_device_run(path, required):
    fieldnames, all_rows = read_csv(path)
    require_fields(path, fieldnames, required)
    return latest_run(all_rows)


def interpolate_angle(rows, elapsed_sec, field):
    times = [row["elapsed_sec"] for row in rows]
    if elapsed_sec < times[0] or elapsed_sec > times[-1]:
        raise ValueError("로버 시간 범위 밖의 각도를 외삽할 수 없음")
    high = bisect.bisect_right(times, elapsed_sec)
    if high == 0:
        return wrap_angle(rows[0][field])
    if high == len(rows):
        return wrap_angle(rows[-1][field])
    before = rows[high - 1]
    after = rows[high]
    width = after["elapsed_sec"] - before["elapsed_sec"]
    if width <= 0:
        return wrap_angle(before[field])
    ratio = (elapsed_sec - before["elapsed_sec"]) / width
    delta = wrap_angle(after[field] - before[field])
    return wrap_angle(before[field] + ratio * delta)


def recognition_metrics(rows, t_start, t_end):
    categories = {
        "success": 0,
        "color_not_detected": 0,
        "camera_frame_timeout": 0,
        "uwb_unavailable": 0,
        "other_error": 0,
    }
    included = []
    for row in rows:
        sample_index = to_int(row.get("sample_index"))
        if sample_index is None or sample_index < 1:
            continue
        command_elapsed = to_float(row.get("command_elapsed_s"))
        scheduled_elapsed = to_float(row.get("scheduled_elapsed_s"))
        evaluation_elapsed = (
            command_elapsed
            if command_elapsed is not None
            else scheduled_elapsed
        )
        if (
            evaluation_elapsed is None
            or evaluation_elapsed < t_start
            or evaluation_elapsed > t_end
        ):
            continue
        if str(row.get("target_color", "")).strip().casefold() != "red":
            continue

        status = str(row.get("status", "")).strip().casefold()
        color_success = to_int(row.get("color_success"))
        if color_success == 1 or status == "success":
            category = "success"
        elif status == "color_not_detected":
            category = "color_not_detected"
        elif status == "camera_frame_timeout":
            category = "camera_frame_timeout"
        elif status == "uwb_unavailable":
            category = "uwb_unavailable"
        else:
            category = "other_error"
        categories[category] += 1
        included.append((sample_index, evaluation_elapsed, category))

    total = len(included)
    if total != sum(categories.values()):
        raise AssertionError("인식 결과 범주 합계가 전체 평가 주기와 다름")
    success = categories["success"]
    detected_attempts = success + categories["color_not_detected"]
    return {
        **categories,
        "total": total,
        "system_rate": 100.0 * success / total if total else None,
        "conditional_rate": (
            100.0 * success / detected_attempts
            if detected_attempts
            else None
        ),
        "included": included,
    }


def command_rows(rows, t_start, t_end):
    selected = []
    for row in rows:
        sample_index = to_int(row.get("sample_index"))
        elapsed_sec = to_float(row.get("command_elapsed_s"))
        command_deg = to_float(row.get("gimbal_command_ros_deg"))
        if (
            sample_index is None
            or sample_index < 1
            or elapsed_sec is None
            or command_deg is None
            or elapsed_sec < t_start
            or elapsed_sec > t_end
        ):
            continue
        selected.append(
            {
                "sample_index": sample_index,
                "elapsed_sec": elapsed_sec,
                "gimbal_command_ros_deg": command_deg,
            }
        )
    selected.sort(key=lambda row: row["elapsed_sec"])
    return selected


def build_rx_points(rover_rows, commands):
    points = []
    for command in commands:
        elapsed_sec = command["elapsed_sec"]
        heading_deg = interpolate_angle(rover_rows, elapsed_sec, "heading_deg")
        ideal_deg = interpolate_angle(rover_rows, elapsed_sec, "rx_to_tx_deg")
        aligned_deg = wrap_angle(
            heading_deg + 90.0 + command["gimbal_command_ros_deg"]
        )
        points.append(
            {
                **command,
                "ideal_deg": ideal_deg,
                "aligned_deg": aligned_deg,
                "error_deg": wrap_angle(aligned_deg - ideal_deg),
            }
        )
    return points


def build_tx_points(rover_rows, commands):
    points = []
    for command in commands:
        elapsed_sec = command["elapsed_sec"]
        ideal_deg = interpolate_angle(rover_rows, elapsed_sec, "tx_to_rx_deg")
        commanded_deg = wrap_angle(command["gimbal_command_ros_deg"])
        points.append(
            {
                **command,
                "ideal_deg": ideal_deg,
                "aligned_deg": commanded_deg,
                "error_deg": wrap_angle(commanded_deg - ideal_deg),
            }
        )
    return points


def unwrap_degrees(values):
    if not values:
        return []
    unwrapped = [float(values[0])]
    for value in values[1:]:
        unwrapped.append(unwrapped[-1] + wrap_angle(value - unwrapped[-1]))
    return unwrapped


def align_unwrapped(reference, observed):
    reference_unwrapped = unwrap_degrees(reference)
    observed_unwrapped = unwrap_degrees(observed)
    if reference_unwrapped and observed_unwrapped:
        shift_turns = round(
            (reference_unwrapped[0] - observed_unwrapped[0]) / 360.0
        )
        observed_unwrapped = [
            value + 360.0 * shift_turns for value in observed_unwrapped
        ]
    return reference_unwrapped, observed_unwrapped


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
            "figure.dpi": 120,
            "savefig.dpi": 300,
        }
    )
    return plt


def plot_tracking(
    plt,
    points,
    output_path,
    title,
    ideal_label,
    actual_label,
    legend_outside=False,
):
    if not points:
        raise ValueError(f"{output_path.parent.name}: 그래프용 유효 명령 없음")
    times = [point["elapsed_sec"] for point in points]
    mae = statistics.mean(abs(point["error_deg"]) for point in points)
    ideal, actual = align_unwrapped(
        [point["ideal_deg"] for point in points],
        [point["aligned_deg"] for point in points],
    )
    figure, axis = plt.subplots(figsize=(10.5, 5.8), constrained_layout=True)
    axis.plot(
        times,
        ideal,
        color="#0072B2",
        linestyle="--",
        linewidth=2.0,
        label=ideal_label,
    )
    axis.plot(
        times,
        actual,
        color="#D55E00",
        linewidth=1.8,
        label=f"{actual_label} (MAE = {mae:.2f}°)",
    )
    if title:
        axis.set_title(title)
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("Direction (deg)")
    axis.grid(True, alpha=0.28)
    if legend_outside:
        axis.legend(
            loc="lower center",
            bbox_to_anchor=(0.5, 1.01),
            borderaxespad=0,
        )
    else:
        axis.legend(loc="best")
    figure.savefig(output_path, dpi=300)
    plt.close(figure)


def plot_rx_tx_overlay(
    plt,
    rx_points,
    tx_points,
    output_path,
    title=None,
    legend_outside=False,
):
    """Plot the ideal, Rx alignment, and Tx alignment directions."""
    if not rx_points or not tx_points:
        raise ValueError(f"{output_path.parent.name}: Rx/Tx graph data missing")

    rx_ideal, rx_actual = align_unwrapped(
        [point["ideal_deg"] for point in rx_points],
        [point["aligned_deg"] for point in rx_points],
    )
    _, tx_actual = align_unwrapped(
        [point["ideal_deg"] for point in tx_points],
        [point["aligned_deg"] for point in tx_points],
    )
    rx_mae = statistics.mean(abs(point["error_deg"]) for point in rx_points)
    tx_mae = statistics.mean(abs(point["error_deg"]) for point in tx_points)

    # Match the established paper figure geometry (3150 x 1940 px at 300 dpi).
    figure, axis = plt.subplots(
        figsize=(10.5, 1940.0 / 300.0),
        constrained_layout=True,
    )
    axis.plot(
        [point["elapsed_sec"] for point in rx_points],
        rx_ideal,
        color="#000000",
        linestyle="--",
        linewidth=2.8,
        label="Ideal direction",
        zorder=2,
    )
    axis.plot(
        [point["elapsed_sec"] for point in rx_points],
        rx_actual,
        color="#FF0000",
        linestyle="-",
        linewidth=2.0,
        marker="o",
        markersize=3.2,
        markevery=8,
        label=f"Rx alignment direction (MAE = {rx_mae:.2f}°)",
        zorder=4,
    )
    axis.plot(
        [point["elapsed_sec"] for point in tx_points],
        tx_actual,
        color="#0000FF",
        linestyle="-.",
        linewidth=2.0,
        marker="s",
        markersize=3.0,
        markevery=8,
        label=f"Tx alignment direction (MAE = {tx_mae:.2f}°)",
        zorder=5,
    )
    if title:
        axis.set_title(title)
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("Direction (deg)")
    axis.grid(True, alpha=0.28)
    if legend_outside:
        # Keep the same legend type size as the established paper figures.
        # Stacking the descriptive entries prevents clipping without shrinking.
        axis.legend(
            loc="lower center",
            bbox_to_anchor=(0.5, 1.01),
            borderaxespad=0,
            ncol=1,
        )
    else:
        axis.legend(loc="best", ncol=1)
    figure.savefig(output_path, dpi=300)
    plt.close(figure)


def format_rate(value):
    return "N/A" if value is None else f"{value:.1f}%"


def count_rate_cell(count, total):
    rate = 100.0 * count / total if total else None
    return f"{count} ({format_rate(rate)})"


def tracking_description(points, subject):
    errors = [point["error_deg"] for point in points]
    ideal = unwrap_degrees([point["ideal_deg"] for point in points])
    aligned = unwrap_degrees([point["aligned_deg"] for point in points])
    mae = statistics.mean(abs(error) for error in errors)
    maximum = max(abs(error) for error in errors)
    ideal_change = ideal[-1] - ideal[0]
    aligned_change = aligned[-1] - aligned[0]
    first = (
        f"로버 운행 시간 안의 유효 {subject} 명령 {len(points)}개를 비교했으며, "
        f"이상적 방향은 분석 구간에서 {ideal_change:+.1f}° 변화하고 "
        f"명령 기반 정렬 방향은 {aligned_change:+.1f}° 변화했다."
    )
    second = (
        f"두 선 사이의 평균 절대 방향 오차(MAE)는 {mae:.2f}°이고 "
        f"최대 절대 차이는 {maximum:.2f}°로 나타났다."
    )
    return first, second


def write_report(
    output_path,
    experiment_id,
    rover_path,
    rx_path,
    tx_path,
    t_start,
    t_end,
    recognition,
    rx_points,
    tx_points,
    overlay_plot_name=RX_TX_OVERLAY_PLOT_NAME,
):
    rx_description = tracking_description(rx_points, "Rx")
    tx_description = tracking_description(tx_points, "Tx")
    total = recognition["total"]
    row = [
        str(total),
        count_rate_cell(recognition["success"], total),
        count_rate_cell(recognition["color_not_detected"], total),
        count_rate_cell(recognition["camera_frame_timeout"], total),
        count_rate_cell(recognition["uwb_unavailable"], total),
        count_rate_cell(recognition["other_error"], total),
        format_rate(recognition["system_rate"]),
        format_rate(recognition["conditional_rate"]),
    ]
    text = f"""# {experiment_id} Dynamic Tracking Analysis

## 분석 범위

- Rover log: `{rover_path.name}`
- Rx log: `{rx_path.name}`
- Tx log: `{tx_path.name}`
- Rover time range: `{t_start:.3f}`–`{t_end:.3f}` s
- 분석 기준: 로버 기록 시간 범위 안의 표본만 사용

## 1. 빨간색 인식률

| 전체 주기 | 인식 성공 | 색상 미검출 | 카메라 타임아웃 | UWB 미수신 | 기타 오류 | 시스템 전체 인식률 | 정상 처리 조건 인식률 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| {" | ".join(row)} |

`sample_index=0` 초기화 행은 제외하고 로버 시간 범위 안의 {total}개 주기를
전체 분모로 사용했다. 시스템 전체 인식률은 모든 실패 원인을 포함하며, 정상
처리 조건 인식률은 UWB 수신과 카메라 프레임 획득 후 실제 색상 판정까지
진행된 `인식 성공 + 색상 미검출` 주기만 분모로 사용했다.

## 2. Rx·Tx 정렬 정확도

![Rx and Tx alignment tracking]({overlay_plot_name})

{rx_description[0]}
{rx_description[1]}
{tx_description[0]}
{tx_description[1]} 두 방향 모두 엔코더 실측각이 아니라 실제 적용된
`gimbal_command_ros_deg`를 좌표계에 맞게 변환한 명령 기반 방향이다.

## 해석 시 주의사항

- Rx와 Tx의 정렬 방향은 엔코더 측정값이 아니라 실제 적용된 짐벌 명령을
  좌표계에 맞게 변환한 값이다.
- 분석은 로버 로그의 시간 범위로 제한했다.
"""
    output_path.write_text(text, encoding="utf-8")


def analyze_experiment(
    plt,
    paths,
    show_title=True,
    legend_outside=False,
    append_experiment_name=False,
):
    directory, rover_path, rx_path, tx_path = paths
    rover_rows = read_rover(rover_path)
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
    rx_run = read_device_run(rx_path, rx_required)
    tx_run = read_device_run(tx_path, tx_required)
    for node, rows in (("Rx", rx_run), ("Tx", tx_run)):
        modes = {
            str(row.get("trajectory_mode", "")).strip()
            for row in rows
            if str(row.get("trajectory_mode", "")).strip()
        }
        distances = {
            to_float(row.get("distance_m"))
            for row in rows
            if to_float(row.get("distance_m")) is not None
        }
        if modes != {"fixed_radius_arc"}:
            raise ValueError(
                f"{directory.name}: {node} trajectory_mode 불일치 {modes}"
            )
        if len(distances) != 1 or next(iter(distances)) not in {1.0, 2.0, 3.0}:
            raise ValueError(
                f"{directory.name}: {node} distance_m 불일치 {distances}"
            )
    rx_distances = {to_float(row.get("distance_m")) for row in rx_run}
    tx_distances = {to_float(row.get("distance_m")) for row in tx_run}
    if rx_distances != tx_distances:
        raise ValueError(f"{directory.name}: Rx/Tx 거리 불일치")
    recognition = recognition_metrics(rx_run, t_start, t_end)
    rx_points = build_rx_points(
        rover_rows, command_rows(rx_run, t_start, t_end)
    )
    tx_points = build_tx_points(
        rover_rows, command_rows(tx_run, t_start, t_end)
    )
    common_differences = [
        abs(wrap_angle(row["rx_to_tx_deg"] - row["tx_to_rx_deg"]))
        for row in rover_rows
    ]
    if max(common_differences) > 45.0:
        raise ValueError(
            f"{directory.name}: Rx/Tx 이상적 방향의 공통 표시 좌표계 불일치"
        )
    if not tx_points or abs(tx_points[0]["error_deg"]) > 45.0:
        raise ValueError(
            f"{directory.name}: Tx 초기 좌표계 차이가 45°를 초과합니다"
        )
    rx_tx_overlay_plot_name = (
        f"{Path(RX_TX_OVERLAY_PLOT_NAME).stem}_{directory.name}.png"
        if append_experiment_name
        else RX_TX_OVERLAY_PLOT_NAME
    )

    plot_rx_tx_overlay(
        plt,
        rx_points,
        tx_points,
        directory / rx_tx_overlay_plot_name,
        f"{directory.name} Rx/Tx Direction Tracking" if show_title else None,
        legend_outside=legend_outside,
    )
    write_report(
        directory / REPORT_NAME_TEMPLATE.format(experiment=directory.name),
        directory.name,
        rover_path,
        rx_path,
        tx_path,
        t_start,
        t_end,
        recognition,
        rx_points,
        tx_points,
        rx_tx_overlay_plot_name,
    )
    return {
        "experiment": directory.name,
        "recognition": recognition,
        "rx_count": len(rx_points),
        "tx_count": len(tx_points),
    }


def main():
    args = parse_args()
    result_root = args.result_root.resolve()
    complete, skipped = discover_experiments(result_root)
    if args.experiment:
        selected = set(args.experiment)
        complete = [
            paths for paths in complete if paths[0].name in selected
        ]
        missing = selected - {paths[0].name for paths in complete}
        if missing:
            raise SystemExit(
                f"선택한 실험을 찾을 수 없습니다: {sorted(missing)}"
            )
    if not complete:
        raise SystemExit("분석 가능한 완전한 실험 폴더가 없습니다.")
    plt = configure_matplotlib()
    if args.font_scale <= 0:
        raise SystemExit("--font-scale은 0보다 커야 합니다.")
    plt.rcParams.update(
        {
            key: value * args.font_scale
            for key, value in {
                "font.size": 11.0,
                "axes.labelsize": 11.0,
                "xtick.labelsize": 11.0,
                "ytick.labelsize": 11.0,
                "legend.fontsize": 11.0,
            }.items()
        }
    )
    results = [
        analyze_experiment(
            plt,
            paths,
            show_title=not args.no_title,
            legend_outside=args.font_scale > 1.0,
            append_experiment_name=args.append_experiment_name,
        )
        for paths in complete
    ]
    for result in results:
        recognition = result["recognition"]
        print(
            f"{result['experiment']}: "
            f"N={recognition['total']} "
            f"success={recognition['success']} "
            f"system={format_rate(recognition['system_rate'])} "
            f"conditional={format_rate(recognition['conditional_rate'])} "
            f"Rx={result['rx_count']} Tx={result['tx_count']}"
        )
    for experiment, reason in skipped:
        print(f"SKIP {experiment}: {reason}")


if __name__ == "__main__":
    main()
