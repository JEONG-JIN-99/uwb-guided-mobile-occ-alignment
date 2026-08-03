#!/usr/bin/env python3
"""Analyze random-path dynamic-tracking experiments."""

from __future__ import annotations

import argparse
import math
import statistics
from pathlib import Path

from analyze_individual_experiments import (
    configure_matplotlib,
    format_rate,
    interpolate_angle,
    latest_run,
    read_csv,
    read_rover,
    require_fields,
    to_float,
    to_int,
    unwrap_degrees,
    wrap_angle,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RESULT_ROOT = PROJECT_ROOT / "result" / "dynamic_tracking"
REPORT_NAME_TEMPLATE = "random_path_dynamic_tracking_analysis_{experiment}.md"
RX_PLOT_NAME = "random_rx_alignment_tracking.png"
TX_PLOT_NAME = "random_tx_alignment_tracking.png"
GIMBAL_MIN_DEG = -90.0
GIMBAL_MAX_DEG = 90.0
TX_ZERO_ABSOLUTE_DEG = -180.0
CATEGORIES = (
    "success",
    "color_not_detected",
    "camera_frame_timeout",
    "uwb_unavailable",
    "other_error",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="랜덤 경로 동적 추적 실험별 분석 결과를 생성한다."
    )
    parser.add_argument(
        "--result-root",
        type=Path,
        default=DEFAULT_RESULT_ROOT,
        help="dynamic_tracking 결과 루트",
    )
    parser.add_argument(
        "--experiment",
        action="append",
        default=[],
        help="분석할 실험 폴더명. 생략하면 random_moving_rover 실행을 탐색",
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


def is_alignable(required_command_deg):
    return (
        GIMBAL_MIN_DEG
        <= required_command_deg
        <= GIMBAL_MAX_DEG
    )


def read_latest_device_run(path, required):
    fieldnames, rows = read_csv(path)
    require_fields(path, fieldnames, required)
    return latest_run(rows)


def find_rover_file(directory):
    required = {
        "elapsed_sec",
        "heading_deg",
        "rx_to_tx_deg",
        "tx_to_rx_deg",
    }
    candidates = []
    for path in sorted(directory.glob("*.csv")):
        if path.name == "rx.csv" or path.name.endswith("tx.csv"):
            continue
        fieldnames, _ = read_csv(path)
        if required <= fieldnames:
            candidates.append(path)
    if len(candidates) != 1:
        raise ValueError(f"로버 로그 후보가 {len(candidates)}개")
    return candidates[0]


def discover_experiments(result_root, selected_names):
    directories = (
        [result_root / name for name in selected_names]
        if selected_names
        else sorted(path for path in result_root.iterdir() if path.is_dir())
    )
    complete = []
    skipped = []
    for directory in directories:
        if not directory.is_dir():
            skipped.append((directory.name, "폴더 없음"))
            continue
        rx_path = directory / "rx.csv"
        tx_paths = sorted(directory.glob("*tx.csv"))
        if not rx_path.is_file() or len(tx_paths) != 1:
            skipped.append(
                (
                    directory.name,
                    f"rx={int(rx_path.is_file())}, tx={len(tx_paths)}",
                )
            )
            continue
        try:
            rover_path = find_rover_file(directory)
            _, rx_rows = read_csv(rx_path)
            rx_run = latest_run(rx_rows)
            modes = {
                str(row.get("trajectory_mode", "")).strip()
                for row in rx_run
                if str(row.get("trajectory_mode", "")).strip()
            }
            distances = {
                to_float(row.get("distance_m"))
                for row in rx_run
                if to_float(row.get("distance_m")) is not None
            }
            if modes != {"random_moving_rover"} or distances != {0.0}:
                raise ValueError(
                    f"랜덤 실행 아님: modes={sorted(modes)}, "
                    f"distances={sorted(distances)}"
                )
        except ValueError as error:
            skipped.append((directory.name, str(error)))
            continue
        complete.append((directory, rover_path, rx_path, tx_paths[0]))
    return complete, skipped


def detect_tx_coordinate_mode(rover_rows):
    initial = wrap_angle(rover_rows[0]["tx_to_rx_deg"])
    if abs(initial) <= 45.0:
        return "local"
    if abs(initial) >= 135.0:
        return "absolute"
    raise ValueError(
        f"Tx 좌표계를 판정할 수 없음: 초기 tx_to_rx_deg={initial:.2f}°"
    )


def rx_required(rover_rows, elapsed_sec):
    heading = interpolate_angle(rover_rows, elapsed_sec, "heading_deg")
    rx_to_tx = interpolate_angle(rover_rows, elapsed_sec, "rx_to_tx_deg")
    return wrap_angle(rx_to_tx - heading - 90.0)


def tx_required(rover_rows, elapsed_sec, coordinate_mode):
    tx_to_rx = interpolate_angle(rover_rows, elapsed_sec, "tx_to_rx_deg")
    if coordinate_mode == "local":
        return wrap_angle(tx_to_rx)
    return wrap_angle(tx_to_rx - TX_ZERO_ABSOLUTE_DEG)


def empty_counts():
    return {category: 0 for category in CATEGORIES}


def classify_status(row):
    status = str(row.get("status", "")).strip().casefold()
    color_success = to_int(row.get("color_success"))
    if color_success == 1 or status == "success":
        return "success"
    if status == "color_not_detected":
        return "color_not_detected"
    if status == "camera_frame_timeout":
        return "camera_frame_timeout"
    if status == "uwb_unavailable":
        return "uwb_unavailable"
    return "other_error"


def recognition_metrics(rx_run, rover_rows):
    t_start = rover_rows[0]["elapsed_sec"]
    t_end = rover_rows[-1]["elapsed_sec"]
    groups = {
        "all": empty_counts(),
        "alignable": empty_counts(),
        "out_of_range": empty_counts(),
    }
    records = []
    for row in rx_run:
        sample_index = to_int(row.get("sample_index"))
        if sample_index is None or sample_index < 1:
            continue
        if str(row.get("target_color", "")).strip().casefold() != "red":
            continue
        command_elapsed = to_float(row.get("command_elapsed_s"))
        scheduled_elapsed = to_float(row.get("scheduled_elapsed_s"))
        elapsed_sec = (
            command_elapsed
            if command_elapsed is not None
            else scheduled_elapsed
        )
        if (
            elapsed_sec is None
            or elapsed_sec < t_start
            or elapsed_sec > t_end
        ):
            continue
        required = rx_required(rover_rows, elapsed_sec)
        range_group = "alignable" if is_alignable(required) else "out_of_range"
        category = classify_status(row)
        groups["all"][category] += 1
        groups[range_group][category] += 1
        records.append(
            {
                "sample_index": sample_index,
                "elapsed_sec": elapsed_sec,
                "required_command_deg": required,
                "range_group": range_group,
                "category": category,
            }
        )
    for counts in groups.values():
        counts["total"] = sum(counts[category] for category in CATEGORIES)
    if groups["all"]["total"] != (
        groups["alignable"]["total"] + groups["out_of_range"]["total"]
    ):
        raise AssertionError("인식 범위 그룹 합계 불일치")
    return groups, records


def direction_commands(rows, rover_rows, node, tx_mode=None):
    t_start = rover_rows[0]["elapsed_sec"]
    t_end = rover_rows[-1]["elapsed_sec"]
    points = []
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
        if node == "rx":
            heading = interpolate_angle(rover_rows, elapsed_sec, "heading_deg")
            ideal = interpolate_angle(rover_rows, elapsed_sec, "rx_to_tx_deg")
            actual = wrap_angle(heading + 90.0 + command_deg)
            required = wrap_angle(ideal - heading - 90.0)
        else:
            ideal = interpolate_angle(rover_rows, elapsed_sec, "tx_to_rx_deg")
            required = tx_required(rover_rows, elapsed_sec, tx_mode)
            actual = (
                wrap_angle(command_deg)
                if tx_mode == "local"
                else wrap_angle(TX_ZERO_ABSOLUTE_DEG + command_deg)
            )
        alignable = is_alignable(required)
        points.append(
            {
                "sample_index": sample_index,
                "elapsed_sec": elapsed_sec,
                "ideal_deg": wrap_angle(ideal),
                "actual_deg": actual,
                "required_command_deg": required,
                "alignable": alignable,
                "error_deg": wrap_angle(actual - ideal),
                "servo_clipped": to_int(row.get("servo_clipped")) == 1,
            }
        )
    points.sort(key=lambda point: point["elapsed_sec"])
    return points


def rover_plot_series(rover_rows, node, tx_mode=None):
    values = []
    for row in rover_rows:
        elapsed_sec = row["elapsed_sec"]
        if node == "rx":
            ideal = wrap_angle(row["rx_to_tx_deg"])
            required = wrap_angle(
                row["rx_to_tx_deg"] - row["heading_deg"] - 90.0
            )
        else:
            ideal = wrap_angle(row["tx_to_rx_deg"])
            required = (
                wrap_angle(row["tx_to_rx_deg"])
                if tx_mode == "local"
                else wrap_angle(
                    row["tx_to_rx_deg"] - TX_ZERO_ABSOLUTE_DEG
                )
            )
        values.append(
            {
                "elapsed_sec": elapsed_sec,
                "ideal_deg": ideal,
                "alignable": is_alignable(required),
            }
        )
    return values


def out_of_range_spans(series, t_start, t_end):
    times = [item["elapsed_sec"] for item in series]
    edges = [t_start]
    edges.extend(
        (times[index - 1] + times[index]) / 2.0
        for index in range(1, len(times))
    )
    edges.append(t_end)
    spans = []
    start = None
    for index, item in enumerate(series):
        if not item["alignable"] and start is None:
            start = edges[index]
        if item["alignable"] and start is not None:
            spans.append((start, edges[index]))
            start = None
    if start is not None:
        spans.append((start, edges[-1]))
    return spans


def aligned_actual_values(points):
    ideal_at_commands = unwrap_degrees(
        [point["ideal_deg"] for point in points]
    )
    actual = unwrap_degrees([point["actual_deg"] for point in points])
    if ideal_at_commands and actual:
        turns = round((ideal_at_commands[0] - actual[0]) / 360.0)
        actual = [value + 360.0 * turns for value in actual]
    return actual


def plot_tracking(
    plt,
    output_path,
    title,
    rover_series,
    points,
    spans,
    ideal_label,
    actual_label,
    range_label,
    legend_outside=False,
):
    from matplotlib.patches import Patch

    valid = [point for point in points if point["alignable"]]
    mae = (
        statistics.mean(abs(point["error_deg"]) for point in valid)
        if valid
        else None
    )
    figure, axis = plt.subplots(figsize=(10.8, 5.9), constrained_layout=True)
    ideal_unwrapped = unwrap_degrees(
        [item["ideal_deg"] for item in rover_series]
    )
    ideal_line = axis.plot(
        [item["elapsed_sec"] for item in rover_series],
        ideal_unwrapped,
        color="#0072B2",
        linestyle="--",
        linewidth=2.0,
        label=ideal_label,
    )[0]

    actual_unwrapped = aligned_actual_values(points)
    plot_times = []
    plot_values = []
    previous_time = None
    for point, value in zip(points, actual_unwrapped):
        current_time = point["elapsed_sec"]
        crosses_excluded = (
            previous_time is not None
            and any(
                span_start < current_time and span_end > previous_time
                for span_start, span_end in spans
            )
        )
        if crosses_excluded:
            plot_times.append((previous_time + current_time) / 2.0)
            plot_values.append(math.nan)
        plot_times.append(current_time)
        plot_values.append(value if point["alignable"] else math.nan)
        previous_time = current_time
    actual_line = axis.plot(
        plot_times,
        plot_values,
        color="#D55E00",
        linewidth=1.8,
        label=(
            f"{actual_label} (Valid-range MAE = {mae:.2f}°)"
            if mae is not None
            else f"{actual_label} (Valid-range MAE = N/A)"
        ),
    )[0]

    for span_start, span_end in spans:
        axis.axvspan(
            span_start,
            span_end,
            color="#999999",
            alpha=0.22,
            linewidth=0,
        )
    legend_handles = [ideal_line, actual_line]
    if spans:
        range_patch = Patch(
            facecolor="#999999",
            alpha=0.22,
            label=(
                f"{range_label} ({len(spans)} interval"
                f"{'' if len(spans) == 1 else 's'})"
            ),
        )
        legend_handles.append(range_patch)
    if title:
        axis.set_title(title)
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("Direction (deg)")
    axis.grid(True, alpha=0.28)
    if legend_outside:
        axis.legend(
            handles=legend_handles,
            loc="lower center",
            bbox_to_anchor=(0.5, 1.01),
            borderaxespad=0,
        )
    else:
        axis.legend(handles=legend_handles, loc="best")
    figure.savefig(output_path, dpi=300)
    plt.close(figure)
    return mae


def cell(count, denominator):
    rate = 100.0 * count / denominator if denominator else None
    return f"{count} ({format_rate(rate)})"


def format_angle(value):
    return "N/A" if value is None else f"{value:.2f}°"


def group_row(label, counts, overall_total):
    total = counts["total"]
    success = counts["success"]
    conditional_denominator = success + counts["color_not_detected"]
    return [
        label,
        cell(total, overall_total),
        cell(success, total),
        cell(counts["color_not_detected"], total),
        cell(counts["camera_frame_timeout"], total),
        cell(counts["uwb_unavailable"], total),
        cell(counts["other_error"], total),
        format_rate(100.0 * success / total if total else None),
        format_rate(
            100.0 * success / conditional_denominator
            if conditional_denominator
            else None
        ),
    ]


def point_summary(points):
    alignable = [point for point in points if point["alignable"]]
    excluded = [point for point in points if not point["alignable"]]
    clipped_feasible = sum(
        point["servo_clipped"] for point in alignable
    )
    unclipped_infeasible = sum(
        not point["servo_clipped"] for point in excluded
    )
    mae = (
        statistics.mean(abs(point["error_deg"]) for point in alignable)
        if alignable
        else None
    )
    return {
        "total": len(points),
        "alignable": len(alignable),
        "excluded": len(excluded),
        "clipped_feasible": clipped_feasible,
        "unclipped_infeasible": unclipped_infeasible,
        "mae": mae,
    }


def write_report(
    directory,
    rover_path,
    rx_path,
    tx_path,
    rover_rows,
    tx_mode,
    recognition,
    rx_points,
    tx_points,
    rx_plot_name=RX_PLOT_NAME,
    tx_plot_name=TX_PLOT_NAME,
):
    overall_total = recognition["all"]["total"]
    rows = [
        group_row("전체 구간", recognition["all"], overall_total),
        group_row("정렬 가능 구간", recognition["alignable"], overall_total),
        group_row(
            "범위 초과 구간",
            recognition["out_of_range"],
            overall_total,
        ),
    ]
    table_rows = "\n".join(f"| {' | '.join(row)} |" for row in rows)
    rx = point_summary(rx_points)
    tx = point_summary(tx_points)
    rx_excluded_rate = (
        100.0 * rx["excluded"] / rx["total"] if rx["total"] else 0.0
    )
    tx_excluded_rate = (
        100.0 * tx["excluded"] / tx["total"] if tx["total"] else 0.0
    )
    recognition_excluded_rate = (
        100.0
        * recognition["out_of_range"]["total"]
        / overall_total
        if overall_total
        else 0.0
    )
    if recognition["out_of_range"]["total"]:
        recognition_note = (
            f"로버 시간 범위 안의 전체 빨간색 평가 주기는 {overall_total}개이며, "
            f"이 중 Rx 짐벌 범위 초과 주기는 "
            f"{recognition['out_of_range']['total']}개 "
            f"({recognition_excluded_rate:.1f}%)였다. 범위 초과 구간의 인식 "
            "성공은 정렬 성공이 아니라 카메라 시야에서 발생할 수 있는 검출로 "
            "해석해야 한다."
        )
    else:
        recognition_note = (
            f"로버 시간 범위 안의 전체 빨간색 평가 주기는 {overall_total}개이며, "
            "Rx 이상적 요구각이 짐벌 범위를 벗어난 주기는 0개였다. 따라서 "
            "전체 구간과 정렬 가능 구간의 인식률은 동일하다."
        )
    text = f"""# {directory.name} Random Path Dynamic Tracking Analysis

## 분석 범위

- Rover log: `{rover_path.name}`
- Rx log: `{rx_path.name}`
- Tx log: `{tx_path.name}`
- Rover time range: `{rover_rows[0]['elapsed_sec']:.3f}`–`{rover_rows[-1]['elapsed_sec']:.3f}` s
- Gimbal command range: `-90°–+90°`
- Tx coordinate mode: `{tx_mode}`

## 1. 빨간색 인식률

| 분석 범위 | 평가 주기 | 인식 성공 | 색상 미검출 | 카메라 타임아웃 | UWB 미수신 | 기타 오류 | 시스템 전체 인식률 | 정상 처리 조건 인식률 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
{table_rows}

{recognition_note}

## 2. Rx 방향 추종

![Rx random-path direction tracking]({rx_plot_name})

유효 Rx 명령 {rx['total']}개 중 정렬 가능 명령은 {rx['alignable']}개이고
범위 초과로 제외한 명령은 {rx['excluded']}개({rx_excluded_rate:.1f}%)다.
정렬 가능 구간의 평균 절대 방향 오차(MAE)는
{format_angle(rx['mae'])}이며, 이상적 요구각은 범위 안이지만
`servo_clipped=1`인 명령은 {rx['clipped_feasible']}개로 MAE에 포함했다.
정렬 방향은 엔코더 실측각이 아니라 UWB 기반 짐벌 명령으로 계산한 방향이다.

## 3. Tx 방향 추종

![Tx random-path direction tracking]({tx_plot_name})

유효 Tx 명령 {tx['total']}개 중 정렬 가능 명령은 {tx['alignable']}개이고
범위 초과로 제외한 명령은 {tx['excluded']}개({tx_excluded_rate:.1f}%)다.
정렬 가능 구간의 평균 절대 방향 오차(MAE)는
{format_angle(tx['mae'])}이며, 이상적 요구각은 범위 안이지만
`servo_clipped=1`인 명령은 {tx['clipped_feasible']}개로 MAE에 포함했다.
Tx 방향도 엔코더 실측각이 아니라 실제 적용된 짐벌 명령 기반 방향이다.

## 해석 시 주의사항

- 회색 음영은 이상적 요구각이 짐벌 가동 범위를 벗어나 MAE에서 제외한 시간이다.
- 시간축은 범위 초과 시간을 제거하지 않은 실제 로버 경과시간이다.
- 정렬 방향은 엔코더 실측이 아니라 실제 적용된 짐벌 명령 기반 방향이다.
"""
    report_name = REPORT_NAME_TEMPLATE.format(experiment=directory.name)
    (directory / report_name).write_text(text, encoding="utf-8")


def analyze_experiment(
    plt,
    paths,
    show_title=True,
    legend_outside=False,
    append_experiment_name=False,
):
    directory, rover_path, rx_path, tx_path = paths
    rover_rows = read_rover(rover_path)
    tx_mode = detect_tx_coordinate_mode(rover_rows)
    rx_required_fields = {
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
    tx_required_fields = {
        "sample_index",
        "trajectory_mode",
        "distance_m",
        "scheduled_elapsed_s",
        "command_elapsed_s",
        "gimbal_command_ros_deg",
        "servo_clipped",
        "status",
    }
    rx_run = read_latest_device_run(rx_path, rx_required_fields)
    tx_run = read_latest_device_run(tx_path, tx_required_fields)
    tx_modes = {
        str(row.get("trajectory_mode", "")).strip()
        for row in tx_run
        if str(row.get("trajectory_mode", "")).strip()
    }
    if tx_modes != {"random_moving_rover"}:
        raise ValueError(f"{directory.name}: Tx 랜덤 모드 불일치 {tx_modes}")

    recognition, _ = recognition_metrics(rx_run, rover_rows)
    rx_points = direction_commands(rx_run, rover_rows, "rx")
    tx_points = direction_commands(tx_run, rover_rows, "tx", tx_mode)
    rx_series = rover_plot_series(rover_rows, "rx")
    tx_series = rover_plot_series(rover_rows, "tx", tx_mode)
    t_start = rover_rows[0]["elapsed_sec"]
    t_end = rover_rows[-1]["elapsed_sec"]
    rx_spans = out_of_range_spans(rx_series, t_start, t_end)
    tx_spans = out_of_range_spans(tx_series, t_start, t_end)
    rx_plot_name = (
        f"{Path(RX_PLOT_NAME).stem}_{directory.name}.png"
        if append_experiment_name
        else RX_PLOT_NAME
    )
    tx_plot_name = (
        f"{Path(TX_PLOT_NAME).stem}_{directory.name}.png"
        if append_experiment_name
        else TX_PLOT_NAME
    )

    rx_mae = plot_tracking(
        plt,
        directory / rx_plot_name,
        (
            f"{directory.name} Random Path Rx Direction Tracking"
            if show_title
            else None
        ),
        rx_series,
        rx_points,
        rx_spans,
        "Ideal Rx→Tx direction",
        "UWB-aligned Rx direction",
        "Out of Rx gimbal range (excluded)",
        legend_outside=legend_outside,
    )
    tx_mae = plot_tracking(
        plt,
        directory / tx_plot_name,
        (
            f"{directory.name} Random Path Tx Direction Tracking"
            if show_title
            else None
        ),
        tx_series,
        tx_points,
        tx_spans,
        "Ideal Tx→Rx direction",
        "UWB gimbal command direction",
        "Out of Tx gimbal range (excluded)",
        legend_outside=legend_outside,
    )
    write_report(
        directory,
        rover_path,
        rx_path,
        tx_path,
        rover_rows,
        tx_mode,
        recognition,
        rx_points,
        tx_points,
        rx_plot_name,
        tx_plot_name,
    )
    return {
        "experiment": directory.name,
        "tx_mode": tx_mode,
        "recognition": recognition,
        "rx": point_summary(rx_points),
        "tx": point_summary(tx_points),
        "rx_mae": rx_mae,
        "tx_mae": tx_mae,
        "rx_spans": len(rx_spans),
        "tx_spans": len(tx_spans),
    }


def main():
    args = parse_args()
    result_root = args.result_root.resolve()
    complete, skipped = discover_experiments(result_root, args.experiment)
    if not complete:
        raise SystemExit("분석 가능한 랜덤 경로 실험이 없습니다.")
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
            f"{result['experiment']}: tx_mode={result['tx_mode']} "
            f"recognition_N={recognition['all']['total']} "
            f"recognition_out={recognition['out_of_range']['total']} "
            f"Rx={result['rx']['alignable']}/{result['rx']['total']} "
            f"Rx_MAE={result['rx_mae']:.2f} "
            f"Tx={result['tx']['alignable']}/{result['tx']['total']} "
            f"Tx_MAE={result['tx_mae']:.2f} "
            f"spans={result['rx_spans']}/{result['tx_spans']}"
        )
    for experiment, reason in skipped:
        print(f"SKIP {experiment}: {reason}")


if __name__ == "__main__":
    main()
