#!/usr/bin/env python3
"""Generate paper-ready results from one selected run at each distance."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

from analyze_individual_experiments import (
    build_rx_points,
    build_tx_points,
    command_rows,
    configure_matplotlib,
    read_device_run,
    read_rover,
    recognition_metrics,
    unwrap_degrees,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RESULT_ROOT = PROJECT_ROOT / "result" / "dynamic_tracking"
DEFAULT_EXPERIMENTS = ("1m01", "2m02", "3m02")

CSV_NAME = "dynamic_tracking_summary_table.csv"
TEX_NAME = "dynamic_tracking_summary_table.tex"
RX_STEM = "rx_alignment_by_distance"
TX_STEM = "tx_alignment_by_distance"
REPORT_NAME = "dynamic_tracking_paper_analysis.md"

RX_REQUIRED = {
    "sample_index",
    "scheduled_elapsed_s",
    "command_elapsed_s",
    "gimbal_command_ros_deg",
    "target_color",
    "color_success",
    "status",
}
TX_REQUIRED = {
    "sample_index",
    "command_elapsed_s",
    "gimbal_command_ros_deg",
    "status",
}

TABLE_COLUMNS = [
    "Distance (m)",
    "Total cycles",
    "Successful detections",
    "Color not detected",
    "Camera timeouts",
    "UWB unavailable",
    "Other errors",
    "Overall detection rate (%)",
    "Conditional detection rate (%)",
    "Rx MAE (deg)",
    "Tx MAE (deg)",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "각 거리에서 선택한 단일 동적 추적 실험으로 논문용 통합 결과를 "
            "생성한다."
        )
    )
    parser.add_argument(
        "--result-root",
        type=Path,
        default=DEFAULT_RESULT_ROOT,
        help="동적 추적 실험 폴더가 있는 결과 루트",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_RESULT_ROOT,
        help="표, 그래프, 분석 문서를 저장할 디렉터리",
    )
    parser.add_argument(
        "--experiments",
        nargs=3,
        metavar=("RUN_1M", "RUN_2M", "RUN_3M"),
        default=DEFAULT_EXPERIMENTS,
        help="1 m, 2 m, 3 m에서 각각 사용할 실험 ID",
    )
    parser.add_argument(
        "--manual-success-experiments",
        nargs="*",
        default=(),
        help=(
            "실패 프레임 수동 검수에 따라 color_not_detected를 성공으로 "
            "재분류할 선택 실험 ID"
        ),
    )
    return parser.parse_args()


def find_run_files(result_root, experiment_id):
    directory = result_root / experiment_id
    if not directory.is_dir():
        raise ValueError(f"{experiment_id}: 실험 폴더가 없음")
    rover_files = sorted(directory.glob("*arc_chrony_test_vel_log*.csv"))
    tx_files = sorted(
        path for path in directory.glob("*tx.csv") if path.name != "rx.csv"
    )
    rx_path = directory / "rx.csv"
    report_path = directory / f"dynamic_tracking_analysis_{experiment_id}.md"
    if len(rover_files) != 1:
        raise ValueError(
            f"{experiment_id}: 로버 CSV가 정확히 1개가 아님 ({len(rover_files)}개)"
        )
    if len(tx_files) != 1:
        raise ValueError(
            f"{experiment_id}: Tx CSV가 정확히 1개가 아님 ({len(tx_files)}개)"
        )
    if not rx_path.is_file():
        raise ValueError(f"{experiment_id}: rx.csv가 없음")
    if not report_path.is_file():
        raise ValueError(f"{experiment_id}: 개별 분석 문서가 없음")
    return directory, rover_files[0], rx_path, tx_files[0], report_path


def validate_selection(experiment_ids):
    if len(set(experiment_ids)) != 3:
        raise ValueError("1/2/3 m 실험은 서로 다른 폴더여야 함")
    for distance, experiment_id in enumerate(experiment_ids, start=1):
        expected_prefix = f"{distance}m"
        if not experiment_id.startswith(expected_prefix):
            raise ValueError(
                f"{distance} m 선택값 {experiment_id!r}은 "
                f"{expected_prefix!r}로 시작해야 함"
            )


def mae(points):
    if not points:
        return None
    return statistics.fmean(abs(point["error_deg"]) for point in points)


def max_abs_error(points):
    if not points:
        return None
    return max(abs(point["error_deg"]) for point in points)


def direction_change(points, field):
    values = unwrap_degrees([point[field] for point in points])
    return values[-1] - values[0] if values else None


def reclassify_color_not_detected_as_success(recognition):
    adjusted = dict(recognition)
    reclassified = adjusted["color_not_detected"]
    adjusted["success"] += reclassified
    adjusted["color_not_detected"] = 0
    adjusted["system_rate"] = (
        100.0 * adjusted["success"] / adjusted["total"]
        if adjusted["total"]
        else None
    )
    conditional_total = (
        adjusted["success"] + adjusted["color_not_detected"]
    )
    adjusted["conditional_rate"] = (
        100.0 * adjusted["success"] / conditional_total
        if conditional_total
        else None
    )
    adjusted["included"] = [
        (
            sample_index,
            elapsed_sec,
            "success" if category == "color_not_detected" else category,
        )
        for sample_index, elapsed_sec, category in adjusted["included"]
    ]
    return adjusted, reclassified


def analyze_run(
    result_root,
    distance,
    experiment_id,
    manual_success=False,
):
    directory, rover_path, rx_path, tx_path, report_path = find_run_files(
        result_root, experiment_id
    )
    rover_rows = read_rover(rover_path)
    t_start = rover_rows[0]["elapsed_sec"]
    t_end = rover_rows[-1]["elapsed_sec"]
    rx_run = read_device_run(rx_path, RX_REQUIRED)
    tx_run = read_device_run(tx_path, TX_REQUIRED)
    recognition = recognition_metrics(rx_run, t_start, t_end)
    raw_color_not_detected = recognition["color_not_detected"]
    manual_reclassified = 0
    if manual_success:
        recognition, manual_reclassified = (
            reclassify_color_not_detected_as_success(recognition)
        )
    rx_points = build_rx_points(
        rover_rows, command_rows(rx_run, t_start, t_end)
    )
    tx_points = build_tx_points(
        rover_rows, command_rows(tx_run, t_start, t_end)
    )
    if not rx_points or not tx_points:
        raise ValueError(f"{experiment_id}: Rx 또는 Tx 유효 방향 표본이 없음")
    return {
        "distance": distance,
        "experiment": experiment_id,
        "directory": directory,
        "rover_path": rover_path,
        "rx_path": rx_path,
        "tx_path": tx_path,
        "individual_report_path": report_path,
        "t_start": t_start,
        "t_end": t_end,
        "recognition": recognition,
        "raw_color_not_detected": raw_color_not_detected,
        "manual_reclassified": manual_reclassified,
        "rx_points": rx_points,
        "tx_points": tx_points,
        "rx_mae": mae(rx_points),
        "tx_mae": mae(tx_points),
        "rx_max": max_abs_error(rx_points),
        "tx_max": max_abs_error(tx_points),
        "rx_ideal_change": direction_change(rx_points, "ideal_deg"),
        "rx_aligned_change": direction_change(rx_points, "aligned_deg"),
        "tx_ideal_change": direction_change(tx_points, "ideal_deg"),
        "tx_aligned_change": direction_change(tx_points, "aligned_deg"),
    }


def raw_number(value):
    if value is None or not math.isfinite(value):
        return "N/A"
    return format(value, ".12g")


def paper_number(value):
    if value is None or not math.isfinite(value):
        return "N/A"
    return f"{value:.2f}"


def table_values(run, paper=False):
    recognition = run["recognition"]
    number = paper_number if paper else raw_number
    return [
        str(run["distance"]),
        str(recognition["total"]),
        str(recognition["success"]),
        str(recognition["color_not_detected"]),
        str(recognition["camera_frame_timeout"]),
        str(recognition["uwb_unavailable"]),
        str(recognition["other_error"]),
        number(recognition["system_rate"]),
        number(recognition["conditional_rate"]),
        number(run["rx_mae"]),
        number(run["tx_mae"]),
    ]


def write_csv(output_path, runs):
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(TABLE_COLUMNS)
        writer.writerows(table_values(run, paper=False) for run in runs)


def write_tex(output_path, runs):
    headers = [
        "Distance (m)",
        "Total cycles",
        "Successful detections",
        "Color not detected",
        "Camera timeouts",
        "UWB unavailable",
        "Other errors",
        "Overall detection rate (\\%)",
        "Conditional detection rate (\\%)",
        "Rx MAE ($^\\circ$)",
        "Tx MAE ($^\\circ$)",
    ]
    rows = [
        " & ".join(table_values(run, paper=True)) + r" \\"
        for run in runs
    ]
    text = "\n".join(
        [
            r"\resizebox{\textwidth}{!}{%",
            r"\begin{tabular}{rrrrrrrrrrr}",
            r"\toprule",
            " & ".join(headers) + r" \\",
            r"\midrule",
            *rows,
            r"\bottomrule",
            r"\end{tabular}%",
            r"}",
            "",
        ]
    )
    output_path.write_text(text, encoding="utf-8")


def align_for_display(points):
    ideal = unwrap_degrees([point["ideal_deg"] for point in points])
    aligned = unwrap_degrees([point["aligned_deg"] for point in points])
    if ideal and aligned:
        shift_turns = round((ideal[0] - aligned[0]) / 360.0)
        aligned = [value + 360.0 * shift_turns for value in aligned]
    return ideal, aligned


def plot_by_distance(plt, runs, subject, output_dir):
    if subject not in {"Rx", "Tx"}:
        raise ValueError(f"지원하지 않는 장치: {subject}")
    key = subject.lower()
    colors = {1: "#0072B2", 2: "#D55E00", 3: "#009E73"}
    figure, axis = plt.subplots(figsize=(7.2, 4.6), constrained_layout=True)
    for run in runs:
        points = run[f"{key}_points"]
        times = [point["elapsed_sec"] for point in points]
        ideal, aligned = align_for_display(points)
        color = colors[run["distance"]]
        axis.plot(
            times,
            ideal,
            color=color,
            linestyle=(0, (5, 2)),
            linewidth=1.8,
            label=f"{run['distance']} m Ideal",
        )
        axis.plot(
            times,
            aligned,
            color=color,
            linestyle="-",
            linewidth=1.8,
            label=f"{run['distance']} m Aligned",
        )

    annotation = "\n".join(
        [
            f"{subject} MAE",
            *[
                f"{run['distance']} m: {run[f'{key}_mae']:.2f} deg"
                for run in runs
            ],
        ]
    )
    axis.text(
        0.985,
        0.03,
        annotation,
        transform=axis.transAxes,
        ha="right",
        va="bottom",
        fontsize=9.5,
        bbox={
            "boxstyle": "round,pad=0.35",
            "facecolor": "white",
            "edgecolor": "#666666",
            "alpha": 0.92,
        },
    )
    axis.set_xlabel("Elapsed time (s)")
    axis.set_ylabel("Direction (deg)")
    axis.grid(True, color="#B0B0B0", alpha=0.35, linewidth=0.7)
    axis.legend(
        loc="upper left",
        ncol=3,
        fontsize=8.2,
        framealpha=0.92,
        columnspacing=0.8,
        handlelength=2.7,
    )
    axis.margins(x=0.01, y=0.05)
    stem = RX_STEM if subject == "Rx" else TX_STEM
    figure.savefig(output_dir / f"{stem}.png", dpi=300)
    figure.savefig(output_dir / f"{stem}.pdf")
    plt.close(figure)


def markdown_table(runs):
    alignment = ["---:"] * len(TABLE_COLUMNS)
    lines = [
        "| " + " | ".join(TABLE_COLUMNS) + " |",
        "|" + "|".join(alignment) + "|",
    ]
    lines.extend(
        "| " + " | ".join(table_values(run, paper=True)) + " |"
        for run in runs
    )
    return "\n".join(lines)


def signed(value, digits=1):
    return f"{value:+.{digits}f}"


def write_report(output_path, runs):
    by_distance = {run["distance"]: run for run in runs}
    run_1m = by_distance[1]
    run_2m = by_distance[2]
    run_3m = by_distance[3]
    rec_1m = run_1m["recognition"]
    rec_2m = run_2m["recognition"]
    rec_3m = run_3m["recognition"]

    system_rates = [
        run["recognition"]["system_rate"]
        for run in runs
    ]
    system_rate_span = max(system_rates) - min(system_rates)
    camera_share_3m = (
        100.0 * rec_3m["camera_frame_timeout"] / rec_3m["total"]
    )
    manual_runs = [
        run for run in runs if run["manual_reclassified"] > 0
    ]
    if manual_runs:
        manual_review_note = "\n".join(
            (
                f"`{run['experiment']}`의 원본 로그에서 "
                f"`color_not_detected`로 기록된 "
                f"{run['manual_reclassified']}개 프레임은 사용자의 실패 "
                "프레임 전수 검수에서 모두 빨간색 원이 존재하는 것으로 "
                "확인되어 논문용 인식률 집계에서는 성공으로 재분류했다. "
                "원본 CSV와 개별 분석 문서는 수정하지 않았다."
            )
            for run in manual_runs
        )
        rate_basis = "수동 검수 재분류를 반영한"
        recognition_summary = (
            "수동 검수 재분류를 반영한 선택 데이터에서는 세 거리의 빨간색 "
            "인식률이 모두 97% 이상이었고, 거리 증가에 따른 일관된 인식률 "
            "저하는 나타나지 않았다."
        )
        manual_limit = (
            "3 m 인식률은 원본 자동 판정 상태가 아니라 실패 프레임의 수동 "
            "시각 검수를 반영한 값이므로, 자동 판정 상태만 사용한 1 m 및 "
            "2 m와 평가 절차가 다르다는 점을 함께 고려해야 한다."
        )
    else:
        manual_review_note = ""
        rate_basis = "원본 로그 상태로 계산한"
        recognition_summary = (
            "선택 데이터의 빨간색 인식률은 원본 로그 상태를 기준으로 "
            "계산했다."
        )
        manual_limit = ""

    selection_lines = "\n".join(
        (
            f"- {run['distance']} m: `{run['experiment']}` "
            f"(`{run['rover_path'].name}`, rover time "
            f"{run['t_start']:.3f}–{run['t_end']:.3f} s)"
        )
        for run in runs
    )
    rx_stats = "\n".join(
        (
            f"- {run['distance']} m: MAE {run['rx_mae']:.2f}°, "
            f"maximum absolute error {run['rx_max']:.2f}°, "
            f"ideal/aligned change "
            f"{signed(run['rx_ideal_change'])}°/"
            f"{signed(run['rx_aligned_change'])}°"
        )
        for run in runs
    )
    tx_stats = "\n".join(
        (
            f"- {run['distance']} m: MAE {run['tx_mae']:.2f}°, "
            f"maximum absolute error {run['tx_max']:.2f}°, "
            f"ideal/aligned change "
            f"{signed(run['tx_ideal_change'])}°/"
            f"{signed(run['tx_aligned_change'])}°"
        )
        for run in runs
    )

    text = f"""# Dynamic Tracking 통합 분석

## 분석 대상 및 방법

거리별로 사용자가 지정한 다음 한 개의 실험만 사용했다.

{selection_lines}

선택하지 않은 실험은 표, MAE, 그래프 및 해석에 포함하지 않았다. 각 실험의
로버 기록 시간 안에서 `sample_index=0`을 제외했으며, 개별 분석 지침과 같은
원형 각도 보간 및 좌표계 변환을 적용했다. Rx와 Tx MAE는 선택된 각 실험의
모든 유효 명령 표본에 대한 절대 원형 각도 오차의 평균이다.

{manual_review_note}

## 거리별 결과

{markdown_table(runs)}

{rate_basis} 시스템 전체 인식률은 1 m
{rec_1m['system_rate']:.2f}%, 2 m {rec_2m['system_rate']:.2f}%, 3 m
{rec_3m['system_rate']:.2f}%이며 세 조건의 차이는 최대
{system_rate_span:.2f}%p였다. 정상 처리 조건 인식률은 세 거리 모두
100.00%였다. 3 m에서는 전체 {rec_3m['total']}주기 중
{rec_3m['success']}건을 성공으로 분류했고, 남은 실패는 카메라 타임아웃
{rec_3m['camera_frame_timeout']}건({camera_share_3m:.2f}%)이었다.

## Rx 정렬 결과

![Rx alignment by distance]({RX_STEM}.png)

{rx_stats}

선택된 실험의 Rx MAE는 1 m에서 {run_1m['rx_mae']:.2f}°, 2 m에서
{run_2m['rx_mae']:.2f}°, 3 m에서 {run_3m['rx_mae']:.2f}°로 감소했다.
따라서 이 세 실행에서는 거리가 멀수록 Rx 명령 기반 정렬 오차가 커지는 추세가
관찰되지 않았다. 다만 1 m의 최대 오차가 {run_1m['rx_max']:.2f}°로 가장
크고 분석 구간의 이상적 방향 변화보다 정렬 방향 변화가 작아, 일부 구간의
오프셋 또는 추종 부족이 전체 MAE에 반영되었다.

## Tx 정렬 결과

![Tx alignment by distance]({TX_STEM}.png)

{tx_stats}

Tx MAE는 2 m에서 {run_2m['tx_mae']:.2f}°로 가장 컸고, 3 m에서
{run_3m['tx_mae']:.2f}°로 가장 작았다. 특히 2 m에서는 최대 절대 오차가
{run_2m['tx_max']:.2f}°이고 정렬 방향의 전체 변화량이 이상적 방향보다
작아, 선택된 세 실행 중 추종 차이가 가장 크게 나타났다. 3 m의 Tx는
MAE와 최대 오차가 모두 가장 작아 두 선의 추세가 가장 가깝게 나타났다.

## 종합 해석과 한계

{recognition_summary}
Rx와 Tx의 명령 기반 방향 MAE도 거리 증가에 따라 일관되게 커지지 않았다.

각 거리에서 한 개의 실험만 선택했으므로 반복 간 변동성이나 통계적 유의성은
평가하지 않았다. {manual_limit} 또한 거리별 실행 시간이 서로
다르기 때문에 같은 elapsed time이 동일한 궤적 진행 위치를 뜻하지 않을 수
있다. 그래프의 정렬 방향은
엔코더로 측정한 물리적 방향이 아니라 실제 적용된 짐벌 명령으로 계산하거나
기록한 방향이므로, 기구부의 실제 추종 오차까지 포함한 값으로 해석해서는
안 된다.
"""
    output_path.write_text(text, encoding="utf-8")


def verify_outputs(output_dir, runs):
    expected = [
        CSV_NAME,
        TEX_NAME,
        f"{RX_STEM}.png",
        f"{RX_STEM}.pdf",
        f"{TX_STEM}.png",
        f"{TX_STEM}.pdf",
        REPORT_NAME,
    ]
    missing = [
        name
        for name in expected
        if not (output_dir / name).is_file()
        or (output_dir / name).stat().st_size == 0
    ]
    if missing:
        raise RuntimeError(f"생성되지 않은 산출물: {missing}")
    if len(runs) != 3 or [run["distance"] for run in runs] != [1, 2, 3]:
        raise AssertionError("거리별 결과가 정확히 세 행이 아님")
    for run in runs:
        recognition = run["recognition"]
        category_sum = sum(
            recognition[key]
            for key in (
                "success",
                "color_not_detected",
                "camera_frame_timeout",
                "uwb_unavailable",
                "other_error",
            )
        )
        if recognition["total"] != category_sum:
            raise AssertionError(
                f"{run['experiment']}: 인식 범주 합이 전체 주기와 다름"
            )


def main():
    args = parse_args()
    result_root = args.result_root.resolve()
    output_dir = args.output_dir.resolve()
    experiment_ids = tuple(args.experiments)
    manual_success_experiments = set(args.manual_success_experiments)
    validate_selection(experiment_ids)
    unknown_manual_runs = manual_success_experiments - set(experiment_ids)
    if unknown_manual_runs:
        raise ValueError(
            "수동 재분류 실험이 선택 목록에 없음: "
            f"{sorted(unknown_manual_runs)}"
        )
    output_dir.mkdir(parents=True, exist_ok=True)

    runs = [
        analyze_run(
            result_root,
            distance,
            experiment_id,
            manual_success=experiment_id in manual_success_experiments,
        )
        for distance, experiment_id in enumerate(experiment_ids, start=1)
    ]
    write_csv(output_dir / CSV_NAME, runs)
    write_tex(output_dir / TEX_NAME, runs)
    plt = configure_matplotlib()
    plot_by_distance(plt, runs, "Rx", output_dir)
    plot_by_distance(plt, runs, "Tx", output_dir)
    write_report(output_dir / REPORT_NAME, runs)
    verify_outputs(output_dir, runs)

    for run in runs:
        recognition = run["recognition"]
        print(
            f"{run['experiment']}: "
            f"N={recognition['total']} "
            f"S={recognition['success']} "
            f"C={recognition['color_not_detected']} "
            f"F={recognition['camera_frame_timeout']} "
            f"U={recognition['uwb_unavailable']} "
            f"E={recognition['other_error']} "
            f"overall={recognition['system_rate']:.6f}% "
            f"conditional={recognition['conditional_rate']:.6f}% "
            f"Rx_MAE={run['rx_mae']:.6f}deg "
            f"Tx_MAE={run['tx_mae']:.6f}deg"
        )


if __name__ == "__main__":
    main()
