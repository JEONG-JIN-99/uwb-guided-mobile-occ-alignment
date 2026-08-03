#!/usr/bin/env python3
"""같은 날짜에 수행한 거리별 빨간색 사전검사 3회를 통합 분석한다."""

from __future__ import annotations

import argparse
import csv
import math
import shutil
from collections import Counter, defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from plot_red_detection_precheck_table import (
    DetectionSummary,
    configure_matplotlib,
    plot_table,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_RESULT_ROOT = PROJECT_ROOT / "result" / "red_detection_precheck"
ALLOWED_STATUSES = {
    "success",
    "color_not_detected",
    "camera_frame_timeout",
}
CONFIG_FIELDS = (
    "crop_scale",
    "interval_s",
    "color_min_area_px",
    "color_min_component_area_px",
    "red_saturation_min",
    "red_value_min",
)
AGGREGATE_NAME = "red_detection_precheck_aggregate.csv"
ANALYSIS_NAME = "red_detection_precheck_analysis.md"
TABLE_STEM = "red_detection_precheck_performance_table"


@dataclass(frozen=True)
class RunSummary:
    path: Path
    distance_m: int
    total: int
    success: int
    color_not_detected: int
    camera_timeout: int
    config: tuple[float, ...]

    @property
    def overall_rate_pct(self):
        return 100.0 * self.success / self.total

    @property
    def frames_received(self):
        return self.success + self.color_not_detected

    @property
    def conditional_rate_pct(self):
        if not self.frames_received:
            return None
        return 100.0 * self.success / self.frames_received


def build_parser():
    parser = argparse.ArgumentParser(
        description="지정 날짜의 빨간색 인식 사전검사 9개 실행을 통합한다."
    )
    parser.add_argument(
        "--date",
        default=datetime.now().strftime("%Y%m%d"),
        help="분석할 실행 날짜(YYYYMMDD, 기본값: 오늘)",
    )
    parser.add_argument(
        "--result-root",
        type=Path,
        default=DEFAULT_RESULT_ROOT,
        help="red_detection_precheck 결과 루트",
    )
    return parser


def parse_float(row, field, csv_path):
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{csv_path}: {field} 값이 유효하지 않습니다") from error
    if not math.isfinite(value):
        raise ValueError(f"{csv_path}: {field} 값이 유한수가 아닙니다")
    return value


def read_run(csv_path, expected_date):
    with csv_path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = {
            "attempt",
            "distance_m",
            "target_color",
            "status",
            "started_at",
            *CONFIG_FIELDS,
        }
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"{csv_path}: 필수 열 누락 {sorted(missing)}")
        rows = list(reader)

    if len(rows) != 100:
        raise ValueError(f"{csv_path}: 행 수 {len(rows)}개, 예상값 100개")

    attempts = []
    distances = set()
    targets = set()
    statuses = []
    configs = set()
    for row in rows:
        try:
            attempts.append(int(row["attempt"]))
        except ValueError as error:
            raise ValueError(f"{csv_path}: attempt 값이 정수가 아닙니다") from error
        distances.add(parse_float(row, "distance_m", csv_path))
        targets.add(row["target_color"].strip().lower())
        status = row["status"].strip()
        statuses.append(status)
        configs.add(tuple(parse_float(row, field, csv_path) for field in CONFIG_FIELDS))
        if not row["started_at"].startswith(
            f"{expected_date[:4]}-{expected_date[4:6]}-{expected_date[6:8]}"
        ):
            raise ValueError(f"{csv_path}: 지정 날짜가 아닌 행이 있습니다")

    if attempts != list(range(1, 101)):
        raise ValueError(f"{csv_path}: attempt 1~100이 순서대로 존재하지 않습니다")
    if len(distances) != 1:
        raise ValueError(f"{csv_path}: 하나의 거리만 포함해야 합니다: {distances}")
    distance_value = distances.pop()
    if distance_value not in (1.0, 2.0, 3.0):
        raise ValueError(f"{csv_path}: 분석 대상이 아닌 거리 {distance_value:g} m")
    if targets != {"red"}:
        raise ValueError(f"{csv_path}: target_color가 red로 통일되지 않았습니다")
    unexpected = set(statuses).difference(ALLOWED_STATUSES)
    if unexpected:
        raise ValueError(f"{csv_path}: 예상하지 않은 상태 {sorted(unexpected)}")
    if len(configs) != 1:
        raise ValueError(f"{csv_path}: 한 실행 안에서 검출 설정이 달라졌습니다")

    counts = Counter(statuses)
    return RunSummary(
        path=csv_path,
        distance_m=int(distance_value),
        total=len(rows),
        success=counts["success"],
        color_not_detected=counts["color_not_detected"],
        camera_timeout=counts["camera_frame_timeout"],
        config=configs.pop(),
    )


def discover_runs(result_root, date):
    paths = sorted(
        result_root.glob(
            f"run_{date}_*_distance_*m_crop_*/red_detection_results.csv"
        )
    )
    runs = [read_run(path, date) for path in paths]
    if len(runs) != 9:
        raise ValueError(f"{date}: 실행 {len(runs)}개 발견, 예상값 9개")

    grouped = defaultdict(list)
    for run in runs:
        grouped[run.distance_m].append(run)
    counts = {distance: len(grouped[distance]) for distance in (1, 2, 3)}
    if counts != {1: 3, 2: 3, 3: 3}:
        raise ValueError(f"거리별 실행 수가 3개씩이 아닙니다: {counts}")

    all_configs = {run.config for run in runs}
    if len(all_configs) != 1:
        raise ValueError("9개 실행의 카메라·색상 검출 설정이 일치하지 않습니다")
    return runs, grouped, all_configs.pop()


def aggregate(grouped):
    summaries = {}
    for distance in (1, 2, 3):
        runs = grouped[distance]
        total = sum(run.total for run in runs)
        success = sum(run.success for run in runs)
        misses = sum(run.color_not_detected for run in runs)
        timeouts = sum(run.camera_timeout for run in runs)
        if total != 300 or total != success + misses + timeouts:
            raise ValueError(
                f"{distance} m: N={total}, S+C+F={success + misses + timeouts}"
            )
        summaries[distance] = DetectionSummary(
            total=total,
            success=success,
            color_not_detected=misses,
            camera_timeout=timeouts,
        )
    return summaries


def write_aggregate_csv(summaries, output_path):
    fields = (
        "distance_m",
        "run_count",
        "total_attempts",
        "successful_detections",
        "color_not_detected",
        "camera_frame_timeouts",
        "frames_received",
        "overall_recognition_rate_pct",
        "frame_conditional_recognition_rate_pct",
    )
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for distance in (1, 2, 3):
            item = summaries[distance]
            writer.writerow(
                {
                    "distance_m": distance,
                    "run_count": 3,
                    "total_attempts": item.total,
                    "successful_detections": item.success,
                    "color_not_detected": item.color_not_detected,
                    "camera_frame_timeouts": item.camera_timeout,
                    "frames_received": item.frames_received,
                    "overall_recognition_rate_pct": (
                        f"{item.overall_rate_pct:.12f}"
                    ),
                    "frame_conditional_recognition_rate_pct": (
                        f"{item.frame_conditional_rate_pct:.12f}"
                    ),
                }
            )


def rate_text(numerator, denominator):
    if denominator == 0:
        return "N/A"
    return f"{numerator}/{denominator} ({100.0 * numerator / denominator:.1f}%)"


def build_interpretation(summaries, grouped):
    all_misses = sum(item.color_not_detected for item in summaries.values())
    all_timeouts = sum(item.camera_timeout for item in summaries.values())
    sentences = []
    if all_misses == 0:
        sentences.append(
            "새 카메라 프레임이 수신된 모든 회차에서 빨간색 표식이 검출되어 "
            "세 거리의 프레임 조건부 인식률이 모두 100.0%였다."
        )
    else:
        sentences.append(
            f"전체 900회에서 색상 미검출이 {all_misses}회 발생했으며, 거리별 "
            "조건부 인식률 차이는 수신 프레임에서의 검출 실패를 반영한다."
        )
    if all_timeouts:
        sentences.append(
            f"전체 인식률 감소에는 총 {all_timeouts}회의 카메라 프레임 "
            "타임아웃이 포함되었다. 따라서 전체 인식률과 조건부 인식률의 "
            "차이는 색상 검출이 아니라 프레임 가용성의 영향을 나타낸다."
        )
    else:
        sentences.append(
            "카메라 프레임 타임아웃이 없어 전체 인식률과 프레임 조건부 "
            "인식률이 동일했다."
        )

    failure_runs = []
    for distance in (1, 2, 3):
        for run in grouped[distance]:
            failures = run.color_not_detected + run.camera_timeout
            if failures:
                failure_runs.append(f"`{run.path.parent.name}` {failures}회")
    if failure_runs:
        sentences.append(
            "실패가 기록된 실행은 " + ", ".join(failure_runs) + "였다."
        )
    else:
        sentences.append("9개 실행 모두 100회 전부 성공했다.")
    return "\n\n".join(sentences)


def write_analysis(output_path, date, runs, grouped, summaries, config):
    selected_lines = []
    for distance in (1, 2, 3):
        selected_lines.append(f"### {distance} m")
        selected_lines.extend(
            f"- `{run.path.relative_to(PROJECT_ROOT)}`" for run in grouped[distance]
        )
        selected_lines.append("")

    run_rows = []
    for distance in (1, 2, 3):
        for index, run in enumerate(grouped[distance], start=1):
            conditional = (
                "N/A"
                if run.conditional_rate_pct is None
                else f"{run.conditional_rate_pct:.1f}%"
            )
            run_rows.append(
                f"| {distance} | {index} | `{run.path.parent.name}` | "
                f"{run.total} | {run.success} | {run.color_not_detected} | "
                f"{run.camera_timeout} | {run.overall_rate_pct:.1f}% | "
                f"{conditional} |"
            )

    aggregate_rows = []
    for distance in (1, 2, 3):
        item = summaries[distance]
        aggregate_rows.append(
            f"| {distance} | {item.total} | {item.success} | "
            f"{item.color_not_detected} | {item.camera_timeout} | "
            f"{item.overall_rate_pct:.1f}% | "
            f"{item.frame_conditional_rate_pct:.1f}% |"
        )

    config_rows = "\n".join(
        f"| `{field}` | {value:g} |" for field, value in zip(CONFIG_FIELDS, config)
    )
    interpretation = build_interpretation(summaries, grouped)
    text = f"""# 빨간색 인식 사전검사 통합 분석

## 분석 범위

- 실행 날짜: `{date}`
- 분석 실행: 9개(거리별 3개)
- 실행당 시도: 100회
- 전체 시도: 900회
- 제외 데이터: 매칭되는 날짜가 아닌 기존 실행

{chr(10).join(selected_lines)}
## 입력 검증

- 모든 실행에 `attempt=1..100`이 중복과 누락 없이 존재했다.
- 모든 행의 대상 색상은 `red`였다.
- 모든 상태는 `success`, `color_not_detected`,
  `camera_frame_timeout` 중 하나였다.
- 거리별 실행 수는 정확히 3개이며 거리별 합산 분모는 300회다.
- 9개 실행의 기록된 검출 설정은 다음과 같이 일치했다.

| 설정 | 값 |
|---|---:|
{config_rows}

카메라 장치 번호, 워밍업 시간, 조명 및 표식 자세는 현재 CSV에 기록되지 않으므로
파일만으로 재검증할 수 없다. 이 항목들은 동일하게 유지했다는 실험자의 조건을
전제로 한다.

## 실행별 결과

| Distance (m) | Replicate | Run | Total | Success | Color misses | Camera timeouts | Overall rate | Frame-conditional rate |
|---:|---:|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(run_rows)}

## 거리별 합산 결과

| Distance (m) | Total attempts | Successful detections | Color not detected | Camera frame timeouts | Overall recognition rate | Frame-conditional recognition rate |
|---:|---:|---:|---:|---:|---:|---:|
{chr(10).join(aggregate_rows)}

전체 인식률은 `S/N`, 프레임 조건부 인식률은 카메라 타임아웃을 제외한
`S/(S+C)`로 계산했다. 같은 거리의 세 실행은 백분율을 평균하지 않고 원시
건수를 먼저 합산했다.

## 논문용 표

![Red detection precheck performance](red_detection_precheck_performance_table.png)

## 결과 해석

{interpretation}

거리별 결과는 기본 빨간색 검출 성능과 카메라 프레임 가용성을 분리해 보여준다.
단, 세 거리의 촬영 시각과 물리적 환경이 완전히 동일했는지는 로그만으로 확인할
수 없으므로 관찰된 차이를 거리 자체의 직접적인 영향으로 단정하지 않는다.

## 정적 정렬 실험 해석에 사용하는 방법

프레임 조건부 인식률이 높다면 이후 정적 정렬 실험에서 발생하는
`color_not_detected`는 고정 표식에 대한 기본 검출 한계뿐 아니라 정렬 동작,
영상 획득 시점, 모션 블러 및 시야 내 표식 위치와 함께 해석해야 한다. 전체
인식률의 감소가 타임아웃에 의한 경우에는 색상 검출 실패와 구분해 보고한다.
"""
    output_path.write_text(text, encoding="utf-8")


def generate_table(summaries, result_root):
    plt = configure_matplotlib(language="en")
    generated_png, generated_pdf = plot_table(
        plt,
        summaries,
        result_root,
        language="en",
    )
    final_png = result_root / f"{TABLE_STEM}.png"
    final_pdf = result_root / f"{TABLE_STEM}.pdf"
    shutil.move(generated_png, final_png)
    shutil.move(generated_pdf, final_pdf)
    return final_png, final_pdf


def main(argv=None):
    args = build_parser().parse_args(argv)
    if len(args.date) != 8 or not args.date.isdigit():
        raise SystemExit("--date는 YYYYMMDD 형식이어야 합니다")
    result_root = args.result_root.resolve()
    try:
        runs, grouped, config = discover_runs(result_root, args.date)
        summaries = aggregate(grouped)
    except ValueError as error:
        raise SystemExit(f"검증 실패: {error}") from error

    aggregate_path = result_root / AGGREGATE_NAME
    analysis_path = result_root / ANALYSIS_NAME
    write_aggregate_csv(summaries, aggregate_path)
    table_png, table_pdf = generate_table(summaries, result_root)
    write_analysis(
        analysis_path,
        args.date,
        runs,
        grouped,
        summaries,
        config,
    )

    for distance in (1, 2, 3):
        item = summaries[distance]
        print(
            f"{distance} m: S={item.success}, C={item.color_not_detected}, "
            f"F={item.camera_timeout}, overall={item.overall_rate_pct:.1f}%, "
            f"conditional={item.frame_conditional_rate_pct:.1f}%"
        )
    print(aggregate_path)
    print(table_png)
    print(table_pdf)
    print(analysis_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
