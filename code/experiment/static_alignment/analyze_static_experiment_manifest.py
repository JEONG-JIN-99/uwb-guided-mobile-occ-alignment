#!/usr/bin/env python3
"""매니페스트로 고정된 18개 정적 정렬 실행의 논문용 분석물을 생성한다."""

import argparse
import csv
import math
import os
import statistics
import tempfile
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MANIFEST = (
    PROJECT_ROOT
    / "result"
    / "static_alignment"
    / "static_alignment_analysis_manifest.csv"
)
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "result" / "static_alignment"
CONDITIONS = ("no_stabilization", "stabilization_0.5s")
CONDITION_LABELS = {
    "no_stabilization": "No stabilization",
    "stabilization_0.5s": "0.5 s stabilization",
}
CONDITION_COLORS = {
    "no_stabilization": "#FF0000",
    "stabilization_0.5s": "#0000FF",
}
CONDITION_MARKERS = {"no_stabilization": "o", "stabilization_0.5s": "s"}
BIN_EDGES = (0, 10, 20, 30, 40, 50)
BIN_LABELS = tuple(
    f"{lower}–{upper}" for lower, upper in zip(BIN_EDGES, BIN_EDGES[1:])
)
ALLOWED_STATUSES = {"success", "color_not_detected", "camera_frame_timeout"}
FIXED_CONFIG_FIELDS = (
    "interval_s",
    "crop_scale",
    "color_min_area_px",
    "color_min_component_area_px",
    "red_saturation_min",
    "red_value_min",
    "target_color",
)


@dataclass(frozen=True)
class ManifestEntry:
    distance_m: int
    condition: str
    settle_time_s: float
    replicate: int
    reference_deg: float
    relative_path: str
    csv_path: Path


@dataclass(frozen=True)
class Trial:
    entry: ManifestEntry
    row: dict
    attempt: int
    status: str
    initial_angle_deg: float
    command_angle_deg: float | None
    signed_error_deg: float | None
    absolute_error_deg: float | None
    servo_clipped: bool


def parse_finite(value, field, source, allow_blank=False):
    text = str(value).strip()
    if allow_blank and not text:
        return None
    try:
        number = float(text)
    except ValueError as error:
        raise ValueError(f"{source}: {field} 값이 숫자가 아닙니다: {text!r}") from error
    if not math.isfinite(number):
        raise ValueError(f"{source}: {field} 값이 유한수가 아닙니다")
    return number


def wrap_degrees(angle):
    return ((angle + 180.0) % 360.0) - 180.0


def angle_bin(angle):
    value = abs(angle)
    for lower, upper, label in zip(BIN_EDGES, BIN_EDGES[1:], BIN_LABELS):
        # 실험 범위의 최댓값인 50°는 마지막 40–50° 구간에 포함한다.
        if lower <= value < upper or (upper == BIN_EDGES[-1] and value == upper):
            return label
    return None


def percentile(values, probability):
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * probability
    lower_index = math.floor(position)
    upper_index = math.ceil(position)
    if lower_index == upper_index:
        return ordered[lower_index]
    fraction = position - lower_index
    return ordered[lower_index] * (1.0 - fraction) + ordered[upper_index] * fraction


def fmt_number(value, digits=3):
    if value is None:
        return "N/A"
    return f"{value:.{digits}f}"


def fmt_pct(value):
    if value is None:
        return "N/A"
    return f"{value:.1f}%"


def read_manifest(path, project_root):
    required = {
        "distance_m",
        "stabilization_condition",
        "settle_time_s",
        "replicate",
        "reference_tx_ros_deg",
        "csv_path",
    }
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"매니페스트 필수 열 누락: {sorted(missing)}")
        raw_entries = list(reader)
    if len(raw_entries) != 18:
        raise ValueError(f"매니페스트는 18행이어야 합니다: 현재 {len(raw_entries)}행")

    entries = []
    paths = set()
    groups = defaultdict(list)
    for raw in raw_entries:
        distance = int(parse_finite(raw["distance_m"], "distance_m", path))
        condition = raw["stabilization_condition"].strip()
        settle = parse_finite(raw["settle_time_s"], "settle_time_s", path)
        replicate = int(parse_finite(raw["replicate"], "replicate", path))
        reference = parse_finite(
            raw["reference_tx_ros_deg"], "reference_tx_ros_deg", path
        )
        relative = raw["csv_path"].strip()
        absolute = (project_root / relative).resolve()
        if distance not in (1, 2, 3):
            raise ValueError(f"분석 대상이 아닌 거리: {distance} m")
        if condition not in CONDITIONS:
            raise ValueError(f"알 수 없는 안정화 조건: {condition}")
        expected_settle = 0.0 if condition == "no_stabilization" else 0.5
        if not math.isclose(settle, expected_settle, abs_tol=1e-9):
            raise ValueError(f"{relative}: 조건과 안정화 시간이 일치하지 않습니다")
        if relative in paths:
            raise ValueError(f"중복 CSV 경로: {relative}")
        if not absolute.is_file():
            raise FileNotFoundError(absolute)
        paths.add(relative)
        groups[(distance, condition)].append(replicate)
        entries.append(
            ManifestEntry(
                distance,
                condition,
                settle,
                replicate,
                reference,
                relative,
                absolute,
            )
        )
    for distance in (1, 2, 3):
        for condition in CONDITIONS:
            replicates = sorted(groups[(distance, condition)])
            if replicates != [1, 2, 3]:
                raise ValueError(
                    f"{distance} m/{condition}: 반복 번호가 1,2,3이 아닙니다: {replicates}"
                )
    return sorted(entries, key=lambda e: (e.distance_m, CONDITIONS.index(e.condition), e.replicate))


def read_trials(entries):
    trials = []
    config_values = {field: set() for field in FIXED_CONFIG_FIELDS}
    run_summaries = []
    for entry in entries:
        with entry.csv_path.open(newline="", encoding="utf-8-sig") as stream:
            reader = csv.DictReader(stream)
            required = {
                "attempt",
                "distance_m",
                "pre_recognition_settle_time_s",
                "target_color",
                "status",
                "initial_gimbal_ros_deg",
                "gimbal_command_ros_deg",
                "servo_clipped",
                *FIXED_CONFIG_FIELDS,
            }
            missing = required.difference(reader.fieldnames or ())
            if missing:
                raise ValueError(f"{entry.csv_path}: 필수 열 누락 {sorted(missing)}")
            rows = list(reader)
        if len(rows) != 100:
            raise ValueError(f"{entry.csv_path}: 100행이 아니라 {len(rows)}행입니다")
        attempts = [int(row["attempt"]) for row in rows]
        if attempts != list(range(1, 101)):
            raise ValueError(f"{entry.csv_path}: attempt 1~100 순서가 유효하지 않습니다")

        statuses = Counter()
        for row in rows:
            distance = parse_finite(row["distance_m"], "distance_m", entry.csv_path)
            settle = parse_finite(
                row["pre_recognition_settle_time_s"],
                "pre_recognition_settle_time_s",
                entry.csv_path,
            )
            if not math.isclose(distance, entry.distance_m, abs_tol=1e-9):
                raise ValueError(f"{entry.csv_path}: 매니페스트와 거리 불일치")
            if not math.isclose(settle, entry.settle_time_s, abs_tol=1e-9):
                raise ValueError(f"{entry.csv_path}: 매니페스트와 안정화 시간 불일치")
            status = row["status"].strip()
            if status not in ALLOWED_STATUSES:
                raise ValueError(f"{entry.csv_path}: 예상하지 않은 상태 {status!r}")
            if row["target_color"].strip().lower() != "red":
                raise ValueError(f"{entry.csv_path}: target_color가 red가 아닙니다")
            for field in FIXED_CONFIG_FIELDS:
                config_values[field].add(row[field].strip())
            initial = parse_finite(
                row["initial_gimbal_ros_deg"], "initial_gimbal_ros_deg", entry.csv_path
            )
            command = parse_finite(
                row["gimbal_command_ros_deg"],
                "gimbal_command_ros_deg",
                entry.csv_path,
                allow_blank=True,
            )
            signed = None if command is None else wrap_degrees(command - entry.reference_deg)
            absolute = None if signed is None else abs(signed)
            clipped = row["servo_clipped"].strip().lower() in {"1", "true", "yes"}
            trials.append(
                Trial(
                    entry,
                    row,
                    int(row["attempt"]),
                    status,
                    initial,
                    command,
                    signed,
                    absolute,
                    clipped,
                )
            )
            statuses[status] += 1
        run_summaries.append((entry, statuses))

    for field, values in config_values.items():
        if len(values) != 1:
            raise ValueError(f"18개 실행에서 공통 설정 {field}가 다릅니다: {sorted(values)}")
    return trials, run_summaries, {field: next(iter(values)) for field, values in config_values.items()}


def group_trials(trials):
    grouped = defaultdict(list)
    for trial in trials:
        grouped[(trial.entry.distance_m, trial.entry.condition)].append(trial)
    return grouped


def recognition_summaries(grouped):
    summaries = []
    for distance in (1, 2, 3):
        for condition in CONDITIONS:
            values = grouped[(distance, condition)]
            counts = Counter(trial.status for trial in values)
            total = len(values)
            success = counts["success"]
            misses = counts["color_not_detected"]
            timeouts = counts["camera_frame_timeout"]
            frames = success + misses
            summaries.append(
                {
                    "distance_m": distance,
                    "stabilization_condition": condition,
                    "settle_time_s": 0.0 if condition == "no_stabilization" else 0.5,
                    "run_count": 3,
                    "total_attempts": total,
                    "successful_detections": success,
                    "color_not_detected": misses,
                    "camera_frame_timeouts": timeouts,
                    "frames_received": frames,
                    "overall_recognition_rate_pct": 100.0 * success / total,
                    "frame_conditional_recognition_rate_pct": (
                        None if frames == 0 else 100.0 * success / frames
                    ),
                }
            )
    return summaries


def alignment_summaries(grouped):
    summaries = []
    for distance in (1, 2, 3):
        for condition in CONDITIONS:
            trials = grouped[(distance, condition)]
            values = [t.absolute_error_deg for t in trials if t.absolute_error_deg is not None]
            q1 = percentile(values, 0.25)
            median = percentile(values, 0.5)
            q3 = percentile(values, 0.75)
            summaries.append(
                {
                    "distance_m": distance,
                    "stabilization_condition": condition,
                    "settle_time_s": 0.0 if condition == "no_stabilization" else 0.5,
                    "total_attempts": len(trials),
                    "valid_alignment_samples": len(values),
                    "excluded_missing_command": len(trials) - len(values),
                    "servo_clipped_count": sum(t.servo_clipped for t in trials),
                    "mae_deg": statistics.fmean(values) if values else None,
                    "median_deg": median,
                    "q1_deg": q1,
                    "q3_deg": q3,
                    "iqr_deg": None if q1 is None else q3 - q1,
                    "p95_deg": percentile(values, 0.95),
                    "max_deg": max(values) if values else None,
                }
            )
    return summaries


def no_settle_bins(trials):
    output = []
    for distance in (1, 2, 3):
        distance_trials = [
            t for t in trials
            if t.entry.distance_m == distance and t.entry.condition == "no_stabilization"
        ]
        for label in BIN_LABELS:
            values = [t for t in distance_trials if angle_bin(t.initial_angle_deg) == label]
            counts = Counter(t.status for t in values)
            success = counts["success"]
            misses = counts["color_not_detected"]
            timeouts = counts["camera_frame_timeout"]
            frames = success + misses
            output.append(
                {
                    "distance_m": distance,
                    "initial_abs_angle_bin_deg": label,
                    "total_attempts": len(values),
                    "successful_detections": success,
                    "color_not_detected": misses,
                    "camera_frame_timeouts": timeouts,
                    "frames_received": frames,
                    "frame_conditional_recognition_rate_pct": (
                        None if frames == 0 else 100.0 * success / frames
                    ),
                }
            )
    return output


def alignment_bins(trials):
    output = []
    for distance in (1, 2, 3):
        for condition in CONDITIONS:
            group = [
                t for t in trials
                if t.entry.distance_m == distance and t.entry.condition == condition
            ]
            for label in BIN_LABELS:
                candidates = [t for t in group if angle_bin(t.initial_angle_deg) == label]
                values = [
                    t.absolute_error_deg
                    for t in candidates
                    if t.absolute_error_deg is not None
                ]
                output.append(
                    {
                        "distance_m": distance,
                        "stabilization_condition": condition,
                        "initial_abs_angle_bin_deg": label,
                        "total_attempts": len(candidates),
                        "valid_alignment_samples": len(values),
                        "excluded_missing_command": len(candidates) - len(values),
                        "mean_absolute_alignment_error_deg": (
                            statistics.fmean(values) if values else None
                        ),
                    }
                )
    return output


def write_csv(path, rows, fieldnames):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    key: ("" if value is None else value)
                    for key, value in row.items()
                }
            )


def write_detail_csv(path, trials):
    fields = (
        "distance_m",
        "stabilization_condition",
        "settle_time_s",
        "replicate",
        "source_csv",
        "attempt",
        "status",
        "initial_gimbal_ros_deg",
        "initial_abs_angle_bin_deg",
        "gimbal_command_ros_deg",
        "reference_tx_ros_deg",
        "signed_alignment_error_deg",
        "absolute_alignment_error_deg",
        "alignment_error_included",
        "alignment_error_exclusion_reason",
        "servo_clipped",
    )
    rows = []
    for trial in trials:
        included = trial.absolute_error_deg is not None
        rows.append(
            {
                "distance_m": trial.entry.distance_m,
                "stabilization_condition": trial.entry.condition,
                "settle_time_s": trial.entry.settle_time_s,
                "replicate": trial.entry.replicate,
                "source_csv": trial.entry.relative_path,
                "attempt": trial.attempt,
                "status": trial.status,
                "initial_gimbal_ros_deg": trial.initial_angle_deg,
                "initial_abs_angle_bin_deg": angle_bin(trial.initial_angle_deg) or "outside_0_50",
                "gimbal_command_ros_deg": trial.command_angle_deg,
                "reference_tx_ros_deg": trial.entry.reference_deg,
                "signed_alignment_error_deg": trial.signed_error_deg,
                "absolute_alignment_error_deg": trial.absolute_error_deg,
                "alignment_error_included": int(included),
                "alignment_error_exclusion_reason": "" if included else "missing_command",
                "servo_clipped": int(trial.servo_clipped),
            }
        )
    write_csv(path, rows, fields)


def configure_matplotlib():
    os.environ.setdefault(
        "MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "uwb_alignment_matplotlib")
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 26,
            "axes.labelsize": 30,
            "axes.titlesize": 32,
            "legend.fontsize": 24,
            "xtick.labelsize": 24,
            "ytick.labelsize": 24,
            "axes.unicode_minus": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    return plt


def plot_recognition_table(plt, summaries, output_dir):
    rows = []
    for s in summaries:
        rows.append(
            [
                f"{s['distance_m']} m",
                CONDITION_LABELS[s["stabilization_condition"]],
                f"{s['successful_detections']}/{s['total_attempts']}",
                str(s["color_not_detected"]),
                str(s["camera_frame_timeouts"]),
                fmt_pct(s["overall_recognition_rate_pct"]),
                f"{s['successful_detections']}/{s['frames_received']} "
                f"({fmt_pct(s['frame_conditional_recognition_rate_pct'])})",
            ]
        )
    figure, axis = plt.subplots(figsize=(13.8, 4.8))
    axis.axis("off")
    table = axis.table(
        cellText=rows,
        colLabels=(
            "Distance",
            "Stabilization\ncondition",
            "Successful\ndetections",
            "Color\nmisses",
            "Camera\ntimeouts",
            "Overall\nrecognition rate",
            "Frame-conditional\nrecognition rate",
        ),
        cellLoc="center",
        colLoc="center",
        colWidths=(0.08, 0.17, 0.13, 0.11, 0.13, 0.15, 0.23),
        bbox=(0.015, 0.04, 0.97, 0.92),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9.0)
    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor("#FFFFFF")
        cell.set_linewidth(1.4)
        if row == 0:
            cell.set_facecolor("#315A75")
            cell.get_text().set_color("white")
            cell.get_text().set_fontweight("bold")
        else:
            settled = summaries[row - 1]["stabilization_condition"] == "stabilization_0.5s"
            cell.set_facecolor("#E7F1F5" if settled else "#F8EEE5")
            if column in (0, 1):
                cell.get_text().set_fontweight("bold")
    png = output_dir / "static_alignment_stabilization_recognition_table_en.png"
    pdf = output_dir / "static_alignment_stabilization_recognition_table_en.pdf"
    figure.savefig(png, dpi=300, bbox_inches="tight", pad_inches=0.10, facecolor="white")
    figure.savefig(pdf, bbox_inches="tight", pad_inches=0.10, facecolor="white")
    plt.close(figure)


def plot_alignment_boxplot(plt, grouped, output_dir):
    figure, axis = plt.subplots(figsize=(12.5, 7.5))
    legend_handles = []
    for condition, offset in zip(CONDITIONS, (-0.18, 0.18)):
        values = [
            [
                t.absolute_error_deg
                for t in grouped[(distance, condition)]
                if t.absolute_error_deg is not None
            ]
            for distance in (1, 2, 3)
        ]
        positions = [distance + offset for distance in (1, 2, 3)]
        color = CONDITION_COLORS[condition]
        parts = axis.boxplot(
            values,
            positions=positions,
            widths=0.30,
            patch_artist=True,
            whis=1.5,
            showfliers=True,
            boxprops={"facecolor": color, "edgecolor": color, "alpha": 0.24, "linewidth": 1.7},
            medianprops={"color": color, "linewidth": 2.2},
            whiskerprops={"color": color, "linewidth": 1.5},
            capprops={"color": color, "linewidth": 1.5},
            flierprops={"marker": ".", "markerfacecolor": color, "markeredgecolor": color, "alpha": 0.55, "markersize": 5},
        )
        means = [statistics.fmean(group) for group in values]
        axis.scatter(
            positions,
            means,
            marker="o",
            s=70,
            facecolor=color,
            edgecolor="white",
            linewidth=1.0,
            zorder=4,
        )
        legend_handles.append(parts["boxes"][0])
    from matplotlib.lines import Line2D
    legend_handles.append(
        Line2D([0], [0], marker="o", color="none", markerfacecolor="#555555", markeredgecolor="white", markersize=8, label="MAE")
    )
    axis.set_xlabel("Distance (m)", fontsize=45)
    axis.set_ylabel("Alignment error (°)", fontsize=45)
    axis.set_xticks((1, 2, 3), ("1", "2", "3"))
    axis.tick_params(axis="both", labelsize=36)
    axis.grid(axis="y", color="#B8B8B8", linewidth=0.7, alpha=0.45)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.legend(
        legend_handles,
        ("No stabilization", "0.5 s stabilization", "MAE"),
        loc="upper left",
        frameon=True,
    )
    figure.subplots_adjust(left=0.23, right=0.97, top=0.96, bottom=0.22)
    png = output_dir / "static_alignment_alignment_error_boxplot_en.png"
    pdf = output_dir / "static_alignment_alignment_error_boxplot_en.pdf"
    figure.savefig(png, dpi=300, facecolor="white")
    figure.savefig(pdf, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def plot_no_settle_rates(plt, rows, output_dir):
    figure, axis = plt.subplots(figsize=(12.5, 7.5))
    colors = {1: "#FF0000", 2: "#0000FF", 3: "#000000"}
    markers = {1: "o", 2: "s", 3: "^"}
    offsets = {1: -0.10, 2: 0.0, 3: 0.10}
    for distance in (1, 2, 3):
        selected = [r for r in rows if r["distance_m"] == distance]
        positions = [index + offsets[distance] for index in range(len(selected))]
        values = [r["frame_conditional_recognition_rate_pct"] for r in selected]
        axis.plot(
            positions,
            values,
            color=colors[distance],
            marker=markers[distance],
            markersize=8.7,
            markeredgecolor="white",
            markeredgewidth=1.05,
            linewidth=2.7,
            label=f"{distance} m",
            zorder=3,
        )
    axis.set_xlabel("Absolute initial-angle bin (°)", fontsize=44)
    axis.set_ylabel("Recognition\nrate (%)", fontsize=44)
    axis.set_xticks(range(len(BIN_LABELS)), BIN_LABELS)
    axis.tick_params(axis="y", labelsize=38)
    axis.tick_params(axis="x", labelsize=34)
    axis.set_xlim(-0.35, len(BIN_LABELS) - 0.65)
    axis.set_ylim(-8, 113)
    axis.set_yticks(range(0, 101, 20))
    axis.grid(axis="y", color="#B8B8B8", linewidth=0.7, alpha=0.55)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.legend(title="Distance", loc="lower left", frameon=True, framealpha=0.92, fontsize=36, title_fontsize=36)
    figure.subplots_adjust(left=0.23, right=0.97, top=0.96, bottom=0.22)
    png = output_dir / "static_alignment_no_settle_recognition_rate_en_wide_large.png"
    pdf = output_dir / "static_alignment_no_settle_recognition_rate_en_wide_large.pdf"
    figure.savefig(png, dpi=300, facecolor="white")
    figure.savefig(pdf, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def plot_alignment_bins(plt, rows, output_dir):
    figure, axes = plt.subplots(1, 3, figsize=(19.5, 7.2), sharey=True)
    centers = list(range(1, len(BIN_LABELS) + 1))
    offsets = {"no_stabilization": -0.09, "stabilization_0.5s": 0.09}
    for axis, distance in zip(axes, (1, 2, 3)):
        for condition in CONDITIONS:
            selected = [
                row for row in rows
                if row["distance_m"] == distance and row["stabilization_condition"] == condition
            ]
            positions = [center + offsets[condition] for center in centers]
            values = [row["mean_absolute_alignment_error_deg"] for row in selected]
            axis.plot(
                positions,
                values,
                color=CONDITION_COLORS[condition],
                marker=CONDITION_MARKERS[condition],
                markersize=8,
                markeredgecolor="white",
                markeredgewidth=0.8,
                linewidth=2.4,
                label=CONDITION_LABELS[condition],
            )
        axis.set_title(f"Distance = {distance} m", pad=8)
        axis.set_xticks(centers, BIN_LABELS, rotation=25)
        axis.grid(axis="y", alpha=0.25)
    figure.supxlabel("Absolute initial-angle bin (°)", fontsize=30, y=0.045)
    figure.supylabel("Mean absolute\nalignment error (°)", fontsize=30, x=0.018)
    axes[-1].legend(loc="upper left")
    figure.subplots_adjust(left=0.11, right=0.985, top=0.84, bottom=0.27, wspace=0.16)
    png = output_dir / "alignment_accuracy_initial_angle_mean_en.png"
    pdf = output_dir / "alignment_accuracy_initial_angle_mean_en.pdf"
    figure.savefig(png, dpi=300, facecolor="white")
    figure.savefig(pdf, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def read_precheck(project_root):
    path = project_root / "result" / "red_detection_precheck" / "red_detection_precheck_aggregate.csv"
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def write_analysis(
    path,
    manifest_path,
    recognition,
    alignment,
    run_summaries,
    rate_bins,
    error_bins,
    config,
    precheck,
):
    recognition_lookup = {
        (row["distance_m"], row["stabilization_condition"]): row
        for row in recognition
    }
    alignment_lookup = {
        (row["distance_m"], row["stabilization_condition"]): row
        for row in alignment
    }
    lines = [
        "# 정적 정렬 실험 통합 분석",
        "",
        "## 1. 분석 범위와 검증",
        "",
        f"- 입력 매니페스트: `{manifest_path.relative_to(PROJECT_ROOT)}`",
        "- 분석 실행: 18개(3개 거리 × 2개 안정화 조건 × 3회 반복)",
        "- 전체 시도: 1,800회(실행별 100회)",
        "- 매니페스트 밖의 과거 실행: 분석에서 제외",
        "- 모든 CSV에서 거리, 안정화 시간, 빨간색 표적, 시도 번호 1–100 검증 완료",
        "- 18개 실행의 기록된 공통 검출 설정 일치 확인",
        "",
        "공통 기록 설정: "
        + ", ".join(f"`{key}={value}`" for key, value in config.items())
        + ".",
        "",
        "## 2. 인식 성능",
        "",
        "| Distance | Stabilization | Success | Color misses | Camera timeouts | Overall rate | Frame-conditional rate |",
        "|---:|---|---:|---:|---:|---:|---:|",
    ]
    for row in recognition:
        lines.append(
            f"| {row['distance_m']} m | {CONDITION_LABELS[row['stabilization_condition']]} "
            f"| {row['successful_detections']}/{row['total_attempts']} "
            f"| {row['color_not_detected']} | {row['camera_frame_timeouts']} "
            f"| {fmt_pct(row['overall_recognition_rate_pct'])} "
            f"| {row['successful_detections']}/{row['frames_received']} "
            f"({fmt_pct(row['frame_conditional_recognition_rate_pct'])}) |"
        )
    lines += ["", "### 안정화 효과", ""]
    for distance in (1, 2, 3):
        no = recognition_lookup[(distance, "no_stabilization")]
        settled = recognition_lookup[(distance, "stabilization_0.5s")]
        overall_gain = settled["overall_recognition_rate_pct"] - no["overall_recognition_rate_pct"]
        conditional_gain = settled["frame_conditional_recognition_rate_pct"] - no["frame_conditional_recognition_rate_pct"]
        lines.append(
            f"- {distance} m: 0.5초 안정화 후 전체 인식률은 "
            f"{fmt_pct(no['overall_recognition_rate_pct'])}에서 {fmt_pct(settled['overall_recognition_rate_pct'])}로 "
            f"{overall_gain:.1f}%p 증가했고, 프레임 조건부 인식률은 "
            f"{fmt_pct(no['frame_conditional_recognition_rate_pct'])}에서 "
            f"{fmt_pct(settled['frame_conditional_recognition_rate_pct'])}로 {conditional_gain:.1f}%p 증가했다."
        )
    lines += [
        "",
        "안정화 조건에서는 세 거리 모두 색상 미검출이 0회였다. 남은 실패는 카메라 프레임 타임아웃이며, "
        "따라서 안정화 조건의 프레임 조건부 인식률은 모두 100%이다. 이는 정렬 명령 직후의 촬영 시점이 "
        "색상 검출에 영향을 주었을 가능성과 일치하지만, 실패 프레임만으로 원인을 모션 블러라고 단정할 수는 없다.",
        "",
        "무안정화 전체 인식률은 67.7–70.0%, 안정화 전체 인식률은 96.0–98.3% 범위로 거리 간 차이보다 "
        "안정화 조건 간 차이가 훨씬 컸다. 두 조건 모두 거리 증가에 따른 단조 감소가 나타나지 않았으므로 "
        "이번 세 거리 결과만으로 거리 자체의 악화 효과를 주장하지 않는다.",
        "",
        "### 반복 실행 일관성",
        "",
        "| Distance | Stabilization | Replicate | Success | Color misses | Camera timeouts |",
        "|---:|---|---:|---:|---:|---:|",
    ]
    for entry, counts in run_summaries:
        lines.append(
            f"| {entry.distance_m} m | {CONDITION_LABELS[entry.condition]} | {entry.replicate} "
            f"| {counts['success']} | {counts['color_not_detected']} | {counts['camera_frame_timeout']} |"
        )
    lines += [
        "",
        "반복별 성공 건수의 범위는 무안정화 조건에서 66–71회, 0.5초 안정화 조건에서 94–99회이다. "
        "안정화 조건의 가장 낮은 성공 건수는 1 m와 2 m의 첫 반복에서 각각 94회였으며, 이 실패는 모두 카메라 타임아웃이었다.",
        "",
        "## 3. 명령 기반 정렬오차",
        "",
        "오차는 `abs(wrap(gimbal_command_ros_deg - reference_tx_ros_deg))`로 계산했다. "
        "색상 인식 결과와 무관하게 유효한 명령값을 모두 포함했다.",
        "",
        "| Distance | Stabilization | n | Excluded | Clipped | MAE (°) | Median (°) | Q1 (°) | Q3 (°) | IQR (°) | P95 (°) | Max (°) |",
        "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in alignment:
        lines.append(
            f"| {row['distance_m']} m | {CONDITION_LABELS[row['stabilization_condition']]} "
            f"| {row['valid_alignment_samples']} | {row['excluded_missing_command']} | {row['servo_clipped_count']} "
            f"| {fmt_number(row['mae_deg'])} | {fmt_number(row['median_deg'])} "
            f"| {fmt_number(row['q1_deg'])} | {fmt_number(row['q3_deg'])} "
            f"| {fmt_number(row['iqr_deg'])} | {fmt_number(row['p95_deg'])} | {fmt_number(row['max_deg'])} |"
        )
    lines += [
        "",
        "모든 조건에서 유효 명령 표본 수와 제외 건수는 위 표에 명시했다. 0.5초 안정화는 명령 이후의 "
        "인식 대기시간일 뿐 기록된 명령각을 직접 변경하는 처리가 아니다. 그러므로 조건 간 명령오차 차이는 "
        "물리적 안정화 정확도 개선의 증거로 해석하지 않는다. 이 지표는 엔코더 실측 자세가 아닌 "
        "command-based alignment error이다.",
        "",
        "조건별 MAE는 5.549–5.800°로 좁은 범위였고 모든 조건에서 명령값 누락과 서보 제한은 0회였다. "
        "1 m 무안정화 조건은 IQR과 최댓값이 상대적으로 컸지만, 거리별 분포 차이는 단조롭지 않다.",
        "",
        "## 4. 절대 초기각 구간별 결과",
        "",
        "무안정화 인식률 그래프는 거리별 세 반복의 원시 건수를 구간별로 먼저 합산하고 "
        "`success/(success+color_not_detected)`로 계산했다. 카메라 타임아웃은 분모에서 제외했다.",
        "",
        "| Distance | Bin (°) | Success/frames | Conditional rate | Timeouts |",
        "|---:|---:|---:|---:|---:|",
    ]
    for row in rate_bins:
        lines.append(
            f"| {row['distance_m']} m | {row['initial_abs_angle_bin_deg']} "
            f"| {row['successful_detections']}/{row['frames_received']} "
            f"| {fmt_pct(row['frame_conditional_recognition_rate_pct'])} "
            f"| {row['camera_frame_timeouts']} |"
        )
    lines += [
        "",
        "세 거리 모두 0–20°에서는 프레임 조건부 인식률이 100%였고, 20–30°에서도 98.3–100%였다. "
        "30–40°에서 52.6–63.3%로 급감하고 40–50°에서 0–1.7%가 되었다.",
        "",
        "평균 정렬오차 그래프도 실행별 평균을 다시 평균하지 않고, 거리·조건·초기각 구간별 유효 회차를 "
        "모두 결합한 산술평균이다. 각 점의 표본 수는 "
        "`static_alignment_alignment_error_by_initial_angle.csv`에 기록했다. 초기각 증가에 따른 인식률 변화에는 "
        "짐벌 이동량과 이동 중 촬영 효과가 함께 포함되므로 거리나 초기 관측각만의 인과효과로 단정하지 않는다.",
        "",
        "평균 명령오차는 조건별 차이는 있으나 대체로 절대 초기각이 커질수록 증가하는 경향을 보였다.",
        "",
        "## 5. 빨간색 사전검사와의 비교",
        "",
    ]
    if precheck:
        precheck_by_distance = {int(float(row["distance_m"])): row for row in precheck}
        for distance in (1, 2, 3):
            row = precheck_by_distance[distance]
            no = recognition_lookup[(distance, "no_stabilization")]
            lines.append(
                f"- {distance} m 사전검사의 프레임 조건부 인식률은 "
                f"{float(row['frame_conditional_recognition_rate_pct']):.1f}%이고, 정렬 직후 무안정화 실험은 "
                f"{no['frame_conditional_recognition_rate_pct']:.1f}%이다."
            )
        lines.append(
            "사전검사는 세 거리 모두 프레임 조건부 100%였지만 무안정화 정렬 직후에는 낮아졌다. "
            "이는 기본 빨간색 임계값의 거리별 한계보다는 정렬 동작과 영상 획득 시점의 영향을 우선 검토해야 함을 시사한다."
        )
    else:
        lines.append("사전검사 통합 CSV가 없어 직접 비교하지 않았다.")
    lines += [
        "",
        "## 6. 생성 산출물",
        "",
        "- `static_alignment_recognition_aggregate.csv`: 거리×안정화 조건별 인식 성능",
        "- `static_alignment_alignment_error_details.csv`: 1,800회 명령오차 상세",
        "- `static_alignment_alignment_error_summary.csv`: 박스플롯 검증 통계",
        "- `static_alignment_no_settle_recognition_by_initial_angle.csv`: 무안정화 초기각 구간별 인식률",
        "- `static_alignment_alignment_error_by_initial_angle.csv`: 초기각 구간별 평균 명령오차와 표본 수",
        "- `static_alignment_stabilization_recognition_table_en.{png,pdf}`: 영문 인식 성능 표",
        "- `static_alignment_alignment_error_boxplot_en.{png,pdf}`: 영문 명령오차 박스플롯",
        "- `static_alignment_no_settle_recognition_rate_en_wide_large.{png,pdf}`: 영문 무안정화 초기각별 인식률",
        "- `alignment_accuracy_initial_angle_mean_en.{png,pdf}`: 영문 초기각별 평균 명령오차",
        "",
        "## 7. 논문 작성 시 핵심 한계",
        "",
        "- 전체 인식률은 카메라 타임아웃을 실패로 포함하고, 프레임 조건부 인식률은 제외한다.",
        "- 안정화 효과는 색상 인식 시점에 관한 결과이며 실제 짐벌 자세 안정화를 직접 측정한 결과가 아니다.",
        "- 명령 기반 정렬오차는 UWB 계산과 좌표변환을 포함한 적용 명령의 잔차이며 엔코더 또는 영상 기반 실측 오차가 아니다.",
        "- 거리, 조명, 촬영 시각 등 완전히 분리되지 않은 실험 요인이 있으므로 관찰된 차이를 단일 원인의 인과효과로 표현하지 않는다.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=PROJECT_ROOT)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    project_root = args.project_root.resolve()
    manifest = args.manifest.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    entries = read_manifest(manifest, project_root)
    trials, run_summaries, config = read_trials(entries)
    if len(trials) != 1800:
        raise ValueError(f"전체 시도 수가 1,800이 아닙니다: {len(trials)}")
    grouped = group_trials(trials)
    recognition = recognition_summaries(grouped)
    alignment = alignment_summaries(grouped)
    rate_bins = no_settle_bins(trials)
    error_bins = alignment_bins(trials)

    recognition_path = output_dir / "static_alignment_recognition_aggregate.csv"
    write_csv(recognition_path, recognition, tuple(recognition[0]))
    detail_path = output_dir / "static_alignment_alignment_error_details.csv"
    write_detail_csv(detail_path, trials)
    summary_path = output_dir / "static_alignment_alignment_error_summary.csv"
    write_csv(summary_path, alignment, tuple(alignment[0]))
    rate_bins_path = output_dir / "static_alignment_no_settle_recognition_by_initial_angle.csv"
    write_csv(rate_bins_path, rate_bins, tuple(rate_bins[0]))
    error_bins_path = output_dir / "static_alignment_alignment_error_by_initial_angle.csv"
    write_csv(error_bins_path, error_bins, tuple(error_bins[0]))

    plt = configure_matplotlib()
    plot_recognition_table(plt, recognition, output_dir)
    plot_alignment_boxplot(plt, grouped, output_dir)
    plot_no_settle_rates(plt, rate_bins, output_dir)
    plot_alignment_bins(plt, error_bins, output_dir)
    analysis_path = output_dir / "static_alignment_analysis.md"
    write_analysis(
        analysis_path,
        manifest,
        recognition,
        alignment,
        run_summaries,
        rate_bins,
        error_bins,
        config,
        read_precheck(project_root),
    )
    print(f"Validated runs: {len(entries)}")
    print(f"Validated attempts: {len(trials)}")
    for path in (
        recognition_path,
        detail_path,
        summary_path,
        rate_bins_path,
        error_bins_path,
        analysis_path,
    ):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
