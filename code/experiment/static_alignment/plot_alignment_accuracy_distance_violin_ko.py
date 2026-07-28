#!/usr/bin/env python3
"""거리별 절대 잔여 명령 오차의 한국어 바이올린 그림을 생성한다."""

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
OUTPUT_STEM = "alignment_accuracy_distance_violin"

DATASETS = (
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

CONDITION_LABELS = {
    "no_settle": "안정화 없음",
    "settle_0.5s": "0.5초 안정화",
}
CONDITION_COLORS = {
    "no_settle": "#E68613",
    "settle_0.5s": "#2474B5",
}
CONDITION_OFFSETS = {
    "no_settle": -0.16,
    "settle_0.5s": 0.16,
}


@dataclass(frozen=True)
class AlignmentPoint:
    distance_m: int
    condition: str
    command_angle_deg: float

    @property
    def absolute_error_deg(self):
        return abs(self.command_angle_deg - REFERENCE_ANGLE_DEG)


def read_points(csv_path, expected_distance_m, condition):
    with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {"distance_m", "gimbal_command_ros_deg"}
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
            distance_m = float(row["distance_m"])
            if not math.isclose(distance_m, float(expected_distance_m)):
                raise ValueError(
                    f"{csv_path}의 거리 {distance_m:g} m가 예상값 "
                    f"{expected_distance_m:g} m와 다릅니다."
                )
            point = AlignmentPoint(
                distance_m=expected_distance_m,
                condition=condition,
                command_angle_deg=float(raw_command),
            )
            if math.isfinite(point.command_angle_deg):
                points.append(point)

    if len(points) != 100:
        raise ValueError(
            f"{csv_path}의 유효 데이터가 {len(points)}개입니다. "
            "예상값은 100개입니다."
        )
    return points


def select_errors(points, distance_m, condition):
    return [
        point.absolute_error_deg
        for point in points
        if point.distance_m == distance_m and point.condition == condition
    ]


def configure_matplotlib(requested_font_path=None, language="ko"):
    os.environ.setdefault(
        "MPLCONFIGDIR",
        str(Path(tempfile.gettempdir()) / "uwb_alignment_matplotlib"),
    )
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import font_manager
    import matplotlib.pyplot as plt

    font_candidates = (
        Path(requested_font_path).expanduser() if requested_font_path else None,
        Path("/usr/share/fonts/truetype/nanum/NanumMyeongjo.ttf"),
        Path("/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc"),
        (
            Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
            if language == "en"
            else None
        ),
    )
    font_path = next(
        (path for path in font_candidates if path is not None and path.exists()),
        None,
    )
    if font_path is None:
        raise RuntimeError(
            "그림에 사용할 한국어 글꼴을 찾지 못했습니다. "
            "--font-path로 한국어 TTF/OTF 글꼴을 지정하십시오."
        )
    font_manager.fontManager.addfont(font_path)
    font_name = font_manager.FontProperties(fname=font_path).get_name()

    plt.rcParams.update(
        {
            "font.family": font_name,
            "font.size": 20,
            "axes.labelsize": 22,
            "legend.fontsize": 18,
            "xtick.labelsize": 20,
            "ytick.labelsize": 20,
            "axes.unicode_minus": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    return plt


def style_violin(parts, color):
    for body in parts["bodies"]:
        body.set_facecolor(color)
        body.set_edgecolor(color)
        body.set_alpha(0.45)
    for key in ("cbars", "cmins", "cmaxes"):
        parts[key].set_color(color)
        parts[key].set_linewidth(1.5)


def plot_figure(plt, points, output_dir, language="ko"):
    figure, axis = plt.subplots(figsize=(12.5, 7.5))
    base_positions = (1, 2, 3)
    condition_labels = (
        CONDITION_LABELS
        if language == "ko"
        else {
            "no_settle": "No stabilization",
            "settle_0.5s": "0.5 s stabilization",
        }
    )

    for condition in condition_labels:
        datasets = [
            select_errors(points, distance_m, condition)
            for distance_m in base_positions
        ]
        positions = [
            distance_m + CONDITION_OFFSETS[condition]
            for distance_m in base_positions
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
        means = [statistics.fmean(values) for values in datasets]
        axis.scatter(
            positions,
            means,
            color=CONDITION_COLORS[condition],
            edgecolors="white",
            linewidths=1.2,
            s=90,
            zorder=3,
            label=condition_labels[condition],
        )

    axis.set_xlabel("Distance (m)" if language == "en" else "거리 (m)")
    axis.set_ylabel(
        "Alignment error (°)" if language == "en" else "정렬 오차 (°)"
    )
    axis.set_xticks(base_positions, [str(value) for value in base_positions])
    axis.set_xlim(0.55, 3.45)
    axis.grid(axis="y", alpha=0.25)
    axis.legend(
        title="Stabilization condition" if language == "en" else "안정화 조건",
        loc="upper right",
    )
    figure.subplots_adjust(
        left=0.13,
        right=0.975,
        top=0.96,
        bottom=0.15,
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    output_stem = f"{OUTPUT_STEM}_en" if language == "en" else OUTPUT_STEM
    png_path = output_dir / f"{output_stem}.png"
    pdf_path = output_dir / f"{output_stem}.pdf"
    figure.savefig(png_path, dpi=300, facecolor="white")
    figure.savefig(pdf_path, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    return png_path, pdf_path


def build_parser():
    parser = argparse.ArgumentParser(
        description="거리별 절대 잔여 명령 오차의 한국어 바이올린 그림을 생성합니다."
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
        help="PNG와 PDF 출력 폴더",
    )
    parser.add_argument(
        "--font-path",
        type=Path,
        help="한국어를 지원하는 TTF/OTF 글꼴 경로",
    )
    parser.add_argument(
        "--language",
        choices=("ko", "en"),
        default="ko",
        help="그림 언어. en을 선택하면 파일명에 _en을 붙입니다.",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    project_root = args.project_root.resolve()
    points = []
    for distance_m, condition, relative_path in DATASETS:
        points.extend(
            read_points(
                project_root / relative_path,
                distance_m,
                condition,
            )
        )
    if len(points) != 600:
        raise ValueError(
            f"전체 유효 데이터가 {len(points)}개입니다. 예상값은 600개입니다."
        )

    plt = configure_matplotlib(args.font_path, language=args.language)
    png_path, pdf_path = plot_figure(
        plt,
        points,
        args.output_dir.resolve(),
        language=args.language,
    )
    print(f"PNG 저장: {png_path}")
    print(f"PDF 저장: {pdf_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
