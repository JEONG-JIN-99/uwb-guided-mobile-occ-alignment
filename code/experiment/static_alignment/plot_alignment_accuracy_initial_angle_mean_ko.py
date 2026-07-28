#!/usr/bin/env python3
"""절대 초기각 구간별 평균 절대 잔여 명령 오차의 한국어 그림을 생성한다."""

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
BIN_EDGES = (0, 10, 20, 30, 40, 50, 60)
OUTPUT_STEM = "alignment_accuracy_initial_angle_mean"

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
CONDITION_MARKERS = {
    "no_settle": "o",
    "settle_0.5s": "s",
}
CONDITION_OFFSETS = {
    "no_settle": -0.09,
    "settle_0.5s": 0.09,
}


@dataclass(frozen=True)
class AlignmentPoint:
    distance_m: int
    condition: str
    initial_angle_deg: float
    command_angle_deg: float

    @property
    def absolute_error_deg(self):
        return abs(self.command_angle_deg - REFERENCE_ANGLE_DEG)


def read_points(csv_path, expected_distance_m, condition):
    with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {
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
            distance_m = float(row["distance_m"])
            if not math.isclose(distance_m, float(expected_distance_m)):
                raise ValueError(
                    f"{csv_path}의 거리 {distance_m:g} m가 예상값 "
                    f"{expected_distance_m:g} m와 다릅니다."
                )
            point = AlignmentPoint(
                distance_m=expected_distance_m,
                condition=condition,
                initial_angle_deg=float(row["initial_gimbal_ros_deg"]),
                command_angle_deg=float(raw_command),
            )
            if math.isfinite(point.initial_angle_deg) and math.isfinite(
                point.command_angle_deg
            ):
                points.append(point)

    if len(points) != 100:
        raise ValueError(
            f"{csv_path}의 유효 데이터가 {len(points)}개입니다. "
            "예상값은 100개입니다."
        )
    return points


def select(points, distance_m, condition):
    return [
        point
        for point in points
        if point.distance_m == distance_m and point.condition == condition
    ]


def values_by_bin(points):
    return [
        [
            point.absolute_error_deg
            for point in points
            if lower <= abs(point.initial_angle_deg) < upper
        ]
        for lower, upper in zip(BIN_EDGES, BIN_EDGES[1:])
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
            "axes.titlesize": 24,
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


def plot_figure(plt, points, output_dir, language="ko"):
    figure, axes = plt.subplots(
        1,
        3,
        figsize=(19.5, 7.2),
        sharey=True,
    )
    centers = list(range(1, len(BIN_EDGES)))
    tick_labels = [
        f"{lower}–{upper}"
        for lower, upper in zip(BIN_EDGES, BIN_EDGES[1:])
    ]

    condition_labels = (
        CONDITION_LABELS
        if language == "ko"
        else {
            "no_settle": "No stabilization",
            "settle_0.5s": "0.5 s stabilization",
        }
    )

    for axis, distance_m in zip(axes, (1, 2, 3)):
        for condition in condition_labels:
            group = select(points, distance_m, condition)
            means = [
                statistics.fmean(values) for values in values_by_bin(group)
            ]
            positions = [
                center + CONDITION_OFFSETS[condition] for center in centers
            ]
            axis.plot(
                positions,
                means,
                color=CONDITION_COLORS[condition],
                marker=CONDITION_MARKERS[condition],
                markersize=8,
                linewidth=2.4,
                label=condition_labels[condition],
            )

        axis.set_title(
            (
                f"Distance = {distance_m} m"
                if language == "en"
                else f"거리 = {distance_m} m"
            ),
            pad=12,
        )
        axis.set_xlabel(
            (
                "Absolute initial-angle bin (°)"
                if language == "en"
                else "절대 초기각 구간 (°)"
            )
        )
        axis.set_xticks(centers, tick_labels, rotation=25)
        axis.grid(axis="y", alpha=0.25)

    axes[0].set_ylabel(
        (
            "Mean absolute alignment error (°)"
            if language == "en"
            else "평균 절대 잔여 명령 오차 (°)"
        )
    )
    axes[-1].legend(loc="upper left")
    figure.subplots_adjust(
        left=0.095,
        right=0.985,
        top=0.91,
        bottom=0.23,
        wspace=0.09,
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
        description=(
            "절대 초기각 구간별 평균 절대 잔여 명령 오차의 한국어 그림을 "
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
