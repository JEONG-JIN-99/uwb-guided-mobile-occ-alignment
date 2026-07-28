#!/usr/bin/env python3
"""거리 및 안정화 조건별 빨간색 인식 성능 표 이미지를 생성한다."""

import argparse
import csv
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
OUTPUT_STEM = "static_alignment_stabilization_recognition_table"

DATASETS = (
    (
        1,
        "안정화 없음",
        0.0,
        "result/static_alignment/run_20260727_220119/"
        "static_alignment_results.csv",
    ),
    (
        1,
        "0.5초 안정화",
        0.5,
        "result/static_alignment_after_settle/run_20260727_220727/"
        "static_alignment_results.csv",
    ),
    (
        2,
        "안정화 없음",
        0.0,
        "result/static_alignment/run_20260727_213725/"
        "static_alignment_results.csv",
    ),
    (
        2,
        "0.5초 안정화",
        0.5,
        "result/static_alignment_after_settle/run_20260727_214708/"
        "static_alignment_results.csv",
    ),
    (
        3,
        "안정화 없음",
        0.0,
        "result/static_alignment/run_20260727_210016/"
        "static_alignment_results.csv",
    ),
    (
        3,
        "0.5초 안정화",
        0.5,
        "result/static_alignment_after_settle/run_20260727_205444/"
        "static_alignment_results.csv",
    ),
)


@dataclass(frozen=True)
class RecognitionSummary:
    distance_m: int
    condition: str
    total: int
    success: int
    color_not_detected: int
    camera_timeout: int

    @property
    def frames_received(self):
        return self.total - self.camera_timeout

    @property
    def overall_rate_pct(self):
        return 100.0 * self.success / self.total

    @property
    def conditional_rate_pct(self):
        return 100.0 * self.success / self.frames_received


def read_summary(
    csv_path,
    expected_distance_m,
    condition,
    expected_settle_time_s,
):
    with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {
            "distance_m",
            "pre_recognition_settle_time_s",
            "target_color",
            "status",
        }
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                f"{csv_path}에 필요한 열이 없습니다: {sorted(missing)}"
            )

        statuses = []
        for row in reader:
            distance_m = float(row["distance_m"])
            settle_time_s = float(row["pre_recognition_settle_time_s"])
            if not math.isclose(distance_m, float(expected_distance_m)):
                raise ValueError(
                    f"{csv_path}의 거리 {distance_m:g} m가 예상값 "
                    f"{expected_distance_m:g} m와 다릅니다."
                )
            if not math.isclose(settle_time_s, expected_settle_time_s):
                raise ValueError(
                    f"{csv_path}의 안정화 시간 {settle_time_s:g}초가 "
                    f"예상값 {expected_settle_time_s:g}초와 다릅니다."
                )
            if row["target_color"].strip().lower() != "red":
                raise ValueError(f"{csv_path}에 빨간색이 아닌 대상이 있습니다.")
            statuses.append(row["status"].strip())

    if len(statuses) != 100:
        raise ValueError(
            f"{csv_path}의 실험 횟수가 {len(statuses)}회입니다. "
            "예상값은 100회입니다."
        )
    allowed_statuses = {
        "success",
        "color_not_detected",
        "camera_frame_timeout",
    }
    unexpected = set(statuses).difference(allowed_statuses)
    if unexpected:
        raise ValueError(f"{csv_path}에 알 수 없는 상태가 있습니다: {unexpected}")

    return RecognitionSummary(
        distance_m=expected_distance_m,
        condition=condition,
        total=len(statuses),
        success=statuses.count("success"),
        color_not_detected=statuses.count("color_not_detected"),
        camera_timeout=statuses.count("camera_frame_timeout"),
    )


def format_percentage(value):
    return (
        f"{value:.0f}%"
        if math.isclose(value, round(value))
        else f"{value:.1f}%"
    )


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
            "표에 사용할 한국어 글꼴을 찾지 못했습니다. "
            "--font-path로 한국어 TTF/OTF 글꼴을 지정하십시오."
        )
    font_manager.fontManager.addfont(font_path)
    font_name = font_manager.FontProperties(fname=font_path).get_name()
    plt.rcParams.update(
        {
            "font.family": font_name,
            "font.size": 10,
            "axes.unicode_minus": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    return plt


def build_table_rows(summaries, language="ko"):
    return [
        [
            f"{summary.distance_m} m",
            (
                (
                    "0.5 s stabilization"
                    if summary.condition == "0.5초 안정화"
                    else "No stabilization"
                )
                if language == "en"
                else summary.condition
            ),
            f"{summary.success}/{summary.total}",
            (
                str(summary.color_not_detected)
                if language == "en"
                else f"{summary.color_not_detected}회"
            ),
            (
                str(summary.camera_timeout)
                if language == "en"
                else f"{summary.camera_timeout}회"
            ),
            format_percentage(summary.overall_rate_pct),
            (
                f"{summary.success}/{summary.frames_received} "
                f"({format_percentage(summary.conditional_rate_pct)})"
            ),
        ]
        for summary in summaries
    ]


def plot_table(plt, summaries, output_dir, language="ko"):
    figure, axis = plt.subplots(figsize=(13.8, 4.8))
    axis.axis("off")
    table = axis.table(
        cellText=build_table_rows(summaries, language=language),
        colLabels=(
            (
                (
                    "Distance",
                    "Stabilization\ncondition",
                    "Successful\ndetections",
                    "Color\nmisses",
                    "Camera\ntimeouts",
                    "Overall\nrecognition rate",
                    "Frame-conditional\nrecognition rate",
                )
                if language == "en"
                else (
                    "거리",
                    "안정화 조건",
                    "인식 성공",
                    "색상 미검출",
                    "카메라 타임아웃",
                    "전체 인식률",
                    "프레임 조건부 인식률",
                )
            )
        ),
        cellLoc="center",
        colLoc="center",
        colWidths=(0.08, 0.17, 0.13, 0.13, 0.15, 0.13, 0.21),
        bbox=(0.015, 0.04, 0.97, 0.92),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9.0 if language == "en" else 10.5)

    header_color = "#315A75"
    no_settle_color = "#F8EEE5"
    settle_color = "#E7F1F5"
    edge_color = "#FFFFFF"

    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor(edge_color)
        cell.set_linewidth(1.4)
        if row == 0:
            cell.set_facecolor(header_color)
            cell.get_text().set_color("white")
            cell.get_text().set_fontweight("bold")
        else:
            is_settled = summaries[row - 1].condition == "0.5초 안정화"
            cell.set_facecolor(settle_color if is_settled else no_settle_color)
            if column in (0, 1):
                cell.get_text().set_fontweight("bold")

    output_dir.mkdir(parents=True, exist_ok=True)
    output_stem = f"{OUTPUT_STEM}_en" if language == "en" else OUTPUT_STEM
    png_path = output_dir / f"{output_stem}.png"
    pdf_path = output_dir / f"{output_stem}.pdf"
    figure.savefig(
        png_path,
        dpi=300,
        bbox_inches="tight",
        pad_inches=0.10,
        facecolor="white",
    )
    figure.savefig(
        pdf_path,
        bbox_inches="tight",
        pad_inches=0.10,
        facecolor="white",
    )
    plt.close(figure)
    return png_path, pdf_path


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "거리 및 안정화 조건별 빨간색 인식 성능 표 이미지를 생성합니다."
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
        help="표 언어. en을 선택하면 파일명에 _en을 붙입니다.",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    project_root = args.project_root.resolve()
    summaries = [
        read_summary(
            project_root / relative_path,
            distance_m,
            condition,
            settle_time_s,
        )
        for distance_m, condition, settle_time_s, relative_path in DATASETS
    ]

    for summary in summaries:
        print(
            f"{summary.distance_m} m, {summary.condition}: "
            f"성공 {summary.success}/{summary.total}, "
            f"색상 미검출 {summary.color_not_detected}, "
            f"카메라 타임아웃 {summary.camera_timeout}, "
            f"조건부 인식률 {summary.conditional_rate_pct:.1f}%"
        )

    plt = configure_matplotlib(args.font_path, language=args.language)
    png_path, pdf_path = plot_table(
        plt,
        summaries,
        args.output_dir.resolve(),
        language=args.language,
    )
    print(f"PNG 저장: {png_path}")
    print(f"PDF 저장: {pdf_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
