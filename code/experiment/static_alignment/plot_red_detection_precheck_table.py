#!/usr/bin/env python3
"""정렬 전 빨간색 검출 사전검사의 거리별 성능 표 이미지를 생성한다."""

import argparse
import csv
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
OUTPUT_STEM = "red_detection_precheck_performance_table"

DATASETS = {
    1: (
        "result/red_detection_precheck/"
        "run_20260727_221312_distance_1m_crop_0.6/"
        "red_detection_results.csv"
    ),
    2: (
        "result/red_detection_precheck/"
        "run_20260727_215243_distance_2m_crop_0.6/"
        "red_detection_results.csv"
    ),
    3: (
        "result/red_detection_precheck/"
        "run_20260727_210645_distance_3m_crop_0.6/"
        "red_detection_results.csv"
    ),
}


@dataclass(frozen=True)
class DetectionSummary:
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
    def frame_conditional_rate_pct(self):
        return 100.0 * self.success / self.frames_received


def summarize_csv(csv_path, expected_distance_m):
    with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {"distance_m", "target_color", "status"}
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                f"{csv_path}에 필요한 열이 없습니다: {sorted(missing)}"
            )

        statuses = []
        for row in reader:
            distance_m = float(row["distance_m"])
            if not math.isclose(distance_m, float(expected_distance_m)):
                raise ValueError(
                    f"{csv_path}의 거리 {distance_m:g} m가 예상값 "
                    f"{expected_distance_m:g} m와 다릅니다."
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

    return DetectionSummary(
        total=len(statuses),
        success=statuses.count("success"),
        color_not_detected=statuses.count("color_not_detected"),
        camera_timeout=statuses.count("camera_frame_timeout"),
    )


def format_rate(numerator, denominator):
    rate = 100.0 * numerator / denominator
    rate_text = f"{rate:.0f}" if math.isclose(rate, round(rate)) else f"{rate:.1f}"
    return f"{numerator}/{denominator} ({rate_text}%)"


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
            "font.size": 11,
            "axes.unicode_minus": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    return plt


def build_table_rows(summaries, language="ko"):
    rows = [
        (
            "Overall recognition rate" if language == "en" else "전체 인식률",
            lambda item: format_rate(item.success, item.total),
        ),
        (
            (
                "Frame-conditional\nrecognition rate"
                if language == "en"
                else "프레임 조건부 인식률"
            ),
            lambda item: format_rate(item.success, item.frames_received),
        ),
    ]
    return [
        [label, *(formatter(summaries[distance_m]) for distance_m in (1, 2, 3))]
        for label, formatter in rows
    ]


def plot_table(plt, summaries, output_dir, language="ko"):
    figure, axis = plt.subplots(figsize=(11.2, 2.6))
    axis.axis("off")

    table = axis.table(
        cellText=build_table_rows(summaries, language=language),
        colLabels=(
            (
                "Color detection\nperformance"
                if language == "en"
                else "색상 검출 성능"
            ),
            "1 m",
            "2 m",
            "3 m",
        ),
        cellLoc="center",
        colLoc="center",
        colWidths=(0.40, 0.20, 0.20, 0.20),
        bbox=(0.02, 0.06, 0.96, 0.88),
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10.5 if language == "en" else 11.5)

    header_color = "#315A75"
    first_column_color = "#E7EEF3"
    stripe_color = "#F5F7F9"
    rate_row_color = "#E8F3F8"
    edge_color = "#FFFFFF"

    for (row, column), cell in table.get_celld().items():
        cell.set_edgecolor(edge_color)
        cell.set_linewidth(1.4)
        if row == 0:
            cell.set_facecolor(header_color)
            cell.get_text().set_color("white")
            cell.get_text().set_fontweight("bold")
        elif column == 0:
            cell.set_facecolor(first_column_color)
            cell.get_text().set_fontweight("bold")
            cell.get_text().set_ha("left")
            cell.PAD = 0.08
        elif row in (1, 2):
            cell.set_facecolor(rate_row_color)
            cell.get_text().set_fontweight("bold")
        elif row % 2 == 0:
            cell.set_facecolor(stripe_color)
        else:
            cell.set_facecolor("white")

    output_dir.mkdir(parents=True, exist_ok=True)
    output_stem = f"{OUTPUT_STEM}_en" if language == "en" else OUTPUT_STEM
    png_path = output_dir / f"{output_stem}.png"
    pdf_path = output_dir / f"{output_stem}.pdf"
    figure.savefig(
        png_path,
        dpi=300,
        bbox_inches="tight",
        pad_inches=0.12,
        facecolor="white",
    )
    figure.savefig(
        pdf_path,
        bbox_inches="tight",
        pad_inches=0.12,
        facecolor="white",
    )
    plt.close(figure)
    return png_path, pdf_path


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "정렬 전 정지 상태에서 수행한 빨간색 검출 사전검사의 거리별 "
            "성능 표 이미지를 생성합니다."
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
    summaries = {
        distance_m: summarize_csv(project_root / relative_path, distance_m)
        for distance_m, relative_path in DATASETS.items()
    }

    for distance_m in (1, 2, 3):
        item = summaries[distance_m]
        print(
            f"{distance_m} m: 성공 {item.success}/{item.total}, "
            f"색상 미검출 {item.color_not_detected}, "
            f"카메라 타임아웃 {item.camera_timeout}, "
            f"프레임 조건부 인식률 "
            f"{item.frame_conditional_rate_pct:.1f}%"
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
