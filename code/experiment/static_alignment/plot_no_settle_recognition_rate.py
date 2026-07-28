#!/usr/bin/env python3
"""안정화 없는 정적 정렬의 절대 초기각별 인식률 선그래프를 생성한다."""

import argparse
import csv
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
BIN_EDGES = (0, 10, 20, 30, 40, 50, 60)
OUTPUT_STEM = "static_alignment_no_settle_recognition_rate"

DATASETS = {
    1: (
        "result/static_alignment/run_20260727_220119/"
        "static_alignment_results.csv"
    ),
    2: (
        "result/static_alignment/run_20260727_213725/"
        "static_alignment_results.csv"
    ),
    3: (
        "result/static_alignment/run_20260727_210016/"
        "static_alignment_results.csv"
    ),
}

COLORS = {
    1: "#0072B2",
    2: "#D55E00",
    3: "#009E73",
}
MARKERS = {
    1: "o",
    2: "s",
    3: "^",
}
X_OFFSETS = {
    1: -0.10,
    2: 0.00,
    3: 0.10,
}
@dataclass(frozen=True)
class Trial:
    initial_angle_deg: float
    status: str

    @property
    def success(self):
        return self.status == "success"

    @property
    def frame_received(self):
        return self.status != "camera_frame_timeout"


@dataclass(frozen=True)
class RatePoint:
    successes: int
    denominator: int

    @property
    def rate_pct(self):
        return 100.0 * self.successes / self.denominator


def read_trials(csv_path, expected_distance_m):
    with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {
            "distance_m",
            "pre_recognition_settle_time_s",
            "initial_gimbal_ros_deg",
            "status",
        }
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                f"{csv_path}에 필요한 열이 없습니다: {sorted(missing)}"
            )

        trials = []
        for row in reader:
            distance_m = float(row["distance_m"])
            settle_time_s = float(row["pre_recognition_settle_time_s"])
            if not math.isclose(distance_m, float(expected_distance_m)):
                raise ValueError(
                    f"{csv_path}의 거리 {distance_m:g} m가 예상값 "
                    f"{expected_distance_m:g} m와 다릅니다."
                )
            if not math.isclose(settle_time_s, 0.0):
                raise ValueError(
                    f"{csv_path}에 안정화 시간이 0초가 아닌 행이 있습니다."
                )
            trials.append(
                Trial(
                    initial_angle_deg=float(row["initial_gimbal_ros_deg"]),
                    status=row["status"].strip(),
                )
            )

    if len(trials) != 100:
        raise ValueError(
            f"{csv_path}의 실험 횟수가 {len(trials)}회입니다. 예상값은 100회입니다."
        )
    return trials


def group_by_absolute_initial_angle(trials):
    groups = [[] for _ in range(len(BIN_EDGES) - 1)]
    for trial in trials:
        absolute_angle = abs(trial.initial_angle_deg)
        for index, (lower, upper) in enumerate(
            zip(BIN_EDGES, BIN_EDGES[1:])
        ):
            if lower <= absolute_angle < upper:
                groups[index].append(trial)
                break
        else:
            raise ValueError(
                f"절대 초기각 {absolute_angle:g}°가 그래프 구간을 벗어났습니다."
            )
    return groups


def calculate_rates(groups, conditional_on_frame):
    rates = []
    for group in groups:
        denominator_trials = (
            [trial for trial in group if trial.frame_received]
            if conditional_on_frame
            else group
        )
        rates.append(
            RatePoint(
                successes=sum(trial.success for trial in denominator_trials),
                denominator=len(denominator_trials),
            )
        )
    return rates


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
            "그래프에 사용할 한국어 글꼴을 찾지 못했습니다. "
            "--font-path로 한국어 TTF/OTF 글꼴을 지정하십시오."
        )
    font_manager.fontManager.addfont(font_path)
    font_name = font_manager.FontProperties(fname=font_path).get_name()

    plt.rcParams.update(
        {
            "font.family": font_name,
            "font.size": 19,
            "axes.titlesize": 24,
            "axes.labelsize": 22,
            "legend.fontsize": 18,
            "axes.unicode_minus": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )
    return plt


def add_rate_series(axis, distance_m, rates):
    base_positions = list(range(len(rates)))
    positions = [
        position + X_OFFSETS[distance_m] for position in base_positions
    ]
    values = [rate.rate_pct for rate in rates]

    axis.plot(
        positions,
        values,
        color=COLORS[distance_m],
        marker=MARKERS[distance_m],
        markersize=5.8,
        markeredgecolor="white",
        markeredgewidth=0.7,
        linewidth=1.8,
        label=f"{distance_m} m",
        zorder=3,
    )


def plot_figure(plt, grouped_trials, output_dir, language="ko"):
    figure, axes = plt.subplots(
        1,
        2,
        figsize=(15.5, 7.2),
        sharex=True,
        sharey=True,
    )
    panel_specs = (
        (
            "(a) Overall recognition rate"
            if language == "en"
            else "(가) 전체 인식률",
            False,
        ),
        (
            "(b) Frame-conditional recognition rate"
            if language == "en"
            else "(나) 프레임 조건부 인식률",
            True,
        ),
    )
    tick_labels = [
        f"{lower}–{upper}"
        for lower, upper in zip(BIN_EDGES, BIN_EDGES[1:])
    ]

    for axis, (title, conditional_on_frame) in zip(axes, panel_specs):
        for distance_m in (1, 2, 3):
            rates = calculate_rates(
                grouped_trials[distance_m],
                conditional_on_frame=conditional_on_frame,
            )
            add_rate_series(axis, distance_m, rates)

        axis.set_title(title, pad=10)
        axis.set_xlabel(
            (
                "Absolute initial-angle bin (°)"
                if language == "en"
                else "절대 초기각 구간 (°)"
            )
        )
        axis.set_xticks(range(len(tick_labels)), tick_labels)
        axis.set_xlim(-0.35, len(tick_labels) - 0.65)
        axis.set_ylim(-8, 113)
        axis.set_yticks(range(0, 101, 20))
        axis.grid(axis="y", color="#B8B8B8", linewidth=0.7, alpha=0.55)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)

    axes[0].set_ylabel(
        "Recognition rate (%)" if language == "en" else "인식률 (%)"
    )
    axes[1].legend(
        title="Distance" if language == "en" else "측정 거리",
        loc="lower left",
        frameon=True,
        framealpha=0.92,
    )

    figure.subplots_adjust(
        left=0.105,
        right=0.985,
        top=0.89,
        bottom=0.18,
        wspace=0.10,
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    output_stem = f"{OUTPUT_STEM}_en" if language == "en" else OUTPUT_STEM
    png_path = output_dir / f"{output_stem}.png"
    pdf_path = output_dir / f"{output_stem}.pdf"
    figure.savefig(png_path, dpi=300, facecolor="white")
    figure.savefig(pdf_path, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    return png_path, pdf_path


def print_summary(grouped_trials):
    print("거리,절대 초기각 구간,전체 인식률,프레임 조건부 인식률")
    for distance_m in (1, 2, 3):
        total_rates = calculate_rates(
            grouped_trials[distance_m],
            conditional_on_frame=False,
        )
        conditional_rates = calculate_rates(
            grouped_trials[distance_m],
            conditional_on_frame=True,
        )
        for (lower, upper), total, conditional in zip(
            zip(BIN_EDGES, BIN_EDGES[1:]),
            total_rates,
            conditional_rates,
        ):
            print(
                f"{distance_m} m,{lower}–{upper}°,"
                f"{total.successes}/{total.denominator} "
                f"({total.rate_pct:.1f}%),"
                f"{conditional.successes}/{conditional.denominator} "
                f"({conditional.rate_pct:.1f}%)"
            )


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "안정화 없는 정적 정렬에서 절대 초기각에 따른 거리별 전체 및 "
            "프레임 조건부 인식률 선그래프를 생성합니다."
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
        help="그래프 언어. en을 선택하면 파일명에 _en을 붙입니다.",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    project_root = args.project_root.resolve()
    grouped_trials = {}
    for distance_m, relative_path in DATASETS.items():
        trials = read_trials(project_root / relative_path, distance_m)
        grouped_trials[distance_m] = group_by_absolute_initial_angle(trials)

    print_summary(grouped_trials)
    plt = configure_matplotlib(args.font_path, language=args.language)
    png_path, pdf_path = plot_figure(
        plt,
        grouped_trials,
        args.output_dir.resolve(),
        language=args.language,
    )
    print(f"PNG 저장: {png_path}")
    print(f"PDF 저장: {pdf_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
