#!/usr/bin/env python3
"""Plot a rover XY trajectory and heading in the world coordinate frame."""

from __future__ import annotations

import argparse
import csv
import math
import os
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def parse_args():
    parser = argparse.ArgumentParser(
        description="로버 위치 로그를 전체 좌표계 XY 경로로 그린다."
    )
    parser.add_argument("rover_csv", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--title")
    return parser.parse_args()


def read_rows(path):
    required = {"elapsed_sec", "pose_x", "pose_y", "heading_deg"}
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"필수 열 누락: {sorted(missing)}")
        rows = []
        for row in reader:
            try:
                rows.append(
                    {
                        "elapsed_sec": float(row["elapsed_sec"]),
                        "pose_x": float(row["pose_x"]),
                        "pose_y": float(row["pose_y"]),
                        "heading_deg": float(row["heading_deg"]),
                    }
                )
            except (TypeError, ValueError):
                continue
    if len(rows) < 2:
        raise ValueError("그릴 수 있는 로버 위치가 2개 미만입니다.")
    rows.sort(key=lambda row: row["elapsed_sec"])
    return rows


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
            "savefig.dpi": 300,
        }
    )
    return plt


def cumulative_distance(rows):
    return sum(
        math.hypot(
            current["pose_x"] - previous["pose_x"],
            current["pose_y"] - previous["pose_y"],
        )
        for previous, current in zip(rows, rows[1:])
    )


def plot_trajectory(plt, rows, output_path, title):
    x = [row["pose_x"] for row in rows]
    y = [row["pose_y"] for row in rows]
    elapsed = [row["elapsed_sec"] for row in rows]

    figure, axis = plt.subplots(figsize=(8.4, 7.2), constrained_layout=True)
    axis.plot(x, y, color="#555555", linewidth=1.3, alpha=0.65, zorder=1)
    points = axis.scatter(
        x,
        y,
        c=elapsed,
        cmap="viridis",
        s=24,
        edgecolors="none",
        zorder=2,
    )

    arrow_step = max(1, len(rows) // 12)
    arrow_rows = rows[::arrow_step]
    arrow_scale = 0.13
    axis.quiver(
        [row["pose_x"] for row in arrow_rows],
        [row["pose_y"] for row in arrow_rows],
        [
            arrow_scale * math.cos(math.radians(row["heading_deg"]))
            for row in arrow_rows
        ],
        [
            arrow_scale * math.sin(math.radians(row["heading_deg"]))
            for row in arrow_rows
        ],
        angles="xy",
        scale_units="xy",
        scale=1,
        color="#222222",
        width=0.004,
        headwidth=4.2,
        headlength=5.2,
        label="Rover heading",
        zorder=3,
    )

    start = rows[0]
    end = rows[-1]
    axis.scatter(
        start["pose_x"],
        start["pose_y"],
        marker="o",
        s=110,
        color="#009E73",
        edgecolor="white",
        linewidth=1.0,
        label="Start",
        zorder=4,
    )
    axis.scatter(
        end["pose_x"],
        end["pose_y"],
        marker="X",
        s=125,
        color="#D55E00",
        edgecolor="white",
        linewidth=1.0,
        label="End",
        zorder=4,
    )
    axis.annotate(
        f"Start ({start['pose_x']:.2f}, {start['pose_y']:.2f})",
        (start["pose_x"], start["pose_y"]),
        xytext=(10, -20),
        textcoords="offset points",
        va="top",
    )
    axis.annotate(
        f"End ({end['pose_x']:.2f}, {end['pose_y']:.2f})",
        (end["pose_x"], end["pose_y"]),
        xytext=(-10, 12),
        textcoords="offset points",
        ha="right",
        va="bottom",
    )

    axis.set_title(title, pad=14)
    axis.set_xlabel("World x (m)")
    axis.set_ylabel("World y (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.28)
    axis.legend(loc="best")
    colorbar = figure.colorbar(points, ax=axis, pad=0.02)
    colorbar.set_label("Elapsed time (s)")
    figure.savefig(output_path, dpi=300)
    plt.close(figure)


def main():
    args = parse_args()
    rover_csv = args.rover_csv.resolve()
    rows = read_rows(rover_csv)
    output_path = (
        args.output.resolve()
        if args.output
        else rover_csv.with_name("rover_world_trajectory.png")
    )
    title = args.title or f"{rover_csv.parent.name} Rover Trajectory in World Frame"
    plot_trajectory(configure_matplotlib(), rows, output_path, title)
    print(
        f"saved={output_path} samples={len(rows)} "
        f"time={rows[0]['elapsed_sec']:.3f}..{rows[-1]['elapsed_sec']:.3f}s "
        f"distance={cumulative_distance(rows):.3f}m"
    )


if __name__ == "__main__":
    main()
