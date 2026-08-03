#!/usr/bin/env python3
"""Add Rx/Tx subfigure labels below paper-ready tracking plots."""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


PROJECT_ROOT = Path(__file__).resolve().parents[3]
RESULT_ROOT = PROJECT_ROOT / "result" / "dynamic_tracking"
OUTPUT_ROOT = PROJECT_ROOT / "docs" / "paper" / "dynamic_tracking"
EXPERIMENTS = ("1m01", "2m02", "3m02")
RANDOM_EXPERIMENTS = ("random01",)
FONT_PATH = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
FONT_SIZE = 92
BOTTOM_MARGIN = 200


def add_label(source: Path, output: Path, label: str) -> None:
    with Image.open(source) as image:
        original = image.convert("RGBA")
        canvas = Image.new(
            "RGBA",
            (original.width, original.height + BOTTOM_MARGIN),
            "white",
        )
        canvas.paste(original, (0, 0))

        draw = ImageDraw.Draw(canvas)
        font = ImageFont.truetype(str(FONT_PATH), FONT_SIZE)
        box = draw.textbbox((0, 0), label, font=font)
        text_width = box[2] - box[0]
        text_height = box[3] - box[1]
        x = (canvas.width - text_width) / 2
        y = original.height + (BOTTOM_MARGIN - text_height) / 2 - box[1]
        draw.text((x, y), label, fill="black", font=font)

        output.parent.mkdir(parents=True, exist_ok=True)
        canvas.convert("RGB").save(output, dpi=(300, 300))


def combine_side_by_side(left_path: Path, right_path: Path, output: Path) -> None:
    """Combine two equally tall paper figures into a single horizontal panel."""
    with Image.open(left_path) as left_image, Image.open(right_path) as right_image:
        left = left_image.convert("RGB")
        right = right_image.convert("RGB")
        if left.height != right.height:
            raise ValueError(
                "Panel heights must match: "
                f"{left_path} ({left.height}) != {right_path} ({right.height})"
            )

        canvas = Image.new("RGB", (left.width + right.width, left.height), "white")
        canvas.paste(left, (0, 0))
        canvas.paste(right, (left.width, 0))

        output.parent.mkdir(parents=True, exist_ok=True)
        canvas.save(output, dpi=(300, 300))


def main() -> None:
    for experiment in EXPERIMENTS:
        items = (
            (
                f"rx_alignment_tracking_{experiment}.png",
                "(a) Rx direction tracking",
            ),
            (
                f"tx_alignment_tracking_{experiment}.png",
                "(b) Tx direction tracking",
            ),
        )
        for filename, label in items:
            source = RESULT_ROOT / experiment / filename
            if not source.is_file():
                raise FileNotFoundError(source)
            output = OUTPUT_ROOT / filename
            add_label(source, output, label)
            print(output)

        combined_output = OUTPUT_ROOT / f"rx_tx_alignment_tracking_{experiment}.png"
        combine_side_by_side(
            OUTPUT_ROOT / f"rx_alignment_tracking_{experiment}.png",
            OUTPUT_ROOT / f"tx_alignment_tracking_{experiment}.png",
            combined_output,
        )
        print(combined_output)

    for experiment in RANDOM_EXPERIMENTS:
        items = (
            (
                f"random_rx_alignment_tracking_{experiment}.png",
                "(a) Rx direction tracking",
            ),
            (
                f"random_tx_alignment_tracking_{experiment}.png",
                "(b) Tx direction tracking",
            ),
        )
        for filename, label in items:
            source = RESULT_ROOT / experiment / filename
            if not source.is_file():
                raise FileNotFoundError(source)
            output = OUTPUT_ROOT / filename
            add_label(source, output, label)
            print(output)

        combined_output = OUTPUT_ROOT / f"rx_tx_alignment_tracking_{experiment}.png"
        combine_side_by_side(
            OUTPUT_ROOT / f"random_rx_alignment_tracking_{experiment}.png",
            OUTPUT_ROOT / f"random_tx_alignment_tracking_{experiment}.png",
            combined_output,
        )
        print(combined_output)


if __name__ == "__main__":
    main()
