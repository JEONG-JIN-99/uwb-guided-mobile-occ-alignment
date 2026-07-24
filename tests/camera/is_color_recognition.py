#!/usr/bin/env python3
"""카메라만 사용해 선택한 색상 원이 지정 시간 안에 인식되는지 반복 측정한다."""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

RESULT_FIELDS = (
    "attempt",
    "window_started_at",
    "target_color",
    "color_visible",
    "recognition_time_ms",
    "color_center_x",
    "color_center_y",
    "color_center_distance_px",
    "color_area_px",
    "color_circularity",
    "camera_frame_id",
    "camera_captured_ns",
    "detected_frame",
    "failure_frame",
)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Camera-only test that checks whether a selected color circle is recognized "
            "within consecutive fixed-duration windows."
        )
    )
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument("--crop-scale", type=float, default=0.3)
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="color-circle recognition window in seconds (default: 0.2)",
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=100,
        help="number of recognition attempts (default: 100)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=5.0,
        help="camera warmup time before measurements (default: 5.0)",
    )
    parser.add_argument(
        "--target-color",
        choices=("red", "orange", "yellow", "green", "blue", "purple"),
        default="red",
        help="circle color to detect (default: red)",
    )
    parser.add_argument(
        "--color-min-area",
        type=float,
        default=500.0,
        help="minimum color contour area in pixels (default: 500)",
    )
    parser.add_argument(
        "--color-min-circularity",
        type=float,
        default=0.6,
        help="minimum contour circularity from 0 to 1 (default: 0.6)",
    )
    parser.add_argument(
        "--live-stream",
        action="store_true",
        help="show the RealSense camera stream",
    )
    parser.add_argument(
        "--save-detected-frames",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="save frames classified as containing the target color circle (default: off)",
    )
    parser.add_argument(
        "--save-failure-frames",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="save the last examined frame when the target color is not detected (default: off)",
    )
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "result" / "is_color_recognition"),
        help="parent directory for per-run CSV results",
    )
    return parser


def validate_args(parser, args):
    if not 0 < args.crop_scale <= 1:
        parser.error("--crop-scale must be greater than 0 and at most 1")
    if args.interval <= 0:
        parser.error("--interval must be greater than 0")
    if args.attempts <= 0:
        parser.error("--attempts must be greater than 0")
    if args.warmup < 0:
        parser.error("--warmup must be 0 or greater")
    if args.color_min_area < 0:
        parser.error("--color-min-area must be 0 or greater")
    if not 0 <= args.color_min_circularity <= 1:
        parser.error("--color-min-circularity must be between 0 and 1")


def iso_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    # --help와 인자 검증은 카메라 의존성을 불러오지 않고도 사용할 수 있게 한다.
    from camera.realsense_scanner import HardwareScanner
    if args.save_detected_frames or args.save_failure_frames:
        import cv2

    scanner = HardwareScanner(
        device_index=args.device_index,
        crop_scale=args.crop_scale,
        live_stream=args.live_stream,
        color_min_area_px=args.color_min_area,
        color_min_circularity=args.color_min_circularity,
        target_color=args.target_color,
    )

    print(
        f"[CAMERA] starting /dev/video{args.device_index}; "
        f"warmup={args.warmup:g}s"
    )
    if not scanner.start_capture(warmup_sec=args.warmup):
        print(f"[ERROR] failed to open /dev/video{args.device_index}")
        return 1

    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.output_dir) / f"run_{run_stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    results_path = run_dir / "color_recognition_results.csv"

    interval_ns = int(args.interval * 1_000_000_000)
    attempts_completed = 0
    successes = 0

    print(
        f"[READY] place or move the {args.target_color} circle in the camera "
        f"view; interval={args.interval:g}s"
    )
    print(f"[RESULT] results will be saved to {results_path}")
    print("[INFO] press Ctrl+C to stop")

    try:
        with results_path.open("w", newline="", encoding="utf-8") as result_file:
            writer = csv.DictWriter(result_file, fieldnames=RESULT_FIELDS)
            writer.writeheader()
            result_file.flush()

            for attempt in range(1, args.attempts + 1):
                window_started_at = iso_now()
                window_started_ns = time.monotonic_ns()
                deadline_ns = window_started_ns + interval_ns

                result = scanner.detect_color_until(deadline_ns)
                attempts_completed += 1
                if result.visible:
                    successes += 1
                    recognition_time_ms = (
                        (result.captured_ns - window_started_ns) / 1_000_000
                        if result.captured_ns is not None
                        else None
                    )
                else:
                    recognition_time_ms = None

                center = result.center or ("", "")
                detected_frame = ""
                failure_frame = ""
                if (
                    args.save_detected_frames
                    and result.visible
                    and result.frame is not None
                ):
                    relative_path = Path("detected_frames") / (
                        f"attempt_{attempt:05d}_detected.jpg"
                    )
                    frame_path = run_dir / relative_path
                    frame_path.parent.mkdir(parents=True, exist_ok=True)
                    if not cv2.imwrite(str(frame_path), result.frame):
                        raise RuntimeError(
                            f"failed to save detected frame: {frame_path}"
                        )
                    detected_frame = str(relative_path)
                elif (
                    args.save_failure_frames
                    and not result.visible
                    and result.frame is not None
                ):
                    relative_path = Path("failed_frames") / (
                        f"attempt_{attempt:05d}_not_detected.jpg"
                    )
                    frame_path = run_dir / relative_path
                    frame_path.parent.mkdir(parents=True, exist_ok=True)
                    if not cv2.imwrite(str(frame_path), result.frame):
                        raise RuntimeError(
                            f"failed to save failure frame: {frame_path}"
                        )
                    failure_frame = str(relative_path)

                writer.writerow(
                    {
                        "attempt": attempt,
                        "window_started_at": window_started_at,
                        "target_color": args.target_color,
                        "color_visible": int(result.visible),
                        "recognition_time_ms": (
                            f"{recognition_time_ms:.3f}"
                            if recognition_time_ms is not None
                            else ""
                        ),
                        "color_center_x": center[0],
                        "color_center_y": center[1],
                        "color_center_distance_px": (
                            result.distance_px
                            if result.distance_px is not None
                            else ""
                        ),
                        "color_area_px": (
                            result.area_px
                            if result.area_px is not None
                            else ""
                        ),
                        "color_circularity": (
                            result.circularity
                            if result.circularity is not None
                            else ""
                        ),
                        "camera_frame_id": result.frame_id or "",
                        "camera_captured_ns": result.captured_ns or "",
                        "detected_frame": detected_frame,
                        "failure_frame": failure_frame,
                    }
                )
                result_file.flush()

                rate = successes / attempts_completed * 100
                recognition_text = (
                    f"{recognition_time_ms:.3f}ms"
                    if recognition_time_ms is not None
                    else "not detected"
                )
                print(
                    f"[{attempt:05d}/{args.attempts:05d}] "
                    f"color={args.target_color} "
                    f"color_visible={int(result.visible)} "
                    f"recognition_time={recognition_text} "
                    f"center={result.center} "
                    f"success_rate={rate:.2f}%"
                )

                # 조기 인식 여부와 관계없이 다음 시도는 interval 경계에서 시작한다.
                remaining_ns = deadline_ns - time.monotonic_ns()
                if remaining_ns > 0:
                    time.sleep(remaining_ns / 1_000_000_000)
    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")
    finally:
        scanner.stop()

    success_rate = (
        successes / attempts_completed * 100 if attempts_completed else 0.0
    )
    print(
        f"[SUMMARY] attempts={attempts_completed}, successes={successes}, "
        f"failures={attempts_completed - successes}, "
        f"success_rate={success_rate:.2f}%"
    )
    print(f"[RESULT] saved to {results_path}")
    return 0 if successes else 2


if __name__ == "__main__":
    raise SystemExit(main())
