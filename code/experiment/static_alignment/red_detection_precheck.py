#!/usr/bin/env python3
"""정적 정렬 실험과 같은 조건으로 빨간색 인식을 100회 사전 점검한다."""

import argparse
import csv
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))


RESULT_FIELDS = (
    "attempt",
    "distance_m",
    "target_color",
    "crop_scale",
    "interval_s",
    "color_min_area_px",
    "color_min_component_area_px",
    "red_saturation_min",
    "red_value_min",
    "color_visible",
    "color_recognition_time_ms",
    "red_pixel_count",
    "red_pixel_ratio_pct",
    "red_hue_circular_mean_deg",
    "red_saturation_mean",
    "red_value_mean",
    "red_b_mean",
    "red_g_mean",
    "red_r_mean",
    "color_component_area_px",
    "color_center_distance_px",
    "camera_frame_id",
    "camera_captured_ns",
    "status",
    "started_at",
    "finished_at",
)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Check red-color recognition 100 times using the same camera "
            "window and thresholds as the static-alignment experiment."
        )
    )
    parser.add_argument(
        "--distance",
        type=float,
        required=True,
        help="manually measured camera-to-target distance in meters",
    )
    parser.add_argument("--attempts", type=int, default=100)
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument("--crop-scale", type=float, default=0.6)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--camera-warmup", type=float, default=5.0)
    parser.add_argument("--color-min-area", type=float, default=125.0)
    parser.add_argument(
        "--color-min-component-area",
        type=float,
        default=50.0,
    )
    parser.add_argument("--live-stream", action="store_true")
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "result" / "red_detection_precheck"),
    )
    return parser


def validate_args(parser, args):
    if args.attempts <= 0:
        parser.error("--attempts must be greater than 0")
    if not math.isfinite(args.distance) or args.distance <= 0:
        parser.error("--distance must be a finite number greater than 0")
    if args.device_index < 0:
        parser.error("--device-index must be 0 or greater")
    if not math.isfinite(args.crop_scale) or not 0 < args.crop_scale <= 1:
        parser.error("--crop-scale must be greater than 0 and at most 1")
    if args.interval <= 0:
        parser.error("--interval must be greater than 0")
    if args.camera_warmup < 0:
        parser.error("--camera-warmup must be 0 or greater")
    if args.color_min_area < 0:
        parser.error("--color-min-area must be 0 or greater")
    if args.color_min_component_area < 0:
        parser.error("--color-min-component-area must be 0 or greater")


def iso_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def build_run_directory_name(stamp, distance_m, crop_scale):
    """실행 시각과 주요 영상 조건을 사람이 읽을 수 있는 폴더명으로 만든다."""
    return (
        f"run_{stamp}"
        f"_distance_{float(distance_m):g}m"
        f"_crop_{float(crop_scale):g}"
    )


def classify_result(result, baseline_frame_id, started_ns, deadline_ns):
    got_new_frame = (
        result.frame_id is not None
        and result.frame_id > baseline_frame_id
        and result.captured_ns is not None
        and started_ns <= result.captured_ns <= deadline_ns
    )
    if not got_new_frame:
        return False, "camera_frame_timeout"
    if result.visible:
        return True, "success"
    return False, "color_not_detected"


def empty_red_pixel_metrics():
    return {
        "red_pixel_count": "",
        "red_pixel_ratio_pct": "",
        "red_hue_circular_mean_deg": "",
        "red_saturation_mean": "",
        "red_value_mean": "",
        "red_b_mean": "",
        "red_g_mean": "",
        "red_r_mean": "",
    }


def summarize_red_pixels(frame, hsv, selected):
    """선택된 빨간 픽셀의 개수와 BGR·HSV 대표값을 요약한다."""
    empty_metrics = {
        **empty_red_pixel_metrics(),
        "red_pixel_count": 0,
        "red_pixel_ratio_pct": 0.0,
    }
    pixel_count = int(np.count_nonzero(selected))
    if pixel_count == 0:
        return empty_metrics

    selected_hsv = hsv[selected].astype(np.float64)
    selected_bgr = frame[selected].astype(np.float64)

    # OpenCV Hue는 0~179가 0~358도에 대응한다. 빨간색은 0도 경계를
    # 가로지르므로 산술평균 대신 원형평균을 사용한다.
    hue_radians = np.deg2rad(selected_hsv[:, 0] * 2.0)
    hue_mean_deg = (
        math.degrees(
            math.atan2(
                float(np.mean(np.sin(hue_radians))),
                float(np.mean(np.cos(hue_radians))),
            )
        )
        + 360.0
    ) % 360.0
    b_mean, g_mean, r_mean = np.mean(selected_bgr, axis=0)
    pixel_ratio_pct = pixel_count / selected.size * 100.0

    return {
        "red_pixel_count": pixel_count,
        "red_pixel_ratio_pct": pixel_ratio_pct,
        "red_hue_circular_mean_deg": hue_mean_deg,
        "red_saturation_mean": float(np.mean(selected_hsv[:, 1])),
        "red_value_mean": float(np.mean(selected_hsv[:, 2])),
        "red_b_mean": float(b_mean),
        "red_g_mean": float(g_mean),
        "red_r_mean": float(r_mean),
    }


def calculate_red_pixel_metrics(frame, cv2, red_hsv_ranges):
    """검출 기준에 포함된 빨간 픽셀의 개수와 대표 색상값을 계산한다."""
    if frame is None:
        return empty_red_pixel_metrics()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = None
    for lower, upper in red_hsv_ranges:
        range_mask = cv2.inRange(hsv, lower, upper)
        mask = (
            range_mask
            if mask is None
            else cv2.bitwise_or(mask, range_mask)
        )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return summarize_red_pixels(frame, hsv, mask > 0)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    import cv2

    from camera.realsense_scanner import COLOR_HSV_RANGES, HardwareScanner

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.output_dir) / build_run_directory_name(
        stamp,
        args.distance,
        args.crop_scale,
    )
    results_path = run_dir / "red_detection_results.csv"
    run_dir.mkdir(parents=True, exist_ok=True)

    scanner = HardwareScanner(
        device_index=args.device_index,
        crop_scale=args.crop_scale,
        live_stream=args.live_stream,
        color_min_area_px=args.color_min_area,
        color_min_component_area_px=args.color_min_component_area,
        target_color="red",
    )

    completed = 0
    success_count = 0
    color_failure_count = 0
    frame_timeout_count = 0

    print("[PRECHECK] Static-alignment red detection")
    print(f"[PRECHECK] Attempts: {args.attempts}")
    print(f"[PRECHECK] Distance: {args.distance:g}m")
    print(
        f"[PRECHECK] /dev/video{args.device_index}, "
        f"crop-scale={args.crop_scale:g}, interval={args.interval:g}s"
    )
    print(
        f"[PRECHECK] red area >= {args.color_min_area:g}px, "
        f"component >= {args.color_min_component_area:g}px"
    )
    print(f"[PRECHECK] Results: {results_path}")

    try:
        print(
            f"[CAMERA] Starting capture; warmup={args.camera_warmup:g}s"
        )
        if not scanner.start_capture(warmup_sec=args.camera_warmup):
            raise RuntimeError(f"failed to open /dev/video{args.device_index}")

        with results_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=RESULT_FIELDS)
            writer.writeheader()

            for attempt in range(1, args.attempts + 1):
                if not scanner.is_running:
                    print("[STOP] Camera capture stopped.")
                    break

                baseline = scanner.get_latest_frame()
                baseline_frame_id = (
                    baseline.frame_id if baseline is not None else 0
                )
                started_at = iso_now()
                started_ns = time.monotonic_ns()
                deadline_ns = started_ns + int(
                    args.interval * 1_000_000_000
                )
                result = scanner.detect_color_presence_until(
                    deadline_ns,
                    target_color="red",
                    after_frame_id=baseline_frame_id,
                    after_captured_ns=started_ns,
                )
                completed_ns = time.monotonic_ns()
                success, status = classify_result(
                    result,
                    baseline_frame_id,
                    started_ns,
                    deadline_ns,
                )

                if success:
                    success_count += 1
                elif status == "color_not_detected":
                    color_failure_count += 1
                else:
                    frame_timeout_count += 1
                completed += 1

                recognition_time_ms = (
                    (completed_ns - started_ns) / 1_000_000
                    if success
                    else ""
                )
                pixel_metrics = (
                    calculate_red_pixel_metrics(
                        result.frame,
                        cv2,
                        COLOR_HSV_RANGES["red"],
                    )
                    if status != "camera_frame_timeout"
                    else empty_red_pixel_metrics()
                )
                row = {
                    "attempt": attempt,
                    "distance_m": args.distance,
                    "target_color": "red",
                    "crop_scale": args.crop_scale,
                    "interval_s": args.interval,
                    "color_min_area_px": args.color_min_area,
                    "color_min_component_area_px": (
                        args.color_min_component_area
                    ),
                    "red_saturation_min": COLOR_HSV_RANGES["red"][0][0][1],
                    "red_value_min": COLOR_HSV_RANGES["red"][0][0][2],
                    "color_visible": (
                        int(success)
                        if status != "camera_frame_timeout"
                        else ""
                    ),
                    "color_recognition_time_ms": recognition_time_ms,
                    **pixel_metrics,
                    "color_component_area_px": (
                        result.component_area_px
                        if result.component_area_px is not None
                        else ""
                    ),
                    "color_center_distance_px": (
                        result.distance_px
                        if result.distance_px is not None
                        else ""
                    ),
                    "camera_frame_id": (
                        result.frame_id
                        if result.frame_id is not None
                        else ""
                    ),
                    "camera_captured_ns": (
                        result.captured_ns
                        if result.captured_ns is not None
                        else ""
                    ),
                    "status": status,
                    "started_at": started_at,
                    "finished_at": iso_now(),
                }
                writer.writerow(row)
                csv_file.flush()
                print(
                    f"[{attempt:03d}/{args.attempts}] "
                    f"red={int(success)} "
                    f"red_pixels={pixel_metrics['red_pixel_count']} "
                    f"status={status}"
                )
    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user.")
    finally:
        scanner.stop()

    success_rate = (
        success_count / completed * 100.0 if completed else 0.0
    )
    print(
        f"[SUMMARY] recognized={success_count}/{completed} "
        f"({success_rate:.1f}%)"
    )
    print(
        f"[SUMMARY] color_not_detected={color_failure_count}, "
        f"camera_frame_timeout={frame_timeout_count}"
    )
    print(f"[SUMMARY] CSV: {results_path}")


if __name__ == "__main__":
    main()
