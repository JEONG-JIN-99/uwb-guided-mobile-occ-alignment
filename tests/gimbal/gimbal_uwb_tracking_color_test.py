#!/usr/bin/env python3
"""UWB yaw 정렬마다 최신 프레임의 선택 색상 원 검출 결과를 기록한다."""

import argparse
import csv
import socket
import sys
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

TRACKING_INTERVAL_SEC = 0.2
MAX_CORRECTION_PER_FRAME_DEG = 60.0
RESULT_FIELDS = (
    "attempt",
    "uwb_source",
    "uwb_distance_m",
    "uwb_raw_relative_deg",
    "uwb_ros_relative_deg",
    "correction_ros_deg",
    "previous_gimbal_ros_deg",
    "gimbal_command_ros_deg",
    "servo_kit_angle_deg",
    "target_color",
    "color_visible",
    "color_center_x",
    "color_center_y",
    "color_center_distance_px",
    "color_area_px",
    "color_circularity",
    "camera_frame_id",
    "camera_captured_ns",
    "detection_time_ms",
)


def parse_uwb_packet(data):
    message = data.decode("utf-8").strip()
    parts = message.split(",")
    if len(parts) < 4:
        raise ValueError(f"packet has too few fields: {message}")
    if parts[0] != "1":
        return None
    return float(parts[1]), float(parts[2]), float(parts[3])


def receive_latest_available_packet(sock, buffer_size=1024):
    latest_packet = None
    previous_timeout = sock.gettimeout()
    sock.setblocking(False)
    try:
        while True:
            latest_packet = sock.recvfrom(buffer_size)
    except BlockingIOError:
        return latest_packet
    finally:
        sock.settimeout(previous_timeout)


def limit_uwb_correction(uwb_relative_deg):
    return max(
        -MAX_CORRECTION_PER_FRAME_DEG,
        min(MAX_CORRECTION_PER_FRAME_DEG, uwb_relative_deg),
    )


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Track a UWB target with the yaw gimbal and record color-circle "
            "visibility after every processed alignment."
        )
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--servo-channel", type=int, default=0)
    parser.add_argument(
        "--pca9685-address",
        type=lambda value: int(value, 0),
        default=0x40,
    )
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument("--crop-scale", type=float, default=0.3)
    parser.add_argument("--camera-warmup", type=float, default=1.0)
    parser.add_argument("--initial-deg", type=float, default=0.0)
    parser.add_argument(
        "--target-color",
        choices=("red", "orange", "yellow", "green", "blue", "purple"),
        default="red",
    )
    parser.add_argument("--color-min-area", type=float, default=500.0)
    parser.add_argument("--color-min-circularity", type=float, default=0.6)
    parser.add_argument("--live-stream", action="store_true")
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "result" / "gimbal_uwb_tracking_color_test"),
    )
    return parser


def validate_args(parser, args):
    if not 0 < args.crop_scale <= 1:
        parser.error("--crop-scale must be greater than 0 and at most 1")
    if args.camera_warmup < 0:
        parser.error("--camera-warmup must be 0 or greater")
    if args.color_min_area < 0:
        parser.error("--color-min-area must be 0 or greater")
    if not 0 <= args.color_min_circularity <= 1:
        parser.error("--color-min-circularity must be between 0 and 1")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    from camera.realsense_scanner import HardwareScanner
    from gimbal.gimbal_controller_yaw import GimbalController

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(TRACKING_INTERVAL_SEC)
    gimbal = GimbalController(
        servo_channel=args.servo_channel,
        pca9685_address=args.pca9685_address,
    )
    scanner = HardwareScanner(
        device_index=args.device_index,
        crop_scale=args.crop_scale,
        live_stream=args.live_stream,
        color_min_area_px=args.color_min_area,
        color_min_circularity=args.color_min_circularity,
        target_color=args.target_color,
    )

    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.output_dir) / f"run_{run_stamp}"
    results_path = run_dir / "color_results.csv"
    source_printed = False
    attempt = 0
    visible_count = 0

    try:
        if not scanner.start_capture(warmup_sec=args.camera_warmup):
            raise RuntimeError(f"failed to open /dev/video{args.device_index}")

        run_dir.mkdir(parents=True, exist_ok=True)
        gimbal_command_deg = gimbal.move_to(args.initial_deg)
        print(
            f"[READY] listening on {args.host}:{args.port}, "
            f"initial_gimbal_deg={gimbal_command_deg:.2f}"
        )
        print(f"[RESULT] color results will be saved to {results_path}")
        print(
            f"[INFO] {args.target_color} circle detection runs after every "
            "processed alignment."
        )

        with results_path.open("w", newline="", encoding="utf-8") as result_file:
            writer = csv.DictWriter(result_file, fieldnames=RESULT_FIELDS)
            writer.writeheader()
            result_file.flush()
            next_update_ns = time.monotonic_ns()

            while True:
                remaining_ns = next_update_ns - time.monotonic_ns()
                if remaining_ns > 0:
                    time.sleep(remaining_ns / 1_000_000_000)
                next_update_ns += int(TRACKING_INTERVAL_SEC * 1_000_000_000)

                latest_packet = receive_latest_available_packet(sock)
                if latest_packet is None:
                    continue
                data, address = latest_packet

                try:
                    parsed = parse_uwb_packet(data)
                    if parsed is None:
                        continue
                    distance_m, uwb_relative_deg, _elevation = parsed
                    correction_deg = limit_uwb_correction(uwb_relative_deg)
                    uwb_ros_deg = -uwb_relative_deg
                    correction_ros_deg = -correction_deg
                    previous_gimbal_deg = gimbal.current_degree
                    gimbal_command_deg = gimbal.move_by_uwb_relative(
                        correction_deg,
                        wait=False,
                    )
                    attempt += 1

                    detection_started_ns = time.monotonic_ns()
                    snapshot = scanner.get_latest_frame()
                    if snapshot is None:
                        color_result = scanner.detect_color_circle(None)
                        frame_id = None
                        captured_ns = None
                    else:
                        color_result = scanner.detect_color_circle(snapshot.frame)
                        frame_id = snapshot.frame_id
                        captured_ns = snapshot.captured_ns
                    detection_time_ms = (
                        time.monotonic_ns() - detection_started_ns
                    ) / 1_000_000

                    if color_result.visible:
                        visible_count += 1
                    center = color_result.center or ("", "")
                    writer.writerow(
                        {
                            "attempt": attempt,
                            "uwb_source": f"{address[0]}:{address[1]}",
                            "uwb_distance_m": distance_m,
                            "uwb_raw_relative_deg": uwb_relative_deg,
                            "uwb_ros_relative_deg": uwb_ros_deg,
                            "correction_ros_deg": correction_ros_deg,
                            "previous_gimbal_ros_deg": previous_gimbal_deg,
                            "gimbal_command_ros_deg": gimbal_command_deg,
                            "servo_kit_angle_deg": (
                                gimbal.ros_yaw_to_servo_angle(gimbal_command_deg)
                            ),
                            "target_color": args.target_color,
                            "color_visible": int(color_result.visible),
                            "color_center_x": center[0],
                            "color_center_y": center[1],
                            "color_center_distance_px": (
                                color_result.distance_px
                                if color_result.distance_px is not None
                                else ""
                            ),
                            "color_area_px": (
                                color_result.area_px
                                if color_result.area_px is not None
                                else ""
                            ),
                            "color_circularity": (
                                color_result.circularity
                                if color_result.circularity is not None
                                else ""
                            ),
                            "camera_frame_id": frame_id or "",
                            "camera_captured_ns": captured_ns or "",
                            "detection_time_ms": detection_time_ms,
                        }
                    )
                    result_file.flush()

                    if not source_printed:
                        print(f"[SOURCE] UWB packets from {address[0]}:{address[1]}")
                        source_printed = True
                    rate = visible_count / attempt * 100
                    print(
                        "[TRACK]\n"
                        f"  attempt             : {attempt}\n"
                        f"  uwb_raw_deg         : {uwb_relative_deg:.2f}\n"
                        f"  uwb_ros_deg         : {uwb_ros_deg:.2f}\n"
                        f"  correction_ros_deg  : {correction_ros_deg:.2f}\n"
                        f"  gimbal_ros_deg      : {gimbal_command_deg:.2f}\n"
                        f"  target_color        : {args.target_color}\n"
                        f"  color_visible       : {int(color_result.visible)}\n"
                        f"  color_center        : {color_result.center}\n"
                        f"  center_distance_px   : {color_result.distance_px}\n"
                        f"  detection_time_ms    : {detection_time_ms:.3f}\n"
                        f"  cumulative_rate      : {rate:.2f}%"
                    )
                except Exception as exc:
                    print(f"[WARN] failed to process packet {data!r}: {exc}")

    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")
    finally:
        scanner.stop()
        gimbal.move_to(0.0)
        time.sleep(gimbal.ALIGN_INTERVAL_SEC)
        gimbal.cleanup()
        sock.close()

    success_rate = visible_count / attempt * 100 if attempt else 0.0
    print(
        f"[SUMMARY] alignments={attempt}, color_visible={visible_count}, "
        f"success_rate={success_rate:.2f}%"
    )
    return 0 if visible_count else 2


if __name__ == "__main__":
    raise SystemExit(main())
