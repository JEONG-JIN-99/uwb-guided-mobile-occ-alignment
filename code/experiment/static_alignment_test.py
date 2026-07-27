#!/usr/bin/env python3
"""무작위 초기각에서 UWB로 정렬한 뒤 제한시간 내 색상 인식률을 측정한다."""

import argparse
import csv
import math
import random
import socket
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
    "distance_m",
    "interval_s",
    "initial_gimbal_ros_deg",
    "uwb_source",
    "uwb_raw_azimuth_deg",
    "uwb_ros_azimuth_deg",
    "target_calculated_ros_deg",
    "gimbal_command_ros_deg",
    "servo_clipped",
    "target_color",
    "color_visible",
    "color_success",
    "color_recognition_time_ms",
    "camera_frame_id",
    "camera_captured_ns",
    "failure_frame",
    "status",
    "started_at",
    "finished_at",
    "error_message",
)


def normalize_angle(angle_deg):
    """각도를 [-180, 180) 범위로 정규화한다."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def estimate_tx_azimuth(initial_gimbal_deg, uwb_relative_azimuth_deg):
    """UWB CW 상대각을 ROS CCW 좌표계의 절대 목표각으로 변환한다."""
    uwb_ros_azimuth_deg = -float(uwb_relative_azimuth_deg)
    return normalize_angle(initial_gimbal_deg + uwb_ros_azimuth_deg)


def clamp_servo_command(requested_deg, min_deg, max_deg):
    if min_deg > max_deg:
        raise ValueError("servo minimum angle must not exceed maximum angle")
    applied = min(max(float(requested_deg), float(min_deg)), float(max_deg))
    return applied, applied != float(requested_deg)


def parse_uwb_packet(data):
    message = data.decode("utf-8").strip()
    parts = message.split(",")
    if len(parts) < 4 or parts[0] != "1":
        return None
    distance, azimuth, elevation = map(float, parts[1:4])
    if not all(math.isfinite(value) for value in (distance, azimuth, elevation)):
        return None
    if not -180.0 <= azimuth < 180.0:
        return None
    return distance, azimuth, elevation


class UwbReceiver:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, int(port)))
        self.socket.setblocking(False)

    def discard_pending(self):
        while True:
            try:
                self.socket.recvfrom(4096)
            except BlockingIOError:
                return

    def receive_first_valid(self, timeout_s):
        deadline_ns = time.monotonic_ns() + int(timeout_s * 1_000_000_000)
        self.socket.setblocking(True)
        try:
            while True:
                remaining_s = (
                    deadline_ns - time.monotonic_ns()
                ) / 1_000_000_000
                if remaining_s <= 0:
                    return None
                self.socket.settimeout(remaining_s)
                try:
                    data, address = self.socket.recvfrom(4096)
                except socket.timeout:
                    return None
                received_ns = time.monotonic_ns()
                try:
                    parsed = parse_uwb_packet(data)
                except (UnicodeDecodeError, ValueError):
                    continue
                if parsed is not None:
                    return parsed, received_ns, address
        finally:
            self.socket.setblocking(False)

    def close(self):
        self.socket.close()


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Move to a random initial angle, align from UWB, and test color "
            "recognition immediately after the alignment command."
        )
    )
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument(
        "--crop-scale",
        type=float,
        default=0.3,
        help="centered camera crop ratio (default: 0.3)",
    )
    parser.add_argument(
        "--distance",
        type=float,
        required=True,
        help="manually measured experiment distance in meters",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help=(
            "color recognition deadline measured from the alignment command "
            "(default: 0.2)"
        ),
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=100,
        help="number of complete static alignment attempts (default: 100)",
    )
    parser.add_argument(
        "--camera-warmup",
        "--warmup",
        dest="camera_warmup",
        type=float,
        default=5.0,
        help=(
            "camera auto-exposure/white-balance stabilization time "
            "(default: 5.0)"
        ),
    )
    parser.add_argument("--live-stream", action="store_true")
    parser.add_argument("--servo-channel", type=int, default=0)
    parser.add_argument(
        "--pca9685-address",
        type=lambda value: int(value, 0),
        default=0x40,
    )
    parser.add_argument(
        "--initial-min",
        type=int,
        default=-50,
        help="minimum initial ROS gimbal angle (default: -50)",
    )
    parser.add_argument(
        "--initial-max",
        type=int,
        default=50,
        help="maximum initial ROS gimbal angle (default: 50)",
    )
    parser.add_argument(
        "--initial-settle-time",
        "--settle-time",
        dest="initial_settle_time",
        type=float,
        default=1.0,
        help="stabilization after moving to the random angle (default: 1.0)",
    )
    parser.add_argument(
        "--zero-settle-time",
        type=float,
        default=1.0,
        help="stabilization after returning to zero (default: 1.0)",
    )
    parser.add_argument(
        "--alignment-settle-time",
        type=float,
        default=1.0,
        help=(
            "total stabilization period measured from the alignment command; "
            "the color window is included in it (default: 1.0)"
        ),
    )
    parser.add_argument(
        "--target-color",
        choices=("red", "orange", "yellow", "green", "blue", "purple"),
        default="red",
    )
    parser.add_argument(
        "--color-min-area",
        type=float,
        default=500.0,
        help="minimum total selected-color mask pixels (default: 500)",
    )
    parser.add_argument(
        "--color-min-component-area",
        type=float,
        default=200.0,
        help="minimum largest connected color area in pixels (default: 200)",
    )
    parser.add_argument(
        "--save-failure-frames",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="save the last frame on color recognition failure (default: on)",
    )
    parser.add_argument("--uwb-host", default="0.0.0.0")
    parser.add_argument("--uwb-port", type=int, default=5005)
    parser.add_argument("--uwb-timeout", type=float, default=1.0)
    parser.add_argument("--random-seed", type=int, default=20260721)
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "result" / "static_alignment"),
    )
    return parser


def validate_args(parser, args):
    if args.attempts <= 0:
        parser.error("--attempts must be greater than 0")
    if args.distance <= 0:
        parser.error("--distance must be greater than 0")
    if args.interval <= 0:
        parser.error("--interval must be greater than 0")
    if (
        args.camera_warmup < 0
        or args.initial_settle_time < 0
        or args.zero_settle_time < 0
        or args.alignment_settle_time < 0
    ):
        parser.error("camera warmup and settle times must be 0 or greater")
    if args.alignment_settle_time < args.interval:
        parser.error("--alignment-settle-time must be at least --interval")
    if args.uwb_timeout <= 0:
        parser.error("--uwb-timeout must be greater than 0")
    if not 0 < args.crop_scale <= 1:
        parser.error("--crop-scale must be greater than 0 and at most 1")
    if args.initial_min < -90 or args.initial_max > 90:
        parser.error("initial angle range must stay within -90 to 90 degrees")
    if args.initial_min > args.initial_max:
        parser.error("--initial-min must not exceed --initial-max")
    if args.color_min_area < 0:
        parser.error("--color-min-area must be 0 or greater")
    if args.color_min_component_area < 0:
        parser.error("--color-min-component-area must be 0 or greater")


def iso_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def show_live_during_wait(cv2, scanner, duration_s, window_name):
    """안정화 대기 중에도 선택적으로 영상 창을 갱신한다."""
    deadline_ns = time.monotonic_ns() + int(duration_s * 1_000_000_000)
    while time.monotonic_ns() < deadline_ns:
        if cv2 is None:
            remaining_s = (
                deadline_ns - time.monotonic_ns()
            ) / 1_000_000_000
            time.sleep(max(0.0, remaining_s))
            return False
        snapshot = scanner.get_latest_frame()
        if snapshot is not None:
            cv2.imshow(window_name, snapshot.frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return True
        time.sleep(0.02)
    return False


def show_color_result(cv2, result, target_color, window_name):
    if cv2 is None or result.frame is None:
        return False
    display = result.frame.copy()
    label = (
        f"{target_color.upper()} DETECTED"
        if result.visible
        else f"{target_color.upper()} NOT DETECTED"
    )
    color = (0, 255, 0) if result.visible else (0, 0, 255)
    cv2.putText(
        display,
        label,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.imshow(window_name, display)
    return cv2.waitKey(1) & 0xFF == ord("q")


def save_failure_frame(cv2, run_dir, attempt, status, color_result):
    if color_result.frame is None:
        return ""
    relative_path = (
        Path("failed_frames") / f"attempt_{attempt:03d}_{status}.jpg"
    )
    absolute_path = run_dir / relative_path
    absolute_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(absolute_path), color_result.frame):
        raise RuntimeError(f"failed to save color failure frame: {absolute_path}")
    return str(relative_path)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    from camera.realsense_scanner import HardwareScanner
    from gimbal.gimbal_controller_yaw import GimbalController

    import cv2 as cv2_module

    cv2 = None
    window_name = "Static Alignment Color Test"
    if args.live_stream:
        cv2 = cv2_module
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.output_dir) / f"run_{stamp}"
    results_path = run_dir / "static_alignment_results.csv"

    print(f"Attempts: {args.attempts}")
    print(f"Experiment distance: {args.distance:g}m")
    print(f"Random initial range: {args.initial_min} to {args.initial_max} deg")
    print(f"Camera color warmup: {args.camera_warmup:g}s")
    print(f"Color recognition window: {args.interval:g}s from alignment command")
    print(f"Post-alignment total settle time: {args.alignment_settle_time:g}s")
    print("Servo PWM: kept active between moves")
    print(f"Results: {results_path}")

    run_dir.mkdir(parents=True, exist_ok=True)
    random_generator = random.Random(args.random_seed)
    gimbal = None
    uwb = None
    scanner = None
    attempts_completed = 0
    success_count = 0
    stop_requested = False

    try:
        gimbal = GimbalController(
            servo_channel=args.servo_channel,
            pca9685_address=args.pca9685_address,
        )
        uwb = UwbReceiver(args.uwb_host, args.uwb_port)
        scanner = HardwareScanner(
            device_index=args.device_index,
            crop_scale=args.crop_scale,
            live_stream=False,
            color_min_area_px=args.color_min_area,
            color_min_component_area_px=args.color_min_component_area,
            target_color=args.target_color,
        )
        print(
            "Starting camera capture and stabilizing color for "
            f"{args.camera_warmup:g}s."
        )
        if not scanner.start_capture(warmup_sec=args.camera_warmup):
            raise RuntimeError(f"failed to open /dev/video{args.device_index}")

        with results_path.open("w", newline="", encoding="utf-8") as result_file:
            writer = csv.DictWriter(result_file, fieldnames=RESULT_FIELDS)
            writer.writeheader()
            result_file.flush()

            for attempt in range(1, args.attempts + 1):
                row = {field: "" for field in RESULT_FIELDS}
                row.update(
                    {
                        "attempt": attempt,
                        "distance_m": f"{args.distance:.6f}",
                        "interval_s": f"{args.interval:.6f}",
                        "target_color": args.target_color,
                        # 실제 새 프레임을 판정하기 전에는 미검출(0)이 아니라
                        # 판정하지 않음(빈 값)으로 구분한다.
                        "color_visible": "",
                        "color_success": 0,
                        "servo_clipped": 0,
                        "status": "error",
                        "started_at": iso_now(),
                    }
                )
                initial_deg = random_generator.randint(
                    args.initial_min, args.initial_max
                )
                row["initial_gimbal_ros_deg"] = initial_deg

                try:
                    print(f"[{attempt:03d}/{args.attempts}] zero -> {initial_deg} deg")
                    gimbal.move_to(0.0)
                    if show_live_during_wait(
                        cv2,
                        scanner,
                        args.zero_settle_time,
                        window_name,
                    ):
                        stop_requested = True
                        break

                    gimbal.move_to(float(initial_deg))
                    if show_live_during_wait(
                        cv2,
                        scanner,
                        args.initial_settle_time,
                        window_name,
                    ):
                        stop_requested = True
                        break

                    uwb.discard_pending()
                    uwb_result = uwb.receive_first_valid(args.uwb_timeout)
                    if uwb_result is None:
                        row["status"] = "uwb_timeout"
                        print("  UWB timeout")
                    else:
                        (
                            (_uwb_distance, raw_azimuth, _elevation),
                            _received_ns,
                            address,
                        ) = uwb_result
                        target_calculated = estimate_tx_azimuth(
                            initial_deg,
                            raw_azimuth,
                        )
                        ros_azimuth = -raw_azimuth
                        command, clipped = clamp_servo_command(
                            target_calculated,
                            -90.0,
                            90.0,
                        )
                        row.update(
                            {
                                "uwb_source": f"{address[0]}:{address[1]}",
                                "uwb_raw_azimuth_deg": raw_azimuth,
                                "uwb_ros_azimuth_deg": ros_azimuth,
                                "target_calculated_ros_deg": target_calculated,
                                "gimbal_command_ros_deg": command,
                                "servo_clipped": int(clipped),
                            }
                        )

                        baseline = scanner.get_latest_frame()
                        baseline_frame_id = (
                            baseline.frame_id if baseline is not None else 0
                        )
                        alignment_commanded_ns = time.monotonic_ns()
                        gimbal.move_to(command)
                        color_deadline_ns = alignment_commanded_ns + int(
                            args.interval * 1_000_000_000
                        )
                        color_result = scanner.detect_color_presence_until(
                            color_deadline_ns,
                            target_color=args.target_color,
                            after_frame_id=baseline_frame_id,
                            after_captured_ns=alignment_commanded_ns,
                        )
                        detection_completed_ns = time.monotonic_ns()

                        got_new_frame = (
                            color_result.frame_id is not None
                            and color_result.frame_id > baseline_frame_id
                            and color_result.captured_ns is not None
                            and color_result.captured_ns
                            >= alignment_commanded_ns
                            and color_result.captured_ns
                            <= color_deadline_ns
                        )
                        color_success = bool(
                            got_new_frame and color_result.visible
                        )
                        if color_success:
                            status = "success"
                            success_count += 1
                        elif not got_new_frame:
                            status = "camera_frame_timeout"
                        else:
                            status = "color_not_detected"

                        recognition_time_ms = ""
                        if color_success:
                            recognition_time_ms = (
                                detection_completed_ns
                                - alignment_commanded_ns
                            ) / 1_000_000

                        failure_frame = ""
                        if not color_success and args.save_failure_frames:
                            failure_frame = save_failure_frame(
                                cv2_module,
                                run_dir,
                                attempt,
                                status,
                                color_result,
                            )

                        row.update(
                            {
                                "color_visible": (
                                    int(color_result.visible)
                                    if got_new_frame
                                    else ""
                                ),
                                "color_success": int(color_success),
                                "color_recognition_time_ms": recognition_time_ms,
                                "camera_frame_id": (
                                    color_result.frame_id
                                    if color_result.frame_id is not None
                                    else ""
                                ),
                                "camera_captured_ns": (
                                    color_result.captured_ns
                                    if color_result.captured_ns is not None
                                    else ""
                                ),
                                "failure_frame": failure_frame,
                                "status": status,
                            }
                        )

                        alignment_settle_deadline_ns = (
                            alignment_commanded_ns
                            + int(
                                args.alignment_settle_time
                                * 1_000_000_000
                            )
                        )
                        remaining_settle_s = max(
                            0.0,
                            (
                                alignment_settle_deadline_ns
                                - time.monotonic_ns()
                            )
                            / 1_000_000_000,
                        )
                        if show_live_during_wait(
                            cv2,
                            scanner,
                            remaining_settle_s,
                            window_name,
                        ):
                            stop_requested = True

                        print(
                            f"  UWB raw={raw_azimuth:.2f} deg, "
                            f"UWB ROS={ros_azimuth:.2f} deg, "
                            f"target={target_calculated:.2f} deg, "
                            f"command={command:.2f} deg, "
                            f"color_success={int(color_success)}, "
                            f"frame_id={row['camera_frame_id']}, "
                            f"status={status}"
                        )
                        if failure_frame:
                            print(f"  Failure frame: {run_dir / failure_frame}")
                        if show_color_result(
                            cv2,
                            color_result,
                            args.target_color,
                            window_name,
                        ):
                            stop_requested = True

                except Exception as exc:
                    row["status"] = "error"
                    row["error_message"] = f"{type(exc).__name__}: {exc}"
                    print(f"  ERROR: {row['error_message']}")
                finally:
                    row["finished_at"] = iso_now()
                    writer.writerow(row)
                    result_file.flush()
                    attempts_completed += 1
                    gimbal.move_to(0.0)

                if stop_requested:
                    break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if gimbal is not None:
            try:
                gimbal.move_to(0.0)
                time.sleep(gimbal.ALIGN_INTERVAL_SEC)
            finally:
                gimbal.cleanup()
        if uwb is not None:
            uwb.close()
        if scanner is not None:
            scanner.stop()
        if cv2 is not None:
            cv2.destroyAllWindows()

    success_rate = (
        success_count / attempts_completed * 100
        if attempts_completed
        else 0.0
    )
    print(
        f"Summary: attempts={attempts_completed}, "
        f"color_success={success_count} ({success_rate:.1f}%)"
    )
    print(f"Results saved to: {results_path}")
    return 0 if success_count else 2


if __name__ == "__main__":
    raise SystemExit(main())
