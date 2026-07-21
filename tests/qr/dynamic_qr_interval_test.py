#!/usr/bin/env python3
"""짐벌을 무작위 초기각에 둔 뒤 UWB 정렬 중 QR 인식률을 측정한다."""

import argparse
import csv
import random
import sys
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
TEST_QR_DIR = Path(__file__).resolve().parent
for path in (str(CODE_DIR), str(TEST_QR_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)


RESULT_FIELDS = (
    "qr_success",
    "qr_recognition_time_ms",
)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Move to a random initial angle, align from UWB, and test QR "
            "recognition during the following detection window."
        )
    )
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument("--crop-scale", type=float, default=0.3)
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="QR detection window after the UWB alignment command (default: 2.0)",
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=30,
        help="number of complete dynamic alignment attempts (default: 30)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=1.0,
        help="camera warmup after its capture thread starts (default: 1.0)",
    )
    parser.add_argument("--live-stream", action="store_true")
    parser.add_argument("--yaw-pin", type=int, default=18)
    parser.add_argument(
        "--initial-min",
        type=int,
        default=0,
        help="minimum initial gimbal angle (default: 0)",
    )
    parser.add_argument(
        "--initial-max",
        type=int,
        default=0,
        help="maximum initial gimbal angle (default: 0)",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=3.0,
        help="stabilization time after moving to the random angle (default: 3.0)",
    )
    parser.add_argument(
        "--zero-settle-time",
        type=float,
        default=3.0,
        help="stabilization time after returning to zero (default: 3.0)",
    )
    parser.add_argument(
        "--alignment-settle-time",
        type=float,
        default=3.0,
        help="stabilization time after the UWB alignment command (default: 3.0)",
    )
    parser.add_argument(
        "--servo-drive-time",
        type=float,
        default=0.6,
        help=(
            "time to keep PWM active after each move before disabling the "
            "control signal (default: 0.6)"
        ),
    )
    parser.add_argument(
        "--keep-pwm-active",
        action="store_true",
        help="do not disable PWM after moves; useful for A/B jitter comparison",
    )
    parser.add_argument("--uwb-host", default="0.0.0.0")
    parser.add_argument("--uwb-port", type=int, default=5005)
    parser.add_argument("--uwb-timeout", type=float, default=1.0)
    parser.add_argument("--random-seed", type=int, default=20260721)
    parser.add_argument(
        "--output-dir",
        default=str(PROJECT_ROOT / "result" / "dynamic_qr"),
    )
    return parser


def validate_args(parser, args):
    if args.attempts <= 0:
        parser.error("--attempts must be greater than 0")
    if args.interval <= 0:
        parser.error("--interval must be greater than 0")
    if (
        args.warmup < 0
        or args.settle_time < 0
        or args.zero_settle_time < 0
        or args.alignment_settle_time < 0
        or args.servo_drive_time < 0
    ):
        parser.error("warmup and settle times must be 0 or greater")
    if args.uwb_timeout <= 0:
        parser.error("--uwb-timeout must be greater than 0")
    if not 0 < args.crop_scale <= 1:
        parser.error("--crop-scale must be greater than 0 and at most 1")
    if args.initial_min < -90 or args.initial_max > 90:
        parser.error("initial angle range must stay within -90 to 90 degrees")
    if args.initial_min > args.initial_max:
        parser.error("--initial-min must not exceed --initial-max")


def iso_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def show_result(cv2, result, recognition_text, window_name):
    if cv2 is None or result.frame is None:
        return False

    display = result.frame.copy()
    height, width = display.shape[:2]
    color = (0, 0, 255)
    thickness = max(3, min(height, width) // 150)
    if result.detected and result.rect is not None:
        x, y, qr_width, qr_height = result.rect
        cv2.rectangle(
            display,
            (x, y),
            (x + qr_width, y + qr_height),
            color,
            thickness,
        )
        label = f"SUCCESS {recognition_text} {result.data or ''}"
    else:
        cv2.rectangle(
            display,
            (thickness, thickness),
            (width - thickness, height - thickness),
            color,
            thickness,
        )
        label = "FAIL: QR not decoded"

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


def show_live_during_wait(cv2, scanner, duration_s, window_name):
    """안정화 대기 중에도 VNC 영상 창을 갱신한다."""
    deadline_ns = time.monotonic_ns() + int(duration_s * 1_000_000_000)
    while time.monotonic_ns() < deadline_ns:
        if cv2 is None:
            time.sleep(max(0.0, (deadline_ns - time.monotonic_ns()) / 1_000_000_000))
            return False
        snapshot = scanner.get_latest_frame()
        if snapshot is not None:
            cv2.imshow(window_name, snapshot.frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return True
        time.sleep(0.02)
    return False


def settle_after_move(
    cv2,
    scanner,
    gimbal,
    total_settle_s,
    servo_drive_s,
    keep_pwm_active,
    window_name,
):
    """이동 PWM 후 신호를 끄고 남은 안정화 시간을 기다린다."""
    active_s = min(total_settle_s, servo_drive_s)
    if show_live_during_wait(cv2, scanner, active_s, window_name):
        return True

    if not keep_pwm_active:
        gimbal.disable_control_signal()

    remaining_s = max(0.0, total_settle_s - active_s)
    return show_live_during_wait(cv2, scanner, remaining_s, window_name)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    from experiment.static_alignment.angle_utils import (
        clamp_servo_command,
        estimate_tx_azimuth,
    )
    from experiment.static_alignment.devices import UwbReceiver
    from gimbal.gimbal_controller_yaw import GimbalController
    from realsense_scanner import HardwareScanner

    cv2 = None
    window_name = "Dynamic QR Interval Test"
    if args.live_stream:
        import cv2 as cv2_module

        cv2 = cv2_module
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.output_dir) / f"run_{stamp}"
    results_path = run_dir / "dynamic_qr_results.csv"

    print(f"Attempts: {args.attempts}")
    print(f"Random initial range: {args.initial_min} to {args.initial_max} deg")
    print(f"Post-alignment settle time: {args.alignment_settle_time:g}s")
    print(
        "Servo PWM after move: "
        + (
            "kept active"
            if args.keep_pwm_active
            else f"disabled after {args.servo_drive_time:g}s"
        )
    )
    print(f"QR detection window: {args.interval:g}s")
    print(f"Results: {results_path}")

    run_dir.mkdir(parents=True, exist_ok=True)
    random_generator = random.Random(args.random_seed)
    gimbal = None
    uwb = None
    scanner = None
    attempts_completed = 0
    successes = 0
    stop_requested = False

    try:
        gimbal = GimbalController(yaw_pin=args.yaw_pin)
        uwb = UwbReceiver(args.uwb_host, args.uwb_port)
        scanner = HardwareScanner(
            device_index=args.device_index,
            crop_scale=args.crop_scale,
            live_stream=False,
        )
        print(
            f"Starting camera capture and warming up for {args.warmup:g}s."
        )
        if not scanner.start_capture(warmup_sec=args.warmup):
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
                        "qr_success": False,
                        "servo_clipped": False,
                        "status": "error",
                        "started_at": iso_now(),
                    }
                )
                initial_deg = random_generator.randint(
                    args.initial_min, args.initial_max
                )
                row["initial_gimbal_deg"] = initial_deg

                try:
                    print(f"[{attempt:03d}/{args.attempts}] zero -> {initial_deg} deg")
                    gimbal.move_to(0.0)
                    if settle_after_move(
                        cv2,
                        scanner,
                        gimbal,
                        args.zero_settle_time,
                        args.servo_drive_time,
                        args.keep_pwm_active,
                        window_name,
                    ):
                        stop_requested = True
                        break

                    gimbal.move_to(float(initial_deg))
                    if settle_after_move(
                        cv2,
                        scanner,
                        gimbal,
                        args.settle_time,
                        args.servo_drive_time,
                        args.keep_pwm_active,
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
                        (distance, raw_azimuth, _elevation), _received_ns, address = uwb_result
                        requested = estimate_tx_azimuth(initial_deg, raw_azimuth)
                        command, clipped = clamp_servo_command(requested, -90.0, 90.0)
                        row.update(
                            {
                                "uwb_distance_m": distance,
                                "uwb_raw_azimuth_deg": raw_azimuth,
                                "requested_gimbal_command_deg": requested,
                                "gimbal_command_deg": command,
                                "servo_clipped": clipped,
                                "uwb_source": f"{address[0]}:{address[1]}",
                            }
                        )

                        # UWB 정렬 명령 후 지정된 안정화 시간을 온전히 기다린다.
                        # QR 인식 시간은 안정화가 끝난 뒤 별도로 측정한다.
                        gimbal.move_to(command)
                        if settle_after_move(
                            cv2,
                            scanner,
                            gimbal,
                            args.alignment_settle_time,
                            args.servo_drive_time,
                            args.keep_pwm_active,
                            window_name,
                        ):
                            stop_requested = True

                        qr_started_ns = time.monotonic_ns()
                        qr_result = scanner.detect_until(
                            qr_started_ns + int(args.interval * 1_000_000_000)
                        )
                        if qr_result.detected:
                            recognition_ms = (
                                (qr_result.captured_ns - qr_started_ns) / 1_000_000
                                if qr_result.captured_ns is not None
                                else None
                            )
                            recognition_text = (
                                f"{recognition_ms:.3f}ms"
                                if recognition_ms is not None
                                else "unknown"
                            )
                            successes += 1
                            row.update(
                                {
                                    "qr_success": True,
                                    "qr_recognition_time_ms": (
                                        f"{recognition_ms:.3f}"
                                        if recognition_ms is not None
                                        else ""
                                    ),
                                    "qr_data": qr_result.data or "",
                                    "status": "success",
                                }
                            )
                        else:
                            recognition_text = "timeout"
                            row["status"] = "qr_timeout"

                        print(
                            f"  UWB={raw_azimuth:.2f} deg, command={command:.2f} deg, "
                            f"QR={row['status']}"
                        )
                        if show_result(cv2, qr_result, recognition_text, window_name):
                            stop_requested = True

                except Exception as exc:
                    row["status"] = "error"
                    row["error_message"] = f"{type(exc).__name__}: {exc}"
                    print(f"  ERROR: {row['error_message']}")
                finally:
                    row["finished_at"] = iso_now()
                    writer.writerow(
                        {
                            "qr_success": row["qr_success"],
                            "qr_recognition_time_ms": row[
                                "qr_recognition_time_ms"
                            ],
                        }
                    )
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

    failures = attempts_completed - successes
    rate = successes / attempts_completed * 100 if attempts_completed else 0.0
    print(
        f"Summary: attempts={attempts_completed}, successes={successes}, "
        f"failures={failures}, success_rate={rate:.1f}%"
    )
    print(f"Results saved to: {results_path}")
    return 0 if successes else 2


if __name__ == "__main__":
    raise SystemExit(main())
