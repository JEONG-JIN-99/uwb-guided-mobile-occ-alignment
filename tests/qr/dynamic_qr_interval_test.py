#!/usr/bin/env python3
"""짐벌을 무작위 초기각에 둔 뒤 UWB 정렬 중 QR 인식률을 측정한다."""

import argparse
import csv
import random
import sys
import threading
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
    "distance_m",
    "interval_s",
    "initial_gimbal_deg",
    "uwb_raw_azimuth_deg",
    "gimbal_command_deg",
    "qr_visible",
    "qr_detected",
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
        "--distance",
        type=float,
        required=True,
        help="manually measured experiment distance in meters",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="QR detection window immediately after the UWB alignment command (default: 1.0)",
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=100,
        help="number of complete dynamic alignment attempts (default: 100)",
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
        default=-50,
        help="minimum initial gimbal angle (default: -50)",
    )
    parser.add_argument(
        "--initial-max",
        type=int,
        default=50,
        help="maximum initial gimbal angle (default: 50)",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=2.0,
        help="stabilization time after moving to the random angle (default: 2.0)",
    )
    parser.add_argument(
        "--zero-settle-time",
        type=float,
        default=2.0,
        help="stabilization time after returning to zero (default: 2.0)",
    )
    parser.add_argument(
        "--alignment-settle-time",
        type=float,
        default=2.0,
        help="total stabilization time after the UWB alignment command (default: 2.0)",
    )
    parser.add_argument(
        "--servo-drive-time",
        type=float,
        default=0.4,
        help=(
            "time to keep PWM active after each move before disabling the "
            "control signal (default: 0.4)"
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
    if args.distance <= 0:
        parser.error("--distance must be greater than 0")
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


def show_result(cv2, result, window_name):
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
        label = f"DETECTED {result.data or ''}"
    elif result.visible:
        color = (0, 255, 255)
        label = "VISIBLE: QR pattern detected"
    else:
        cv2.rectangle(
            display,
            (thickness, thickness),
            (width - thickness, height - thickness),
            color,
            thickness,
        )
        label = "NOT VISIBLE"

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

    import cv2 as cv2_module

    cv2 = None
    window_name = "Dynamic QR Interval Test"
    if args.live_stream:
        cv2 = cv2_module
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.output_dir) / f"run_{stamp}"
    results_path = run_dir / "dynamic_qr_results.csv"

    print(f"Attempts: {args.attempts}")
    print(f"Experiment distance: {args.distance:g}m")
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
    visible_count = 0
    detected_count = 0
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
                        "distance_m": f"{args.distance:.6f}",
                        "interval_s": f"{args.interval:.6f}",
                        "qr_visible": False,
                        "qr_detected": False,
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
                        (_uwb_distance, raw_azimuth, _elevation), _received_ns, address = uwb_result
                        requested = estimate_tx_azimuth(initial_deg, raw_azimuth)
                        command, clipped = clamp_servo_command(requested, -90.0, 90.0)
                        row.update(
                            {
                                "uwb_raw_azimuth_deg": raw_azimuth,
                                "gimbal_command_deg": command,
                                "servo_clipped": clipped,
                                "uwb_source": f"{address[0]}:{address[1]}",
                            }
                        )

                        # UWB 정렬 명령 직후 QR을 검사한다. PWM은 정렬 명령 후
                        # servo_drive_time까지 유지하고, 이후 남은 안정화 시간을 기다린다.
                        gimbal.move_to(command)
                        alignment_started_ns = time.monotonic_ns()
                        qr_started_ns = alignment_started_ns
                        qr_deadline_ns = (
                            qr_started_ns + int(args.interval * 1_000_000_000)
                        )
                        qr_holder = {}

                        def detect_qr_during_alignment():
                            try:
                                qr_holder["result"] = scanner.detect_until(
                                    qr_deadline_ns
                                )
                            except Exception as exc:
                                qr_holder["error"] = exc

                        qr_thread = threading.Thread(
                            target=detect_qr_during_alignment,
                            name="dynamic-qr-detection",
                        )
                        qr_thread.start()

                        pwm_disable_ns = (
                            alignment_started_ns
                            + int(args.servo_drive_time * 1_000_000_000)
                        )
                        pwm_remaining_s = max(
                            0.0,
                            (pwm_disable_ns - time.monotonic_ns()) / 1_000_000_000,
                        )
                        if show_live_during_wait(
                            cv2, scanner, pwm_remaining_s, window_name
                        ):
                            stop_requested = True

                        if not args.keep_pwm_active:
                            gimbal.disable_control_signal()

                        alignment_deadline_ns = (
                            alignment_started_ns
                            + int(args.alignment_settle_time * 1_000_000_000)
                        )
                        settle_remaining_s = max(
                            0.0,
                            (alignment_deadline_ns - time.monotonic_ns())
                            / 1_000_000_000,
                        )
                        if show_live_during_wait(
                            cv2, scanner, settle_remaining_s, window_name
                        ):
                            stop_requested = True

                        qr_thread.join()
                        if "error" in qr_holder:
                            raise qr_holder["error"]
                        qr_result = qr_holder["result"]

                        if qr_result.detected:
                            detected_count += 1
                            row.update(
                                {
                                    "qr_visible": True,
                                    "qr_detected": True,
                                    "qr_data": qr_result.data or "",
                                    "status": "success",
                                }
                            )
                        elif qr_result.visible:
                            row.update(
                                {
                                    "qr_visible": True,
                                    "qr_detected": False,
                                    "status": "visible_not_decoded",
                                }
                            )
                        else:
                            row["status"] = "qr_timeout"

                        if not qr_result.detected and qr_result.frame is not None:
                            failed_frames_dir = run_dir / "failed_frames"
                            failed_frames_dir.mkdir(parents=True, exist_ok=True)
                            failure_path = failed_frames_dir / (
                                f"attempt_{attempt:03d}_{row['status']}.jpg"
                            )
                            if not cv2_module.imwrite(
                                str(failure_path), qr_result.frame
                            ):
                                raise RuntimeError(
                                    f"failed to save QR failure frame: {failure_path}"
                                )
                            print(f"  Failure frame: {failure_path}")

                        if qr_result.visible:
                            visible_count += 1

                        print(
                            f"  UWB={raw_azimuth:.2f} deg, command={command:.2f} deg, "
                            f"visible={int(row['qr_visible'])}, "
                            f"detected={int(row['qr_detected'])}"
                        )
                        if show_result(cv2, qr_result, window_name):
                            stop_requested = True

                except Exception as exc:
                    row["status"] = "error"
                    row["error_message"] = f"{type(exc).__name__}: {exc}"
                    print(f"  ERROR: {row['error_message']}")
                finally:
                    row["finished_at"] = iso_now()
                    writer.writerow(
                        {
                            "distance_m": row["distance_m"],
                            "interval_s": row["interval_s"],
                            "initial_gimbal_deg": row["initial_gimbal_deg"],
                            "uwb_raw_azimuth_deg": row["uwb_raw_azimuth_deg"],
                            "gimbal_command_deg": row["gimbal_command_deg"],
                            "qr_visible": row["qr_visible"],
                            "qr_detected": row["qr_detected"],
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

    visible_rate = (
        visible_count / attempts_completed * 100 if attempts_completed else 0.0
    )
    detected_rate = (
        detected_count / attempts_completed * 100 if attempts_completed else 0.0
    )
    print(
        f"Summary: attempts={attempts_completed}, "
        f"visible={visible_count} ({visible_rate:.1f}%), "
        f"detected={detected_count} ({detected_rate:.1f}%)"
    )
    print(f"Results saved to: {results_path}")
    return 0 if visible_count else 2


if __name__ == "__main__":
    raise SystemExit(main())
