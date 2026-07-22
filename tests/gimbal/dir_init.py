#!/usr/bin/env python3
"""짐벌을 0도로 맞춘 뒤 QR 수동 배치를 위한 카메라 화면을 표시한다."""

import argparse
import os
import sys
import time


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(CURRENT_DIR))
CODE_DIR = os.path.join(PROJECT_ROOT, "code")
CROP_SCALE = 0.3
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Align the yaw gimbal to 0 degrees and disable PWM. Optionally "
            "show the camera view continuously for manual QR placement."
        )
    )
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument("--yaw-pin", type=int, default=18)
    parser.add_argument(
        "--live-stream",
        action="store_true",
        help="show the camera view after alignment; omit for headless alignment only",
    )
    parser.add_argument(
        "--servo-drive-time",
        type=float,
        default=0.6,
        help="seconds to drive the servo at 0 degrees before disabling PWM",
    )
    return parser


def validate_args(parser, args):
    if args.servo_drive_time < 0:
        parser.error("--servo-drive-time must be 0 or greater")


def crop_with_alignment_guide(cv2, frame):
    """실험과 동일한 중앙 30% 영역만 잘라 중앙점을 표시한다."""
    height, width = frame.shape[:2]
    source_center_x, source_center_y = width // 2, height // 2
    crop_width = int(width * CROP_SCALE)
    crop_height = int(height * CROP_SCALE)
    x1 = source_center_x - crop_width // 2
    y1 = source_center_y - crop_height // 2
    x2 = x1 + crop_width
    y2 = y1 + crop_height

    display = frame[y1:y2, x1:x2].copy()
    display_height, display_width = display.shape[:2]
    center_x, center_y = display_width // 2, display_height // 2
    cv2.drawMarker(
        display,
        (center_x, center_y),
        (0, 0, 255),
        cv2.MARKER_CROSS,
        30,
        2,
    )
    cv2.putText(
        display,
        "Center 30% crop | q/ESC: quit",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return display


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    from gimbal.gimbal_controller_yaw import GimbalController

    window_name = "Gimbal 0 deg - QR Placement"
    cv2 = None
    gimbal = None
    camera = None

    try:
        gimbal = GimbalController(yaw_pin=args.yaw_pin)
        gimbal.move_to(0.0)
        print(
            f"[GIMBAL] Moving to 0 deg; PWM will turn off after "
            f"{args.servo_drive_time:g}s."
        )
        time.sleep(args.servo_drive_time)
        gimbal.disable_control_signal()
        print("[GIMBAL] Aligned to 0 deg; PWM control signal is off.")

        if not args.live_stream:
            print("[COMPLETE] Headless alignment complete.")
            return

        import cv2 as cv2_module

        cv2 = cv2_module
        camera = cv2.VideoCapture(args.device_index, cv2.CAP_V4L2)
        if not camera.isOpened():
            raise RuntimeError(f"failed to open /dev/video{args.device_index}")

        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        print(
            f"[CAMERA] Showing /dev/video{args.device_index}. "
            "Place the QR at the red center marker; press q or ESC to quit."
        )

        while True:
            success, frame = camera.read()
            if not success:
                raise RuntimeError("failed to read a camera frame")

            display = crop_with_alignment_guide(cv2, frame)
            cv2.imshow(window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user.")
    finally:
        if camera is not None:
            camera.release()
        if cv2 is not None:
            cv2.destroyAllWindows()
        if gimbal is not None:
            gimbal.cleanup()
        print("[CLEANUP] Camera, window, PWM, and GPIO resources released.")


if __name__ == "__main__":
    main()
