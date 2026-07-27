#!/usr/bin/env python3
"""정적 정렬 실험 전에 짐벌과 카메라 중심을 수동으로 맞춘다."""

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
DEFAULT_STABILIZATION_TIME_SEC = 3.0
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Move the yaw gimbal to 0 degrees and optionally show the camera "
            "view with a center marker for static-alignment setup."
        )
    )
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument("--servo-channel", type=int, default=0)
    parser.add_argument(
        "--pca9685-address",
        type=lambda value: int(value, 0),
        default=0x40,
    )
    parser.add_argument(
        "--crop-scale",
        type=float,
        default=0.3,
        help=(
            "centered display crop ratio; use the same value as the experiment "
            "(default: 0.3)"
        ),
    )
    parser.add_argument(
        "--stabilization-time",
        type=float,
        default=DEFAULT_STABILIZATION_TIME_SEC,
        help="seconds to hold the gimbal at 0 degrees before disabling PWM",
    )
    parser.add_argument(
        "--live-stream",
        action="store_true",
        help="show the camera center guide; omit for headless gimbal alignment",
    )
    return parser


def validate_args(parser, args):
    if args.device_index < 0:
        parser.error("--device-index must be 0 or greater")
    if not 0 <= args.servo_channel < 16:
        parser.error("--servo-channel must be between 0 and 15")
    if not 0 <= args.pca9685_address <= 0x7F:
        parser.error("--pca9685-address must be a 7-bit I2C address")
    if not 0 < args.crop_scale <= 1:
        parser.error("--crop-scale must be greater than 0 and at most 1")
    if args.stabilization_time < 0:
        parser.error("--stabilization-time must be 0 or greater")


def crop_with_alignment_guide(cv2, frame, crop_scale):
    """실험과 같은 중앙 영역을 잘라 중심 십자선을 표시한다."""
    height, width = frame.shape[:2]
    crop_width = max(1, int(width * crop_scale))
    crop_height = max(1, int(height * crop_scale))
    x1 = (width - crop_width) // 2
    y1 = (height - crop_height) // 2
    display = frame[y1 : y1 + crop_height, x1 : x1 + crop_width].copy()

    display_height, display_width = display.shape[:2]
    center = (display_width // 2, display_height // 2)
    cv2.drawMarker(
        display,
        center,
        (0, 0, 255),
        cv2.MARKER_CROSS,
        30,
        2,
    )
    cv2.putText(
        display,
        f"Center guide | crop-scale={crop_scale:g} | q/ESC: quit",
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

    try:
        from gimbal.gimbal_controller_yaw import GimbalController
    except ModuleNotFoundError as exc:
        if exc.name == "adafruit_servokit":
            parser.error(
                "PCA9685 dependency is missing; run "
                "'python -m pip install -r requirements.txt' first"
            )
        raise

    window_name = "Static Alignment Setup - Gimbal 0 deg"
    cv2 = None
    gimbal = None
    camera = None

    try:
        gimbal = GimbalController(
            servo_channel=args.servo_channel,
            pca9685_address=args.pca9685_address,
        )
        gimbal.move_to(0.0)
        print(
            f"[GIMBAL] Moving to 0 deg; waiting "
            f"{args.stabilization_time:g}s for stabilization."
        )
        time.sleep(args.stabilization_time)
        gimbal.disable_control_signal()
        print("[GIMBAL] Aligned to 0 deg; PWM control signal is off.")

        if not args.live_stream:
            print("[COMPLETE] Headless gimbal alignment complete.")
            return

        try:
            import cv2 as cv2_module
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "OpenCV is required for --live-stream; run "
                "'python -m pip install -r requirements.txt' first"
            ) from exc

        cv2 = cv2_module
        camera = cv2.VideoCapture(args.device_index, cv2.CAP_V4L2)
        if not camera.isOpened():
            raise RuntimeError(f"failed to open /dev/video{args.device_index}")

        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        print(
            f"[CAMERA] Showing /dev/video{args.device_index}. "
            "Align the target with the red center marker; press q or ESC to quit."
        )

        while True:
            success, frame = camera.read()
            if not success:
                raise RuntimeError("failed to read a camera frame")

            display = crop_with_alignment_guide(cv2, frame, args.crop_scale)
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
        print("[CLEANUP] Camera, window, and PCA9685 servo resources released.")


if __name__ == "__main__":
    main()
