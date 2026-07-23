#!/usr/bin/env python3
"""Yaw 짐벌을 0도, +90도, -90도, 0도 순서로 움직인다."""

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

TEST_ANGLES_DEG = (0.0, 90.0, -90.0, 0.0)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Move the yaw gimbal through 0, +90, -90, and back to 0 degrees."
    )
    parser.add_argument(
        "--yaw-pin",
        type=int,
        default=18,
        help="BCM GPIO pin for the yaw servo (default: 18)",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=3.0,
        help="seconds to wait at each angle (default: 3.0)",
    )
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.wait < 0:
        parser.error("--wait must be 0 or greater")

    from gimbal.gimbal_controller_yaw import GimbalController

    gimbal = None

    try:
        gimbal = GimbalController(yaw_pin=args.yaw_pin)
        print("[START] Gimbal range test: 0 -> +90 -> -90 -> 0 degrees")

        for step, target_deg in enumerate(TEST_ANGLES_DEG, start=1):
            commanded_deg = gimbal.move_to(target_deg)
            print(
                f"[{step}/{len(TEST_ANGLES_DEG)}] "
                f"Commanded {commanded_deg:+.0f} degrees; waiting {args.wait:g}s."
            )
            time.sleep(args.wait)

        gimbal.disable_control_signal()
        print("[COMPLETE] Returned to 0 degrees; PWM control signal is off.")
    except KeyboardInterrupt:
        print("\n[STOP] Gimbal range test interrupted by user.")
    finally:
        if gimbal is not None:
            gimbal.cleanup()
        print("[CLEANUP] PWM and GPIO resources released.")


if __name__ == "__main__":
    main()
