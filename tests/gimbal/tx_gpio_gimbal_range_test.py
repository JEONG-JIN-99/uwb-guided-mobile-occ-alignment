#!/usr/bin/env python3
"""Tx 짐벌을 서보 드라이버 없이 ROS yaw -90도, +90도, 0도로 움직인다."""

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

TEST_ANGLES_DEG = (-90.0, 90.0)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Drive the Tx yaw servo directly through BCM GPIO in ROS yaw order "
            "-90, +90, and back to 0 degrees."
        )
    )
    parser.add_argument(
        "--yaw-pin",
        type=int,
        default=18,
        help="BCM GPIO pin connected to the yaw servo signal (default: 18)",
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
    if args.yaw_pin < 0:
        parser.error("--yaw-pin must be 0 or greater")
    if args.wait < 0:
        parser.error("--wait must be 0 or greater")

    from gimbal.gimbal_controller_yaw_gpio import GPIOGimbalController

    gimbal = None
    try:
        gimbal = GPIOGimbalController(yaw_pin=args.yaw_pin)
        print("[START] Tx GPIO gimbal range test (ROS yaw): -90 -> +90 -> 0")

        for step, target_deg in enumerate(TEST_ANGLES_DEG, start=1):
            commanded_deg = gimbal.move_to(target_deg)
            print(
                f"[{step}/3] Commanded {commanded_deg:+.0f} degrees; "
                f"waiting {args.wait:g}s."
            )
            time.sleep(args.wait)
    except KeyboardInterrupt:
        print("\n[STOP] Tx GPIO gimbal range test interrupted by user.")
    finally:
        if gimbal is not None:
            try:
                commanded_deg = gimbal.move_to(0.0)
                print(
                    f"[3/3] Commanded {commanded_deg:+.0f} degrees; "
                    f"waiting {args.wait:g}s."
                )
                time.sleep(args.wait)
            finally:
                gimbal.cleanup()
            print("[CLEANUP] Returned to ROS yaw 0 degrees and released GPIO.")


if __name__ == "__main__":
    main()
