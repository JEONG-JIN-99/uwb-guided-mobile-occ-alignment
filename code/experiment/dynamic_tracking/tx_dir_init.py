#!/usr/bin/env python3
"""동적 추적 실험 전에 Tx GPIO yaw 짐벌을 0도로 맞춘다."""

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
            "Move the Tx yaw gimbal directly connected to a BCM GPIO pin "
            "to 0 degrees before dynamic tracking."
        )
    )
    parser.add_argument(
        "--yaw-pin",
        type=int,
        default=18,
        help="BCM GPIO pin connected directly to the Tx yaw servo (default: 18)",
    )
    parser.add_argument(
        "--stabilization-time",
        type=float,
        default=DEFAULT_STABILIZATION_TIME_SEC,
        help="seconds to hold the gimbal at 0 degrees before disabling PWM",
    )
    return parser


def validate_args(parser, args):
    if args.yaw_pin < 0:
        parser.error("--yaw-pin must be 0 or greater")
    if args.stabilization_time < 0:
        parser.error("--stabilization-time must be 0 or greater")


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_args(parser, args)

    try:
        from gimbal.gimbal_controller_yaw_gpio import GPIOGimbalController
    except (ImportError, RuntimeError) as exc:
        parser.error(
            "RPi.GPIO is required on the Tx Raspberry Pi; "
            f"failed to load the GPIO gimbal controller: {exc}"
        )

    gimbal = None
    try:
        gimbal = GPIOGimbalController(yaw_pin=args.yaw_pin)
        gimbal.move_to(0.0)
        print(
            f"[GIMBAL] Moving Tx GPIO yaw gimbal to 0 deg; waiting "
            f"{args.stabilization_time:g}s for stabilization."
        )
        time.sleep(args.stabilization_time)
        gimbal.disable_control_signal()
        print("[COMPLETE] Tx gimbal aligned to 0 deg; PWM control signal is off.")
    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user.")
    finally:
        if gimbal is not None:
            gimbal.cleanup()
        print("[CLEANUP] Tx PWM and GPIO resources released.")


if __name__ == "__main__":
    main()
