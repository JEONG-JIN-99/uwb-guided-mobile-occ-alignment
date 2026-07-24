#!/usr/bin/env python3
"""수동 입력으로 yaw 짐벌의 구간별 정렬 시간을 측정한다."""

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))


ALIGNMENT_TRIALS = (
    (-90.0, 0.0),
    (90.0, 0.0),
    (-90.0, 90.0),
    (90.0, -90.0),
)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Manually measure yaw-gimbal alignment duration for four fixed "
            "start/target angle pairs."
        )
    )
    parser.add_argument("--servo-channel", type=int, default=0)
    parser.add_argument(
        "--pca9685-address",
        type=lambda value: int(value, 0),
        default=0x40,
    )
    return parser


def wait_for_enter(prompt):
    """Enter만 허용해 실수로 다음 동작을 시작하는 것을 막는다."""
    while True:
        if input(prompt).strip() == "":
            return
        print("Enter만 누르세요.")


def main(argv=None):
    args = build_parser().parse_args(argv)

    from gimbal.gimbal_controller_yaw import GimbalController

    gimbal = None
    results = []

    print("\n=== 짐벌 정렬 시간 수동 측정 ===")
    print("각 실험은 출발각 배치 -> Enter로 이동 시작 -> Enter로 측정 종료 순서입니다.")
    print("측정 종료 Enter를 누르는 즉시 PWM 신호를 끕니다.")

    try:
        gimbal = GimbalController(
            servo_channel=args.servo_channel,
            pca9685_address=args.pca9685_address,
        )

        for number, (start_deg, target_deg) in enumerate(ALIGNMENT_TRIALS, start=1):
            print(
                f"\n[{number}/{len(ALIGNMENT_TRIALS)}] 준비: "
                f"짐벌을 출발각 {start_deg:+.0f}도로 이동합니다."
            )
            gimbal.move_to(start_deg)
            wait_for_enter(
                "출발각 도착과 시계 준비를 확인한 뒤 Enter를 누르면 "
                f"{target_deg:+.0f}도로 이동을 시작합니다: "
            )

            started_ns = time.monotonic_ns()
            gimbal.move_to(target_deg)
            print(
                f"[START] 실험 {number}: {start_deg:+.0f}도 -> "
                f"{target_deg:+.0f}도"
            )
            wait_for_enter(
                "목표각에 도착했다고 판단한 순간 Enter를 눌러 측정을 종료하세요: "
            )
            stopped_ns = time.monotonic_ns()
            gimbal.disable_control_signal()

            elapsed_s = (stopped_ns - started_ns) / 1_000_000_000
            results.append((start_deg, target_deg, elapsed_s))
            print(f"[STOP] PWM OFF | 프로그램 측정값: {elapsed_s:.3f}초")

        print("\n=== 측정 결과 ===")
        for number, (start_deg, target_deg, elapsed_s) in enumerate(results, start=1):
            print(
                f"{number}. {start_deg:+.0f}도 -> {target_deg:+.0f}도: "
                f"{elapsed_s:.3f}초"
            )
    except (KeyboardInterrupt, EOFError):
        print("\n[STOP] 사용자 입력으로 실험을 중단했습니다.")
    finally:
        if gimbal is not None:
            gimbal.cleanup()
        print("[CLEANUP] PCA9685 서보 제어 신호를 비활성화했습니다.")


if __name__ == "__main__":
    main()
