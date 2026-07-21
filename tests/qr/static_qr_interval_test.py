#!/usr/bin/env python3
"""고정된 QR을 0.2초 단위 탐지 구간으로 반복 검사한다."""

import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
TEST_QR_DIR = Path(__file__).resolve().parent
for path in (str(CODE_DIR), str(TEST_QR_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

def build_parser():
    parser = argparse.ArgumentParser(
        description="Test static QR recognition in consecutive fixed-duration windows."
    )
    parser.add_argument("--device-index", type=int, default=4)
    parser.add_argument("--crop-scale", type=float, default=0.3)
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="QR detection window length in seconds (default: 0.2)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="total test duration in seconds; 0 means run until Ctrl+C",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=1.0,
        help="wait after the background capture thread starts (default: 1.0)",
    )
    parser.add_argument(
        "--live-stream",
        action="store_true",
        help="show the camera image in an OpenCV window",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.interval <= 0:
        raise SystemExit("--interval must be greater than 0")
    if args.duration < 0:
        raise SystemExit("--duration must be 0 or greater")
    if args.warmup < 0:
        raise SystemExit("--warmup must be 0 or greater")

    # --help는 카메라 의존성을 import하지 않고도 확인할 수 있게 한다.
    from realsense_scanner import HardwareScanner

    cv2 = None
    if args.live_stream:
        import cv2 as cv2_module

        cv2 = cv2_module
        cv2.namedWindow("Static QR Interval Test", cv2.WINDOW_NORMAL)

    scanner = HardwareScanner(
        device_index=args.device_index,
        crop_scale=args.crop_scale,
        # 판정에 사용한 바로 그 프레임을 아래 메인 스레드에서 표시한다.
        live_stream=False,
    )
    print(
        f"Starting background capture and warming up for {args.warmup:g}s "
        "before QR detection."
    )
    if not scanner.start_capture(warmup_sec=args.warmup):
        print(f"Camera connection failed: /dev/video{args.device_index}")
        return 1

    # 워밍업은 테스트 duration에 포함하지 않는다.
    started_ns = time.monotonic_ns()
    interval_ns = int(args.interval * 1_000_000_000)
    duration_ns = int(args.duration * 1_000_000_000)
    test_deadline_ns = started_ns + duration_ns if duration_ns else None
    attempts = 0
    successes = 0

    print(
        f"Static QR test started: device=/dev/video{args.device_index}, "
        f"crop_scale={args.crop_scale:g}, interval={args.interval:g}s"
    )
    print("Keep the QR code stationary in the camera view. Press Ctrl+C to stop.")

    try:
        while test_deadline_ns is None or time.monotonic_ns() < test_deadline_ns:
            attempts += 1
            window_started_ns = time.monotonic_ns()
            interval_deadline_ns = window_started_ns + interval_ns
            deadline_ns = (
                min(interval_deadline_ns, test_deadline_ns)
                if test_deadline_ns is not None
                else interval_deadline_ns
            )
            result = scanner.detect_until(deadline_ns)
            finished_ns = time.monotonic_ns()

            if result.detected:
                successes += 1
                recognition_ms = (
                    (result.captured_ns - window_started_ns) / 1_000_000
                    if result.captured_ns is not None
                    else None
                )
                recognition_text = (
                    f"{recognition_ms:.3f}ms" if recognition_ms is not None else "unknown"
                )
                print(
                    f"[{attempts:04d}] SUCCESS time={recognition_text} "
                    f"frame={result.frame_id} data={result.data!r}"
                )
            else:
                elapsed_ms = (finished_ns - window_started_ns) / 1_000_000
                print(
                    f"[{attempts:04d}] FAIL elapsed={elapsed_ms:.3f}ms "
                    f"last_frame={result.frame_id}"
                )

            if cv2 is not None and result.frame is not None:
                display = result.frame.copy()
                height, width = display.shape[:2]
                color = (0, 0, 255)  # OpenCV BGR: red
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
                cv2.imshow("Static QR Interval Test", display)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("Stopped by q key.")
                    break

            # QR을 일찍 찾았더라도 다음 회차는 interval 경계에서 시작한다.
            # 따라서 --interval 1이면 성공 여부와 무관하게 초당 한 번 실행된다.
            remaining_ns = deadline_ns - time.monotonic_ns()
            if remaining_ns > 0:
                time.sleep(remaining_ns / 1_000_000_000)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        scanner.stop()

    rate = successes / attempts * 100 if attempts else 0.0
    print(
        f"Summary: attempts={attempts}, successes={successes}, "
        f"failures={attempts - successes}, success_rate={rate:.1f}%"
    )
    return 0 if successes else 2


if __name__ == "__main__":
    raise SystemExit(main())
