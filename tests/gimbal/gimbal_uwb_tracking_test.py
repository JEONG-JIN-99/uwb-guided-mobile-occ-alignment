import argparse
import csv
import os
import socket
import sys
import time
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CODE_DIR = os.path.join(PROJECT_ROOT, "code")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

from gimbal.gimbal_controller_yaw import GimbalController


UWB_DEADBAND_DEG = 0.0
MAX_CORRECTION_PER_FRAME_DEG = 60.0
QR_RESULT_FIELDS = ("qr_visible", "qr_decoded", "qr_data")


def parse_uwb_packet(data):
    """
    Expected UWB packet:
    header,distance,azimuth,elevation,...

    header == "1" means UWB mode. Azimuth is treated as a relative angle in degrees.
    """
    message = data.decode("utf-8").strip()
    parts = message.split(",")
    if len(parts) < 4:
        raise ValueError(f"packet has too few fields: {message}")

    header = parts[0]
    if header != "1":
        return None

    distance = float(parts[1])
    azimuth = float(parts[2])
    elevation = float(parts[3])
    return distance, azimuth, elevation


def limit_uwb_correction(uwb_relative_deg):
    """Apply the tracking deadband and per-frame correction limit."""
    if abs(uwb_relative_deg) < UWB_DEADBAND_DEG:
        return 0.0

    return max(
        -MAX_CORRECTION_PER_FRAME_DEG,
        min(MAX_CORRECTION_PER_FRAME_DEG, uwb_relative_deg),
    )


def main():
    parser = argparse.ArgumentParser(
        description="Hardware test: track a moving UWB target with yaw gimbal."
    )
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--port", type=int, default=5005, help="UDP bind port")
    parser.add_argument("--yaw-pin", type=int, default=18, help="BCM GPIO pin for yaw servo")
    parser.add_argument(
        "--qr-device-index",
        type=int,
        default=4,
        help="QR camera V4L2 device index (default: 4 for /dev/video4)",
    )
    parser.add_argument(
        "--qr-crop-scale",
        type=float,
        default=0.3,
        help="centered QR detection crop ratio (default: 0.3)",
    )
    parser.add_argument(
        "--qr-timeout",
        type=float,
        default=0.2,
        help="QR detection window after each alignment in seconds (default: 0.2)",
    )
    parser.add_argument(
        "--camera-warmup",
        type=float,
        default=1.0,
        help="camera warmup time in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--output-dir",
        default=str(Path(PROJECT_ROOT) / "result" / "gimbal_uwb_tracking"),
        help="parent directory for per-run QR CSV results",
    )
    parser.add_argument(
        "--live-stream",
        action="store_true",
        help="show the QR camera video while tracking",
    )
    parser.add_argument(
        "--initial-deg",
        type=float,
        default=0.0,
        help="Initial gimbal command angle in degrees, -90 to 90",
    )
    args = parser.parse_args()
    if not 0 < args.qr_crop_scale <= 1.0:
        parser.error("--qr-crop-scale must be greater than 0 and at most 1")
    if args.qr_timeout <= 0:
        parser.error("--qr-timeout must be greater than 0")
    if args.camera_warmup < 0:
        parser.error("--camera-warmup must be 0 or greater")

    from qr.realsense_scanner import HardwareScanner

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(0.2)

    gimbal = GimbalController(yaw_pin=args.yaw_pin)
    qr_scanner = HardwareScanner(
        device_index=args.qr_device_index,
        crop_scale=args.qr_crop_scale,
        live_stream=args.live_stream,
    )
    source_printed = False
    qr_capture_started = False
    result_file = None

    try:
        if not qr_scanner.start_capture(warmup_sec=args.camera_warmup):
            raise RuntimeError(
                f"failed to start QR camera /dev/video{args.qr_device_index}"
            )
        qr_capture_started = True

        run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = Path(args.output_dir) / f"run_{run_stamp}"
        run_dir.mkdir(parents=True, exist_ok=True)
        results_path = run_dir / "qr_results.csv"
        result_file = results_path.open("w", newline="", encoding="utf-8")
        result_writer = csv.DictWriter(result_file, fieldnames=QR_RESULT_FIELDS)
        result_writer.writeheader()
        result_file.flush()

        gimbal_command_deg = gimbal.move_to(args.initial_deg)
        print(
            f"[READY] listening on {args.host}:{args.port}, "
            f"initial gimbal_command_deg={gimbal_command_deg:.2f}"
        )
        print(f"[RESULT] QR results will be saved to {results_path}")
        print("[INFO] Move the opposite UWB module. Press Ctrl+C to stop.")

        while True:
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue

            try:
                parsed = parse_uwb_packet(data)
                if parsed is None:
                    continue

                _distance, uwb_relative_deg, _elevation = parsed
                correction_deg = limit_uwb_correction(uwb_relative_deg)
                if not source_printed:
                    print(f"[SOURCE] receiving UWB packets from {addr[0]}:{addr[1]}")
                    source_printed = True

                before_command_deg = gimbal.current_degree
                gimbal_command_deg = gimbal.move_by_uwb_relative(
                    correction_deg,
                    wait=False,
                )
                qr_deadline_ns = (
                    time.monotonic_ns() + int(args.qr_timeout * 1_000_000_000)
                )
                qr_result = qr_scanner.detect_until(qr_deadline_ns)
                result_writer.writerow(
                    {
                        "qr_visible": int(qr_result.visible),
                        "qr_decoded": int(qr_result.detected),
                        "qr_data": qr_result.data or "",
                    }
                )
                result_file.flush()

                print(
                    "[TRACK]\n"
                    f"  relative_deg       : {uwb_relative_deg:.2f}\n"
                    f"  correction_deg     : {correction_deg:.2f}\n"
                    f"  prev_gimbal_deg    : {before_command_deg:.2f}\n"
                    f"  gimbal_command_deg : {gimbal_command_deg:.2f}\n"
                    f"  qr_visible         : {int(qr_result.visible)}\n"
                    f"  qr_decoded         : {int(qr_result.detected)}\n"
                    f"  qr_data            : {qr_result.data or ''}"
                )
            except Exception as exc:
                print(f"[WARN] failed to process packet {data!r}: {exc}")

    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")
    finally:
        if result_file is not None:
            result_file.close()
        if qr_capture_started:
            qr_scanner.stop()
        gimbal.move_to(0.0)
        time.sleep(gimbal.ALIGN_INTERVAL_SEC)
        gimbal.cleanup()
        sock.close()
        print("[DONE] gimbal returned to 0 deg and GPIO cleaned up")


if __name__ == "__main__":
    main()
