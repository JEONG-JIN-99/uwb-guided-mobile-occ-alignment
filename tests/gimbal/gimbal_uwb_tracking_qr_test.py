import argparse
import csv
import os
import queue
import socket
import sys
import threading
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
TRACKING_INTERVAL_SEC = 0.2
QR_RESULT_FIELDS = ("qr_visible", "qr_decoded", "qr_data", "failure_frame")


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


def receive_latest_packet(sock, buffer_size=1024):
    """Wait for a packet, then discard the backlog and return only the newest one."""
    latest_packet = sock.recvfrom(buffer_size)
    previous_timeout = sock.gettimeout()
    sock.setblocking(False)

    try:
        while True:
            latest_packet = sock.recvfrom(buffer_size)
    except BlockingIOError:
        return latest_packet
    finally:
        sock.settimeout(previous_timeout)


def receive_latest_available_packet(sock, buffer_size=1024):
    """Drain queued packets without waiting and return the newest, if any."""
    latest_packet = None
    previous_timeout = sock.gettimeout()
    sock.setblocking(False)

    try:
        while True:
            latest_packet = sock.recvfrom(buffer_size)
    except BlockingIOError:
        return latest_packet
    finally:
        sock.settimeout(previous_timeout)


def wait_until_next_update(next_update_ns):
    """Keep the gimbal control loop on its fixed cadence."""
    remaining_ns = next_update_ns - time.monotonic_ns()
    if remaining_ns > 0:
        time.sleep(remaining_ns / 1_000_000_000)


def save_qr_failure_frame(cv2_module, run_dir, attempt, qr_result):
    """Save the frame returned for a failed QR attempt and return its relative path."""
    if qr_result.detected or qr_result.frame is None:
        return ""

    failure_status = "visible_not_decoded" if qr_result.visible else "not_visible"
    relative_path = Path("failed_frames") / (
        f"attempt_{attempt:06d}_{failure_status}.jpg"
    )
    failure_path = run_dir / relative_path
    failure_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2_module.imwrite(str(failure_path), qr_result.frame):
        raise RuntimeError(f"failed to save QR failure frame: {failure_path}")

    return str(relative_path)


class QRRecognitionWorker:
    """Recognize and persist QR results without blocking gimbal control."""

    def __init__(self, scanner, cv2_module, run_dir, result_file, result_writer):
        self.scanner = scanner
        self.cv2_module = cv2_module
        self.run_dir = run_dir
        self.result_file = result_file
        self.result_writer = result_writer
        self.jobs = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name="qr-recognition",
            daemon=True,
        )

    def start(self):
        self.thread.start()

    def submit(self, attempt, deadline_ns):
        """Schedule one recognition window, dropping it if the worker is still busy."""
        try:
            self.jobs.put_nowait((attempt, deadline_ns))
            return True
        except queue.Full:
            print(f"[QR WARN] skipped attempt {attempt}: QR worker is still busy")
            return False

    def _run(self):
        while not self.stop_event.is_set():
            try:
                job = self.jobs.get(timeout=0.1)
            except queue.Empty:
                continue

            if job is None:
                self.jobs.task_done()
                break

            attempt, deadline_ns = job
            try:
                qr_result = self.scanner.detect_until(deadline_ns)
                failure_frame = save_qr_failure_frame(
                    self.cv2_module,
                    self.run_dir,
                    attempt,
                    qr_result,
                )
                self.result_writer.writerow(
                    {
                        "qr_visible": int(qr_result.visible),
                        "qr_decoded": int(qr_result.detected),
                        "qr_data": qr_result.data or "",
                        "failure_frame": failure_frame,
                    }
                )
                self.result_file.flush()
                print(
                    "[QR]\n"
                    f"  attempt            : {attempt}\n"
                    f"  qr_visible         : {int(qr_result.visible)}\n"
                    f"  qr_decoded         : {int(qr_result.detected)}\n"
                    f"  qr_data            : {qr_result.data or ''}\n"
                    f"  failure_frame      : {failure_frame}"
                )
            except Exception as exc:
                print(f"[QR WARN] attempt {attempt} failed: {exc}")
            finally:
                self.jobs.task_done()

    def stop(self):
        self.stop_event.set()
        try:
            self.jobs.put_nowait(None)
        except queue.Full:
            pass
        self.thread.join()


def main():
    parser = argparse.ArgumentParser(
        description="Hardware test: track a moving UWB target with yaw gimbal."
    )
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--port", type=int, default=5005, help="UDP bind port")
    parser.add_argument("--servo-channel", type=int, default=0, help="PCA9685 channel for yaw servo")
    parser.add_argument("--pca9685-address", type=lambda value: int(value, 0), default=0x40)
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
        "--camera-warmup",
        type=float,
        default=1.0,
        help="camera warmup time in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--output-dir",
        default=str(Path(PROJECT_ROOT) / "result" / "gimbal_uwb_tracking_qr_test"),
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
    if args.camera_warmup < 0:
        parser.error("--camera-warmup must be 0 or greater")

    import cv2
    from qr.realsense_scanner import HardwareScanner

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(0.2)

    gimbal = GimbalController(
        servo_channel=args.servo_channel,
        pca9685_address=args.pca9685_address,
    )
    qr_scanner = HardwareScanner(
        device_index=args.qr_device_index,
        crop_scale=args.qr_crop_scale,
        live_stream=args.live_stream,
    )
    source_printed = False
    qr_capture_started = False
    qr_worker = None
    result_file = None
    attempt = 0

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
        qr_worker = QRRecognitionWorker(
            qr_scanner,
            cv2,
            run_dir,
            result_file,
            result_writer,
        )
        qr_worker.start()

        gimbal_command_deg = gimbal.move_to(args.initial_deg)
        print(
            f"[READY] listening on {args.host}:{args.port}, "
            f"initial gimbal_command_deg={gimbal_command_deg:.2f}"
        )
        print(f"[RESULT] QR results will be saved to {results_path}")
        print(
            f"[INFO] Gimbal tracking runs every {TRACKING_INTERVAL_SEC:.1f}s "
            "using only the latest UWB packet."
        )
        print("[INFO] QR recognition runs in parallel until the next alignment.")
        print("[INFO] Move the opposite UWB module. Press Ctrl+C to stop.")

        next_update_ns = time.monotonic_ns()
        while True:
            wait_until_next_update(next_update_ns)
            next_update_ns += int(TRACKING_INTERVAL_SEC * 1_000_000_000)

            latest_packet = receive_latest_available_packet(sock)
            if latest_packet is None:
                continue
            data, addr = latest_packet

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
                attempt += 1
                qr_worker.submit(attempt, next_update_ns)

                print(
                    "[TRACK]\n"
                    f"  relative_deg       : {uwb_relative_deg:.2f}\n"
                    f"  correction_deg     : {correction_deg:.2f}\n"
                    f"  prev_gimbal_deg    : {before_command_deg:.2f}\n"
                    f"  gimbal_command_deg : {gimbal_command_deg:.2f}\n"
                    f"  qr_attempt         : {attempt}"
                )
            except Exception as exc:
                print(f"[WARN] failed to process packet {data!r}: {exc}")

    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")
    finally:
        if qr_worker is not None:
            qr_worker.stop()
        if result_file is not None:
            result_file.close()
        if qr_capture_started:
            qr_scanner.stop()
        gimbal.move_to(0.0)
        time.sleep(gimbal.ALIGN_INTERVAL_SEC)
        gimbal.cleanup()
        sock.close()
        print("[DONE] gimbal returned to 0 deg and PCA9685 control signal disabled")


if __name__ == "__main__":
    main()
