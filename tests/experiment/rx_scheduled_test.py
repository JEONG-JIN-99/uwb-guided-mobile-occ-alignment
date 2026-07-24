import argparse
import os
import queue
import socket
import sys
import threading
import time


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CODE_DIR = os.path.join(PROJECT_ROOT, "code")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

from gimbal.gimbal_controller_yaw import GimbalController
from logger.result_logger import ResultLogger


UWB_DEADBAND_DEG = 1.0
MAX_CORRECTION_DEG = 60.0
DEFAULT_ALIGNMENT_PERIOD_SEC = 0.2


def parse_uwb_packet(data):
    parts = data.decode("utf-8").strip().split(",")
    if len(parts) < 4:
        raise ValueError(f"packet has too few fields: {data!r}")
    if parts[0] != "1":
        return None
    return float(parts[1]), float(parts[2]), float(parts[3])


def limit_correction(relative_deg):
    if abs(relative_deg) < UWB_DEADBAND_DEG:
        return 0.0
    return max(-MAX_CORRECTION_DEG, min(MAX_CORRECTION_DEG, relative_deg))


def log_fields(node_id):
    fields = [
        "experiment_id",
        "node_id",
        "sample_index",
        "nominal_elapsed_sec",
        "actual_elapsed_sec",
        f"{node_id}_uwb_azimuth_deg",
    ]
    if node_id == "rx":
        fields.append("rx_correction_deg")
    fields.append(f"{node_id}_gimbal_command_deg")
    return tuple(fields)


def run_scheduled_experiment(node_id="rx"):
    parser = argparse.ArgumentParser(
        description=(
            f"{node_id.upper()} quick test: align at a fixed local period "
            "without Chrony or a shared UTC start."
        )
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--servo-channel", type=int, default=0)
    parser.add_argument("--pca9685-address", type=lambda value: int(value, 0), default=0x40)
    parser.add_argument("--initial-deg", type=float, default=0.0)
    parser.add_argument("--samples", type=int, required=True)
    parser.add_argument(
        "--alignment-period-sec",
        type=float,
        default=DEFAULT_ALIGNMENT_PERIOD_SEC,
    )
    parser.add_argument(
        "--experiment-id",
        help="Optional local experiment ID; generated automatically if omitted",
    )
    args = parser.parse_args()
    if args.samples <= 0:
        parser.error("--samples must be greater than zero")
    if args.alignment_period_sec <= 0:
        parser.error("--alignment-period-sec must be greater than zero")

    gimbal = GimbalController(
        servo_channel=args.servo_channel,
        pca9685_address=args.pca9685_address,
    )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(0.2)

    packet_queue = queue.Queue(maxsize=1)
    stop_event = threading.Event()
    receiver_started = False
    result_logger = None

    def receiver_loop():
        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            packet = (data, addr, time.monotonic_ns())
            try:
                packet_queue.put_nowait(packet)
            except queue.Full:
                try:
                    packet_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    packet_queue.put_nowait(packet)
                except queue.Full:
                    pass

    receiver_thread = threading.Thread(target=receiver_loop, daemon=True)

    try:
        result_logger = ResultLogger(
            target_dir_name="result",
            experiment_code=f"{node_id}_scheduled_test",
            experiment_id=args.experiment_id,
            node_id=node_id,
            fieldnames=log_fields(node_id),
        )
        gimbal.move_to(args.initial_deg)
        receiver_thread.start()
        receiver_started = True
        print(
            f"[READY] {node_id.upper()} scheduled quick test, "
            f"period={args.alignment_period_sec:.3f}s, "
            f"csv={result_logger.csv_path}"
        )
        print("[START] the first valid UWB packet starts the local experiment")

        experiment_start_ns = None
        next_alignment_ns = None
        period_ns = int(args.alignment_period_sec * 1_000_000_000)
        sample_index = 0

        while sample_index < args.samples:
            if next_alignment_ns is not None:
                remaining_ns = next_alignment_ns - time.monotonic_ns()
                if remaining_ns > 0:
                    time.sleep(remaining_ns / 1_000_000_000)

            try:
                data, _addr, recv_ns = packet_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                parsed = parse_uwb_packet(data)
                if parsed is None:
                    continue
                _distance, relative_deg, _elevation = parsed

                if experiment_start_ns is None:
                    experiment_start_ns = recv_ns

                correction_deg = limit_correction(relative_deg)
                previous_deg = gimbal.current_degree
                command_deg = gimbal.move_by_uwb_relative(
                    correction_deg,
                    wait=False,
                )
                command_ns = time.monotonic_ns()
                nominal_sec = sample_index * args.alignment_period_sec
                actual_sec = (
                    command_ns - experiment_start_ns
                ) / 1_000_000_000

                row = {
                    "sample_index": sample_index,
                    "nominal_elapsed_sec": f"{nominal_sec:.6f}",
                    "actual_elapsed_sec": f"{actual_sec:.6f}",
                    f"{node_id}_uwb_azimuth_deg": f"{relative_deg:.6f}",
                    f"{node_id}_gimbal_command_deg": f"{command_deg:.6f}",
                }
                if node_id == "rx":
                    row["rx_correction_deg"] = f"{correction_deg:.6f}"
                result_logger.log_sample(row)

                print(
                    f"[TRACK] sample {sample_index + 1}/{args.samples}\n"
                    f"  relative_deg       : {relative_deg:.2f}\n"
                    f"  correction_deg     : {correction_deg:.2f}\n"
                    f"  prev_gimbal_deg    : {previous_deg:.2f}\n"
                    f"  gimbal_command_deg : {command_deg:.2f}\n"
                    f"  nominal_elapsed_sec: {nominal_sec:.3f}\n"
                    f"  actual_elapsed_sec : {actual_sec:.3f}"
                )
                sample_index += 1
                next_alignment_ns = (
                    experiment_start_ns + sample_index * period_ns
                )
            except Exception as exc:
                print(f"[WARN] failed to process packet {data!r}: {exc}")

        print(f"[COMPLETE] recorded {sample_index} alignments")
    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")
    finally:
        stop_event.set()
        if result_logger is not None:
            result_logger.close()
        gimbal.move_to(0.0)
        time.sleep(gimbal.ALIGN_INTERVAL_SEC)
        gimbal.cleanup()
        sock.close()
        if receiver_started:
            receiver_thread.join(timeout=1.0)
        print("[DONE] gimbal returned to 0 deg and PCA9685 control signal disabled")


if __name__ == "__main__":
    run_scheduled_experiment("rx")
