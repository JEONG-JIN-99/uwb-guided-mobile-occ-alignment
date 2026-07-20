import argparse
import queue
import socket
import threading
import time

from gimbal.gimbal_controller_yaw import GimbalController
from logger.result_logger import ResultLogger
from time_sync.chrony_clock import (
    format_utc_epoch_ns,
    parse_utc_epoch_ns,
    wait_for_chrony_sync,
    wait_until_utc_ns,
)


UWB_DEADBAND_DEG = 1.0
MAX_CORRECTION_PER_PACKET_DEG = 60.0
DEFAULT_NOMINAL_PERIOD_SEC = 0.2

RX_LOG_FIELDS = (
    "experiment_id",
    "node_id",
    "sample_index",
    "nominal_elapsed_sec",
    "actual_elapsed_sec",
    "rx_uwb_azimuth_deg",
    "rx_correction_deg",
    "rx_gimbal_command_deg",
)

TX_LOG_FIELDS = (
    "experiment_id",
    "node_id",
    "sample_index",
    "nominal_elapsed_sec",
    "actual_elapsed_sec",
    "tx_uwb_azimuth_deg",
    "tx_gimbal_command_deg",
)


def parse_uwb_packet(data):
    message = data.decode("utf-8").strip()
    parts = message.split(",")
    if len(parts) < 4:
        raise ValueError(f"packet has too few fields: {message}")
    if parts[0] != "1":
        return None

    distance = float(parts[1])
    azimuth = float(parts[2])
    elevation = float(parts[3])
    return distance, azimuth, elevation


def limit_uwb_correction(uwb_relative_deg):
    if abs(uwb_relative_deg) < UWB_DEADBAND_DEG:
        return 0.0

    return max(
        -MAX_CORRECTION_PER_PACKET_DEG,
        min(MAX_CORRECTION_PER_PACKET_DEG, uwb_relative_deg),
    )


def build_parser(node_id):
    parser = argparse.ArgumentParser(
        description=(
            f"{node_id.upper()} packet-immediate experiment: align on every "
            "latest valid UWB packet received after the shared UTC start."
        )
    )
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--port", type=int, default=5005, help="UDP bind port")
    parser.add_argument("--yaw-pin", type=int, default=18, help="BCM GPIO pin for yaw servo")
    parser.add_argument(
        "--initial-deg",
        type=float,
        default=0.0,
        help="Initial gimbal command angle in degrees, -90 to 90",
    )
    parser.add_argument(
        "--samples",
        type=int,
        required=True,
        help="Number of valid UWB packets to align and log before stopping",
    )
    parser.add_argument(
        "--experiment-id",
        required=True,
        help="Shared experiment ID used by both Tx and Rx",
    )
    parser.add_argument(
        "--start-utc",
        type=parse_utc_epoch_ns,
        required=True,
        metavar="EPOCH_SEC",
        help="Shared future UTC start as Unix epoch seconds",
    )
    parser.add_argument(
        "--nominal-period-sec",
        type=float,
        default=DEFAULT_NOMINAL_PERIOD_SEC,
        help=(
            "Expected UWB period used only for nominal_elapsed_sec "
            "(default: 0.2)"
        ),
    )
    parser.add_argument(
        "--chrony-max-correction-sec",
        type=float,
        default=0.005,
        help="Maximum Chrony remaining correction before start (default: 0.005)",
    )
    parser.add_argument(
        "--chrony-wait-tries",
        type=int,
        default=60,
        help="Maximum one-second Chrony synchronization checks (default: 60)",
    )
    return parser


def run_packet_immediate_experiment(node_id):
    node_id = node_id.lower()
    if node_id not in ("rx", "tx"):
        raise ValueError("node_id must be 'rx' or 'tx'")

    parser = build_parser(node_id)
    args = parser.parse_args()
    if args.samples <= 0:
        parser.error("--samples must be greater than zero")
    if args.nominal_period_sec <= 0:
        parser.error("--nominal-period-sec must be greater than zero")
    if args.chrony_max_correction_sec <= 0:
        parser.error("--chrony-max-correction-sec must be greater than zero")
    if args.chrony_wait_tries <= 0:
        parser.error("--chrony-wait-tries must be greater than zero")

    print("[SYNC] waiting for local Chrony synchronization")
    wait_for_chrony_sync(
        max_tries=args.chrony_wait_tries,
        max_correction_sec=args.chrony_max_correction_sec,
    )
    print("[SYNC] Chrony synchronization is ready")

    gimbal = GimbalController(yaw_pin=args.yaw_pin)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(0.2)

    packet_queue = queue.Queue(maxsize=1)
    stop_event = threading.Event()
    source_printed = False
    result_logger = None
    receiver_started = False

    def receiver_loop():
        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            latest_packet = (data, addr, time.monotonic_ns())
            try:
                packet_queue.put_nowait(latest_packet)
            except queue.Full:
                try:
                    packet_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    packet_queue.put_nowait(latest_packet)
                except queue.Full:
                    pass

    receiver_thread = threading.Thread(target=receiver_loop, daemon=True)
    log_fields = RX_LOG_FIELDS if node_id == "rx" else TX_LOG_FIELDS

    try:
        result_logger = ResultLogger(
            target_dir_name="result",
            experiment_id=args.experiment_id,
            node_id=node_id,
            fieldnames=log_fields,
        )
        gimbal_command_deg = gimbal.move_to(args.initial_deg)
        print(
            f"[READY] listening on {args.host}:{args.port}, "
            f"initial gimbal_command_deg={gimbal_command_deg:.2f}"
        )
        print(
            f"[LOG] experiment_id={result_logger.experiment_id}, "
            f"csv={result_logger.csv_path}"
        )
        receiver_thread.start()
        receiver_started = True
        print(
            f"[WAIT] shared UTC start={format_utc_epoch_ns(args.start_utc)}"
        )

        experiment_start_ns = wait_until_utc_ns(args.start_utc)
        print("[START] shared UTC start reached; packet-immediate control enabled")

        sample_index = 0
        while sample_index < args.samples:
            try:
                data, addr, recv_monotonic_ns = packet_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            # Do not use a packet measured before the shared experiment start.
            if recv_monotonic_ns < experiment_start_ns:
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
                alignment_time_ns = time.monotonic_ns()
                nominal_elapsed_sec = (
                    sample_index * args.nominal_period_sec
                )
                actual_elapsed_sec = (
                    alignment_time_ns - experiment_start_ns
                ) / 1_000_000_000

                log_row = {
                    "sample_index": sample_index,
                    "nominal_elapsed_sec": f"{nominal_elapsed_sec:.6f}",
                    "actual_elapsed_sec": f"{actual_elapsed_sec:.6f}",
                    f"{node_id}_uwb_azimuth_deg": f"{uwb_relative_deg:.6f}",
                    f"{node_id}_gimbal_command_deg": f"{gimbal_command_deg:.6f}",
                }
                if node_id == "rx":
                    log_row["rx_correction_deg"] = f"{correction_deg:.6f}"
                result_logger.log_sample(log_row)

                print(
                    f"[TRACK] sample {sample_index + 1}/{args.samples}\n"
                    f"  relative_deg       : {uwb_relative_deg:.2f}\n"
                    f"  correction_deg     : {correction_deg:.2f}\n"
                    f"  prev_gimbal_deg    : {before_command_deg:.2f}\n"
                    f"  gimbal_command_deg : {gimbal_command_deg:.2f}\n"
                    f"  nominal_elapsed_sec: {nominal_elapsed_sec:.3f}\n"
                    f"  actual_elapsed_sec : {actual_elapsed_sec:.3f}"
                )
                sample_index += 1
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
        print("[DONE] gimbal returned to 0 deg and GPIO cleaned up")
