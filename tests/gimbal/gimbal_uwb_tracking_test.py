import argparse
import os
import socket
import sys
import threading
import time


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CODE_DIR = os.path.join(PROJECT_ROOT, "code")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

from gimbal.gimbal_controller_yaw import GimbalController


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


def main():
    parser = argparse.ArgumentParser(
        description="Hardware test: track a moving UWB target with yaw gimbal."
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
        "--no-wait",
        action="store_true",
        help="Do not wait inside move_by_uwb_relative after each command",
    )
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(0.2)

    gimbal = GimbalController(yaw_pin=args.yaw_pin)
    latest_packet = None
    latest_seq = 0
    packet_lock = threading.Lock()
    stop_event = threading.Event()
    source_printed = False

    def receiver_loop():
        nonlocal latest_packet, latest_seq

        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            recv_time_ns = time.time_ns()
            with packet_lock:
                latest_seq += 1
                latest_packet = (latest_seq, data, addr, recv_time_ns)

    receiver_thread = threading.Thread(target=receiver_loop, daemon=True)

    try:
        gimbal_command_deg = gimbal.move_to(args.initial_deg)
        print(
            f"[READY] listening on {args.host}:{args.port}, "
            f"initial gimbal_command_deg={gimbal_command_deg:.2f}"
        )
        receiver_thread.start()
        print("[INFO] Move the opposite UWB module. Press Ctrl+C to stop.")

        last_processed_seq = 0
        while True:
            with packet_lock:
                packet = latest_packet

            if packet is None:
                time.sleep(0.05)
                continue

            seq, data, addr, _recv_time_ns = packet
            if seq == last_processed_seq:
                time.sleep(0.05)
                continue
            last_processed_seq = seq

            try:
                parsed = parse_uwb_packet(data)
                if parsed is None:
                    continue

                _distance, uwb_relative_deg, _elevation = parsed
                if not source_printed:
                    print(f"[SOURCE] receiving UWB packets from {addr[0]}:{addr[1]}")
                    source_printed = True

                before_command_deg = gimbal.current_degree
                gimbal_command_deg = gimbal.move_by_uwb_relative(
                    uwb_relative_deg,
                    wait=not args.no_wait,
                )

                print(
                    "[TRACK]\n"
                    f"  relative_deg       : {uwb_relative_deg:.2f}\n"
                    f"  prev_gimbal_deg    : {before_command_deg:.2f}\n"
                    f"  gimbal_command_deg : {gimbal_command_deg:.2f}"
                )
            except Exception as exc:
                print(f"[WARN] failed to process packet {data!r}: {exc}")

    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")
    finally:
        stop_event.set()
        gimbal.move_to(0.0)
        time.sleep(gimbal.ALIGN_INTERVAL_SEC)
        gimbal.cleanup()
        sock.close()
        receiver_thread.join(timeout=1.0)
        print("[DONE] gimbal returned to 0 deg and GPIO cleaned up")


if __name__ == "__main__":
    main()
