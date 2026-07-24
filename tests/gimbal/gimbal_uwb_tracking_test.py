import argparse
import os
import socket
import sys
import time


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CODE_DIR = os.path.join(PROJECT_ROOT, "code")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

from gimbal.gimbal_controller_yaw import GimbalController


TRACKING_INTERVAL_SEC = 0.2
UWB_DEADBAND_DEG = 0.0
MAX_CORRECTION_PER_FRAME_DEG = 60.0


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

    if parts[0] != "1":
        return None

    distance = float(parts[1])
    azimuth = float(parts[2])
    elevation = float(parts[3])
    return distance, azimuth, elevation


def limit_uwb_correction(uwb_relative_deg):
    """Apply the tracking deadband and per-update correction limit."""
    if abs(uwb_relative_deg) < UWB_DEADBAND_DEG:
        return 0.0

    return max(
        -MAX_CORRECTION_PER_FRAME_DEG,
        min(MAX_CORRECTION_PER_FRAME_DEG, uwb_relative_deg),
    )


def receive_latest_packet(sock, buffer_size=1024):
    """Wait for a packet, discard the backlog, and return only the newest one."""
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
    """Keep the tracking loop on a fixed 0.2-second cadence."""
    remaining_ns = next_update_ns - time.monotonic_ns()
    if remaining_ns > 0:
        time.sleep(remaining_ns / 1_000_000_000)


def main():
    parser = argparse.ArgumentParser(
        description="Hardware test: track the latest UWB angle every 0.2 seconds."
    )
    parser.add_argument("--host", default="0.0.0.0", help="UDP bind host")
    parser.add_argument("--port", type=int, default=5005, help="UDP bind port")
    parser.add_argument("--servo-channel", type=int, default=0, help="PCA9685 channel for yaw servo")
    parser.add_argument("--pca9685-address", type=lambda value: int(value, 0), default=0x40)
    parser.add_argument(
        "--initial-deg",
        type=float,
        default=0.0,
        help="Initial gimbal command angle in degrees, -90 to 90",
    )
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    sock.settimeout(TRACKING_INTERVAL_SEC)

    gimbal = GimbalController(
        servo_channel=args.servo_channel,
        pca9685_address=args.pca9685_address,
    )
    source_printed = False

    try:
        gimbal_command_deg = gimbal.move_to(args.initial_deg)
        print(
            f"[READY] listening on {args.host}:{args.port}, "
            f"initial gimbal_command_deg={gimbal_command_deg:.2f}"
        )
        print(
            f"[INFO] Tracking every {TRACKING_INTERVAL_SEC:.1f}s with only the "
            "latest UWB packet. Press Ctrl+C to stop."
        )

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
                print(
                    "[TRACK]\n"
                    f"  relative_deg       : {uwb_relative_deg:.2f}\n"
                    f"  correction_deg     : {correction_deg:.2f}\n"
                    f"  prev_gimbal_deg    : {before_command_deg:.2f}\n"
                    f"  gimbal_command_deg : {gimbal_command_deg:.2f}"
                )
            except Exception as exc:
                print(f"[WARN] failed to process packet {data!r}: {exc}")

    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")
    finally:
        gimbal.move_to(0.0)
        time.sleep(gimbal.ALIGN_INTERVAL_SEC)
        gimbal.cleanup()
        sock.close()
        print("[DONE] gimbal returned to 0 deg and PCA9685 control signal disabled")


if __name__ == "__main__":
    main()
