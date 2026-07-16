import argparse
import socket
import time


def sweep_relative_steps(start, stop, step):
    if step <= 0:
        raise ValueError("step must be positive")

    values = []
    current_position = 0.0
    direction = 1.0

    while True:
        next_position = current_position + (direction * step)
        if next_position >= stop:
            next_position = stop
            direction = -1.0
        elif next_position <= start:
            next_position = start
            direction = 1.0

        values.append(next_position - current_position)
        current_position = next_position

        if current_position == 0.0 and direction > 0 and len(values) > 1:
            break

    return values


def main():
    parser = argparse.ArgumentParser(
        description="Send fake relative UWB azimuth packets for gimbal tracking tests."
    )
    parser.add_argument("--host", default="127.0.0.1", help="UDP target host")
    parser.add_argument("--port", type=int, default=5005, help="UDP target port")
    parser.add_argument("--start", type=float, default=-80.0, help="Minimum simulated gimbal angle")
    parser.add_argument("--stop", type=float, default=80.0, help="Maximum simulated gimbal angle")
    parser.add_argument("--step", type=float, default=10.0, help="Relative azimuth step")
    parser.add_argument("--interval", type=float, default=0.7, help="Seconds between packets")
    parser.add_argument("--distance", type=float, default=100.0, help="Fake UWB distance")
    parser.add_argument("--elevation", type=float, default=0.0, help="Fake UWB elevation")
    args = parser.parse_args()

    values = sweep_relative_steps(args.start, args.stop, args.step)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(
        f"[FAKE UWB] sending to {args.host}:{args.port}, "
        f"relative step=+/-{args.step}, simulated range={args.start}..{args.stop}, "
        f"interval={args.interval}s"
    )
    print("[FAKE UWB] Press Ctrl+C to stop.")

    try:
        while True:
            for relative_azimuth in values:
                read_time_ns = time.time_ns()
                message = (
                    f"1,{args.distance:.2f},{relative_azimuth:.2f},"
                    f"{args.elevation:.2f},0,0,{read_time_ns}"
                )
                sock.sendto(message.encode("utf-8"), (args.host, args.port))
                print(f"[SEND] relative_az={relative_azimuth:.2f} packet={message}")
                time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
