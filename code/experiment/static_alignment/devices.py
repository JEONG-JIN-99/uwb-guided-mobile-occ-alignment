import math
import socket
import time


def parse_uwb_packet(data):
    message = data.decode("utf-8").strip()
    parts = message.split(",")
    if len(parts) < 4 or parts[0] != "1":
        return None
    distance, azimuth, elevation = map(float, parts[1:4])
    if not all(math.isfinite(value) for value in (distance, azimuth, elevation)):
        return None
    if not -180.0 <= azimuth < 180.0:
        return None
    return distance, azimuth, elevation


class UwbReceiver:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, int(port)))
        self.socket.setblocking(False)

    def discard_pending(self):
        while True:
            try:
                self.socket.recvfrom(4096)
            except BlockingIOError:
                return

    def receive_first_valid(self, timeout_s):
        deadline_ns = time.monotonic_ns() + int(timeout_s * 1_000_000_000)
        self.socket.setblocking(True)
        try:
            while True:
                remaining_s = (deadline_ns - time.monotonic_ns()) / 1_000_000_000
                if remaining_s <= 0:
                    return None
                self.socket.settimeout(remaining_s)
                try:
                    data, address = self.socket.recvfrom(4096)
                except socket.timeout:
                    return None
                received_ns = time.monotonic_ns()
                try:
                    parsed = parse_uwb_packet(data)
                except (UnicodeDecodeError, ValueError):
                    continue
                if parsed is not None:
                    return parsed, received_ns, address
        finally:
            self.socket.setblocking(False)

    def close(self):
        self.socket.close()
