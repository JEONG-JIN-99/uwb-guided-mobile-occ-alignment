"""Rx/Tx 동적 추적 실험에서 공유하는 UWB 수신과 각도 계산 도구."""

from dataclasses import dataclass
import math
import queue
import socket
import threading
import time


ALIGNMENT_PERIOD_SEC = 0.2
ALIGNMENT_PERIOD_NS = int(ALIGNMENT_PERIOD_SEC * 1_000_000_000)
UWB_DEADBAND_DEG = 1.0
MAX_CORRECTION_PER_ALIGNMENT_DEG = 60.0


@dataclass(frozen=True)
class UwbSample:
    distance_m: float
    raw_azimuth_deg: float
    elevation_deg: float
    source: str
    received_monotonic_ns: int


@dataclass(frozen=True)
class AlignmentCommand:
    uwb_ros_azimuth_deg: float
    correction_ros_deg: float
    target_calculated_ros_deg: float
    command_requested_ros_deg: float
    gimbal_command_ros_deg: float
    servo_clipped: bool


def parse_uwb_packet(data):
    """``1,distance,azimuth,elevation,...`` 형식의 유효 UWB 값을 반환한다."""
    message = data.decode("utf-8").strip()
    parts = message.split(",")
    if len(parts) < 4 or parts[0] != "1":
        return None

    distance, azimuth, elevation = map(float, parts[1:4])
    if not all(math.isfinite(value) for value in (distance, azimuth, elevation)):
        return None
    if distance < 0 or not -180.0 <= azimuth < 180.0:
        return None
    return distance, azimuth, elevation


def calculate_alignment(previous_gimbal_ros_deg, uwb_raw_azimuth_deg):
    """UWB 상대각으로 다음 동적 짐벌 명령을 계산한다.

    ``target_calculated_ros_deg``에는 제한 전 논리적 목표를 기록한다. 실제
    0.2초 명령에서는 서보 속도를 고려해 한 번의 보정을 60도로 제한한 뒤,
    최종적으로 서보 가동 범위인 -90~90도로 제한한다.
    """
    previous = float(previous_gimbal_ros_deg)
    uwb_raw = float(uwb_raw_azimuth_deg)
    uwb_ros = -uwb_raw
    target_calculated = previous + uwb_ros

    if abs(uwb_raw) < UWB_DEADBAND_DEG:
        correction_raw = 0.0
    else:
        correction_raw = max(
            -MAX_CORRECTION_PER_ALIGNMENT_DEG,
            min(MAX_CORRECTION_PER_ALIGNMENT_DEG, uwb_raw),
        )
    correction_ros = -correction_raw
    requested = previous + correction_ros
    command = max(-90.0, min(90.0, requested))
    return AlignmentCommand(
        uwb_ros_azimuth_deg=uwb_ros,
        correction_ros_deg=correction_ros,
        target_calculated_ros_deg=target_calculated,
        command_requested_ros_deg=requested,
        gimbal_command_ros_deg=command,
        servo_clipped=command != requested,
    )


class LatestUwbReceiver:
    """유효한 미처리 UWB 패킷 중 가장 최신 패킷 하나만 유지한다."""

    def __init__(self, host, port, warning_callback=print):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, int(port)))
        self.socket.settimeout(0.2)
        self._queue = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._warning_callback = warning_callback
        self._thread = threading.Thread(
            target=self._receive_loop,
            name="dynamic-tracking-uwb-receiver",
            daemon=True,
        )

    def start(self):
        self._thread.start()

    def _receive_loop(self):
        while not self._stop_event.is_set():
            try:
                data, address = self.socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            received_ns = time.monotonic_ns()
            try:
                parsed = parse_uwb_packet(data)
            except (UnicodeDecodeError, ValueError) as exc:
                self._warning_callback(
                    f"[WARN] ignored malformed UWB packet {data!r}: {exc}"
                )
                continue
            if parsed is None:
                continue

            distance, azimuth, elevation = parsed
            sample = UwbSample(
                distance_m=distance,
                raw_azimuth_deg=azimuth,
                elevation_deg=elevation,
                source=f"{address[0]}:{address[1]}",
                received_monotonic_ns=received_ns,
            )
            try:
                self._queue.put_nowait(sample)
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._queue.put_nowait(sample)
                except queue.Full:
                    # 소비자와 경합해 더 최신인 값이 이미 들어온 경우다.
                    pass

    def take_latest_after(self, cutoff_monotonic_ns):
        """기준시각 이후 받은 최신 미처리 패킷을 한 번만 반환한다."""
        try:
            sample = self._queue.get_nowait()
        except queue.Empty:
            return None
        if sample.received_monotonic_ns < int(cutoff_monotonic_ns):
            return None
        return sample

    def close(self):
        self._stop_event.set()
        self.socket.close()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


def distance_trajectory(distance_m):
    """거리 조건값에 대응하는 이동 경로 설명을 반환한다."""
    if int(distance_m) == 0:
        return "random_moving_rover"
    return "fixed_radius_arc"
