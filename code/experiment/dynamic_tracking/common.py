"""Rx/Tx 동적 추적 실험에서 공유하는 UWB 수신과 각도 계산 도구."""

from dataclasses import dataclass
import math
import queue
import socket
import threading
import time


ALIGNMENT_PERIOD_SEC = 0.2
ALIGNMENT_PERIOD_NS = int(ALIGNMENT_PERIOD_SEC * 1_000_000_000)
MAX_CORRECTION_PER_ALIGNMENT_DEG = 60.0


@dataclass(frozen=True)
class UwbSample:
    distance_m: float
    raw_azimuth_deg: float
    elevation_deg: float
    source: str
    received_monotonic_ns: int


@dataclass(frozen=True)
class UwbSelection:
    sample: UwbSample
    reused: bool


@dataclass(frozen=True)
class AlignmentCommand:
    uwb_corrected_azimuth_deg: float
    uwb_ros_azimuth_deg: float
    correction_ros_deg: float
    target_calculated_ros_deg: float
    command_requested_ros_deg: float
    gimbal_command_ros_deg: float
    servo_clipped: bool


@dataclass(frozen=True)
class UwbCalibrationResult:
    sample_count: int
    bias_deg: float
    circular_std_deg: float
    started_monotonic_ns: int
    completed_monotonic_ns: int


class UwbCalibrationError(RuntimeError):
    """UWB 영점 편향을 신뢰할 수 있게 계산하지 못한 경우."""


def normalize_angle_deg(angle_deg):
    """각도를 ``[-180, 180)`` 범위로 정규화한다."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def calculate_circular_statistics(angles_deg):
    """각도 표본의 원형 평균과 원형 표준편차를 degree 단위로 반환한다."""
    angles = [float(angle) for angle in angles_deg]
    if not angles:
        raise ValueError("at least one angle is required")
    if not all(math.isfinite(angle) for angle in angles):
        raise ValueError("all angles must be finite")

    mean_sin = sum(math.sin(math.radians(angle)) for angle in angles) / len(
        angles
    )
    mean_cos = sum(math.cos(math.radians(angle)) for angle in angles) / len(
        angles
    )
    bias_deg = normalize_angle_deg(
        math.degrees(math.atan2(mean_sin, mean_cos))
    )
    resultant_length = min(1.0, math.hypot(mean_sin, mean_cos))
    if resultant_length <= 0.0:
        circular_std_deg = math.inf
    else:
        circular_std_deg = math.degrees(
            math.sqrt(-2.0 * math.log(resultant_length))
        )
    return bias_deg, circular_std_deg


def calibration_deadline_monotonic_ns(start_utc_ns, margin_sec):
    """UTC 시작시각보다 지정한 여유시간만큼 앞선 monotonic 시각을 구한다."""
    wall_now_ns = time.time_ns()
    monotonic_now_ns = time.monotonic_ns()
    return (
        monotonic_now_ns
        + int(start_utc_ns)
        - wall_now_ns
        - int(float(margin_sec) * 1_000_000_000)
    )


class UwbBiasCalibrator:
    """수신 스레드에서 전달된 서로 다른 UWB 패킷으로 편향을 계산한다."""

    def __init__(self):
        self._condition = threading.Condition()
        self._target_sample_count = 0
        self._angles_deg = []
        self._collecting = False
        self._started_monotonic_ns = 0
        self._completed_monotonic_ns = 0

    def begin(self, sample_count):
        sample_count = int(sample_count)
        if sample_count <= 0:
            raise ValueError("sample_count must be greater than 0")
        with self._condition:
            if self._collecting:
                raise RuntimeError("UWB calibration is already in progress")
            self._target_sample_count = sample_count
            self._angles_deg = []
            self._collecting = True
            self._started_monotonic_ns = time.monotonic_ns()
            self._completed_monotonic_ns = 0

    def add_sample(self, sample):
        """캘리브레이션 중일 때 새로 수신된 패킷 하나를 한 번만 누적한다."""
        with self._condition:
            if not self._collecting:
                return
            self._angles_deg.append(float(sample.raw_azimuth_deg))
            if len(self._angles_deg) >= self._target_sample_count:
                self._collecting = False
                self._completed_monotonic_ns = time.monotonic_ns()
                self._condition.notify_all()

    def wait_for_result(
        self,
        deadline_monotonic_ns,
        *,
        max_std_deg,
        max_abs_bias_deg,
    ):
        """마감시각까지 수집을 기다리고 품질 기준을 만족하는 결과를 반환한다."""
        deadline_ns = int(deadline_monotonic_ns)
        with self._condition:
            while self._collecting:
                remaining_ns = deadline_ns - time.monotonic_ns()
                if remaining_ns <= 0:
                    self._collecting = False
                    break
                self._condition.wait(timeout=remaining_ns / 1_000_000_000)

            collected = len(self._angles_deg)
            target = self._target_sample_count
            if collected < target:
                raise UwbCalibrationError(
                    "UWB calibration deadline reached after collecting "
                    f"{collected}/{target} packets"
                )
            angles_deg = tuple(self._angles_deg)
            started_ns = self._started_monotonic_ns
            completed_ns = self._completed_monotonic_ns

        bias_deg, circular_std_deg = calculate_circular_statistics(angles_deg)
        if circular_std_deg > float(max_std_deg):
            raise UwbCalibrationError(
                "UWB calibration is unstable: circular std "
                f"{circular_std_deg:.3f} deg exceeds "
                f"{float(max_std_deg):.3f} deg"
            )
        if abs(bias_deg) > float(max_abs_bias_deg):
            raise UwbCalibrationError(
                "UWB calibration bias is too large: "
                f"{bias_deg:.3f} deg exceeds +/-"
                f"{float(max_abs_bias_deg):.3f} deg"
            )
        return UwbCalibrationResult(
            sample_count=collected,
            bias_deg=bias_deg,
            circular_std_deg=circular_std_deg,
            started_monotonic_ns=started_ns,
            completed_monotonic_ns=completed_ns,
        )

    def cancel(self):
        with self._condition:
            self._collecting = False
            self._condition.notify_all()


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


def calculate_alignment(
    previous_gimbal_ros_deg,
    uwb_raw_azimuth_deg,
    uwb_bias_deg=0.0,
):
    """UWB 상대각으로 다음 동적 짐벌 명령을 계산한다.

    ``target_calculated_ros_deg``에는 제한 전 논리적 목표를 기록한다. 실제
    0.2초 명령에서는 서보 속도를 고려해 한 번의 보정을 60도로 제한한 뒤,
    최종적으로 서보 가동 범위인 -90~90도로 제한한다.
    """
    previous = float(previous_gimbal_ros_deg)
    uwb_raw = float(uwb_raw_azimuth_deg)
    uwb_corrected = normalize_angle_deg(uwb_raw - float(uwb_bias_deg))
    uwb_ros = -uwb_corrected
    target_calculated = previous + uwb_ros

    correction_raw = max(
        -MAX_CORRECTION_PER_ALIGNMENT_DEG,
        min(MAX_CORRECTION_PER_ALIGNMENT_DEG, uwb_corrected),
    )
    correction_ros = -correction_raw
    requested = previous + correction_ros
    command = max(-90.0, min(90.0, requested))
    return AlignmentCommand(
        uwb_corrected_azimuth_deg=uwb_corrected,
        uwb_ros_azimuth_deg=uwb_ros,
        correction_ros_deg=correction_ros,
        target_calculated_ros_deg=target_calculated,
        command_requested_ros_deg=requested,
        gimbal_command_ros_deg=command,
        servo_clipped=command != requested,
    )


class LatestUwbReceiver:
    """최신 UWB 패킷을 유지하고 설정에 따라 마지막 선택값을 재사용한다."""

    def __init__(
        self,
        host,
        port,
        warning_callback=print,
        reuse_latest=False,
    ):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, int(port)))
        self.socket.settimeout(0.2)
        self._queue = queue.Queue(maxsize=1)
        self._calibrator = UwbBiasCalibrator()
        self._reuse_latest = bool(reuse_latest)
        self._last_selection_sample = None
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
            self._calibrator.add_sample(sample)
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

    def calibrate(
        self,
        sample_count,
        deadline_monotonic_ns,
        *,
        max_std_deg,
        max_abs_bias_deg,
    ):
        """수신을 유지하면서 시작 시각 전 UWB 영점 편향을 계산한다."""
        self._calibrator.begin(sample_count)
        return self._calibrator.wait_for_result(
            deadline_monotonic_ns,
            max_std_deg=max_std_deg,
            max_abs_bias_deg=max_abs_bias_deg,
        )

    def take_latest_after(self, cutoff_monotonic_ns):
        """기준시각 이후 최신 패킷과 재사용 여부를 반환한다."""
        try:
            sample = self._queue.get_nowait()
        except queue.Empty:
            if self._reuse_latest and self._last_selection_sample is not None:
                return UwbSelection(
                    sample=self._last_selection_sample,
                    reused=True,
                )
            return None
        if sample.received_monotonic_ns < int(cutoff_monotonic_ns):
            self._last_selection_sample = None
            return None
        self._last_selection_sample = sample
        return UwbSelection(sample=sample, reused=False)

    def close(self):
        self._stop_event.set()
        self._calibrator.cancel()
        self.socket.close()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


def distance_trajectory(distance_m):
    """거리 조건값에 대응하는 이동 경로 설명을 반환한다."""
    if int(distance_m) == 0:
        return "random_moving_rover"
    return "fixed_radius_arc"
