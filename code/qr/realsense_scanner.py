"""RealSense 컬러 카메라를 이용한 QR 탐지 도구.

실험에서는 ``run()`` 대신 다음 순서로 사용하는 것을 전제로 한다.

1. ``start_capture()``로 카메라 수신 스레드를 시작한다.
2. 짐벌 정렬 명령을 내린다.
3. ``detect_until(next_alignment_ns)``로 다음 정렬 시각까지만 QR을 찾는다.
4. 실험 종료 시 ``stop()``으로 카메라와 스레드를 정리한다.

수신 스레드는 프레임을 큐에 쌓지 않고 가장 최신 프레임 한 장만 보관한다.
따라서 오래된 프레임이 누적되거나 메모리 사용량이 계속 증가하지 않는다.
"""

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Optional

import cv2
from pyzbar import pyzbar


@dataclass(frozen=True)
class FrameSnapshot:
    """수신 프레임과 프레임의 순서/수신 시각을 함께 보관한다."""

    frame: Any
    frame_id: int
    captured_ns: int


@dataclass(frozen=True)
class QRDetectionResult:
    """한 번의 QR 탐지 결과.

    성공하면 ``frame``에 실제 QR 인식에 사용된 프레임이 들어간다.
    실패하면 제한시간 동안 마지막으로 검사한 최신 프레임이 들어간다.
    카메라에서 새 프레임을 전혀 받지 못한 경우에만 ``frame``이 None이다.
    """

    visible: bool = False
    detected: bool = False
    frame: Optional[Any] = None
    data: Optional[str] = None
    barcode_type: Optional[str] = None
    distance_px: Optional[float] = None
    frame_id: Optional[int] = None
    captured_ns: Optional[int] = None
    rect: Optional[tuple[int, int, int, int]] = None


class HardwareScanner:
    def __init__(self, device_index=0, crop_scale=1.0, live_stream=False):
        """
        Args:
            device_index: ``/dev/videoX``에서 X에 해당하는 정수값.
            crop_scale: 중앙을 기준으로 사용할 영상 영역의 비율.
            live_stream: True이면 OpenCV 창에 수신 영상을 표시한다.
        """
        self.device_index = device_index
        self.cap = None
        self.last_data = None
        self.is_running = False
        self.crop_scale = crop_scale
        self.live_stream = live_stream
        self.qr_detector = cv2.QRCodeDetector()

        # 카메라 수신 스레드의 생명주기를 관리한다.
        self.capture_thread = None
        self.stop_event = threading.Event()
        self.capture_error = None

        # 최신 프레임은 이 조건 변수의 lock을 잡은 상태에서만 교체/조회한다.
        # Condition을 사용하면 QR 탐지 쪽에서 busy-wait 없이 새 프레임을 기다릴 수 있다.
        self.frame_condition = threading.Condition()
        self.latest_frame = None
        self.latest_frame_id = 0
        self.latest_frame_time_ns = None

    def connect(self):
        """V4L2 컬러 카메라를 열고 요청 해상도를 설정한다."""
        if self.cap is not None and self.cap.isOpened():
            return True

        print(f"[*] Connecting to: /dev/video{self.device_index}")
        self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            print(
                f"[!] Error: Could not open video device "
                f"/dev/video{self.device_index}"
            )
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 컬러 카메라 노드에서는 Y16 포맷을 강제하지 않는다. 강제 설정하면
        # 컬러 영상이 깨질 수 있다.
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"Y16 "))

        print("[+] Hardware camera connected successfully.")
        if self.live_stream:
            cv2.namedWindow("QR Scanner Live Stream", cv2.WINDOW_NORMAL)
            print("[*] Live stream initialized. Press 'q' to stop.")
        return True

    def cropped_frame(self, frame, crop_scale):
        """영상 중앙을 crop_scale 비율로 자르고 원본 크기로 확대한다."""
        if crop_scale >= 1.0 or crop_scale <= 0:
            return frame

        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        crop_width = int(width * crop_scale)
        crop_height = int(height * crop_scale)

        y_start = max(0, center_y - crop_height // 2)
        y_end = min(height, center_y + crop_height // 2)
        x_start = max(0, center_x - crop_width // 2)
        x_end = min(width, center_x + crop_width // 2)

        cropped = frame[y_start:y_end, x_start:x_end]
        return cv2.resize(
            cropped,
            (width, height),
            interpolation=cv2.INTER_LINEAR,
        )

    def _decode_first_qr(self, frame):
        """프레임에서 첫 번째 QR의 데이터와 중심 거리를 추출한다."""
        if frame.dtype == "uint16":
            frame = cv2.normalize(
                frame,
                None,
                0,
                255,
                cv2.NORM_MINMAX,
                dtype=cv2.CV_8U,
            )

        height, width = frame.shape[:2]
        camera_center_x = width // 2
        camera_center_y = height // 2

        for obj in pyzbar.decode(frame):
            # 이번 실험의 대상은 일반 바코드가 아닌 QR 코드다.
            if obj.type != "QRCODE":
                continue

            data = obj.data.decode("utf-8", errors="replace")
            x, y, qr_width, qr_height = obj.rect
            qr_center_x = x + qr_width // 2
            qr_center_y = y + qr_height // 2
            distance = math.hypot(
                camera_center_x - qr_center_x,
                camera_center_y - qr_center_y,
            )
            return obj, data, distance

        return None

    def detect_qr(self, frame):
        """고정된 프레임에서 QR 위치 패턴 검출과 데이터 디코딩을 수행한다.

        성공 결과에는 전달받은 프레임 자체가 포함된다. 수신 스레드는 기존
        배열을 수정하지 않고 새 배열로 교체하므로 탐지 중인 프레임은 안전하다.
        이 메서드는 카메라를 직접 읽지 않아 저장 영상에도 사용할 수 있다.
        """
        visible, _points = self.qr_detector.detect(frame)
        decoded = self._decode_first_qr(frame)
        if decoded is None:
            return QRDetectionResult(visible=bool(visible), detected=False)

        obj, data, distance = decoded
        return QRDetectionResult(
            # 디코딩 성공은 QR이 영상에 존재한다는 더 강한 증거이므로,
            # OpenCV 위치 검출 결과와 관계없이 visible도 참으로 둔다.
            visible=True,
            detected=True,
            frame=frame,
            data=data,
            barcode_type=obj.type,
            distance_px=distance,
            rect=(obj.rect.left, obj.rect.top, obj.rect.width, obj.rect.height),
        )

    def process_frame(self, frame):
        """기존 실행 방식과 호환되도록 프레임의 모든 코드를 처리한다."""
        if frame.dtype == "uint16":
            frame = cv2.normalize(
                frame,
                None,
                0,
                255,
                cv2.NORM_MINMAX,
                dtype=cv2.CV_8U,
            )

        height, width = frame.shape[:2]
        camera_center_x = width // 2
        camera_center_y = height // 2

        for obj in pyzbar.decode(frame):
            data = obj.data.decode("utf-8", errors="replace")
            x, y, qr_width, qr_height = obj.rect
            qr_center_x = x + qr_width // 2
            qr_center_y = y + qr_height // 2

            if self.live_stream:
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + qr_width, y + qr_height),
                    (0, 255, 0),
                    2,
                )
                cv2.circle(frame, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)

            distance = math.hypot(
                camera_center_x - qr_center_x,
                camera_center_y - qr_center_y,
            )
            self.on_detect(obj.type, data, distance)

        if self.live_stream:
            cv2.drawMarker(
                frame,
                (camera_center_x, camera_center_y),
                (255, 0, 0),
                cv2.MARKER_CROSS,
                20,
                2,
            )

    def on_detect(self, barcode_type, data, distance):
        """기존 run() 방식에서 코드가 탐지되었을 때 로그를 출력한다."""
        timestamp = time.strftime("%H:%M:%S")
        print(
            f"[{timestamp}] Detect ({barcode_type}): {data} | "
            f"Center Distance: {distance:.2f}px"
        )

    def start_capture(self, warmup_sec=1.0):
        """카메라를 연결하고 최신 프레임 수신 스레드를 시작한다.

        실험 시작 시 한 번 호출한다. 이미 수신 중이면 아무 작업 없이 True를
        반환하며, 카메라 연결에 실패하면 False를 반환한다. 새 수신 스레드를
        시작한 뒤에는 카메라가 첫 프레임과 자동 노출/초점을 준비할 수 있도록
        ``warmup_sec`` 동안 기다린다. 이때 수신 스레드는 계속 프레임을 받는다.
        """
        if warmup_sec < 0:
            raise ValueError("warmup_sec must be 0 or greater")
        if self.capture_thread is not None and self.capture_thread.is_alive():
            return True
        if not self.connect():
            return False

        self.stop_event.clear()
        self.capture_error = None
        with self.frame_condition:
            self.latest_frame = None
            self.latest_frame_id = 0
            self.latest_frame_time_ns = None

        self.is_running = True
        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            name="realsense-frame-capture",
            daemon=True,
        )
        self.capture_thread.start()
        time.sleep(float(warmup_sec))
        return True

    def _capture_loop(self):
        """새 프레임을 계속 읽어 가장 최신 프레임 한 장만 보관한다."""
        try:
            while not self.stop_event.is_set():
                success, frame = self.cap.read()
                if not success:
                    if not self.stop_event.is_set():
                        self.capture_error = "Failed to grab frame from camera"
                    break

                processed_frame = self.cropped_frame(frame, self.crop_scale)
                captured_ns = time.monotonic_ns()

                with self.frame_condition:
                    self.latest_frame = processed_frame
                    self.latest_frame_id += 1
                    self.latest_frame_time_ns = captured_ns
                    self.frame_condition.notify_all()

                if self.live_stream:
                    cv2.imshow("QR Scanner Live Stream", processed_frame)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        self.stop_event.set()
                        break
        except Exception as exc:
            self.capture_error = str(exc)
        finally:
            self.is_running = False
            # 새 프레임을 기다리는 호출이 종료 상태를 확인하도록 깨운다.
            with self.frame_condition:
                self.frame_condition.notify_all()

    def get_latest_frame(self):
        """현재 보관 중인 최신 프레임의 독립적인 복사본을 반환한다."""
        with self.frame_condition:
            if self.latest_frame is None:
                return None
            return FrameSnapshot(
                frame=self.latest_frame.copy(),
                frame_id=self.latest_frame_id,
                captured_ns=self.latest_frame_time_ns,
            )

    def wait_for_new_frame(self, after_frame_id, deadline_ns):
        """지정한 번호보다 새로운 프레임을 절대 목표 시각까지만 기다린다.

        Args:
            after_frame_id: 이미 처리한 마지막 프레임 번호.
            deadline_ns: ``time.monotonic_ns()`` 기준 절대 마감 시각.
        """
        with self.frame_condition:
            while self.latest_frame_id <= after_frame_id:
                if self.stop_event.is_set() or not self.is_running:
                    return None

                remaining_ns = deadline_ns - time.monotonic_ns()
                if remaining_ns <= 0:
                    return None

                self.frame_condition.wait(timeout=remaining_ns / 1_000_000_000)

            return FrameSnapshot(
                # 수신 스레드는 이 배열을 수정하지 않고 latest_frame 참조만
                # 교체한다. 내부 탐지 경로에서는 불필요한 대용량 복사를 피한다.
                frame=self.latest_frame,
                frame_id=self.latest_frame_id,
                captured_ns=self.latest_frame_time_ns,
            )

    def detect_until(self, deadline_ns):
        """호출 이후 도착한 프레임을 다음 정렬 목표 시각까지 검사한다.

        디코딩에 성공하면 즉시 반환한다. 위치 패턴만 검출된 경우에는 그 결과를
        보존하면서 이후 프레임의 디코딩 성공을 계속 기다린다.
        """
        if self.capture_thread is None or not self.capture_thread.is_alive():
            return QRDetectionResult()

        # 호출 시점에 이미 있던 프레임은 정렬 이전 영상일 수 있으므로 제외한다.
        snapshot = self.get_latest_frame()
        last_snapshot = snapshot
        last_frame_id = snapshot.frame_id if snapshot is not None else 0
        visible_result = None

        while time.monotonic_ns() < deadline_ns:
            snapshot = self.wait_for_new_frame(last_frame_id, deadline_ns)
            if snapshot is None:
                break

            last_snapshot = snapshot
            last_frame_id = snapshot.frame_id
            result = self.detect_qr(snapshot.frame)
            if result.detected:
                return QRDetectionResult(
                    visible=True,
                    detected=True,
                    frame=result.frame,
                    data=result.data,
                    barcode_type=result.barcode_type,
                    distance_px=result.distance_px,
                    frame_id=snapshot.frame_id,
                    captured_ns=snapshot.captured_ns,
                    rect=result.rect,
                )
            if result.visible and visible_result is None:
                visible_result = QRDetectionResult(
                    visible=True,
                    detected=False,
                    frame=snapshot.frame,
                    frame_id=snapshot.frame_id,
                    captured_ns=snapshot.captured_ns,
                )

        if visible_result is not None:
            return visible_result

        if last_snapshot is None:
            return QRDetectionResult()

        return QRDetectionResult(
            visible=False,
            detected=False,
            frame=last_snapshot.frame,
            frame_id=last_snapshot.frame_id,
            captured_ns=last_snapshot.captured_ns,
        )

    def run(self):
        """기존 호환용 동기 스캐너 루프.

        실험 병합 코드에서는 이 메서드와 ``start_capture()``를 동시에 사용하지
        않는다. 병합 환경에서는 ``start_capture()``와 ``detect_until()``을 사용한다.
        """
        if self.capture_thread is not None and self.capture_thread.is_alive():
            raise RuntimeError("run() and start_capture() cannot be used together")
        if not self.cap or not self.cap.isOpened():
            if not self.connect():
                return

        self.is_running = True
        try:
            while self.is_running:
                success, frame = self.cap.read()
                if not success:
                    print("[!] Failed to grab frame from local hardware.")
                    break

                cropped = self.cropped_frame(frame, self.crop_scale)
                self.process_frame(cropped)

                if self.live_stream:
                    cv2.imshow("QR Scanner Live Stream", cropped)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        except KeyboardInterrupt:
            print("\n[+] Stopped by user.")
        finally:
            self.stop()

    def stop(self):
        """수신 스레드를 중단하고 카메라 및 OpenCV 자원을 해제한다."""
        self.is_running = False
        self.stop_event.set()
        with self.frame_condition:
            self.frame_condition.notify_all()

        # release()는 cap.read()가 카메라 프레임을 기다리는 경우 종료를 돕는다.
        if self.cap is not None:
            self.cap.release()

        current_thread = threading.current_thread()
        if (
            self.capture_thread is not None
            and self.capture_thread.is_alive()
            and self.capture_thread is not current_thread
        ):
            self.capture_thread.join(timeout=1.0)

        self.capture_thread = None
        self.cap = None
        cv2.destroyAllWindows()
        print("[*] Scanner resources released.")


if __name__ == "__main__":
    # 단독 카메라/QR 점검용 실행부다. rx_main 병합 환경에서는 run()을 쓰지 않는다.
    scanner = HardwareScanner(device_index=4, crop_scale=0.3, live_stream=True)
    scanner.run()
