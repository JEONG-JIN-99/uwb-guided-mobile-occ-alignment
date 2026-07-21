# 한번 QR 인식 후 종료
import os
import time
import cv2

from qr.scanner import SmartPhoneScanner

class OneShotQRScanner(SmartPhoneScanner):
    def __init__(
        self,
        ip,
        port="8080",
        crop_scale=0.3,
        experiment_code="one_shot_scanner",
    ):
        super().__init__(ip, port, crop_scale)
        self.detected_result = None
        current_file = os.path.abspath(__file__)
        qr_dir = os.path.dirname(current_file)
        code_dir = os.path.dirname(qr_dir)
        project_root = os.path.dirname(code_dir)
        if experiment_code in ("", ".", "..") or os.path.basename(experiment_code) != experiment_code:
            raise ValueError("experiment_code must be a single directory name")
        self.frame_save_dir = os.path.join(
            project_root,
            "result",
            experiment_code,
            "detect_frame",
        )

        os.makedirs(self.frame_save_dir, exist_ok=True)

    def whole_save(self, time, frame):
        filename = f"whole_frame_{time}.jpg"
        full_path = os.path.join(self.frame_save_dir, filename)

        # cv2.imwrite(파일명, 저장할_프레임_데이터)
        # 여기서는 프레임 전체를 저장합니다.
        cv2.imwrite(full_path, frame)

    def cropped_save(self, time, cropped_frame):
        filename = f"cropped_frame_{time}.jpg"
        full_path = os.path.join(self.frame_save_dir, filename)

        # cv2.imwrite(파일명, 저장할_프레임_데이터)
        # 여기서는 크롭/확대된 현재 프레임 전체를 저장합니다.
        cv2.imwrite(full_path, cropped_frame)

    # QR 인식 시 실행하는 함수
    # input: 인식된 코드 타입, 인식된 데이터, 거리(카메라와 QR 중심 간의 픽셀 거리)
    # output: 없음 그냥 인식된 결과 self.detected_result 딕셔너리로 저장
    def on_detect(self, b_type, data, distance):
        self.detected_result = {
            "type": b_type,
            "data": data,
            "distance_px": distance,
        }

    # 한번 QR 인식 후 종료
    # input: 인식 시간 제한(초)
    # output: 인식된 결과 elf.detected_result 딕셔너리 반환
    def scan_once(self, timeout_sec=3.0):
        # 카메라 연결 확인
        if not self.cap or not self.cap.isOpened():
            if not self.connect():
                return None
        # 인식된 결과 초기화
        self.detected_result = None
        start_time = time.time()

        # 인식 시간 제한 동안 프레임 읽음음
        while time.time() - start_time < timeout_sec:
            success, frame = self.cap.read()
            if not success:
                continue

            # 프레임에서 qr 찾기
            # process_frame 함수에서 on_detect 함수 실행
            self.whole_save(start_time, frame)
            cropped_frame = self.cropped_frame(frame, self.crop_scale)

            self.cropped_save(start_time, cropped_frame)
            self.process_frame(cropped_frame)

            if self.detected_result is not None:
                return self.detected_result

        return None
