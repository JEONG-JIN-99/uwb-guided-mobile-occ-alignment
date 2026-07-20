import cv2
from pyzbar import pyzbar
import time
import math

class HardwareScanner:
    def __init__(self, device_index=0, crop_scale=1.0, live_stream=False):
        """
        device_index: /dev/videoX 에서 X에 해당하는 정수값 (기본값 0)
        crop_scale: 프레임 크롭 비율
        live_stream: True일 경우 cv2.imshow()를 통해 VNC 화면에 영상을 출력합니다.
        """
        self.device_index = device_index
        self.cap = None
        self.last_data = None
        self.is_running = False
        self.crop_scale = crop_scale
        self.live_stream = live_stream  # 라이브 스트림 재생 여부 저장

    def connect(self):
        """로컬 비디오 장치 연결 시도"""
        print(f"[*] Connecting to: /dev/video{self.device_index}")
        
        # 리눅스 환경에서 V4L2 가속 백엔드를 사용하여 장치를 엽니다.
        self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            print(f"[!] Error: Could not open video device /dev/video{self.device_index}")
            return False
        
        # 해상도 지정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # [중요 변경] 2번 노드는 RGB 컬러 카메라 노드이므로 
        # 기존의 강제 'Y16 ' 포맷 설정을 비활성화합니다. (활성화 시 컬러 이미지가 깨짐)
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'Y16 '))

        print("[+] Hardware camera connected successfully.")
        
        # 라이브 스트림 모드가 켜져 있다면 미리 윈도우 생성
        if self.live_stream:
            cv2.namedWindow("QR Scanner Live Stream", cv2.WINDOW_NORMAL)
            print("[*] Live stream window initialized. Press 'q' on the image window to exit.")
            
        return True

    def cropped_frame(self, frame, crop_scale):
        """중앙을 기준으로 crop_scale 비율만큼 프레임을 크롭하고 원본 크기로 확대"""
        if crop_scale >= 1.0 or crop_scale <= 0:
            return frame

        h, w = frame.shape[:2]
        cam_center_x, cam_center_y = w // 2, h // 2

        crop_w = int(w * crop_scale)
        crop_h = int(h * crop_scale)

        y_start = max(0, cam_center_y - (crop_h // 2))
        y_end = min(h, cam_center_y + (crop_h // 2))
        x_start = max(0, cam_center_x - (crop_w // 2))
        x_end = min(w, cam_center_x + (crop_w // 2))

        cropped = frame[y_start:y_end, x_start:x_end]
        zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
        
        return zoomed

    def process_frame(self, frame):
        """프레임에서 바코드 디코딩 및 처리"""
        # (혹시 모를 예외 처리 유지) Depth 센서 포맷일 경우 8비트 변환
        if frame.dtype == 'uint16':
            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        
        if len(frame.shape) == 2:
            h, w = frame.shape
        else:
            h, w, _ = frame.shape
            
        cam_center_x, cam_center_y = w // 2, h // 2

        # QR 디코딩 진행
        decoded_objects = pyzbar.decode(frame)
    
        for obj in decoded_objects:
            current_data = obj.data.decode("utf-8")
            barcode_type = obj.type
            
            (x, y, w_qr, h_qr) = obj.rect
            qr_center_x = x + (w_qr // 2)
            qr_center_y = y + (h_qr // 2)
            
            # 사각형 그리기용 (라이브 화면 표시용 변수 확보)
            if self.live_stream:
                cv2.rectangle(frame, (x, y), (x + w_qr, y + h_qr), (0, 255, 0), 2)
                cv2.circle(frame, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)
            
            distance = math.sqrt((cam_center_x - qr_center_x)**2 + (cam_center_y - qr_center_y)**2)
            self.on_detect(barcode_type, current_data, distance)
            
        # 라이브 화면 모드일 때 화면 한가운데 조준점(Center Crosshair) 그리기
        if self.live_stream:
            cv2.drawMarker(frame, (cam_center_x, cam_center_y), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

    def on_detect(self, b_type, data, distance):
        """탐지 시 실행할 액션"""
        timestamp = time.strftime('%H:%M:%S')
        print(f"[{timestamp}] Detect ({b_type}): {data} | Center Distance: {distance:.2f}px")

    def run(self):
        """스캐너 루프 실행"""
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
                
                # 프레임 처리 (QR 코드 탐지 및 탐지 시 사각형 드로잉)
                self.process_frame(cropped)
                
                # 인자값이 True일 때만 VNC UI 화면을 출력합니다.
                if self.live_stream:
                    cv2.imshow("QR Scanner Live Stream", cropped)
                
                # 키 입력 대기 (q 버튼 누르면 종료)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[*] 'q' key pressed. Stopping...")
                    break
                    
        except KeyboardInterrupt:
            print("\n[+] Stopped by user.")
        finally:
            self.stop()

    def stop(self):
        """자원 해제"""
        self.is_running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("[*] Scanner resources released.")

# --- 메인 실행부 ---
if __name__ == "__main__":
    # 라이브 화면을 켜고 싶다면 live_stream=True 전달
    # 화면을 끄고 터미널 로그만 보고 싶다면 live_stream=False 로 설정
    scanner = HardwareScanner(device_index=4, crop_scale=1.0, live_stream=True)
    scanner.run()