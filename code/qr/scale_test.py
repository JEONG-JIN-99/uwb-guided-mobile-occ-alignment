import cv2
from pyzbar import pyzbar
import time
import math

class SmartPhoneScanner:
    def __init__(self, ip, port="8080", crop_scale=1.0):
        self.stream_url = f"http://{ip}:{port}/video"
        self.cap = None
        self.last_data = None
        self.is_running = False
        self.crop_scale = crop_scale

    def connect(self):
        """스트림 연결 시도"""
        print(f"[*] Connecting to: {self.stream_url}")
        self.cap = cv2.VideoCapture(self.stream_url)
        
        if not self.cap.isOpened():
            print("[!] Error: Could not open video stream.")
            return False
        
        print("[+] Connected successfully.")
        return True

    def cropped_frame(self, frame, crop_scale):
        """중앙을 기준으로 crop_scale 비율만큼 프레임을 크롭하고 원본 크기로 확대(디지털 줌)"""
        if crop_scale >= 1.0 or crop_scale <= 0:
            return frame

        h, w, _ = frame.shape
        cam_center_x, cam_center_y = w // 2, h // 2

        crop_w = int(w * crop_scale)
        crop_h = int(h * crop_scale)

        y_start = max(0, cam_center_y - (crop_h // 2))
        y_end = min(h, cam_center_y + (crop_h // 2))
        x_start = max(0, cam_center_x - (crop_w // 2))
        x_end = min(w, cam_center_x + (crop_w // 2))

        cropped = frame[y_start:y_end, x_start:x_end]
        
        # 슬라이싱된 이미지를 원본 해상도 크기로 확대하여 선명한 '줌 효과' 제공
        zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
        return zoomed

    def process_frame(self, frame):
        """프레임에서 바코드 디코딩 및 시각화 가이드라인 추가"""
        h, w, _ = frame.shape
        cam_center_x, cam_center_y = w // 2, h // 2

        decoded_objects = pyzbar.decode(frame)
    
        for obj in decoded_objects:
            current_data = obj.data.decode("utf-8")
            barcode_type = obj.type
            
            (x, y, w_qr, h_qr) = obj.rect
            qr_center_x = x + (w_qr // 2)
            qr_center_y = y + (h_qr // 2)
            distance = math.sqrt((cam_center_x - qr_center_x)**2 + (cam_center_y - qr_center_y)**2)

            self.on_detect(barcode_type, current_data, distance)

            # 🎨 시각화 레이어 추가: 검출된 QR 코드에 초록색 테두리 사각형 그리기
            cv2.rectangle(frame, (x, y), (x + w_qr, y + h_qr), (0, 255, 0), 2)
            
            # 🎨 시각화 레이어 추가: 화면 중심에서 QR 중심까지 파란색 선 연결 및 거리 텍스트 표시
            cv2.line(frame, (cam_center_x, cam_center_y), (qr_center_x, qr_center_y), (255, 0, 0), 2)
            cv2.putText(frame, f"{distance:.1f}px", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # 🎨 시각화 레이어 추가: 화면 정중앙에 빨간색 조준점 표시
        cv2.circle(frame, (cam_center_x, cam_center_y), 5, (0, 0, 255), -1)

    def on_detect(self, b_type, data, distance):
        """탐지 시 실행할 액션"""
        timestamp = time.strftime('%H:%M:%S')
        print(f"[{timestamp}] Detect ({b_type}): {data} | Center Distance: {distance:.2f}px")

    def run(self):
        """스캐너 루프 실행 및 윈도우 디스플레이"""
        if not self.cap or not self.cap.isOpened():
            if not self.connect():
                return

        self.is_running = True
        try:
            while self.is_running:
                success, frame = self.cap.read()
                if not success:
                    print("[!] Failed to grab frame.")
                    break
                
                # 1. 줌인(크롭 후 확대) 처리된 프레임 획득 (인스턴스 변수 self.crop_scale 사용)
                cropped_img = self.cropped_frame(frame, self.crop_scale)

                # 2. QR 스캔 진행 및 스캔된 프레임 위에 그래픽 가이드라인(사각형, 선) 그리기
                self.process_frame(cropped_img)
                
                # 3. 🖥️ 화면에 실시간으로 시각화 창 띄우기
                cv2.imshow("SmartPhone Scanner - Zoomed Window", cropped_img)
                
                # 원본 화면과 크롭 화면을 비교하고 싶다면 아래 주석을 해제하세요
                # cv2.imshow("Original WebCam Frame", frame)
                
                # 키 입력 대기 (창을 유지하고 'q' 누르면 종료되도록 제어)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            print("\n[+] Stopped by user.")
        finally:
            self.stop()

    def stop(self):
        """자원 해제 및 생성된 OpenCV 창 닫기"""
        self.is_running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows() # 🖥️ 프로그램 종료 시 띄워놓은 imshow 창을 모두 닫아줌
        print("[*] Scanner resources released.")

# --- 사용 예시 ---
if __name__ == "__main__":
    MY_PHONE_IP = "192.168.0.6" 
    
    # crop_scale=0.6 적용 시, 스마트폰 화면 중앙 60%만 크롭 후 
    # 원본 해상도로 늘려서(디지털 줌) 스캔 창을 보여줍니다.
    scanner = SmartPhoneScanner(MY_PHONE_IP, crop_scale=0.3)
    scanner.run()