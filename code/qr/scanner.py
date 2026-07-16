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

    # case 1. 배율로 크기 키운 다음에 crop (특정 배율 값을 지정할 수 있게끔 코드 작성해야함) v
    # case 2. 디텍팅 되고 나서 crop (디텍팅이 되지 않으면, 크롭이 안됨)
    def cropped_frame(self, frame, crop_scale):
        """중앙을 기준으로 crop_scale 비율만큼 프레임을 크롭"""
        # 배율이 1.0이거나 그 이상이면 원본 그대로 반환
        if crop_scale >= 1.0 or crop_scale <= 0:
            return frame

        h, w, _ = frame.shape
        cam_center_x, cam_center_y = w // 2, h // 2

        # 배율에 따른 크롭할 영역의 가로, 세로 크기 계산
        crop_w = int(w * crop_scale)
        crop_h = int(h * crop_scale)

        # 중심점 기준 시작점과 끝점 계산
        y_start = max(0, cam_center_y - (crop_h // 2))
        y_end = min(h, cam_center_y + (crop_h // 2))
        x_start = max(0, cam_center_x - (crop_w // 2))
        x_end = min(w, cam_center_x + (crop_w // 2))

        # 크롭 수행
        cropped = frame[y_start:y_end, x_start:x_end]
        zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)
        
        return zoomed

    def process_frame(self, frame):
        """프레임에서 바코드 디코딩 및 처리"""
        # 성능을 위해 흑백 변환 (필요시 사용)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        h, w, _ = frame.shape
        cam_center_x, cam_center_y = w // 2, h // 2

        decoded_objects = pyzbar.decode(frame)
    
        for obj in decoded_objects:
            current_data = obj.data.decode("utf-8")
            barcode_type = obj.type

            # 중복 데이터 출력 방지
            # if current_data != self.last_data:
            #     self.last_data = current_data
            #     self.on_detect(barcode_type, current_data)
            
            (x, y, w_qr, h_qr) = obj.rect
            qr_center_x = x + (w_qr // 2)
            qr_center_y = y + (h_qr // 2)
            
            distance = math.sqrt((cam_center_x - qr_center_x)**2 + (cam_center_y - qr_center_y)**2)

            # self.on_detect(barcode_type, current_data, distance)
            self.on_detect(barcode_type, current_data, distance)

    def on_detect(self, b_type, data, distance):
        """탐지 시 실행할 액션 (상속이나 콜백으로 확장 가능)"""
        timestamp = time.strftime('%H:%M:%S')
        print(f"[{timestamp}] Detect ({b_type}): {data} | Center Distance: {distance:.2f}px")
        # self.stop()

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
                    print("[!] Failed to grab frame.")
                    break
                
                cropped_frame = self.cropped_frame(frame, self.crop_scale)

                self.process_frame(cropped_frame)
                
                # 실시간성을 유지하기 위해 아주 짧은 대기 (버퍼 방지)
                # VNC 환경이 아니라면 waitKey가 효율적입니다.
                if cv2.waitKey(1) & 0xFF == ord('q'):
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
        print("[*] Scanner resources released.")

# --- 사용 예시 ---
if __name__ == "__main__":
    # 테더링 시 할당받은 스마트폰의 IP 입력
    MY_PHONE_IP = "192.168.0.6" 
    
    scanner = SmartPhoneScanner(MY_PHONE_IP, crop_scale=0.3)
    scanner.run()