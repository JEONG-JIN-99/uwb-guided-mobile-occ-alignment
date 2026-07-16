import cv2
from pyzbar import pyzbar
import time
import math

"""
    임시로 모니터에 qr을 띄워놓고 확인했을 때, 약 4m 정도에서 인식이 안됨.
    -> 논문에서는 test용으로 qr을 사용한 것이므로, 실제 occ는 빛을 이용함으로, 이러한 문제를 해결할 수 있을 것임을 언급할 수 있을 듯.
"""

class SmartPhoneScanner:
    def __init__(self, ip, port="8080"):
        self.stream_url = f"http://{ip}:{port}/video"
        self.cap = None
        self.last_data = None
        self.is_running = False

    def connect(self):
        """스트림 연결 시도"""
        print(f"[*] Connecting to: {self.stream_url}")
        self.cap = cv2.VideoCapture(self.stream_url)
        
        if not self.cap.isOpened():
            print("[!] Error: Could not open video stream.")
            return False
        
        print("[+] Connected successfully.")
        return True

    def process_frame(self, frame):
        """프레임 처리 및 시각화"""
        h, w, _ = frame.shape
        cam_center_x, cam_center_y = w // 2, h // 2

        # --- 시각화: 화면 정중앙 가이드 십자선 (녹색) ---
        cv2.line(frame, (cam_center_x - 20, cam_center_y), (cam_center_x + 20, cam_center_y), (0, 255, 0), 2)
        cv2.line(frame, (cam_center_x, cam_center_y - 20), (cam_center_x, cam_center_y + 20), (0, 255, 0), 2)

        decoded_objects = pyzbar.decode(frame)
        
        for obj in decoded_objects:
            current_data = obj.data.decode("utf-8")
            barcode_type = obj.type
            
            # QR 위치 정보
            (x, y, w_qr, h_qr) = obj.rect
            qr_center_x = x + (w_qr // 2)
            qr_center_y = y + (h_qr // 2)

            # 중앙과의 거리 계산
            distance = math.sqrt((cam_center_x - qr_center_x)**2 + (cam_center_y - qr_center_y)**2)

            # --- 시각화: QR 테두리 및 중심 연결선 ---
            # 1. QR 코드 박스 (파란색)
            cv2.rectangle(frame, (x, y), (x + w_qr, y + h_qr), (255, 0, 0), 2)
            
            # 2. QR 중심점 (보라색)
            cv2.circle(frame, (qr_center_x, qr_center_y), 5, (255, 0, 255), -1)
            
            # 3. 화면 중심과 QR 중심을 잇는 선 (하늘색)
            cv2.line(frame, (cam_center_x, cam_center_y), (qr_center_x, qr_center_y), (255, 255, 0), 2)

            # 4. 거리 정보 텍스트 표시 (흰색)
            cv2.putText(frame, f"Dist: {distance:.2f}px", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            self.on_detect(barcode_type, current_data, distance)

        # 결과 화면 출력
        cv2.imshow("QR Scanner Tracker", frame)

    def on_detect(self, b_type, data, distance):
        """탐지 시 콘솔 출력"""
        timestamp = time.strftime('%H:%M:%S')
        print(f"[{timestamp}] Detect ({b_type}): {data} | Dist: {distance:.2f}px")

    def run(self):
        """스캐너 루프 실행"""
        if not self.cap or not self.cap.isOpened():
            if not self.connect():
                return

        self.is_running = True
        print("[*] Press 'q' on the image window to quit.")
        
        try:
            while self.is_running:
                success, frame = self.cap.read()
                if not success:
                    print("[!] Failed to grab frame.")
                    break
                
                self.process_frame(frame)
                
                # 영상 출력을 위해 waitKey(1) 필수
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
        cv2.destroyAllWindows()
        print("[*] Scanner resources released.")

# --- 사용 예시 ---
if __name__ == "__main__":
    MY_PHONE_IP = "192.168.0.6" 
    scanner = SmartPhoneScanner(MY_PHONE_IP)
    scanner.run()