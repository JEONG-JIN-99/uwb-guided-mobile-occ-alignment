import cv2
import time
import math
from picamera2 import Picamera2

class HardwareScanner:
    def __init__(self, crop_scale=1.0, live_stream=False):
        self.picam2 = None
        self.last_data = None
        self.is_running = False
        self.crop_scale = crop_scale
        self.live_stream = live_stream
        
        self.qr_detector = cv2.QRCodeDetector()

    def connect(self):
        """Picamera2 자원 초기화 및 시작"""
        print("[*] Connecting to Raspberry Pi Camera via Picamera2...")
        try:
            self.picam2 = Picamera2()
            
            # RGB 대신 BGR888로 초기화하여 OpenCV와의 색상 변환 연산(cv2.cvtColor) 완전 제거!
            config = self.picam2.create_preview_configuration(
                main={"format": "BGR888", "size": (1280, 720)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            
            print("[+] Picamera2 hardware connected and started successfully.")
            
            if self.live_stream:
                cv2.namedWindow("QR Scanner Live Stream", 0)
                print("[*] Live stream window initialized. Press 'q' on the window to exit.")
                
            return True
        except Exception as e:
            print(f"[!] Error starting Picamera2: {e}")
            return False

    def cropped_frame(self, frame, crop_scale):
        if crop_scale >= 1.0 or crop_scale <= 0:
            return frame, 0, 0

        h, w = frame.shape[:2]
        cam_center_x, cam_center_y = w // 2, h // 2

        crop_w = int(w * crop_scale)
        crop_h = int(h * crop_scale)

        y_start = max(0, cam_center_y - (crop_h // 2))
        y_end = min(h, cam_center_y + (crop_h // 2))
        x_start = max(0, cam_center_x - (crop_w // 2))
        x_end = min(w, cam_center_x + (crop_w // 2))

        cropped = frame[y_start:y_end, x_start:x_end]
        return cropped, x_start, y_start

    def process_frame(self, frame, offset_x=0, offset_y=0, orig_w=1280, orig_h=720):
        cam_center_x, cam_center_y = orig_w // 2, orig_h // 2

        data, points, _ = self.qr_detector.detectAndDecode(frame)

        if points is not None and data:
            pts = points[0]

            qr_local_center_x = int(sum(p[0] for p in pts) / 4)
            qr_local_center_y = int(sum(p[1] for p in pts) / 4)

            qr_center_x = offset_x + qr_local_center_x
            qr_center_y = offset_y + qr_local_center_y

            if self.live_stream:
                pts_int = pts.astype(int)
                cv2.polylines(frame, [pts_int], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.circle(frame, (qr_local_center_x, qr_local_center_y), 5, (0, 0, 255), -1)

            distance = math.sqrt((cam_center_x - qr_center_x)**2 + (cam_center_y - qr_center_y)**2)
            self.on_detect("QRCODE", data, distance)

        if self.live_stream:
            ch, cw = frame.shape[:2]
            cv2.drawMarker(frame, (cw // 2, ch // 2), (255, 0, 0), 0, 20, 2)

    def on_detect(self, b_type, data, distance):
        timestamp = time.strftime('%H:%M:%S')
        print(f"[{timestamp}] Detect ({b_type}): {data} | Center Distance: {distance:.2f}px")

    def run(self):
        if not self.picam2:
            if not self.connect():
                return

        self.is_running = True
        try:
            while self.is_running:
                # BGR 포맷으로 프레임 수신
                frame = self.picam2.capture_array()
                if frame is None:
                    print("[!] Failed to grab frame from Picamera2.")
                    break
                
                orig_h, orig_w = frame.shape[:2]
                
                cropped, offset_x, offset_y = self.cropped_frame(frame, self.crop_scale)
                
                self.process_frame(cropped, offset_x, offset_y, orig_w=orig_w, orig_h=orig_h)
                
                # 색상 변환 없이(cv2.cvtColor 제거) 바로 출력!
                if self.live_stream:
                    display_frame = cv2.resize(cropped, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
                    cv2.imshow("QR Scanner Live Stream", display_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[*] 'q' key pressed. Stopping...")
                    break
                    
        except KeyboardInterrupt:
            print("\n[+] Stopped by user.")
        finally:
            self.stop()

    def stop(self):
        self.is_running = False
        if self.picam2:
            try:
                self.picam2.stop()
                print("[*] Picamera2 stopped.")
            except:
                pass
        cv2.destroyAllWindows()
        print("[*] Scanner resources released.")

if __name__ == "__main__":
    scanner = HardwareScanner(crop_scale=1.0, live_stream=True)
    scanner.run()