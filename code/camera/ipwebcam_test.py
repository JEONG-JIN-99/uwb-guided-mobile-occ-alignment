import cv2
from pyzbar import pyzbar
import time

# 스마트폰 IP Webcam 주소 (USB 테더링 시 보통 192.168.42.129 등 확인 필요)
# /video 대신 /shot.jpg를 쓰면 프레임별로 가져와서 네트워크 부하를 줄일 수 있습니다.
stream_url = "http://192.168.0.6:8080/video"

def start_scanner():
    print(f"connecting : {stream_url}")
    cap = cv2.VideoCapture(stream_url)
    
    if not cap.isOpened():
        print("check app & addr")
        return

    print("streaming...")
    
    last_data = None # 중복 출력 방지용

    try:
        while True:
            success, frame = cap.read()
            if not success:
                print("non-frame")
                break
            
            # 성능 향상을 위해 흑백 변환
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 바코드 디코딩
            decoded_objects = pyzbar.decode(frame)

            for obj in decoded_objects:
                current_data = obj.data.decode("utf-8")
                barcode_type = obj.type
                
                # 새로운 데이터가 발견되었을 때만 출력 (터미널 도배 방지)
                if current_data != last_data:
                    print(f"[{time.strftime('%H:%M:%S')}] detect ({barcode_type}): {current_data}")
                    last_data = current_data

    except KeyboardInterrupt:
        print("\n interrupt")
    finally:
        cap.release()

if __name__ == "__main__":
    start_scanner()