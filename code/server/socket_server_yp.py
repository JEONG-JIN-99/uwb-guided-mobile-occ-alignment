import socket
from gimbal_controller import GimbalController
import time
import gps.GPS
import uwb.UWB

# --- 설정 ---
MY_POS = (37.5, 127.0, 50)  # 내 RTK GPS 위치 (Lat, Lon, Alt)
UDP_IP = "0.0.0.0"          # 모든 인터페이스에서 수신
UDP_PORT = 5005             # 타겟 칩이 쏠 포트 번호

# 짐벌 객체 생성
gimbal = GimbalController(yaw_pin=12, pitch_pin=18)

# 소켓 설정
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Server started at {UDP_PORT}. Waiting for target GPS...")

try:
    while True:
        # 1. 데이터 수신 (Blocking - 데이터 올 때까지 대기)
        data, addr = sock.recvfrom(1024)
        
        try:
            # 수신 데이터 형식 가정: "위도,경도,고도" (예: "37.51,127.02,100")
            msg = data.decode('utf-8')
            target_pos = list(map(float, msg.split(',')))
            
            if len(target_pos) < 3:
                continue

            # 2. 각도 계산
            yaw, pitch = gimbal.calculate_angles(MY_POS, target_pos)

            # 3. 짐벌 구동
            gimbal.move_to(yaw, pitch)

            print(f"Target: {target_pos[0]:.5f}, {target_pos[1]:.5f} | "
                  f"Gimbal -> Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°")

        except Exception as e:
            print(f"Data conversion error: {e}")

except KeyboardInterrupt:
    print("\nShutting down server...")
finally:
    gimbal.cleanup()
    sock.close()
    print("Cleanup complete.")