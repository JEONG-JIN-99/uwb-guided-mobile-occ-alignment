'''
작동전 확인사항
1. 현재 나의 위치가 하드코딩 위치를 넣어줘야 함

'''
import socket
import time
import threading
import os
import math 

from gimbal.gimbal_controller_yaw import GimbalController


# --- 통신 설정 ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# 짐벌 객체 생성
gimbal = GimbalController(servo_channel=0, pca9685_address=0x40)

# 소켓 설정
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# GPS 객체 생성
# gps = GPS(port='/dev/ttyUSB0')
# gps.update()
# location = gps.get_location()
# my_pos = (location['lat'], location['lon'])

# 백그라운드에서 최신 데이터를 저장할 글로벌 변수와 충돌 방지 락(Lock)
latest_data_info = None
data_lock = threading.Lock()

def udp_receiver_thread():
    """
    백그라운드에서 계속 실행되며 최신 타겟 위치 데이터를 갱신하는 함수
    """
    global latest_data_info
    print(f"Background receiver started at {UDP_PORT}...")
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            recv_time_ns = time.time_ns()
            
            # Lock을 걸고 최신 데이터 갱신
            with data_lock:
                latest_data_info = (data, recv_time_ns)
        except Exception as e:
            print(f"[Thread Error] {e}")

# 스레드 실행 (데몬 스레드로 설정하여 메인 프로그램 종료 시 함께 종료되도록 함)
receiver_thread = threading.Thread(target=udp_receiver_thread, daemon=True)
receiver_thread.start()

try:
    with data_lock:
        current_target = latest_data_info
                
        # 데이터 분해
        data, gps_recv_time_ns = current_target
            

        try:
            msg_str = data.decode('utf-8')
            parts = msg_str.split(',')
            header = parts[0]

            mode_name = ""

            if header == "1": # UWB 모드
                mode_name = "uwb"
                dist, az, el = map(float, parts[1:4])
                yaw = az 
                yaw_deg = yaw

                # 타겟 각도(degree)로 짐벌 이동
                gimbal_command_deg = gimbal.move_to(yaw)
                print(f"[UWB] Target Az: {yaw_deg}° | gimbal_command_deg: {gimbal_command_deg:.2f}°")

            else: # GPS 모드
                mode_name = "gps"
                # 내 위치 하드 코딩 
                my_pos = (37.5, 127.0) 

                target_pos = [float(parts[4]), float(parts[5])] 
                gps_read_time_ns = int(parts[6])

                yaw = gimbal.calculate_gps_angles(my_pos, target_pos)
                yaw_deg = math.degrees(yaw)

                # GPS yaw는 라디안이므로 degree로 변환해 짐벌 이동
                gimbal_command_deg = gimbal.move_to(yaw_deg)
                print(f"[GPS] Target Yaw: {yaw_deg}° | gimbal_command_deg: {gimbal_command_deg:.2f}°")

                tx_latency_ms = (gps_recv_time_ns - gps_read_time_ns) / 1_000_000.0
                print(f"tx_latency_ms: {tx_latency_ms:.2f} ms")

                # QR 인식
                qr_result = qr_scanner.scan_once(timeout_sec=3.0)
                if qr_result is None:
                    print("[QR] 인식 실패")
                else:
                    qr_detected = True
                    qr_data = qr_result["data"]
                    qr_distance_px = qr_result["distance_px"]
                    print(f"[QR] data={qr_data}, distance={qr_distance_px:.2f}px")

                # 정렬 및 인식 끝난 시간 계산
                align_recog_done_time_ns = time.time_ns()
                align_recog_time_ms = (align_recog_done_time_ns - command_time_ns) / 1_000_000.0
                print(f"정렬 및 인식 소요 시간 (Alignment Time): {align_recog_time_ms:.2f} ms")

                # 로깅 모듈 사용 (로거에는 우리가 알아보기 쉬운 디그리 값을 그대로 기록합니다)
                if mode_name == "uwb":
                    logger.log_t_result("uwb", {
                        "gimbal_command_deg": gimbal_command_deg,
                    })
                    logger.log_a_result("uwb", {
                        "gimbal_command_deg": gimbal_command_deg,
                    })
                else:
                    logger.log_t_result("gps", {
                        "gimbal_command_deg": gimbal_command_deg,
                    })
                    logger.log_a_result("gps", {
                        "gimbal_command_deg": gimbal_command_deg,
                    })
                    
                print(f"[{mode_name.upper()}] {i+1}회차 결과 로깅 완료.")

        except Exception as e:
            print(f"Data processing error in iteration {i+1}: {e}")

            # 타겟 위치까지 정렬을 마친 뒤 5초간 대기
            time.sleep(5.0)

        # -----------------------------------------------------------------
        # 10회 반복이 끝난 후 본 위치인 90도로 정렬
        # -----------------------------------------------------------------
        print("\n[실험 종료] 10회 반복을 마쳤습니다. 짐벌을 본 위치(0도)로 복귀합니다.")
        
        # 💡 복귀 각도 라디안으로 변환 (90.0도)
        gimbal.move_to(0.0)
        
        print("명령을 입력하세요 [엔터: 다음 10회 실험 시작, q: 종료] : ", end="", flush=True)

except KeyboardInterrupt:
    print("\n강제 종료됨...")

finally:
    print("\nShutting down server...")
    gimbal.cleanup()
    sock.close()
