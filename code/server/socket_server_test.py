'''
작동전 확인사항
1. 현재 나의 위치가 하드코딩 위치를 넣어줘야 함

'''
import socket
import time
import threading
import os
import math 

from gps.sensor import GPS
from uwb.sensor import UWB

from logger.result_logger import ResultLogger
from gimbal.gimbal_controller_yaw import GimbalController
from qr.scanner import SmartPhoneScanner
from qr.one_shot_scanner import OneShotQRScanner

# --- 통신 설정 ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# --- 스마트폰 설정 ---
PHONE_IP = "192.168.0.6"
PHONE_PORT = "8080"

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

# CSV 로거 객체 생성 (자동으로 result 폴더 관리)
logger = ResultLogger(
    target_dir_name="result",
    experiment_code="socket_server_test",
)

# QR 스캐너 객체 생성
qr_scanner = OneShotQRScanner(
    PHONE_IP,
    PHONE_PORT,
    experiment_code="socket_server_test",
)

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

print("\n=== 짐벌 제어 서버가 준비되었습니다 ===")
print("명령 대기 중... (엔터키를 누르면 10회 반복 실험을 시작합니다. 'q'를 누르면 종료)")

try:
    while True:
        # 사용자 명령 대기 (Blocking)
        command = input("\n명령을 입력하세요 [엔터: 10회 실험 시작, q: 종료] : ")
        
        if command.lower() == 'q':
            break
            
        # 내가 만든 파일(start_positions.txt)에서 출발 위치(디그리) 읽어오기
        start_angles = []
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_name = os.path.join(os.path.dirname(script_dir), "start_positions.txt")
        
        if os.path.exists(file_name):
            print(f"파일 읽기 성공: {file_name}")
            with open(file_name, "r") as f:
                for line in f:
                    val = line.strip()
                    if val:
                        try:
                            start_angles.append(float(val))
                        except ValueError:
                            pass
        
        # 파일이 없거나 데이터가 비어있을 경우를 대비한 기본값(Fallback) 설정
        if not start_angles:
            print(f"경고: '{file_name}' 파일이 없거나 데이터를 읽을 수 없습니다. 기본 임의 각도로 진행합니다.")
            start_angles = [-60.0, -50.0, -40.0, -30.0, -20.0, 20.0, 30.0, 40.0, 50.0, 60.0]
            
        print("\n--- 10회 반복 실험을 시작합니다 ---")
        
        # 2-1 과정을 10번 반복
        for i in range(10):
            # QR 결과 초기화 (이전 루프의 잔존 데이터 오염 및 NameError 방지)
            qr_detected = False
            qr_data = None
            qr_distance_px = None

            # 파일에서 읽어온 임의의 출발 위치(디그리) 선택
            start_angle_deg = start_angles[i % len(start_angles)]
            
            print(f"\n[실험 {i+1}/10] 출발 위치({start_angle_deg}°)로 이동합니다.")
            
            # 💡 출발 각도 디그리 -> 라디안 변환 후 짐벌 이동
            gimbal.move_to(start_angle_deg)
            
            # 임의의 출발 위치로 간 뒤 1초 대기
            time.sleep(1.0)
            
            # 1초 대기 후 가장 최신의 타겟 데이터를 락을 걸고 가져옴
            with data_lock:
                current_target = latest_data_info
                
            if current_target is None:
                print("아직 클라이언트로부터 수신된 데이터가 없습니다. 이번 회차는 건너뜁니다.")
                time.sleep(5.0) # 데이터가 없어도 실험 주기를 맞추기 위해 5초 대기
                continue
                
            # 데이터 분해
            data, gps_recv_time_ns = current_target
            
            # 정렬 명령을 내린 시점
            command_time_ns = time.time_ns()

            try:
                msg_str = data.decode('utf-8')
                parts = msg_str.split(',')
                header = parts[0]

                mode_name = ""
                tx_latency_ms = 0.0

                if header == "1": # UWB 모드
                    mode_name = "uwb"
                    dist, az, el = map(float, parts[1:4])
                    yaw = az 
                    yaw_deg = yaw
                    
                    # 💡 타겟 각도 디그리 -> 라디안 변환 후 짐벌 이동
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
                    
                    # 💡 타겟 각도 디그리 -> 라디안 변환 후 짐벌 이동
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
