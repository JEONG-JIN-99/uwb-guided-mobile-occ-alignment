'''
작동전 확인사항
1. 현재 나의 위치가 하드코딩 위치를 넣어줘야 함
2. 이 코드는 code 디렉토리에서 python -m server.test2 또는 python main2.py 형태로 직접 실행할 수 있습니다.
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
PHONE_IP = "10.62.175.213"
PHONE_PORT = "8080"

# 짐벌 객체 생성
gimbal = GimbalController(yaw_pin=18)

# 소켓 설정
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# 백그라운드에서 최신 데이터를 저장할 글로벌 변수와 충돌 방지 락(Lock)
latest_data_info = None
data_lock = threading.Lock()

# CSV 로거 객체 생성 (자동으로 result 폴더 관리)
logger = ResultLogger(target_dir_name="result")

# QR 스캐너 객체 생성
qr_scanner = OneShotQRScanner(PHONE_IP, PHONE_PORT, crop_scale=0.3)

def load_start_angles():
    """
    main2.py가 어느 위치에서 실행되든 code/start_positions.txt를 기준으로 읽는다.
    줄 단위 숫자와 콤마로 나열된 숫자를 모두 허용한다.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_name = os.path.join(script_dir, "start_positions.txt")
    start_angles = []

    if os.path.exists(file_name):
        with open(file_name, "r") as f:
            for line in f:
                for val in line.replace(",", " ").split():
                    try:
                        start_angles.append(float(val))
                    except ValueError:
                        pass

    return file_name, start_angles

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

print("\n=== 짐벌 제어 서버(main2)가 준비되었습니다 ===")
print("명령 대기 중... (엔터키를 누르면 40회 반복 실험을 시작합니다. 'q'를 누르면 종료)")

try:
    while True:
        # 사용자 명령 대기 (Blocking)
        command = input("\n명령을 입력하세요 [엔터: 40회 실험 시작, q: 종료] : ")
        
        if command.lower() == 'q':
            break
            
        # 내가 만든 파일(start_positions.txt)에서 출발 위치(디그리) 읽어오기
        file_name, start_angles = load_start_angles()
        
        # 파일이 없거나 데이터가 비어있을 경우를 대비한 기본값(Fallback) 설정
        if not start_angles:
            print(f"경고: '{file_name}' 파일이 없거나 데이터를 읽을 수 없습니다. 기본 임의 각도로 진행합니다.")
            start_angles = [-60.0, -50.0, -40.0, -30.0, -20.0, 20.0, 30.0, 40.0, 50.0, 60.0]
            
        align_interval_sec = 180.0 * gimbal.SERVO_SPEED_SEC_PER_DEG

        print("\n--- 40회 반복 실험을 시작합니다 ---")
        
        # 2-1 과정을 40번 반복
        for i in range(40):
            # QR 결과 및 로깅 변수 초기화 (NameError 및 이전 루프 데이터 오염 방지)
            qr_detected = False
            qr_data = None
            qr_distance_px = None
            mode_name = ""
            tx_latency_ms = 0.0
            yaw_deg = 0.0
            gimbal_command_deg = 0.0
            last_processed_recv_time_ns = None

            # 파일에서 읽어온 임의의 출발 위치(디그리) 선택
            start_angle_deg = start_angles[i % len(start_angles)]
            
            print(f"\n[실험 {i+1}/40] 출발 위치({start_angle_deg}°)로 이동합니다.")
            
            # 출발 각도(degree)로 짐벌 이동
            gimbal.move_to(start_angle_deg)
            
            # 최대 180도 이동 시간을 고려해 다음 정렬 전까지 대기
            time.sleep(align_interval_sec)
            
            # 정렬 및 QR 탐색 시작 시점 기록
            command_time_ns = time.time_ns()
            print("[QR 탐색 시작] QR 코드가 인식될 때까지 실시간 조향 및 인식을 지속합니다.")

            # 💡 QR 코드가 성공적으로 검출될 때까지 계속 루프 구동
            while not qr_detected:
                with data_lock:
                    current_target = latest_data_info
                    
                if current_target is None:
                    print("아직 클라이언트로부터 수신된 데이터가 없습니다. 대기 중...")
                    time.sleep(align_interval_sec)
                    continue
                    
                # 데이터 분해
                data, gps_recv_time_ns = current_target
                if gps_recv_time_ns == last_processed_recv_time_ns:
                    time.sleep(align_interval_sec)
                    continue
                last_processed_recv_time_ns = gps_recv_time_ns

                try:
                    msg_str = data.decode('utf-8')
                    parts = msg_str.split(',')
                    header = parts[0]

                    if header == "1": # UWB 모드
                        mode_name = "uwb"
                        dist, az, el = map(float, parts[1:4])
                        yaw = az 
                        yaw_deg = yaw
                        
                        # UWB 상대각을 짐벌 컨트롤러에서 절대 명령각으로 변환해 이동한다.
                        gimbal_command_deg = gimbal.move_by_uwb_relative(yaw)
                        print(f"[UWB] Target Az: {yaw_deg}° | gimbal_command_deg: {gimbal_command_deg:.2f}°")

                    else: # GPS 모드
                        mode_name = "gps"
                        # 내 위치 하드 코딩 
                        my_pos = (35.134761, 129.102698) 

                        target_pos = [float(parts[4]), float(parts[5])] 
                        gps_read_time_ns = int(parts[6])

                        yaw = gimbal.calculate_gps_angles(my_pos, target_pos)
                        yaw_deg = math.degrees(yaw)
                        
                        # GPS는 현재 사용하지 않지만 move_to 계약에 맞춰 절대 짐벌 명령각으로 처리한다.
                        gimbal_command_deg = gimbal.move_to(yaw_deg)
                        print(f"[GPS] Target Yaw: {yaw_deg}° | gimbal_command_deg: {gimbal_command_deg:.2f}°")
                        time.sleep(align_interval_sec)

                        tx_latency_ms = (gps_recv_time_ns - gps_read_time_ns) / 1_000_000.0
                        print(f"tx_latency_ms: {tx_latency_ms:.2f} ms")

                    # QR 인식 시도 후 다음 정렬은 0.6초 주기로 수행한다.
                    qr_result = qr_scanner.scan_once(timeout_sec=0.1)
                    if qr_result is None:
                        print("[QR] 인식 실패 - 실시간 조향 및 검색 중...")
                    else:
                        qr_detected = True
                        qr_data = qr_result["data"]
                        qr_distance_px = qr_result["distance_px"]
                        print(f"[QR] 인식 성공! data={qr_data}, distance={qr_distance_px:.2f}px")

                except Exception as e:
                    print(f"[실시간 루프 에러] {e}")
                    time.sleep(align_interval_sec)

            # 정렬 및 인식 성공 완료 시간 계산
            align_recog_done_time_ns = time.time_ns()
            align_recog_time_ms = (align_recog_done_time_ns - command_time_ns) / 1_000_000.0
            print(f"정렬 및 인식 완료 소요 시간 (Total Time): {align_recog_time_ms:.2f} ms")

            # 로깅 모듈 사용 (로거에는 우리가 알아보기 쉬운 디그리 값을 그대로 기록합니다)
            try:
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
                print(f"로깅 처리 중 에러 발생: {e}")

            # 타겟 위치까지 정렬 및 인식 완료 후 다음 회차로 가기 전 1초 대기
            time.sleep(0.1)
        # -----------------------------------------------------------------
        # 40ls
        #10 회 반복이 끝난 후 본 위치인 90도로 정렬
        # -----------------------------------------------------------------
        print("\n[실험 종료] 40회 반복을 마쳤습니다. 짐벌을 본 위치(90도)로 복귀합니다.")
        
        # 💡 복귀 각도 라디안으로 변환 (90.0도)
        gimbal.move_to(0.0)
        
        print("명령을 입력하세요 [엔터: 다음 40회 실험 시작, q: 종료] : ", end="", flush=True)

except KeyboardInterrupt:
    print("\n강제 종료됨...")

finally:
    print("\nShutting down server...")
    gimbal.cleanup()
    sock.close()

    if 'qr_scanner' in locals():
        qr_scanner.stop()
