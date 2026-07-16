import socket
import time
from gps.sensor import GPS
from uwb.sensor import UWB

# packet set
# header | distance | azimuth | elevation | latitude | langitude
#                      방위각     고도각        위도        경도

# 짐벌 파이의 IP 주소
# cier
GIMBAL_IP = "192.168.0.31" 
# hotspot

# GIMBAL_IP = "10.185.103.85" 
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# gps = GPS(port='/dev/ttyUSB0')

while True:
    # 센서값 읽어온 시간
    gps_read_time_ns = time.time_ns()
    # 센서 update 속도에 맞춰서 해야함 (ms 정도의 속도)
    # GPS 모듈에서 읽어온 실제 좌표라고 가정
    # mode, dist, az, elev, lat, lng = GPS
    # gps.update()  # 데이터를 계속 갱신
    # location = gps.get_location()

    
            
    if True: #location:
        # print(f"위도: {location['lat']:.6f}, 경도: {location['lon']:.6f}, 시간: {location['time']}")
        target_pos = (35.135145, 129.103154) # 약 +45도
        #target_pos = (35.135014, 129.102441) # 약 -45도
        mode, dist, az, elev, lat, lng = 0,0,0,0,target_pos[0], target_pos[1] #location['lat'],location['lon']
        message = (
            f"{mode},{dist},{az},{elev},{lat},{lng},"
            f"{gps_read_time_ns}"
        )
        sock.sendto(message.encode(), (GIMBAL_IP, PORT))
        print(f"transmission message: {message}")
        time.sleep(0.1) # 10Hz 출력
    else:
        print("신호를 기다리는 중 (GPS 고정 안 됨)...")

    
    # mode, dist, az, elev, lat, lng = 0,2,3,4,120.5,127.0
    # message = f"{mode},{dist},{az},{elev},{lat},{lng}"
    # sock.sendto(message.encode(), (GIMBAL_IP, PORT))
    # time.sleep(0.1) # 10Hz 출력