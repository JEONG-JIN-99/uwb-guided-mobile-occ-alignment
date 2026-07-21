import math
import time
import RPi.GPIO as GPIO

# packet set
# header | distance | azimuth | elevation | latitude | langitude
#                      방위각     고도각        위도        경도

class GimbalController:
    def __init__(self, yaw_pin=18):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(yaw_pin, GPIO.OUT)
        
        self.yaw_pwm = GPIO.PWM(yaw_pin, 50)
        self.yaw_pwm.start(7.5)   # 90도 대기

        # 현재 짐벌의 마지막 명령각을 저장 (-90~90도, 초기값 0도)
        self.current_degree = 0.0
        # 서보 모터의 사양에 따른 동작 속도 (초당 회전 가능한 각도)
        # SG995 서보 기준: 0.2초당 60도 이동 가능(4.8V 기준)
        self.SERVO_SPEED_SEC_PER_DEG = 0.2 / 60.0
        self.ALIGN_INTERVAL_SEC = 180.0 * self.SERVO_SPEED_SEC_PER_DEG

        print("Gimbal Initialized")
        time.sleep(1.0)

    # non-heading
    # input: 내 위치(my_pos)와 타겟 위치(target_pos)의 위도, 경도 값
    # output: 라디안 값으로 방위각 반환 
    def calculate_gps_angles(self, my_pos, target_pos):
        """
        GPS 기반 방위각(Yaw) 계산
        pos 형식: (lat, lon)
        """
        my_lat, my_lon = map(math.radians, my_pos)
        # print(my_lat, my_lon)
        tar_lat, tar_lon = map(math.radians, target_pos)
        # print(tar_lat, tar_lon)

        # 1. Yaw (Bearing) 계산
        d_lon = tar_lon - my_lon

        y = math.sin(d_lon) * math.cos(tar_lat)
        x = math.cos(my_lat) * math.sin(tar_lat) - \
            math.sin(my_lat) * math.cos(tar_lat) * math.cos(d_lon)
        
        yaw = math.atan2(y, x)
        return yaw

    # 북쪽을 바라보고 있다는 제약 (아래의 코드 사용하려면, 기존 gimbal의 각도(current_heading)에 대해 알고 있어야함.)
    def get_rotation_angle(self, my_pos, target_pos, current_heading):
        """
        current_heading: 센서(나침반 등)로 측정한 현재 내 장비의 정면 방향 (0~360도)
        """
        # 1. 타겟의 절대 방위각 계산 (기존 코드 활용)
        target_bearing_rad = self.calculate_gps_angles(my_pos, target_pos)
        print("target_bearing_rad", target_bearing_rad)
        current_heading_rad = math.radians(current_heading)
        print("current_heading_rad", current_heading_rad)

        # 3. 상대 각도 계산 (목표 - 현재)
        relative_rad = target_bearing_rad - current_heading_rad
        print("relative_rad", relative_rad)

        # 4. -pi ~ pi 범위로 정규화 (가장 가까운 회전 방향 선택)
        while relative_rad > math.pi: relative_rad -= 2 * math.pi
        while relative_rad < -math.pi: relative_rad += 2 * math.pi
        print("relative_rad after while", relative_rad)

        return relative_rad

        # # 2. 실제 회전해야 할 상대 각도 계산
        # # (타겟 방향 - 내 현재 방향)
        # relative_angle = target_bearing - current_heading

        # # 3. 결과값을 -180 ~ 180 사이로 정규화 (가까운 쪽으로 돌기 위해)
        # if relative_angle > 180:
        #     relative_angle -= 360
        # elif relative_angle < -180:
        #     relative_angle += 360

        # return relative_angle  # 이 값만큼 모터를 돌려야 함


    def calculate_uwb_angles(self, my_pos, target_pos):
        """
        target_pos: [distance, azimuth, elevation]
        이미 상대 각도로 들어오는 경우
        """
        # target_pos[1]이 Azimuth(방위각)이므로 이를 바로 사용
        yaw = target_pos[1]
        
        # 만약 방위각 범위가 -180~180이라면 0~360으로 변환 (필요시)
        # yaw = (yaw_ + 360) % 360
        
        return yaw

    # input: 들어오는 방위각 (degree)
    # output: 실제로 짐벌에게 정렬 명령한 방위각 (degree)
    def move_to(self, az_degree):
        """
        상대 degree 값을 받아 서보 모터 이동
        relative_degree: -90도 ~ 90도 범위를 주동력으로 사용

        이 함수는 0.1초 주기 제어에서 호출이 밀리지 않도록 블로킹하지 않는다.
        self.current_degree는 실제 센서 피드백이 아니라 마지막으로 명령한 상대 짐벌 각도다.
        """
        # 디버깅 표시용 각도는 -90~90도 기준으로 사용
        input_az_degree = az_degree

        # 짐벌 명령각은 기준 7.5 PWM을 0도로 보는 -90~90도 범위로 제한한다.
        if input_az_degree < -90:
            gimbal_command_deg = -90
        elif input_az_degree > 90:
            gimbal_command_deg = 90
        else:
            gimbal_command_deg = input_az_degree

        current_az_degree = self.current_degree
        target_degree = gimbal_command_deg + 90.0

        # # [핵심 수정] 기어 반전 적용
        # # 실제 기계가 target_degree(예: 120도)로 가길 원한다면, 
        # # 반대로 도는 모터는 (180 - 120) = 60도 지점으로 명령을 내려야 합니다.
        # motor_target_degree = 180.0 - target_degree

        # 3. PWM 적용
        # 반대반향 회전 기어를 고려한 듀티 사이클 계산
        duty = (target_degree / 18.0) + 2.5
        self.yaw_pwm.ChangeDutyCycle(duty)

        # 4. 마지막 명령 위치를 목표 위치로 갱신
        self.current_degree = gimbal_command_deg
        
        return gimbal_command_deg

    def move_by_uwb_relative(self, uwb_relative_degree, wait=True):
        """
        UWB가 내는 현재 바라보는 방향 기준 상대각을 받아 짐벌 절대 명령각으로 변환해 이동한다.

        예: 마지막 짐벌 명령각이 10도이고 UWB 상대각이 20도이면 move_to(30)을 실행한다.
        반환값은 실제로 명령된 기준 7.5 PWM 상대각(gimbal_command_deg)이다.
        """
        next_command_deg = self.current_degree + uwb_relative_degree
        gimbal_command_deg = self.move_to(next_command_deg)

        if wait:
            time.sleep(self.ALIGN_INTERVAL_SEC)

        return gimbal_command_deg

    def disable_control_signal(self):
        """PWM 제어 펄스를 끄되 GPIO와 현재 명령각 상태는 유지한다.

        서보가 목표 위치에 도달한 뒤 발생하는 소프트웨어 PWM 지터를 비교
        시험할 때 사용한다. 제어 펄스를 끄면 서보의 유지 토크도 사라지므로
        외력이나 하중이 크면 실제 각도가 처질 수 있다.
        """
        self.yaw_pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.yaw_pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    gimbal = GimbalController()
    # my_pos = [35.134739, 129.102724]
    # # target_pos = [35.134505, 129.102517] # bench
    # target_pos = [35.135144, 129.102290] # park
    # current_heading = 0.0 # 북쪽 방향 기준
    # yaw = gimbal.calculate_gps_angles(my_pos, target_pos)
    # print(yaw)
    gimbal.move_to(-90)
    time.sleep(5)
    gimbal.move_to(90)
    time.sleep(5)
