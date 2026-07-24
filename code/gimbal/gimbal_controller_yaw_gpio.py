"""RPi.GPIO의 소프트웨어 PWM으로 yaw 서보를 직접 제어한다.

PCA9685를 사용하는 ``gimbal_controller_yaw.GimbalController``와 구분하기
위해 별도 모듈과 클래스 이름을 사용한다. 이 구현은 ``origin/main``의 GPIO
짐벌 컨트롤러 동작을 보존하며, 서보 드라이버가 없는 Tx 장치용이다.
"""

import math
import time

import RPi.GPIO as GPIO


class GPIOGimbalController:
    """BCM GPIO 핀에서 50Hz PWM을 생성하는 yaw 짐벌 컨트롤러."""

    def __init__(self, yaw_pin=18):
        self.yaw_pin = int(yaw_pin)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.yaw_pin, GPIO.OUT)

        self.yaw_pwm = GPIO.PWM(self.yaw_pin, 50)
        self.yaw_pwm.start(7.5)

        self.current_degree = 0.0
        self.SERVO_SPEED_SEC_PER_DEG = 0.2 / 60.0
        self.ALIGN_INTERVAL_SEC = 180.0 * self.SERVO_SPEED_SEC_PER_DEG

        print("GPIO Gimbal Initialized")
        time.sleep(1.0)

    def calculate_gps_angles(self, my_pos, target_pos):
        """두 GPS 좌표로부터 목표 compass bearing을 라디안으로 계산한다."""
        my_lat, my_lon = map(math.radians, my_pos)
        target_lat, target_lon = map(math.radians, target_pos)
        delta_lon = target_lon - my_lon

        y = math.sin(delta_lon) * math.cos(target_lat)
        x = (
            math.cos(my_lat) * math.sin(target_lat)
            - math.sin(my_lat)
            * math.cos(target_lat)
            * math.cos(delta_lon)
        )
        return math.atan2(y, x)

    def get_rotation_angle(self, my_pos, target_pos, current_heading):
        """현재 heading에서 GPS 목표까지의 최단 상대각을 반환한다."""
        target_bearing_rad = self.calculate_gps_angles(my_pos, target_pos)
        current_heading_rad = math.radians(current_heading)
        relative_rad = target_bearing_rad - current_heading_rad

        while relative_rad > math.pi:
            relative_rad -= 2 * math.pi
        while relative_rad < -math.pi:
            relative_rad += 2 * math.pi
        return relative_rad

    def calculate_uwb_angles(self, my_pos, target_pos):
        """UWB 데이터의 azimuth 값을 그대로 반환한다."""
        del my_pos
        return target_pos[1]

    def move_to(self, az_degree):
        """상대각을 -90~90도로 제한하고 해당 PWM duty를 적용한다."""
        gimbal_command_deg = max(-90.0, min(90.0, float(az_degree)))
        target_degree = gimbal_command_deg + 90.0
        duty = (target_degree / 18.0) + 2.5
        self.yaw_pwm.ChangeDutyCycle(duty)
        self.current_degree = gimbal_command_deg
        return gimbal_command_deg

    def move_by_uwb_relative(self, uwb_relative_degree, wait=True):
        """UWB 상대각을 이전 명령각에 더해 다음 절대 명령각으로 이동한다."""
        next_command_deg = self.current_degree + float(uwb_relative_degree)
        gimbal_command_deg = self.move_to(next_command_deg)

        if wait:
            time.sleep(self.ALIGN_INTERVAL_SEC)
        return gimbal_command_deg

    def disable_control_signal(self):
        """PWM duty를 0으로 만들어 제어 펄스를 비활성화한다."""
        self.yaw_pwm.ChangeDutyCycle(0)

    def cleanup(self):
        """PWM과 GPIO 자원을 정리한다."""
        self.yaw_pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    gimbal = GPIOGimbalController()
    try:
        gimbal.move_to(-90)
        time.sleep(5)
        gimbal.move_to(90)
        time.sleep(5)
    finally:
        gimbal.move_to(0)
        time.sleep(gimbal.ALIGN_INTERVAL_SEC)
        gimbal.cleanup()
