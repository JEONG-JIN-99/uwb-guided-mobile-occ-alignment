import time

from adafruit_servokit import ServoKit


# PCA9685 설정
PCA9685_ADDRESS = 0x40
SERVO_CHANNEL = 0
WAIT_SECONDS = 2.0

# 16채널 PCA9685 초기화
kit = ServoKit(
    channels=16,
    address=PCA9685_ADDRESS,
    frequency=50,
)

servo = kit.servo[SERVO_CHANNEL]
servo.set_pulse_width_range(500, 2500)


def move_to_relative_angle(relative_angle: int) -> None:
    """
    사용자 기준 각도 -90~+90도를
    ServoKit 기준 0~180도로 변환한다.

    사용자 -90도 -> ServoKit 0도
    사용자   0도 -> ServoKit 90도
    사용자 +90도 -> ServoKit 180도
    """
    if not -90 <= relative_angle <= 90:
        raise ValueError(
            f"각도는 -90~90도 범위여야 합니다: {relative_angle}"
        )

    servo_angle = relative_angle + 90

    print(
        f"[MOVE] 사용자 기준 {relative_angle:+d}도 "
        f"-> ServoKit {servo_angle}도"
    )

    servo.angle = servo_angle
    time.sleep(WAIT_SECONDS)


try:
    print("[START] PCA9685 MG996R 동작 테스트")

    # 중앙에서 시작
    move_to_relative_angle(0)

    # 왼쪽 끝으로 이동한 뒤 중앙 복귀
    move_to_relative_angle(-90)
    move_to_relative_angle(0)

    # 오른쪽 끝으로 이동한 뒤 중앙 복귀
    move_to_relative_angle(90)
    move_to_relative_angle(0)

    print("[COMPLETE] 테스트 완료")

except KeyboardInterrupt:
    print("\n[STOP] 사용자가 테스트를 중단했습니다.")

except Exception as error:
    print(f"[ERROR] {error}")

finally:
    # 서보 PWM 신호를 끈다.
    # 이후 외력에 의해 축이 움직일 수 있다.
    servo.angle = None
    print("[CLEANUP] 서보 PWM 출력을 비활성화했습니다.")
