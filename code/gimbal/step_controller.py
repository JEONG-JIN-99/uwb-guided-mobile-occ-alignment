import math
import time
import sys
import os

# 상위 폴더(code/)를 sys.path에 추가하여 단독 실행 시 패키지 임포트 문제 해결
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from gimbal.gimbal_controller_yaw import GimbalController

class GimbalStepController(GimbalController):
    def __init__(self, yaw_pin=18):
        """
        GimbalController를 상속받아 초기 정렬을 정북방향 0도(물리각도 90도, duty 7.5)로 설정하는 컨트롤러
        """
        super().__init__(yaw_pin)
        # 초기 정렬: 정북방향 0도 (물리 각도 90도)
        self.current_degree = 0.0
        self.yaw_pwm.ChangeDutyCycle(7.5)

    def step_move_by_data(self, mode, **kwargs):
        """
        gps 혹은 uwb 데이터를 받아 회전 방향(시계/반시계)을 결정한 뒤,
        시계방향(타겟 각도 > 0)이면 duty를 0.1씩 증가시켜 12.5(상대 각도 90도)까지,
        반시계방향(타겟 각도 < 0)이면 duty를 0.1씩 감소시켜 2.5(상대 각도 -90도)까지 구동하는 함수.
        """
        direction = 0  # 1: 시계방향, -1: 반시계방향, 0: 유지

        if mode.lower() == 'gps':
            my_pos = kwargs.get('my_pos')
            target_pos = kwargs.get('target_pos')
            if my_pos and target_pos:
                # 0도(정북) 기준 상대 각도 구하기
                relative_rad = self.get_rotation_angle(my_pos, target_pos, 0.0)
                if relative_rad > 0:
                    direction = 1  # 시계방향
                elif relative_rad < 0:
                    direction = -1 # 반시계방향
        elif mode.lower() == 'uwb':
            azimuth = kwargs.get('azimuth')
            if azimuth is None and 'target_pos' in kwargs:
                target_pos = kwargs.get('target_pos')
                if len(target_pos) >= 2:
                    azimuth = target_pos[1]
            
            if azimuth is not None:
                if azimuth > 0:
                    direction = 1  # 시계방향
                elif azimuth < 0:
                    direction = -1 # 반시계방향

        # 현재 상대 각도(-90~90도)로부터 duty 계산
        current_duty = ((self.current_degree + 90.0) / 18.0) + 2.5

        if direction == 1:
            # 시계방향으로 12.5(상대각도 90도)까지 0.1씩 조정
            target_duty = 12.5
            while current_duty < target_duty - 1e-9:
                current_duty = round(min(current_duty + 0.1, target_duty), 10)
                self.yaw_pwm.ChangeDutyCycle(current_duty)
                time.sleep(0.02)
        elif direction == -1:
            # 반시계방향으로 2.5(상대각도 -90도)까지 0.1씩 조정
            target_duty = 2.5
            while current_duty > target_duty + 1e-9:
                current_duty = round(max(current_duty - 0.1, target_duty), 10)
                self.yaw_pwm.ChangeDutyCycle(current_duty)
                time.sleep(0.02)

        # 최종 상대 각도 갱신
        self.current_degree = ((current_duty - 2.5) * 18.0) - 90.0
        return self.current_degree

if __name__ == "__main__":
    print("=== GimbalStepController 단독 실행 데모 ===")
    
    # 짐벌 객체 생성 (GPIO 초기 정렬 0도/물리 90도 시작)
    controller = GimbalStepController(yaw_pin=18)
    
    try:
        # 1. 시계방향 테스트 (GPS 모사)
        # 내 위치 기준 동쪽에 타겟이 위치하는 상황 -> 시계방향 회전
        print("\n[테스트 1] 시계방향 회전 시작 (+90도 방향으로 duty 0.1씩 조절)")
        my_pos = (35.134761, 129.102698)
        target_pos = (35.135145, 129.103154) # 약 +45도
        #target_pos = (35.135014, 129.102441) # 약 -45도
        controller.step_move_by_data('gps', my_pos=my_pos, target_pos=target_pos)
        print(f"회전 완료 - 현재 상대 각도: {controller.current_degree:.2f}도")
        time.sleep(2.0)
        
        # 2. 반시계방향 테스트 (UWB 모사)
        # UWB 방위각 음수값 수신 -> 반시계방향 회전
        print("\n[테스트 2] 반시계방향 회전 시작 (-90도 방향으로 duty 0.1씩 조절)")
        controller.step_move_by_data('uwb', azimuth=-30.0)
        print(f"회전 완료 - 현재 상대 각도: {controller.current_degree:.2f}도")
        time.sleep(2.0)
        
        # 3. 홈 복귀
        print("\n[테스트 3] 정북 정중앙(0도)으로 복귀")
        controller.move_to(0.0)
        print(f"복귀 완료 - 현재 상대 각도: {controller.current_degree:.2f}도")
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        print("\n[!] 사용자에 의해 강제 종료되었습니다.")
    finally:
        print("\n자원 해제 및 종료...")
        controller.cleanup()
