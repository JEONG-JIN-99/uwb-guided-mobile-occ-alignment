import sys
import os
from unittest.mock import MagicMock, patch

# RPi.GPIO 모듈 Mocking (Windows 등 개발 환경 호환성 확보)
sys.modules['RPi'] = MagicMock()
sys.modules['RPi.GPIO'] = MagicMock()

import unittest
import math

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(CURRENT_DIR))
CODE_DIR = os.path.join(PROJECT_ROOT, "code")
if CURRENT_DIR not in sys.path:
    sys.path.insert(0, CURRENT_DIR)
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

from step_controller import GimbalStepController
from gimbal.gimbal_controller_yaw import GimbalController

class TestGimbalController(unittest.TestCase):
    def setUp(self):
        self.sleep_patcher = patch('time.sleep', return_value=None)
        self.mock_sleep = self.sleep_patcher.start()

        self.gimbal = GimbalController(yaw_pin=18)
        self.gimbal.yaw_pwm.ChangeDutyCycle = MagicMock()
        self.mock_sleep.reset_mock()

    def tearDown(self):
        self.sleep_patcher.stop()

    def test_move_to_accepts_degrees_without_blocking_or_cutting_pwm(self):
        gimbal_command_deg = self.gimbal.move_to(30.0)

        self.assertAlmostEqual(gimbal_command_deg, 30.0)
        self.assertAlmostEqual(self.gimbal.current_degree, 30.0)
        self.gimbal.yaw_pwm.ChangeDutyCycle.assert_called_once_with((120.0 / 18.0) + 2.5)
        self.mock_sleep.assert_not_called()

    def test_move_to_clamps_relative_degree_range(self):
        gimbal_command_deg = self.gimbal.move_to(120.0)

        self.assertAlmostEqual(gimbal_command_deg, 90.0)
        self.assertAlmostEqual(self.gimbal.current_degree, 90.0)
        self.gimbal.yaw_pwm.ChangeDutyCycle.assert_called_once_with(12.5)

    def test_move_by_uwb_relative_adds_to_last_command(self):
        self.gimbal.current_degree = 10.0

        gimbal_command_deg = self.gimbal.move_by_uwb_relative(20.0)

        self.assertAlmostEqual(gimbal_command_deg, 30.0)
        self.assertAlmostEqual(self.gimbal.current_degree, 30.0)
        self.gimbal.yaw_pwm.ChangeDutyCycle.assert_called_once_with((120.0 / 18.0) + 2.5)
        self.mock_sleep.assert_called_once_with(self.gimbal.ALIGN_INTERVAL_SEC)

class TestGimbalStepController(unittest.TestCase):
    def setUp(self):
        # time.sleep을 모킹하여 테스트가 지연 없이 실행되도록 함
        self.sleep_patcher = patch('time.sleep', return_value=None)
        self.mock_sleep = self.sleep_patcher.start()
        
        # GimbalStepController 객체 생성
        self.gimbal = GimbalStepController(yaw_pin=18)
        
        # ChangeDutyCycle 감시용 Mock 설정
        self.gimbal.yaw_pwm.ChangeDutyCycle = MagicMock()

    def tearDown(self):
        self.sleep_patcher.stop()

    def test_initialization(self):
        # 초기 정렬 상태 확인: 상대 각도 0도 (물리 각도 90도)
        self.assertEqual(self.gimbal.current_degree, 0.0)

    def test_gps_mode_clockwise(self):
        # 내 위치에서 동쪽(시계방향 회전 필요)에 타겟이 있는 경우
        my_pos = (37.5, 127.0)
        target_pos = (37.5, 127.1)
        
        self.gimbal.current_degree = 0.0
        self.gimbal.yaw_pwm.ChangeDutyCycle.reset_mock()
        
        final_degree = self.gimbal.step_move_by_data('gps', my_pos=my_pos, target_pos=target_pos)
        
        # 최종 상대 각도는 +90도여야 함
        self.assertAlmostEqual(final_degree, 90.0)
        
        # 호출된 duty cycle 값들이 0.1씩 증가했는지 확인
        calls = [call[0][0] for call in self.gimbal.yaw_pwm.ChangeDutyCycle.call_args_list]
        self.assertTrue(len(calls) > 0)
        # 첫 번째 조정 값은 7.6 부근이어야 함 (7.5 + 0.1)
        self.assertAlmostEqual(calls[0], 7.6)
        # 마지막 조정 값은 12.5여야 함
        self.assertAlmostEqual(calls[-1], 12.5)
        # 모든 간격이 0.1인지 확인
        for i in range(len(calls) - 1):
            self.assertAlmostEqual(calls[i+1] - calls[i], 0.1)

    def test_gps_mode_counter_clockwise(self):
        # 내 위치에서 서쪽(반시계방향 회전 필요)에 타겟이 있는 경우
        my_pos = (37.5, 127.0)
        target_pos = (37.5, 126.9)
        
        self.gimbal.current_degree = 0.0
        self.gimbal.yaw_pwm.ChangeDutyCycle.reset_mock()
        
        final_degree = self.gimbal.step_move_by_data('gps', my_pos=my_pos, target_pos=target_pos)
        
        # 최종 상대 각도는 -90도여야 함
        self.assertAlmostEqual(final_degree, -90.0)
        
        calls = [call[0][0] for call in self.gimbal.yaw_pwm.ChangeDutyCycle.call_args_list]
        self.assertTrue(len(calls) > 0)
        # 첫 번째 조정 값은 7.4 부근이어야 함 (7.5 - 0.1)
        self.assertAlmostEqual(calls[0], 7.4)
        # 마지막 조정 값은 2.5여야 함
        self.assertAlmostEqual(calls[-1], 2.5)
        # 모든 간격이 -0.1인지 확인
        for i in range(len(calls) - 1):
            self.assertAlmostEqual(calls[i+1] - calls[i], -0.1)

    def test_uwb_mode_clockwise(self):
        # UWB 방위각이 양수(시계방향)인 경우
        self.gimbal.current_degree = 0.0
        self.gimbal.yaw_pwm.ChangeDutyCycle.reset_mock()
        
        final_degree = self.gimbal.step_move_by_data('uwb', azimuth=0.5)
        
        self.assertAlmostEqual(final_degree, 90.0)
        calls = [call[0][0] for call in self.gimbal.yaw_pwm.ChangeDutyCycle.call_args_list]
        self.assertAlmostEqual(calls[0], 7.6)
        self.assertAlmostEqual(calls[-1], 12.5)

    def test_uwb_mode_counter_clockwise(self):
        # UWB 방위각이 음수(반시계방향)인 경우
        self.gimbal.current_degree = 0.0
        self.gimbal.yaw_pwm.ChangeDutyCycle.reset_mock()
        
        final_degree = self.gimbal.step_move_by_data('uwb', azimuth=-0.5)
        
        self.assertAlmostEqual(final_degree, -90.0)
        calls = [call[0][0] for call in self.gimbal.yaw_pwm.ChangeDutyCycle.call_args_list]
        self.assertAlmostEqual(calls[0], 7.4)
        self.assertAlmostEqual(calls[-1], 2.5)

if __name__ == '__main__':
    unittest.main()
