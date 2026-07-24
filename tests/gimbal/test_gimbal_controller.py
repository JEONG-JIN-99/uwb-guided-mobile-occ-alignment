import sys
import os
from pathlib import Path
from unittest.mock import MagicMock, patch

# ServoKit 모듈 Mocking (PCA9685가 없는 개발 환경 호환성 확보)
servokit_module = MagicMock()
sys.modules["adafruit_servokit"] = servokit_module

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
from gimbal_uwb_tracking_qr_test import save_qr_failure_frame
from gimbal_uwb_tracking_test import (
    receive_latest_available_packet,
    receive_latest_packet,
)


class RecordingServo:
    def __init__(self):
        self.angle_history = []
        self.set_pulse_width_range = MagicMock()

    @property
    def angle(self):
        return self.angle_history[-1] if self.angle_history else None

    @angle.setter
    def angle(self, value):
        self.angle_history.append(value)


class TestReceiveLatestPacket(unittest.TestCase):
    def test_discards_backlog_and_returns_newest_packet(self):
        sock = MagicMock()
        sock.gettimeout.return_value = 0.2
        sock.recvfrom.side_effect = [
            (b"oldest", ("127.0.0.1", 5005)),
            (b"middle", ("127.0.0.1", 5005)),
            (b"newest", ("127.0.0.1", 5005)),
            BlockingIOError(),
        ]

        packet = receive_latest_packet(sock)

        self.assertEqual(packet, (b"newest", ("127.0.0.1", 5005)))
        sock.setblocking.assert_called_once_with(False)
        sock.settimeout.assert_called_once_with(0.2)

    def test_returns_none_when_no_packet_is_available(self):
        sock = MagicMock()
        sock.gettimeout.return_value = 0.2
        sock.recvfrom.side_effect = BlockingIOError()

        packet = receive_latest_available_packet(sock)

        self.assertIsNone(packet)
        sock.setblocking.assert_called_once_with(False)
        sock.settimeout.assert_called_once_with(0.2)


class TestSaveQrFailureFrame(unittest.TestCase):
    def test_saves_failed_frame_beside_qr_results(self):
        cv2_module = MagicMock()
        cv2_module.imwrite.return_value = True
        qr_result = MagicMock(detected=False, visible=True, frame="frame")

        with patch("pathlib.Path.mkdir") as mkdir:
            relative_path = save_qr_failure_frame(
                cv2_module,
                Path("/tmp/run"),
                7,
                qr_result,
            )

        self.assertEqual(
            relative_path,
            "failed_frames/attempt_000007_visible_not_decoded.jpg",
        )
        mkdir.assert_called_once_with(parents=True, exist_ok=True)
        cv2_module.imwrite.assert_called_once_with(
            "/tmp/run/failed_frames/attempt_000007_visible_not_decoded.jpg",
            "frame",
        )

    def test_does_not_save_successful_detection(self):
        cv2_module = MagicMock()
        qr_result = MagicMock(detected=True, visible=True, frame="frame")

        relative_path = save_qr_failure_frame(
            cv2_module,
            Path("/tmp/run"),
            1,
            qr_result,
        )

        self.assertEqual(relative_path, "")
        cv2_module.imwrite.assert_not_called()


class TestGimbalController(unittest.TestCase):
    def setUp(self):
        self.sleep_patcher = patch('time.sleep', return_value=None)
        self.mock_sleep = self.sleep_patcher.start()

        self.servo = RecordingServo()
        self.kit = MagicMock()
        self.kit.servo.__getitem__.return_value = self.servo
        servokit_module.ServoKit.return_value = self.kit
        servokit_module.ServoKit.reset_mock()
        self.gimbal = GimbalController(
            servo_channel=0,
            pca9685_address=0x40,
        )
        self.servo.angle_history.clear()
        self.mock_sleep.reset_mock()

    def tearDown(self):
        self.sleep_patcher.stop()

    def test_initializes_servokit_with_expected_configuration(self):
        servokit_module.ServoKit.assert_called_once_with(
            channels=16,
            address=0x40,
            frequency=50,
        )
        self.kit.servo.__getitem__.assert_called_once_with(0)
        self.servo.set_pulse_width_range.assert_called_once_with(500, 2500)

    def test_move_to_accepts_degrees_without_blocking_or_cutting_signal(self):
        gimbal_command_deg = self.gimbal.move_to(30.0)

        self.assertAlmostEqual(gimbal_command_deg, 30.0)
        self.assertAlmostEqual(self.gimbal.current_degree, 30.0)
        self.assertEqual(self.servo.angle_history, [120.0])
        self.mock_sleep.assert_not_called()

    def test_move_to_clamps_relative_degree_range(self):
        gimbal_command_deg = self.gimbal.move_to(120.0)

        self.assertAlmostEqual(gimbal_command_deg, 90.0)
        self.assertAlmostEqual(self.gimbal.current_degree, 90.0)
        self.assertEqual(self.servo.angle_history, [180.0])

    def test_move_by_uwb_relative_adds_to_last_command(self):
        self.gimbal.current_degree = 10.0

        gimbal_command_deg = self.gimbal.move_by_uwb_relative(20.0)

        self.assertAlmostEqual(gimbal_command_deg, 30.0)
        self.assertAlmostEqual(self.gimbal.current_degree, 30.0)
        self.assertEqual(self.servo.angle_history, [120.0])
        self.mock_sleep.assert_called_once_with(self.gimbal.ALIGN_INTERVAL_SEC)

    def test_disable_control_signal_releases_servo(self):
        self.gimbal.disable_control_signal()

        self.assertEqual(self.servo.angle_history, [None])

    def test_rejects_invalid_channel(self):
        with self.assertRaisesRegex(ValueError, "servo_channel"):
            GimbalController(servo_channel=16)

class TestGimbalStepController(unittest.TestCase):
    def setUp(self):
        # time.sleep을 모킹하여 테스트가 지연 없이 실행되도록 함
        self.sleep_patcher = patch('time.sleep', return_value=None)
        self.mock_sleep = self.sleep_patcher.start()
        
        self.servo = RecordingServo()
        self.kit = MagicMock()
        self.kit.servo.__getitem__.return_value = self.servo
        servokit_module.ServoKit.return_value = self.kit
        self.gimbal = GimbalStepController(
            servo_channel=0,
            pca9685_address=0x40,
        )
        self.servo.angle_history.clear()

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
        self.servo.angle_history.clear()
        
        final_degree = self.gimbal.step_move_by_data('gps', my_pos=my_pos, target_pos=target_pos)
        
        # 최종 상대 각도는 +90도여야 함
        self.assertAlmostEqual(final_degree, 90.0)
        
        calls = self.servo.angle_history
        self.assertTrue(len(calls) > 0)
        self.assertAlmostEqual(calls[0], 91.8)
        self.assertAlmostEqual(calls[-1], 180.0)
        for i in range(len(calls) - 1):
            self.assertAlmostEqual(calls[i+1] - calls[i], 1.8)

    def test_gps_mode_counter_clockwise(self):
        # 내 위치에서 서쪽(반시계방향 회전 필요)에 타겟이 있는 경우
        my_pos = (37.5, 127.0)
        target_pos = (37.5, 126.9)
        
        self.gimbal.current_degree = 0.0
        self.servo.angle_history.clear()
        
        final_degree = self.gimbal.step_move_by_data('gps', my_pos=my_pos, target_pos=target_pos)
        
        # 최종 상대 각도는 -90도여야 함
        self.assertAlmostEqual(final_degree, -90.0)
        
        calls = self.servo.angle_history
        self.assertTrue(len(calls) > 0)
        self.assertAlmostEqual(calls[0], 88.2)
        self.assertAlmostEqual(calls[-1], 0.0)
        for i in range(len(calls) - 1):
            self.assertAlmostEqual(calls[i+1] - calls[i], -1.8)

    def test_uwb_mode_clockwise(self):
        # UWB 방위각이 양수(시계방향)인 경우
        self.gimbal.current_degree = 0.0
        self.servo.angle_history.clear()
        
        final_degree = self.gimbal.step_move_by_data('uwb', azimuth=0.5)
        
        self.assertAlmostEqual(final_degree, 90.0)
        calls = self.servo.angle_history
        self.assertAlmostEqual(calls[0], 91.8)
        self.assertAlmostEqual(calls[-1], 180.0)

    def test_uwb_mode_counter_clockwise(self):
        # UWB 방위각이 음수(반시계방향)인 경우
        self.gimbal.current_degree = 0.0
        self.servo.angle_history.clear()
        
        final_degree = self.gimbal.step_move_by_data('uwb', azimuth=-0.5)
        
        self.assertAlmostEqual(final_degree, -90.0)
        calls = self.servo.angle_history
        self.assertAlmostEqual(calls[0], 88.2)
        self.assertAlmostEqual(calls[-1], 0.0)

if __name__ == '__main__':
    unittest.main()
