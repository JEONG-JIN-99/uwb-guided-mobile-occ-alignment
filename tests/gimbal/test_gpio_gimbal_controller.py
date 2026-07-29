"""서보 드라이버 없이 RPi.GPIO PWM을 사용하는 Tx 짐벌 테스트."""

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, call, patch


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

gpio_module = MagicMock()
rpi_module = MagicMock()
rpi_module.GPIO = gpio_module
sys.modules["RPi"] = rpi_module
sys.modules["RPi.GPIO"] = gpio_module

from gimbal.gimbal_controller_yaw_gpio import GPIOGimbalController


class TestGPIOGimbalController(unittest.TestCase):
    def setUp(self):
        self.sleep_patcher = patch("time.sleep", return_value=None)
        self.sleep_patcher.start()
        self.pwm = MagicMock()
        gpio_module.PWM.return_value = self.pwm
        gpio_module.reset_mock()
        self.gimbal = GPIOGimbalController(yaw_pin=18)

    def tearDown(self):
        self.sleep_patcher.stop()

    def test_initializes_bcm_gpio_pwm(self):
        gpio_module.setmode.assert_called_once_with(gpio_module.BCM)
        gpio_module.setup.assert_called_once_with(18, gpio_module.OUT)
        gpio_module.PWM.assert_called_once_with(18, 50)
        self.pwm.start.assert_called_once_with(7.5)

    def test_move_to_clamps_and_converts_angle_to_duty(self):
        command = self.gimbal.move_to(120)

        self.assertEqual(command, 90.0)
        self.assertEqual(self.gimbal.current_degree, 90.0)
        self.pwm.ChangeDutyCycle.assert_called_with(2.5)

    def test_move_by_uwb_relative_converts_raw_angle_to_ros_yaw(self):
        self.gimbal.current_degree = 10.0

        command = self.gimbal.move_by_uwb_relative(20.0, wait=False)

        self.assertEqual(command, -10.0)
        self.pwm.ChangeDutyCycle.assert_called_with(8.055555555555555)

    def test_move_to_resumes_pwm_after_control_signal_is_disabled(self):
        self.gimbal.disable_control_signal()
        self.gimbal.move_to(0.0)

        self.pwm.ChangeDutyCycle.assert_has_calls([call(0), call(7.5)])

    def test_coordinate_conversions(self):
        self.assertEqual(self.gimbal.uwb_to_ros_yaw(30.0), -30.0)
        self.assertEqual(self.gimbal.uwb_to_ros_yaw(-30.0), 30.0)
        self.assertEqual(self.gimbal.ros_yaw_to_servo_angle(30.0), 60.0)
        self.assertEqual(self.gimbal.ros_yaw_to_servo_angle(-30.0), 120.0)

    def test_cleanup_stops_pwm_and_gpio(self):
        self.gimbal.cleanup()

        self.pwm.stop.assert_called_once_with()
        gpio_module.cleanup.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
