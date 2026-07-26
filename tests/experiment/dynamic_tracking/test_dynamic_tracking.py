import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.dynamic_tracking.common import (
    ALIGNMENT_PERIOD_SEC,
    calculate_alignment,
    distance_trajectory,
    parse_uwb_packet,
)
from experiment.dynamic_tracking.rx_dynamic_tracking import (
    RX_FIELDS,
    build_parser as build_rx_parser,
    validate_args as validate_rx_args,
)
from experiment.dynamic_tracking.tx_dynamic_tracking import (
    TX_FIELDS,
    build_parser as build_tx_parser,
    validate_args as validate_tx_args,
)


class DynamicTrackingTests(unittest.TestCase):
    def test_parse_valid_uwb_packet(self):
        self.assertEqual(
            parse_uwb_packet(b"1,2.5,-41.3,0.7"),
            (2.5, -41.3, 0.7),
        )

    def test_reject_non_uwb_and_invalid_values(self):
        self.assertIsNone(parse_uwb_packet(b"0,2.5,-41.3,0.7"))
        self.assertIsNone(parse_uwb_packet(b"1,2.5,180,0.7"))
        self.assertIsNone(parse_uwb_packet(b"1,-1,10,0.7"))
        self.assertIsNone(parse_uwb_packet(b"1,nan,10,0.7"))

    def test_alignment_coordinate_conversion(self):
        result = calculate_alignment(20, 10)
        self.assertEqual(result.uwb_ros_azimuth_deg, -10)
        self.assertEqual(result.target_calculated_ros_deg, 10)
        self.assertEqual(result.gimbal_command_ros_deg, 10)
        self.assertFalse(result.servo_clipped)

    def test_alignment_deadband_and_per_period_limit(self):
        deadband = calculate_alignment(20, 0.5)
        self.assertEqual(deadband.gimbal_command_ros_deg, 20)

        limited = calculate_alignment(20, 100)
        self.assertEqual(limited.target_calculated_ros_deg, -80)
        self.assertEqual(limited.correction_ros_deg, -60)
        self.assertEqual(limited.gimbal_command_ros_deg, -40)

    def test_servo_clipping(self):
        result = calculate_alignment(-70, 60)
        self.assertEqual(result.command_requested_ros_deg, -130)
        self.assertEqual(result.gimbal_command_ros_deg, -90)
        self.assertTrue(result.servo_clipped)

    def test_distance_modes(self):
        self.assertEqual(distance_trajectory(0), "random_moving_rover")
        self.assertEqual(distance_trajectory(1), "fixed_radius_arc")
        self.assertEqual(distance_trajectory(3), "fixed_radius_arc")

    def test_rx_defaults_and_schema(self):
        parser = build_rx_parser()
        args = parser.parse_args(
            [
                "--distance",
                "2",
                "--experiment-id",
                "test",
                "--start-utc",
                "1785069000",
            ]
        )
        validate_rx_args(parser, args)
        self.assertEqual(ALIGNMENT_PERIOD_SEC, 0.2)
        self.assertEqual(args.camera_warmup, 5.0)
        self.assertEqual(args.crop_scale, 1.0)
        self.assertEqual(args.target_color, "red")
        self.assertIn("color_visible", RX_FIELDS)
        self.assertIn("camera_frame_id", RX_FIELDS)
        self.assertIn("failure_frame", RX_FIELDS)

    def test_tx_defaults_and_schema(self):
        parser = build_tx_parser()
        args = parser.parse_args(
            [
                "--distance",
                "0",
                "--experiment-id",
                "test",
                "--start-utc",
                "1785069000",
            ]
        )
        validate_tx_args(parser, args)
        self.assertEqual(args.yaw_pin, 18)
        self.assertIn("previous_gimbal_ros_deg", TX_FIELDS)
        self.assertIn("target_calculated_ros_deg", TX_FIELDS)
        self.assertNotIn("color_visible", TX_FIELDS)


if __name__ == "__main__":
    unittest.main()
