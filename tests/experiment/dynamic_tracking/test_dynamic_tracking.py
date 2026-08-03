import queue
import sys
import time
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.dynamic_tracking.common import (
    ALIGNMENT_PERIOD_SEC,
    LatestUwbReceiver,
    UwbBiasCalibrator,
    UwbCalibrationError,
    UwbSample,
    calculate_circular_statistics,
    calculate_alignment,
    distance_trajectory,
    normalize_angle_deg,
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
    @staticmethod
    def make_uwb_sample(received_monotonic_ns=101):
        return UwbSample(
            distance_m=2.0,
            raw_azimuth_deg=10.0,
            elevation_deg=0.0,
            source="127.0.0.1:5005",
            received_monotonic_ns=received_monotonic_ns,
        )

    @staticmethod
    def make_receiver(reuse_latest):
        receiver = LatestUwbReceiver.__new__(LatestUwbReceiver)
        receiver._queue = queue.Queue(maxsize=1)
        receiver._reuse_latest = reuse_latest
        receiver._last_selection_sample = None
        return receiver

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
        self.assertEqual(result.uwb_corrected_azimuth_deg, 10)
        self.assertEqual(result.uwb_ros_azimuth_deg, -10)
        self.assertEqual(result.target_calculated_ros_deg, 10)
        self.assertEqual(result.gimbal_command_ros_deg, 10)
        self.assertFalse(result.servo_clipped)

    def test_alignment_applies_raw_bias_before_ros_conversion(self):
        result = calculate_alignment(20, 13, 3)
        self.assertEqual(result.uwb_corrected_azimuth_deg, 10)
        self.assertEqual(result.uwb_ros_azimuth_deg, -10)
        self.assertEqual(result.gimbal_command_ros_deg, 10)

    def test_alignment_applies_small_corrected_angle(self):
        result = calculate_alignment(20, 3.5, 3.0)
        self.assertEqual(result.uwb_corrected_azimuth_deg, 0.5)
        self.assertEqual(result.gimbal_command_ros_deg, 19.5)

    def test_angle_normalization_and_circular_mean_across_wrap(self):
        self.assertEqual(normalize_angle_deg(181), -179)
        self.assertEqual(normalize_angle_deg(-181), 179)
        mean_deg, std_deg = calculate_circular_statistics((179, -179))
        self.assertAlmostEqual(abs(mean_deg), 180.0)
        self.assertLess(std_deg, 2.0)

    def test_calibrator_collects_distinct_deliveries_and_returns_bias(self):
        calibrator = UwbBiasCalibrator()
        calibrator.begin(3)
        for azimuth in (2.0, 3.0, 4.0):
            sample = self.make_uwb_sample()
            sample = UwbSample(
                distance_m=sample.distance_m,
                raw_azimuth_deg=azimuth,
                elevation_deg=sample.elevation_deg,
                source=sample.source,
                received_monotonic_ns=sample.received_monotonic_ns,
            )
            calibrator.add_sample(sample)

        result = calibrator.wait_for_result(
            time.monotonic_ns() + 1_000_000_000,
            max_std_deg=5.0,
            max_abs_bias_deg=20.0,
        )
        self.assertEqual(result.sample_count, 3)
        self.assertAlmostEqual(result.bias_deg, 3.0)
        self.assertLess(result.circular_std_deg, 1.0)

    def test_calibrator_rejects_incomplete_or_unstable_collection(self):
        incomplete = UwbBiasCalibrator()
        incomplete.begin(2)
        incomplete.add_sample(self.make_uwb_sample())
        with self.assertRaisesRegex(UwbCalibrationError, "1/2"):
            incomplete.wait_for_result(
                time.monotonic_ns(),
                max_std_deg=5.0,
                max_abs_bias_deg=20.0,
            )

        unstable = UwbBiasCalibrator()
        unstable.begin(3)
        for azimuth in (-10.0, 0.0, 10.0):
            sample = self.make_uwb_sample()
            unstable.add_sample(
                UwbSample(
                    distance_m=sample.distance_m,
                    raw_azimuth_deg=azimuth,
                    elevation_deg=sample.elevation_deg,
                    source=sample.source,
                    received_monotonic_ns=sample.received_monotonic_ns,
                )
            )
        with self.assertRaisesRegex(UwbCalibrationError, "unstable"):
            unstable.wait_for_result(
                time.monotonic_ns() + 1_000_000_000,
                max_std_deg=5.0,
                max_abs_bias_deg=20.0,
            )

        excessive_bias = UwbBiasCalibrator()
        excessive_bias.begin(2)
        for _ in range(2):
            sample = self.make_uwb_sample()
            excessive_bias.add_sample(
                UwbSample(
                    distance_m=sample.distance_m,
                    raw_azimuth_deg=25.0,
                    elevation_deg=sample.elevation_deg,
                    source=sample.source,
                    received_monotonic_ns=sample.received_monotonic_ns,
                )
            )
        with self.assertRaisesRegex(UwbCalibrationError, "too large"):
            excessive_bias.wait_for_result(
                time.monotonic_ns() + 1_000_000_000,
                max_std_deg=5.0,
                max_abs_bias_deg=20.0,
            )

    def test_alignment_per_period_limit(self):
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

    def test_latest_uwb_receiver_reuses_last_selected_packet_when_enabled(self):
        receiver = self.make_receiver(reuse_latest=True)
        receiver._queue.put_nowait(self.make_uwb_sample())
        first = receiver.take_latest_after(100)
        reused = receiver.take_latest_after(100)

        self.assertIsNotNone(first)
        self.assertFalse(first.reused)
        self.assertIsNotNone(reused)
        self.assertTrue(reused.reused)
        self.assertEqual(reused.sample, first.sample)

    def test_latest_uwb_receiver_does_not_reuse_when_disabled(self):
        receiver = self.make_receiver(reuse_latest=False)
        receiver._queue.put_nowait(self.make_uwb_sample())
        first = receiver.take_latest_after(100)

        self.assertIsNotNone(first)
        self.assertFalse(first.reused)
        self.assertIsNone(receiver.take_latest_after(100))

    def test_packet_received_before_start_is_never_reused(self):
        receiver = self.make_receiver(reuse_latest=True)
        receiver._queue.put_nowait(
            self.make_uwb_sample(received_monotonic_ns=99)
        )
        self.assertIsNone(receiver.take_latest_after(100))
        self.assertIsNone(receiver.take_latest_after(100))

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
        self.assertTrue(args.uwb_calibration)
        self.assertEqual(args.uwb_calibration_samples, 100)
        self.assertEqual(args.crop_scale, 0.6)
        self.assertEqual(args.target_color, "red")
        self.assertEqual(args.color_min_area, 125.0)
        self.assertEqual(args.color_min_component_area, 50.0)
        self.assertIn("color_visible", RX_FIELDS)
        self.assertIn("camera_frame_id", RX_FIELDS)
        self.assertIn("failure_frame", RX_FIELDS)
        self.assertIn("uwb_packet_reused", RX_FIELDS)
        self.assertIn("uwb_calibration_bias_deg", RX_FIELDS)
        self.assertIn("uwb_corrected_azimuth_deg", RX_FIELDS)

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
        self.assertEqual(args.prestart_gimbal_stabilization_sec, 1.0)
        self.assertTrue(args.uwb_calibration)
        self.assertEqual(args.uwb_calibration_samples, 100)
        self.assertIn("previous_gimbal_ros_deg", TX_FIELDS)
        self.assertIn("target_calculated_ros_deg", TX_FIELDS)
        self.assertIn("uwb_packet_reused", TX_FIELDS)
        self.assertIn("uwb_calibration_bias_deg", TX_FIELDS)
        self.assertIn("uwb_corrected_azimuth_deg", TX_FIELDS)
        self.assertNotIn("color_visible", TX_FIELDS)


if __name__ == "__main__":
    unittest.main()
