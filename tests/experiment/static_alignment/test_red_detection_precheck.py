import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.static_alignment.red_detection_precheck import (
    RESULT_FIELDS,
    build_parser,
    build_run_directory_name,
    classify_result,
    empty_red_pixel_metrics,
    summarize_red_pixels,
    validate_args,
)


class RedDetectionPrecheckTests(unittest.TestCase):
    def test_defaults_match_static_alignment_color_conditions(self):
        parser = build_parser()
        args = parser.parse_args(["--distance", "3"])
        validate_args(parser, args)

        self.assertEqual(args.distance, 3.0)
        self.assertEqual(args.attempts, 100)
        self.assertEqual(args.device_index, 4)
        self.assertEqual(args.crop_scale, 0.6)
        self.assertEqual(args.interval, 0.2)
        self.assertEqual(args.camera_warmup, 5.0)
        self.assertEqual(args.color_min_area, 125.0)
        self.assertEqual(args.color_min_component_area, 50.0)
        self.assertEqual(
            Path(args.output_dir),
            PROJECT_ROOT / "result" / "red_detection_precheck",
        )
        self.assertIn("distance_m", RESULT_FIELDS)
        self.assertIn("crop_scale", RESULT_FIELDS)
        self.assertIn("red_saturation_min", RESULT_FIELDS)
        self.assertIn("red_value_min", RESULT_FIELDS)

    def test_run_directory_name_contains_distance_and_crop(self):
        self.assertEqual(
            build_run_directory_name("20260727_193000", 3, 0.6),
            "run_20260727_193000_distance_3m_crop_0.6",
        )

    def test_classifies_visible_new_frame_as_success(self):
        result = SimpleNamespace(
            frame_id=11,
            captured_ns=150,
            visible=True,
        )
        self.assertEqual(
            classify_result(result, 10, 100, 200),
            (True, "success"),
        )

    def test_classifies_new_frame_without_red_as_detection_failure(self):
        result = SimpleNamespace(
            frame_id=11,
            captured_ns=150,
            visible=False,
        )
        self.assertEqual(
            classify_result(result, 10, 100, 200),
            (False, "color_not_detected"),
        )

    def test_classifies_missing_new_frame_as_timeout(self):
        result = SimpleNamespace(
            frame_id=10,
            captured_ns=90,
            visible=False,
        )
        self.assertEqual(
            classify_result(result, 10, 100, 200),
            (False, "camera_frame_timeout"),
        )

    def test_calculates_red_pixel_count_and_color_statistics(self):
        frame = np.zeros((20, 30, 3), dtype=np.uint8)
        frame[:, :] = (0, 0, 255)
        hsv = np.zeros((20, 30, 3), dtype=np.uint8)
        hsv[:, :] = (0, 255, 255)
        selected = np.ones((20, 30), dtype=bool)

        metrics = summarize_red_pixels(frame, hsv, selected)

        self.assertEqual(metrics["red_pixel_count"], 600)
        self.assertEqual(metrics["red_pixel_ratio_pct"], 100.0)
        self.assertAlmostEqual(
            metrics["red_hue_circular_mean_deg"],
            0.0,
        )
        self.assertEqual(metrics["red_saturation_mean"], 255.0)
        self.assertEqual(metrics["red_value_mean"], 255.0)
        self.assertEqual(metrics["red_b_mean"], 0.0)
        self.assertEqual(metrics["red_g_mean"], 0.0)
        self.assertEqual(metrics["red_r_mean"], 255.0)

    def test_empty_frame_has_no_red_pixel_statistics(self):
        metrics = empty_red_pixel_metrics()

        self.assertEqual(metrics["red_pixel_count"], "")
        self.assertEqual(metrics["red_pixel_ratio_pct"], "")


if __name__ == "__main__":
    unittest.main()
