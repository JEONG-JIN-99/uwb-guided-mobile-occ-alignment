import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.static_alignment.analyze_alignment_error import (
    angular_difference_deg,
    percentile,
    summarize,
)


class AnalyzeAlignmentErrorTests(unittest.TestCase):
    def test_angular_difference_wraps_at_180_degrees(self):
        self.assertEqual(angular_difference_deg(10, 0), 10)
        self.assertEqual(angular_difference_deg(359, 0), -1)
        self.assertEqual(angular_difference_deg(-179, 179), 2)

    def test_summary_metrics(self):
        result = summarize([-2, 0, 4])
        self.assertEqual(result["count"], 3)
        self.assertAlmostEqual(result["mean_signed_error_deg"], 2 / 3)
        self.assertAlmostEqual(result["mean_absolute_error_deg"], 2)
        self.assertAlmostEqual(result["rmse_deg"], math.sqrt(20 / 3))
        self.assertAlmostEqual(result["median_absolute_error_deg"], 2)

    def test_percentile_uses_linear_interpolation(self):
        self.assertEqual(percentile([0, 10], 95), 9.5)


if __name__ == "__main__":
    unittest.main()
