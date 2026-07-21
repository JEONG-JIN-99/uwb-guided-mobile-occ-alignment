import os
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = os.path.join(PROJECT_ROOT, "code")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

from experiment.static_alignment.angle_utils import (
    calculate_final_alignment_error,
    clamp_servo_command,
    estimate_tx_azimuth,
    normalize_angle,
)
from experiment.static_alignment.config_loader import load_config
from experiment.static_alignment.result_store import ResultStore
from experiment.static_alignment.trial_plan import build_trial_plan, initial_angles_for_tx


CONFIG_PATH = os.path.join(CODE_DIR, "experiment", "static_alignment", "config.yaml")


class StaticAlignmentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(CONFIG_PATH)

    def test_angle_calculations(self):
        self.assertEqual(normalize_angle(180), -180)
        self.assertEqual(normalize_angle(-181), 179)
        estimated = estimate_tx_azimuth(-20, -41.3)
        self.assertAlmostEqual(estimated, -61.3)
        self.assertAlmostEqual(calculate_final_alignment_error(estimated, -60), -1.3)

    def test_servo_clipping(self):
        self.assertEqual(clamp_servo_command(95, -90, 90), (90.0, True))

    def test_initial_angles_and_repetitions(self):
        negative = initial_angles_for_tx(self.config, -60)
        positive = initial_angles_for_tx(self.config, 60)
        self.assertEqual(negative, list(range(-60, 1, 2)))
        self.assertEqual(positive, list(range(60, -1, -2)))
        plan = build_trial_plan(self.config, 2, -60)
        self.assertEqual(len(plan), 93)
        self.assertEqual(Counter(item.rx_initial_gimbal_deg for item in plan), Counter({float(angle): 3 for angle in negative}))
        self.assertEqual(plan, build_trial_plan(self.config, 2, -60))

    def test_all_blocks_have_558_trials(self):
        count = sum(
            len(build_trial_plan(self.config, distance, azimuth))
            for distance in self.config.experiment["distances_m"]
            for azimuth in self.config.experiment["tx_azimuths_deg"]
        )
        self.assertEqual(count, 558)

    def test_result_store_resumes_without_duplicates(self):
        plan = build_trial_plan(self.config, 1, 60)
        with tempfile.TemporaryDirectory() as temp_dir:
            store = ResultStore(temp_dir, self.config, plan)
            row = plan[0].as_row()
            row["trial_status"] = "uwb_timeout"
            store.append(row)
            resumed = ResultStore(temp_dir, self.config, plan)
            self.assertEqual(resumed.completed_trial_ids, {plan[0].trial_id})
            with self.assertRaises(ValueError):
                resumed.append(row)


if __name__ == "__main__":
    unittest.main()
