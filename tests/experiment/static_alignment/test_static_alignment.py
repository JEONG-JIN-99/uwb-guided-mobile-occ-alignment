import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.static_alignment_test import (
    build_parser,
    clamp_servo_command,
    estimate_tx_azimuth,
    normalize_angle,
    parse_uwb_packet,
    validate_args,
)


class StaticAlignmentTests(unittest.TestCase):
    def test_angle_calculations(self):
        self.assertEqual(normalize_angle(180), -180)
        self.assertEqual(normalize_angle(-181), 179)
        self.assertAlmostEqual(estimate_tx_azimuth(-20, -41.3), -61.3)

    def test_servo_clipping(self):
        self.assertEqual(
            clamp_servo_command(95, -90, 90),
            (90.0, True),
        )

    def test_parse_valid_uwb_packet(self):
        self.assertEqual(
            parse_uwb_packet(b"1,2.5,-41.3,0.7"),
            (2.5, -41.3, 0.7),
        )

    def test_reject_invalid_uwb_packet(self):
        self.assertIsNone(parse_uwb_packet(b"0,2.5,-41.3,0.7"))
        self.assertIsNone(parse_uwb_packet(b"1,2.5,180,0.7"))

    def test_default_pca9685_settings(self):
        parser = build_parser()
        args = parser.parse_args(["--distance", "2"])
        validate_args(parser, args)

        self.assertEqual(args.servo_channel, 0)
        self.assertEqual(args.pca9685_address, 0x40)


if __name__ == "__main__":
    unittest.main()
