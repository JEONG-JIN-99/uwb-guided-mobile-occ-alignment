import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.static_alignment.dir_init import (
    build_parser as build_rx_parser,
    validate_args as validate_rx_args,
)
from experiment.static_alignment.tx_dir_init import (
    build_parser as build_tx_parser,
    validate_args as validate_tx_args,
)


class StaticDirectionInitializationTests(unittest.TestCase):
    def test_rx_defaults(self):
        parser = build_rx_parser()
        args = parser.parse_args([])
        validate_rx_args(parser, args)

        self.assertEqual(args.device_index, 4)
        self.assertEqual(args.servo_channel, 0)
        self.assertEqual(args.pca9685_address, 0x40)
        self.assertEqual(args.stabilization_time, 3.0)

    def test_tx_defaults(self):
        parser = build_tx_parser()
        args = parser.parse_args([])
        validate_tx_args(parser, args)

        self.assertEqual(args.yaw_pin, 18)
        self.assertEqual(args.stabilization_time, 3.0)


if __name__ == "__main__":
    unittest.main()
