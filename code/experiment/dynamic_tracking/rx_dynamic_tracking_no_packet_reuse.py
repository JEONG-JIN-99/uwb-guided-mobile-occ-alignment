#!/usr/bin/env python3
"""UWB 패킷을 한 번만 사용하는 Rx 동적 추적 보존 버전."""

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.dynamic_tracking.rx_dynamic_tracking import main


if __name__ == "__main__":
    main(
        reuse_uwb_packets=False,
        experiment_code="dynamic_tracking_no_packet_reuse",
    )
