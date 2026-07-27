#!/usr/bin/env python3
"""짐벌 안정화 후 0.2초 동안 색상 인식을 측정하는 정적 정렬 실험."""

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.static_alignment.static_alignment_test import main as run_main


DEFAULT_PRE_RECOGNITION_SETTLE_TIME_SEC = 0.5
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "result" / "static_alignment_after_settle"


def main(argv=None):
    return run_main(
        argv,
        pre_recognition_settle_time_default=(
            DEFAULT_PRE_RECOGNITION_SETTLE_TIME_SEC
        ),
        output_dir_default=DEFAULT_OUTPUT_DIR,
    )


if __name__ == "__main__":
    raise SystemExit(main())
