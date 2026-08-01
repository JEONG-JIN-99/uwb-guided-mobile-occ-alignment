#!/usr/bin/env python3
"""정적 정렬 실험 전에 Rx PCA9685 짐벌과 카메라 중심을 맞춘다."""

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.static_alignment.dir_init import main as run_main


def main(argv=None):
    """기존 Rx 초기 정렬 구현을 명시적인 Rx 명령으로 실행한다."""
    return run_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
