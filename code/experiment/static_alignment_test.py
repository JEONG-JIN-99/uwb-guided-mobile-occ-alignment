#!/usr/bin/env python3
"""이전 실행 경로를 유지하는 정적 정렬 실험 호환 진입점."""

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.insert(0, str(CODE_DIR))

from experiment.static_alignment.static_alignment_test import main


if __name__ == "__main__":
    main()
