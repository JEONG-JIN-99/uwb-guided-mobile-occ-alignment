import os
import sys
import time


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(CURRENT_DIR))
CODE_DIR = os.path.join(PROJECT_ROOT, "code")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

from gimbal.gimbal_controller_yaw import GimbalController


def main():
    gimbal = GimbalController(yaw_pin=18)

    try:
        gimbal.move_to(0.0)
        print("0도 정렬")
        time.sleep(1.0)
    finally:
        gimbal.cleanup()


if __name__ == "__main__":
    main()
