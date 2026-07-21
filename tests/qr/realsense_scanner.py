"""Static QR tests에서 사용하는 production RealSense scanner export.

구현을 복사해 두면 실험 코드와 테스트 코드가 서로 달라질 수 있으므로,
실제 static-alignment 실험이 사용하는 클래스를 그대로 다시 내보낸다.
"""

from qr.realsense_scanner import FrameSnapshot, HardwareScanner, QRDetectionResult


__all__ = ["FrameSnapshot", "HardwareScanner", "QRDetectionResult"]
