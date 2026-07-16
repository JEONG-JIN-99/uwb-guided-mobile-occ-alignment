import cv2
from pyzbar import pyzbar
import time
import math

# 스마트폰 IP Webcam 주소
stream_url = "http://10.62.175.213:8080/video"

def start_scanner():
    print(f"Connecting to: {stream_url}")
    cap = cv2.VideoCapture(stream_url)
    
    if not cap.isOpened():
        print("Check app & address")
        return

    # VNC에서 프레임 속도가 느려질 수 있으므로 해상도를 조절할 수도 있습니다.
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Streaming started... Press 'q' to quit.")

    try:
        while True:
            success, frame = cap.read()
            if not success:
                break
            
            # 1. 화면의 중심 계산
            h, w, _ = frame.shape
            cam_center_x, cam_center_y = w // 2, h // 2

            # 화면 중앙에 가이드 십자선 그리기
            cv2.line(frame, (cam_center_x - 20, cam_center_y), (cam_center_x + 20, cam_center_y), (0, 255, 0), 2)
            cv2.line(frame, (cam_center_x, cam_center_y - 20), (cam_center_x, cam_center_y + 20), (0, 255, 0), 2)

            # 2. 바코드/QR 디코딩
            decoded_objects = pyzbar.decode(frame)

            for obj in decoded_objects:
                # QR 코드의 경계 사각형 좌표
                (x, y, w_qr, h_qr) = obj.rect
                
                # QR 코드의 중심 계산
                qr_center_x = x + (w_qr // 2)
                qr_center_y = y + (h_qr // 2)

                # 3. 카메라 중심과 QR 중심 간의 거리 계산 (픽셀 단위)
                # Euclidean Distance: sqrt((x2-x1)^2 + (y2-y1)^2)
                distance = math.sqrt((cam_center_x - qr_center_x)**2 + (cam_center_y - qr_center_y)**2)

                # 시각화: QR 테두리
                cv2.rectangle(frame, (x, y), (x + w_qr, y + h_qr), (255, 0, 0), 2)
                
                # 시각화: QR 중심점
                cv2.circle(frame, (qr_center_x, qr_center_y), 5, (255, 0, 255), -1)

                # 시각화: 카메라 중심과 QR 중심을 잇는 선
                cv2.line(frame, (cam_center_x, cam_center_y), (qr_center_x, qr_center_y), (255, 255, 0), 2)

                # 거리 및 데이터 텍스트 표시
                qr_data = obj.data.decode("utf-8")
                cv2.putText(frame, f"Dist: {distance:.2f}px", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # 터미널 출력
                # print(f"QR Pos: ({qr_center_x}, {qr_center_y}), Dist: {distance:.2f}")

            # 4. VNC 화면 출력을 위한 창 띄우기
            cv2.imshow("QR Scanner & Tracker", frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nInterrupt received")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    start_scanner()