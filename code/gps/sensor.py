import serial
import pynmea2

"""
순서대로
GPGGA : 시간,위도,경도
"""


class GPS:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.latitude = 0.0
        self.longitude = 0.0
        self.timestamp = None
        self.is_valid = False

        # 시리얼 포트 설정 (timeout은 데이터가 없을 때 무한 대기를 방지합니다)
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port}")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.ser = None

    def update(self):
        """시리얼로부터 한 줄을 읽어 클래스 변수를 업데이트합니다."""
        if not self.ser:
            return

        line = self.ser.readline().decode('ascii', errors='replace')

        # NMEA 문장 중 위치 정보
        if line.startswith('$GNGGA'):
            try:
                msg = pynmea2.parse(line)
                # print(dir(msg))
                # print("data : ", msg.data)
                # print("longitude : ", msg.longitude)
                # print("latitude : ", msg.latitude)
                # print("valid : ", msg.is_valid)
                if msg.is_valid:
                    self.latitude = msg.latitude
                    self.longitude = msg.longitude
                    self.timestamp = msg.timestamp
                    self.is_valid = True
                else:
                    self.is_valid = False
            except pynmea2.ParseError:
                pass

    def get_location(self):
        """현재 저장된 위치 정보를 반환합니다."""
        if self.is_valid:
            return {
                "lat": self.latitude,
                "lon": self.longitude,
                "time": self.timestamp
            }
        else:
            return None

# --- 사용 예시 ---
if __name__ == "__main__":
    gps = GPS(port='/dev/ttyUSB0')

    print("GPS 데이터를 읽는 중... (Ctrl+C로 종료)")
    # test
    # gps.update()
    # location = gps.get_location()

    try:
        while True:
            gps.update()  # 데이터를 계속 갱신
            location = gps.get_location()

            if location:
                print(f"위도: {location['lat']:.6f}, 경도: {location['lon']:.6f}, 시간: {location['time']}")
            # else:
                # print("신호를 기다리는 중 (GPS 고정 안 됨)...")
                
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")