# OCC 통신을 위한 정렬 시스템 (Alignment System for OCC)
# jj 브랜치가 현재 최신 버전 반드시 jj 브랜치를 볼 것 
Optical Camera Communication(OCC) 통신을 위한 GPS UWB 기반의 짐벌 정렬 및 제어 시스템입니다. 클라이언트-서버 구조를 통해 센서 데이터를 주고받으며, QR 코드 인식 및 데이터 로깅 기능을 포함하고 있습니다.

---

## 📂 프로젝트 구조 및 주요 기능 (Directory Structure)

### 🛰️ Connection & Core (클라이언트 / 서버)
* **`server/socket_server_test.py`**
  * 백그라운드에서 상시로 정보를 수신합니다. (GPS만 UWB는 구현 X)
  * 10개의 값을 읽어와서 초기 헤딩으로 설정 정렬 명령을 10번 반복하며 실험 데이터를 기록합니다.
  * 실험이 끝나면 짐벌을 0도로 초기화합니다.
* **`server/socket_server.py`**
  * 백그라운드에서 상시로 정보를 수신합니다. (GPS만 UWB는 구현 X)
  * 정렬 명령 하달 시, 수신된 정보를 기반으로 짐벌을 정렬하고 실험 데이터를 기록합니다.
  * 작동 30초 후에 짐벌을 0도로 초기화합니다.
* **`client/socket_client.py`**
  * GPS 모듈로부터 Raw 데이터를 받아 서버로 실시간 전송합니다.(GPS만 UWB는 구현 X)

### 🎮 Control (짐벌 제어)
* **`gimbal/gimbal_controller_yaw.py`**
  * 수신된 GPS 위치 데이터를 기반으로 짐벌의 Yaw 각도 및 서보 모터 제어를 위한 Duty Cycle을 계산합니다.
  * *(현재 UWB 기반 제어는 구현되어 있지 않습니다.)*

### 📷 Vision (QR 코드 프로세싱)
* **`qr/scanner.py`**
  * 카메라를 통해 QR 코드를 인식하고 스캔하는 소스코드입니다.
* **`qr/dist.py`**
  * 인식된 QR 코드의 중앙점과 카메라 화면 중심부 사이의 이격 거리를 계산합니다.

### 🔌 Sensors (센서 데이터 파싱)
* **`gps/sensor.py`**
  * GPS 모듈에서 들어오는 Raw 데이터를 파싱하여 유효한 위치 정보로 변환합니다.
* **`uwb/sensor.py`**
  * UWB(Ultra-Wideband) Raw 데이터를 파싱하는 코드입니다. *(현재 미구현)*

### 📊 Logging (데이터 기록)
* **`logger/result_logger.py`**
  * 실험 중 발생하는 주요 데이터 및 원하는 측정값을 프로젝트 루트 디렉토리 하위의 `result/` 폴더 내에 CSV 파일로 기록합니다.

---

## ⚠️ 참고 사항 (Status Notes)
* **미구현 기능:** `client/socket_client.py`, `uwb/sensor.py` 및 `gimbal/gimbal_controller_yaw.py` 내의 UWB 관련 로직은 현재 구현되지 않은 상태입니다.

---

## 코드 전체 분석 및 프로젝트 동작 설명

이 프로젝트는 OCC(Optical Camera Communication) 수신 장치가 송신 장치 방향을 바라보도록 짐벌을 정렬하는 실험용 시스템입니다. 전체 흐름은 `클라이언트 센서 데이터 송신 -> 서버 UDP 수신 -> GPS/UWB 기반 방위각 계산 -> 짐벌 서보 제어 -> QR 인식으로 정렬 결과 확인 -> CSV 저장` 구조입니다.

### 1. 전체 실행 흐름

1. 클라이언트 장치가 UDP로 타겟 위치 데이터를 전송합니다.
   - 메시지 형식은 대체로 `mode,dist,az,elev,lat,lng,timestamp_ns`입니다.
   - `mode == 1`이면 UWB 모드로 보고 `az` 값을 사용합니다.
   - 그 외에는 GPS 모드로 보고 `lat,lng`를 타겟 좌표로 사용합니다.
2. 서버는 `0.0.0.0:5005`에서 UDP 패킷을 계속 수신합니다.
   - 백그라운드 스레드가 가장 최근 패킷만 `latest_data_info`에 저장합니다.
   - `threading.Lock`으로 메인 루프와 수신 스레드의 데이터 충돌을 막습니다.
3. 사용자가 엔터를 누르면 실험 루프가 시작됩니다.
   - `start_positions.txt`에서 시작 각도 목록을 읽습니다.
   - 짐벌을 시작 각도로 이동시킨 뒤, 최신 타겟 데이터로 목표 방위각을 계산합니다.
4. 짐벌은 계산된 yaw 각도로 이동합니다.
   - GPS는 내 위치와 타겟 위치의 bearing을 계산합니다.
   - UWB는 수신된 azimuth 값을 yaw로 사용합니다.
5. 스마트폰 IP Webcam 영상에서 QR 코드를 인식합니다.
   - QR 데이터와 화면 중심에서 QR 중심까지의 픽셀 거리를 계산합니다.
   - 인식 프레임은 `result/detect_frame/`에 저장됩니다.
6. 결과는 `result/` 아래 CSV 파일로 저장됩니다.
   - `gps_t_result.csv`, `gps_a_result.csv`
   - `uwb_t_result.csv`, `uwb_a_result.csv`

### 2. 주요 실행 파일

#### `code/main.py`

현재 프로젝트의 통합 실험 실행 스크립트입니다.

- UDP 서버를 열고 `5005` 포트에서 클라이언트 데이터를 수신합니다.
- `GimbalController`로 yaw 서보 모터를 제어합니다.
- `OneShotQRScanner`로 QR을 한 번씩 탐색합니다.
- `code/start_positions.txt`의 시작 각도를 순서대로 사용해 22회 반복 실험을 수행합니다.
- GPS 모드에서는 현재 내 위치가 `(35.134761, 129.102698)`로 하드코딩되어 있습니다.
- 실험마다 다음 값을 기록합니다.
  - 시작 각도
  - 목표 yaw/azimuth
  - 실제 이동량
  - 정렬 및 QR 인식 소요 시간
  - GPS 송수신 지연 시간
  - QR 인식 여부, QR 데이터, 중심 거리
- 실험 종료 후 짐벌을 0도 상대 방향으로 복귀시키고 GPIO와 카메라 자원을 정리합니다.

#### `code/main2.py`

`main.py`와 구조는 거의 같지만 QR이 인식될 때까지 실시간 조향을 반복하는 버전입니다.

- 40회 반복 실험을 수행합니다.
- 각 회차에서 시작 각도로 이동한 뒤, QR이 검출될 때까지 최신 UDP 데이터를 계속 읽고 짐벌을 재조향합니다.
- `scan_once(timeout_sec=0.5)`처럼 짧은 QR 탐색을 반복하여 조향 주기를 유지합니다.
- QR 인식 성공 시 해당 회차의 정렬 시간을 기록합니다.
- 스마트폰 IP는 `10.62.175.213`, crop 비율은 `0.3`으로 설정되어 있습니다.

### 3. 통신 코드

#### `code/client/socket_client.py`

UDP 클라이언트 예제입니다.

- 짐벌 서버 IP인 `GIMBAL_IP`와 `PORT=5005`로 데이터를 보냅니다.
- 현재 GPS 모듈 실제 읽기는 주석 처리되어 있고, 타겟 좌표가 하드코딩되어 있습니다.
- 0.1초마다 GPS 모드 메시지를 전송합니다.
- 메시지에는 `time.time_ns()`로 찍은 센서 읽기 시각이 포함되어 서버에서 전송 지연 계산에 사용됩니다.

#### `code/client/test.py`

UWB 모드 UDP 송신 테스트 코드입니다.

- 로컬 서버 `127.0.0.1:5005`로 모의 UWB 데이터를 보냅니다.
- `mode=1`, 거리, azimuth, elevation, timestamp를 10Hz로 전송합니다.

#### `code/server/socket_server.py`

수동 정렬용 UDP 서버입니다.

- 백그라운드 수신 스레드가 최신 UDP 패킷을 저장합니다.
- 사용자가 엔터를 누르면 최신 패킷 기준으로 한 번 정렬합니다.
- 정렬 시간과 GPS 전송 지연을 CSV로 기록합니다.
- 정렬 후 `threading.Timer`로 일정 시간 뒤 홈 위치 복귀를 시도합니다.
- 현재 코드에는 `return_home()`에서 `gimbal.move_to(7.5)`처럼 라디안 입력을 기대하는 함수에 duty처럼 보이는 값을 넣는 부분이 있어 점검이 필요합니다.

#### `code/server/socket_server_test.py`

10회 반복 실험용 서버입니다.

- `main.py`의 이전 형태에 가까운 파일입니다.
- UDP 최신 패킷 수신, 시작 각도 이동, GPS/UWB 정렬, QR 인식, CSV 로깅을 수행합니다.
- `code/start_positions.txt`를 읽어 시작 각도로 사용합니다.
- GPS 기준 내 위치가 `(37.5, 127.0)`으로 하드코딩되어 있습니다.

#### `code/server/socket_server_yp.py`

초기 프로토타입 서버로 보입니다.

- `"위도,경도,고도"` 형식의 단순 UDP 데이터를 수신한다고 가정합니다.
- yaw/pitch 2축 짐벌 컨트롤러를 사용하는 형태지만, 현재 프로젝트의 `gimbal.gimbal_controller_yaw.GimbalController` 구조와 맞지 않는 import가 포함되어 있습니다.
- 현재 코드 기준으로 바로 실행하기보다는 참고용/구버전 코드로 보는 것이 적절합니다.

#### `code/server/socket_motor_test.py`

모터 단독 수신 테스트를 만들던 중간 코드로 보입니다.

- UDP 수신 후 GPS/UWB 모드에 따라 짐벌 이동, QR 인식, 로깅까지 넣으려는 구조입니다.
- 현재 들여쓰기 오류, 정의되지 않은 변수(`qr_scanner`, `logger`, `command_time_ns`, `start_angle_deg`, `i` 등)가 있어 그대로 실행하기 어렵습니다.
- 정리 또는 삭제 후보입니다.

#### `code/client/mqtt_client.py`, `code/server/mqtt_server.py`

MQTT 기반 UWB 데이터 송수신 예제입니다.

- `mqtt_client.py`는 `uwb/data` 토픽으로 JSON 데이터를 발행합니다.
- `mqtt_server.py`는 `uwb/data` 토픽을 구독하고 azimuth/elevation/위치 값을 출력합니다.
- 실제 통합 실험 흐름은 UDP 중심이며, MQTT 코드는 별도 실험 또는 대체 통신 방식 예제에 가깝습니다.

### 4. 짐벌 제어 코드

#### `code/gimbal/gimbal_controller_yaw.py`

Yaw 1축 서보 모터 제어의 핵심 클래스입니다.

- `RPi.GPIO`를 사용해 라즈베리파이 BCM 핀 기준 PWM을 생성합니다.
- 기본 yaw 핀은 GPIO 18, PWM 주파수는 50Hz입니다.
- `calculate_gps_angles(my_pos, target_pos)`
  - 내 GPS 좌표와 타겟 GPS 좌표를 받아 bearing을 계산합니다.
  - 반환값은 라디안입니다.
- `get_rotation_angle(my_pos, target_pos, current_heading)`
  - 현재 장비 heading을 알고 있을 때 목표까지의 상대 회전각을 계산합니다.
  - `-pi ~ pi` 범위로 정규화합니다.
- `calculate_uwb_angles(my_pos, target_pos)`
  - UWB 타겟 데이터에서 azimuth 값을 그대로 yaw로 사용하는 단순 함수입니다.
- `move_to(az_degree)`
  - `-90~90도` 기준의 짐벌 명령각을 입력받아 서보 물리각 `0~180도`로 매핑합니다.
  - 명령각 0도는 서보 90도, -90도는 서보 0도, +90도는 서보 180도로 해석합니다.
  - duty cycle은 `(target_degree / 18.0) + 2.5` 공식으로 계산합니다.
  - 반환값은 실제로 짐벌에 명령한 `gimbal_command_deg`입니다.
- `move_by_uwb_relative(uwb_relative_degree, wait=True)`
  - UWB가 준 현재 방향 기준 상대각을 이전 짐벌 명령각에 더해 다음 짐벌 명령각으로 변환합니다.
  - 기본값에서는 명령 후 약 0.6초 대기합니다.
- `cleanup()`
  - PWM 정지와 GPIO cleanup을 수행합니다.

#### `tests/gimbal/step_controller.py`

`GimbalController`를 상속한 단계 이동 실험용/레거시 컨트롤러입니다.

- 목표 각도로 바로 이동하지 않고, 방향만 판단해서 duty를 0.1씩 증가/감소시킵니다.
- GPS 모드에서는 `get_rotation_angle()`로 상대 방향을 판단합니다.
- UWB 모드에서는 azimuth 부호로 회전 방향을 판단합니다.
- 최종적으로 시계방향이면 duty 12.5, 반시계방향이면 duty 2.5까지 이동합니다.

#### `tests/gimbal/dir_init.py`

서보를 정북/중앙 위치로 초기화하는 간단한 하드웨어 테스트 코드입니다.

- GPIO 18에 PWM 50Hz를 설정합니다.
- duty 7.5를 1초 동안 주고 duty를 0으로 낮춥니다.
- MG995/MG995 계열 서보 방향과 duty 관계를 확인하기 위한 파일입니다.

#### `tests/gimbal/test_gimbal_controller.py`

`GimbalController`와 `GimbalStepController` 단위 테스트입니다.

- `RPi.GPIO`를 `MagicMock`으로 대체하여 라즈베리파이가 아닌 환경에서도 테스트할 수 있게 구성했습니다.
- GPS/UWB 입력에 따라 duty가 0.1씩 증가 또는 감소하는지 확인합니다.
- 초기 위치가 상대각 0도인지 확인합니다.

### 5. 센서 코드

#### `code/gps/sensor.py`

GPS 시리얼 데이터를 읽는 클래스입니다.

- 기본 포트는 `/dev/ttyUSB0`, baudrate는 `115200`입니다.
- `serial.Serial`로 GPS 모듈에 연결합니다.
- `update()`에서 한 줄씩 읽고 `$GNGGA` NMEA 문장만 파싱합니다.
- `pynmea2`로 문장을 해석하고 유효하면 위도, 경도, timestamp를 저장합니다.
- `get_location()`은 유효한 GPS fix가 있을 때 `{"lat": ..., "lon": ..., "time": ...}` 딕셔너리를 반환합니다.

#### `code/uwb/sensor.py`

UWB 센서 클래스 자리만 정의되어 있습니다.

- `UWB.__init__()`과 `get_location()` 모두 `pass`입니다.
- 실제 UWB 장치 파싱, 거리/방위각/고도각 반환 로직은 아직 구현되어 있지 않습니다.

### 6. QR / 비전 코드

#### `code/qr/scanner.py`

스마트폰 IP Webcam 영상을 읽어 QR을 탐지하는 기본 스캐너입니다.

- OpenCV `VideoCapture`로 `http://{ip}:{port}/video` 스트림에 연결합니다.
- `cropped_frame()`은 화면 중앙 일부를 crop한 뒤 원본 크기로 확대해 디지털 줌처럼 사용합니다.
- `process_frame()`은 `pyzbar.decode()`로 QR/바코드를 찾습니다.
- QR 중심과 카메라 화면 중심 사이의 유클리드 픽셀 거리를 계산합니다.
- `run()`은 실시간으로 프레임을 계속 읽으며 QR 탐지를 수행합니다.

#### `code/qr/one_shot_scanner.py`

QR을 한 번 탐지하거나 timeout이 지나면 반환하는 실험용 스캐너입니다.

- `SmartPhoneScanner`를 상속합니다.
- `scan_once(timeout_sec)`는 제한 시간 동안 프레임을 읽고 QR이 검출되면 결과 딕셔너리를 반환합니다.
- 반환값은 `{"type": ..., "data": ..., "distance_px": ...}`입니다.
- 탐색 중 원본 프레임과 crop 프레임을 `result/detect_frame/`에 JPG로 저장합니다.

#### `code/qr/dist.py`

QR 중심 거리 계산을 시각적으로 확인하는 독립 실행 테스트입니다.

- 스마트폰 영상에 화면 중심 십자선, QR 박스, QR 중심점, 중심 간 거리선을 그립니다.
- OpenCV 창으로 실시간 확인할 수 있습니다.
- QR 중심과 화면 중심 사이의 거리(px)를 표시합니다.

#### `code/qr/ipwebcam_test.py`

IP Webcam 연결과 QR 인식 여부만 간단히 확인하는 테스트 코드입니다.

- 스마트폰 스트림에 연결합니다.
- QR 데이터가 바뀔 때만 터미널에 출력합니다.
- 거리 계산이나 시각화는 최소화되어 있습니다.

#### `code/qr/scale_test.py`

중앙 crop/확대 배율을 적용한 QR 인식 테스트 코드입니다.

- `scanner.py`와 비슷하지만 OpenCV 창에 crop된 화면과 QR 가이드 시각화를 보여줍니다.
- crop 비율에 따른 QR 인식 가능성을 확인할 때 사용합니다.

#### `code/qr/test.py`

QR 거리 측정과 시각화 테스트 코드입니다.

- QR 박스, 중심점, 화면 중심과 QR 중심 연결선, 거리 텍스트를 표시합니다.
- 코드 주석에는 약 4m 거리에서 모니터 QR 인식이 어려웠다는 실험 메모가 포함되어 있습니다.

### 7. 로깅 및 결과 파일

#### `code/logger/result_logger.py`

실험 결과를 CSV로 저장하는 공용 로거입니다.

- 현재 파일 위치 기준으로 프로젝트 루트를 계산합니다.
- 루트 아래 `result/` 폴더를 만들고, 모드별 CSV에 append합니다.
- `log_t_result(mode, data_dict)`
  - 시간/지연/소요시간 계열 데이터를 `{mode}_t_result.csv`에 저장합니다.
- `log_a_result(mode, data_dict)`
  - QR 인식 여부, 거리, 정렬 정확도 계열 데이터를 `{mode}_a_result.csv`에 저장합니다.
- `timestamp` 키가 없으면 현재 시간을 자동으로 추가합니다.

#### `result/*.csv`

실험 결과 CSV입니다.

- `gps_t_result.csv`: GPS 기반 정렬 시간, 전송 지연 등 시간 관련 결과
- `gps_a_result.csv`: GPS 기반 정렬 후 QR 인식 여부/거리 등 정확도 관련 결과
- `uwb_t_result.csv`: UWB 기반 시간 결과
- `uwb_a_result.csv`: UWB 기반 정확도 결과

#### `result/detect_frame/*.jpg`

`OneShotQRScanner`가 저장한 QR 탐색 프레임입니다.

- `whole_frame_*.jpg`: 원본 프레임
- `cropped_frame_*.jpg`: crop/확대 후 QR 인식에 사용한 프레임

### 8. 설정 및 보조 파일

#### `requirements.txt`

프로젝트 실행에 필요한 Python 패키지 목록입니다.

- `opencv-python`, `pyzbar`: QR/영상 처리
- `pyserial`, `pynmea2`: GPS 시리얼/NMEA 파싱
- `paho-mqtt`: MQTT 예제
- `RPi.GPIO`: 라즈베리파이 GPIO/PWM 제어
- `numpy` 및 GPS 관련 보조 패키지들이 포함되어 있습니다.

#### `code/start_positions.txt`

통합 실험에서 사용할 시작 각도 목록입니다.

- 현재 값은 `0, -25, -22.5, ... , 25` 형태의 콤마 구분 각도입니다.
- `main.py`, `main2.py`는 콤마와 공백 구분을 모두 처리할 수 있습니다.

#### `code/server/start_positions.txt`

서버 디렉토리 안의 별도 시작 각도 파일입니다.

- 현재는 `30`이 10번 반복되어 있습니다.
- 현재 주요 실행 흐름에서는 `code/start_positions.txt`가 더 직접적으로 사용됩니다.

#### `__init__.py` 파일들

`code/client`, `code/server`, `code/gimbal`, `code/gps`, `code/qr`, `code/uwb`, `code/logger` 패키지 구성을 위한 파일입니다.

- 대부분 내용은 비어 있습니다.
- Python이 각 폴더를 패키지로 인식하도록 돕습니다.

### 9. 현재 코드 기준 주의할 점

- 라즈베리파이와 실제 서보 모터가 없으면 `RPi.GPIO`를 사용하는 주요 실행 파일은 바로 실행하기 어렵습니다.
- GPS 기준 내 위치가 여러 파일에서 하드코딩되어 있어 실제 실험 위치에 맞게 수정해야 합니다.
- 스마트폰 IP Webcam 주소도 파일마다 다르므로 실험 환경에 맞게 통일해야 합니다.
- UWB 센서 파싱 클래스는 아직 구현되어 있지 않습니다. 다만 UDP 메시지의 `mode=1` 경로에서는 수신된 azimuth를 사용해 짐벌을 움직이는 로직이 일부 존재합니다.
- `socket_motor_test.py`, `socket_server_yp.py`는 현재 구조와 맞지 않거나 실행 오류 가능성이 있어 정리 필요 파일로 보는 것이 안전합니다.
- `ResultLogger`는 CSV 파일이 이미 존재할 때 새 컬럼 구성이 달라지면 기존 헤더와 새 행의 컬럼이 어긋날 수 있습니다. 실험 항목을 바꿀 때는 결과 파일을 분리하거나 헤더 정책을 정리하는 것이 좋습니다.
- `OneShotQRScanner`는 `scan_once()` 호출마다 프레임을 저장하므로 장시간 실험 시 `result/detect_frame/` 용량이 빠르게 커질 수 있습니다.
