# OCC 통신을 위한 정렬 시스템 (Alignment System for OCC)

Optical Camera Communication(OCC)을 위한 UWB 기반 짐벌 정렬 및 제어
시스템입니다. UWB 방위각을 이용한 짐벌 제어, 카메라 기반 인식 및 실험 결과
로깅 기능을 포함합니다.

---

## 📂 프로젝트 구조 및 주요 기능 (Directory Structure)

### 🛰️ Connection & Core

* **`rx_main.py`, `tx_main.py`**
  * UWB 패킷을 수신해 Rx/Tx 짐벌을 추적하고 결과를 기록합니다.
* **`rx_packet_main.py`, `tx_packet_main.py`**
  * 수신 패킷마다 즉시 정렬하는 실험 진입점입니다.

### 🎮 Control (짐벌 제어)
* **`gimbal/gimbal_controller_yaw.py`**
  * UWB 상대 방위각을 기반으로 PCA9685 서보 드라이버의 Yaw 축을 제어합니다.

### 📷 Vision (QR 코드 프로세싱)
* **`qr/scanner.py`**
  * 카메라를 통해 QR 코드를 인식하고 스캔하는 소스코드입니다.
* **`qr/dist.py`**
  * 인식된 QR 코드의 중앙점과 카메라 화면 중심부 사이의 이격 거리를 계산합니다.

### 📊 Logging (데이터 기록)
* **`logger/result_logger.py`**
  * 실험 중 발생하는 주요 데이터 및 원하는 측정값을 `result/<실험 코드>/` 폴더 내에 CSV 파일로 기록합니다.

---

## 코드 전체 분석 및 프로젝트 동작 설명

이 프로젝트는 OCC(Optical Camera Communication) 장치가 상대 장치 방향을
바라보도록 짐벌을 정렬하는 실험용 시스템입니다. 전체 흐름은
`UWB 패킷 수신 -> 상대 방위각 보정 -> 짐벌 서보 제어 -> 카메라 인식 ->
CSV 저장` 구조입니다.

### 1. 전체 실행 흐름

1. 각 장치는 UDP로 UWB 거리, azimuth, elevation 패킷을 수신합니다.
2. 최신 UWB azimuth를 ROS Yaw 좌표계로 변환하고 보정량을 제한합니다.
3. Rx는 PCA9685, Tx는 Raspberry Pi GPIO를 통해 짐벌을 제어합니다.
4. 실험 종류에 따라 카메라로 QR 또는 색상을 인식합니다.
5. 정렬 명령과 인식 결과를 `result/<실험 코드>/` 아래 CSV로 저장합니다.

### 정적 alignment 실험

`code/experiment/static_alignment_test.py`는 짐벌을 무작위 초기각으로 이동한 뒤
UWB로 정렬하고, 정렬 명령 시점부터 0.2초 안에 새 카메라 프레임에서 선택한
색상이 인식되는지 측정합니다. 기본 색상은 빨간색이고 영상 크롭은 하지
않습니다.

```bash
python code/experiment/static_alignment_test.py \
  --distance 2 \
  --attempts 100 \
  --servo-channel 0 \
  --pca9685-address 0x40
```

카메라 색상 안정화 시간은 기본 5초이며, 0도 복귀·초기각 이동·정렬 명령 후
안정화 시간은 각각 기본 1초입니다. 실험 중에는 PCA9685 PWM을 계속 유지합니다.

결과는 기본적으로 `result/static_alignment/run_YYYYMMDD_HHMMSS/` 아래의
`static_alignment_results.csv`에 저장됩니다. CSV에는 실험 거리, 인식 제한시간,
초기 짐벌각, UWB 원시/ROS 방위각, 계산 목표각, 실제 짐벌 명령각, 색상 인식 및
성공 여부, 프레임 ID와 캡처 시각이 포함됩니다. 색상 미검출이나 새 프레임
타임아웃이 발생하면 실패 원인과 마지막 프레임도 저장됩니다.

전체 옵션과 안정화 시간의 의미는
[`code/experiment/README.md`](code/experiment/README.md)를 참고합니다.

### 2. 짐벌 제어 코드

#### `code/gimbal/gimbal_controller_yaw.py`

Yaw 1축 서보 모터 제어의 핵심 클래스입니다.

- `adafruit-circuitpython-servokit`을 사용해 PCA9685에서 서보 제어 신호를 생성합니다.
- 기본 I²C 주소는 `0x40`, yaw 서보 채널은 `0`, 주파수는 50Hz입니다.
- `calculate_uwb_angles(my_pos, target_pos)`
  - UWB 타겟 데이터에서 azimuth 값을 그대로 yaw로 사용하는 단순 함수입니다.
- `move_to(az_degree)`
  - ROS yaw `-90~90도`를 입력받으며 양수는 반시계방향입니다.
  - 명령각 0도는 서보 90도, -90도(CW)는 서보 180도, +90도(CCW)는 서보 0도로 해석합니다.
  - UWB의 CW 양수 방위각은 부호를 반전해 ROS yaw로 변환합니다.
  - 반환값은 실제로 짐벌에 명령한 `gimbal_command_deg`입니다.
- `move_by_uwb_relative(uwb_relative_degree, wait=True)`
  - UWB가 준 현재 방향 기준 상대각을 이전 짐벌 명령각에 더해 다음 짐벌 명령각으로 변환합니다.
  - 기본값에서는 명령 후 약 0.6초 대기합니다.
- `cleanup()`
  - `servo.angle = None`으로 제어 신호를 비활성화합니다.

#### `code/gimbal/gimbal_controller_yaw_gpio.py`

PCA9685 없이 Raspberry Pi GPIO에서 50Hz 소프트웨어 PWM을 직접 생성하는 Tx용
yaw 컨트롤러입니다. `origin/main`의 직접 GPIO 구현을
`GPIOGimbalController`라는 별도 이름으로 가져와 PCA9685 컨트롤러와 함께
사용할 수 있게 했습니다.

- 기본 BCM GPIO 핀은 18번입니다.
- UWB의 CW 양수 원시각을 ROS의 CCW 양수 yaw로 반전해 누적합니다.
- ROS yaw는 `서보 각도 = 90 - ROS yaw`로 PWM에 변환하므로 기존 물리 회전
  방향은 유지됩니다.
- `tx_main.py`만 이 컨트롤러를 사용합니다.
- Rx 및 짐벌 카메라 실험은 기존 PCA9685 컨트롤러를 계속 사용합니다.
- Tx 실행 시 `--yaw-pin`으로 GPIO 핀을 변경할 수 있습니다.

```bash
python code/tx_main.py \
  --yaw-pin 18 \
  --samples 100 \
  --experiment-id EXPERIMENT_ID \
  --start-utc EPOCH_SEC
```

#### `tests/gimbal/step_controller.py`

`GimbalController`를 상속한 단계 이동 실험용/레거시 컨트롤러입니다.

- 목표 각도로 바로 이동하지 않고, 방향만 판단해서 ServoKit 각도를 1.8도씩 증가/감소시킵니다.
- UWB 모드에서는 azimuth 부호로 회전 방향을 판단합니다.
- 최종적으로 시계방향(ROS 음수)이면 ServoKit 180도, 반시계방향(ROS 양수)이면 0도까지 이동합니다.

#### `tests/gimbal/dir_init.py`

서보를 정북/중앙 위치로 초기화하는 간단한 하드웨어 테스트 코드입니다.

- PCA9685 주소 `0x40`, 채널 `0`, 50Hz로 서보를 설정합니다.
- 상대각 `-90°`, `0°`, `+90°` 동작과 제어 신호 비활성화를 확인합니다.
- MG996R 서보와 PCA9685 연결을 확인하기 위한 파일입니다.

#### `tests/gimbal/gimbal_uwb_tracking_color_test.py`

0.2초마다 가장 최신 UWB 상대각으로 yaw 짐벌을 동적으로 보정하고, 다음 정렬
시각까지 새 카메라 프레임에서 선택한 색상의 존재 여부를 검사합니다. 원형도는
성공 조건으로 사용하지 않으며, 전체 색상 면적과 최대 연결 영역 면적으로
판정합니다. 기본 대상은 빨간색이며
`--target-color`로 `red`, `orange`, `yellow`, `green`, `blue`, `purple` 중
하나를 선택할 수 있습니다. 기본 100회의 정렬·색상 판정을 마치면 자동으로
종료합니다.

```bash
python tests/gimbal/gimbal_uwb_tracking_color_test.py \
  --target-color red \
  --live-stream
```

결과는
`result/gimbal_uwb_tracking_color_test/run_YYYYMMDD_HHMMSS/color_results.csv`에
저장되며 색상 미검출 프레임은 같은 실행 폴더의 `failed_frames/`에 저장됩니다.
전체 알고리즘과 PCA9685·카메라 옵션은
`tests/gimbal/README.md`를 참고합니다.

#### `tests/gimbal/test_gimbal_controller.py`

`GimbalController`와 `GimbalStepController` 단위 테스트입니다.

- `ServoKit`을 mock으로 대체하여 PCA9685가 없는 환경에서도 테스트할 수 있게 구성했습니다.
- UWB 입력에 따라 ServoKit 각도가 1.8도씩 증가 또는 감소하는지 확인합니다.
- 초기 위치가 상대각 0도인지 확인합니다.

### 5. QR / 비전 코드

#### `code/camera/scanner.py`

스마트폰 IP Webcam 영상을 읽어 QR을 탐지하는 기본 스캐너입니다.

- OpenCV `VideoCapture`로 `http://{ip}:{port}/video` 스트림에 연결합니다.
- `cropped_frame()`은 화면 중앙 일부를 crop한 뒤 원본 크기로 확대해 디지털 줌처럼 사용합니다.
- `process_frame()`은 `pyzbar.decode()`로 QR/바코드를 찾습니다.
- QR 중심과 카메라 화면 중심 사이의 유클리드 픽셀 거리를 계산합니다.
- `run()`은 실시간으로 프레임을 계속 읽으며 QR 탐지를 수행합니다.

#### `code/camera/one_shot_scanner.py`

QR을 한 번 탐지하거나 timeout이 지나면 반환하는 실험용 스캐너입니다.

- `SmartPhoneScanner`를 상속합니다.
- `scan_once(timeout_sec)`는 제한 시간 동안 프레임을 읽고 QR이 검출되면 결과 딕셔너리를 반환합니다.
- 반환값은 `{"type": ..., "data": ..., "distance_px": ...}`입니다.
- 탐색 중 원본 프레임과 crop 프레임을 `result/<실험 코드>/detect_frame/`에 JPG로 저장합니다.

#### `code/camera/dist.py`

QR 중심 거리 계산을 시각적으로 확인하는 독립 실행 테스트입니다.

- 스마트폰 영상에 화면 중심 십자선, QR 박스, QR 중심점, 중심 간 거리선을 그립니다.
- OpenCV 창으로 실시간 확인할 수 있습니다.
- QR 중심과 화면 중심 사이의 거리(px)를 표시합니다.

#### `code/camera/ipwebcam_test.py`

IP Webcam 연결과 QR 인식 여부만 간단히 확인하는 테스트 코드입니다.

- 스마트폰 스트림에 연결합니다.
- QR 데이터가 바뀔 때만 터미널에 출력합니다.
- 거리 계산이나 시각화는 최소화되어 있습니다.

#### `code/camera/scale_test.py`

중앙 crop/확대 배율을 적용한 QR 인식 테스트 코드입니다.

- `scanner.py`와 비슷하지만 OpenCV 창에 crop된 화면과 QR 가이드 시각화를 보여줍니다.
- crop 비율에 따른 QR 인식 가능성을 확인할 때 사용합니다.

#### `code/camera/test.py`

QR 거리 측정과 시각화 테스트 코드입니다.

- QR 박스, 중심점, 화면 중심과 QR 중심 연결선, 거리 텍스트를 표시합니다.
- 코드 주석에는 약 4m 거리에서 모니터 QR 인식이 어려웠다는 실험 메모가 포함되어 있습니다.

### 6. 로깅 및 결과 파일

#### `code/logger/result_logger.py`

실험 결과를 CSV로 저장하는 공용 로거입니다.

- 현재 파일 위치 기준으로 프로젝트 루트를 계산합니다.
- 루트 아래 `result/<실험 코드>/` 폴더를 만들고, 모드별 CSV에 append합니다.
- `log_t_result(mode, data_dict)`
  - 시간/지연/소요시간 계열 데이터를 `{mode}_t_result.csv`에 저장합니다.
- `log_a_result(mode, data_dict)`
  - QR 인식 여부, 거리, 정렬 정확도 계열 데이터를 `{mode}_a_result.csv`에 저장합니다.
- `timestamp` 키가 없으면 현재 시간을 자동으로 추가합니다.

#### `result/<실험 코드>/*.csv`

실험 결과 CSV입니다.

- `uwb_t_result.csv`: UWB 기반 시간 결과
- `uwb_a_result.csv`: UWB 기반 정확도 결과

#### `result/<실험 코드>/detect_frame/*.jpg`

`OneShotQRScanner`가 저장한 QR 탐색 프레임입니다.

- `whole_frame_*.jpg`: 원본 프레임
- `cropped_frame_*.jpg`: crop/확대 후 QR 인식에 사용한 프레임

### 7. 설정 및 보조 파일

#### `requirements.txt`

프로젝트 실행에 필요한 Python 패키지 목록입니다.

- `opencv-python`, `pyzbar`: QR/영상 처리
- `adafruit-circuitpython-servokit`: PCA9685 기반 서보 제어
- `numpy`: 수치 계산

#### `code/start_positions.txt`

짐벌 실험에서 재사용할 수 있는 시작 각도 목록입니다.

- 현재 값은 `0, -25, -22.5, ... , 25` 형태의 콤마 구분 각도입니다.

#### `__init__.py` 파일들

`code/gimbal`, `code/camera`, `code/uwb`, `code/logger` 패키지 구성을 위한
파일입니다.

- 대부분 내용은 비어 있습니다.
- Python이 각 폴더를 패키지로 인식하도록 돕습니다.

### 8. 현재 코드 기준 주의할 점

- PCA9685와 실제 서보 모터가 없으면 주요 하드웨어 실행 파일은 바로 실행하기 어렵습니다.
- 스마트폰 IP Webcam 주소도 파일마다 다르므로 실험 환경에 맞게 통일해야 합니다.
- `ResultLogger`는 CSV 파일이 이미 존재할 때 새 컬럼 구성이 달라지면 기존 헤더와 새 행의 컬럼이 어긋날 수 있습니다. 실험 항목을 바꿀 때는 결과 파일을 분리하거나 헤더 정책을 정리하는 것이 좋습니다.
- `OneShotQRScanner`는 `scan_once()` 호출마다 프레임을 저장하므로 장시간 실험 시 `result/<실험 코드>/detect_frame/` 용량이 빠르게 커질 수 있습니다.
