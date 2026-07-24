# Gimbal UWB Tracking QR Test

`gimbal_uwb_tracking_qr_test.py`는 이동하는 UWB 상대 장치를 yaw 짐벌로 추적하면서
QR 인식을 병렬로 수행하는 하드웨어 테스트다. 짐벌 제어는 0.2초 주기로 실행되며,
각 주기에서 대기 중인 UWB 패킷 중 가장 최신 패킷만 사용한다.

## 필요한 하드웨어

- Raspberry Pi
- PCA9685 서보 드라이버
- yaw 짐벌 서보모터
- UWB 수신 모듈
- RealSense 카메라

기본 PCA9685 설정은 다음과 같다.

| 설정 | 기본값 |
|---|---:|
| I2C 주소 | `0x40` |
| yaw 서보 채널 | 0 |
| PWM 주파수 | 50Hz |
| 서보 펄스 폭 | 500~2500µs |

## UWB 패킷 형식

UDP로 수신하는 패킷은 쉼표로 구분된 텍스트다.

```text
header,distance,azimuth,elevation,...
```

예:

```text
1,2.5,-12.4,0.7
```

- `header`가 `1`인 패킷만 UWB 모드 패킷으로 처리한다.
- `azimuth`는 현재 짐벌 방향을 기준으로 한 상대각(degree)으로 사용한다.
- 다른 header를 가진 패킷은 무시한다.

## 실행 알고리즘

### 1. 장치 초기화

1. 지정한 host와 port에 UDP 소켓을 연다.
2. PCA9685 기반 `GimbalController`를 초기화한다.
3. RealSense 카메라 캡처를 시작한다.
4. 실행별 결과 디렉터리와 `qr_results.csv`를 생성한다.
5. 짐벌 제어를 막지 않도록 별도의 QR 인식 스레드를 시작한다.
6. 짐벌을 `--initial-deg`로 지정한 초기 상대각으로 이동한다.

### 2. UWB 추적

추적 루프는 `0.2초`마다 실행된다.

1. UDP 수신 버퍼에 쌓인 패킷을 모두 읽는다.
2. 오래된 패킷은 버리고 가장 최신 패킷 하나만 선택한다.
3. header가 `1`인 패킷에서 UWB 상대 방위각을 읽는다.
4. 한 주기의 보정각을 `-60도~+60도`로 제한한다.
5. 이전 짐벌 명령각에 보정각을 더한다.
6. 최종 짐벌 명령각을 `-90도~+90도`로 제한한다.
7. PCA9685를 통해 새로운 각도를 서보에 명령한다.

각도 계산은 다음과 같다.

```text
주기별 보정각 = UWB 상대 방위각을 -60도 이상 +60도 이하로 제한
새 짐벌 명령각 = 이전 짐벌 명령각 + 주기별 보정각
최종 명령각 = 새 짐벌 명령각을 -90도 이상 +90도 이하로 제한
```

예:

```text
이전 짐벌 명령각:  +20도
UWB 상대 방위각:   -15도
새 짐벌 명령각:     +5도
```

UWB 패킷이 없는 주기에는 짐벌 명령과 QR 작업을 생성하지 않는다.

### 3. QR 인식

각 UWB 정렬 명령 직후 QR 작업을 별도 스레드에 전달한다.

- QR 인식 마감 시각은 다음 짐벌 제어 주기가 시작되는 시각이다.
- 따라서 각 QR 인식 구간은 최대 약 0.2초다.
- QR 작업 큐의 크기는 1이다.
- 이전 QR 작업이 끝나지 않았다면 새 QR 작업은 건너뛴다.
- QR 인식 때문에 짐벌의 0.2초 추적 주기가 직접 정지하지 않는다.

QR 결과는 다음과 같이 분류된다.

| 결과 | 의미 |
|---|---|
| `qr_visible=1`, `qr_decoded=1` | QR을 찾고 내용을 디코딩함 |
| `qr_visible=1`, `qr_decoded=0` | QR 형태는 보이지만 디코딩하지 못함 |
| `qr_visible=0`, `qr_decoded=0` | QR을 찾지 못함 |

QR 디코딩에 실패하면 해당 프레임을 `failed_frames/`에 저장한다.

### 4. 종료 처리

`Ctrl+C`로 종료하면 다음 순서로 정리한다.

1. QR 인식 스레드를 종료한다.
2. CSV 파일을 닫는다.
3. RealSense 카메라 캡처를 종료한다.
4. 짐벌을 상대각 `0도`로 복귀시킨다.
5. PCA9685 서보 제어 신호를 비활성화한다.
6. UDP 소켓을 닫는다.

## 기본 실행 명령어

프로젝트 루트에서 실행한다.

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py
```

기본 실행 설정:

| 옵션 | 기본값 |
|---|---:|
| UDP bind host | `0.0.0.0` |
| UDP port | 5005 |
| PCA9685 주소 | `0x40` |
| 서보 채널 | 0 |
| 초기 짐벌 상대각 | 0도 |
| RealSense 장치 번호 | 4 (`/dev/video4`) |
| QR 중앙 crop 비율 | 0.3 |
| 카메라 준비 시간 | 1초 |
| 실시간 영상 | 사용하지 않음 |

## 전체 옵션 실행 예시

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py \
  --host 0.0.0.0 \
  --port 5005 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --initial-deg 0 \
  --qr-device-index 4 \
  --qr-crop-scale 0.3 \
  --camera-warmup 1
```

## 실시간 영상 표시

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py \
  --live-stream
```

## 초기 짐벌각 지정

실험 시작 시 짐벌을 `-30도`에 배치하려면 다음과 같이 실행한다.

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py \
  --initial-deg -30
```

짐벌 명령각은 컨트롤러에서 `-90도~+90도`로 제한된다.

## PCA9685 설정 변경

채널 1과 I2C 주소 `0x41`을 사용하려면 다음과 같이 실행한다.

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py \
  --servo-channel 1 \
  --pca9685-address 0x41
```

## 결과 저장 위치 변경

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py \
  --output-dir result/my_tracking_test
```

실행 시각별로 다음 구조의 디렉터리가 생성된다.

```text
result/my_tracking_test/
└── run_20260724_153000/
    ├── qr_results.csv
    └── failed_frames/
        ├── attempt_000001_not_visible.jpg
        └── attempt_000002_visible_not_decoded.jpg
```

`qr_results.csv`는 다음 필드를 포함한다.

| 필드 | 설명 |
|---|---|
| `qr_visible` | QR 형태가 프레임에서 보였는지 여부 |
| `qr_decoded` | QR 데이터를 디코딩했는지 여부 |
| `qr_data` | 디코딩한 QR 문자열 |
| `failure_frame` | 실패 프레임의 실행 디렉터리 기준 상대 경로 |

## 주요 옵션

| 옵션 | 설명 |
|---|---|
| `--host` | UWB UDP bind 주소 |
| `--port` | UWB UDP bind 포트 |
| `--servo-channel` | PCA9685 yaw 서보 채널 |
| `--pca9685-address` | PCA9685 I2C 주소 |
| `--initial-deg` | 시작 시 짐벌 상대각 |
| `--qr-device-index` | QR 카메라 V4L2 장치 번호 |
| `--qr-crop-scale` | QR 탐지에 사용할 중앙 영상 비율, `0` 초과 `1` 이하 |
| `--camera-warmup` | 카메라 캡처 시작 후 준비 시간 |
| `--output-dir` | 실행별 결과가 저장될 상위 디렉터리 |
| `--live-stream` | QR 카메라 실시간 영상 표시 |

## 종료

실험을 종료하려면 터미널에서 다음 키를 누른다.

```text
Ctrl+C
```
