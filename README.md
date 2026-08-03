# UWB-Guided Mobile OCC Alignment

UWB 상대 방위각으로 송신기(Tx)와 수신기(Rx)의 yaw 짐벌을 정렬하고,
카메라의 색상·QR 인식 결과를 기록하는 모바일 OCC(Optical Camera
Communication) 실험 프로젝트입니다.

Rx는 PCA9685 기반 짐벌과 카메라를 사용하고, Tx는 Raspberry Pi GPIO PWM으로
짐벌을 제어합니다. 정적 정렬 실험과 Chrony 기반 동기 동적 추적 실험, 결과
분석 및 시각화 도구를 포함합니다.

## 주요 기능

- UDP로 최신 UWB 거리·방위각·고도각 수신
- UWB 영점 편향의 원형 평균 기반 사전 보정
- UWB의 CW(시계방향) 양수 좌표계를 ROS yaw의 CCW(반시계방향) 양수로 변환
- Rx PCA9685 및 Tx GPIO 기반 `-90°~90°` yaw 짐벌 제어
- Chrony로 동기화된 Rx·Tx의 0.2초 주기 동적 추적
- 카메라 색상 표식 및 QR 인식, 실패 프레임과 CSV 기록
- 정적/동적 실험 결과의 통계 분석과 논문용 그래프 생성

## 시스템 구성

```text
UWB 장치 ── UDP:5005 ──> Rx / Tx 실험 프로세스
                             │
                  UWB 편향 보정 및 각도 변환
                             │
                 ┌───────────┴───────────┐
                 │                       │
          Rx: PCA9685 짐벌         Tx: GPIO PWM 짐벌
          + V4L2 카메라                   │
                 │                       │
          색상/QR 인식              정렬 상태 기록
                 └───────────┬───────────┘
                             │
                     result/ 아래 CSV 저장
```

동적 추적의 기본 각도 계산은 다음과 같습니다.

```text
보정 UWB 방위각 = normalize(UWB 원시 방위각 - UWB 영점 편향)
UWB ROS 상대각  = -보정 UWB 방위각
목표 ROS yaw    = 이전 짐벌 명령각 + UWB ROS 상대각
```

한 번의 동적 보정은 최대 60도로 제한하며, 최종 서보 명령은 `-90°~90°`로
제한합니다.

## 하드웨어 및 시스템 요구사항

### Rx

- Linux/Raspberry Pi 계열 장치
- PCA9685 서보 드라이버와 yaw 서보
- `/dev/videoX`로 접근 가능한 카메라
- UDP로 UWB 패킷을 받을 네트워크 인터페이스

### Tx

- Raspberry Pi
- BCM GPIO PWM으로 제어할 yaw 서보(기본 GPIO 18)
- UDP로 UWB 패킷을 받을 네트워크 인터페이스

### 공통

- Python 3
- 동적 동기 실험을 위한 `chrony`와 `chronyc`
- QR 인식을 위한 시스템 `zbar` 라이브러리
- Tx의 `RPi.GPIO` 패키지(Raspberry Pi OS 환경에서 별도 설치)

서보의 전원과 Raspberry Pi의 전원은 하드웨어 사양에 맞게 구성하고 GND를
공유해야 합니다. 서보를 연결하기 전에 PWM 핀, PCA9685 주소와 채널, 회전
방향을 확인하십시오.

## 설치

```bash
git clone https://github.com/JEONG-JIN-99/uwb-guided-mobile-occ-alignment.git
cd uwb-guided-mobile-occ-alignment

python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

운영체제 패키지는 사용하는 배포판에 맞게 설치합니다. Debian/Raspberry Pi
OS 계열의 예시는 다음과 같습니다.

```bash
sudo apt update
sudo apt install chrony libzbar0 python3-rpi.gpio
```

설치 후 각 진입점의 전체 옵션은 `--help`로 확인할 수 있습니다.

```bash
python code/experiment/dynamic_tracking/rx_dynamic_tracking.py --help
python code/experiment/static_alignment/static_alignment_test.py --help
```

## UWB 입력 형식

실험 코드는 기본적으로 `0.0.0.0:5005`에 바인딩하고 다음 UTF-8 CSV 형식의
UDP 데이터그램을 받습니다.

```text
1,distance,azimuth,elevation,...
```

- 첫 필드는 유효 UWB 패킷을 나타내는 `1`이어야 합니다.
- `distance`는 0 이상의 미터 값입니다.
- `azimuth`는 `-180 <= azimuth < 180` 범위이며 CW가 양수입니다.
- `elevation`은 현재 로그에 보존되지만 yaw 제어에는 사용하지 않습니다.
- 뒤에 추가 필드가 있어도 앞의 네 필드만 파싱합니다.

## 빠른 시작: 동적 추적

동적 실험은 Rx와 Tx의 시계를 Chrony로 동기화하고, 양쪽에 동일한
`experiment-id`, `start-utc`, `distance`를 전달합니다. `distance`는 1/2/3 m
고정 반경 호 실험에서 각각 `1`, `2`, `3`을 사용하고 랜덤 이동 로버는 `0`을
사용합니다.

### 1. 짐벌 초기 정렬

Rx에서 PCA9685 짐벌과 카메라 중심을 맞춥니다.

```bash
python code/experiment/dynamic_tracking/rx_dir_init.py \
  --device-index 4 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --live-stream
```

Tx에서 GPIO 짐벌을 0도로 맞춥니다.

```bash
python code/experiment/dynamic_tracking/tx_dir_init.py --yaw-pin 18
```

### 2. 공통 시작 시각 생성

두 프로세스를 준비할 여유를 두고 미래 UTC epoch seconds를 한 번 생성합니다.

```bash
START_UTC=$(date -u -d '60 seconds' +%s)
echo "$START_UTC"
```

출력된 값을 Rx와 Tx에서 동일하게 사용합니다. 기본 UWB 캘리브레이션은 시작
1초 전까지 새 패킷 100개를 모으므로 실제 패킷 주기에 맞게 충분한 여유를
두어야 합니다.

### 3. Rx와 Tx 실행

Rx:

```bash
python code/experiment/dynamic_tracking/rx_dynamic_tracking.py \
  --experiment-id dynamic_01 \
  --start-utc "$START_UTC" \
  --distance 2 \
  --device-index 4 \
  --target-color red
```

Tx:

```bash
python code/experiment/dynamic_tracking/tx_dynamic_tracking.py \
  --experiment-id dynamic_01 \
  --start-utc "$START_UTC" \
  --distance 2 \
  --yaw-pin 18
```

두 프로그램은 0.2초의 절대 예정 시각마다 최신 UWB 패킷으로 정렬하며
`Ctrl+C`를 누를 때까지 실행합니다. 기본 버전은 새 패킷이 없을 때 실험 시작
후 마지막 유효 패킷을 재사용합니다. 패킷을 한 번만 사용하려면
`rx_dynamic_tracking_no_packet_reuse.py`와
`tx_dynamic_tracking_no_packet_reuse.py`를 실행합니다.

세부 알고리즘, 캘리브레이션 기준, CSV 스키마와 모든 옵션은
[동적 추적 실험 문서](code/experiment/dynamic_tracking/README.md)를 참고하십시오.

## 빠른 시작: 정적 정렬

정적 실험 전에는 Rx/Tx 방향 초기화 후 카메라 색상 인식 사전 검사를 권장합니다.

```bash
python code/experiment/static_alignment/rx_dir_init.py --live-stream
python code/experiment/static_alignment/tx_dir_init.py --yaw-pin 18
python code/experiment/static_alignment/red_detection_precheck.py --distance 2
```

정렬 명령 직후 0.2초 동안 색상 인식을 측정합니다.

```bash
python code/experiment/static_alignment/static_alignment_test.py \
  --distance 2 \
  --attempts 100
```

정렬 후 기본 0.5초 안정화한 다음 0.2초 동안 측정하려면 다음 진입점을
사용합니다.

```bash
python code/experiment/static_alignment/static_alignment_after_settle_test.py \
  --distance 2 \
  --attempts 100
```

실험 순서, 초기각 층화 표본, 색상 판정 기준과 전체 옵션은
[정적 정렬 실험 문서](code/experiment/static_alignment/README.md)를 참고하십시오.

## 결과와 분석

모든 런타임 결과는 프로젝트 루트의 `result/` 아래에 저장됩니다.

```text
result/
├── <dynamic-condition>/<experiment-id>/
│   ├── rx.csv
│   ├── tx.csv
│   └── failed_frames/
├── static_alignment/run_YYYYMMDD_HHMMSS/
│   ├── static_alignment_results.csv
│   └── failed_frames/
└── static_alignment_after_settle/run_YYYYMMDD_HHMMSS/
    ├── static_alignment_results.csv
    └── failed_frames/
```

대표 분석 명령:

```bash
# 정적 정렬 명령각 오차 분석
python code/experiment/static_alignment/analyze_alignment_error.py \
  result/static_alignment/run_YYYYMMDD_HHMMSS

# 동적 추적 실험별 그래프와 보고서 생성
python code/experiment/dynamic_tracking/analyze_individual_experiments.py \
  --result-root result/dynamic_tracking \
  --experiment EXPERIMENT_ID

# 로버 전체 좌표계 XY 궤적 시각화
python code/experiment/dynamic_tracking/plot_rover_world_trajectory.py \
  path/to/rover_log.csv
```

`result/`, 루트 실험 로그 CSV와 생성 지도 이미지는 `.gitignore` 대상입니다.
따라서 결과 데이터와 실패 프레임은 로컬에 남지만 일반적인 `git add`나
`git push`에는 포함되지 않습니다.

## 프로젝트 구조

```text
code/
├── camera/                 # V4L2/IP 카메라, QR 및 색상 탐지
├── experiment/
│   ├── dynamic_tracking/   # Chrony 동기 Rx/Tx 추적 및 결과 분석
│   └── static_alignment/   # 정적 정렬, 사전 검사 및 결과 분석
├── gimbal/                 # PCA9685/GPIO yaw 짐벌 컨트롤러
├── logger/                 # CSV 결과 로거
├── time_sync/              # UTC/monotonic 변환과 Chrony 검사
├── packet_immediate_runner.py
├── rx_main.py              # 고정 샘플 수 기반 Rx 실험 진입점
├── tx_main.py              # 고정 샘플 수 기반 Tx 실험 진입점
├── rx_packet_main.py       # UWB 패킷 도착 즉시 정렬하는 Rx 진입점
└── tx_packet_main.py       # UWB 패킷 도착 즉시 정렬하는 Tx 진입점
tests/                      # 계산, 파서, 로깅 및 하드웨어 mock 테스트
requirements.txt
```

`rx_main.py`/`tx_main.py`와 packet-immediate 진입점은 비교 실험용으로 보존되어
있습니다. 현재 정적·동적 실험은 `code/experiment/` 아래의 전용 진입점을
우선 사용하십시오.

## 테스트

테스트는 Python 표준 `unittest`로 실행할 수 있습니다. 일부 `tests/` 파일은
실제 하드웨어 확인용 스크립트이므로 전체 자동 탐색보다 대상 테스트를
명시하는 방식을 권장합니다.

```bash
python tests/experiment/dynamic_tracking/test_dynamic_tracking.py -q
python tests/experiment/static_alignment/test_static_alignment.py -q
python tests/experiment/static_alignment/test_analyze_alignment_error.py -q
```

하드웨어 없이 실행하는 테스트는 PCA9685, GPIO 또는 카메라 접근을 mock하거나
순수 계산 함수만 검증합니다.

## 주의사항

- 실험 시작 전 Rx와 Tx가 정면을 향한 정지 상태에서 UWB 영점 보정을 수행해야
  합니다.
- Chrony 동기화가 설정 기준을 만족하지 않거나 `start-utc`가 이미 지난 경우
  동적 실험은 시작하지 않습니다.
- 기본 동적 추적은 마지막 UWB 패킷의 재사용 시간 제한이 없습니다. UWB 송신이
  중단되면 마지막 상대각이 반복 적용될 수 있으므로 로그의
  `uwb_packet_reused`와 `servo_clipped`를 함께 확인하십시오.
- `--live-stream`은 GUI 세션이 필요합니다. 헤드리스 장치에서는 생략하십시오.
- 현재 저장소에는 별도의 라이선스 파일이 없습니다. 재사용 또는 배포 전
  저장소 소유자에게 이용 조건을 확인하십시오.
