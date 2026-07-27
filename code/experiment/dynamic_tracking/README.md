# Dynamic Tracking Experiment

이 폴더에는 Chrony로 시각을 맞춘 Rx와 Tx가 같은 UTC 시각에 동적 UWB 추적을
시작하는 실험 코드가 있다.

- `rx_dynamic_tracking.py`: PCA9685 서보 드라이버로 Rx yaw 짐벌을 정렬하고
  다음 정렬 전까지 색상을 인식한다.
- `tx_dynamic_tracking.py`: Raspberry Pi GPIO 50Hz PWM으로 Tx yaw 짐벌만
  정렬한다.
- `rx_dynamic_tracking_no_packet_reuse.py`,
  `tx_dynamic_tracking_no_packet_reuse.py`: 각 UWB 패킷을 한 번만 사용하는
  기존 동작의 보존 버전이다.

두 프로그램 모두 사용자가 `Ctrl+C`를 누를 때까지 계속 실행한다. 현재 종료
시각이나 샘플 수에 의한 자동 종료 조건은 없다.

## 실험 전 Rx 짐벌·카메라 중앙 정렬

동적 추적을 시작하기 전에 카메라가 연결된 Rx 장치에서 다음 준비 스크립트를
실행한다. 처음 실행하는 장치에서는 프로젝트 의존성을 먼저 설치해야 한다.

```bash
python -m pip install -r requirements.txt
```

```bash
python code/experiment/dynamic_tracking/dir_init.py \
  --device-index 4 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --crop-scale 0.3 \
  --live-stream
```

스크립트는 Rx 짐벌을 ROS yaw 0도로 이동해 3초간 안정화한 뒤 PWM 신호를
끄고, 카메라 화면 중앙에 빨간 십자선을 표시한다. 십자선을 기준으로 카메라와
실험 표식의 중앙을 맞춘 다음 `q` 또는 `Esc`로 종료한다. `--device-index`,
`--servo-channel`, `--pca9685-address`, `--crop-scale`은 이어서 실행할
`rx_dynamic_tracking.py`와 같은 값을 사용해야 한다.

카메라 화면 없이 Rx 짐벌만 0도로 맞추려면 `--live-stream`을 생략한다.
PWM을 끈 뒤에는 서보 유지 토크가 사라지므로 중앙 정렬 후 짐벌이 물리적으로
움직이지 않도록 주의한다. Tx에는 카메라가 없고 GPIO 짐벌을 사용하므로 이
PCA9685 준비 스크립트는 Rx에서만 실행한다.

## 공통 동작 원칙

### Chrony 동기 시작

Rx와 Tx에서 각각 `chronyc waitsync`로 로컬 시계의 동기 상태를 확인한 다음,
동일한 `--start-utc`와 `--experiment-id`를 사용한다.

`--start-utc`는 Unix epoch seconds 형식의 **실험 시작 예정 시각**이다.
1970-01-01 00:00:00 UTC부터 해당 시각까지 흐른 초를 뜻하며, 장치나
프로그램이 자동으로 정해 주는 값이 아니다. 사용자가 미래 시각을 하나
만들어 Rx와 Tx에 같은 값을 입력해야 한다. Rx는 하드웨어 초기화와 카메라
안정화에 기본 5초가 필요하므로 두 프로그램을 실행하고도 충분히 시간이
남도록 보통 15~30초 뒤의 시각을 지정한다.

Linux에서 현재 시각의 30초 뒤를 시작시각으로 만드는 예:

```bash
date -u -d '30 seconds' +%s
```

예를 들어 위 명령이 `1785069000`을 출력했다면 `--start-utc 1785069000`으로
입력한다. 60초 뒤에 시작하고 싶으면 `30`을 `60`으로 바꾼다.

```bash
date -u -d '60 seconds' +%s
```

`+%s`는 사람이 읽는 날짜를 Unix epoch seconds 정수로 출력하라는 의미다.
반대로 `@`를 붙인 다음 명령은 epoch seconds가 실제로 어떤 UTC 날짜와
시각인지 확인할 때만 사용한다. 실험 실행에 필수인 명령은 아니다.

```bash
date -u -d @1785069000 '+%Y-%m-%d %H:%M:%S UTC'
```

위 명령은 다음처럼 사람이 읽을 수 있는 UTC 시각을 출력한다. 여기서 `@`는
뒤의 숫자를 Unix epoch seconds로 해석하라는 뜻이다.

```text
2026-07-26 12:30:00 UTC
```

정리하면 다음 순서로 사용한다.

1. 한 장치에서 `date -u -d '30 seconds' +%s`를 한 번 실행한다.
2. 출력된 숫자를 복사한다.
3. 그 숫자를 Rx와 Tx 양쪽의 `--start-utc`에 동일하게 입력한다.

예를 들어 두 장치에서 다음 두 값을 똑같이 사용한다. 아래 값은 형식을
보여주기 위한 예시이므로 실제 실행할 때는 위 명령으로 새 미래 시각을 만든다.

```text
experiment-id: dynamic_20260726_01
start-utc:     1785069000
```

### 고정 0.2초 정렬

정렬 주기는 옵션으로 변경할 수 없으며 항상 0.2초다. 공유 시작시각을 기준으로
절대 목표 시각을 계산하기 때문에 각 반복의 처리시간을 이전 목표에 더하는
방식의 누적 드리프트는 발생하지 않는다.

```text
sample 0: start + 0.0초
sample 1: start + 0.2초
sample 2: start + 0.4초
...
```

Python, 운영체제 및 하드웨어는 실시간 시스템이 아니므로 실제 명령 지연은
`command_elapsed_s - scheduled_elapsed_s`로 분석한다.

### 최신 UWB 패킷 재사용

수신 스레드는 유효한 UWB 패킷 중 최신 한 개만 크기 1의 큐에 유지한다.
더 새로운 패킷이 들어오면 아직 처리하지 않은 이전 패킷을 버린다.

기본 `rx_dynamic_tracking.py`와 `tx_dynamic_tracking.py`는 각 0.2초 정렬
시점에 새 패킷이 있으면 그중 최신 패킷을 사용한다. 새 패킷이 없으면 실험
시작 후 마지막으로 사용한 유효 패킷을 다시 사용한다. 따라서 첫 유효 패킷을
받은 뒤에는 UWB 송신 주기와 정렬 주기가 어긋나도 마지막 방위각으로 계속
보정한다. 공유 시작시각 이전에 받은 패킷은 캐시하거나 재사용하지 않는다.
아직 실험 시작 후 유효 패킷을 한 번도 받지 못한 경우에만 짐벌 명령을 만들지
않고 `status=uwb_unavailable`을 기록한다.

재사용 시간 제한은 없다. UWB 송신이 중단되어도 마지막 패킷을 매 0.2초마다
계속 적용하므로 필요하면 `Ctrl+C`로 실험을 중단해야 한다.

CSV의 `uwb_packet_reused`는 새 패킷이면 `0`, 이전 패킷을 다시 쓴
정렬이면 `1`이다.

UWB 방위각은 상대 오차각이므로 재사용 시 같은 오차가 이전 짐벌 명령각에
반복해서 더해질 수 있다. 실험 결과를 분석할 때는
`uwb_packet_reused`, `uwb_received_monotonic_ns`, `servo_clipped`를 함께
확인한다. 패킷을 재사용하지 않는 기존 동작은 아래의 별도 보존 버전으로
실행할 수 있다.

유효 패킷 형식:

```text
1,distance,azimuth,elevation,...
```

`header=1`, 유한한 숫자, 음수가 아닌 거리, `-180 <= azimuth < 180`을
만족해야 한다.

### 동적 각도 계산

UWB 원시 방위각은 CW 양수, 짐벌 로그와 명령은 ROS CCW 양수다.

```text
UWB ROS 상대각 = -UWB 원시 상대각
계산 목표각 = 이전 짐벌 ROS 명령각 + UWB ROS 상대각
```

0.2초 동안 서보가 이동할 수 있는 범위를 고려해 한 번의 실제 보정은 최대
60도로 제한한다. 1도 미만은 deadband로 처리한다. 그다음 실제 명령을 서보
가동 범위 `-90~90도`로 제한한다.

- `target_calculated_ros_deg`: deadband, 60도 주기 제한 및 서보 범위를
  적용하기 전 논리적 목표각
- `gimbal_command_ros_deg`: 60도 주기 제한과 서보 범위 제한을 적용해 실제로
  보낸 명령각
- `servo_clipped`: 60도 보정 제한 이후 요청각이 `-90~90도`를 벗어나 최종
  서보 범위에서 잘렸는지 여부

## Rx 실행

### 필수 옵션

Rx 실행 시 아래 세 옵션은 반드시 입력해야 한다. 나머지 옵션은 기본값이
있으며 장치 구성과 실험 조건에 맞을 때만 변경한다.

| 옵션 | 입력할 값 |
|---|---|
| `--experiment-id` | 이번 실험을 구분할 ID. Rx와 Tx에 같은 값 입력 |
| `--start-utc` | 위 방법으로 만든 미래 Unix epoch seconds. Rx와 Tx에 같은 값 입력 |
| `--distance` | `1`, `2`, `3`: 해당 m의 고정 반경 호, `0`: 랜덤 이동 |

최소 실행 명령:

```bash
python code/experiment/dynamic_tracking/rx_dynamic_tracking.py \
  --experiment-id dynamic_20260726_01 \
  --start-utc 1785069000 \
  --distance 2
```

카메라 번호, 서보 채널 및 PCA9685 주소가 기본값과 다르면 해당 옵션도
실제 실행에 맞게 지정해야 한다.

```bash
python code/experiment/dynamic_tracking/rx_dynamic_tracking.py \
  --experiment-id dynamic_20260726_01 \
  --start-utc 1785069000 \
  --distance 2 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --device-index 4 \
  --target-color red
```

### Rx 실행 알고리즘

1. Chrony 동기 상태를 확인한다.
2. PCA9685 Rx 짐벌을 초기화하고 `--initial-deg`로 이동한다.
3. 카메라 수신 스레드를 시작하고 `--camera-warmup` 5초 동안 자동 노출과
   화이트 밸런스를 안정화한다.
4. UWB 최신값 수신 스레드와 결과 저장 스레드를 시작한다.
5. `--start-utc`까지 기다린다.
6. 매 0.2초 절대 예정 시각에 최신 UWB 패킷으로 정렬하고, 새 패킷이 없으면
   마지막 패킷을 재사용한다.
7. 정렬 직전 프레임 ID와 정렬 명령 시각을 기준으로, 다음 정렬 절대
   예정 시각까지만 새 카메라 프레임의 지정 색상을 검사한다.
8. 결과 CSV와 실패 이미지는 다음 정렬을 지연시키지 않도록 별도 저장
   스레드에 전달한다.
9. `Ctrl+C`를 누르면 큐에 남은 결과를 저장하고 카메라, UWB 소켓 및 짐벌을
   정리한다.

Rx 색상 결과:

| 상황 | `color_visible` | `color_success` | `status` |
|---|---:|---:|---|
| 새 프레임에서 지정 색상 검출 | 1 | 1 | `success` |
| 새 프레임에서 지정 색상 미검출 | 0 | 0 | `color_not_detected` |
| 다음 정렬까지 새 프레임 없음 | 빈 값 | 0 | `camera_frame_timeout` |
| 실험 시작 후 유효 UWB를 아직 받지 못함 | 빈 값 | 0 | `uwb_unavailable` |
| 처리 예외 | 판정 상태에 따라 다름 | 0 | `error` |

실패 프레임 저장은 기본으로 켜져 있다. `color_not_detected`와
`camera_frame_timeout`에서 저장할 프레임이 있으면 `failed_frames/`에
기록한다. `--no-save-failure-frames`로 끌 수 있다.

### Rx 옵션

| 옵션 | 기본값 | 의미 |
|---|---:|---|
| `--experiment-id` | 필수 | Rx/Tx가 공유할 실험 ID |
| `--start-utc` | 필수 | 공유 미래 시작시각, Unix epoch seconds |
| `--distance` | 필수 | `1/2/3`: 해당 m의 고정 반경 호, `0`: 랜덤 이동 |
| `--host`, `--port` | `0.0.0.0`, `5005` | UWB UDP 수신 주소 |
| `--servo-channel` | 0 | PCA9685 yaw 서보 채널 |
| `--pca9685-address` | `0x40` | PCA9685 I2C 주소 |
| `--initial-deg` | 0 | 시작 ROS 짐벌각 |
| `--device-index` | 4 | 카메라 `/dev/videoX` 번호 |
| `--crop-scale` | 0.3 | 중앙 영상 사용 비율, 기본은 중앙 30% |
| `--camera-warmup` | 5초 | 시작 전 카메라 색상 안정화 시간 |
| `--target-color` | `red` | 찾을 색상 |
| `--color-min-area` | 500 | 최소 전체 색상 마스크 픽셀 |
| `--color-min-component-area` | 200 | 최소 최대 연결 색상 영역 |
| `--save-failure-frames` | 켜짐 | 색상 실패 프레임 저장 |
| `--live-stream` | 꺼짐 | 카메라 영상 창 표시 |
| `--chrony-max-correction-sec` | 0.005초 | 허용할 Chrony 잔여 보정 |
| `--chrony-wait-tries` | 60 | 1초 간격 Chrony 확인 최대 횟수 |

## Tx 실행

### 필수 옵션

Tx 실행 시에도 아래 세 옵션을 반드시 입력해야 한다. 나머지 옵션은 기본값이
있다.

| 옵션 | 입력할 값 |
|---|---|
| `--experiment-id` | 이번 실험을 구분할 ID. Rx와 Tx에 같은 값 입력 |
| `--start-utc` | 위 방법으로 만든 미래 Unix epoch seconds. Rx와 Tx에 같은 값 입력 |
| `--distance` | `1`, `2`, `3`: 해당 m의 고정 반경 호, `0`: 랜덤 이동 |

최소 실행 명령:

```bash
python code/experiment/dynamic_tracking/tx_dynamic_tracking.py \
  --experiment-id dynamic_20260726_01 \
  --start-utc 1785069000 \
  --distance 2
```

사용할 BCM GPIO 핀이 기본값 18번과 다르면 `--yaw-pin`도 지정해야 한다.

```bash
python code/experiment/dynamic_tracking/tx_dynamic_tracking.py \
  --experiment-id dynamic_20260726_01 \
  --start-utc 1785069000 \
  --distance 2 \
  --yaw-pin 18
```

### Tx 실행 알고리즘

1. Chrony 동기 상태를 확인한다.
2. Raspberry Pi GPIO 50Hz PWM 짐벌을 초기화하고 `--initial-deg`로 이동한다.
3. UWB 최신값 수신 스레드를 시작한다.
4. `--start-utc`까지 기다린다.
5. 매 0.2초 절대 예정 시각에 최신 UWB 패킷으로 정렬하고, 새 패킷이 없으면
   마지막 패킷을 재사용한다.
6. 성공 또는 `uwb_unavailable` 결과를 정렬 주기를 막지 않는 별도 저장
   스레드에서 CSV에 기록한다.
7. `Ctrl+C`를 누르면 짐벌을 0도로 복귀시키고 PWM과 GPIO를 정리한다.

Tx에는 카메라와 색상 인식이 없다.

### Tx 옵션

| 옵션 | 기본값 | 의미 |
|---|---:|---|
| `--experiment-id` | 필수 | Rx/Tx가 공유할 실험 ID |
| `--start-utc` | 필수 | 공유 미래 시작시각, Unix epoch seconds |
| `--distance` | 필수 | `1/2/3`: 해당 m의 고정 반경 호, `0`: 랜덤 이동 |
| `--host`, `--port` | `0.0.0.0`, `5005` | UWB UDP 수신 주소 |
| `--yaw-pin` | 18 | 직접 PWM을 출력할 BCM GPIO 핀 |
| `--initial-deg` | 0 | 시작 ROS 짐벌각 |
| `--chrony-max-correction-sec` | 0.005초 | 허용할 Chrony 잔여 보정 |
| `--chrony-wait-tries` | 60 | 1초 간격 Chrony 확인 최대 횟수 |

## 패킷을 재사용하지 않는 보존 버전

기존 방식처럼 각 UWB 패킷을 한 번만 사용하려면 파일명에
`no_packet_reuse`가 붙은 Rx와 Tx를 실행한다. 두 장치에서 반드시 같은
버전을 사용해야 한다.

Rx:

```bash
python code/experiment/dynamic_tracking/rx_dynamic_tracking_no_packet_reuse.py \
  --experiment-id dynamic_no_reuse_20260726_01 \
  --start-utc 1785069000 \
  --distance 2
```

Tx:

```bash
python code/experiment/dynamic_tracking/tx_dynamic_tracking_no_packet_reuse.py \
  --experiment-id dynamic_no_reuse_20260726_01 \
  --start-utc 1785069000 \
  --distance 2
```

이 버전은 정렬 시점에 새 미처리 패킷이 없으면 짐벌 명령을 만들지 않고
`status=uwb_unavailable`을 기록한다. 결과는 기본 재사용 버전과 섞이지 않게
`result/dynamic_tracking_no_packet_reuse/<experiment-id>/`에 저장한다.

## 결과 위치

두 프로그램 모두 같은 `experiment-id`를 사용하면 논리적으로 다음 구조에
저장된다. Rx와 Tx가 서로 다른 장치에서 실행되면 각 장치에 해당 CSV가
생성되므로 실험 후 한곳에 모을 수 있다.

```text
result/dynamic_tracking/
└── dynamic_20260726_01/
    ├── rx.csv
    ├── tx.csv
    └── failed_frames/
        └── rx_sample_000123_color_not_detected.jpg
```

## CSV 공통 필드

| 필드 | 의미 |
|---|---|
| `experiment_id` | Rx/Tx 공유 실험 ID |
| `node_id` | `rx` 또는 `tx` |
| `sample_index` | 공유 시작 후 0부터 증가하는 0.2초 주기 번호 |
| `distance_m` | 거리 조건. `1/2/3`은 m, `0`은 랜덤 이동 특수값 |
| `trajectory_mode` | `fixed_radius_arc` 또는 `random_moving_rover` |
| `alignment_period_s` | 고정 정렬 주기 0.2초 |
| `scheduled_elapsed_s` | 공유 시작 기준 해당 명령의 예정 경과시간 |
| `command_elapsed_s` | 공유 시작 기준 실제 짐벌 명령 경과시간 |
| `previous_gimbal_ros_deg` | 정렬 직전의 이전 짐벌 명령각, ROS 좌표계 |
| `uwb_source` | UWB UDP 송신 주소 |
| `uwb_received_monotonic_ns` | 로컬에서 UWB 패킷을 받은 monotonic 시각 |
| `uwb_packet_reused` | 새 패킷 사용은 `0`, 이전 패킷 재사용은 `1` |
| `uwb_raw_azimuth_deg` | UWB 원시 CW 상대 방위각 |
| `uwb_ros_azimuth_deg` | ROS CCW로 부호 변환한 UWB 상대각 |
| `correction_ros_deg` | deadband와 회당 60도 제한을 적용한 ROS 보정각 |
| `target_calculated_ros_deg` | 이전 짐벌각과 전체 UWB ROS 상대각으로 계산한 목표 |
| `gimbal_command_ros_deg` | 제한 후 실제 적용한 ROS 짐벌 명령각 |
| `servo_clipped` | 최종 요청이 서보 `-90~90도` 범위에서 제한됐는지 여부 |
| `status` | `success`, `uwb_unavailable`, 색상 실패 또는 `error` |
| `started_at`, `finished_at` | 해당 주기 처리 시작 및 종료 지역 시각 |
| `error_message` | 예외 진단 메시지 |

사용할 수 있는 UWB가 없는 주기도 CSV에 한 행을 남긴다. 기본 재사용 버전은
첫 유효 패킷을 받기 전, 비재사용 보존 버전은 새 미처리 패킷이 없는 주기가
이에 해당한다. 따라서 Rx와 Tx의 `sample_index`와 `scheduled_elapsed_s`를
기준으로 시간축을 비교할 수 있다.

## Rx 전용 CSV 필드

| 필드 | 의미 |
|---|---|
| `target_color` | 찾을 색상 |
| `color_visible` | 판정한 새 프레임에서 색상이 보였는지 여부. 판정하지 못하면 빈 값 |
| `color_success` | 해당 정렬 주기의 최종 색상 인식 성공 여부 |
| `color_recognition_time_ms` | 정렬 명령부터 색상 판정 성공까지 걸린 시간 |
| `camera_frame_id` | 판정 또는 실패 진단에 사용한 프레임 번호 |
| `camera_captured_ns` | 해당 프레임 캡처 시각, local monotonic ns |
| `failure_frame` | 실패 이미지의 실험 폴더 기준 상대 경로 |

`monotonic_ns` 값은 각 장치의 로컬 단조 증가 시계이므로 서로 다른 장치 간
절대시각으로 직접 비교하지 않는다. 장치 간 비교에는 공유
`sample_index`, `scheduled_elapsed_s`, `started_at`을 사용한다.
