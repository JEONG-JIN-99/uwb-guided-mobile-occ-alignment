# Static Alignment Color Test

`static_alignment_test.py`는 짐벌을 무작위 초기각으로 이동한 뒤 UWB 상대
방위각으로 송신기 방향을 계산하고, 정렬 명령 시점부터 제한시간 안에 선택한
색상이 인식되는지 측정하는 하드웨어 실험 코드다. QR은 사용하지 않는다.

## 실험 전 짐벌·카메라 중앙 정렬

정적 정렬 실험을 시작하기 전에 다음 준비 스크립트를 실행한다.
처음 실행하는 장치에서는 프로젝트 의존성을 먼저 설치해야 한다.

```bash
python -m pip install -r requirements.txt
```

```bash
python code/experiment/static_alignment/dir_init.py \
  --device-index 4 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --crop-scale 1.0 \
  --live-stream
```

스크립트는 짐벌을 ROS yaw 0도로 이동해 3초간 안정화한 뒤 PWM 신호를 끄고,
카메라 화면 중앙에 빨간 십자선을 표시한다. 십자선을 기준으로 카메라와 실험
표식의 중앙을 맞춘 다음 `q` 또는 `Esc`로 종료한다. `--device-index`,
`--servo-channel`, `--pca9685-address`, `--crop-scale`은 이어서 실행할
`static_alignment_test.py`와 같은 값을 사용해야 한다.

카메라 화면 없이 짐벌만 0도로 맞추려면 `--live-stream`을 생략한다. PWM을
끈 뒤에는 서보 유지 토크가 사라지므로 중앙 정렬 후 짐벌이 물리적으로
움직이지 않도록 주의한다.

## 동작 순서

각 시도는 다음 순서로 진행된다.

1. PCA9685 서보 드라이버, UWB UDP 수신기, RealSense 카메라를 초기화한다.
2. 카메라 자동 노출과 화이트 밸런스를 위해 `--camera-warmup` 동안 기다린다.
3. 짐벌을 ROS yaw `0도`로 이동하고 `--zero-settle-time` 동안 안정화한다.
4. `--initial-min`과 `--initial-max` 사이의 무작위 초기각으로 이동하고
   `--initial-settle-time` 동안 안정화한다.
5. 이전 UWB 패킷을 버리고 `--uwb-timeout` 안에 도착한 첫 유효 패킷을 받는다.
6. UWB 원시 CW 상대 방위각을 ROS CCW 상대각으로 바꾸고 목표각을 계산한다.

   ```text
   UWB ROS 상대각 = -UWB 원시 상대 방위각
   계산 목표각 = 초기 ROS 짐벌각 + UWB ROS 상대각
   짐벌 명령각 = 계산 목표각을 -90도 이상 +90도 이하로 제한한 값
   ```

7. 짐벌 정렬 명령을 보낸 순간부터 `--interval` 동안 도착하는 새 카메라
   프레임에서 색상을 찾는다. 기본 제한시간은 0.2초다.
8. 정렬 명령 후 총 `--alignment-settle-time`이 지날 때까지 나머지 시간을
   기다린다. 색상 인식에 사용된 시간도 이 안정화 시간에 포함된다.
9. 결과를 CSV에 기록한다. 실패하면 마지막 프레임을 `failed_frames/`에
   저장한다.
10. 짐벌을 0도로 돌리고 다음 시도를 진행한다.

실험 도중에는 PCA9685 PWM을 끄지 않는다. 프로그램 종료 시에만 하드웨어
자원 정리를 위해 짐벌 제어기를 정리한다.

UWB 패킷 형식은 `1,distance,azimuth,elevation`이다.

## 기본 실행

`--distance`는 수동으로 측정한 실험 거리(m)이며 필수다.

```bash
python code/experiment/static_alignment_test.py --distance 2
```

기본 설정:

| 설정 | 옵션 | 기본값 |
|---|---|---:|
| 시도 횟수 | `--attempts` | 100 |
| 무작위 초기각 | `--initial-min`, `--initial-max` | -50도 ~ +50도 |
| 색상 인식 제한시간 | `--interval` | 0.2초 |
| 카메라 색상 안정화 | `--camera-warmup` | 5초 |
| 0도 복귀 안정화 | `--zero-settle-time` | 1초 |
| 초기각 이동 안정화 | `--initial-settle-time` | 1초 |
| 정렬 명령 후 전체 안정화 | `--alignment-settle-time` | 1초 |
| 영상 크롭 비율 | `--crop-scale` | 1.0 |
| 인식 색상 | `--target-color` | red |
| PCA9685 주소 | `--pca9685-address` | `0x40` |
| 서보 채널 | `--servo-channel` | 0 |
| UWB UDP 포트 | `--uwb-port` | 5005 |

### 안정화 시간 옵션의 차이

- `--camera-warmup`: 실험 시작 시 카메라 자동 노출과 화이트 밸런스가 색을
  안정적으로 표현하도록 한 번만 기다리는 시간이다.
- `--zero-settle-time`: 매 시도에서 짐벌을 0도로 보낸 뒤 기다리는 시간이다.
- `--initial-settle-time`: 무작위 초기각으로 이동한 뒤 UWB를 받기 전에
  기다리는 시간이다. 이전 이름인 `--settle-time`도 호환된다.
- `--alignment-settle-time`: UWB 목표각으로 정렬 명령을 보낸 시점부터 세는
  전체 안정화 시간이다. 첫 0.2초의 색상 인식 구간을 포함하므로
  `--interval`보다 작게 설정할 수 없다.

`--warmup`은 `--camera-warmup`의 이전 이름으로 계속 사용할 수 있다.

## 색상 판정

지원 색상은 `red`, `orange`, `yellow`, `green`, `blue`, `purple`이다.
기본값은 `red`다.

```bash
python code/experiment/static_alignment_test.py \
  --distance 2 \
  --target-color red \
  --color-min-area 500 \
  --color-min-component-area 200
```

정렬 직전의 프레임 ID를 기준으로 잡고, 정렬 명령 시각 이후에 캡처된 새
프레임만 검사한다.

- `success`: 새 프레임에서 제한시간 안에 색상을 인식했다.
- `color_not_detected`: 새 프레임은 들어왔지만 색상을 인식하지 못했다.
- `camera_frame_timeout`: 제한시간 안에 새 프레임이 들어오지 않았다.
- `uwb_timeout`: 제한시간 안에 유효 UWB 패킷을 받지 못했다.
- `error`: 처리 중 예외가 발생했다.

`color_visible`은 실제 새 프레임을 검사했을 때만 `0` 또는 `1`로 기록한다.
새 프레임이 없거나 UWB 타임아웃으로 영상 판정 자체를 수행하지 못하면 빈
값으로 남긴다. `color_success`는 전체 시도의 최종 성공 여부이므로 이 경우에도
`0`이다.

색상 실패 프레임 저장은 기본으로 활성화된다. 필요할 때만
`--no-save-failure-frames`로 끌 수 있다.

## 결과 저장

기본 경로는 `result/static_alignment/`이다.

```text
result/static_alignment/
└── run_YYYYMMDD_HHMMSS/
    ├── static_alignment_results.csv
    └── failed_frames/
        └── attempt_001_color_not_detected.jpg
```

다른 상위 경로는 `--output-dir`로 지정한다.

CSV 필드:

| 필드 | 의미 |
|---|---|
| `attempt` | 시도 번호 |
| `distance_m` | 수동으로 지정한 실험 거리 |
| `interval_s` | 정렬 명령부터 색상을 기다리는 제한시간 |
| `initial_gimbal_ros_deg` | 초기 짐벌각, ROS CCW 좌표계 |
| `uwb_source` | UWB 송신 UDP 주소 |
| `uwb_raw_azimuth_deg` | UWB 원시 CW 상대 방위각 |
| `uwb_ros_azimuth_deg` | ROS CCW로 변환한 UWB 상대각 |
| `target_calculated_ros_deg` | 초기각과 UWB 상대각으로 계산한 목표각 |
| `gimbal_command_ros_deg` | 범위 제한 후 실제 적용한 짐벌 명령각 |
| `servo_clipped` | 목표각이 서보 범위 때문에 제한됐는지 여부 |
| `target_color` | 찾을 색상 |
| `color_visible` | 판정한 새 프레임에서 색상이 보였는지 여부. 판정하지 못하면 빈 값 |
| `color_success` | 해당 정렬 시도의 최종 색상 인식 성공 여부 |
| `color_recognition_time_ms` | 정렬 명령부터 색상 판정 성공까지 걸린 시간 |
| `camera_frame_id` | 판정에 사용한 프레임 번호 |
| `camera_captured_ns` | 판정 프레임 캡처 시각, monotonic ns |
| `failure_frame` | 실패 이미지의 실행 폴더 기준 상대 경로 |
| `status` | 성공 또는 실패 원인 |
| `started_at`, `finished_at` | 시도 시작 및 종료 시각 |
| `error_message` | 예외 진단 메시지 |

## 전체 옵션 예시

```bash
python code/experiment/static_alignment_test.py \
  --distance 2 \
  --attempts 100 \
  --device-index 4 \
  --crop-scale 1 \
  --camera-warmup 5 \
  --initial-min -50 \
  --initial-max 50 \
  --zero-settle-time 1 \
  --initial-settle-time 1 \
  --alignment-settle-time 1 \
  --interval 0.2 \
  --target-color red \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --uwb-host 0.0.0.0 \
  --uwb-port 5005 \
  --uwb-timeout 1 \
  --output-dir result/static_alignment
```

`--live-stream`을 추가하면 영상을 표시하며, 영상 창에서 `q` 또는 터미널에서
`Ctrl+C`로 중단할 수 있다.
