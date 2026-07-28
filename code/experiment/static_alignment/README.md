# Static Alignment Color Test

`static_alignment_test.py`는 짐벌을 무작위 초기각으로 이동한 뒤 UWB 상대
방위각으로 송신기 방향을 계산하고, 정렬 명령 시점부터 제한시간 안에 선택한
색상이 인식되는지 측정하는 하드웨어 실험 코드다. QR은 사용하지 않는다.

`static_alignment_after_settle_test.py`는 같은 정렬을 수행하되, 정렬 명령 후
기본 0.5초 동안 짐벌을 안정화한 다음 새로 들어오는 카메라 프레임을 기본
0.2초 동안 검사하는 별도 실험 코드다.

## 실험 전 준비 순서

정적 정렬 본 실험 전에는 다음 순서로 준비한다.

1. `dir_init.py`로 짐벌 0도와 카메라 중앙을 맞춘다.
2. `red_detection_precheck.py`로 같은 카메라 조건에서 빨간색 인식 성능을
   100회 확인한다.
3. `static_alignment_test.py`로 정적 정렬 본 실험을 실행한다.

정렬 직후가 아니라 짐벌 안정화가 끝난 뒤의 인식률을 측정하려면 3단계에서
`static_alignment_after_settle_test.py`를 실행한다.

처음 실행하는 장치에서는 프로젝트 의존성을 먼저 설치한다.

```bash
python -m pip install -r requirements.txt
```

### `dir_init.py`: 짐벌·카메라 중앙 맞춤

이 스크립트는 색상 인식 성능을 측정하는 코드가 아니다. 본 실험 전에 Rx
짐벌을 ROS yaw 0도로 이동하고, 카메라의 영상 중심과 실험 표식의 물리적
중심을 맞추기 위한 수동 준비 도구다.

```bash
python code/experiment/static_alignment/dir_init.py \
  --device-index 4 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --crop-scale 0.6 \
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

주요 옵션:

| 옵션 | 기본값 | 의미 |
|---|---:|---|
| `--device-index` | 4 | 카메라 `/dev/videoX` 번호 |
| `--servo-channel` | 0 | PCA9685 yaw 서보 채널 |
| `--pca9685-address` | `0x40` | PCA9685 I2C 주소 |
| `--crop-scale` | 0.6 | 중앙 맞춤 화면에 표시할 중앙 영상 비율 |
| `--stabilization-time` | 3초 | 짐벌을 0도로 명령한 후 PWM을 끄기 전 대기시간 |
| `--live-stream` | 꺼짐 | 중앙 빨간 십자선이 있는 카메라 창 표시 |

`--live-stream`을 사용하면 `q`, `Esc`, `Ctrl+C` 또는 창 닫기로 종료할 수
있다. 종료할 때 카메라와 PCA9685 자원을 정리한다.

### `red_detection_precheck.py`: 빨간색 인식 100회 사전 검사

이 스크립트는 짐벌과 UWB를 사용하지 않고 카메라 빨간색 인식만 검사한다.
기본값은 정적 정렬 본 실험과 동일하다.

- 카메라: `/dev/video4`
- 중앙 크롭: 0.6
- 카메라 워밍업: 5초
- 회당 인식 제한시간: 0.2초
- 빨간색 최소 채도: 130
- 빨간색 최소 밝기: 110
- 전체 빨간 마스크 최소 면적: 125px
- 최대 연결 빨간 영역 최소 면적: 50px
- 검사 횟수: 100회

각 회차는 시작 전에 있던 프레임을 제외하고, 시작 후 0.2초 안에 도착한 새
프레임에서 빨간색이 한 번이라도 조건을 만족하면 성공이다. 100회가 끝나면
터미널에 `recognized=인식횟수/100 (성공률%)` 형식으로 출력한다.

기본 실행:

```bash
python code/experiment/static_alignment/red_detection_precheck.py \
  --distance 3
```

카메라 화면도 확인하려면:

```bash
python code/experiment/static_alignment/red_detection_precheck.py \
  --distance 3 \
  --live-stream
```

본 실험과 다른 옵션을 사용할 예정이면 사전 검사에도 같은 값을 지정한다.

```bash
python code/experiment/static_alignment/red_detection_precheck.py \
  --distance 3 \
  --attempts 100 \
  --device-index 4 \
  --crop-scale 0.6 \
  --interval 0.2 \
  --camera-warmup 5 \
  --color-min-area 125 \
  --color-min-component-area 50
```

각 회차 결과와 마지막 요약의 근거가 되는 값은 다음 CSV에 저장한다.

```text
result/red_detection_precheck/
└── run_YYYYMMDD_HHMMSS_distance_3m_crop_0.6/
    └── red_detection_results.csv
```

사전검사는 정렬 직후 인식 실험과 안정화 후 인식 실험에 공통으로 사용하는
카메라 검사이므로, 결과를 어느 한 정렬 방식의 폴더 아래에 넣지 않고
`result/red_detection_precheck/`에 독립적으로 저장한다.

`--distance`는 반드시 입력해야 하며 수동으로 측정한 카메라와 빨간 표식
사이의 거리(m)다. 거리와 `--crop-scale`은 CSV의 `distance_m`,
`crop_scale` 필드에 매 회차 기록되고 실행 폴더 이름에도 포함된다.

주요 CSV 필드는 회차 번호, 거리, 크롭 비율, 빨간색 인식 여부, 빨간 픽셀
개수와 화면 대비 비율, 대표 색상값, 최대 연결 영역 면적, 색상 중심과 화면
중심 사이 거리, 프레임 번호, 인식 시간 및 상태다.
`status`는 `success`, `color_not_detected`, `camera_frame_timeout` 중 하나다.
`--live-stream` 사용 중 `q`를 누르거나 `Ctrl+C`로 중단하면 완료된 회차만
분모로 사용해 인식률을 출력한다.

빨간 픽셀 관련 필드:

| 필드 | 의미 |
|---|---|
| `red_saturation_min`, `red_value_min` | 실행에 사용한 빨간색 채도·밝기 하한 |
| `red_pixel_count` | 본 실험과 같은 HSV 범위와 morphology를 통과한 빨간 픽셀 수 |
| `red_pixel_ratio_pct` | 전체 크롭·확대 영상 중 빨간 픽셀이 차지하는 비율(%) |
| `red_hue_circular_mean_deg` | 0도 경계를 고려한 빨간 픽셀 평균 Hue(0~360도) |
| `red_saturation_mean` | 빨간 픽셀의 평균 채도(0~255) |
| `red_value_mean` | 빨간 픽셀의 평균 밝기(0~255) |
| `red_b_mean`, `red_g_mean`, `red_r_mean` | 빨간 픽셀의 평균 BGR 값(0~255) |

예를 들어 `red_pixel_count`와 `red_pixel_ratio_pct`는 표식이 영상에서 얼마나
크게 잡히는지, 평균 채도와 밝기는 조명 아래에서 표식이 얼마나 선명하게
보이는지 판단하는 지표로 사용할 수 있다. Hue는 빨간색이 0도와 360도 경계에
걸치므로 일반 산술평균이 아니라 원형평균으로 기록한다.

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

## 필수 옵션

`static_alignment_test.py`에서 반드시 입력해야 하는 옵션은
`--distance` 하나다. 나머지 옵션은 모두 기본값이 있다.

| 옵션 | 입력할 값 | 의미 |
|---|---|---|
| `--distance` | `0`보다 큰 실수 또는 정수 | 수동으로 측정한 송신기와 수신기 사이의 실험 거리(m) |

예를 들어 실험 거리가 2m라면 다음이 최소 실행 명령이다.

```bash
python code/experiment/static_alignment/static_alignment_test.py --distance 2
```

`--distance 0`이나 음수는 허용되지 않는다. 이 값은 UWB 패킷 안의 측정
거리와 별개로, 실험 조건을 CSV에 기록하기 위해 사용자가 직접 입력하는
거리다.

카메라 번호, PCA9685 채널·주소 또는 UWB 수신 주소가 기본 구성과 다르면
필수 옵션은 아니더라도 실제 장치에 맞춰 `--device-index`,
`--servo-channel`, `--pca9685-address`, `--uwb-host`, `--uwb-port`도
지정해야 한다.

## 안정화 후 0.2초 인식 실험

정렬이 끝난 뒤 표식이 실제로 보이는지를 별도로 측정하려면 다음 명령을
사용한다.

```bash
python code/experiment/static_alignment/static_alignment_after_settle_test.py \
  --distance 2
```

이 파일의 기본 시간 순서는 다음과 같다.

1. UWB로 계산한 목표각을 짐벌에 명령한다.
2. `--pre-recognition-settle-time`의 기본값인 0.5초를 기다린다.
3. 대기 전에 들어온 프레임을 기준 프레임으로 제외한다.
4. 이후 새로 캡처된 프레임만 `--interval`의 기본값인 0.2초 동안 검사한다.

따라서 이 실험의 0.2초는 정렬 명령 시점부터가 아니라 0.5초 안정화가 끝난
시점부터 측정된다. 안정화 시간을 바꾸려면 다음처럼 지정한다.

```bash
python code/experiment/static_alignment/static_alignment_after_settle_test.py \
  --distance 2 \
  --pre-recognition-settle-time 0.7 \
  --alignment-settle-time 1.0 \
  --interval 0.2
```

`--alignment-settle-time`은 정렬 명령부터 다음 단계로 넘어가기 전까지의 전체
시간이므로 `--pre-recognition-settle-time + --interval` 이상이어야 한다.
이 실험의 기본 결과는 기존 정렬 직후 실험과 섞이지 않도록
`result/static_alignment_after_settle/`에 저장된다.

## 기본 설정

| 설정 | 옵션 | 기본값 |
|---|---|---:|
| 시도 횟수 | `--attempts` | 100 |
| 무작위 초기각 | `--initial-min`, `--initial-max` | -50도 ~ +50도 |
| 색상 인식 제한시간 | `--interval` | 0.2초 |
| 인식 전 정렬 안정화 | `--pre-recognition-settle-time` | 0초 (`after_settle` 실험은 0.5초) |
| 카메라 색상 안정화 | `--camera-warmup` | 5초 |
| 0도 복귀 안정화 | `--zero-settle-time` | 1초 |
| 초기각 이동 안정화 | `--initial-settle-time` | 1초 |
| 정렬 명령 후 전체 안정화 | `--alignment-settle-time` | 1초 |
| 영상 크롭 비율 | `--crop-scale` | 0.6 |
| 인식 색상 | `--target-color` | red |
| 전체 색상 마스크 최소 면적 | `--color-min-area` | 125px |
| 최대 연결 색상 영역 최소 면적 | `--color-min-component-area` | 50px |
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
  전체 안정화 시간이다. 인식 전 안정화와 색상 인식 구간을 포함하므로
  `--pre-recognition-settle-time + --interval`보다 작게 설정할 수 없다.
- `--pre-recognition-settle-time`: 정렬 명령 후 색상 인식을 시작하기 전에
  기다리는 시간이다. 기존 `static_alignment_test.py`는 0초,
  `static_alignment_after_settle_test.py`는 0.5초가 기본값이다.

`--warmup`은 `--camera-warmup`의 이전 이름으로 계속 사용할 수 있다.

## 색상 판정

지원 색상은 `red`, `orange`, `yellow`, `green`, `blue`, `purple`이다.
기본값은 `red`다.

```bash
python code/experiment/static_alignment/static_alignment_test.py \
  --distance 2 \
  --target-color red \
  --color-min-area 125 \
  --color-min-component-area 50
```

빨간색은 Hue 0~7 또는 173~180, Saturation 130 이상, Value 110 이상을
사용한다. 전체 마스크 125px 이상과 최대 연결 영역 50px 이상을 동시에
만족해야 인식 성공이다.

인식 구간 시작 직전의 프레임 ID를 기준으로 잡고, 인식 시작 시각 이후에
캡처된 새 프레임만 검사한다.

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
| `interval_s` | 안정화 후 색상을 기다리는 인식 구간 길이 |
| `pre_recognition_settle_time_s` | 정렬 명령 후 색상 인식 시작 전 안정화 시간 |
| `crop_scale` | 실행에 사용한 중앙 크롭 비율 |
| `color_min_area_px` | 전체 색상 마스크 최소 면적 설정 |
| `color_min_component_area_px` | 최대 연결 색상 영역 최소 면적 설정 |
| `red_saturation_min`, `red_value_min` | 빨간색 HSV 채도·밝기 하한 |
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
| `color_recognition_time_ms` | 인식 구간 시작부터 색상 판정 성공까지 걸린 시간 |
| `red_pixel_count` | 실제 판정 마스크의 빨간 픽셀 수 |
| `red_pixel_ratio_pct` | 처리 영상에서 빨간 마스크가 차지하는 비율 |
| `red_saturation_mean`, `red_value_mean` | 마스크 픽셀의 평균 채도·밝기 |
| `color_component_area_px` | 가장 큰 연결 색상 영역의 면적 |
| `color_center_distance_px` | 가장 큰 색상 영역 중심과 화면 중심 사이 거리 |
| `camera_frame_id` | 판정에 사용한 프레임 번호 |
| `camera_captured_ns` | 판정 프레임 캡처 시각, monotonic ns |
| `failure_frame` | 실패 이미지의 실행 폴더 기준 상대 경로 |
| `status` | 성공 또는 실패 원인 |
| `started_at`, `finished_at` | 시도 시작 및 종료 시각 |
| `error_message` | 예외 진단 메시지 |

## 소프트웨어 정렬오차 분석

Tx UWB가 실제 ROS yaw `0도` 방향에 고정된 실험에서는 실제 적용된
`gimbal_command_ros_deg`와 `0도`의 차이를 소프트웨어 정렬오차로 볼 수 있다.
특정 실행 결과는 다음처럼 분석한다.

```bash
python code/experiment/static_alignment/analyze_alignment_error.py \
  "result/static_alignment/run_YYYYMMDD_HHMMSS"
```

Tx 기준 방향이 0도가 아니라면 `--reference-angle`로 지정한다. 출력은 원본
결과와 같은 실행 폴더에 저장된다.

```text
alignment_error_results.csv   # 시도별 부호 오차와 절대 오차
alignment_error_summary.json  # 전체 및 상태별 MAE, RMSE 등의 통계
alignment_error_plot.png      # 시도별 부호 오차와 절대 오차 분포
```

부호 오차는 `짐벌 명령각 - Tx 기준각`의 최소 각도차이며, 절대 정렬오차는
그 값의 절댓값이다. 이는 엔코더나 영상 기반으로 측정한 실제 기계 각도가
아니므로 UWB 측정, 좌표 변환 및 명령 계산을 포함한 소프트웨어 오차 지표다.

### 1m·2m·3m 비교 그림

현재 논문용 1m·2m·3m 결과의 안정화 유무를 비교하는 세 그림은 다음
명령으로 재생성한다.

```bash
python code/experiment/static_alignment/plot_alignment_accuracy.py
```

정렬오차는 이상적인 목표 0도와 실제 짐벌 명령각 차이의 절댓값으로
계산한다.

```text
absolute residual command error
    = abs(gimbal_command_ros_deg - 0도)
```

생성되는 그림은 다음과 같다.

```text
docs/paper/figures/alignment_accuracy_initial_angle_scatter.png
docs/paper/figures/alignment_accuracy_initial_angle_mean.png
docs/paper/figures/alignment_accuracy_distance_violin.png
```

산점도와 초기각 구간별 평균오차 그래프는 거리를 1m·2m·3m 패널로
분리한다. 평균오차 그래프의 점은 산술평균이고 오차막대는 근사 95%
신뢰구간이다. 거리별 바이올린 플롯의 흰 테두리 점도 각 분포의 평균
절대오차(MAE)를 나타낸다. 모든 그림에서 안정화 없음과 0.5초 안정화
결과를 서로 다른 색으로 표시하며 합치지 않는다.

## 전체 옵션 예시

```bash
python code/experiment/static_alignment/static_alignment_test.py \
  --distance 2 \
  --attempts 100 \
  --device-index 4 \
  --crop-scale 0.6 \
  --camera-warmup 5 \
  --initial-min -50 \
  --initial-max 50 \
  --zero-settle-time 1 \
  --initial-settle-time 1 \
  --pre-recognition-settle-time 0 \
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
