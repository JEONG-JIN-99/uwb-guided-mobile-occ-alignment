# Static Alignment Test

`static_alignment_test.py`는 짐벌을 무작위 초기각으로 이동한 뒤 UWB 방위각으로
송신기 방향을 추정하고, 정렬과 안정화가 끝난 이후 QR 인식률을 측정하는
하드웨어 실험 코드다.

## 동작 알고리즘

각 시도는 다음 순서로 진행된다.

1. PCA9685 서보 드라이버, UWB UDP 수신기와 RealSense 카메라를 초기화한다.
2. 짐벌을 상대각 `0도`로 이동하고 `--zero-settle-time` 동안 기다린다.
3. `--initial-min`과 `--initial-max` 사이에서 무작위 초기각을 선택한다.
4. 짐벌을 선택한 초기각으로 이동하고 `--settle-time` 동안 기다린다.
5. 짐벌 이동 전에 수신된 오래된 UWB 패킷을 모두 버린다.
6. `--uwb-timeout` 동안 새로운 유효 UWB 패킷을 기다린다.
7. 다음 공식으로 송신기 방향과 짐벌 명령각을 계산한다.

   ```text
   요청각 = 초기 짐벌각 + UWB 상대 방위각
   명령각 = 요청각을 -90도 이상 +90도 이하로 제한한 값
   ```

8. 계산된 명령각으로 짐벌을 이동하고 `--alignment-settle-time` 동안 기다린다.
9. 안정화가 끝난 시점부터 `--interval` 동안 QR을 탐지한다.
10. QR 인식 결과를 CSV에 기록한다. 인식에 실패하면 마지막 프레임도 저장한다.
11. 짐벌을 다시 `0도`로 이동하고 다음 시도를 반복한다.
12. 모든 시도가 끝나면 짐벌, UWB 소켓과 카메라 자원을 정리하고 인식률을 출력한다.

UWB 패킷은 다음 형식을 사용한다.

```text
1,distance,azimuth,elevation
```

예:

```text
1,2.5,-31.2,0.4
```

## 기본 실행

`--distance`는 필수 인자다.

```bash
python code/experiment/static_alignment_test.py --distance 2
```

기본 설정은 다음과 같다.

| 설정 | 기본값 |
|---|---:|
| 시도 횟수 | 100 |
| 무작위 초기각 | -50도 ~ +50도 |
| QR 탐지 시간 | 1초 |
| 초기각 안정화 시간 | 2초 |
| 0도 복귀 안정화 시간 | 2초 |
| UWB 정렬 안정화 시간 | 2초 |
| 서보 제어 신호 유지 시간 | 1초 |
| PCA9685 주소 | `0x40` |
| 서보 채널 | 0 |
| UWB UDP 포트 | 5005 |

## 전체 옵션을 지정한 실행 예시

```bash
python code/experiment/static_alignment_test.py \
  --distance 2 \
  --attempts 100 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --device-index 4 \
  --crop-scale 0.3 \
  --initial-min -50 \
  --initial-max 50 \
  --uwb-host 0.0.0.0 \
  --uwb-port 5005 \
  --uwb-timeout 1 \
  --interval 1 \
  --settle-time 2 \
  --zero-settle-time 2 \
  --alignment-settle-time 2 \
  --servo-drive-time 1
```

## 실시간 영상 표시

```bash
python code/experiment/static_alignment_test.py \
  --distance 2 \
  --live-stream
```

영상 창에서 `q`를 누르거나 터미널에서 `Ctrl+C`를 누르면 실험을 중단할 수 있다.

## 서보 제어 신호 설정

기본 동작에서는 각 이동 후 `--servo-drive-time` 동안만 서보 제어 신호를 유지한다.
그 이후에는 유지 토크와 지터 비교를 위해 제어 신호를 비활성화한다.

```bash
python code/experiment/static_alignment_test.py \
  --distance 2 \
  --servo-drive-time 1
```

정렬 후에도 제어 신호와 유지 토크를 계속 활성화하려면 다음 옵션을 사용한다.

```bash
python code/experiment/static_alignment_test.py \
  --distance 2 \
  --keep-pwm-active
```

## 결과 저장

기본 결과 경로는 `result/dynamic_qr/`다. 다른 위치를 사용하려면
`--output-dir`을 지정한다.

```bash
python code/experiment/static_alignment_test.py \
  --distance 2 \
  --output-dir result/static_alignment
```

실행 시각마다 다음 구조의 폴더가 생성된다.

```text
result/static_alignment/
└── run_20260724_153000/
    ├── dynamic_qr_results.csv
    └── failed_frames/
```

CSV에는 다음 항목이 저장된다.

- 실험 거리
- QR 탐지 시간
- 초기 짐벌각
- UWB 원시 방위각
- 최종 짐벌 명령각
- QR 가시 여부
- QR 디코딩 성공 여부

## 주요 옵션

| 옵션 | 설명 |
|---|---|
| `--distance` | 수동으로 측정한 실험 거리(m), 필수 |
| `--attempts` | 전체 반복 횟수 |
| `--initial-min`, `--initial-max` | 무작위 초기각 범위 |
| `--interval` | 안정화 후 QR을 탐지하는 시간 |
| `--device-index` | RealSense 카메라 장치 번호 |
| `--crop-scale` | QR 탐지에 사용할 중앙 영상 비율 |
| `--servo-channel` | PCA9685 yaw 서보 채널 |
| `--pca9685-address` | PCA9685 I2C 주소 |
| `--uwb-host`, `--uwb-port` | UWB UDP 수신 주소 |
| `--uwb-timeout` | 유효한 UWB 패킷을 기다리는 최대 시간 |
| `--random-seed` | 무작위 초기각 순서를 재현하기 위한 시드 |
| `--live-stream` | 실시간 카메라 영상 표시 |
| `--output-dir` | 결과 저장 상위 경로 |
