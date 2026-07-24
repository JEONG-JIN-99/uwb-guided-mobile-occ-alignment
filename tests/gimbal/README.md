# 짐벌 하드웨어 테스트

이 폴더는 yaw 짐벌과 관련된 수동 하드웨어/통합 테스트 코드를 모아두는 곳입니다.
여기 있는 스크립트는 실제 하드웨어를 움직일 수 있으므로 프로젝트 루트에서 의도적으로 실행해야 합니다.

실제 짐벌 제어 구현은 `code/gimbal/gimbal_controller_yaw.py`에 있고, 이 폴더의 파일들은 검증/실험용입니다.

## `gimbal_range_test.py`

Yaw 짐벌을 아래 순서로 한 번 움직여 전체 가동 범위를 확인합니다.

```text
0도 -> +90도 -> -90도 -> 0도
```

각 위치에서 기본 3초 동안 대기하며, 마지막 0도에 도착하면 PCA9685 서보
제어 신호를 비활성화하고 종료합니다.

```bash
python tests/gimbal/gimbal_range_test.py
```

PCA9685 채널, I²C 주소와 각 위치의 대기 시간을 변경할 수 있습니다.

```bash
python tests/gimbal/gimbal_range_test.py --servo-channel 0 --pca9685-address 0x40 --wait 5
```

## `gimbal_uwb_tracking_test.py`

QR 인식 없이 UWB UDP 패킷만 사용해 yaw 짐벌을 구동하는 빠른 추적
테스트 코드입니다. 0.2초마다 소켓에 쌓인 패킷을 비우고 가장 최신 UWB 값
하나만 짐벌 보정에 사용합니다.

```bash
python tests/gimbal/gimbal_uwb_tracking_test.py
```

주요 옵션:

```bash
python tests/gimbal/gimbal_uwb_tracking_test.py \
  --host 0.0.0.0 \
  --port 5005 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --initial-deg 0
```

## `gimbal_uwb_tracking_qr_test.py`

0.2초 주기의 UWB 추적과 QR 인식을 별도 스레드에서 병렬로 수행하고, QR
결과와 실패 프레임을 저장하는 테스트 코드입니다.

패킷 형식:

```text
1,distance,azimuth,elevation,...
```

`azimuth` 값은 UWB가 측정한 상대각으로 취급합니다. 컨트롤러는 이 값을 이전 짐벌 명령각에 더해 다음 절대 짐벌 명령각으로 변환합니다.

실행:

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py
```

기본 실행은 `/dev/video4` 카메라를 함께 열고, 유효한 UWB 패킷으로 짐벌을
보정할 때마다 QR을 검사합니다. QR 결과는 실행별 CSV 파일에 저장됩니다.

```bash
result/gimbal_uwb_tracking_qr_test/run_YYYYMMDD_HHMMSS/qr_results.csv
```

CSV에는 QR 패턴 인식 여부(`qr_visible`), QR 데이터 디코딩 성공 여부
(`qr_decoded`), 디코딩된 내용(`qr_data`), 실패 이미지 경로
(`failure_frame`)를 기록합니다.

실제 카메라 영상을 함께 확인하려면 `--live-stream`을 사용합니다. 라이브
화면에는 중심 거리, UWB 상대각, 짐벌 명령각을 표시하지 않습니다.

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py --live-stream
```

주요 옵션:

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py \
  --port 5005 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --initial-deg 0 \
  --qr-device-index 4 \
  --qr-crop-scale 0.3
```

저장 상위 경로는 `--output-dir`로 바꿀 수 있습니다.

```bash
python tests/gimbal/gimbal_uwb_tracking_qr_test.py \
  --output-dir result/my_qr_tracking
```

UWB 상대각의 절댓값이 코드의
`UWB_DEADBAND_DEG`보다 작으면 보정하지 않고, 한 패킷에서 적용하는 보정각은
최대 ±60도로 제한합니다.
짐벌 제어는 QR 결과를 기다리지 않고 0.2초마다 실행되며, 매 제어 시점에
소켓 버퍼에서 가장 최신 UWB 패킷 하나만 사용합니다. 별도 QR 스레드는 짐벌
보정 직후부터 다음 보정 시각 전까지 도착한 새 카메라 프레임을 검사합니다.
QR 디코딩에 실패하면 해당 프레임을 실행 폴더의 `failed_frames/`에 저장하고,
`qr_results.csv`의 `failure_frame` 열에 이미지의 상대 경로를 함께 기록합니다.

## `send_fake_uwb_sweep.py`

가짜 UWB 상대각 패킷을 `gimbal_uwb_tracking_test.py`로 보내는 테스트 송신 코드입니다.
실제 UWB 태그를 움직이지 않고 UDP 수신 경로와 짐벌 추적 루프를 확인할 때 사용합니다.

한 터미널에서 짐벌 추적 테스트를 실행합니다.

```bash
python tests/gimbal/gimbal_uwb_tracking_test.py
```

다른 터미널에서 가짜 UWB 송신 코드를 실행합니다.

```bash
python tests/gimbal/send_fake_uwb_sweep.py
```

기본 패턴은 짐벌 명령각이 -80도에서 +80도 사이를 왕복하도록 상대각을 보냅니다.

```text
+10, +10, ... 이후 -10, -10, ...
```

주요 옵션:

```bash
python tests/gimbal/send_fake_uwb_sweep.py --step 10 --start -80 --stop 80 --interval 0.7
python tests/gimbal/send_fake_uwb_sweep.py --host 127.0.0.1 --port 5005
```

## `test_gimbal_controller.py`

`GimbalController`와 레거시 `GimbalStepController`의 단위 테스트입니다.
ServoKit은 mock으로 대체하므로 실제 모터를 움직이지 않습니다.

실행:

```bash
python -m unittest tests/gimbal/test_gimbal_controller.py
```

## `step_controller.py`

이전 step 방식의 짐벌 제어 실험 코드입니다.
현재 주 제어 경로는 `GimbalController.move_to()`와 `GimbalController.move_by_uwb_relative()`이며, 이 파일은 비교/레거시 테스트 용도로 보관합니다.

직접 실행하면 실제 PCA9685를 사용하므로 I²C와 전원 연결 상태를 확인한 뒤 실행해야 합니다.

```bash
python tests/gimbal/step_controller.py
```

## `dir_init.py`

서보를 중앙 방향(ServoKit 90도, 상대각 0도)으로 정렬하고 3초 동안 안정화한 뒤
제어 신호를 끕니다. 기본 실행은 화면 없는 환경을 위한 정렬 전용 모드입니다.
`--live-stream`을 사용하면 QR 수동 배치를 위해 전체 영상의 중앙 30% 영역만
잘라서 계속 표시합니다. 빨간 십자선은 잘린 영상의 중심입니다.
`q`, `Esc`, `Ctrl+C` 또는 창 닫기로 종료할 수 있으며 종료 시 카메라, 창과
PCA9685 서보 제어 신호를 정리합니다.

실행:

```bash
python tests/gimbal/dir_init.py
```

카메라 화면 표시:

```bash
python tests/gimbal/dir_init.py --live-stream
```

## `alignment__duration_test.py`

PWM을 얼마 동안 유지해야 짐벌이 목표각에 도착하는지 수동으로 측정합니다.
다음 네 구간을 순서대로 시험합니다.

```text
-90도 -> 0도
+90도 -> 0도
-90도 -> +90도
+90도 -> -90도
```

각 회차에서 출발각에 도착한 것을 확인한 뒤 Enter를 누르면 목표각 이동과 시간
측정이 동시에 시작됩니다. 목표각에 도착했다고 판단한 순간 다시 Enter를 누르면
측정을 종료하고 즉시 PWM 신호를 끕니다. 터미널에도 측정 시간이 초 단위로
출력됩니다.

```bash
python tests/gimbal/alignment__duration_test.py
```
