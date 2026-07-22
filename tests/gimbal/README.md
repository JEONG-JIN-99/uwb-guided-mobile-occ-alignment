# 짐벌 하드웨어 테스트

이 폴더는 yaw 짐벌과 관련된 수동 하드웨어/통합 테스트 코드를 모아두는 곳입니다.
여기 있는 스크립트는 실제 하드웨어를 움직일 수 있으므로 프로젝트 루트에서 의도적으로 실행해야 합니다.

실제 짐벌 제어 구현은 `code/gimbal/gimbal_controller_yaw.py`에 있고, 이 폴더의 파일들은 검증/실험용입니다.

## `gimbal_uwb_tracking_test.py`

UWB UDP 패킷을 수신하고 `GimbalController`를 사용해 yaw 짐벌을 구동하는 테스트 코드입니다.

패킷 형식:

```text
1,distance,azimuth,elevation,...
```

`azimuth` 값은 UWB가 측정한 상대각으로 취급합니다. 컨트롤러는 이 값을 이전 짐벌 명령각에 더해 다음 절대 짐벌 명령각으로 변환합니다.

실행:

```bash
python tests/gimbal/gimbal_uwb_tracking_test.py
```

주요 옵션:

```bash
python tests/gimbal/gimbal_uwb_tracking_test.py --port 5005 --yaw-pin 18 --initial-deg 0
```

기본 동작은 패킷 처리 후 대기하지 않습니다. UWB 상대각의 절댓값이 코드의
`UWB_DEADBAND_DEG`보다 작으면 보정하지 않고, 한 패킷에서 적용하는 보정각은
최대 ±60도로 제한합니다.
대기 중 들어온 오래된 UWB 패킷은 따라가지 않고, 다음 제어 시점에 가장 최신 패킷만 처리합니다.

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
GPIO는 mock으로 대체하므로 실제 모터를 움직이지 않습니다.

실행:

```bash
python -m unittest tests/gimbal/test_gimbal_controller.py
```

## `step_controller.py`

이전 step 방식의 짐벌 제어 실험 코드입니다.
현재 주 제어 경로는 `GimbalController.move_to()`와 `GimbalController.move_by_uwb_relative()`이며, 이 파일은 비교/레거시 테스트 용도로 보관합니다.

직접 실행하면 실제 GPIO/PWM을 사용하므로 하드웨어 연결 상태를 확인한 뒤 실행해야 합니다.

```bash
python tests/gimbal/step_controller.py
```

## `dir_init.py`

서보를 중앙 방향(PWM 7.5, 상대각 0도)으로 정렬한 뒤 PWM 신호를 끕니다.
기본 실행은 화면 없는 환경을 위한 정렬 전용 모드로 즉시 종료합니다.
`--live-stream`을 사용하면 QR 수동 배치를 위해 전체 영상의 중앙 30% 영역만
잘라서 계속 표시합니다. 빨간 십자선은 잘린 영상의 중심입니다.
`q`, `Esc`, `Ctrl+C` 또는 창 닫기로 종료할 수 있으며 종료 시 카메라, 창, PWM 및
GPIO 자원을 정리합니다.

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
