# 짐벌 하드웨어 테스트

이 폴더는 yaw 짐벌과 관련된 수동 하드웨어/통합 테스트 코드를 모아두는 곳입니다.
여기 있는 스크립트는 실제 하드웨어를 움직일 수 있으므로 프로젝트 루트에서 의도적으로 실행해야 합니다.

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
python tests/gimbal/gimbal_uwb_tracking_test.py --no-wait
```

기본 동작에서는 명령을 한 번 내린 뒤 서보가 이동할 시간을 고려해 약 0.6초 대기합니다.
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
