# Gimbal UWB Tracking Color Test

`gimbal_uwb_tracking_color_test.py`는 0.2초마다 최신 UWB 패킷으로 yaw 짐벌을
정렬하고, 다음 정렬 시각까지 들어오는 새 카메라 프레임에서 선택한 색상의
존재 여부를 탐지한다.
기본 100회의 정렬·색상 판정을 마치면 자동으로 종료한다.

## 실행

```bash
python tests/gimbal/gimbal_uwb_tracking_color_test.py
```

실시간 영상을 함께 표시하려면:

```bash
python tests/gimbal/gimbal_uwb_tracking_color_test.py --live-stream
```

색상을 찾지 못한 프레임은 기본적으로 실행 폴더의 `failed_frames/`에 저장된다.
저장이 필요하지 않으면 `--no-save-failure-frames`를 지정한다.

주요 설정을 지정한 예:

```bash
python tests/gimbal/gimbal_uwb_tracking_color_test.py \
  --host 0.0.0.0 \
  --port 5005 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --device-index 4 \
  --crop-scale 1.0 \
  --attempts 100 \
  --target-color blue \
  --color-min-area 500 \
  --color-min-component-area 200
```

카메라 자동 노출과 화이트밸런스가 안정되도록 기본 5초 동안 기다린 뒤 측정을
시작한다. 필요하면 `--camera-warmup`으로 변경할 수 있다.

## 색상 존재 판정

`--target-color`는 `red`, `orange`, `yellow`, `green`, `blue`, `purple` 중
하나를 선택한다. 기본값 `--crop-scale 1.0`은 crop 없이 전체 프레임을
검사한다. HSV 색 범위로 마스크를 만든 뒤 다음 조건을 모두 만족하면 선택한
색상이 보인 것으로 판단한다.

- 전체 색상 마스크가 `--color-min-area` 이상(기본 500px)
- 가장 큰 연결 영역이 `--color-min-component-area` 이상(기본 200px)

원형도는 성공 조건으로 사용하지 않는다. 따라서 표식이 움직여 번지거나
프레임 가장자리에서 일부가 잘려도 충분한 색상 면적이 남으면 성공이다.
각 UWB 보정 후 최대 0.2초 동안 새 프레임을 계속 검사하며, 한 프레임에서라도
조건을 만족하면 해당 시도를 성공으로 기록한다.

## 결과

```text
result/gimbal_uwb_tracking_color_test/run_YYYYMMDD_HHMMSS/color_results.csv
```

CSV에는 정렬마다 다음 정보가 저장된다.

- UWB 거리와 상대 방위각
- 적용한 보정각과 짐벌 명령각
- 선택한 색상과 원 검출 여부
- 색상 원 중심 좌표
- 화면 중심과 색상 원 중심 사이 픽셀 거리
- 전체 색상 마스크 면적, 최대 연결 영역 면적과 진단용 원형도
- 카메라 프레임 번호와 촬영 시각
- 색상 검출 처리 시간
- 색상 미검출 프레임의 실행 폴더 기준 상대 경로

논문 분석에서는 `color_visible`을 화면 진입 성공 여부로 사용하고,
`color_center_distance_px`를 정렬 오차 지표로 사용할 수 있다.
