# Gimbal UWB Tracking Color Test

`gimbal_uwb_tracking_color_test.py`는 0.2초마다 최신 UWB 패킷으로 yaw 짐벌을
정렬하고, 각 정렬 직후 최신 카메라 프레임에서 선택한 색상 원을 탐지한다.

## 실행

```bash
python tests/gimbal/gimbal_uwb_tracking_color_test.py
```

실시간 영상을 함께 표시하려면:

```bash
python tests/gimbal/gimbal_uwb_tracking_color_test.py --live-stream
```

주요 설정을 지정한 예:

```bash
python tests/gimbal/gimbal_uwb_tracking_color_test.py \
  --host 0.0.0.0 \
  --port 5005 \
  --servo-channel 0 \
  --pca9685-address 0x40 \
  --device-index 4 \
  --crop-scale 0.3 \
  --target-color blue \
  --color-min-area 500 \
  --color-min-circularity 0.6
```

## 색상 원 판정

`--target-color`는 `red`, `orange`, `yellow`, `green`, `blue`, `purple` 중
하나를 선택한다. HSV 색 범위로 마스크를 만든 뒤 다음 조건을 만족하는 가장 큰
contour를 선택한 색상 원으로 판단한다.

- contour 면적이 `--color-min-area` 이상
- 원형도가 `--color-min-circularity` 이상

```text
원형도 = 4 × π × 면적 / 둘레²
```

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
- 색상 영역의 면적과 원형도
- 카메라 프레임 번호와 촬영 시각
- 색상 검출 처리 시간

논문 분석에서는 `color_visible`을 화면 진입 성공 여부로 사용하고,
`color_center_distance_px`를 정렬 오차 지표로 사용할 수 있다.
