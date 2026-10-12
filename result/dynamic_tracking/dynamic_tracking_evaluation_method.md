# Dynamic Tracking 실험별 분석 작업 지침

## 1. 이 문서의 목적

이 문서는 Codex가 `result/dynamic_tracking/` 아래의 **각 실험 폴더를 서로
독립적으로 분석**하고, 동일한 형식의 논문용 표·그래프·설명 문서를 만들기
위한 실행 지침이다.

각 실험에서 만들 지표는 아래 두 가지뿐이다.

1. 빨간색 인식률 표
2. 시간에 따른 이상적 방향, Rx 정렬 방향 및 Tx 명령 방향을 겹쳐 표시한
   정렬 정확도 그래프

지표를 임의로 추가하거나 여러 실험을 하나의 표 또는 그래프로 합치지 않는다.
각 실험의 결과는 반드시 해당 실험 폴더 안에 저장한다.

---

## 2. 분석 대상 자동 탐색

분석 루트는 다음과 같다.

```text
result/dynamic_tracking/
```

Codex는 루트 바로 아래의 모든 하위 디렉터리를 검사한다. 한 실험을 분석하려면
그 폴더에 다음 세 로그가 모두 있어야 한다.

| 용도 | 파일 선택 규칙 | 필수 열 |
|---|---|---|
| 로버 | 파일명에 `arc_chrony_test_vel_log`가 포함된 CSV | `elapsed_sec`, `heading_deg`, `rx_to_tx_deg`, `tx_to_rx_deg` |
| Rx | `rx.csv` | `sample_index`, `command_elapsed_s`, `gimbal_command_ros_deg`, `target_color`, `color_success`, `status` |
| Tx | `rx.csv`가 아니며 파일명 끝이 `tx.csv`인 CSV | `sample_index`, `command_elapsed_s`, `gimbal_command_ros_deg`, `status` |

예를 들어 `2m02`의 Tx 로그 `2m02tx.csv`는 위 규칙으로 선택한다.

- 세 로그가 모두 있는 폴더만 완전한 분석 대상으로 삼는다.
- 후보 파일이 여러 개이면 자동으로 추측하지 말고 파일명과 수정시각을 확인해
  동일 실행의 파일을 선택한다. 확정할 수 없으면 그 폴더는 건너뛰고 이유를
  최종 작업 보고에 남긴다.
- `failed_frames/`, 기존 분석 산출물 디렉터리, CSV가 없는 디렉터리는 실험으로
  취급하지 않는다.
- 기존 원본 CSV와 실패 프레임은 수정하거나 삭제하지 않는다.

현재 구조에서는 일반적으로 `1m01`, `1m02`, `2m01`, `2m02`, `3m01`,
`3m02`가 분석 후보지만, 폴더명을 하드코딩하지 말고 위 규칙으로 다시 찾는다.

---

## 3. 공통 시간 범위: 로버 시간을 기준으로 자르기

모든 지표는 로버가 실제로 기록된 시간 동안의 데이터만 사용한다.

```text
t_start = 로버 로그의 첫 번째 유효 elapsed_sec
t_end   = 로버 로그의 마지막 유효 elapsed_sec
```

정렬 정확도 그래프에 사용할 Rx와 Tx 행은 다음 조건을 모두 만족해야 한다.

```text
sample_index >= 1
t_start <= command_elapsed_s <= t_end
gimbal_command_ros_deg가 유효한 숫자
```

추가 규칙은 다음과 같다.

- `sample_index=0` 초기화 행은 제외한다.
- 방향 그래프에서는 `command_elapsed_s`가 비어 있거나 숫자가 아니면 제외한다.
- 로버 종료 후에도 Rx 또는 Tx 로그가 계속 기록됐더라도 `t_end` 뒤의 행은
  절대 사용하지 않는다.
- Rx와 Tx 중 더 짧은 로그 길이에 맞춰 로버 데이터를 자르지 않는다. 항상
  로버의 `[t_start, t_end]`를 공통 기준으로 사용하고, 각 장치에서 그 범위에
  실제 존재하는 표본만 사용한다.
- 시간축은 `command_elapsed_s` 및 `elapsed_sec`을 그대로 초 단위로 사용한다.
  별도의 임의 시간 이동을 하지 않는다.

각 Rx 또는 Tx 명령 시각에 정확히 같은 로버 표본이 없으면, 그 시각을 둘러싼
두 로버 표본 사이에서 선형 보간한다. 각도 보간은 `-180°/180°` 경계를
고려해 최단 회전 방향으로 수행한다.

```text
wrap(a) = ((a + 180) mod 360) - 180
delta   = wrap(angle_after - angle_before)
angle(t) = wrap(angle_before + ratio * delta)
```

외삽은 금지한다. 명령 시각이 로버 시간 범위 밖이면 해당 행을 제외한다.

단, 빨간색 인식률 표에는 UWB 미수신이나 처리 오류로
`command_elapsed_s`가 비어 있는 주기도 시스템 실패로 포함해야 한다. 따라서
인식률 표에서 각 Rx 행의 평가 시각은 다음처럼 정한다.

```text
evaluation_elapsed_s
    = command_elapsed_s가 유효하면 command_elapsed_s
    = 그렇지 않으면 scheduled_elapsed_s
```

인식률 표에는 `sample_index >= 1`이고
`t_start <= evaluation_elapsed_s <= t_end`인 모든 Rx 주기를 사용한다.

---

## 4. 지표 1: 빨간색 인식률 표

### 4.1 사용할 행과 전체 분모

공통 시간 범위 안의 Rx 행 중 아래 조건을 만족하는 **모든 시스템 처리
주기**를 분모로 센다.

```text
target_color == "red"  (대소문자와 앞뒤 공백은 무시)
sample_index >= 1
t_start <= evaluation_elapsed_s <= t_end
```

`uwb_unavailable`, `camera_frame_timeout`, `error`처럼 영상 인식 단계까지
정상적으로 도달하지 못한 주기도 시스템이 가진 실패이므로 분모에 포함한다.
`color_success`가 비어 있어도 위 시간 조건을 만족하면 제외하지 않는다.
단, `sample_index=0`은 실험 시작 전 초기화 행이므로 분모에서 제외한다.

### 4.2 계산

```text
전체 평가 주기 N       = 위 조건을 만족하는 모든 Rx 행 수
인식 성공 S            = color_success == 1 또는 status == "success"
색상 미검출 C          = status == "color_not_detected"
카메라 프레임 타임아웃 F = status == "camera_frame_timeout"
UWB 미수신 U           = status == "uwb_unavailable"
기타 오류 E            = status == "error" 또는 위 분류에 속하지 않은 실패
시스템 전체 인식률 (%)  = (S / N) × 100
정상 처리 조건 인식률 (%) = S / (S + C) × 100
각 원인 비율 (%)        = (해당 범주의 건수 / N) × 100
```

모든 행은 위 범주 중 정확히 하나에만 들어가야 하며 다음 등식을 검증한다.

```text
N = S + C + F + U + E
```

`정상 처리 조건 인식률`의 분모 `S + C`는 UWB 수신과 카메라 프레임 획득이
정상적으로 끝나 실제 빨간색 판정까지 도달한 주기만 뜻한다. 따라서
`camera_frame_timeout`, `uwb_unavailable`, `error`는 이 조건부 인식률의
분모에서는 제외하되, 시스템 전체 인식률의 분모 `N`에는 그대로 포함한다.

`color_success=0`인데 알려진 실패 상태가 없는 행은 `기타 오류`로 분류한다.
`N=0`이면 0%로 쓰지 말고 `N/A`로 표시하고 이유를 설명한다. 성공과 실패
범주 셀은 `건수 (전체 대비 비율)` 형식으로 쓰고, 백분율은 소수점 첫째
자리까지 반올림한다. `S + C = 0`이면 정상 처리 조건 인식률을 `N/A`로
표시한다.

### 4.3 문서에 넣을 표

각 실험의 결과 문서에는 아래 형식의 표를 넣는다.

| 전체 주기 | 인식 성공 | 색상 미검출 | 카메라 타임아웃 | UWB 미수신 | 기타 오류 | 시스템 전체 인식률 | 정상 처리 조건 인식률 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| N | S (xx.x%) | C (xx.x%) | F (xx.x%) | U (xx.x%) | E (xx.x%) | xx.x% | xx.x% |

표 아래에는 사용한 로버 시간 범위, `sample_index=0` 제외 기준, 시스템 전체
인식률이 모든 실패 원인을 포함한다는 점과 정상 처리 조건 인식률의 분모가
`S + C`라는 점을 1~2문장으로 기록한다.

---

## 5. 지표 2: Rx·Tx 통합 정렬 정확도 그래프

### 5.1 비교할 값과 오차 계산

로버 로그의 `rx_to_tx_deg`는 각 시각에 Rx가 Tx를 바라봐야 하는 **이상적
절대 방향**이다.

```text
Rx ideal direction(t) = 보간한 rx_to_tx_deg(t)
```

Rx 짐벌 명령 0°의 광축은 로버 헤딩에서 ROS 양의 방향으로 90° 오프셋되어
있다. 따라서 UWB 보정 결과가 누적되어 실제로 적용된
`gimbal_command_ros_deg`를 사용해 **명령 기준 실제 정렬 방향**을 계산한다.

```text
Rx aligned direction(t)
    = wrap(
        보간한 heading_deg(t)
        + 90°
        + gimbal_command_ros_deg(t)
      )

Rx error(t)
    = wrap(Rx aligned direction(t) - Rx ideal direction(t))

Rx MAE
    = (1 / N_rx) × Σ abs(Rx error(t))
```

여기서 `gimbal_command_ros_deg`는 엔코더로 측정한 물리 각도가 아니라 실제로
적용한 명령각이다. 따라서 결과 설명에서는 “실제 측정 방향”이 아닌
**“UWB 기반 짐벌 명령으로 계산한 정렬 방향”**이라고 표현한다.
`N_rx`는 로버 시간 범위 안에서 이상적 방향과 실제 정렬 방향을 모두 계산할
수 있는 유효 Rx 명령 수다.

로버 로그의 `tx_to_rx_deg`는 각 시각에 Tx가 Rx를 바라봐야 하는 이상적
방향값이다.

```text
Tx ideal direction(t) = 보간한 tx_to_rx_deg(t)
```

현재 동적 추적 로그에서 `tx_to_rx_deg`와 Tx의
`gimbal_command_ros_deg`는 모두 실험 시작 정면을 0°로 둔 동일한 로컬 ROS
좌표계로 기록된다. 따라서 실제 UWB 짐벌 명령은 별도의 로버 헤딩이나 90°
오프셋을 더하지 않고 직접 비교한다.

```text
Tx commanded direction(t) = gimbal_command_ros_deg(t)

Tx error(t)
    = wrap(Tx commanded direction(t) - Tx ideal direction(t))

Tx MAE
    = (1 / N_tx) × Σ abs(Tx error(t))
```

분석 전에 첫 유효 구간에서 두 값이 같은 기준(대체로 약 0°)에서 시작하는지
확인한다. 약 90° 또는 180°의 일정한 차이가 처음부터 나타나면 이를 성능
오차로 단정하지 말고 좌표계 정의를 재확인한 뒤 분석을 중단하고 보고한다.
`N_tx`는 로버 시간 범위 안에서 이상적 방향과 실제 명령 방향을 모두 계산할
수 있는 유효 Tx 명령 수다.

Rx와 Tx의 방향값은 그래프에서 동일한 이상적 방향 추세와 비교할 수 있도록
각 장치의 좌표계 정의를 확인한 뒤 공통 표시 기준으로 정렬한다. 현재 로그처럼
Rx의 `rx_to_tx_deg`와 Tx의 로컬 명령 방향이 동일한 증가 방향과 영점을
사용하는 경우 Rx 이상적 방향을 공통 `Ideal direction` 선으로 사용한다.
초기 유효 구간에서 Tx 이상적 방향과 공통 이상적 방향 사이에 약 90° 또는
180°의 일정한 차이나 반대 부호가 나타나면 임의로 맞추지 말고 좌표계 정의를
재확인한 뒤 분석을 중단하고 보고한다. 표시 좌표계를 통일하더라도 Rx MAE는
Rx 이상적 방향, Tx MAE는 Tx 이상적 방향을 기준으로 각각 계산한다.

### 5.2 그래프 형식

`rx_tx_alignment_tracking_overlay.png` 한 장에 다음 세 시계열을 겹쳐 그린다.

- x축: `Elapsed time (s)`
- y축: `Direction (deg)`
- 공통 이상적 방향: 검정 점선, `Ideal direction`
- Rx 명령 기반 정렬 방향: 빨간색 실선과 원형 마커,
  `Rx alignment direction (MAE = x.xx°)`
- Tx 짐벌 명령 방향: 파란색 일점쇄선과 사각형 마커,
  `Tx alignment direction (MAE = x.xx°)`
- 전체 그림 크기는 `10.5 × 6.47 in`, 300 dpi를 기준으로 하여 첨부 예시와
  같은 `3150 × 1940 px` 및 약 `1.62:1`의 가로·세로 비율로 저장한다.
- 범례 글자 크기는 첨부 예시의 선 설명과 동일하게 전체 그래프 글자 배율을
  적용한다. 논문용 `font-scale=2.0`에서는 22 pt를 사용하며 별도로 축소하지
  않는다.
- 거리별 상위 3회 통합 그래프 `alignment_tracking_1m_best3`,
  `alignment_tracking_2m_best3`, `alignment_tracking_3m_best3`의 축 제목,
  눈금, 범례 등 모든 글자는 22 pt의 1.5배인 33 pt로 생성한다.
  캔버스는 `3150 × 1940 px`, 300 dpi를 유지한다.
- 긴 범례를 한 행에 압축하지 않고 그래프 위쪽에 한 열·세 행으로 배치하여
  세 선의 이름과 Rx·Tx MAE가 잘리지 않게 표시한다.
- 논문용 그래프에는 제목을 넣지 않는다.
- 격자를 표시하고 300 dpi 이상으로 저장한다.
- 원형 각도가 `-180°/180°`를 통과하면 세 선을 같은 표시 기준으로 unwrap해
  가짜 수직선을 제거한다. 원본 값과 MAE 계산값은 바꾸지 않는다.
- 선 색뿐 아니라 선 종류와 마커도 달리하여 흑백 출력에서도 구분 가능하게
  한다.

결과 문서의 설명은 세 선의 시간 변화 추세, Rx와 Tx의 지연·과도응답·지속적
오프셋·포화 구간 및 두 MAE를 근거로 한국어 3~5문장으로 작성한다. 그래프만
보고 원인을 단정하지 않으며, Rx와 Tx 모두 엔코더 실측값이 아닌 실제 적용된
짐벌 명령 기반 방향이라는 한계를 반드시 밝힌다.

---

## 6. 각 실험 폴더에 만들 산출물

각 분석 대상 폴더에는 정확히 다음 파일을 만든다.

```text
<experiment_folder>/
├── dynamic_tracking_analysis_<experiment_id>.md
└── rx_tx_alignment_tracking_overlay.png
```

중간 계산 파일이 꼭 필요하면 `analysis_data/` 하위에만 저장한다. 원본 CSV와
같은 이름을 사용하거나 원본을 덮어쓰지 않는다.

`dynamic_tracking_analysis_<experiment_id>.md`는 아래 템플릿을 따른다.

```markdown
# <experiment_id> Dynamic Tracking Analysis

## 분석 범위

- Rover log: `<filename>`
- Rx log: `rx.csv`
- Tx log: `<filename>`
- Rover time range: `<t_start>`–`<t_end>` s
- 분석 기준: 로버 기록 시간 범위 안의 표본만 사용

## 1. 빨간색 인식률

| 전체 주기 | 인식 성공 | 색상 미검출 | 카메라 타임아웃 | UWB 미수신 | 기타 오류 | 시스템 전체 인식률 | 정상 처리 조건 인식률 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| ... | ... (...%) | ... (...%) | ... (...%) | ... (...%) | ... (...%) | ...% | ...% |

<전체 분모, 각 실패 원인, 정상 처리 조건 분모를 설명하는 한국어 1~2문장>

## 2. Rx·Tx 정렬 정확도

![Rx and Tx alignment tracking](rx_tx_alignment_tracking_overlay.png)

<세 선의 추세와 Rx·Tx MAE에 대한 데이터 기반 한국어 설명 3~5문장>

## 해석 시 주의사항

- Rx와 Tx의 정렬 방향은 엔코더 측정값이 아니라 실제 적용된 짐벌 명령을
  좌표계에 맞게 변환한 값이다.
- 분석은 로버 로그의 시간 범위로 제한했다.
```

결과 문서에는 위 두 지표 외의 새로운 성능 표나 그래프를 추가하지 않는다.

---

## 7. Codex 실행 순서

Codex는 다음 순서대로 전체 작업을 수행한다.

1. `result/dynamic_tracking/`의 실험 폴더와 CSV 헤더를 조사한다.
2. 위 파일 선택 규칙으로 완전한 실험과 건너뛸 실험을 구분한다.
3. 반복 실행 가능한 분석 스크립트를 `code/experiment/dynamic_tracking/`
   아래에 작성하거나 기존 스크립트를 안전하게 확장한다.
4. 각 실험의 로버 `[t_start, t_end]`를 구한다.
5. 그 범위 안의 모든 Rx 주기를 성공·색상 미검출·카메라 타임아웃·UWB
   미수신·기타 오류로 분류하고 시스템 전체 인식률과 정상 처리 조건
   인식률을 계산한다.
6. 각 Rx 명령 시각에 로버 `heading_deg`와 `rx_to_tx_deg`를 원형 보간해
   Rx 방향과 MAE를 계산한다.
7. 각 Tx 명령 시각에 로버 `tx_to_rx_deg`를 원형 보간해 Tx 방향과 MAE를
   계산하고, 공통 표시 좌표계를 검증한다.
8. 이상적 방향, Rx 정렬 방향 및 Tx 명령 방향을 한 그래프에 겹쳐 그리고
   범례에 Rx·Tx MAE를 병기한다.
9. 각 실험 폴더에 통합 정렬 그래프 PNG와 결과 MD를 저장한다.
10. 생성된 PNG를 직접 열어 선·축·범례·영문 렌더링을 확인한다.
11. 모든 결과 MD의 수치가 원본 CSV로 다시 계산한 값과 일치하는지 검증한다.
12. 마지막에 분석 완료 폴더, 건너뛴 폴더와 이유, 생성 파일을 사용자에게
    보고한다.

논문용 그래프는 다음 실행 형식을 사용한다. `<experiment_id>`에는 분석할
실험 폴더명을 넣으며 여러 실험은 `--experiment`를 반복한다.

```bash
python3 code/experiment/dynamic_tracking/analyze_individual_experiments.py \
  --experiment <experiment_id> \
  --no-title \
  --append-experiment-name \
  --font-scale 2.0
```

이 옵션 조합과 코드에 고정된 `10.5 × 6.47 in` 규격을 사용해야 이후에도
동일한 비율과 22 pt 범례로 재생성할 수 있다.

---

## 8. 필수 검증 체크리스트

각 실험마다 아래 항목을 모두 확인한다.

- [ ] 선택한 세 CSV가 같은 실험의 로그다.
- [ ] 모든 사용 행이 로버의 `[t_start, t_end]` 안에 있다.
- [ ] `sample_index=0`을 제외했다.
- [ ] 빨간색 인식률 분모에 시간 범위 안의 `uwb_unavailable`,
  `camera_frame_timeout`, `error`를 포함했다.
- [ ] `command_elapsed_s`가 없는 실패 행은 `scheduled_elapsed_s`로 시간 범위를
  판정했다.
- [ ] `N = S + C + F + U + E`가 성립한다.
- [ ] 정상 처리 조건 인식률을 `S / (S + C)`로 계산했다.
- [ ] Rx 계산에 `heading_deg + 90° + gimbal_command_ros_deg`를 사용했다.
- [ ] Tx 계산에서 `tx_to_rx_deg`와 `gimbal_command_ros_deg`의 시작 좌표계를 확인했다.
- [ ] 각도 보간과 차이 계산에 원형 각도 경계를 고려했다.
- [ ] Rx·Tx MAE를 각각 `mean(abs(wrap(actual - ideal)))`로 계산했다.
- [ ] 한 그래프에 이상적 방향, Rx 정렬 방향 및 Tx 명령 방향의 세 선이 있다.
- [ ] Rx와 Tx 범례에 각 MAE를 소수점 둘째 자리까지 표시했다.
- [ ] 그래프 크기가 300 dpi 기준 `3150 × 1940 px`이고 가로·세로 비율이
  첨부 예시와 같은 약 `1.62:1`이다.
- [ ] 논문용 범례가 22 pt이며 세 항목이 축소되거나 잘리지 않았다.
- [ ] 거리별 상위 3회 통합 그래프의 모든 글자가 33 pt이며
  `3150 × 1940 px`, 300 dpi 캔버스에서 잘리지 않았다.
- [ ] 통합 정렬 그래프 PNG가 깨지지 않고 MD에서 상대경로로 표시된다.
- [ ] 표 설명과 통합 정렬 그래프 해석을 한국어로 작성했다.
- [ ] 설명에서 명령 기반 방향을 물리적 실측 방향이라고 과장하지 않았다.
- [ ] 원본 CSV와 실패 프레임을 수정하지 않았다.

이 체크리스트 중 하나라도 만족하지 못하면 해당 실험을 완료로 표시하지
말고, 문제와 필요한 조치를 결과 문서 또는 최종 작업 보고에 명시한다.
