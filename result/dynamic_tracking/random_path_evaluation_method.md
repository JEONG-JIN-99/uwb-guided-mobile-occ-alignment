# Random Path Dynamic Tracking 개별 실험 분석 작업 지침

## 1. 목적과 분석 결과

이 문서는 Codex가 `result/dynamic_tracking/` 아래의 랜덤 경로 동적 추적
실험을 각각 독립적으로 분석하기 위한 구현 지침이다. 거리별 원호 실험과
동일하게 다음 세 결과를 만들되, 랜덤 경로에서 발생하는 짐벌 가동 범위
초과를 분석에 명시적으로 반영한다.

1. 짐벌 정렬 가능 여부를 구분한 빨간색 인식률 표
2. Rx 이상적 방향과 UWB 기반 정렬 방향 그래프
3. Tx 이상적 방향과 UWB 짐벌 명령 방향 그래프

Rx·Tx 방향 MAE는 짐벌로 물리적으로 정렬할 수 있는 구간에서만 계산한다.
범위 초과 시간을 시간축에서 삭제하거나 압축하지 않는다.

---

## 2. 랜덤 경로 실험 선택

분석 루트는 다음과 같다.

```text
result/dynamic_tracking/
```

폴더명만 보고 랜덤 실험을 판단하지 않는다. 다음 조건을 확인한다.

- Rx 또는 Tx 로그의 `trajectory_mode`가 `random_moving_rover`다.
- 일반적으로 `distance_m=0`으로 기록된다.
- 로버가 고정 반경 원호가 아닌 랜덤 경로로 이동한 로그라는 실험 기록이 있다.
- Rx·Tx·로버 로그가 같은 공유 시작시각의 실행인지 확인할 수 있다.

`trajectory_mode=fixed_radius_arc`인 로그는 폴더명이 랜덤 실험처럼 보이더라도
자동으로 포함하지 않는다. 현재 `3333`의 `rx.csv`는
`trajectory_mode=fixed_radius_arc`이므로 별도 실험 기록 없이 랜덤 경로
분석 대상으로 간주하지 않는다.

한 랜덤 실험에는 다음 세 로그가 필요하다.

| 용도 | 필수 열 |
|---|---|
| 로버 | `elapsed_sec`, `heading_deg`, `rx_to_tx_deg`, `tx_to_rx_deg` |
| Rx | `sample_index`, `scheduled_elapsed_s`, `command_elapsed_s`, `gimbal_command_ros_deg`, `servo_clipped`, `target_color`, `color_success`, `status` |
| Tx | `sample_index`, `scheduled_elapsed_s`, `command_elapsed_s`, `gimbal_command_ros_deg`, `servo_clipped`, `status` |

CSV에 여러 실행이 이어 붙어 있으면 `sample_index=0`을 실행 경계로 사용한다.
로버 로그 및 상대 장치의 시작시각과 일치하는 실행 구간만 선택한다. 확정할 수
없으면 분석하지 말고 이유를 보고한다.

---

## 3. 공통 시간 범위

모든 표와 그래프는 로버가 기록된 시간 동안의 데이터만 사용한다.

```text
t_start = 로버 로그의 첫 번째 유효 elapsed_sec
t_end   = 로버 로그의 마지막 유효 elapsed_sec
```

방향 분석용 장치 행은 다음 조건을 만족해야 한다.

```text
sample_index >= 1
t_start <= command_elapsed_s <= t_end
gimbal_command_ros_deg가 유효한 숫자
```

인식률 표에서 UWB 미수신이나 처리 오류로 `command_elapsed_s`가 비어 있으면
다음 평가 시각을 사용한다.

```text
evaluation_elapsed_s
    = command_elapsed_s가 유효하면 command_elapsed_s
    = 그렇지 않으면 scheduled_elapsed_s
```

인식률 표에는 `sample_index >= 1`이고
`t_start <= evaluation_elapsed_s <= t_end`인 모든 Rx 주기를 사용한다.
`sample_index=0`은 초기화 행이므로 제외한다.

로버 값이 장치 명령 시각에 정확히 없으면 인접 로버 표본 사이에서 선형
보간한다. 각도는 최단 회전 방향으로 원형 보간한다.

```text
wrap(a) = ((a + 180) mod 360) - 180
delta = wrap(angle_after - angle_before)
angle(t) = wrap(angle_before + ratio × delta)
```

시간 범위 밖의 외삽은 금지한다.

---

## 4. 짐벌 가동 범위와 정렬 가능 여부

현재 Rx와 Tx yaw 짐벌의 ROS 명령 가능 범위는 다음과 같다.

```text
GIMBAL_MIN = -90°
GIMBAL_MAX = +90°
```

구현할 때 코드의 실제 제한값도 확인한다. 하드웨어 또는 제어 코드의 범위가
바뀌었으면 문서와 분석 설정을 함께 갱신한다.

### 4.1 Rx 이상적 요구 명령각

Rx 짐벌 0° 광축은 로버 헤딩에서 ROS 양의 방향으로 90° 오프셋되어 있다.

```text
Rx required command(t)
    = wrap(
        interpolated rx_to_tx_deg(t)
        - interpolated heading_deg(t)
        - 90°
      )
```

```text
Rx alignable(t)
    = -90° <= Rx required command(t) <= +90°
```

### 4.2 Tx 이상적 요구 명령각

Tx 로그의 좌표계는 분석 전에 확인한다.

#### 경우 A: `tx_to_rx_deg`가 Tx 정면을 0°로 둔 로컬 방향

초기 `tx_to_rx_deg`가 약 0°이고 Tx 짐벌 명령 0°와 같은 기준이면 다음 식을
사용한다.

```text
Tx required command(t) = wrap(tx_to_rx_deg(t))
```

#### 경우 B: `tx_to_rx_deg`가 ROS 절대 방향

Tx 짐벌 명령 0°의 절대 광축이 `-180°`이고 초기 `tx_to_rx_deg`도 약
`-180°`이면 다음 식을 사용한다.

```text
TX_ZERO_ABSOLUTE_DIRECTION = -180°

Tx required command(t)
    = wrap(tx_to_rx_deg(t) - TX_ZERO_ABSOLUTE_DIRECTION)
    = wrap(tx_to_rx_deg(t) + 180°)
```

```text
Tx alignable(t)
    = -90° <= Tx required command(t) <= +90°
```

좌표계 경우를 자동 추측한 뒤 바로 분석하지 않는다. 첫 유효 구간에서 이상적
요구각과 짐벌 명령의 기준이 일치하는지 확인하고, 선택한 경우 A 또는 B를
각 실험 결과 문서에 기록한다.

### 4.3 `servo_clipped` 사용 규칙

정렬 가능 여부와 MAE 제외 여부는 로버 기준의 `required command`로 판단한다.
`servo_clipped=1`만으로 표본을 제외하지 않는다.

- `required command`가 범위 밖이면 물리적 정렬 불가능 표본으로 분류한다.
- `required command`가 범위 안인데 `servo_clipped=1`이면 실제 제어 또는
  UWB 추정 과정에서 포화된 결과이므로 유효 구간 성능에 포함한다.
- 두 판정이 불일치하는 표본 수를 검증 로그에 남긴다.

`servo_clipped` 표본을 무조건 제외하면 정렬 가능한 상황에서 발생한 시스템
오차를 숨겨 MAE가 인위적으로 낮아질 수 있다.

---

## 5. 빨간색 인식률 표

### 5.1 상태 분류

로버 시간 범위 안에서 `target_color`가 `red`인 모든 Rx 평가 주기를 다음 중
정확히 하나로 분류한다. 색상 문자열은 대소문자와 앞뒤 공백을 무시해
비교한다.

```text
S = 인식 성공
    color_success == 1 또는 status == "success"

C = 색상 미검출
    status == "color_not_detected"

F = 카메라 프레임 타임아웃
    status == "camera_frame_timeout"

U = UWB 미수신
    status == "uwb_unavailable"

E = 기타 오류
    status == "error" 또는 위 분류에 속하지 않은 실패
```

각 Rx 주기는 같은 시각의 로버 `heading_deg`와 `rx_to_tx_deg`로 계산한
`Rx required command`에 따라 다음 범위 행에도 배정한다.

```text
전체 구간
정렬 가능 구간: -90° <= Rx required command <= +90°
범위 초과 구간: Rx required command < -90° 또는 > +90°
```

UWB 미수신처럼 `command_elapsed_s`가 없는 행도 `scheduled_elapsed_s`에서
로버 값을 보간해 범위 여부를 판단한다. 해당 시각의 로버 값이 없어 판단할 수
없으면 임의로 정렬 가능 또는 범위 초과로 넣지 말고 `범위 판정 불가`로 따로
보고한다.

### 5.2 표 형식

각 실험 결과 문서에 아래 표를 넣는다. 각 상태 셀의 비율은 해당 행의
`평가 주기`를 분모로 계산한다.

| 분석 범위 | 평가 주기 | 인식 성공 | 색상 미검출 | 카메라 타임아웃 | UWB 미수신 | 기타 오류 | 시스템 전체 인식률 | 정상 처리 조건 인식률 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 전체 구간 | N | S | C | F | U | E | S/N | S/(S+C) |
| 정렬 가능 구간 | V | S_v | C_v | F_v | U_v | E_v | S_v/V | S_v/(S_v+C_v) |
| 범위 초과 구간 | R | S_r | C_r | F_r | U_r | E_r | S_r/R | S_r/(S_r+C_r) |

표의 숫자는 다음 형식으로 쓴다.

```text
평가 주기:
    건수 (전체 N 대비 비율)

각 성공/실패 상태:
    건수 (해당 행 평가 주기 대비 비율)

인식률:
    소수점 첫째 자리 백분율
```

다음 등식을 반드시 검증한다.

```text
N = V + R

각 행에서:
평가 주기 = S + C + F + U + E
```

범위 판정 불가 행이 있으면 `N = V + R + Q`로 계산하고 `Q` 행을 표에
추가한다.

두 인식률의 의미는 다음과 같다.

- `시스템 전체 인식률`: 해당 범위 행의 UWB·카메라·기타 오류를 모두 포함
- `정상 처리 조건 인식률`: UWB와 카메라가 정상이고 실제 색상 판정까지
  도달한 `S + C`만 분모로 사용

표 아래에는 반드시 다음을 서술한다.

- 전체 경로 중 Rx 정렬 가능 주기 비율 `V/N`
- Rx 범위 초과 비율 `R/N`
- 전체 경로 인식률과 정렬 가능 구간 인식률의 차이
- 범위 초과 구간에서도 우연히 인식 성공한 표본이 있는지 여부

범위 초과 구간의 인식 성공을 정렬 성공으로 해석하지 않는다. 카메라 시야나
표적 크기 때문에 빨간색이 우연히 검출될 수 있다.

---

## 6. Rx 방향 추종 그래프와 Valid-range MAE

Rx와 Tx 그래프는 기존 `10.8 × 5.9 in`, 300 dpi 캔버스 크기를 유지한다.
축 제목, 눈금, 범례 등 그래프 내부의 모든 글자는 기본 설정의 2배
(`--font-scale 2`)로 생성한다. 글자 확대에 따른 범례 잘림은 캔버스를 키우지
않고 범례를 그래프 위쪽 바깥에 배치하여 방지한다. 논문용 그래프에서는 내부
제목을 생략하고(`--no-title`) Rx와 Tx의 하단 패널 표기는 결합 단계에서 추가한다.

### 6.1 방향과 오차

```text
Rx ideal direction(t)
    = wrap(rx_to_tx_deg(t))

Rx aligned direction(t)
    = wrap(
        heading_deg(t)
        + 90°
        + gimbal_command_ros_deg(t)
      )

Rx error(t)
    = wrap(Rx aligned direction(t) - Rx ideal direction(t))
```

Rx MAE에는 다음 조건을 모두 만족하는 명령만 사용한다.

```text
Rx alignable(t) == True
command_elapsed_s가 로버 시간 범위 안
gimbal_command_ros_deg가 유효
이상적 방향과 실제 정렬 방향을 모두 계산 가능
```

```text
Rx valid-range MAE
    = mean(abs(Rx error(t))) over valid alignable commands
```

### 6.2 시간축과 제외 구간 표시

x축의 실제 로버 경과시간은 절대 삭제하거나 재배열하지 않는다.

- x축: `Elapsed time (s)`
- 파란색 점선: 전체 시간의 `Ideal Rx→Tx direction`
- 빨간색 실선: 정렬 가능 시간의 `UWB-aligned Rx direction`
- 옅은 회색 음영: `Rx required command`가 `-90~90°`를 벗어난 시간
- 범례: `Out of Rx gimbal range (excluded)`
- 빨간색 범례:
  `UWB-aligned Rx direction (Valid-range MAE = x.xx°)`

범위 초과 위치의 실제 정렬 방향 배열에는 `NaN`을 넣거나 정렬 가능한 연속
구간별로 선을 나눠 그린다. 행을 단순 삭제하면 Matplotlib이 범위 초과 전후
점을 직선으로 연결하므로 금지한다.

회색 음영 경계는 이산 표본의 중간시각을 사용한다.

```text
한 범위 초과 표본의 표시 구간
    = 이전 표본과의 중간시각부터
      다음 표본과의 중간시각까지
```

연속 범위 초과 표본은 하나의 음영 구간으로 합친다. 이상적 방향선은 전체
경로를 보여주기 위해 음영 구간에서도 유지한다. `-180°/180°` 경계를 지날
때는 표시용 시계열만 unwrap하고 MAE는 wrap된 원형 오차로 계산한다.

### 6.3 그래프 설명

결과 MD에는 한국어 2~4문장으로 다음을 설명한다.

- 전체 Rx 평가 명령 수
- MAE에 사용한 정렬 가능 유효 명령 수
- 범위 초과 제외 명령 수와 비율
- Rx valid-range MAE
- 정렬 가능 구간에서 두 선의 추세, 지연, 오프셋 또는 포화

그래프만 보고 원인을 단정하지 않는다. 정렬 방향은 엔코더 실측각이 아니라
UWB 기반 명령으로 계산한 방향이라는 한계를 밝힌다.

---

## 7. Tx 방향 추종 그래프와 Valid-range MAE

### 7.1 같은 좌표계로 변환

선택한 Tx 좌표계에 따라 두 선을 같은 좌표로 만든다.

#### 로컬 방향을 사용하는 경우

```text
Tx ideal direction(t) = wrap(tx_to_rx_deg(t))
Tx commanded direction(t) = wrap(gimbal_command_ros_deg(t))
```

#### 절대 방향을 사용하는 경우

```text
Tx ideal direction(t) = wrap(tx_to_rx_deg(t))
Tx commanded direction(t)
    = wrap(-180° + gimbal_command_ros_deg(t))
```

```text
Tx error(t)
    = wrap(Tx commanded direction(t) - Tx ideal direction(t))

Tx valid-range MAE
    = mean(abs(Tx error(t))) over valid alignable commands
```

MAE에는 `Tx required command`가 `-90~90°` 안에 있고 두 방향을 모두 계산할
수 있는 명령만 사용한다.

### 7.2 그래프 표시

- x축: `Elapsed time (s)`
- 파란색 점선: 전체 시간의 `Ideal Tx→Rx direction`
- 빨간색 실선: 정렬 가능 시간의 `UWB gimbal command direction`
- 옅은 회색 음영: Tx 짐벌 범위 초과 시간
- 범례: `Out of Tx gimbal range (excluded)`
- 빨간색 범례:
  `UWB gimbal command (Valid-range MAE = x.xx°)`

Rx와 동일하게 실제 시간축을 유지하고 범위 초과 위치는 `NaN` 또는 분리된
선 구간으로 표현한다. 이상적 방향선은 음영 구간에서도 유지한다.

### 7.3 그래프 설명

한국어 2~4문장으로 전체 Tx 명령 수, 정렬 가능 유효 명령 수, 범위 초과
제외 수와 비율, Tx valid-range MAE 및 정렬 가능한 구간의 추세를 설명한다.
Tx 방향도 엔코더 실측각이 아니라 명령 기반 방향임을 명시한다.

---

## 8. 각 랜덤 실험 폴더의 산출물

### 8.1 상위 3회 통합 결과

random01–05의 개별 Rx·Tx MAE를 모두 같은 방식으로 계산하고, 거리별 상위
3회 통합 분석과 동일하게 `(Rx MAE + Tx MAE) / 2`가 작은 순서로 세 실행을
선정한다. 인식률은 선정 기준으로 사용하지 않는다. 선정·제외 실행과 순위를
매니페스트에 모두 기록한다.

선정된 세 실행의 인식률은 실행별 백분율을 평균하지 않고 성공, 색상 미검출,
카메라 타임아웃, UWB 미수신 및 기타 오류의 원시 건수를 먼저 합산해 계산한다.
수동 검수 확정표 `red_recognition_analysis_table.csv`가 있는 실행은 해당 값을
우선하고, 없는 실행은 `rx.csv`의 상태 건수를 사용한다.

통합 방향 그래프는 각 실행의 로버 시작시각을 0초로 맞추고, 세 실행 모두
존재하는 공통 시간 구간을 0.2초 간격으로 보간하여 Ideal·Rx·Tx 방향을 각각
산술평균한다. Tx→Rx 방향은 180° 회전해 Rx→Tx와 같은 공통 방향 좌표계로
표시한다. 그래프 범례의 MAE는 평균 방향 시계열에 대한 값이며, 세 실행의 모든
원시 오차를 합친 pooled MAE는 요약 CSV에 별도로 기록한다.

랜덤 상위 3회 통합 그래프의 축 제목, 눈금, 범례 등 모든 글자는
거리별 상위 3회 통합 그래프의 22 pt 글자보다 1.5배 큰 33 pt로
생성한다. 캔버스는 기존 `10.5 × 6.467 in`, 300 dpi를 유지하고,
글자 확대로 인한 범례·축 제목 잘림은 레이아웃 조정으로 방지한다.

상위 3회 통합 산출물은 다음과 같다.

```text
result/dynamic_tracking/
├── random_tracking_best3_selection_manifest.csv
├── random_tracking_summary_best3.csv
├── random_tracking_mean_trajectory_best3.csv
├── alignment_tracking_random_best3.png
├── alignment_tracking_random_best3.pdf
└── random_tracking_best3_analysis.md
```

거리별 분석 파일과 이름이 충돌하지 않도록 다음 파일을 생성한다.

```text
<random_experiment_folder>/
├── random_path_dynamic_tracking_analysis_<experiment_id>.md
├── random_rx_alignment_tracking.png
└── random_tx_alignment_tracking.png
```

반복 실행 가능한 분석 스크립트는 다음 위치에 둔다.

```text
code/experiment/dynamic_tracking/analyze_random_path_experiments.py
```

원본 CSV, 기존 실패 프레임 및 기존 분석 결과를 덮어쓰거나 삭제하지 않는다.

---

## 9. 결과 MD 템플릿

```markdown
# <experiment_id> Random Path Dynamic Tracking Analysis

## 분석 범위

- Rover log: `<filename>`
- Rx log: `<filename>`
- Tx log: `<filename>`
- Rover time range: `<t_start>`–`<t_end>` s
- Gimbal command range: `-90°–+90°`
- Tx coordinate mode: `<local 또는 absolute>`

## 1. 빨간색 인식률

| 분석 범위 | 평가 주기 | 인식 성공 | 색상 미검출 | 카메라 타임아웃 | UWB 미수신 | 기타 오류 | 시스템 전체 인식률 | 정상 처리 조건 인식률 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 전체 구간 | ... | ... | ... | ... | ... | ... | ...% | ...% |
| 정렬 가능 구간 | ... | ... | ... | ... | ... | ... | ...% | ...% |
| 범위 초과 구간 | ... | ... | ... | ... | ... | ... | ...% | ...% |

<범위 초과 비율과 인식률 차이에 대한 한국어 설명>

## 2. Rx 방향 추종

![Rx random-path direction tracking](random_rx_alignment_tracking.png)

<정렬 가능 표본, 범위 초과 표본, Valid-range MAE 및 추세에 대한 한국어 설명>

## 3. Tx 방향 추종

![Tx random-path direction tracking](random_tx_alignment_tracking.png)

<정렬 가능 표본, 범위 초과 표본, Valid-range MAE 및 추세에 대한 한국어 설명>

## 해석 시 주의사항

- 회색 음영은 이상적 요구각이 짐벌 가동 범위를 벗어나 MAE에서 제외한 시간이다.
- 시간축은 범위 초과 시간을 제거하지 않은 실제 로버 경과시간이다.
- 정렬 방향은 엔코더 실측이 아니라 실제 적용된 짐벌 명령 기반 방향이다.
```

---

## 10. Codex 구현 및 실행 순서

1. 랜덤 경로 후보 폴더의 로버·Rx·Tx CSV와 헤더를 조사한다.
2. `trajectory_mode`, `distance_m`, 실험 기록으로 랜덤 경로 실행을 확정한다.
3. 여러 실행이 합쳐졌으면 공유 시작시각에 맞는 실행 구간만 선택한다.
4. 로버의 `[t_start, t_end]`를 구하고 모든 장치 데이터를 그 범위로 제한한다.
5. Rx·Tx 명령 시각에서 필요한 로버 각도를 원형 선형 보간한다.
6. Tx 로그가 로컬 방향인지 절대 방향인지 검증하고 결과 MD에 기록한다.
7. 각 시각의 Rx·Tx `required command`와 정렬 가능 여부를 계산한다.
8. Rx 인식 주기를 전체·정렬 가능·범위 초과로 나눠 인식률 표를 만든다.
9. 전체 시간축을 유지하고 범위 초과 구간을 회색 음영 및 선 공백으로 표시한다.
10. 정렬 가능한 유효 표본만으로 Rx·Tx valid-range MAE를 계산해 범례에 병기한다.
11. 각 랜덤 실험 폴더에 두 PNG와 결과 MD를 저장한다.
12. 모든 PNG를 직접 열어 시간 공백, 회색 음영, 범례와 MAE를 확인한다.
13. 표의 범주 합계와 MAE 표본 수를 원본 CSV로 독립 재검증한다.
14. 완료 폴더와 건너뛴 폴더 및 이유를 사용자에게 보고한다.

---

## 11. 필수 검증 체크리스트

- [ ] 실제 랜덤 경로 실험인지 확인했다.
- [ ] Rx·Tx·로버 로그가 같은 실행인지 확인했다.
- [ ] 모든 분석 데이터가 로버 시간 범위 안에 있다.
- [ ] `sample_index=0` 초기화 행을 제외했다.
- [ ] Rx 요구각을 `wrap(rx_to_tx - heading - 90°)`로 계산했다.
- [ ] Tx 좌표계가 로컬인지 절대 방향인지 확인했다.
- [ ] 정렬 가능 여부를 이상적 `required command`로 판정했다.
- [ ] `servo_clipped`만을 이유로 MAE 표본을 제외하지 않았다.
- [ ] 인식률 표에 전체·정렬 가능·범위 초과 행을 모두 표시했다.
- [ ] `N = V + R` 및 각 상태 범주의 합계를 검증했다.
- [ ] 시간축에서 범위 초과 시간을 삭제하거나 압축하지 않았다.
- [ ] 범위 초과 구간에서 빨간 실선이 앞뒤로 연결되지 않는다.
- [ ] 범위 초과 구간을 회색 음영으로 표시했다.
- [ ] 이상적 방향선은 전체 시간에 걸쳐 표시했다.
- [ ] Rx·Tx valid-range MAE에 정렬 가능 유효 표본만 사용했다.
- [ ] 두 그래프 범례에 MAE를 소수점 둘째 자리까지 표시했다.
- [ ] Rx·Tx 그래프의 캔버스는 `10.8 × 5.9 in`, 300 dpi로 유지했다.
- [ ] 그래프 내부의 모든 글자 크기를 기본 설정의 2배로 적용했다.
- [ ] random01–05를 평균 Rx·Tx MAE 오름차순으로 정렬하고 상위 3회를 선정했다.
- [ ] 상위 3회 인식 건수를 먼저 합산한 뒤 전체·조건부 인식률을 계산했다.
- [ ] 수동 검수 확정표가 있는 실행은 해당 인식 건수를 우선했다.
- [ ] 상위 3회의 공통 시간 구간에서 평균 Ideal·Rx·Tx 궤적을 생성했다.
- [ ] 랜덤 상위 3회 통합 그래프의 모든 글자를 33 pt로 생성했다.
- [ ] 랜덤 통합 그래프의 `10.5 × 6.467 in`, 300 dpi 캔버스를 유지했다.
- [ ] 평균 궤적 MAE와 원시 표본 pooled MAE를 구분해 기록했다.
- [ ] 보고서 설명을 한국어로 작성했다.
- [ ] 명령 기반 방향을 물리적 실측 방향으로 과장하지 않았다.
- [ ] 원본 로그와 기존 결과를 수정하거나 삭제하지 않았다.
