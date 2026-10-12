# Dynamic Tracking 논문용 통합 결과 제작 계획

## 1. 문서의 목적과 범위

이 문서는 1 m, 2 m, 3 m에서 각각 선택한 세 실험의 개별 분석이 끝난 뒤,
세 결과를 하나의 논문용 결과로 통합하는 후속 작업 계획이다. 인식 성능은
기존과 같은 거리별 3행 표로 비교하고, 정렬 정확도는 첨부 예시와 같은
Rx·Tx 중첩 시계열 그래프를 거리마다 한 장씩 만든다.

개별 실험의 분석 방법과 산출물 규칙은 이번 요청에 맞게 함께 개정한
`dynamic_tracking_evaluation_method.md`를 따른다. 개별 분석에서는 인식률과
거리별 Rx·Tx 통합 정렬 그래프를 만들고, 이 문서는 그 세 실험 결과를 입력으로
사용하는 통합·시각화 단계를 정의한다.

비교 대상 거리는 다음 세 조건이다. 각 거리에서는 사용자가 지정한 실험
데이터 **한 개만** 사용한다.

| Distance | Candidate experiments | Selected experiment |
|---:|---|---|
| 1 m | `1m01`, `1m02` | 사용자 지정 |
| 2 m | `2m01`, `2m02` | 사용자 지정 |
| 3 m | `3m01`, `3m02` | 사용자 지정 |

산출물을 만들기 전에 사용자가 거리별 선택 실험을 확정해야 한다. 선택하지
않은 나머지 실험은 평균, 합산, 보정 또는 참고값 계산에 사용하지 않는다.
선택 결과는 통합 분석 문서의 분석 대상 절에 명시해 재현할 수 있게 한다.

폴더명만 신뢰하지 않고 선택된 각 실험의 개별 분석 문서와 원본 CSV의 존재
여부를 다시 검증한다. 선택된 실험의 개별 분석이 완료되지 않았거나 검증을
통과하지 못하면 다른 실험으로 자동 대체하지 않고 작업을 중단하여 사용자에게
새 선택 또는 보완을 요청한다.

---

## 2. 최종 산출물과 저장 위치

모든 산출물은 다음 디렉터리에 저장한다.

```text
result/dynamic_tracking/
```

계획된 파일은 다음과 같다.

```text
result/dynamic_tracking/
├── dynamic_tracking_evaluation_method.md        # 개정된 개별 분석 지침
├── dynamic_tracking_paper_results_plan.md       # 이 계획 문서
├── dynamic_tracking_summary_table.csv           # 영문 3행 통합 표 데이터
├── dynamic_tracking_summary_table.tex           # 제목·캡션 없는 논문 삽입용 표
├── alignment_tracking_1m.png                    # 1 m Rx·Tx 통합 그래프
├── alignment_tracking_1m.pdf                    # 1 m 벡터 그래프
├── alignment_tracking_2m.png                    # 2 m Rx·Tx 통합 그래프
├── alignment_tracking_2m.pdf                    # 2 m 벡터 그래프
├── alignment_tracking_3m.png                    # 3 m Rx·Tx 통합 그래프
├── alignment_tracking_3m.pdf                    # 3 m 벡터 그래프
└── dynamic_tracking_paper_analysis.md            # 통합 결과와 한국어 분석
```

PNG는 최소 300 dpi로 저장하고, 가능한 경우 PDF를 논문 원고의 우선 삽입
파일로 사용한다. 중간 집계 데이터가 필요하면
`result/dynamic_tracking/paper_analysis_data/` 아래에만 저장한다.

---

## 3. 입력 자료와 선행 조건

사용자가 거리별로 선택한 세 실험 폴더에서만 다음 자료를 입력으로 사용한다.

1. `dynamic_tracking_analysis_<experiment_id>.md`
2. 로버 CSV
3. `rx.csv`
4. Tx CSV

개별 분석 문서에 적힌 수치는 요약 확인에 사용하되, 최종 표의 지표와 MAE는
반올림된 문서 수치를 재사용하지 않고 원본 CSV에서 다시 계산한다. 계산식,
시간 범위, 유효 행 선택, 원형 각도 보간 및 좌표계 변환은
`dynamic_tracking_evaluation_method.md`와 동일하게 유지한다.

통합 작업 전에 다음 조건을 확인한다.

- 사용자가 1/2/3 m별 실험을 정확히 한 개씩 지정했는가
- 선택되지 않은 실험 데이터가 계산 입력에 포함되지 않았는가
- 선택된 각 실험이 개별 검증을 통과했는가
- 선택된 각 실험의 로버 시간 범위가 확정되어 있는가
- `sample_index=0` 및 시간 범위 밖 표본이 제외되었는가
- Rx와 Tx의 이상적 방향 및 명령 기반 정렬 방향 계산식이 기존 지침과
  일치하는가
- 모든 방향 오차가 단순 뺄셈이 아니라 원형 각도 차이로 계산되었는가

---

## 4. 거리별 3행 통합 표

기존의 실험별 1행 표를 거리별로 합쳐, 데이터 행이 `1 m`, `2 m`, `3 m`인
하나의 3행 표로 만든다. 각 행에는 해당 거리에서 사용자가 선택한 단일 실험의
지표만 기록한다. 같은 거리의 다른 실험과 건수 또는 비율을 합산하거나
평균하지 않는다.

거리 \(d\)에서 선택한 실험의 분류 건수를 다음처럼 계산한다.

```text
Overall detection rate_d = S_d / N_d × 100
Conditional detection rate_d = S_d / (S_d + C_d) × 100
```

여기서 `N_d`, `S_d`, `C_d`, `F_d`, `U_d`, `E_d`는 모두 거리 \(d\)에서
선택한 한 실험의 값이다.

Rx 및 Tx MAE도 선택한 한 실험 안의 모든 유효 표본에 대한 절대 원형 각도
오차로 계산한다.

```text
circular_error_deg = wrap(aligned_direction - ideal_direction)
MAE_d = Σ |circular_error_deg| / number_of_valid_samples_d
```

표의 데이터 행은 거리별 세 행만 유지한다. 선택한 실험 ID는 표에 넣지 않고
통합 분석 문서의 분석 대상 절에 기록한다.

표의 모든 문자열, 열 이름 및 단위는 영어로 작성하고 표 제목이나 캡션은
넣지 않는다. 기본 열 구성은 다음과 같다.

| Distance (m) | Total cycles | Successful detections | Color not detected | Camera timeouts | UWB unavailable | Other errors | Overall detection rate (%) | Conditional detection rate (%) | Rx MAE (deg) | Tx MAE (deg) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | ... | ... | ... | ... | ... | ... | ... | ... | ... | ... |
| 2 | ... | ... | ... | ... | ... | ... | ... | ... | ... | ... |
| 3 | ... | ... | ... | ... | ... | ... | ... | ... | ... | ... |

건수 열에는 정수만 표시하고, 비율과 MAE는 소수점 둘째 자리까지 표시하는 것을
기본으로 한다. 논문 전체의 표기 정밀도가 따로 정해지면 그 규칙에 맞추되,
CSV에는 반올림 전 계산값을 보존한다. `N_d=0` 또는 유효 방향 표본이 없으면
0으로 쓰지 않고 `N/A`로 표시한다.

---

## 5. 거리별 정렬 정확도 그래프

선택한 세 실험을 하나의 그림에 섞지 않고, 1 m, 2 m, 3 m마다 정렬 정확도
그래프를 한 장씩 만든다. 각 그래프는 첨부 예시처럼 공통 이상적 방향,
Rx 명령 기반 정렬 방향 및 Tx 짐벌 명령 방향의 세 선을 동일한 시간축에
겹쳐 표시한다.

```text
alignment_tracking_1m: Ideal direction + Rx alignment direction + Tx alignment direction
alignment_tracking_2m: Ideal direction + Rx alignment direction + Tx alignment direction
alignment_tracking_3m: Ideal direction + Rx alignment direction + Tx alignment direction
```

Rx 값은 기존 지침과 동일하게 계산한다.

```text
Ideal Rx direction(t) = interpolated rx_to_tx_deg(t)

Aligned Rx direction(t)
    = wrap(
        interpolated heading_deg(t)
        + 90 deg
        + gimbal_command_ros_deg(t)
      )
```

Tx 값도 기존 지침과 동일하게 계산한다.

```text
Ideal Tx direction(t) = interpolated tx_to_rx_deg(t)
Tx commanded direction(t) = gimbal_command_ros_deg(t)
```

그래프의 공통 `Ideal direction`은 개별 분석 지침에 따라 Rx와 Tx 좌표계가
같은 영점과 증가 방향을 사용하는지 확인한 뒤 표시한다. Rx MAE는 Rx 이상적
방향을, Tx MAE는 Tx 이상적 방향을 기준으로 각각 계산하므로 공통 표시선을
사용하더라도 오차 기준은 바뀌지 않는다. 좌표계가 일치하지 않으면 임의
오프셋이나 부호 보정으로 겹치지 말고 통합 그래프 생성을 중단해 보고한다.

각 거리에서는 사용자가 선택한 단일 실험의 시계열만 사용한다. 같은 거리의
다른 실험과 평균하거나 합산하지 않는다. 각 장치의 `command_elapsed_s`를
x축으로 사용하고, 해당 장치의 유효 표본이 끝나는 실제 시각에서 선을
끝낸다. 원형 각도가 `-180 deg/180 deg` 경계를 지날 때 생기는 가짜 수직선은
세 표시용 시계열을 동일 기준으로 unwrap하여 제거하되 원본 값과 MAE 계산값은
변경하지 않는다.

그래프 형식은 다음을 따른다.

- x-axis: `Elapsed time (s)`
- y-axis: `Direction (deg)`
- 공통 이상적 방향은 검정 점선과 `Ideal direction` 범례를 사용한다.
- Rx 방향은 빨간 실선과 원형 마커를 사용하고 범례를
  `Rx alignment direction (MAE = x.xx°)`로 표시한다.
- Tx 방향은 파란 일점쇄선과 사각형 마커를 사용하고 범례를
  `Tx alignment direction (MAE = x.xx°)`로 표시한다.
- 전체 그림 크기는 `10.5 × 6.47 in`, 300 dpi로 설정하여 첨부 예시와 같은
  `3150 × 1940 px` 및 약 `1.62:1`의 가로·세로 비율을 사용한다.
- 범례 글자에는 첨부 예시의 선 설명과 동일한 전체 글자 배율을 적용한다.
  논문용 `font-scale=2.0`에서는 22 pt이며, 세 항목을 맞추기 위해 글자를
  별도로 축소하지 않는다.
- 범례는 그래프 위쪽에 한 열·세 행으로 배치하여 긴 세 항목과 MAE가 잘리지
  않게 한다.
- 그래프 내부와 축, 눈금, 범례, 주석은 모두 영어로 작성한다.
- 그래프 제목은 넣지 않는다.
- 색상만으로 선을 구분하지 않도록 선 종류를 함께 사용한다.
- 최종 원고의 단일 열 또는 이중 열 폭에서 읽을 수 있는 글자 크기를 사용한다.
- 세 거리 그래프의 크기, 축, 여백, 색상, 선 종류, 마커 및 글꼴 체계를
  통일해 거리 간 시각 비교가 가능하게 한다.

Rx·Tx MAE는 각 거리에서 선택한 단일 실험의 모든 유효 표본으로 계산하고,
첨부 예시처럼 각 장치의 범례에 소수점 둘째 자리까지 병기한다.

---

## 6. 통합 분석 문서

`dynamic_tracking_paper_analysis.md`는 세 실험의 인식률 통합표와 거리별
정렬 정확도 그래프 세 장을 한 문서에서 제시하고
논문 작성에 활용할 수 있는 데이터 기반 해석을 제공한다. 설명은 한국어로
작성해도 되지만, 삽입되는 표와 그래프 자체의 텍스트는 모두 영어로 유지한다.

문서 구성은 다음과 같다.

```markdown
# Dynamic Tracking 통합 분석

## 분석 대상 및 통합 방법
<거리별로 선택한 실험 ID, 선택되지 않은 실험을 계산에서 제외했다는 점>

## 거리별 통합 결과
<영문 3행 표>
<거리 증가에 따른 인식률 및 Rx/Tx MAE 변화 분석>

## 거리별 정렬 정확도

### 1 m
![1 m Rx and Tx alignment](alignment_tracking_1m.png)
<1 m의 이상적·Rx·Tx 추세, 오프셋, 지연, 포화 및 MAE 분석>

### 2 m
![2 m Rx and Tx alignment](alignment_tracking_2m.png)
<2 m의 이상적·Rx·Tx 추세, 오프셋, 지연, 포화 및 MAE 분석>

### 3 m
![3 m Rx and Tx alignment](alignment_tracking_3m.png)
<3 m의 이상적·Rx·Tx 추세, 오프셋, 지연, 포화 및 MAE 분석>

<세 거리의 Rx·Tx MAE와 추종 특성을 함께 비교한 해석>

## 종합 해석과 한계
<거리 효과, 거리별 단일 실험 선택의 한계, 명령 기반 방향이라는 측정 한계>
```

분석 문장은 실제 계산 결과가 나온 뒤 작성하며 다음 원칙을 지킨다.

- 거리 증가가 성능 변화의 직접 원인이라고 단정하지 않는다.
- 각 거리에서 단일 실험만 사용했으므로 반복 간 변동성과 재현성을 이
  결과만으로 평가할 수 없음을 명시한다.
- 선택되지 않은 실험의 결과를 분석 근거나 보조 통계로 사용하지 않는다.
- MAE와 그래프의 추세를 함께 근거로 사용한다.
- Rx와 Tx의 정렬 방향은 엔코더 실측 방향이 아니라 짐벌 명령으로 계산하거나
  기록한 방향임을 명시한다.
- 거리별 실행 시간이 다를 수 있으므로 같은 elapsed time이 동일한 궤적
  진행 위치를 의미하지 않을 수 있다는 한계를 기록한다.
- 그래프만으로 확인할 수 없는 원인을 추정하거나 단정하지 않는다.

---

## 7. 구현 순서

1. 사용자에게 1/2/3 m별로 사용할 실험 ID를 하나씩 받아 확정한다.
2. 선택된 세 실험의 개별 분석 완료 여부와 입력 CSV를 검사한다.
3. 기존 평가 지침과 같은 계산 코드로 선택된 실험의 건수와 방향 오차를
   재계산한다.
4. 개별 분석 문서의 수치와 재계산 결과가 일치하는지 검증한다.
5. 각 거리에서 선택한 단일 실험의 인식률과 Rx·Tx MAE를 계산한다.
6. 선택된 각 실험의 이상적·Rx·Tx 시계열로 거리별 통합 그래프 세 장을
   생성한다.
7. 영문 3행 표를 CSV와 LaTeX 형식으로 생성한다.
8. 통합 분석 문서에 표, 그래프 및 한국어 해석을 작성한다.
9. PNG와 PDF를 직접 확인하고 수치 및 렌더링 검증을 수행한다.

반복 실행 가능한 통합 분석 스크립트가 필요하면
`code/experiment/dynamic_tracking/` 아래에 두며, 원본 CSV나 기존 개별
산출물을 수정하지 않는다. 거리별 그래프 생성 시에는 개별 분석 지침의
`--no-title --append-experiment-name --font-scale 2.0` 옵션과 코드에 고정된
`10.5 × 6.47 in` 규격을 그대로 사용하여 세 거리의 출력 형식을 재현한다.

---

## 8. 최종 검증 체크리스트

- [ ] 개별 분석 결과가 개정된 `dynamic_tracking_evaluation_method.md` 형식을
  따른다.
- [ ] 사용자가 1/2/3 m별 실험을 정확히 하나씩 지정했다.
- [ ] 선택되지 않은 실험은 표, MAE, 그래프 및 분석 계산에서 제외했다.
- [ ] 1/2/3 m가 각각 표의 한 행으로 표시된다.
- [ ] 표의 각 행은 해당 거리에서 선택한 한 실험의 값만 사용한다.
- [ ] 선택한 실험 ID는 표가 아니라 통합 분석 문서에 기록했다.
- [ ] Rx·Tx MAE를 선택한 실험의 전체 유효 표본으로 계산했다.
- [ ] 1/2/3 m 거리마다 정렬 정확도 그래프가 정확히 한 장씩 있다.
- [ ] 각 거리 그래프에 Ideal direction, Rx alignment direction,
  Tx alignment direction의 세 선이 있다.
- [ ] 각 거리 그래프의 Rx·Tx 범례에 해당 MAE가 영어로 병기되어 있다.
- [ ] 세 그래프 모두 300 dpi 기준 `3150 × 1940 px`, 약 `1.62:1` 비율이다.
- [ ] 범례 글자가 논문용 22 pt이고 세 항목이 축소되거나 잘리지 않았다.
- [ ] 표와 그래프의 모든 텍스트가 영어다.
- [ ] 표와 그래프에 제목이나 캡션을 넣지 않았다.
- [ ] 세 거리 그래프의 색상, 선 스타일, 마커 및 글꼴 체계가 일치한다.
- [ ] 각도 오차와 보간에 원형 각도 경계를 반영했다.
- [ ] elapsed time을 사용했으며 거리별 선은 실제 실험 종료 시각에서 끝난다.
- [ ] 표시용 unwrap 과정에서 인위적인 각도 불연속이 생기지 않았다.
- [ ] CSV 수치, LaTeX 표, 그래프 주석 및 분석 문서의 수치가 서로 일치한다.
- [ ] PNG가 300 dpi 이상이고 PDF가 정상적인 벡터 파일이다.
- [ ] 분석 문서가 명령 기반 방향을 물리적 실측 방향으로 과장하지 않는다.
- [ ] 원본 로그와 기존 개별 분석 산출물을 수정하지 않았다.
