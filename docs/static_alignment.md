# Static Alignment Experiment Development Plan

## 1. 문서 목적

이 문서는 고정된 정적 alignment 실험환경에 맞는 실험 코드를 구현하기 위한 개발 계획서다.  
Codex는 이 문서를 기준으로 설정 파일, 실험 순서 생성, 짐벌 제어, UWB 수신, QR 인식, 타임아웃 처리, 결과 저장 기능을 구현해야 한다.

이 문서에서 정의하지 않은 동작을 임의로 추가하지 말고, 모호한 경우에는 설정값과 로그를 통해 재현 가능하도록 구현한다.

---

## 2. 실험 목적

Rx가 정지한 상태에서 UWB PDoA 방위각을 이용해 Rx 짐벌을 Tx 방향으로 정렬하고, 다음 성능을 평가한다.

1. 실제 Tx 방향과 UWB 기반 추정 방향의 차이
2. UWB 방위각을 이용한 짐벌 명령 결과
3. 제한 시간 내 QR 인식 성공 여부
4. 첫 유효 UWB 데이터 수신부터 QR 인식까지의 시간
5. UWB 메시지 누락과 서보 각도 제한 발생 여부
6. 거리와 초기 정렬각에 따른 성능 차이
7. UWB 측정 범위 경계인 `-60°`, `+60°` 조건의 성능

`±60°` 조건은 의도적으로 포함한 경계 성능 평가 조건이다. 코드에서 해당 조건을 제외하거나 자동 보정하지 않는다.

---

## 3. 좌표계와 부호 규칙

모든 각도는 Rx 본체의 정면축을 기준으로 정의한다.

- Rx 본체 정면: `0°`
- Rx 기준 오른쪽: 양수
- Rx 기준 왼쪽: 음수
- 단위: degree
- 각도 정규화 범위: `[-180°, 180°)`

예시:

- Tx가 Rx 왼쪽 60° 방향에 있으면 실제 Tx 방위각은 `-60°`
- Tx가 Rx 오른쪽 60° 방향에 있으면 실제 Tx 방위각은 `+60°`
- Rx 짐벌이 `-20°`를 보고 있고 Tx가 `-60°`에 있으면 이상적인 UWB 상대 방위각은 `-40°`

기본 관계식은 다음과 같다.

```text
estimated_tx_azimuth_deg
    = normalize_angle(
        rx_initial_gimbal_deg
        + uwb_raw_azimuth_deg
      )
```

여기서 `uwb_raw_azimuth_deg`는 현재 짐벌 방향을 기준으로 Tx가 위치한 상대 방위각이다.

---

## 4. 물리적 실험환경

### 4.1 거리

다음 세 거리에서 각각 실험한다.

```text
1 m
2 m
3 m
```

거리는 사람이 직접 Tx 또는 Rx의 물리적 위치를 변경하는 실험조건이다.  
코드가 거리를 자동 제어하지는 않지만, 현재 거리값은 설정과 모든 결과 로그에 반드시 포함해야 한다.

### 4.2 Tx 설치 위치

두 종류의 Tx 방위각 조건을 사용한다.

```text
-60°
+60°
```

Tx는 각 실험 블록 시작 전에 사람이 수동으로 설치한다.

- Tx 위치가 `-60°`이면 Tx의 QR 또는 OCC 송신면이 Rx를 향하도록 고정한다.
- Tx 위치가 `+60°`이면 Tx의 QR 또는 OCC 송신면이 Rx를 향하도록 고정한다.
- Tx는 한 실험 블록이 진행되는 동안 움직이지 않는다.
- Tx 짐벌 제어 또는 자동 정렬 기능은 구현하지 않는다.

---

## 5. 실험조건

### 5.1 Tx가 `-60°`에 있는 경우

Rx 초기 짐벌각 후보는 다음과 같다.

```text
-60°, -58°, -56°, ..., -2°, 0°
```

- 시작값: `-60°`
- 종료값: `0°`
- 간격: `2°`
- 각도 수: `31개`

### 5.2 Tx가 `+60°`에 있는 경우

Rx 초기 짐벌각 후보는 다음과 같다.

```text
+60°, +58°, +56°, ..., +2°, 0°
```

- 시작값: `+60°`
- 종료값: `0°`
- 간격: `-2°`
- 각도 수: `31개`

### 5.3 반복 횟수

각 초기각 조건은 정확히 `3회` 반복한다.

따라서 실험 횟수는 다음과 같다.

```text
거리 3개
× Tx 위치 2개
× 초기각 31개
× 반복 3회
= 총 558회
```

각 거리와 Tx 위치의 조합을 하나의 실험 블록으로 본다.

```text
블록당 실험 수
= 초기각 31개 × 반복 3회
= 93회
```

총 블록 수는 다음과 같다.

```text
3개 거리 × 2개 Tx 위치 = 6개 블록
```

---

## 6. 실험 순서 생성

각 블록에서는 31개의 초기각을 각각 3회 포함한 총 93개의 시행 목록을 생성한다.

각도 순서는 무작위로 섞어도 된다. 단, 다음 조건을 만족해야 한다.

1. 각 초기각이 정확히 3번 포함되어야 한다.
2. 무작위 순서는 `random_seed`로 재현 가능해야 한다.
3. 생성된 실제 시행 순서를 파일로 저장해야 한다.
4. 프로그램 재시작 후 미완료 시행부터 재개할 수 있도록 각 시행에 고유한 `trial_id`를 부여해야 한다.

권장 생성 방식:

```python
trial_angles = initial_angles * repetitions_per_angle
random.Random(random_seed).shuffle(trial_angles)
```

권장 `trial_id` 형식:

```text
static_d{distance}_tx{tx_azimuth}_idx{sequence}_rep{repeat}
```

예시:

```text
static_d2_tx-60_idx017_rep2
```

---

## 7. 블록 시작 절차

각 거리와 Tx 위치 조합별로 프로그램은 다음 정보를 표시하고 사용자 확인을 기다린다.

```text
Distance: 2 m
Tx ground-truth azimuth: -60°
Number of trials: 93
```

사용자가 물리적 설치를 완료한 뒤 명시적으로 확인해야 실험을 시작한다.

권장 확인 입력:

```text
Type START to begin the block:
```

확인 전에는 짐벌을 움직이거나 UWB/QR 타이머를 시작하지 않는다.

---

## 8. 단일 시행 진행 방식

각 시행은 반드시 다음 순서로 실행한다.

### 8.1 상태 순서

```text
1. MOVE_TO_ZERO
2. WAIT_ZERO_SETTLE
3. MOVE_TO_INITIAL
4. WAIT_INITIAL_SETTLE
5. RESET_TRIAL_STATE
6. WAIT_FOR_FIRST_VALID_UWB
7. COMPUTE_ALIGNMENT
8. COMMAND_GIMBAL
9. WAIT_FOR_QR_OR_TIMEOUT
10. SAVE_RESULT
11. RETURN_TO_ZERO
```

### 8.2 상세 동작

#### 1단계: 0°로 이동

짐벌에 `0°` 명령을 보낸다.

```text
target = 0°
```

#### 2단계: 0° 안정화 대기

설정값 `zero_settle_time_s`만큼 기다린다.

이 대기 시간은 정렬 시간이나 QR 인식 시간에 포함하지 않는다.

#### 3단계: 해당 시행의 초기각으로 이동

현재 시행에 지정된 `rx_initial_gimbal_deg`로 짐벌을 이동한다.

#### 4단계: 초기각 안정화 대기

설정값 `initial_settle_time_s`만큼 기다린다.

이 대기 시간도 정렬 시간이나 QR 인식 시간에 포함하지 않는다.

#### 5단계: 시행 상태 초기화

다음 상태를 초기화한다.

- 이전 UWB 수신 버퍼
- 이전 UWB 메시지
- 이전 QR 검출 결과
- 이전 QR 디코딩 결과
- 이전 시행 타이머
- 이전 서보 클리핑 상태
- 이전 오류 상태

0° 이동 중 또는 초기각 이동 중 수신된 UWB 데이터는 현재 시행의 측정값으로 사용하지 않는다.

#### 6단계: 첫 유효 UWB 데이터 대기

새 시행 상태가 시작된 이후 수신된 첫 번째 유효 UWB 방위각을 사용한다.

유효 UWB 데이터의 조건:

- 파싱 성공
- 방위각 값 존재
- NaN 또는 무한대가 아님
- 코드가 허용한 데이터 형식과 범위를 만족
- 현재 시행 시작 전에 수신된 오래된 메시지가 아님

`uwb_receive_timeout_s` 이내에 유효 데이터가 도착하지 않으면:

```text
uwb_message_missing = true
```

로 저장하고, 해당 시행은 UWB 실패로 종료한다.

이 경우:

- 짐벌 정렬 명령을 생성하지 않는다.
- `qr_success = false`
- `qr_recognition_time_ms = null`
- 결과를 저장한다.
- 0°로 복귀한다.

#### 7단계: 정렬각 계산

첫 유효 UWB 데이터가 수신된 순간을 `first_valid_uwb_time`으로 기록한다.

추정 Tx 절대방위각:

```text
estimated_tx_azimuth_deg
    = normalize_angle(
        rx_initial_gimbal_deg
        + uwb_raw_azimuth_deg
      )
```

사용자 정의 최종 정렬 오차:

```text
final_alignment_error_deg
    = normalize_angle(
        estimated_tx_azimuth_deg
        - tx_ground_truth_azimuth_deg
      )
```

절대오차가 필요할 경우 분석 단계에서 계산한다.

```text
absolute_alignment_error_deg
    = abs(final_alignment_error_deg)
```

중요:

- 본 문서에서 `final_alignment_error_deg`는 엔코더로 측정한 실제 물리적 최종 짐벌 오차가 아니다.
- UWB 상대방위각과 Rx 초기 짐벌각으로 계산한 논리적 Tx 추정방위각과 실제 Tx 방위각의 차이다.
- 코드와 로그에서 이 의미를 변경하지 않는다.

#### 8단계: 짐벌 명령 생성 및 적용

정렬 목표각은 추정 Tx 절대방위각이다.

```text
requested_gimbal_command_deg
    = estimated_tx_azimuth_deg
```

서보 허용 범위를 적용한다.

```text
applied_gimbal_command_deg
    = clamp(
        requested_gimbal_command_deg,
        servo_min_angle_deg,
        servo_max_angle_deg
      )
```

클리핑 여부:

```text
servo_clipped
    = requested_gimbal_command_deg
      != applied_gimbal_command_deg
```

필수 로그 필드인 `gimbal_command_deg`에는 실제 서보에 전달한 클리핑 후 명령각을 저장한다.

```text
gimbal_command_deg
    = applied_gimbal_command_deg
```

#### 9단계: QR 인식 또는 타임아웃

QR 인식 제한시간은 설정값 `qr_success_timeout_s`로 지정한다.

QR 인식 시간의 시작점은 첫 유효 UWB 데이터 수신 시각이다.

```text
qr_recognition_time_ms
    = qr_detected_time
      - first_valid_uwb_time
```

성공 조건:

1. `qr_success_timeout_s` 이내에 QR이 검출되어야 한다.
2. QR 디코딩이 성공해야 한다.
3. QR 내용은 비교하지 않고 QR 검출 여부만 판정한다.

```text
qr_success = true
```

QR 제한시간 내에 성공하지 못하면:

```text
qr_success = false
qr_recognition_time_ms = null
```

QR 실패 시 타임아웃 값을 인식 시간으로 저장하지 않는다.

#### 10단계: 결과 저장

결과는 한 시행당 한 행으로 저장한다.

파일 저장이 완료되기 전에는 다음 시행으로 넘어가지 않는다.

#### 11단계: 0° 복귀

결과 저장 후 짐벌을 다시 `0°`로 이동시킨다.

다음 시행도 다시 0° 안정화 단계부터 시작한다.

---

## 9. 필수 설정값

권장 설정 파일 이름:

```text
config/static_alignment.yaml
```

예시:

```yaml
experiment:
  name: static_alignment

  distances_m:
    - 1.0
    - 2.0
    - 3.0

  tx_azimuths_deg:
    - -60.0
    - 60.0

  negative_side:
    initial_start_deg: -60.0
    initial_end_deg: 0.0
    initial_step_deg: 2.0

  positive_side:
    initial_start_deg: 60.0
    initial_end_deg: 0.0
    initial_step_deg: -2.0

  repetitions_per_angle: 3
  randomize_trial_order: true
  random_seed: 20260721

timing:
  zero_settle_time_s: 1.0
  initial_settle_time_s: 1.0
  uwb_receive_timeout_s: 1.0
  qr_success_timeout_s: 0.2

servo:
  zero_angle_deg: 0.0
  min_angle_deg: -90.0
  max_angle_deg: 90.0

logging:
  output_directory: "data/static_alignment"
  summary_filename: "static_alignment_results.csv"
  trial_plan_filename: "static_alignment_trial_plan.csv"
  flush_after_each_trial: true
```

모든 시간과 각도 제한값은 코드에 하드코딩하지 않고 설정 파일에서 변경 가능해야 한다.

---

## 10. 필수 결과 로그

### 10.1 사용자 결정에 따른 핵심 필드

다음 필드는 반드시 저장한다.

| 필드명 | 타입 | 단위 | 의미 |
|---|---:|---:|---|
| `uwb_raw_azimuth_deg` | float/null | degree | 현재 초기 짐벌 방향을 기준으로 처음 수신한 유효 UWB 상대방위각 |
| `rx_initial_gimbal_deg` | float | degree | 해당 시행 시작 시 Rx 짐벌 초기각 |
| `gimbal_command_deg` | float/null | degree | 클리핑 후 실제 서보에 전달한 절대 목표각 |
| `tx_ground_truth_azimuth_deg` | float | degree | 물리적으로 설치된 실제 Tx 방위각 |
| `final_alignment_error_deg` | float/null | degree | `initial + UWB 상대각`으로 계산한 Tx 추정방위각과 실제 Tx 방위각의 부호 있는 차이 |
| `qr_success` | bool | - | 설정된 제한시간 내 올바른 QR 문자열을 인식했는지 여부 |
| `qr_recognition_time_ms` | float/null | ms | 첫 유효 UWB 수신부터 QR 성공까지의 시간. 실패 또는 UWB 누락 시 null |
| `uwb_message_missing` | bool | - | 설정된 UWB 수신 제한시간 내 유효 메시지를 받지 못했는지 여부 |
| `servo_clipped` | bool | - | 계산된 목표각이 서보 허용범위를 벗어나 제한되었는지 여부 |
| `distance_m` | float | m | 현재 실험 거리 |

### 10.2 재현성과 운영을 위한 필수 메타데이터

다음 필드는 실험을 재현하고 중단 후 재개하기 위해 함께 저장한다.

| 필드명 | 타입 | 의미 |
|---|---:|---|
| `trial_id` | string | 시행 고유 식별자 |
| `block_id` | string | 거리와 Tx 위치 조합 식별자 |
| `sequence_index` | int | 실제 실행 순서 |
| `repetition_index` | int | 동일 초기각 조건의 반복 번호, 1~3 |
| `random_seed` | int | 시행 순서를 만든 난수 시드 |
| `trial_status` | string | `success`, `qr_timeout`, `uwb_timeout`, `error` 중 하나 |
| `started_at` | string | 사람이 읽을 수 있는 시행 시작 시각 |
| `finished_at` | string | 시행 종료 시각 |
| `error_message` | string/null | 예외 또는 하드웨어 오류 설명 |

### 10.3 권장 보조 필드

다음 필드는 분석과 디버깅을 위해 저장을 권장한다.

| 필드명 | 타입 | 의미 |
|---|---:|---|
| `estimated_tx_azimuth_deg` | float/null | 초기각과 UWB 상대각으로 계산한 Tx 추정 절대방위각 |
| `requested_gimbal_command_deg` | float/null | 클리핑 전 계산된 절대 목표각 |
| `qr_timeout_s` | float | 해당 시행에 적용한 QR 성공 제한시간 |
| `uwb_timeout_s` | float | 해당 시행에 적용한 UWB 수신 제한시간 |
| `zero_settle_time_s` | float | 적용된 0° 안정화 시간 |
| `initial_settle_time_s` | float | 적용된 초기각 안정화 시간 |

---

## 11. CSV 예시

```csv
trial_id,block_id,sequence_index,repetition_index,random_seed,distance_m,tx_ground_truth_azimuth_deg,rx_initial_gimbal_deg,uwb_raw_azimuth_deg,estimated_tx_azimuth_deg,requested_gimbal_command_deg,gimbal_command_deg,final_alignment_error_deg,qr_success,qr_recognition_time_ms,uwb_message_missing,servo_clipped,trial_status,started_at,finished_at,error_message
static_d2_tx-60_idx017_rep2,d2_tx-60,17,2,20260721,2.0,-60.0,-20.0,-41.3,-61.3,-61.3,-61.3,-1.3,true,824.0,false,false,success,2026-07-21T15:10:00+09:00,2026-07-21T15:10:02+09:00,
```

UWB 누락 예시:

```csv
static_d2_tx-60_idx018_rep1,d2_tx-60,18,1,20260721,2.0,-60.0,-10.0,,,,,,false,,true,false,uwb_timeout,2026-07-21T15:10:05+09:00,2026-07-21T15:10:07+09:00,
```

---

## 12. 시간 측정 기준

모든 경과시간 계산에는 시스템 벽시계가 아니라 monotonic clock을 사용한다.

Python 권장:

```python
time.monotonic_ns()
```

QR 인식 시간:

```python
qr_recognition_time_ms = (
    qr_detected_monotonic_ns
    - first_valid_uwb_monotonic_ns
) / 1_000_000
```

사람이 읽는 로그 시각은 별도로 ISO 8601 형식으로 저장한다.

```text
2026-07-21T15:10:00+09:00
```

---

## 13. 권장 모듈 구조

```text
static_alignment/
├── config/
│   └── static_alignment.yaml
├── src/
│   ├── app.py
│   ├── config_loader.py
│   ├── trial_plan.py
│   ├── experiment_runner.py
│   ├── gimbal_controller.py
│   ├── uwb_receiver.py
│   ├── qr_detector.py
│   ├── angle_utils.py
│   ├── result_logger.py
│   └── models.py
├── tests/
│   ├── test_angle_utils.py
│   ├── test_trial_plan.py
│   ├── test_alignment_calculation.py
│   ├── test_timeout_handling.py
│   └── test_result_logger.py
└── data/
    └── static_alignment/
```

기존 프로젝트 구조가 이미 있다면 동일한 책임 분리를 유지하되 현재 구조에 맞춰 파일 위치를 조정한다.

---

## 14. 핵심 함수 요구사항

### 14.1 각도 정규화

```python
def normalize_angle(angle_deg: float) -> float:
    """Return angle in [-180, 180)."""
```

검증 예시:

```text
normalize_angle(180)   == -180
normalize_angle(181)   == -179
normalize_angle(-181)  == 179
normalize_angle(360)   == 0
```

### 14.2 Tx 추정방위각

```python
def estimate_tx_azimuth(
    initial_gimbal_deg: float,
    uwb_relative_azimuth_deg: float,
) -> float:
    return normalize_angle(
        initial_gimbal_deg + uwb_relative_azimuth_deg
    )
```

### 14.3 최종 정렬 오차

```python
def calculate_final_alignment_error(
    estimated_tx_azimuth_deg: float,
    tx_ground_truth_azimuth_deg: float,
) -> float:
    return normalize_angle(
        estimated_tx_azimuth_deg
        - tx_ground_truth_azimuth_deg
    )
```

### 14.4 서보 클리핑

```python
def clamp_servo_command(
    requested_deg: float,
    min_deg: float,
    max_deg: float,
) -> tuple[float, bool]:
    applied_deg = min(max(requested_deg, min_deg), max_deg)
    clipped = applied_deg != requested_deg
    return applied_deg, clipped
```

### 14.5 시행 계획 생성

```python
def build_trial_plan(
    initial_angles_deg: list[float],
    repetitions_per_angle: int,
    randomize: bool,
    random_seed: int,
) -> list[TrialSpec]:
    ...
```

### 14.6 단일 시행 실행

```python
def run_trial(
    trial: TrialSpec,
    config: StaticAlignmentConfig,
) -> TrialResult:
    ...
```

---

## 15. 상태 머신 요구사항

권장 상태 열거형:

```python
class TrialState(Enum):
    MOVE_TO_ZERO = auto()
    WAIT_ZERO_SETTLE = auto()
    MOVE_TO_INITIAL = auto()
    WAIT_INITIAL_SETTLE = auto()
    RESET_TRIAL_STATE = auto()
    WAIT_FOR_FIRST_VALID_UWB = auto()
    COMPUTE_ALIGNMENT = auto()
    COMMAND_GIMBAL = auto()
    WAIT_FOR_QR_OR_TIMEOUT = auto()
    SAVE_RESULT = auto()
    RETURN_TO_ZERO = auto()
    COMPLETE = auto()
    FAILED = auto()
```

상태 전환과 오류를 로그에 남겨야 한다.

예외가 발생해도 가능한 경우 다음 동작을 수행한다.

```text
1. 현재 시행 오류 결과 저장
2. 짐벌 0° 복귀 시도
3. 안전하게 다음 시행 또는 프로그램 종료
```

---

## 16. 카메라 및 QR 인식 요구사항

1. 실험 시작 전에 카메라 동작 테스트 기능을 제공한다.
2. 카메라 프레임 수신 여부를 확인할 수 있어야 한다.
3. QR 디코더가 QR 코드를 올바르게 검출하는지 확인할 수 있어야 한다.
4. QR 인식 성공 제한시간은 설정 파일에서 변경 가능해야 한다.
5. 첫 UWB 수신 이전에 검출된 QR 결과는 현재 시행의 성공으로 사용하지 않는다.
6. QR 성공 시 최초 성공 시각만 기록한다.
7. 같은 시행에서 QR이 여러 번 검출되어도 결과는 한 번만 저장한다.

권장 카메라 테스트 실행 예시:

```bash
python -m src.app camera-test
```

---

## 17. UWB 수신 요구사항

1. 시행 시작 전에 남아 있던 메시지를 폐기한다.
2. 현재 시행에서 처음 수신한 유효 메시지를 정렬 계산에 사용한다.
3. 첫 유효 메시지 수신 시각을 정확히 기록한다.
4. UWB 수신 제한시간은 설정 파일에서 변경 가능해야 한다.
5. 제한시간 내 유효 메시지가 없으면 `uwb_message_missing = true`로 처리한다.
6. 파싱 오류가 발생해도 프로그램 전체가 중단되지 않도록 한다.
7. 원시 방위각 값을 임의로 필터링하거나 평균내지 않는다.
8. 추후 필터 기능을 추가하더라도 기본 정적 실험 모드에서는 원시값을 그대로 사용한다.

---

## 18. 결과 파일 안전성

1. 각 시행 결과 저장 후 즉시 파일을 flush한다.
2. 가능하면 `fsync`까지 수행해 전원 손실 시 데이터 유실을 줄인다.
3. 기존 결과 파일이 있으면 덮어쓰지 않고 이어쓰기 또는 새 run 디렉터리를 사용한다.
4. 동일 `trial_id`가 중복 저장되지 않도록 한다.
5. 프로그램 재시작 시 완료된 `trial_id`를 읽어 미완료 시행만 실행할 수 있어야 한다.
6. 실행에 사용한 설정 파일 복사본을 결과 디렉터리에 저장한다.
7. 실제 시행 순서 파일을 결과 디렉터리에 저장한다.

권장 결과 디렉터리:

```text
data/static_alignment/
└── run_20260721_151000/
    ├── static_alignment.yaml
    ├── static_alignment_trial_plan.csv
    ├── static_alignment_results.csv
    └── experiment.log
```

---

## 19. 오류 처리 규칙

### UWB 타임아웃

```text
uwb_message_missing = true
qr_success = false
qr_recognition_time_ms = null
gimbal_command_deg = null
final_alignment_error_deg = null
trial_status = "uwb_timeout"
```

### QR 타임아웃

```text
uwb_message_missing = false
qr_success = false
qr_recognition_time_ms = null
trial_status = "qr_timeout"
```

### 서보 클리핑

서보 클리핑이 발생해도 시행은 계속한다.

```text
servo_clipped = true
gimbal_command_deg = clipped command
```

QR 결과까지 평가한 뒤 정상적으로 로그를 저장한다.

### 예외 발생

```text
trial_status = "error"
error_message = exception summary
```

오류 결과를 저장한 후 0° 복귀를 시도한다.

---

## 20. 테스트 요구사항

### 20.1 시행 수 검증

각 블록:

```text
31 angles × 3 repetitions = 93 trials
```

전체:

```text
3 distances × 2 Tx positions × 93 trials = 558 trials
```

### 20.2 초기각 목록 검증

음의 조건:

```text
[-60, -58, ..., -2, 0]
```

양의 조건:

```text
[60, 58, ..., 2, 0]
```

각 목록의 길이는 정확히 31이어야 한다.

### 20.3 반복 횟수 검증

무작위 셔플 후에도 각 초기각이 정확히 3번 존재해야 한다.

### 20.4 계산 검증 예시

입력:

```text
tx_ground_truth_azimuth_deg = -60
rx_initial_gimbal_deg = -20
uwb_raw_azimuth_deg = -41.3
```

기대 결과:

```text
estimated_tx_azimuth_deg = -61.3
final_alignment_error_deg = -1.3
requested_gimbal_command_deg = -61.3
```

입력:

```text
tx_ground_truth_azimuth_deg = 60
rx_initial_gimbal_deg = 20
uwb_raw_azimuth_deg = 39.0
```

기대 결과:

```text
estimated_tx_azimuth_deg = 59.0
final_alignment_error_deg = -1.0
requested_gimbal_command_deg = 59.0
```

### 20.5 타이밍 검증

입력:

```text
first_valid_uwb_time = 10.000 s
qr_detected_time = 10.824 s
```

기대 결과:

```text
qr_recognition_time_ms = 824
```

### 20.6 클리핑 검증

입력:

```text
requested_gimbal_command_deg = 95
servo range = [-90, 90]
```

기대 결과:

```text
gimbal_command_deg = 90
servo_clipped = true
```

---

## 21. 완료 기준

다음 조건을 모두 만족하면 정적 alignment 실험 코드 구현이 완료된 것으로 본다.

1. 카메라 테스트 모드가 정상 동작한다.
2. 거리와 Tx 위치를 설정하여 실험 블록을 시작할 수 있다.
3. 각 블록에서 31개 초기각을 각각 3회 수행한다.
4. 난수 시드에 따라 시행 순서를 재현할 수 있다.
5. 각 시행은 반드시 `0° → 초기각 → 정렬 → 결과 저장 → 0°` 순서로 실행된다.
6. `±60°` 경계 조건이 제외되지 않는다.
7. QR 성공 제한시간을 설정 파일에서 변경할 수 있다.
8. QR 인식 시간은 첫 유효 UWB 수신부터 측정된다.
9. UWB 누락, QR 타임아웃, 서보 클리핑이 구분되어 저장된다.
10. 거리값이 모든 결과 행에 저장된다.
11. 최종 정렬 오차가 본 문서의 논리적 정의대로 계산된다.
12. 총 558개 시행을 중단 후 재개 가능한 형태로 실행할 수 있다.
13. 각 시행 결과가 즉시 저장되어 프로그램 종료 시에도 이전 결과가 유지된다.

---

## 22. 구현 시 금지사항

1. `±60°` 조건을 센서 경계라는 이유로 자동 제외하지 않는다.
2. 각도를 코드 내부에만 하드코딩하지 않는다.
3. QR 제한시간을 코드에 고정하지 않는다.
4. UWB 수신 전의 QR 검출을 해당 시행 성공으로 처리하지 않는다.
5. QR 실패 시 제한시간 값을 QR 인식 시간으로 저장하지 않는다.
6. 원시 UWB 방위각에 자동 필터를 적용하지 않는다.
7. 서보 클리핑을 숨기거나 정상 명령으로 기록하지 않는다.
8. 실제 엔코더 측정값이 없는데 `final_gimbal_angle`이라는 물리적 실측값을 생성하지 않는다.
9. 0° 복귀와 초기각 이동 시간을 QR 인식 시간에 포함하지 않는다.
10. 결과 파일 저장 전에 다음 시행을 시작하지 않는다.
