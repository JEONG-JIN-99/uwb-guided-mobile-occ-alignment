# Dynamic Tracking 통합 분석

## 분석 대상 및 방법

거리별로 사용자가 지정한 다음 한 개의 실험만 사용했다.

- 1 m: `1m01` (`1m01_arc_chrony_test_vel_log_20260730_061840.csv`, rover time 0.100–21.001 s)
- 2 m: `2m02` (`2m02_arc_chrony_test_vel_log_20260730_061316.csv`, rover time 0.100–17.801 s)
- 3 m: `3m02` (`3m02_arc_chrony_test_vel_log_20260730_064124.csv`, rover time 0.100–27.401 s)

선택하지 않은 실험은 표, MAE, 그래프 및 해석에 포함하지 않았다. 각 실험의
로버 기록 시간 안에서 `sample_index=0`을 제외했으며, 개별 분석 지침과 같은
원형 각도 보간 및 좌표계 변환을 적용했다. Rx와 Tx MAE는 선택된 각 실험의
모든 유효 명령 표본에 대한 절대 원형 각도 오차의 평균이다.

`3m02`의 원본 로그에서 `color_not_detected`로 기록된 78개 프레임은 사용자의 실패 프레임 전수 검수에서 모두 빨간색 원이 존재하는 것으로 확인되어 논문용 인식률 집계에서는 성공으로 재분류했다. 원본 CSV와 개별 분석 문서는 수정하지 않았다.

## 거리별 결과

| Distance (m) | Total cycles | Successful detections | Color not detected | Camera timeouts | UWB unavailable | Other errors | Overall detection rate (%) | Conditional detection rate (%) | Rx MAE (deg) | Tx MAE (deg) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 104 | 102 | 0 | 2 | 0 | 0 | 98.08 | 100.00 | 10.47 | 3.68 |
| 2 | 88 | 86 | 0 | 2 | 0 | 0 | 97.73 | 100.00 | 6.32 | 6.91 |
| 3 | 136 | 135 | 0 | 1 | 0 | 0 | 99.26 | 100.00 | 4.61 | 3.25 |

수동 검수 재분류를 반영한 시스템 전체 인식률은 1 m
98.08%, 2 m 97.73%, 3 m
99.26%이며 세 조건의 차이는 최대
1.54%p였다. 정상 처리 조건 인식률은 세 거리 모두
100.00%였다. 3 m에서는 전체 136주기 중
135건을 성공으로 분류했고, 남은 실패는 카메라 타임아웃
1건(0.74%)이었다.

## Rx 정렬 결과

![Rx alignment by distance](rx_alignment_by_distance.png)

- 1 m: MAE 10.47°, maximum absolute error 28.12°, ideal/aligned change +89.0°/+79.5°
- 2 m: MAE 6.32°, maximum absolute error 19.13°, ideal/aligned change +81.7°/+84.0°
- 3 m: MAE 4.61°, maximum absolute error 9.61°, ideal/aligned change +84.3°/+77.1°

선택된 실험의 Rx MAE는 1 m에서 10.47°, 2 m에서
6.32°, 3 m에서 4.61°로 감소했다.
따라서 이 세 실행에서는 거리가 멀수록 Rx 명령 기반 정렬 오차가 커지는 추세가
관찰되지 않았다. 다만 1 m의 최대 오차가 28.12°로 가장
크고 분석 구간의 이상적 방향 변화보다 정렬 방향 변화가 작아, 일부 구간의
오프셋 또는 추종 부족이 전체 MAE에 반영되었다.

## Tx 정렬 결과

![Tx alignment by distance](tx_alignment_by_distance.png)

- 1 m: MAE 3.68°, maximum absolute error 13.65°, ideal/aligned change +88.3°/+88.1°
- 2 m: MAE 6.91°, maximum absolute error 23.99°, ideal/aligned change +82.7°/+67.4°
- 3 m: MAE 3.25°, maximum absolute error 6.32°, ideal/aligned change +85.0°/+80.4°

Tx MAE는 2 m에서 6.91°로 가장 컸고, 3 m에서
3.25°로 가장 작았다. 특히 2 m에서는 최대 절대 오차가
23.99°이고 정렬 방향의 전체 변화량이 이상적 방향보다
작아, 선택된 세 실행 중 추종 차이가 가장 크게 나타났다. 3 m의 Tx는
MAE와 최대 오차가 모두 가장 작아 두 선의 추세가 가장 가깝게 나타났다.

## 종합 해석과 한계

수동 검수 재분류를 반영한 선택 데이터에서는 세 거리의 빨간색 인식률이 모두 97% 이상이었고, 거리 증가에 따른 일관된 인식률 저하는 나타나지 않았다.
Rx와 Tx의 명령 기반 방향 MAE도 거리 증가에 따라 일관되게 커지지 않았다.

각 거리에서 한 개의 실험만 선택했으므로 반복 간 변동성이나 통계적 유의성은
평가하지 않았다. 3 m 인식률은 원본 자동 판정 상태가 아니라 실패 프레임의 수동 시각 검수를 반영한 값이므로, 자동 판정 상태만 사용한 1 m 및 2 m와 평가 절차가 다르다는 점을 함께 고려해야 한다. 또한 거리별 실행 시간이 서로
다르기 때문에 같은 elapsed time이 동일한 궤적 진행 위치를 뜻하지 않을 수
있다. 그래프의 정렬 방향은
엔코더로 측정한 물리적 방향이 아니라 실제 적용된 짐벌 명령으로 계산하거나
기록한 방향이므로, 기구부의 실제 추종 오차까지 포함한 값으로 해석해서는
안 된다.
