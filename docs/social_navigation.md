# 공식 RVO2의 진행 가능성 기반 보행 제어

랜덤 창고·병원 시나리오는 공식 [ORCA/RVO2](https://github.com/snape/RVO2)를 사용한다.
코어는 `third_party/rvo2/src/`에 고정 버전으로 포함했다.
`UPSTREAM.json`에 원본 commit, 라이선스, 수정 내용을 기록하고,
`LOCAL_CHANGES.patch`로 원본과의 차이를 검토할 수 있다.
Python에서 ORCA 제약식이나 선형 계획법을 다시 구현하지 않는다.

## 계층과 데이터 흐름

```text
NavMesh 경로 + 모든 agent의 관측된 world 위치·속도·몸 방향
  → behavior/stiffness.py        고정 μ → 사람별 예측 시간·추가 안전 반경
  → behavior/responsibility.py   앞·옆·뒤 방향 → 보행자 쌍의 회피 책임
  → behavior/preference.py       목적지 방향의 preferred velocity
  → avoidance/rvo2.py            공식 RVO2로 기본 회피 속도와 제약 생성
  → behavior/progress.py         같은 제약에서 진행·옆 회피 속도 조회
  → behavior/yielding.py         지속적인 진행 불가 상태에서만 교착 회복 시도
  → avoidance/rvo2.py            양보 의도도 공식 ORCA 제약으로 제한
  → runtime/animation.py         native NavMesh의 모서리를 유지한 이동 preview
  → runtime/isaac.py             관측 및 저장 어댑터
  → pedestrians.py              AnimationGraph 속도·이동 방향 적용

context/walkable.py              NavMesh 선분 검사 및 밀도·통로 폭 기록
```

`navigation/controller.py`는 위 계층을 조합한다. 정책을 변경하려면 `behavior/`,
밀도 정의를 변경하려면 `context/`, ROS 전송을 변경하려면 `runtime/publisher.py`,
저장 형식을 변경하려면 `runtime/recording.py`를 수정한다.
`config.py`는 ROS launch에서 읽으므로 NumPy/SciPy/Isaac 의존성을 갖지 않는다.
`state.py`는 실제 위치 변화로부터 world 속도를 계산한다. `/cmd_vel`을 보행자
회피 판단에 사용하지 않는다.

## 방향별 회피 책임

창고·병원 랜덤 프리셋은 Social Stiffness를 유지하고 방향별 책임을 함께 사용한다.
`yielding_enabled: false`는 그대로다. 방향별 책임은 PRISM의 세 파라미터에 포함되지
않는 자체 확장이다. 목적지 방향인 preferred velocity와 물리 원형 몸체는 바꾸지 않는다.

`behavior/responsibility.py`는 각 사람의 몸 방향과 상대 위치 사이 각도 θ로
`w = 0.1 + 0.9 × (1 + cos θ) / 2`를 계산한다. 앞 1, 옆 0.55, 뒤 0.1이며
`α_ij = w_i(j) / (w_i(j) + w_j(i))`로 나눈다. 서로 마주 보면 0.5/0.5,
같은 방향에서 앞뒤로 접근하면 앞사람 약 0.091, 뒤 사람 약 0.909다. 추월 여부를
별도로 분류하지 않고 모든 보행자 쌍에 적용한다. 이것은 확률이 아닌 보정량의 비율이다.

Isaac Character의 관측 world quaternion을 사용하므로 정지 중에도 몸 방향이 남는다.
People 캐릭터의 정면은 local -Y이므로 quaternion으로 이 축을 world로 회전시킨다.
로봇처럼 local +X의 yaw를 그대로 쓰면 90도 틀린 앞뒤가 되므로 사용하지 않는다.
몸 yaw와 책임 비율을 `directional_smoothing_s`(0.2초)의 지수 필터로 완만하게
바꾼다. 각 제어 관측에서 한 번 갱신하고, 같은 관측에서 여러 ORCA solve가 필요해도
동일한 책임을 재사용한다. 비율 합은 항상 1이다.

몸 표면 간 거리(중심 거리 − 두 물리 반경)가 0.45m에서 0.15m로 줄어드는 동안
smoothstep으로 0.5/0.5를 복원한다. 0.15m 이하는 필터 지연 없이 동등한 책임이다.
μ의 추가 안전 반경은 이 거리에 포함하지 않는다. 이 값들은 행동 조절을 위한 초기
설정이며 사람 행동 실험으로 검증된 추정값은 아니다.

RVO2의 속도 장애물 기하 계산과 LP는 유지하고, 사람별 ORCA 경계의
`line.point = velocity + α × u`에서 기존 0.5를 바꾼다. **한쪽만 상대를 탐지한 경우**
탐지한 쪽은 1.0을 부담한다. 양쪽의 현재 neighbor 목록을 사용하도록 C++의 neighbor
조회와 velocity 계산 단계를 분리했다. Teleop 로봇에 대해서는 방향과 상관없이
보행자 책임 1.0을 유지하고 관측 로봇 속도를 변경하지 않는다.

`directional_responsibility_enabled: false`로 기존 책임 배분으로 돌아간다.
`directional_rear_weight: 1.0`은 방향에 따른 차이를 없앤다. 단, 방향별 옵션이 켜져
있으면 한쪽 탐지 시 책임 1.0 처리는 유지된다. Social Stiffness와 독립적으로 조절된다.
실제 적용 책임은 `RVO2Solver.agent_responsibility(i, j)`로 읽는다. 미탐지 상대는
`None`이다. 스키마와 실행 보고서에 설정, 매 제어 관측에는 필터된 몸 방향을 기록한다.

쌍의 책임 비율이 합계 1이어도 서로 다른 μ의 예측 시간, 좁은 공간의 제약 불가능,
실제 애니메이션 추종 지연을 포함한 모든 상황의 무충돌을 보장하는 것은 아니다.

## 1. ORCA 기본 회피

보행자끼리는 기본 ORCA의 0.5 / 0.5 책임을 방향별로 조정한다. 이 확장은
`directional_responsibility_enabled`로 켜고 끈다. 실제 위치와 실제 속도를
매 50Hz 제어 틱마다 모두 관측한 뒤 한꺼번에 solver에 전달한다. NavMesh 래스터의
이동 가능/불가능 경계도 RVO2 정적 장애물로 전달한다. 래스터는 이미 보행자 중심
반경만큼 erode된 영역이므로 정적 장애물 반경에는 별도의 0.1m 여유(`obstacle_clearance_m`)를 둔다.
사람 간 제약에는 `tracking_margin_m`(기본 사람별 0.02m)을 추가해 실제 보행
애니메이션이 명령 방향을 즉시 따라가지 못하는 오차를 완충한다. 물리 반경을
바꾸지는 않으며 기록되는 `radius`는 원래 몸체 반경이다.

사람 간 중심 거리 제약은 `r_i + r_j + safety_margin_m + 2 × tracking_margin_m + b_i + b_j`이다.
기본 `safety_margin_m=0.02`, `tracking_margin_m=0.02`로 두 사람 사이의 추가 여유는
총 0.06m다. μ 안전 여유가 없는 반경 0.3m 두 사람은 중심 거리 0.66m에서 스쳐 지나갈 수 있다.
기본 `b_i=0.15μ_i`로 두 사람 모두 μ=1이면 제약은 0.96m가 된다.
이전 설정의 1.08m보다 좁다. 몸체 사이 추가 여유를 없애려면 두 항목 모두 0으로
설정할 수 있다. 실제 애니메이션의 몸 흔들림·속도 추종 때문에 관측 거리가 ORCA의
이상적인 거리 제약과 일치하지 않을 수 있다.

`avoidance/clearance.py`가 ORCA와 양보 판단에 사용할 pair 여유를 함께 계산한다.
양보 판단의 별도 0.15m 가산과 대기 지점 검사의 고정 0.5m 상대 반경을 없앴다.
상대의 실제 몸체 반경을 사용하며 대기 지점 도착 오차 0.35m만 별도로 고려한다.

정확히 대칭인 정면 만남이 멈춤 상태로 수렴하지 않도록 preferred velocity에
작은 오른쪽 편향(기본 0.015m/s)을 둔다. 이는 양보 우선순위를 부여하지 않는다.

## 2. Teleop 로봇 회피

로봇의 관측 위치·속도·반경도 같은 solver에 넣는다. 로봇은 externally controlled
agent이므로 solver가 로봇 속도를 바꾸거나 회피를 가정하지 않는다. 보행자→로봇
제약의 책임은 1.0이며, 이 부분만 공식 코드에 작은 확장으로 추가했다.
로봇과 사람 사이 추가 여유는 `robot_safety_margin_m`(기본 0.33m)로 분리한다.
기본 반경의 사람(0.3m)과 A2(0.55m)의 ORCA 중심 거리 제약은 `1.18m + b_i`다.
외부 제어 로봇의 solver 반경에 남은 pair 여유를 할당하며 기록되는 물리 반경은
원래 값이다. 사람 간 간격을 좁혀도 로봇과의 pair 여유를 함께 줄이지 않는다.
정지 로봇 앞에서 멈춰 있는 국소 최소점은 preferred 방향의 옆 우회로 해결한다.
로봇의 실제 움직임은 계속 조이스틱과 A2 제어기가 결정한다.

## 3. ORCA 진행 가능성 판단

`behavior/progress.py`가 기본 ORCA 출력과 기존 제약에서 안전하게 진행하거나 옆으로
피할 수 있는지 확인한다. 한 번 느린 속도가 나왔다고 즉시 양보하지 않는다.
선택된 속도가 충분하지 않으면 RVO2의 원래 `linearProgram2`로 최대 전진 속도와
좌우 방향 후보를 읽기 전용으로 조회한다. 기존 선택이 정체되고 유용한 다른
후보가 있으면 같은 제약을 만족하는 그 후보를 실제 이동 속도로 사용한다. 별도의 미래 충돌 예측기나 ORCA 수학을
추가하지 않는다. 각 후보는 같은 ORCA 제약·최대 속도·NavMesh 선분 검사를 만족해야 한다.

유용한 속도는 크기가 `max(progress_min_mps, 원하는 속도 × progress_min_fraction)`
이상이며 목적지 방향으로 뒤로 가지 않는 속도이다. 전진·감속·옆 이동을 허용한다.
현재 기본값은 0.08m/s와 15%이므로 원하는 속도 0.8m/s에서는 0.12m/s이다.
전진 속도 최대값은 공식 LP로 구하고, 좌우 탈출은 ±30/60/90도 후보로 검사한다.
이산적인 방향 후보 검사이므로 모든 우회 경로의 존재를 증명하는 전역 경로 계획은 아니다.

## 4. 지속적인 정체 후 양보

충돌이 예상되는 만남에서 유용한 안전 속도가 부족한 상태가 연속
`progress_hold_s`(기본 0.5초) 유지될 때 양보를 검토한다. 순간적인 감속이나
잠시 정지한 뒤 진행 가능한 상태가 되면 증거 시간을 즉시 초기화한다.
새 목적지나 도착 후 idle에도 이전 정체를 넘겨서 사용하지 않는다.

μ로 역할을 한 번 선택하고 만남 동안 유지한다. 양보자는 현재 위치에서 기다리거나
상대 경로를 비워 주는 대기 지점으로 간다. 좌우 및 앞뒤 대기 공간을 찾고 경로를
native NavMesh로 검증한다. 직선 거리 대신 실제 경로 길이로 가까운 대기 지점을
고르고, 다른 대기 중인 사람을 가로질러야 하는 경로는 제외한다. 선택한 역할의
대기 경로가 없고 반대 역할만 가능하면 실행 가능한 역할을 사용한다. 대기 공간이
없으면 고정된 역할을 유지하면서 공간을 다시 확인한다. 양보자가 자리를 비우는 동안 상대는 기다리며, 자리를
비우면 상대가 지나간다. 대기 공간이 없으면 `waiting_for_space`를 기록한다.
통과 후에는 현재 위치에서 원래 목적지로 native NavMesh 복귀 경로를 새로 구한다.
이미 양보 중인 사람을 다른 만남의 새 양보 역할에 중복 배정하지 않는다.

정지/양보는 preferred velocity에만 적용한다. 이후 ORCA를 다시 실행하므로 위험하면
정지 의도인 보행자도 회피할 수 있다. 양보 중인 명령을 기본 진행 불가 증거로
사용하지 않도록 먼저 원래 경로 의도의 ORCA 해를 평가한다.

지역 밀도와 통로 폭은 **기록용**이다. 밀도는 반경 `density_radius_m`(기본 1.5m)
안의 보행자 수 / 같은 원 안의 실제 NavMesh 이동 가능 면적이며 자기 자신을 포함한다.
로봇은 인원 수에 포함하지 않는다. 면적에서 벽·unknown을 제외하며 통로 폭은
NavMesh 경계 거리와 bake 반경으로 추정한다. 밀도나 폭은 양보 선택에 사용하지 않는다.

0.12m 래스터는 경계 근처의 실제 NavMesh 점을 막힌 셀로 분류할 수 있다.
이때 native NavMesh 투영과 원래 map/floor 허용 조건을 다시 확인한다. 허용 오차는
한 셀과 `조회 반경 − 몸체 반경` 중 작은 값으로 제한한다. 애니메이션에 사용하는
NavMesh 반경으로 투영했을 때 경계 밖 오차가 더 커지는 이동도 제외한다.
장애물이나 unknown을 임의로 free로 바꾸지 않는다.

`navigation/paths.py`는 실제 root가 경계에서 조금 벗어나도 시작점과 목표를 모두
native NavMesh에 투영해 경로를 조회한다. 실제 위치에서 투영된 시작점까지의
복귀 구간도 경로에 포함하며 위치를 순간이동시키지 않는다. 애니메이션에 긴
직선 목표만 전달하면 벽 모서리를 가로지를 수 있으므로 `runtime/animation.py`는
native 경로의 중간 모서리를 유지한다. 사람용 bake 최소 반경은 몸체 반경(기본
0.3m), 경로 조회 반경은 `몸체 반경 + animation_path_clearance_m`(기본 0.05m)로
기본 0.35m다. 로봇 task는 scene 복사본에서 0.55m로 조회하고, bake 최대 반경에
로봇 크기도 포함한다. 로봇 task를 켰다고 사람의 bake·조회 반경을 키우지 않는다.
창고의 사람용 occupancy map clearance도 0.35m이며, 로봇 task의 map clearance
0.65m는 별도로 적용한다. unknown 셀은 계속 막는다.

`runtime/animation.py`의 `AnimationCommands`가 이동 입력의 상태를 유지한다.
매 틱 0.7m 임시 목표를 다시 만들던 방식에서, 1.2m 이상 앞의 native 경로를
재사용하고 목표에 가까워지거나 회피 방향이 달라질 때 갱신하도록 바꿨다.
동일한 PathPoints와 Action을 매 틱 다시 쓰지 않는다. 일반적인 방향 재갱신
간격은 최소 0.12초이며, 기존 경로가 제약을 벗어나면 즉시 재검사한다.

속도 목표에는 가속 1.5m/s²·감속 3.0m/s², 방향 목표에는 3.14rad/s의 변화율을
적용한다. 이후 같은 관측의 공식 ORCA 제약으로 다시 투영하므로 제약이 더 강한
경우에는 이 편안한 변화율을 넘길 수 있다. 이 횟수는 safety_overrides에 기록한다.
안전 영역이 비어 있는 상태는 해결됐다고 표시하지 않고 infeasible로 유지하며,
경계 회복용 제한 입력은 native 이동 검사를 거친다. 무충돌 보장을 추가한 것은 아니다.

Walk는 m/s가 아닌 0~1 애니메이션 입력으로 취급한다. 기본 클립 기준 속도
`animation_walk_reference_mps`(1.0)는 이동 상한이 아니다. 선택 속도가 기준
이하면 기존 Walk blend를 유지하고, 초과하면 Walk=1에서 애니메이션 시간을
`선택 속도 / 기준 속도`배 진행해 발 동작과 root motion을 함께 빠르게 한다.
`runtime/clock.py`는 확장 로드 전에 자동 갱신을 끄고, 렌더 프레임마다 경과한
시뮬레이션 시간 dt에 각 보행자의 배속을 곱해 캐릭터를 명시적으로 갱신한다.
이후 그래프 관리자의 정상 dt 갱신으로 자세를 장면에 반영한다. 이미 명시적으로
갱신한 캐릭터는 해당 프레임의 배속 dt를 유지한다. 물리 dt나 벽시계 시간을 쓰지 않는다.
정지하면 배속은 1로 돌아와 Idle과 LookAround를 정상 속도로 재생한다.
`speed_range_mps`로 선택한 목표가 1m/s에서 잘리지 않으며 ORCA의 감속은 유지한다.
클립 특성·회전·경로 끝 감속에 따라 실제 측정 속도는 목표값과 차이가 있다.
작은 속도 hysteresis에 더해 `animation_idle_delay_s`
(기본 0.2초) 동안 정지가 지속될 때만 Action을 Idle로 바꾼다. 이 지연은
이동 허용 시간이 아니다. 정지 입력에서는 Walk=0과 현재 위치의 PathPoints를
즉시 전달해 이전 전진 경로를 지운다. 재출발하면 현재 위치에서 경로를 다시 쓴다.

경로 후보는 긴 preview가 제약을 벗어나더라도 더 짧은 native 경로를 검사한다.
필요하면 같은 ORCA 제약이 보정한 방향으로 최대 3번 재조회하며, 모든 후보가
실패하면 정지한다. preview 끝점에는 원래 목적지의 0.3m 도착 판정을 적용하지
않는다. 새 통과·양보 정책을 추가한 것은 아니다. `recovered_paths`는 이 재검사로
이동을 유지한 횟수, `action_switches`는 실제 Walk/Idle 명령 전환 횟수다. 실제 속도는
여전히 관측 위치의 시간 차분으로 저장한다. 원래 ORCA 속도는 orca_velocity,
실행 입력은 commanded_velocity, 경로 갱신·제약 상태는 animation에 구분해 기록한다.

## 5. 일반 회피 성향과 정체 회복의 분리

각 보행자의 μ∈[0,1]을 에피소드 생성 시 한 번 샘플링하고 에피소드 안에서 고정한다.
`behavior/stiffness.py`가 일반적인 마주침에 사용할 사람별 ORCA 파라미터를 계산한다.

```text
time_horizon_i = 2 + 8 * mu_i seconds
personal_buffer_i = 0.15 * mu_i meters
neighbor_distance_i = 2 + 8 * mu_i meters
orca_radius_i = body_radius_i + safety_margin / 2 + tracking_margin + personal_buffer_i
```

μ가 높으면 충돌을 더 긴 시간 범위에서 예상하고 사람·로봇과의 안전 여유를 크게 둔다.
두 사람 사이에는 각각의 buffer가 합산된다. 몸체 collider, NavMesh bake·벽 여유,
관측 속도, 희망 보행 속도를 키우지 않는다. 이웃 탐색 거리도 2~10m로 사람마다
보간해 공식 RVO2 `addAgent`에 전달한다. 기존 `neighbor_distance_m`(6m)는 성향 기능을
껐을 때와 외부 제어 로봇의 기본값으로만 사용한다. 공식 RVO2의 사람별 time horizon,
neighbor distance와 `setAgentRadius`를 사용하며 ORCA 수식을 다시 구현하지 않는다.

`yielding_enabled: false`로 해도 이 일반 회피 성향은 작동한다. 정체 회복 계층은
기존과 같이 충돌 위험과 진행 가능한 ORCA 후보 부족이 0.5초 지속될 때만 역할을 고른다.
양보 전략이 선택된 보행자 쌍에서 `P(i가 양보)=μ_i/(μ_i+μ_j)`이다. 둘 다 0이면
0.5이다. 큰 μ가 더 자주 양보하며 반드시 항상 양보하는 규칙은 아니다.
움직이는 로봇에 대한 선택적 대기는 μ_i 확률로 선택한다. 선택적 대기를 하지
않더라도 ORCA의 보행자 회피 책임 1.0은 유지한다. 정지 로봇은 대기 대신 우회한다.

[PRISM v1 4.1절](https://arxiv.org/pdf/2609.18125#page=5)의
`Phi_i = mu_i * Phi_cautious + (1 - mu_i) * Phi_aggressive`와 세 항목의 끝점에 맞췄다.
이전 구현에서 누락된 이웃 거리 보간을 수정했다. PRISM의 encoder, RnC 학습,
HEIGHT 로봇 정책 전체를 재현한 것은 아니다.

본문은 Safety Buffer의 0~0.15m 범위는 명시하지만, 반경과 pair 거리 중 어디에
더하는지까지 식으로 제시하지 않는다. 여기서는 각 사람의 ORCA 반경에 `b_i`를
더하고 두 사람 제약에는 `b_i + b_j`가 반영되는 규약을 사용한다. 이 선택은 원문의
명시된 식 자체가 아닌 구현 가정이다. 기존 tracking/safety/robot margin은 이 항과
별도인 Isaac 제어 여유다. 실제 적용 파라미터와 보조 규칙은 metadata에 구분해 저장한다.
μ가 모든 상황에서 더 큰 원호·먼저 양보를 보장하는 것은 아니다.

## 설정

`config/pedestrians/warehouse_random.yaml` 및 `hospital_random.yaml`:

```yaml
social_navigation:
  enabled: true
  yielding_enabled: true
  social_stiffness_enabled: true
  stiffness_time_horizon_min_s: 2.0
  stiffness_time_horizon_max_s: 10.0
  stiffness_buffer_max_m: 0.15
  stiffness_neighbor_distance_min_m: 2.0
  stiffness_neighbor_distance_max_m: 10.0
  mu_range: [0.0, 1.0]
  safety_margin_m: 0.02
  tracking_margin_m: 0.02   # 사람별, 두 사람 사이 총 여유는 0.06m
  robot_safety_margin_m: 0.33
  animation_path_clearance_m: 0.05
  animation_retarget_interval_s: 0.12
  animation_max_acceleration_mps2: 1.5
  animation_max_deceleration_mps2: 3.0
  animation_max_turn_rate_rps: 3.14
  animation_walk_reference_mps: 1.0
  animation_idle_delay_s: 0.2
  density_radius_m: 1.5     # 기록용
  progress_hold_s: 0.5
  progress_min_mps: 0.08
  progress_min_fraction: 0.15
```

성향별 ORCA만 비교하려면 `yielding_enabled: false`로 설정한다.
논문의 세 파라미터만 분리해서 검증하는 reference 시험에는 다음 설정을 함께 쓴다.
아래 값은 비교용이며 기본 Isaac 실행에는 추적 오차 여유와 보조 규칙이 남아 있다.

```yaml
social_navigation:
  enabled: true
  social_stiffness_enabled: true
  yielding_enabled: false
  directional_responsibility_enabled: false
  passing_bias_mps: 0.0
  stationary_robot_bypass_enabled: false
  safety_margin_m: 0.0
  tracking_margin_m: 0.0
  robot_safety_margin_m: 0.0
```

원문에 없는 우측 편향·정지 로봇 우회·정체 후 양보를 별도 옵션으로 꺼 검증한다.
이 설정으로도 NavMesh 경로/애니메이션/teleop 어댑터는 Isaac 고유 구현이므로 논문
시뮬레이터 전체의 동일 궤적 재현을 의미하지는 않는다.
고정 파라미터 ORCA와 비교하려면 성향·양보·방향별 책임 옵션을 모두 false로 설정한다.
`social_stiffness_enabled: false`일 때 기존 `time_horizon_s`(2.5초)와 기본 여유만 사용한다.
이때 이웃 거리는 기존 `neighbor_distance_m`(6m)로 복귀한다.
`social_stiffness_enabled: false`, `yielding_enabled: true` 조합은 기존 정체 회복만 유지한다.
성향별 파라미터는 `crowd_config.json`, `crowd_state_schema.json`의 `social_stiffness`에
사람별로 저장한다. 제어 틱에는 활성화 여부만 추가하며 고정 파라미터를 매번 재계산하지 않는다.
`RVO2Solver.agent_parameters`와 `agent_neighbors`는 공식 C++ getter를 읽어 실제
적용된 세 항목과 kd-tree가 선택한 이웃을 확인한다. 제어 틱의 계산에는 호출하지 않는다.
전체 기본값과 검증 조건은 `isaac_composer/crowd/config.py`에 있다.
기존 고정 시나리오는 `enabled: false`이며 이전 제어기를 유지한다.

전체 이동 가능 면적당 인원 수를 정하려면 `generation`에 다음을 지정한다.

```yaml
generation:
  mode: random
  target_density_per_m2: 0.03
  max_count: 30
```

`round(이동 가능 면적 × 목표 밀도)`를 1~max_count로 제한한다. 미지정 시 기존
`count`를 사용한다. 분리된 영역과 최대 인원 제한 때문에 실제 지역 밀도는 목표
전체 밀도와 다를 수 있다. 실제 인원 수와 전체 밀도를 `crowd_config.json`에 기록한다.

## 출력과 좌표계

`scenario/crowd_states.jsonl`에는 50Hz 실제 보행자·로봇 world 상태와 preferred/ORCA
속도, feasibility와 progress(후보 속도·진행 가능 여부·정체 시간)를 저장한다. `crowd_config.json`에는 seed와 고정 μ를,
`crowd_state_schema.json`에는 필드·단위·상태 코드를 기록한다.

10Hz dataset의 기존 scan/robot position/velocity/goal과 같은 파일 번호로
`pedestrians/*.npy`와 `crowd_context/*.json`도 저장한다. 보행자 배열은 (N,11):

`[id, x, y, vx, vy, mu, local_density, walkable_area_m2, passage_width_m, state, yield_to]`

보행자와 context의 로봇은 **world** 좌표이다. 기존 dataset 로봇 position/goal은
**odom**, velocity는 몸체 좌표 `[vx,vy,omega_z]`를 유지한다.
`robot_task.json`의 `world_from_odom`의 역변환으로 보행자 위치를 맞출 수 있다.
`crowd_context`에는 보행자 시각과 scan 시각을 모두 기록한다. 동기화 오차는
기본 30ms까지 허용하며 일치하는 보행자 상태가 없으면 해당 샘플을 저장하지 않는다.

상태 코드: walking=0, yielding=1, stepping_aside=2, stopped=3, infeasible=4,
waiting_for_space=5. 보행자 ID는 0부터, 로봇 ID는 -1부터이며 yield_to=-999는 없음이다.
실행 보고서 `crowd_navigation`에 최소 거리, 만남/양보/복귀 이벤트 및 infeasible 틱
수를 기록한다. ORCA는 이상적인 원형 agent 속도 모델이며 실제 애니메이션의 회전·
가감속과 갑작스러운 teleop 접근 때문에 제약을 만족하지 못할 수 있어 이 상태를
숨기지 않고 기록한다. 임의 teleop·통과 공간이 없는 상황에서 무충돌/도착을 보장하지 않는다.

## 빌드와 실행

시스템 `g++`가 필요하다. 첫 사용 시 고정된 C++ 소스를 `/tmp/isaac_composer_rvo2/`
내용 hash별 디렉터리에 컴파일한다. Python ABI와 무관한 작은 C 인터페이스를 사용하므로
ROS Python 3.10과 Isaac Python 3.11에서 같은 코어를 사용한다. 이후 캐시를 재사용한다.

```bash
bash ~/isaac_ws/src/isaac_composer/scripts/build_workspace.sh --packages-select isaac_composer data_collection
source ~/isaac_ws/install/setup.bash
ros2 launch data_collection collect_pedestrians_ogm.launch.py
```

로봇은 조이스틱으로 goal까지 이동시키며 goal 도착 시 다음 에피소드를 생성한다.

실행 환경·통과 거리·저장 빈도와 재현 명령은 [검증 결과](social_navigation_validation.md)에 있다.
