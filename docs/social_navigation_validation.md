# 보행 제어 검증 결과 (2026-10-14)

## 2026-10-15 짧은 정지와 애니메이션 전환 수정

- 긴 native preview가 ORCA 조건을 만족하지 않으면 짧은 경로도 검사한다.
  필요하면 같은 ORCA LP가 보정한 방향으로 제한 횟수만 재조회한다.
- 임시 preview 끝점은 원래 목적지가 아니므로 0.3m 도착 판정을 분리한다.
- 정지 입력에는 즉시 Walk=0과 현재 위치의 경로를 전달한다. Action의 Idle
  전환만 기본 0.2초 지연하며, 기존 전진 경로를 계속 사용하지 않는다.
- 단위·통합 테스트: 150개 통과. 짧은 preview, 잘못된 긴 preview의 대체,
  정지 경로의 즉시 전달, 짧은 정지와 지속 정지를 포함한다.
- Isaac Sim: episode_000033의 보행자 시작점·경로·μ를 사용해 10명, 35초 실행.
  A2는 해당 episode의 시작 위치에 고정했다. 원본 teleop의 완전한 재현이나
  동일 조건의 전후 비교는 아니다.
- 경로 변환 거부 0회, 경로 재검사로 이동 유지 132회. 원래 즉시 Walk/Idle
  판정이었다면 66번 바뀌는 입력에서 실제 Action 전환은 55회였다.
  0.15초 이하 짧은 정지 3회는 모두 Action 재시작 없이 처리했다.
- 관측 최소 중심 거리: 보행자 간 0.644m, A2와 2.306m. infeasible agent tick은
  32회였다. 양보·지속 정지와 실제 애니메이션의 추종 오차가 모두 해결된 것은 아니다.
- 원시 실행 기록: `/tmp/isaac-jitter-fix-replay-20261015/`.
  이 실행의 고정 데모 카메라는 벽에 가려져 있어 영상을 시각적 검증 근거로 사용하지 않는다.


공식 RVO2 commit: `75822bba14462c2e872ca96423825017ecd24764`.
ROS Humble, Isaac Sim 5.1, warehouse 환경에서 검증했다.

## 2026-10-15 벽 여유·애니메이션 입력 수정

이번 범위는 사람 NavMesh 반경 축소와 애니메이션 입력 안정화다.
우회 경로 선택 계층이나 새로운 양보 전략은 추가하지 않았다.

| 항목 | 이전 | 수정 후 |
| --- | --- | --- |
| 사람 몸체 / 기본 경로 조회 반경 | 0.3m / 0.55m | 0.3m / 0.35m |
| 사람용 bake 반경 | 0.45m, 로봇 task에서는 0.55m까지 커짐 | 0.3m, 로봇 query 0.55m는 복사본에서 유지 |
| 창고 사람용 map clearance | 0.4m | 0.35m, unknown은 계속 차단 |
| AnimationGraph 목표 | 매 20ms 임시 목표 생성·입력 | native 경로 재사용, 변경 시에만 PathPoints 입력 |
| 속도·방향 목표 | 틱 간 변화 제한 없음 | 가속 1.5m/s², 감속 3m/s², 방향 3.14rad/s 기준 |
| Walk 입력 | m/s 속도 크기를 직접 입력 | 기준 속도에 대해 정규화한 0~1 입력, Walk/Idle hysteresis |

속도를 완만하게 만든 뒤 동일 관측의 공식 ORCA LP로 다시 확인한다. 제약 때문에
편안한 변화율을 넘는 경우 safety_overrides로 기록한다. 해가 없는 상태는
infeasible로 유지하고 native 경계 검사를 거친 제한 입력을 사용한다.
실제 속도는 종전처럼 위치의 시간 차분이며 새 ORCA/animation 필드는 진단용이다.

- 전체 코드 테스트: **146 passed**.
- 단독 보행: 20초 실행, goal 오차 0.264m, infeasible 0, 임시 경로 갱신 11회.
- 두 보행자/폭 1.8m 통로: 35초 실행, 둘 다 goal 0.35m 이내 도착, 최소 사람
  중심 거리 0.797m, 최소 벽·사람 중심 거리 0.433m. 별도 yield 선택 없이 통과했다.
  경로 생성 루틴을 매 틱 호출하던 3,500회 대신 실제 새 경로 66회를 생성했다.
  3~18초의 연속 명령에서 크기 0.1m/s 이상인 속도 간 45도 초과 방향 변경은
  이전 46회에서 이번 실행 0회였다. Native 실행의 완전한 결정성은 보장하지 않는다.
- A2 마주침: 최소 사람·로봇 중심 거리 0.905m로 몸체 반경 합 0.85m보다 컸으나,
  후퇴한 보행자가 정지한 A2의 반대편으로 재진행하지 못했다. goal 검증은 **실패**했다.
  원래 위치(-3,10)에서 멈추는 데모 로봇과 기존 양보/우회 전략의 남은 문제이며,
  이번 두 수정으로 모든 정체를 해결했다고 해석하면 안 된다.

결과·영상은 `generated/demos/wall_animation_fix_20261015/`에 저장했다.
원본 결과는 `/tmp/isaac-wall-animation-locomotion/`,
`/tmp/isaac-wall-animation-pedestrians/`, `/tmp/isaac-wall-animation-robot-final/`이다.

## 사람 간 간격 축소 후 코드 검증

사람 사이 추가 여유를 0.48m에서 0.06m로 축소했다. 반경 0.3m 두 사람의
ORCA 중심 거리 제약은 1.08m에서 0.66m가 된다. 로봇과의 추가 여유는 0.33m로
분리했으며 기본 사람·A2 중심 거리 제약은 1.18m다.

전체 코드 테스트 **140 passed**. 폭이 제한된 공간에서 중심 간격 0.68m의 반대
방향 흐름이 감속 없이 통과하는 회귀 테스트와, 간격 축소 전후 움직이는 로봇에
대한 공식 ORCA 속도 해가 일치하는 테스트를 추가했다.

실제 Isaac `close_pass --duration-sec 60 --light`로 폭 1.8m 중앙 통로를
다시 검증했다. 두 사람이 모두 goal에서 0.35m 이내에 도착했고,
최소 사람 중심 거리 0.707m, 최소 벽·사람 중심 거리 0.486m였다.
4.90초에 필요한 양보를 선택하고 10.22초에 resume했다. 지속 정체로 남지 않았다.
infeasible agent ticks는 46이며 실제 애니메이션과 이상적인 속도 모델의 차이를
계속 기록한다. 결과는 `/tmp/orca-close-pass-release/verification.json`에 있다.
이 테스트의 μ는 seed 43의 랜덤 값이며 아래 이전 테스트의 μ=(1,1)과 다르므로
두 실행을 동일 조건의 성능 비교로 해석하지 않는다.

## 이전 설정의 통합 검증 기록

### 2026-10-15 실제 영상 재실행에서 확인된 거리 조건 미달

GUI와 RGB 녹화를 켠 추가 실행에서는 사람끼리 최소 중심 거리 0.556m,
사람·A2 최소 중심 거리 0.668m가 관측됐다. 원형 몸체 반경 합인 0.60m와 0.85m에
각각 미달했다. 두 사례 모두 목적지 근처까지 이동했지만 거리 제약 검증은 실패했다.
infeasible agent ticks는 96 / 486이다. 앞의 통과 사례만으로 실제 애니메이션의
무충돌을 보장할 수 없다. 거리 유지와 실제 이동 속도의 추종 문제를 추가로 검토해야 한다.

실제 RGB 영상·스크린샷·측정 결과는
`generated/demos/narrow_corridor_20261015/`에 있다. `demo.html`에서 두 영상을
재생하고 근접·양보·재이동 구간으로 이동할 수 있다. 로봇은 이 데모에서만 재현용
고정 명령(목표 0.45m/s)을 사용했다. 본 수집의 teleop 제어는 바꾸지 않았다.

```bash
~/develop/isaacsim/python.sh ~/isaac_ws/src/isaac_composer/test/verify_native_social.py narrow_robot --duration-sec 35 --light --video --gui
```

이 명령은 영상과 측정을 저장한 뒤 거리 조건 미달을 assertion으로 보고한다.

### 간격 축소 전 통합 검증

아래 수치는 간격 축소 전 `safety_margin_m=0.18`, `tracking_margin_m=0.15`로
실행한 기록이다. 최신 설정에서 모든 사례를 다시 실행한 결과를 의미하지 않는다.

- 양보 조건: 유용한 ORCA 속도가 없는 상태가 연속 0.5초 유지됨.
- 유용한 속도: 크기 ≥ max(0.08m/s, 원하는 속도의 15%), 전진 또는 옆 방향.
- 밀도: 반경 1.5m에서 기록만 수행, 양보 선택에 사용하지 않음.
- 애니메이션 추종 여유: 사람별 0.15m, erode된 정적 경계에 추가 0.1m.
- μ: 에피소드 동안 고정되는 양보 성향.

## 코드 및 빌드

`isaac_composer/test`와 `data_collection/test`: **138 passed**.
공식 C++ 코어로 정면·교차·추월, 정지/이동 로봇, 좁은 통로의 대기 공간,
진행 가능성 조회, 순간 정체와 지속 정체 구분, 밀도 무관 조건, μ 확률,
seed 재현성, native 경로의 투영·모서리 보존, 저장 동기화를 확인했다.
ROS `isaac_composer` 및 `data_collection` 빌드도 완료했다.

## 실제 Isaac의 좁은 통로 양보

`test/verify_native_social.py yield --duration-sec 90 --light`는 실제 USD 벽을
추가하고 NavMesh를 다시 bake한다. 중앙 통로 내부 폭은 1.8m이며 양쪽에
대기 공간이 있다. 두 사람은 반대 방향으로 0.8m/s 이동한다. μ는 둘 다 1이다.
밀도 임계값으로 양보를 강제하지 않고 실제 ORCA 진행 불가 조건을 사용했다.

| 항목 | 관측 결과 |
| --- | --- |
| 양보 선택 | 4.46초, 진행 불가 상태 0.50초 유지 후 한 번 선택 |
| 대기 공간 이동 | 양보자가 옆 공간으로 이동, 상대는 기다림 |
| 복귀 | 15.84초에 resume, 원래 목적지로 재이동 |
| 최종 goal까지 XY 거리 | 0.291m / 0.304m |
| 최소 사람 간 중심 거리 | 1.429m, 물리 반경 합 0.6m 이상 |
| 최소 벽과 사람 중심 거리 | 0.495m, 사람 반경 0.3m 이상 |
| infeasible agent ticks | 16 |

결과: 두 사람 도착, 양보·복귀 이벤트, 사람·벽 간 거리 조건을 모두 통과했다.
결과 파일은 `/tmp/orca-progress-safe-tracking/verification.json`이다.

추종 여유가 작았던 중간 설정에서는 실제 root가 ORCA 명령을 늦게 따라가고
NavMesh 경계 밖으로 벗어나, 양보를 선택한 뒤 정체되는 실패를 확인했다.
최종 설정은 추종 여유를 조정하고, 애니메이션의 이동 preview를 native 경로로
만들며, 경로 조회 때 시작점과 목표를 모두 투영한다. 실제 위치를 순간이동시키지 않는다.

infeasible은 ORCA 제약 또는 실행 가능한 NavMesh 이동 검사에 실패한 agent 틱이다.
공식 RVO2의 제약 충돌 시 fallback이 반환될 수 있으므로 이 상태를 기록한다.
위의 거리 조건은 관측 궤적의 검증 결과이며 임의 teleop·밀집 군중에서 무충돌이나
도착을 보장하는 결과로 확대해서 해석하지 않는다.

재현 명령:

```bash
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash
export ROS_DOMAIN_ID=47
~/develop/isaacsim/python.sh ~/isaac_ws/src/isaac_composer/test/verify_native_social.py yield --duration-sec 90 --light
```

case는 head_on / crossing / robot / moving_robot / yield / close_pass / random이다.
출력은 기본 `/tmp/isaac-social-*/` 아래로 격리된다. 부모 프로세스가 저장 결과를
검사하므로 Isaac 종료 과정에서 검증이 건너뛰어지지 않는다. moving_robot만
테스트용 로봇 명령을 적용한다. production 로봇은 teleop을 유지한다.

## 최종 설정의 데이터 수집

별도 `/tmp` 테스트 드라이버로 로봇을 goal까지 이동시켜 실제 수집 launch를 검증했다.
source의 production 주행 제어에는 자동 주행을 추가하지 않았다.

| 항목 | 결과 |
| --- | --- |
| 에피소드 / seed | 000000 / 43 |
| 종료 | goal_reached |
| 저장 샘플 | 232 |
| 최소 사람 간 중심 거리 | 1.499m |
| 최소 사람·로봇 중심 거리 | 1.774m |
| infeasible agent ticks | 0 |
| scan↔crowd 최대 시각 차이 | 6ms, 허용 30ms 이내 |
| 실제 관측 시간 차분과 보행자 저장 속도의 최대 오차 | 6.33e-8m/s |

scan(1440), robot position(3), actual velocity(3), goal(2), pedestrian(5,11),
context의 파일 번호와 개수가 모두 일치했다. μ는 232개 샘플 동안 변하지 않았다.
저장 위치는 `/tmp/orca-progress-release-dataset/episode_000000/`이다.

샘플링 목표는 최대 10Hz이며 이 실행의 실제 저장 빈도는 약 6.6Hz였다.
ROS scan/odom 입력과 동기화 상태에 따라 일부 시각이 저장되지 않는다.
일정한 10Hz를 보장했다고 해석하면 안 된다.
