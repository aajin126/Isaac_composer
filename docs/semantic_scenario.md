# Cosmos semantic scenario planner

Cosmos가 환경·로봇 주행 목적에 맞는 이야기를 만들고, 실제 월드 이미지와 이동 가능한
칸을 사용해 보행자의 활동을 배치하고 검토한다.
JSON 키와 문자열은 영어로 작성한다. 새 topology나 USD 물체 목록을 만들지 않는다.
Cosmos 응답을 파싱한 뒤 영어 문장의 곡선 따옴표·긴 대시·줄임표·특수 공백을
일반 ASCII 문장부호로 정리한다. 실제 비영어 단어를 지우거나 번역하지 않으며,
해당 필드와 문자 코드로 오류를 전달해 다시 작성하게 한다. 원문 응답은 그대로 보존한다.

| 단계 | 입력 | 출력 |
|---|---|---|
| 월드 입력 | 기존 USD, native NavMesh, 칸 크기 | 실제 RGB + 칸 번호 이미지, 유효한 칸 JSON |
| 로봇 과제 | 현재 native NavMesh, seed | 실제 robot start·goal 확정, 직선 주변 우선 칸 |
| 전체 이야기 | 환경, 로봇 이동 목적, 영어 요청, 이전 활동 요약 | `global_scenario.json` |
| 필요 인원 | 전체 이야기, 전체 인원 N | `staffing.json`의 K |
| 역할 | 이야기, K, 선정된 ID | `participant_plan.json` |
| 장면 조율 | 이야기, 전체 역할 | 장면 순서·참여자·공동 활동 조건 |
| 개인별 계획 | 장면 조율, 역할, 이미지, 유효한 칸 | `scene_plan.json` |
| 이동 검증 | 실제 native 경로, 이동·정지 비율 | 문제 있는 목적지·필드만 Cosmos가 재선택 |
| Cosmos 2차 검토 | 전체 계획, 월드·경유 순서 이미지 | `review.json`, 문제 목록 |
| 수정 | 이전 계획, 검토 문제 | 수정 → 새로운 검토 |
| 실행 준비 | 통과 계획, 현재 native NavMesh | 실제 경로, semantic + random 명단 |
| 실행 | 실제 위치, 도착·체류 조건 | 기존 ORCA/애니메이션, 활동 이벤트 |

전체 이야기 요청에는 N을 전달하지 않는다. 이후 Cosmos가 K를 결정하고 코드는
**ceil(N/2) ≤ K ≤ N**을 검사한다. 나머지 N-K명은 기존 무작위 보행자다.
인원 결정에서는 코드가 유효한 구성표를 만든다. N=10이면 `5+5`, `6+4`, …,
`10+0`에 각각 선택 ID를 부여하고 Cosmos는 이야기와 맞는 ID 하나만 고른다.
모델에 인원 계산이나 범위 밖 숫자의 수정을 맡기지 않는다. 선택한 구성표를 그대로
`staffing.json`으로 저장하며, 코드는 임의로 구성표를 선택하거나 인원을 보정하지 않는다.
잘못된 응답은 다시 요청하되 이전의 잘못된 숫자를 후보처럼 재노출하지 않는다.
episode의 역할 요청은 한 사람씩 `role`과 `purpose` 두 문자열만 받는다. 앞서 확정한
actor ID·인원 수·랜덤 ID와 코드가 결합해 `participants` 배열을 만들며 인원 수를
다시 바꾸지 않는다. 공간 맥락이 없는 기존 단독 호출은 배열 응답 방식도 유지한다.
활동 역할과 기존 μ 성향은 별개다. 의미 적절성은 Cosmos가 별도 요청으로 검토한다.
코드는 인원·ID·칸·선후 관계와 native 경로를 검사한다. 기본 구조 재시도 2번,
의미 수정 최대 2번 뒤에도 실패하면 실행하지 않는다. 전체 이야기나 여러 사람의 조율에
문제가 있다고 검토된 경우에만 전체 생성 단계로 돌아간다.
개인 계획 검사는 오류 메시지와 함께 정확한 필드 경로를 반환한다. 잘못된 칸, 행동,
체류 시간, 큐 참조·서비스 행동 등은 해당 필드 값만 Cosmos가 다시 계획한다.
목적지 선택에는 유효 칸 목록만 주며, 같은 잘못된 ID를 여러 번 참조했다면
Cosmos가 고른 대체 칸을 함께 적용한다. 형식이 깨진 경유지나 경유지 목록은 해당 단위만
다시 받는다. 정상 보행자와 다른 정상 필드는 유지한다.
의미 검토가 특정 보행자를 지적하면 그 사람의 역할·목적 또는 특정 장면의 행동 필드만
변경 목록으로 받는다. 다른 보행자·장면 조율·인원·전체 이야기는 변경할 수 없으며,
구조 검사 후 Cosmos의 의미 검토를 다시 통과해야 한다.
각 필드·개인 수정은 최대 3회, 개인 계획의 필드 수정은 최대 24개로 제한한다.
한도를 넘으면 정상 보행자를 재생성하지 않고 해당 후보를 종료한다.
각 요청의 마지막에 완성된 영어 JSON 출력 예시와 반환할 키를 붙인다. 칸·인원·ID가
있는 예시는 그 요청의 실제 허용 후보로 구성하며 예시를 대체 계획으로 사용하지 않는다.
검토에는 통과·거절 예시를 모두 제공하고 `passed/issues`만 짧게 반환하도록 한다.
이전에 입력 계획을 복사한 실패 응답은 검토 입력에 다시 넣지 않는다.
개인 필드 예시는 `{"value":"LookAround"}`, `{"value":5}`처럼 실제 값의 형식까지 보여준다.
JSON 괄호가 잘못된 필드 응답은 다음 요청에서 그 응답의 문법만 수정하도록 한다.
경유지가 4개를 초과하면 Cosmos가 유지할 4개의 인덱스를 고르고, 선택된 경유지의
기존 행동·위치·시간을 유지한다. `lookaround` 같은 대소문자 차이는 `LookAround`로
표기만 정리하며 다른 의미의 행동으로 자동 대체하지 않는다.

## 코드 계층

```text
isaac_composer/semantic_scenario/
├── schema.py                 # 영어 JSON, 인원·칸·순서 검사
├── text.py                   # 영어 문장의 문장부호 정리
├── scene_input.py            # 실제 RGB, 칸 표시, 계획 이미지
├── pipeline.py               # 생성 → 검토 → 수정 조립
├── collection.py             # episode마다 새 계획, 월드 입력 캐시
├── reporting.py              # 터미널에 이야기·역할·경로 출력
├── variation.py              # 이전 이야기·동작의 정확한 반복 검사
├── spatial.py                # 공통 배치 정책을 Cosmos 입력으로 변환
├── navigation_quality.py     # 이동 거리·정지 시간·직선 주변 이동 비율
├── queues.py                 # native Queue 설정 검사
├── cosmos/
│   ├── client.py             # HTTP, 모델 조회, 응답 파싱
│   ├── contracts.py          # 선택적인 JSON Schema 출력 계약
│   ├── output_examples.py    # 요청 후보에 맞는 영어 JSON 출력 형식 예시
│   ├── planner.py            # 전체 이야기 → 인원 → 역할
│   ├── staffing.py           # 유효한 구성표 중 Cosmos가 인원 구성 선택
│   ├── roles.py              # 한 사람씩 짧게 역할 생성, 개인별 재시도
│   ├── start_selector.py     # 남은 후보에서 Cosmos가 출발점 하나씩 선택
│   ├── scene_planner.py      # 장면 조율 → 개인별 경로
│   ├── actor_repair.py       # 오류 필드만 다시 계획하고 재검사
│   ├── route_repair.py       # 잘못된 목적지 필드만 Cosmos가 다시 선택
│   ├── local_revision.py     # 의미 검토가 지적한 개인의 역할·행동만 수정
│   ├── reviewer.py           # 별도 Cosmos 의미 검토
│   └── review_response.py    # 검토 응답의 명확한 형식 차이 정리
└── runtime/
    ├── compiler.py           # 칸 → world 좌표 → native 경로
    ├── executor.py           # 도착·체류·공동 활동 상태
    ├── planning.py           # 실제 로봇 과제 이후 Cosmos 호출, UI 응답 유지
    ├── path_validation.py    # 메인 스레드 native 검증과 도달 가능한 대체 칸
    ├── looping.py            # 계획 반복·복귀 경로·독립/공동 활동의 반복 단위
    └── native_queue.py       # 설치된 QueueCmd/Dequeue와 ORCA 연결
scripts/
├── export_scene_grid.py      # Isaac 실행 진입점
└── plan_semantic_scenario.py  # Cosmos 진입점; Isaac 시작하지 않음
```

## 실행

기본 수집 실행은 **episode마다 새로운 Cosmos 시나리오를 생성**한다.

```bash
source ~/isaac_ws/install/setup.bash
ros2 launch data_collection collect_pedestrians_ogm.launch.py \
  scenario:=warehouse_a2_random episodes:=10
```

첫 episode 준비 때 현재 월드의 이미지와 격자를 자동 생성한다. 월드 입력이 같으면
이를 재사용하며, 이야기·필요 인원·역할·장면·경로는 매 episode마다 Cosmos에 새로 요청한다.
이전 세 episode의 이야기와 새 seed를 제공하고, 같은 이야기·동작의 정확한 반복은 거부한다.
표현만 바꾼 비슷한 상황인지는 Cosmos 검토 단계에서도 판단한다.
이미지에 실제로 보이는 환경 안에서 사람의 목적과 상호작용을 상상하도록 요청한다.
환경 이름만 보고 흔히 있을 법한 시설을 추가하지 않으며, 정체가 불명확한 물체는
기능을 추측하지 않고 위치나 외형으로 표현한다. 움직이는 주체는 도보 보행자이고
기존 물체는 정지한 배경이라는 조건을 시스템 지시와 생성·검토 프롬프트에 적용한다.

```text
현재 월드 확인 → Isaac에서 로봇 start·goal 확정 → 새 이야기 → 인원 → 역할
  → 장면·개인 행동 → native 경로·이동 검증 → Cosmos 검토
  → teleop 수집 → 로봇 goal 도착 → 다음 episode의 새 이야기 → ...
```

전체 인원은 기본 `generation.count`를 따르며 `cosmos_total_count:=6`으로 바꿀 수 있다.
서버는 `cosmos_base_url`, 모델은 `cosmos_model` 인자로 지정한다.
`placement_cell_size_m:=1.0`은 더 촘촘한 줄서기 위치를 계획할 때 사용할 수 있다.
모델 응답이나 검토가 재시도 한도를 넘어서 실패하면 episode를 실패로 남기고,
3초 후 새 seed로 다음 시나리오를 자동 요청한다. `SEMANTIC_SCENARIO_RETRY`에 이유를
출력한다. 연속 후보 실패는 기본 **2개**까지만 허용하며 `cosmos_max_candidates`로
양의 정수 범위에서 변경한다. 한도에 도달하면 `SEMANTIC_COLLECTION_STOPPED`와 마지막
실패 이유·artifact 경로를 출력하고 종료 코드 1로 중단한다. 무제한 재생성은 하지 않는다.
`SEMANTIC_CANDIDATE`와 `SEMANTIC_REVISION`에 후보·수정 횟수의 상한을 출력한다.
이전 계획이나 검토를 통과하지 않은 계획을 실행하지 않는다. 실패한 후보는 요청한 완료
episode 수에 포함하지 않으며 Ctrl+C로 준비를 중지할 수 있다.
환경 설정·월드 내보내기·시뮬레이터 실행·데이터 정렬 오류는 기존처럼 중단한다.
생성 시간에는 recorder와 teleop episode를 시작하지 않는다.

검토 모델에는 경로 선이 겹친 그림 대신 원래 월드·칸 이미지와 구조 검사 결과를 전달한다.
칸 ID·초기 출발점 중복·장면 선후 관계는 코드 검사 결과를 사용하고, Cosmos는 이야기·역할·
활동 위치의 의미를 검토한다. native 경로 검사는 시뮬레이터 로딩 시점에 별도로 수행한다.

전체 이야기 생성 직후 `SEMANTIC_SCENARIO_DRAFT`와 줄거리를 출력한다. 검토 통과 후에는
`SEMANTIC_EPISODE_READY`, 승인된 이야기·사건 순서·인원·개인별 역할과 칸 경로를
시뮬레이터 시작 전에 출력한다. DRAFT는 아직 검토를 통과하지 않은 초안이다.
재시도 시 `SEMANTIC_STAGE_ERROR`에 검사 실패 이유를 표시한다. JSON 문법 오류는
문제가 발견된 줄·열을 함께 표시하며, 깨진 응답의 일부를 실행 계획으로 채택하지 않는다.
재요청에는 오류와 직전의 잘못된 응답을 함께 전달한다. 전체 이야기 재생성에는
이전 이야기와 의미 검토만 제공하고, 개인 경로 JSON은 제외해 요청 형식이 섞이지 않게 한다.
필수 `events` 누락과 객체·배열 괄호 오류는 동일한 전체 이야기 형식으로 수정 요청한다.
검토의 `requested_change`는 문자열을 요청한다. 모델이 `{ "change": "correction" }`처럼
감쌌을 때는 중복 actor ID·scope가 바깥 issue와 일치하는지 확인한 뒤 문장을 꺼낸다.
다른 필드·충돌하는 대상·빈 문장이 있으면 수정 요청한다. 검토의 통과 여부와 문제 내용은
변경하지 않으므로 `passed: false`는 계속 시나리오 수정으로 이어진다. 원문 응답은 보존한다.

출발점은 개인 경로와 분리된 요청에서 Cosmos가 직접 선택한다. 이 요청에는
`available_start_cells`라는 남은 후보 목록과 actor ID만 전달한다. 칸을 선택하면
다음 보행자의 목록에서 제거한다. 전체 유효 칸 목록·이전 경로·이미지의 다른 칸 번호는
출발점 선택 요청에 포함하지 않는다. 유효하지 않은 선택은 제한된 횟수만 다시 요청하며
코드가 대신 출발점을 고르거나 보정하지 않는다. `SEMANTIC_START_SELECTED`로 선택을 출력한다.
경로 생성에는 `selected_start_cell`을 고정된 시작점으로 전달하고 출발점을 다시 선택하지 않게 한다.
개인 경로에는 목적지·경유점으로 `valid_cells`의 ID만 그대로 고르도록 지시한다.
잘못된 응답은 해당 보행자만 다시 요청하며, 직전 응답·누적 오류·선택 가능한 칸을
함께 제공한다. 프롬프트는 강제 디코딩이 아니므로 출력 검사를 계속 적용한다.

다음은 **계획 하나를 별도로 만들고 재생**하는 수동 도구다.

```bash
cd ~/isaac_ws/src/isaac_composer
~/develop/isaacsim/python.sh scripts/export_scene_grid.py \
  warehouse_a2_random --cell-size-m 2 --scene-image \
  --output-dir generated/semantic_scenario/warehouse_context

/usr/bin/python3 -s scripts/plan_semantic_scenario.py \
  --context-dir generated/semantic_scenario/warehouse_context \
  --total-count 10 --base-url http://172.17.0.2:8000 \
  --output-dir generated/semantic_scenario/warehouse_plan \
  --request "Workers inspect storage areas, pause to look around, and walk to the next stop."

bash scripts/build_workspace.sh --packages-select isaac_composer data_collection
source ~/isaac_ws/install/setup.bash
ros2 launch data_collection collect_pedestrians_ogm.launch.py \
  scenario_planner:=replay \
  semantic_plan_dir:=/home/ewhaglab/isaac_ws/src/isaac_composer/generated/semantic_scenario/warehouse_plan
```

월드 입력은 환경이 달라질 때 다시 저장한다. `world_scene.png`는 실제 RGB,
`grid_scene.png`는 RGB 위 칸 번호, `grid_navmesh.png`는 기존 도식이다.
격자 geometry·collider를 추가하지 않는다. 카메라 clipping으로 2.5m보다 높은 부분이
잘릴 수 있지만 물리 geometry와 LiDAR는 변경하지 않는다. meter 단위 stage를 지원한다.

모델은 `/v1/models`에서 자동 조회한다. 여러 모델이면 `--model`로 선택한다.
인증이 필요하면 `COSMOS_API_KEY` 환경 변수를 사용한다. 기본은 최종 JSON을 파싱하며,
지원 서버에서는 `--structured-output`을 선택할 수 있다.
출력 모드와 관계없이 코드 검사를 통과해야 한다. 작은 모델의 계획 품질은 별도 확인이 필요하다.

`scenario_planner:=cosmos`가 기본이다. `random`은 Cosmos 없이 기존 랜덤 보행자를,
`replay`는 명시한 계획을 재생한다. 새 계획은 `episode_XXXXXX/scenario/semantic_plan/`에
직접 생성·보관한다. 자동 이미지 입력은 `_scene_context/`에 캐시한다.
고급 옵션 `cosmos_context_dir`로 미리 내보낸 월드 입력을 지정할 수 있으며,
이때도 환경·USD가 현재 시나리오와 맞는지 검사한다. 월드 geometry가 변경되면 입력도 새로 내보내야 한다.

## 실행 규칙과 저장

- `start_cell → subgoals → goal_cell`로 이동한다. 경로는 native NavMesh, 회피는 기존 ORCA가 담당한다.
- 실제 도착 반경 0.3m 안에서 최근 0.2초 평균 이동 속도가 0.15m/s 이하일 때 체류 시간을 센다.
- `Idle`은 정지, `LookAround`는 기존 animation graph로 표현한다. 검수·대화는 이 동작으로 근사한다.
- `Queue`는 설치된 `omni.anim.people`의 `QueueCmd`로 빈자리 배정·전진을 수행한다.
  맨 앞에서 지정 시간 동안 서비스 동작을 한 뒤, 설치된 `Dequeue`로 다음 목적지로 나간다.
  native 명령의 이동부만 어댑터로 연결하여 최종 회피·애니메이션 명령은 기존 ORCA 경로가 소유한다.
- `all_participants_arrived`는 전원이 첫 subgoal에 도착·정지하면 첫 체류를 함께 시작한다.
- 후속 장면은 선행 장면을 기다리며, 다음 시작 칸까지 걸어간다. teleport하지 않는다.
- 회피로 활동 자리에서 벗어나면 복귀하며 체류 시간 계산을 멈춘다.
- Idle/LookAround의 작은 root 흔들림을 위해 실행 query에만 기존 tracking margin(현재 2cm)을 인공 수집 구역 경계에 적용한다.
  계획 칸 중심, baked NavMesh, 지도 벽 clearance, 몸체·ORCA 반경은 그대로 검사한다.
- 장면을 마친 semantic 보행자는 정지한다. 배경 보행자는 기존 이동을 계속한다.
- A2는 teleop이고 episode는 기존 로봇 goal 도착 조건으로 종료한다.

동시 출발이 그룹 대형 유지나 시선 맞춤을 보장하지는 않는다. 실제 물체 조작,
지게차 이동, 물건 운반은 구현하지 않는다. `planned_routes_XX.png`의 선은 경유 순서이며
실제 우회 경로가 아니다. native 경로는 Isaac에 로딩할 때 검사하며 연결 불가능하면 중단한다.

`approval.json`에 통과 여부와 계획 hash를 저장한다. 통과 후 계획을 수정하면 다시 검토해야 한다.
episode의 `scenario/semantic_plan/`에 계획을 복사하고 `compiled_scene_plan.json`에 실제 경로를 저장한다.
`execution_events.jsonl`에 이동·도착·활동·장면 완료를 기록하고 crowd 상태에도 진행 정보를 추가한다.

```bash
~/develop/isaacsim/python.sh test/verify_native_semantic.py \
  --plan-dir generated/semantic_scenario/warehouse_plan \
  --output-dir /tmp/isaac-semantic-check --duration-sec 180
```

검증은 실제 검토된 계획을 실행하며 LiDAR를 끄고 A2를 정지시켜 활동을 확인한다.
부모 프로세스가 실제 활동 완료와 전체 장면 완료를 파일로 확인한다.

2026-10-15 검증: `http://172.17.0.2:8000`의 `nvidia/Cosmos-Reason2-2B`로 생성·별도 검토를
통과한 `generated/semantic_scenario/warehouse_plan_v8`을 실제 Isaac에서 실행했다.
4명 모두 LookAround 적용, 2명은 다음 칸으로 이동했다. 경계 칸의 애니메이션 흔들림으로
이동이 막히는 경우를 찾아 실행 query에만 2cm 허용을 적용했고, 수정 후 14.6초에 완료됐다.
전체 Python 테스트 181개 및 ROS 두 패키지 빌드도 통과했다.
이는 활동 실행 검증이며 2B 모델의 역할·장소 선택 품질을 보장하지 않는다.

2026-10-16 변경 검증: 테스트 190개와 ROS 두 패키지 빌드가 통과했다.
두 episode가 각각 새 모델 요청·seed·계획 경로를 사용하고, 두 번째 요청에 첫 이야기의
이력을 전달하는 흐름은 통합 테스트로 확인했다. 실제 native Queue 검증용 고정 fixture에서는
3명이 0→1→2 순서로 앞자리에 도착하고, 앞에서 2초 활동 후 Dequeue로 빠져나가
18.44초에 모두 완료했다. 이는 Cosmos가 만든 이야기가 아닌 native 기능 통합 검증이다.

실제 서버의 매 episode 생성 검증에서는 월드 이미지·격자 자동 생성까지 성공했으나,
2B 모델이 정적인 지게차·상자를 움직이는 이야기를 반복하여 의미 검토에서 거절됐다.
**실제 서버로 두 episode 연속 계획 통과는 아직 확인하지 못했다.** 검토 실패 시에는
이전 계획을 재생하거나 성공으로 표시하지 않고 해당 episode 준비를 실패로 남긴다.

Queue 설정 예시는 다음과 같다. spots는 맨 앞부터 나열하고, Queue의 cell_id는 맨 앞 칸이다.
duration_s는 앞자리에 도착한 이후의 서비스 시간이다. Queue와 서비스·Dequeue가 하나의 행동으로 실행된다.

```json
{
  "queues": [{"queue_id": "inspection_line", "spots": ["C5", "C6", "C7"]}],
  "subgoals": [
    {"cell_id": "C5", "action": "Queue", "queue_id": "inspection_line",
     "service_action": "LookAround", "duration_s": 5}
  ]
}
```

이 예시는 전체 scene_plan이 아닌 Queue 관련 필드 발췌다. native 기능은
[Isaac Sim Actor Control의 Queue 문서](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/action_and_event_data_generation/ext_replicator-agent/actor_control.html#queue)를 따른다.

## Robot-relative traffic and ongoing activity

Goal-directed Cosmos collection now prepares the real native robot start and goal **before**
calling Cosmos. The supervisor writes a pending request; the episode's existing Isaac process
samples its robot task, then supplies it to Cosmos while keeping the application responsive.
This avoids loading a second extra world for robot task sampling. Simulation playback and
collection begin after planning, review and native route compilation succeed.

The priority placement region is the walkable band around the **straight line** from the robot
start to its goal, with a default half-width of **3 m on each side (6 m total)**. Set
`route_band_half_width_m:=4.0` at launch for a 4 m half-width. This is independent of
grid cell size and of the 1.5 m minimum robot spawn separation. It is an interaction reference;
teleoperation can take a different path, and NavMesh still computes actual pedestrian routes.
Candidate lists are shuffled per episode, ordered for local spatial spread, and prioritize that band
for every initial-start request. Once its available cells run out, nearby cells are preferred
before remote map edges. Spacing preference saturates at 1.5 grid cells, so it cannot keep
rewarding distant corners. All starts keep at least 1.5 m from the robot.
Cosmos chooses the final cell from the candidates; code never substitutes the first example.
The neutral module `isaac_composer/pedestrian_placement.py` now owns band geometry,
region balancing, candidate ordering and goal pools. Pure random collection, Cosmos start
selection, and the random remainder of a Cosmos cast use this same module. Actor IDs 3 and 7
have no special placement policy. Pure random mode builds only the coarse valid-cell grid
inside its existing simulator; it does not export images or contact Cosmos. It samples random
starts and goals, keeps native obstacle-avoiding routes, and retains ping-pong repetition.
The band prioritizes placement endpoints; native obstacle detours can leave that band.

Use `scenario_planner:=random route_band_half_width_m:=3.0 placement_cell_size_m:=2.0`
for a model-free baseline. Switch only `scenario_planner:=cosmos` to use semantic planning with
the same placement settings. Matching settings share the candidate policy, not necessarily
the selected positions, trajectories or action distribution. `robot_task:=false` leaves random
generation over the whole mesh because there is no robot start-goal segment. Fixed routes
retain their configured endpoints. Replay uses the approved grid and width from its saved plan.
The old `cosmos_route_band_half_width_m` and `cosmos_cell_size_m` launch arguments remain
aliases; explicit shared arguments take precedence. Both modes save `pedestrian_placement.json`
alongside episode artifacts, including the task, valid grid, band cells and actual starts.

All stages receive spatial regions and the navigation objective. Ordinary activity stops have
at most 4 seconds each and a 6-second total budget per actor per scene, divided across its
stops. Native Queue service retains its separate duration allowance. At least 60% of the
planned cast must have meaningful movement (at least 3 m, adjusted for grid size), with an
estimated walking share of at least 40% using 1 m/s for the intent estimate. At least half must
move through priority cells. Queue users are limited to `max(2, ceil(0.3 * planned count))`.
`navigation_quality.json` records these intent metrics. Failures request local actor revisions;
a passing intent check does not prove actual timing, navigation, collisions or semantic quality.
Cosmos must still review the plan and native queries must still succeed.

The previous alphabetical first-cell bias placed actors along one map boundary. Start selection
now sees candidate regions and the assigned purpose; actor examples show actual walking with
an empty stop list instead of stopping at every neighboring cell. Story drafting uses temperature
0.8; routes and repair retain 0.3. History also includes action/goal distribution summaries.

Compiled Cosmos plans set `repeat_semantic_plan: true`. After completion, actors walk back through
their route targets in reverse order to their original start, then repeat the original movement,
Idle, LookAround and Queue activities. They do not teleport or switch to random destinations.
The return trip contains only GoTo legs, without repeating the activity stops in reverse.
Independent actors repeat immediately without waiting for unrelated walkers. Scenes sharing
actors, completion dependencies or an `all_participants_arrived` barrier repeat as a group,
and barriers and dwell counters reset each cycle. A final GoTo hands off to the return path
in the same control tick; deliberate activity pauses and ORCA yielding still apply.
`execution_events.jsonl` records `semantic_loop_restarted`; runtime diagnostics include
`repeat_semantic_plan` and `actor_cycle_counts`. Reviewed robot tasks are reused during
execution and replay so Cosmos's robot context cannot silently change afterwards.

Native path queries run on the Isaac main thread during planning. If a leg fails, only its
actor/scene/destination field is exposed to a repair request, with alternatives that the current
native mesh can actually reach inside the collection region. The corrected plan is checked
again before semantic review and approval. No custom shortest-path solver is introduced.
`GoTo` waypoints have zero dwell and advance immediately; string cell waypoints are represented
as those same pass-through commands. Idle/LookAround remain explicit purposeful stops.
Approval digest version 2 also protects the robot task and spatial guidance used for planning.

In the earlier, pre-loop isolated native check on 2026-10-16, 8 of 10 initial positions were within 1.5 m of the
actual robot start-goal segment, 9 actors displaced by more than 0.3 m over 20 simulated seconds,
and 4 entered background continuation. The robot stayed stationary, LiDAR was disabled for this
motion check, and the probe intentionally stopped at 20 seconds rather than reaching the goal.
These measurements establish placement/motion behavior for this case, not semantic realism
across every generated story or interaction with a moving teleoperated robot.

After the placement and looping update, ten actual Cosmos start-selection requests on that saved
world/task put 9 of 10 starts within 1.5 m of the line; the remaining start was 1.71 m away.
Actors 3 and 7 were 1.05 m and 0.25 m from the line. A separate 60-second native replay retained
the older approved plan and its original positions to test execution only. Six actors completed
17 plan passes in total and immediately entered their next return/repeat cycle; repeated activity
completion events were recorded. The robot was stationary and LiDAR was disabled. Evidence is
saved under `generated/verification/semantic_loop_20261016/`. The start-selection check and
native replay are separate checks, not a new complete scenario rollout at the updated positions.

For enriched episode contexts, global story creation is a short text request about the world
type, navigation purpose and recent activity summaries. It receives no grid IDs or previous
actor/location assignments. Placement and the independent review still receive the actual world
images. This separates story invention from image-caption imitation and grid address selection.
Only the requested story fields are retained; unsolicited actor IDs and locations in a draft
are not carried into staffing or the next episode's history. A valid single-object array wrapper
is normalized losslessly; multiple documents and malformed JSON remain rejected.
