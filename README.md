# Isaac Composer

Cosmos 기반 보행 활동 계획·검토·실행은 [semantic scenario planner](docs/semantic_scenario.md)를 참고한다.

`isaac_composer`는 기존 로봇 이름별 분기를 설정 기반 조합으로 옮기기 위한
ROS 2 Python 패키지다. A2 USD와 Hesai JT128 프로필처럼 로컬에서 필요한
자산도 패키지 안에 포함하므로 다른 소스 패키지의 파일을 참조하지 않는다.

## 한 번의 실행이 조립되는 순서

```text
scenario
  ├─ simulation preset
  ├─ environment preset
  ├─ pedestrian preset
  ├─ robot preset(s)
  ├─ navigation map preset
  └─ robot별 navigation profile
          ↓
  load_scenario_bundle()
          ├──────────────────────────────┐
          ↓                              ↓
  SimulationRunner              build_navigation_plan()
          ↓                              ↓
  spawn/controller/sensor/ROS   navigation_scenario.launch.py
  provider 조립                         ↓ N robots
          ↓                     navigation.launch.py
  Isaac Sim physics/policy      Nav2 + pointcloud_to_laserscan + RViz
```

공통 Python 코드에는 `if robot == "spot"` 같은 분기가 없다. scenario가
프로필 이름을 선택하고, 프로필이 Nav2 파라미터·RViz·토픽·프레임·점군
변환 설정을 제공한다.

## 디렉터리 역할

```text
config/
  scenarios/             한 실행에 필요한 모든 preset과 위치를 묶음
  robots/                USD, controller, sensor, ROS 소유권
  sensors/               재사용 가능한 LiDAR 사양
  environments/          환경 USD
  pedestrians/           보행자와 waypoint
  simulation/            physics/render/timeline
  navigation/
    maps/                 map 이름 → maps/*.yaml
    profiles/             로봇별 Nav2/토픽/프레임/scan 변환 계약
launch/
  scenario.launch.py            simulation과 navigation을 함께 실행
  navigation.launch.py   로봇 1대용 공통 Nav2 launch
  navigation_scenario.launch.py
                          scenario를 읽어 공통 launch를 1회 또는 N회 호출
maps/                     Nav2 occupancy map YAML과 PNG
params/nav2/              실제 Nav2 node 파라미터
rviz/                     단일/namespace RViz 설정
usd/                      패키지에 포함해 설치하는 로컬 USD와 하위 layer
isaac_composer/
  runtime.py              Isaac Sim lifecycle과 provider 실행 순서
  clock.py                실행 시 전역 ROS 2 /clock ActionGraph 생성
  assets.py               로컬/Isaac Assets 경로 해석
  stage.py                USD reference, pose, physics scene 생성
  pedestrians.py          보행자/navmesh scenario adapter
  config.py               YAML 로드, 스키마 검증, 참조 해석
  navigation.py           ROS와 독립적인 navigation 실행 계획 생성
  models.py               provider 사이의 공통 robot/sensor context
  registry.py             이름 → provider 구현 registry
  spawn/                  USD/Spot policy spawn provider
  controllers/            embedded/graph/Spot policy controller
  sensors/                embedded/generated RTX LiDAR provider
  ros/                    embedded/generated ROS graph와 clock provider
```

## 옮긴 기존 설정

- Spot: omni AMCL 모델, 횡방향 속도, Spot footprint, 0.1초 scan 설정을 유지한다.
- Nova Carter 단일: 기존 Carter 차동구동 파라미터와 0.3333초 scan 설정을 유지한다.
- Nova Carter 다중: 기존 hospital/office의 3대 설정을 공통 파라미터 하나로 합쳤다.
  로봇별 초기 위치와 namespace는 scenario에 있고, hospital의 7 m 및 office의
  10 m local costmap 크기도 scenario override에 있다.
- A2: 2026-10-26 MJLab `model_10000.pt`에서 export한
  `A2FlatTerrainPolicy_update.pt`를 package-owned adapter가 50 Hz로 실행한다.
  `cmd_vel`을 47차원 policy observation에
  넣고, 12개 joint-position action을 A2 articulation에 적용한다. Spot과 같은
  generated ROS graph가 odometry, TF, LiDAR를 제공한다.

Nav2 파라미터의 `/scan`, `/odom`, `/carter1/...` 절대 토픽은 상대 토픽으로
바꿨다. 따라서 같은 파일을 `carter1`, `carter2`, `carter3` namespace 아래에서
재사용할 수 있다.

## 사용

로컬 `~/anaconda3/envs/env_isaacsim/bin/python`이 있으면 launch는
`scripts/isaac_python.sh`를 통해 PyTorch가 있는 이 Python으로 시뮬레이터를
실행한다. ROS launch 자체는 ROS 환경을 source한 시스템 Python에서 실행한다.
`ISAACSIM_PYTHON` 또는 launch의 `isaac_python` 인자는 이 자동 선택보다 우선한다.

설정만 검증한다.

```bash
python3 -m isaac_composer.cli warehouse_spot
python3 -m isaac_composer.cli hospital_multi_nova_carter
```

빌드 후 `ISAACSIM_PYTHON`에 Isaac Sim 설치의 `python.sh`를 지정하면 simulation과
navigation을 같은 scenario로 실행할 수 있다.

```bash
colcon build --packages-select isaac_composer
source install/setup.bash
export ISAACSIM_PYTHON=/path/to/isaac-sim/python.sh
ros2 launch isaac_composer scenario.launch.py scenario:=warehouse_spot
```

Isaac Sim만 실행하려면 다음처럼 navigation을 끈다.

```bash
ros2 launch isaac_composer scenario.launch.py \
  scenario:=warehouse_a2 use_navigation:=false
```

`navigation.launch.py`는 낮은 수준의 단일 로봇 진입점이고,
`navigation_scenario.launch.py`는 navigation만 별도로 붙일 때 사용한다.

## NavMesh 보행자 이동과 데이터 수집

화면은 기본적으로 첫 번째 로봇 뒤에서 이동과 수평 회전을 따라간다.
launch의 `view:=top_down`은 로봇 중심의 탑다운, `view:=free`는 자유 시점이다.
카메라 거리·높이·완만한 추적 정도는 `config/simulation/default.yaml`의 `viewport`에서
설정한다. 수집 launch에서도 같은 인자를 사용하며 매 에피소드에 적용된다.

Cosmos 서버와 topology 생성 없이 NavMesh로 시작점과 목적지를 연결한다.
수집 launch는 로봇 시작 위치·방향과 goal도 랜덤으로 생성한다. 시작 속도는 0이고
사용자가 teleop으로 목표까지 조종한다. `robot_task:=false`로 preset 시작 위치를
사용하는 보행자 데모를 실행할 수 있다.

```bash
cd ~/isaac_ws
bash src/isaac_composer/scripts/build_workspace.sh --packages-select isaac_composer data_collection
source install/setup.bash
```

### 1단계: 지정한 두 지점 사이 이동

```bash
ros2 launch data_collection collect_pedestrian_episode.launch.py \
  scenario:=warehouse_a2_fixed robot_task:=false duration_sec:=0
```

`config/pedestrians/warehouse_fixed.yaml`의 `generation.start`, `goal`은 월드 좌표
`[x, y, z]`(m)다. `speed_mps`로 속도를 지정한다. 기본값은 `[4, 4, 0]`에서
`[4, -6, 0]`으로 0.8 m/s 이동하고 도착하면 정지한다. `repeat: true`로 바꾸면
시작점과 목적지 사이를 왕복한다. 두 점을 직선으로 잇지 않고 NavMesh 최단 경로의
전체 경유점을 애니메이션 그래프에 전달해 장애물을 우회한다. 목적지 도착은
XY 거리 0.3 m 이내로 판단한다. 이동 불가능한 지점이나 연결되지 않은 경로는
오류로 알려주며 다른 시작점으로 임의 변경하지 않는다. NavMesh 투영은 최대
0.25 m의 XY 보정만 허용한다.

### 2단계: 여러 보행자의 시작점·목적지·속도 랜덤 생성

```bash
ros2 launch data_collection collect_pedestrians_ogm.launch.py \
  scenario:=warehouse_a2_random episodes:=10
```

`config/pedestrians/warehouse_random.yaml`의 `generation.count`(기본 5명),
`speed_range_mps`(기본 0.6~1.4 m/s)를 변경한다. 목표 속도는 1m/s에서 잘리지 않으며,
기본 걷기보다 빠른 명령은 보행자별 애니메이션과 root motion을 함께 배속한다.
`animation_walk_reference_mps`는 기본 클립의 기준이므로 속도 범위를 높일 때 같이
올리지 않는다. 실제 속도는 클립·회피·회전에 따라 목표값과 차이가 날 수 있다.
시작점과 목적지는 전체 이동 가능
영역에서 샘플링하며, 연결된 경로만 채택한다. 로봇 및 다른 보행자와 너무 가까운
시작점은 다시 뽑는다. 같은 seed는 같은 배치·목표·속도를 재현한다. 기본적으로
선택한 두 지점 사이를 왕복한다. 랜덤 시나리오는 공식 C++ RVO2로 사람 간 및
teleop 로봇 회피를 수행한다. 옆으로 피하거나 감속하며 진행할 수 있으면 ORCA를
유지한다. 안전한 진행·옆 회피 속도가 부족한 상태가 0.5초 지속되면 μ에 따른
확률로 양보자를 정해 기다리거나 대기 공간으로 물러난다. 밀도는 기록용이다.
계층별 구현, 설정, 좌표계 및 데이터 형식은 [보행 제어 문서](docs/social_navigation.md)를 참조한다.
병원은 `scenario:=hospital_a2_random`을 사용한다.

NavMesh는 로봇/사람을 배치하기 전에 환경에서 한 번 bake한다. Include Volume이
없으면 환경 경계에 맞춰 자동 생성한다. 창고는 기존 occupancy map의 장애물과
unknown 주변 0.4 m를 제외하고 `allowed_region_xy: [-9.5, -11.4, 8.5, 17.0]`
범위의 지상 바닥을 사용한다. 경로 선분도 map으로 검사하며 Nav2/map_server는
필요하지 않다. NavMesh 계산에 환경의 rigid body 장애물도 포함한다.

생성 결과는 기본적으로 `generated/pedestrians/<preset>/`에 저장된다.
수집 supervisor로 실행하면 각 `episode_XXXXXX/scenario/`에 따로 저장된다.

- `pedestrian_routes.json`: seed, 로봇 위치, 사람별 시작점·속도·경유점·μ
- `crowd_config.json`, `crowd_state_schema.json`: 밀도/회피 설정과 저장 스키마
- `crowd_states.jsonl`: 50Hz 관측 상태와 회피 결과
- `00_free_space.png`: 이동 가능 영역
- `01_pedestrian_routes.png`: 초기 경로, 시작점(원), 목적지(사각형)
- 수집의 `episode_runtime.json` 안 `crowd_navigation`: 최종 이동/양보 상태와 통과·우회·복귀 이벤트

새 수집의 기본 출력 경로는 `~/isaac_ws/src/data/A2-OGM-Pedestrians`다.

시뮬레이션만 실행하려면 다음 명령을 사용한다. 이때 조이스틱/recorder는 별도로
실행해야 한다.

```bash
ros2 launch isaac_composer scenario.launch.py \
  scenario:=warehouse_a2_fixed use_navigation:=false
```

빌드 스크립트는 `PYTHONNOUSERSITE=1`로 ROS 시스템 setuptools를 사용한다.
직접 빌드할 때도 `PYTHONNOUSERSITE=1 colcon build --symlink-install`을 사용할 수 있다.

## 새 조합 추가

1. 기존 로봇이면 `config/robots`는 건드리지 않는다.
2. 다른 Nav2 동역학/footprint/센서 구성이 필요할 때만 `params/nav2`와
   `config/navigation/profiles`에 프로필을 추가한다.
3. 새 맵은 `maps`에 YAML/이미지를 넣고 `config/navigation/maps`에서 이름을 붙인다.
4. `config/scenarios`에서 환경, 로봇, 위치, map, navigation profile을 묶는다.

로봇 위치 quaternion은 spawn과 AMCL에 함께 사용된다. AMCL은 평면 시스템이라
spawn의 x/y/yaw를 사용하고 z는 0으로 둔다. 둘이 달라야 할 때만 robot instance의
`navigation.initial_pose`를 별도로 지정한다.

## 현재 구현

`SimulationRunner`가 environment와 robot을 만들고 provider registry를 통해
controller, sensor, ROS graph, clock을 연결한다. Spot의 `cmd_vel`은 생성된
OmniGraph에서 읽어 `SpotFlatTerrainPolicy.forward()`에 전달한다. A2도 같은
`from_spawner` 경로를 사용하되 package의 A2 policy adapter가 exported TorchScript를
실행한다. Nova Carter는 USD에 내장된 controller/sensor/ROS graph를 사용한다.
보행자와 navmesh도 동일한 scenario에서 생성한다.

Composer가 Isaac Sim을 실행하면 ROS 2 bridge 확장을 활성화한 다음
`clock.py`를 통해 `/World/ROS_Clock` ActionGraph를 만들며, timeline 재생 중
simulation time을 `/clock`으로 publish한다. topic, queue size, domain ID는 각
`config/robots/*.yaml`의 `clock` 및 `ros` 설정을 따른다.

A2와 Spot은 `from_spawner` controller를 공유하지만 실제 locomotion policy와
observation/action 처리는 각각의 policy runtime이 소유한다.

A2의 `controller.policy_format: mjlab`은 각속도(3), 중력(3), 명령(3),
보행 위상 sin/cos(2), 상대 관절 위치(12), 관절 속도(12), 이전 action(12)을
순서대로 사용한다. 위상 주기는 0.6초이며 명령의 norm이 0.1 미만이면 위상
입력을 0으로 만든다. 관절/action은 FL, FR, RL, RR 다리별 hip, thigh, calf
순서로 정책에 전달하고, 출력은 USD 관절 순서로 되돌린다. 관절 armature는
0.03, 최대 관절 속도는 학습의 MJLab override와 같은 1e9로 설정한다.
기존 48차원 정책은 `policy_format: flat`과 기존 `A2FlatTerrainPolicy.pt`를
함께 지정하면 사용할 수 있다.
Isaac Sim 런타임 API가 필요한 실제 simulation 검증은 Isaac Sim의 `python.sh`로
실행해야 하며, 일반 Python 테스트에서는 설정·registry·navigation·자산 경로를
검증한다.
