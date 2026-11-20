# Isaac Composer

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
  maps/                  Nav2 occupancy map YAML과 PNG
  params/nav2/           실제 Nav2 node 파라미터
  navigation/
    maps/                 map 이름 → config/maps/*.yaml
    profiles/             로봇별 Nav2/토픽/프레임/scan 변환 계약
launch/
  scenario.launch.py            simulation과 navigation을 함께 실행
  navigation.launch.py   로봇 1대용 공통 Nav2 launch
  navigation_scenario.launch.py
                          scenario를 읽어 공통 launch를 1회 또는 N회 호출
pointcloud_to_laserscan_reliable/
                          내부에 포함한 C++ scan 변환 ROS 패키지
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
bash src/isaac_composer/scripts/build_workspace.sh --packages-up-to isaac_composer
source install/setup.bash
export ISAACSIM_PYTHON=/path/to/isaac-sim/python.sh
ros2 launch isaac_composer scenario.launch.py scenario:=warehouse_spot
```

빌드 스크립트는 내부 `pointcloud_to_laserscan_reliable/` 경로도 colcon에 전달한다.
`--packages-up-to isaac_composer`로 변환기까지 함께 빌드한다. 폴더 이동 전에 빌드한
워크스페이스에서는 첫 빌드에 `--cmake-clean-cache`를 추가해 이전 소스 경로를 갱신한다.

Isaac Sim만 실행하려면 다음처럼 navigation을 끈다.

```bash
ros2 launch isaac_composer scenario.launch.py \
  scenario:=warehouse_a2 use_navigation:=false
```

`navigation.launch.py`는 낮은 수준의 단일 로봇 진입점이고,
`navigation_scenario.launch.py`는 navigation만 별도로 붙일 때 사용한다.

## 로봇 시작 위치와 랜덤 보행자

로봇의 시작 위치는 `scenario.launch.py`의 `robot_x`, `robot_y`, `robot_z`,
`robot_yaw` 기본값으로 정하거나 실행 인자로 지정한다. 빈 값은 scenario의 pose를
사용한다. 단위는 m와 rad이며 여러 로봇 중 첫 번째 로봇에 적용한다.
동일한 x/y/yaw가 Nav2 초기 위치에도 적용된다.

```bash
ros2 launch isaac_composer scenario.launch.py \
  scenario:=warehouse_a2_random \
  robot_x:=0.0 robot_y:=0.0 robot_z:=0.65 robot_yaw:=0.0
```

기본 실행은 로봇 + Nav2 + RViz이며 보행자는 생성하지 않는다.
`scenario.launch.py`의 `use_pedestrians` 기본값을 true로 바꾸거나 실행 인자로
`use_pedestrians:=true`를 지정했을 때만 선택한 scenario의 보행자 preset을 생성한다.
`*_a2_random`과 `warehouse_a2_fixed`도 Nav2와 RViz를 기본으로 실행한다.

```bash
# Robot + Nav2 + RViz, no pedestrians
ros2 launch isaac_composer scenario.launch.py scenario:=warehouse_a2_random

# Add the warehouse_random pedestrian preset
ros2 launch isaac_composer scenario.launch.py \
  scenario:=warehouse_a2_random use_pedestrians:=true
```

`use_rviz:=false`는 RViz만 끄고, `use_navigation:=false`는 Nav2와 RViz를 끈다.
Nav2를 끈 경우에도 scan 변환은 실행하며, Nav2를 켠 경우에는 중복 실행하지 않는다.
`navigation.launch.py`는 Nav2/RViz용 하위 launch다. 전체 실행 설정은
`scenario.launch.py`에서 지정한다.

보행자는 `config/pedestrians/*_random.yaml`의 `generation.count`,
`speed_range_mps`, `repeat` 설정으로 생성한다. 시작점과 목적지는 전체 NavMesh에서
샘플링하고 연결된 경로만 채택한다. 초기 배치의 최소 간격은 유지하며,
시작한 뒤에는 ORCA, 양보, 사람 간 회피, 로봇 회피를 사용하지 않는다.
보행자에는 `radius`와 `height`에 맞는 보이지 않는 kinematic capsule을 붙이고,
애니메이션의 실제 위치에 맞춰 갱신한다. 보행자끼리의 물리 충돌만 제외하고
로봇과의 접촉은 허용한다. 보행자는 정해진 경로를 유지하며 로봇은 capsule과 충돌한다.
벽 우회는 환경에서 bake한 NavMesh 경로가 담당한다.

고정 경로 보행자는 다음처럼 실행한다.

```bash
ros2 launch isaac_composer scenario.launch.py scenario:=warehouse_a2_fixed use_pedestrians:=true
```

기본 클립보다 빠른 목표 속도는 보행자별 애니메이션과 root motion을 함께 배속한다.
`animation_walk_reference_mps`의 기본값은 1.0이며 기본 클립의 기준 속도다.
정지·속도 변경 이벤트, 시작 지연, 시작점과 목적지 사이 왕복을 지원한다.

NavMesh는 로봇/사람 배치 전에 환경에서 한 번 bake한다. Include Volume이
없으면 환경 경계에 맞춰 자동 생성한다. 창고는 occupancy map과
`generation.scene.allowed_region_xy`로 지상 바닥 영역을 제한한다.
생성 경로와 seed는 `generated/pedestrians/<preset>/pedestrian_routes.json`,
이동 가능 영역과 경로 이미지는 `00_free_space.png`, `01_pedestrian_routes.png`로 저장한다.

화면은 기본적으로 첫 번째 로봇을 따라간다. `view:=top_down`은 탑다운,
`view:=free`는 자유 시점이다. 카메라 설정은 `config/simulation/default.yaml`에 있다.

## 새 조합 추가

1. 기존 로봇이면 `config/robots`는 건드리지 않는다.
2. 다른 Nav2 동역학/footprint/센서 구성이 필요할 때만 `config/params/nav2`와
   `config/navigation/profiles`에 프로필을 추가한다.
3. 새 맵은 `config/maps`에 YAML/이미지를 넣고 `config/navigation/maps`에서 이름을 붙인다.
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
