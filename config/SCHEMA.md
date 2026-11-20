# Configuration schema version 1

## Shared rendering

모든 기본 시나리오는 `config/simulation/default.yaml`의 렌더 설정을 사용한다.
`rendering.indirect_diffuse_intensity`(기본 10.0)는 Ray Tracing의
Indirect Diffuse Lighting > Intensity에 대응한다. 환경 로드 후 간접 확산광 GI를
활성화하고 해당 배율을 적용한다. 0 이상의 유한 수를 사용하며, 생략하면 이
옵션을 변경하지 않는다. 환경 USD의 조명 자체를 수정하지 않는다.

## Scenario

Scenario는 실행 단위다. 환경과 보행자뿐 아니라 map과 robot별 navigation
profile까지 한곳에서 선택한다.

```yaml
schema_version: 1
name: warehouse_spot
simulation: default
environment: warehouse
pedestrians: default

navigation:
  enabled: true
  map: warehouse
  use_sim_time: true
  autostart: true
  use_rviz: true
  startup_delay_sec: 5.0

robots:
  - preset: spot
    instance_name: spot_1
    namespace: ""
    pose:
      position: [-3.0, -2.0, 0.68]
      orientation_quat_wxyz: [1.0, 0.0, 0.0, 0.0]
    navigation:
      profile: spot
```

`pose`는 instance 값이므로 robot preset에 넣지 않는다. AMCL pose는 기본적으로
spawn pose의 x/y/yaw에서 만들어진다. 별도 보정이 필요하면 다음을 추가한다.

```yaml
navigation:
  profile: spot
  initial_pose: {x: 1.0, y: 2.0, z: 0.0, yaw: 1.57}
```

다중 로봇에서는 각 `namespace`가 유일해야 한다. 동일한 Nav2 파라미터를
namespace마다 재사용하므로 profile과 파라미터 내부 토픽은 상대 이름을 쓴다.

Scenario 또는 robot instance에 Nav2 leaf의 전체 YAML 경로로 override를 줄 수
있다. instance 값이 scenario 값보다 우선한다.

```yaml
navigation:
  parameter_overrides:
    local_costmap.local_costmap.ros__parameters.width: 7
    local_costmap.local_costmap.ros__parameters.height: 7
```

## Navigation map preset

`config/navigation/maps/*.yaml`은 논리적인 map 이름을 설치되는 실제 파일에
연결한다. 경로는 package share 기준 상대 경로다.

```yaml
schema_version: 1
name: hospital
map_file: maps/carter_hospital_navigation.yaml
```

## Navigation profile

Profile은 로봇 이름이 아니라 Nav2와 ROS graph 사이의 계약이다.

```yaml
schema_version: 1
name: spot
supported: true
nav2:
  params_file: params/nav2/spot.yaml
  rviz_file: rviz/spot_navigation.rviz
frames: {map: map, odom: odom, base: base_link, lidar: front_3d_lidar}
topics:
  cmd_vel: cmd_vel
  odom: odom
  pointcloud: front_3d_lidar/lidar_points
  scan: scan
pointcloud_to_laserscan:
  enabled: true
  cloud_topic: front_3d_lidar/lidar_points
  scan_topic: scan
  parameters:
    target_frame: front_3d_lidar
    min_height: -0.4
    max_height: 2.0
```

아직 필수 ROS 계약이 없는 로봇은 실패 원인을 숨기지 않는다.

```yaml
schema_version: 1
name: a2
supported: false
reason: cmd_vel and odometry providers are not implemented
```

A2처럼 학습 policy를 사용하는 로봇은 controller에 package-relative 실행용
TorchScript와 policy 주기를 명시한다.

```yaml
spawn: {provider: a2_policy}
controller:
  provider: from_spawner
  policy_path: config/robots/A2FlatTerrainPolicy.pt
  policy_dt: 0.02
  action_scale: 0.25
  cmd_vel_topic: /cmd_vel
```

## Robot preset

모든 robot 문서는 같은 최상위 section을 갖는다.

```yaml
schema_version: 1
name: robot_lookup_name
spawn: {provider: provider_name}
robot: {usd_path: ..., prim_path: ..., links: ...}
controller: {provider: provider_name}
sensors: {provider: provider_name, items: []}
ros: {provider: provider_name}
clock: {provider: provider_name}
```

Provider 값은 robot identity가 아니라 resource의 소유권을 표현한다.

- `generated`: Python이 생성한다.
- `embedded`: USD 내부에 이미 있으므로 중복 생성하지 않는다.
- `existing`: 기존 prim을 찾아 외부 출력만 선택적으로 붙인다.
- `disabled`: 의도적으로 아무 작업도 하지 않는다.

`spot_policy` 같은 특수 provider가 필요해도 runner는 robot 이름을 비교하지 않고
registry에서 provider 이름으로 구현을 찾는다.

Sensor mount의 `parent`는 `link:base` 또는 `sensor_mount:front_lidar`처럼 명시한다.
generated sensor는 preset, translation, WXYZ quaternion, ROS topic/frame을 가진다.

## Robot task

scenario의 선택적 `robot_task`는 로봇 한 대의 시작 위치·방향과 도달 가능한 goal을
랜덤으로 생성한다. 수집 launch에서는 기본으로 활성화한다.

```yaml
robot_task:
  enabled: true
  map_file: maps/carter_hospital_navigation.yaml
  min_goal_distance_m: 5.0
  max_goal_distance_m: 15.0
  robot_radius_m: 0.55
  clearance_m: 0.65
  spawn_height_m: 0.65
  goal_radius_m: 0.5
  stopped_speed_mps: 0.1
  stopped_yaw_rate_rps: 0.2
  arrival_hold_sec: 0.5
  max_attempts: 500
```

시작 yaw는 `[-pi, pi]`에서 뽑고, 속도·관절 초기 상태는 보행 제어기의 정지 상태를 사용한다.
`map_file`은 패키지 루트 기준 또는 절대 경로의 ROS 점유 맵 YAML이다.
바닥의 spawn 후보는 NavMesh로 확인하고, 로봇 경로는 이 정적 맵에서 계산한다.
맵의 unknown/장애물을 `clearance_m`만큼 확장해 8방향 Dijkstra를 수행하며,
대각선으로 장애물 모서리를 통과하지 않는다. 주행 중 경로 갱신도 같은 맵을 사용한다.
`map_file`을 지정하지 않은 사용자 프리셋은 기존 NavMesh 경로 방식을 유지한다.
goal은 world 좌표로 생성하고 실제 odom 기준에 맞춰 변환해 발행·저장한다.
이 task는 주행 명령을 생성하지 않는다. 이동은 기존 teleop 입력으로 제어한다.

## Social navigation

`record_states_enabled`는 시뮬레이터 내부의 고주기 `crowd_states.jsonl` 저장 옵션이다.
기본값은 true이며 collection launch는 `record_crowd_states:=false`로 이 저장을 끈다.
보행자 회피 제어, 상태 관측과 에피소드 결과 보고는 계속 동작한다.

창고·병원 랜덤 프리셋에서는 다음 방향별 회피 책임도 켠다. 기존 Social Stiffness와
독립적이며 `yielding_enabled: false`는 유지한다.

```yaml
social_navigation:
  directional_responsibility_enabled: true
  directional_rear_weight: 0.1
  directional_smoothing_s: 0.2
  directional_emergency_gap_m: 0.15
  directional_emergency_transition_m: 0.3
```

몸 방향 기준 앞·옆·뒤 가중치는 1/0.55/0.1이며, 두 사람의 가중치를 정규화해
회피 책임 비율로 사용한다. 몸 표면 간 간격 0.45m부터 완만하게 50:50으로 돌아가고
0.15m 이하는 50:50이다. 한쪽만 상대를 탐지하면 탐지자가 100%를 맡는다.
Teleop 로봇에는 보행자 책임 100%를 유지한다. 뒤 가중치는 (0, 1], 나머지는
양의 유한 수여야 한다. 기본 라이브러리 설정은 꺼져 있으며 두 랜덤 프리셋이 켠다.

`pedestrians.social_navigation`은 공식 RVO2와 지속적인 진행 불가 상태·μ 양보를 설정한다.
안전한 전진·감속·옆 이동이 가능하면 ORCA를 유지하고, 유용한 속도가 없는 상태가
`progress_hold_s`(기본 0.5초) 지속되면 양보를 검토한다. 지역 밀도는 반경 1.5m에서
기록하며 양보 조건으로 사용하지 않는다.
사람 간 ORCA 추가 여유는 `safety_margin_m + 2 × tracking_margin_m`(기본 0.06m),
사람·로봇 추가 여유는 `robot_safety_margin_m`(기본 0.33m)로 분리한다.
사람 경로 조회는 몸체 반경 + `animation_path_clearance_m`(기본 0.05m),
정적 맵을 사용하는 로봇 task 경로는 `clearance_m`(기본 0.65m)을 사용한다.
`animation_retarget_interval_s`(0.12초), `animation_max_acceleration_mps2`(1.5),
`animation_max_deceleration_mps2`(3.0), `animation_max_turn_rate_rps`(3.14)는
애니메이션 입력의 변화 목표를 제한한다. `animation_walk_reference_mps`(1.0)는
0~1 Walk 입력으로 변환할 기본 클립의 기준 속도이며 이동 상한이 아니다.
기준 이하에서는 기존 Walk blend를 사용하고, 기준을 넘으면 Walk=1에서
보행자별 애니메이션 시간을 `선택 속도 / 기준 속도`배로 진행한다.
발 동작과 root motion을 함께 배속하며 자동 갱신과 중복 실행하지 않는다.
실제 관측 속도와 목표 속도가 정확히 동일하다고 가정하지 않는다.
`animation_idle_delay_s`(0.2초)는 정지가 지속된 후 Idle Action으로 바꾸는
시간이다. Walk=0 및 정지 경로 전달은 지연하지 않는다.
전체 기본값은 `isaac_composer/crowd/config.py`, 의미와 출력은
[보행 제어 문서](../docs/social_navigation.md)에 있다.
`generation.target_density_per_m2`는 실제 NavMesh 면적 기준 인원 수를 결정하고,
`generation.max_count`(기본 30)가 인원을 제한한다. 미지정 시 `generation.count`를 사용한다.
