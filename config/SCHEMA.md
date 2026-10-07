# Configuration schema version 1

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
