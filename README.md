# Isaac Composer

Robot navigation with optional pedestrians. Requires ROS 2 Humble, Isaac Sim 5.1, and PyTorch.

- Isaac Sim: `~/develop/isaacsim`
- Python: `~/anaconda3/envs/env_isaacsim/bin/python`
- Runtime environment: `scripts/isaac_python.sh`

## Build

Place this package in `~/isaac_ws/src/isaac_composer`.

```bash
cd ~/isaac_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/isaac_composer src/isaac_composer/pointcloud_to_laserscan_reliable --ignore-src -r -y
PYTHONNOUSERSITE=1 colcon build --symlink-install \
  --base-paths src/isaac_composer src/isaac_composer/pointcloud_to_laserscan_reliable \
  --packages-up-to isaac_composer
```

Rebuild after adding or moving configuration files.

## Run

```bash
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash
ros2 launch isaac_composer scenario.launch.py scenario:=office_a2_random
```

Default: robot + Nav2 + RViz, no pedestrians. Choose a scenario from `config/scenarios/`.
Append options such as `use_pedestrians:=true view:=top_down`.

| Option | Default | Description |
|---|---|---|
| `view` | `follow` | Robot follow, `free` camera, or `top_down` follow |
| `use_pedestrians` | `false` | Spawn the scenario's pedestrians |
| `use_navigation` | `true` | Enable Nav2 and RViz |
| `use_rviz` | `true` | Show RViz |
| `headless` | `false` | Hide the Isaac Sim window; RViz is separate |
| `robot_x`, `robot_y`, `robot_z`, `robot_yaw` | Scenario pose | First robot's position (m) and heading (rad) |

Camera distance/height: `config/simulation/default.yaml` → `viewport`.
Pedestrian count/speed: `config/pedestrians/`. Nav2 parameters: `config/params/nav2/`. Maps: `config/maps/`.
