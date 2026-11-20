# SPDX-License-Identifier: Apache-2.0
"""Start Isaac Sim composition and the matching Nav2 scenario."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


from isaac_composer.interpreter import default_isaac_python as _default_isaac_python


def generate_launch_description():
    package_share = get_package_share_directory("isaac_composer")
    scenario = LaunchConfiguration("scenario")
    headless = LaunchConfiguration("headless")
    simulation = ExecuteProcess(
        cmd=[
            LaunchConfiguration("isaac_python"),
            os.path.join(package_share, "scripts", "run_scenario.py"),
            scenario,
            PythonExpression([
                "'--use-pedestrians' if '", LaunchConfiguration("use_pedestrians"),
                "'.lower() == 'true' else '--no-use-pedestrians'",
            ]),
            "--robot-x", LaunchConfiguration("robot_x"),
            "--robot-y", LaunchConfiguration("robot_y"),
            "--robot-z", LaunchConfiguration("robot_z"),
            "--robot-yaw", LaunchConfiguration("robot_yaw"),
            "--max-frames",
            LaunchConfiguration("max_frames"),
            "--duration-sec",
            LaunchConfiguration("duration_sec"),
            "--seed",
            LaunchConfiguration("seed"),
            "--generation-output-dir",
            LaunchConfiguration("generation_output_dir"),
            "--view",
            LaunchConfiguration("view"),
            PythonExpression(
                ["'--headless' if '", headless, "'.lower() == 'true' else '--no-headless'"]
            ),
        ],
        output="screen",
        additional_env={"PYTHONUNBUFFERED": "1"},
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "navigation_scenario.launch.py")
        ),
        launch_arguments={
            "scenario": scenario,
            "robot_x": LaunchConfiguration("robot_x"),
            "robot_y": LaunchConfiguration("robot_y"),
            "robot_z": LaunchConfiguration("robot_z"),
            "robot_yaw": LaunchConfiguration("robot_yaw"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_navigation")),
    )
    scans = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "scan_scenario.launch.py")
        ),
        launch_arguments={
            "scenario": scenario,
            "use_navigation": LaunchConfiguration("use_navigation"),
        }.items(),
    )
    stop_when_simulation_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=simulation,
            on_exit=[Shutdown(reason="Isaac Sim process exited")],
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="warehouse_spot",
                description="Scenario preset from config/scenarios",
            ),
            DeclareLaunchArgument(
                "isaac_python",
                default_value=_default_isaac_python(),
                description="Isaac Sim python.sh path; set ISAACSIM_PYTHON normally",
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            # Set defaults here for a preferred starting pose, or pass these
            # arguments on the command line. Empty values use the scenario.
            DeclareLaunchArgument("robot_x", default_value="", description="First robot x (m)"),
            DeclareLaunchArgument("robot_y", default_value="", description="First robot y (m)"),
            DeclareLaunchArgument("robot_z", default_value="", description="First robot spawn height (m)"),
            DeclareLaunchArgument("robot_yaw", default_value="", description="First robot yaw (rad)"),
            DeclareLaunchArgument(
                "view",
                default_value="follow",
                choices=["follow", "top_down", "free"],
                description="Robot-following, overhead, or manual viewport camera",
            ),
            DeclareLaunchArgument("max_frames", default_value="0"),
            DeclareLaunchArgument("duration_sec", default_value="0.0"),
            DeclareLaunchArgument("seed", default_value="-1"),
            DeclareLaunchArgument("generation_output_dir", default_value=""),
            DeclareLaunchArgument("use_navigation", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument(
                "use_pedestrians", default_value="false", choices=["true", "false"],
                description="Spawn the selected scenario's pedestrian preset only when true",
            ),
            stop_when_simulation_exits,
            simulation,
            scans,
            navigation,
        ]
    )
