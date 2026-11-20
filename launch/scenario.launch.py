# SPDX-License-Identifier: Apache-2.0
"""Start Isaac Sim composition and the matching Nav2 scenario."""

import os
from pathlib import Path

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
            "--max-frames",
            LaunchConfiguration("max_frames"),
            "--duration-sec",
            LaunchConfiguration("duration_sec"),
            "--seed",
            LaunchConfiguration("seed"),
            "--generation-output-dir",
            LaunchConfiguration("generation_output_dir"),
            "--semantic-plan-dir",
            LaunchConfiguration("semantic_plan_dir"),
            "--route-band-half-width-m",
            LaunchConfiguration("route_band_half_width_m"),
            "--placement-cell-size-m",
            LaunchConfiguration("placement_cell_size_m"),
            "--route-band-fraction",
            LaunchConfiguration("route_band_fraction"),
            "--run-report",
            LaunchConfiguration("run_report"),
            "--record-crowd-states",
            LaunchConfiguration("record_crowd_states"),
            "--view",
            LaunchConfiguration("view"),
            "--robot-task",
            LaunchConfiguration("robot_task"),
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
        launch_arguments={"scenario": scenario}.items(),
        condition=IfCondition(LaunchConfiguration("use_navigation")),
    )
    scans = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "scan_scenario.launch.py")
        ),
        launch_arguments={"scenario": scenario}.items(),
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
            DeclareLaunchArgument("route_band_half_width_m", default_value="3.0"),
            DeclareLaunchArgument("placement_cell_size_m", default_value="2.0"),
            DeclareLaunchArgument("route_band_fraction", default_value="0.7"),
            DeclareLaunchArgument(
                "robot_task", default_value="auto", choices=["auto", "true", "false"]
            ),
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
            DeclareLaunchArgument("semantic_plan_dir", default_value=""),
            DeclareLaunchArgument("run_report", default_value=""),
            DeclareLaunchArgument(
                "record_crowd_states", default_value="auto", choices=["auto", "true", "false"]
            ),
            DeclareLaunchArgument("use_navigation", default_value="true"),
            stop_when_simulation_exits,
            simulation,
            scans,
            navigation,
        ]
    )
