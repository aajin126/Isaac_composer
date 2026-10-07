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


def _default_isaac_python():
    """Prefer an explicit or standalone Isaac Sim interpreter over python3."""

    candidates = [os.environ.get("ISAACSIM_PYTHON", "")]
    package_path = os.environ.get("isaac_sim_package_path", "")
    if package_path:
        candidates.append(str(Path(package_path) / "python.sh"))
    candidates.extend(
        [
            str(Path.home() / "develop" / "isaacsim" / "python.sh"),
            str(Path.home() / "isaacsim" / "python.sh"),
        ]
    )
    return next((path for path in candidates if path and Path(path).is_file()), "python3")


def generate_launch_description():
    package_share = get_package_share_directory("isaac_composer")
    scenario = LaunchConfiguration("scenario")
    headless = LaunchConfiguration("headless")
    simulation = ExecuteProcess(
        cmd=[
            LaunchConfiguration("isaac_python"),
            os.path.join(package_share, "scripts", "run_scenario.py"),
            scenario,
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
            DeclareLaunchArgument("use_navigation", default_value="true"),
            stop_when_simulation_exits,
            simulation,
            navigation,
        ]
    )
