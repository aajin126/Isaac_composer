# SPDX-License-Identifier: Apache-2.0
"""Launch every Nav2 instance declared by an Isaac Composer scenario."""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from isaac_composer.config import load_scenario_bundle
from isaac_composer.navigation import build_navigation_plan


def _bool_text(value):
    return str(value).lower()


def _launch_scenario_navigation(context):
    scenario_name = LaunchConfiguration("scenario").perform(context)
    package_share = get_package_share_directory("isaac_composer")
    bundle = load_scenario_bundle(
        scenario_name,
        config_root=os.path.join(package_share, "config"),
    )
    plan = build_navigation_plan(bundle, package_root=package_share)
    if not plan.enabled:
        return [
            LogInfo(msg=f"Scenario {plan.scenario_name}: navigation is disabled")
        ]

    generic_launch = PythonLaunchDescriptionSource(
        os.path.join(package_share, "launch", "navigation.launch.py")
    )
    instance_actions = []
    for instance in plan.instances:
        pose = instance.initial_pose
        instance_actions.extend(
            [
                LogInfo(
                    msg=(
                        f"Launching Nav2 for {instance.instance_name} "
                        f"in namespace /{instance.namespace}"
                        if instance.namespace
                        else f"Launching Nav2 for {instance.instance_name} at root namespace"
                    )
                ),
                IncludeLaunchDescription(
                    generic_launch,
                    launch_arguments={
                        "namespace": instance.namespace,
                        "map": str(instance.map_file),
                        "params_file": str(instance.params_file),
                        "rviz_config": str(instance.rviz_file),
                        "use_sim_time": _bool_text(instance.use_sim_time),
                        "autostart": _bool_text(instance.autostart),
                        "use_rviz": _bool_text(instance.use_rviz),
                        "initial_x": str(pose["x"]),
                        "initial_y": str(pose["y"]),
                        "initial_z": str(pose["z"]),
                        "initial_yaw": str(pose["yaw"]),
                        "pointcloud_enabled": _bool_text(
                            instance.pointcloud_enabled
                        ),
                        "cloud_topic": instance.cloud_topic,
                        "cloud_topics": json.dumps(instance.cloud_topics),
                        "scan_topic": instance.scan_topic,
                        "converter_scan_topics": json.dumps(
                            instance.converter_scan_topics
                        ),
                        "pointcloud_parameters": json.dumps(
                            instance.pointcloud_parameters
                        ),
                        "scan_fusion_parameters": json.dumps(
                            instance.scan_fusion_parameters
                        ),
                        "lidar_fusion_enabled": _bool_text(
                            instance.lidar_fusion_enabled
                        ),
                        "lidar_fusion_input_topics": json.dumps(
                            instance.lidar_fusion_input_topics
                        ),
                        "lidar_fusion_output_topic": instance.lidar_fusion_output_topic,
                        "lidar_fusion_target_frame": instance.lidar_fusion_target_frame,
                        "parameter_overrides": json.dumps(
                            instance.parameter_overrides
                        ),
                    }.items(),
                ),
            ]
        )

    if plan.startup_delay_sec > 0.0:
        return [
            LogInfo(
                msg=(
                    f"Scenario {plan.scenario_name}: waiting "
                    f"{plan.startup_delay_sec:g}s before Nav2 startup"
                )
            ),
            TimerAction(period=plan.startup_delay_sec, actions=instance_actions),
        ]
    return instance_actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="warehouse_spot",
                description="Scenario name from config/scenarios or an absolute YAML path",
            ),
            OpaqueFunction(function=_launch_scenario_navigation),
        ]
    )
