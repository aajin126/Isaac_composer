# SPDX-License-Identifier: Apache-2.0
"""Generic Nav2 bringup for one scenario robot instance."""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from isaac_composer.scan_launch import converter_package


def _as_bool(value):
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _rewrite_string(value):
    if isinstance(value, bool):
        return str(value)
    return str(value)


def _mapping_argument(context, name):
    raw = LaunchConfiguration(name).perform(context)
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError(f"launch argument {name!r} must encode a JSON object")
    return value


def _list_argument(context, name):
    raw = LaunchConfiguration(name).perform(context)
    value = json.loads(raw)
    if not isinstance(value, list):
        raise ValueError(f"launch argument {name!r} must encode a JSON array")
    return value


def _launch_navigation(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    use_namespace = bool(namespace)
    use_sim_time = _as_bool(LaunchConfiguration("use_sim_time").perform(context))
    autostart = _as_bool(LaunchConfiguration("autostart").perform(context))
    use_rviz = _as_bool(LaunchConfiguration("use_rviz").perform(context))
    use_composition = _as_bool(LaunchConfiguration("use_composition").perform(context))
    pointcloud_enabled = _as_bool(
        LaunchConfiguration("pointcloud_enabled").perform(context)
    )
    lidar_fusion_enabled = _as_bool(
        LaunchConfiguration("lidar_fusion_enabled").perform(context)
    )

    rewrites = _mapping_argument(context, "parameter_overrides")
    rewrites.update(
        {
            "amcl.ros__parameters.initial_pose.x": LaunchConfiguration(
                "initial_x"
            ).perform(context),
            "amcl.ros__parameters.initial_pose.y": LaunchConfiguration(
                "initial_y"
            ).perform(context),
            "amcl.ros__parameters.initial_pose.z": LaunchConfiguration(
                "initial_z"
            ).perform(context),
            "amcl.ros__parameters.initial_pose.yaw": LaunchConfiguration(
                "initial_yaw"
            ).perform(context),
        }
    )
    rewrites = {key: _rewrite_string(value) for key, value in rewrites.items()}

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites=rewrites,
        convert_types=True,
    )
    nav2_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
    )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, "bringup_launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": str(use_namespace),
                "map": LaunchConfiguration("map"),
                "params_file": configured_params,
                "use_sim_time": str(use_sim_time),
                "autostart": str(autostart),
                "use_composition": str(use_composition),
            }.items(),
        )
    ]

    if lidar_fusion_enabled:
        actions.append(
            Node(
                package="isaac_composer",
                executable="lidar_fusion_node",
                name="lidar_fusion_node",
                namespace=namespace,
                parameters=[
                    {
                        "input_topics": _list_argument(context, "lidar_fusion_input_topics"),
                        "output_topic": LaunchConfiguration("lidar_fusion_output_topic").perform(
                            context
                        ),
                        "target_frame": LaunchConfiguration(
                            "lidar_fusion_target_frame"
                        ).perform(context),
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            )
        )

    if pointcloud_enabled:
        converter_parameters = _mapping_argument(context, "pointcloud_parameters")
        converter_parameters["use_sim_time"] = use_sim_time
        cloud_topics = _list_argument(context, "cloud_topics")
        converter_scan_topics = _list_argument(context, "converter_scan_topics")
        if not cloud_topics:
            cloud_topics = [LaunchConfiguration("cloud_topic").perform(context)]
        if not converter_scan_topics:
            converter_scan_topics = [LaunchConfiguration("scan_topic").perform(context)]
        if len(cloud_topics) != len(converter_scan_topics):
            raise ValueError("cloud_topics and converter_scan_topics must have equal length")

        for index, (cloud_topic, converter_scan_topic) in enumerate(
            zip(cloud_topics, converter_scan_topics), start=1
        ):
            actions.append(
                Node(
                    package=converter_package(converter_parameters),
                    executable="pointcloud_to_laserscan_node",
                    name=f"pointcloud_to_laserscan_{index}",
                    namespace=namespace,
                    remappings=[
                        ("cloud_in", cloud_topic),
                        ("scan", converter_scan_topic),
                    ],
                    parameters=[converter_parameters],
                    output="screen",
                )
            )

        if len(converter_scan_topics) > 1:
            scan_fusion_parameters = _mapping_argument(
                context, "scan_fusion_parameters"
            )
            scan_fusion_parameters.update(
                {
                    "input_topics": converter_scan_topics,
                    "output_topic": LaunchConfiguration("scan_topic").perform(
                        context
                    ),
                    "use_sim_time": use_sim_time,
                }
            )
            actions.append(
                Node(
                    package="isaac_composer",
                    executable="laser_scan_fusion_node",
                    name="laser_scan_fusion_node",
                    namespace=namespace,
                    parameters=[scan_fusion_parameters],
                    output="screen",
                )
            )

    if use_rviz:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, "rviz_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_namespace": str(use_namespace),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("rviz_config"),
            DeclareLaunchArgument("use_sim_time", default_value="True"),
            DeclareLaunchArgument("autostart", default_value="True"),
            DeclareLaunchArgument("use_rviz", default_value="True"),
            # Isaac Sim initializes its ROS 2 bridge in parallel with Nav2.  With
            # composition enabled, Humble occasionally leaves the container's
            # load_node request waiting forever during that discovery burst.
            # Standalone lifecycle nodes avoid that startup race and are a more
            # reliable default for these single-robot simulation scenarios.
            DeclareLaunchArgument("use_composition", default_value="False"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("pointcloud_enabled", default_value="false"),
            DeclareLaunchArgument("cloud_topic", default_value="pointcloud"),
            DeclareLaunchArgument("cloud_topics", default_value="[]"),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument("converter_scan_topics", default_value="[]"),
            DeclareLaunchArgument("pointcloud_parameters", default_value="{}"),
            DeclareLaunchArgument("scan_fusion_parameters", default_value="{}"),
            DeclareLaunchArgument("lidar_fusion_enabled", default_value="false"),
            DeclareLaunchArgument("lidar_fusion_input_topics", default_value="[]"),
            DeclareLaunchArgument("lidar_fusion_output_topic", default_value="points"),
            DeclareLaunchArgument("lidar_fusion_target_frame", default_value=""),
            DeclareLaunchArgument("parameter_overrides", default_value="{}"),
            OpaqueFunction(function=_launch_navigation),
        ]
    )
