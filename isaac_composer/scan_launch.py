"""Build standalone scan conversion nodes from an existing sensor profile."""

from __future__ import annotations

import copy

from launch_ros.actions import Node


def converter_package(parameters):
    """Select the thin reliable adapter only when the profile requests it."""
    return (
        "pointcloud_to_laserscan_reliable"
        if parameters.get("reliable_input", False)
        else "pointcloud_to_laserscan"
    )


def create_scan_actions(profile, namespace="", use_sim_time=True):
    converter = profile["pointcloud_to_laserscan"]
    if not converter["enabled"]:
        return []
    cloud_topics = converter.get("cloud_topics", [converter["cloud_topic"]])
    scan_topic = converter.get("scan_topic", profile["topics"]["scan"])
    scan_topics = converter.get("intermediate_scan_topics", [scan_topic])
    if not cloud_topics or len(cloud_topics) != len(scan_topics):
        raise ValueError("Scan conversion requires equal, nonempty cloud and scan topic lists")
    parameters = copy.deepcopy(converter.get("parameters", {}))
    parameters["use_sim_time"] = use_sim_time
    actions = [
        Node(
            package=converter_package(parameters),
            executable="pointcloud_to_laserscan_node",
            name=f"pointcloud_to_laserscan_{index}",
            namespace=namespace,
            remappings=[("cloud_in", cloud), ("scan", scan)],
            parameters=[parameters],
            output="screen",
        )
        for index, (cloud, scan) in enumerate(zip(cloud_topics, scan_topics), start=1)
    ]
    if len(scan_topics) > 1:
        fusion = copy.deepcopy(converter.get("fusion_parameters", {}))
        fusion.update(input_topics=scan_topics, output_topic=scan_topic, use_sim_time=use_sim_time)
        actions.append(
            Node(
                package="isaac_composer",
                executable="laser_scan_fusion_node",
                name="laser_scan_fusion_node",
                namespace=namespace,
                parameters=[fusion],
                output="screen",
            )
        )
    return actions
