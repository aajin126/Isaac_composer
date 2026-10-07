"""Create the global ROS 2 simulation clock ActionGraph."""

from __future__ import annotations

from typing import Any


DEFAULT_GRAPH_PATH = "/World/ROS_Clock"


def create_clock(
    graph_path: str = DEFAULT_GRAPH_PATH,
    domain_id: int = 0,
    topic_name: str = "/clock",
    queue_size: int = 10,
    *,
    stage: Any | None = None,
) -> bool:
    """Create the ROS 2 ``/clock`` graph after SimulationApp is ready."""

    import carb
    import omni.graph.core as og
    import omni.usd

    if stage is None:
        stage = omni.usd.get_context().get_stage()

    node_type_sets = (
        {
            "tick": "omni.graph.action.OnPlaybackTick",
            "context": "isaacsim.ros2.bridge.ROS2Context",
            "read_time": "isaacsim.core.nodes.IsaacReadSimulationTime",
            "publish_clock": "isaacsim.ros2.bridge.ROS2PublishClock",
        },
        {
            "tick": "omni.graph.action.OnPlaybackTick",
            "context": "omni.isaac.ros2_bridge.ROS2Context",
            "read_time": "omni.isaac.core_nodes.IsaacReadSimulationTime",
            "publish_clock": "omni.isaac.ros2_bridge.ROS2PublishClock",
        },
    )

    last_error: Exception | None = None
    for node_types in node_type_sets:
        try:
            remove_clock(graph_path, stage=stage)
            keys = og.Controller.Keys
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", node_types["tick"]),
                        ("ROS2Context", node_types["context"]),
                        ("ReadSimulationTime", node_types["read_time"]),
                        ("PublishClock", node_types["publish_clock"]),
                    ],
                    keys.CONNECT: [
                        (
                            "OnPlaybackTick.outputs:tick",
                            "PublishClock.inputs:execIn",
                        ),
                        (
                            "ROS2Context.outputs:context",
                            "PublishClock.inputs:context",
                        ),
                        (
                            "ReadSimulationTime.outputs:simulationTime",
                            "PublishClock.inputs:timeStamp",
                        ),
                    ],
                    keys.SET_VALUES: [
                        ("ROS2Context.inputs:domain_id", int(domain_id)),
                        ("ROS2Context.inputs:useDomainIDEnvVar", True),
                        ("PublishClock.inputs:topicName", topic_name),
                        ("PublishClock.inputs:queueSize", int(queue_size)),
                    ],
                },
            )
            carb.log_info(
                f"Created ROS2 clock graph at {graph_path}, "
                f"topic={topic_name}, domain_id={domain_id}"
            )
            return True
        except Exception as error:
            last_error = error
            carb.log_warn(
                f"Failed to create clock graph with node types {node_types}: "
                f"{error}"
            )

    carb.log_error(f"Failed to create ROS2 clock graph. Last error: {last_error}")
    return False


def remove_clock(
    graph_path: str = DEFAULT_GRAPH_PATH,
    *,
    stage: Any | None = None,
) -> None:
    """Remove a clock graph created by :func:`create_clock`, if present."""

    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(graph_path)
    if prim and prim.IsValid():
        stage.RemovePrim(graph_path)
