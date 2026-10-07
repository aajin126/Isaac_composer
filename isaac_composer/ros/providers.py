"""ROS 2 ActionGraph and simulation-clock providers."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from isaac_composer.clock import DEFAULT_GRAPH_PATH, create_clock, remove_clock
from isaac_composer.models import RobotContext, SensorHandle
from isaac_composer.registry import clock_providers, ros_providers


def _topic(name: str, namespace: str) -> str:
    if not namespace:
        return name
    return f"/{namespace}/{name.lstrip('/')}"


def _node_name(prefix: str, value: str) -> str:
    return prefix + re.sub(r"[^A-Za-z0-9_]", "_", value.title())


class _ProviderBase:
    def __init__(
        self,
        *,
        package_root: str | Path,
        config_root: str | Path | None = None,
    ) -> None:
        del package_root, config_root


@ros_providers.register("embedded")
class EmbeddedRosProvider(_ProviderBase):
    """Configure ROS nodes already embedded in a referenced robot USD."""

    def setup_robot(
        self,
        robot: RobotContext,
        config: dict[str, Any],
    ) -> None:
        import carb
        import omni.usd
        from pxr import Usd

        namespace = robot.metadata.get("namespace", "")
        domain_id = int(config.get("domain_id", 0))
        root = omni.usd.get_context().get_stage().GetPrimAtPath(
            robot.root_prim_path
        )
        if not root or not root.IsValid():
            raise RuntimeError(f"Embedded ROS robot prim is missing: {robot.root_prim_path}")

        namespace_updates = 0
        domain_updates = 0
        for prim in Usd.PrimRange(root):
            namespace_attribute = prim.GetAttribute("inputs:nodeNamespace")
            if namespace_attribute and namespace_attribute.IsValid() and namespace:
                namespace_attribute.Set(namespace)
                namespace_updates += 1
            for attribute_name in ("inputs:domain_id", "inputs:domainId"):
                domain_attribute = prim.GetAttribute(attribute_name)
                if domain_attribute and domain_attribute.IsValid():
                    domain_attribute.Set(domain_id)
                    domain_updates += 1

        if namespace and namespace_updates == 0:
            carb.log_warn(
                f"No embedded ROS nodeNamespace inputs found below "
                f"{robot.root_prim_path}; namespace {namespace!r} was not applied"
            )
        carb.log_info(
            f"Configured embedded ROS graph for {robot.name}: "
            f"namespace_updates={namespace_updates}, domain_updates={domain_updates}"
        )

    def setup_sensors(
        self,
        robot: RobotContext,
        sensors: list[SensorHandle],
        config: dict[str, Any],
    ) -> None:
        del robot, sensors, config

    def shutdown(self) -> None:
        return None


@ros_providers.register("generated")
class GeneratedRosProvider(_ProviderBase):
    """Create robot and RTX LiDAR ROS 2 ActionGraphs from configuration."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.graph_paths: list[str] = []
        self.bridge: str | None = None

    def setup_robot(
        self,
        robot: RobotContext,
        config: dict[str, Any],
    ) -> None:
        self.bridge = config["robot_bridge"]
        if self.bridge in {"spot_mobile_bridge", "a2_mobile_bridge"}:
            self._create_mobile_graph(robot, config)
        elif self.bridge == "a2_joint_bridge":
            self._create_a2_graph(robot, config)
        else:
            raise ValueError(f"Unknown generated ROS robot bridge: {self.bridge}")

    def _create_mobile_graph(
        self,
        robot: RobotContext,
        config: dict[str, Any],
    ) -> None:
        import omni.graph.core as og

        graph_path = f"{robot.root_prim_path}/ROS2_ActionGraph"
        namespace = robot.metadata.get("namespace", "")
        base_link = robot.links.get("base")
        if base_link is None:
            raise RuntimeError(
                f"Mobile ROS bridge requires a configured base link for {robot.name}"
            )
        cmd_vel_topic = _topic(
            robot.metadata["controller_config"].get("cmd_vel_topic", "/cmd_vel"),
            namespace,
        )
        odom_topic = _topic(config.get("odom_topic", "/odom"), namespace)
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                    ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("Context.inputs:domain_id", int(config.get("domain_id", 0))),
                    ("Context.inputs:useDomainIDEnvVar", True),
                    ("SubscribeTwist.inputs:topicName", cmd_vel_topic),
                    ("ComputeOdometry.inputs:chassisPrim", [base_link.prim_path]),
                    ("PublishOdometry.inputs:topicName", odom_topic),
                    (
                        "PublishOdometry.inputs:odomFrameId",
                        config.get("odom_frame_id", "odom"),
                    ),
                    (
                        "PublishOdometry.inputs:chassisFrameId",
                        config.get("base_frame_id", "base_link"),
                    ),
                    ("PublishOdometry.inputs:publishRawVelocities", False),
                    (
                        "PublishOdomTF.inputs:parentFrameId",
                        config.get("odom_frame_id", "odom"),
                    ),
                    (
                        "PublishOdomTF.inputs:childFrameId",
                        config.get("base_frame_id", "base_link"),
                    ),
                    ("PublishOdomTF.inputs:topicName", _topic("/tf", namespace)),
                    ("PublishOdomTF.inputs:nodeNamespace", ""),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                    ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                    ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                    ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                    ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                    ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                    ("OnPlaybackTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
                    ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                    ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
                    ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                    ("Context.outputs:context", "PublishOdometry.inputs:context"),
                    ("Context.outputs:context", "PublishOdomTF.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
                ],
            },
        )
        self.graph_paths.append(graph_path)

    def _create_a2_graph(
        self,
        robot: RobotContext,
        config: dict[str, Any],
    ) -> None:
        import omni.graph.core as og

        if robot.articulation_prim_path is None:
            raise RuntimeError("A2 joint bridge requires an articulation root")
        graph_path = f"{robot.root_prim_path}/ROS2_ActionGraph"
        namespace = robot.metadata.get("namespace", "")
        controller = robot.metadata["controller_config"]
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("SubscribeJointCmd", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                keys.SET_VALUES: [
                    ("Context.inputs:domain_id", int(config.get("domain_id", 0))),
                    ("Context.inputs:useDomainIDEnvVar", True),
                    ("PublishTF.inputs:targetPrims", [robot.root_prim_path]),
                    ("PublishJointState.inputs:targetPrim", [robot.articulation_prim_path]),
                    (
                        "PublishJointState.inputs:topicName",
                        _topic(controller.get("state_topic", "joint_states"), namespace),
                    ),
                    (
                        "SubscribeJointCmd.inputs:topicName",
                        _topic(controller.get("command_topic", "joint_commands"), namespace),
                    ),
                    ("ArticulationController.inputs:targetPrim", [robot.articulation_prim_path]),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointCmd.inputs:execIn"),
                    ("SubscribeJointCmd.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("Context.outputs:context", "PublishTF.inputs:context"),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointCmd.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("SubscribeJointCmd.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointCmd.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                    ("SubscribeJointCmd.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                    ("SubscribeJointCmd.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ],
            },
        )
        self.graph_paths.append(graph_path)

    def setup_sensors(
        self,
        robot: RobotContext,
        sensors: list[SensorHandle],
        config: dict[str, Any],
    ) -> None:
        import omni.graph.core as og
        import omni.replicator.core as rep

        if not sensors:
            return
        graph_path = f"{robot.root_prim_path}/ROS2_ActionGraph"
        namespace = robot.metadata.get("namespace", "")
        nodes = []
        values = []
        connections = []
        for sensor in sensors:
            if sensor.sensor_type != "rtx_lidar":
                continue
            render_product = rep.create.render_product(sensor.sensor_prim_path, [1, 1])
            sensor.render_product = render_product
            publisher = _node_name("Publish", sensor.name)
            nodes.append((publisher, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))
            ros = sensor.metadata.get("ros", {})
            values.extend(
                [
                    (f"{publisher}.inputs:renderProductPath", render_product.path),
                    (f"{publisher}.inputs:topicName", _topic(sensor.topic or "points", namespace)),
                    (f"{publisher}.inputs:frameId", sensor.frame_id),
                    (f"{publisher}.inputs:type", ros.get("output", "point_cloud")),
                    (f"{publisher}.inputs:fullScan", bool(ros.get("full_scan", True))),
                    (f"{publisher}.inputs:queueSize", int(ros.get("queue_size", 1))),
                ]
            )
            connections.extend(
                [
                    (
                        f"{graph_path}/OnPlaybackTick.outputs:tick",
                        f"{publisher}.inputs:execIn",
                    ),
                    (
                        f"{graph_path}/Context.outputs:context",
                        f"{publisher}.inputs:context",
                    ),
                ]
            )

            if self.bridge in {"spot_mobile_bridge", "a2_mobile_bridge"}:
                transform = _node_name("PublishTF", sensor.name)
                nodes.append(
                    (transform, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")
                )
                w, x, y, z = sensor.metadata["orientation_quat_wxyz"]
                values.extend(
                    [
                        (f"{transform}.inputs:parentFrameId", sensor.metadata["parent_frame_id"]),
                        (f"{transform}.inputs:childFrameId", sensor.frame_id),
                        (f"{transform}.inputs:staticPublisher", True),
                        (f"{transform}.inputs:topicName", _topic("/tf_static", namespace)),
                        (f"{transform}.inputs:nodeNamespace", ""),
                        (f"{transform}.inputs:translation", sensor.metadata["translation"]),
                        (f"{transform}.inputs:rotation", [x, y, z, w]),
                    ]
                )
                connections.extend(
                    [
                        (
                            f"{graph_path}/OnPlaybackTick.outputs:tick",
                            f"{transform}.inputs:execIn",
                        ),
                        (
                            f"{graph_path}/Context.outputs:context",
                            f"{transform}.inputs:context",
                        ),
                        (
                            f"{graph_path}/ReadSimTime.outputs:simulationTime",
                            f"{transform}.inputs:timeStamp",
                        ),
                    ]
                )

        if nodes:
            keys = og.Controller.Keys
            og.Controller.edit(
                # setup_robot() owns creation of the robot ActionGraph.  Passing
                # a graph descriptor here asks OmniGraph to create that graph a
                # second time; passing its path opens the existing graph so the
                # sensor nodes can be attached modularly.
                graph_path,
                {
                    keys.CREATE_NODES: nodes,
                    keys.SET_VALUES: values,
                    keys.CONNECT: connections,
                },
            )

    def shutdown(self) -> None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        for graph_path in reversed(self.graph_paths):
            prim = stage.GetPrimAtPath(graph_path)
            if prim and prim.IsValid():
                stage.RemovePrim(graph_path)
        self.graph_paths.clear()


@clock_providers.register("embedded")
class EmbeddedClockProvider(_ProviderBase):
    def setup(self, config: dict[str, Any], *, stage: Any) -> None:
        del config, stage

    def shutdown(self) -> None:
        return None


@clock_providers.register("generated")
class GeneratedClockProvider(_ProviderBase):
    """Publish one global ROS 2 /clock from Isaac simulation time."""

    graph_path = DEFAULT_GRAPH_PATH

    def setup(self, config: dict[str, Any], *, stage: Any) -> None:
        created = create_clock(
            graph_path=self.graph_path,
            domain_id=int(config.get("domain_id", 0)),
            topic_name=config.get("topic", "/clock"),
            queue_size=int(config.get("queue_size", 10)),
            stage=stage,
        )
        if not created:
            raise RuntimeError("Failed to create ROS 2 clock graph")

    def shutdown(self) -> None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        remove_clock(self.graph_path, stage=stage)
