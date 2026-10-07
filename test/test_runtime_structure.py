from __future__ import annotations

import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock, patch

from isaac_composer.assets import as_reference_url, resolve_asset
from isaac_composer.clock import DEFAULT_GRAPH_PATH
from isaac_composer.controllers.a2_policy import _joint_parameters, build_observation
from isaac_composer.registry import (
    clock_providers,
    controller_providers,
    ros_providers,
    sensor_providers,
    spawn_providers,
)
from isaac_composer.runtime import load_builtin_providers
from isaac_composer.models import RobotContext, SensorHandle
from isaac_composer.ros.providers import GeneratedClockProvider, GeneratedRosProvider


class RuntimeStructureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        load_builtin_providers()

    def test_all_configured_provider_names_are_implemented(self):
        self.assertTrue(
            {"usd_reference", "spot_policy", "a2_policy"}.issubset(
                spawn_providers.names()
            )
        )
        self.assertTrue(
            {"embedded", "joint_command_graph", "from_spawner"}.issubset(
                controller_providers.names()
            )
        )
        self.assertTrue({"embedded", "generated"}.issubset(sensor_providers.names()))
        self.assertTrue({"embedded", "generated"}.issubset(ros_providers.names()))
        self.assertTrue({"embedded", "generated"}.issubset(clock_providers.names()))

    def test_package_owned_asset_resolution(self):
        package_root = Path(__file__).resolve().parents[1]
        resolved = resolve_asset("usd/a2/a2.usd", package_root)
        self.assertEqual(Path(resolved), package_root / "usd/a2/a2.usd")
        self.assertTrue(as_reference_url(resolved).startswith("file://"))

    def test_runtime_modules_do_not_require_isaac_imports_at_import_time(self):
        # Reaching this assertion under ordinary CPython proves registrations and
        # orchestration can be tested without constructing SimulationApp.
        self.assertIn("spot_policy", spawn_providers.names())

    def test_composer_owns_the_global_clock_module(self):
        self.assertEqual(DEFAULT_GRAPH_PATH, "/World/ROS_Clock")

    def test_generated_clock_provider_calls_composer_clock_module(self):
        provider = GeneratedClockProvider(package_root=Path(__file__).parents[1])
        stage = object()
        with patch(
            "isaac_composer.ros.providers.create_clock",
            return_value=True,
        ) as create_clock:
            provider.setup(
                {
                    "domain_id": 7,
                    "topic": "/simulation_clock",
                    "queue_size": 3,
                },
                stage=stage,
            )

        create_clock.assert_called_once_with(
            graph_path="/World/ROS_Clock",
            domain_id=7,
            topic_name="/simulation_clock",
            queue_size=3,
            stage=stage,
        )

    def test_generated_sensor_publishers_extend_existing_robot_graph(self):
        keys = SimpleNamespace(
            CREATE_NODES="create_nodes",
            SET_VALUES="set_values",
            CONNECT="connect",
        )
        controller = SimpleNamespace(Keys=keys, edit=MagicMock())
        graph_module = SimpleNamespace(Controller=controller)
        render_product = SimpleNamespace(path="/Render/Product")
        replicator_module = SimpleNamespace(
            create=SimpleNamespace(
                render_product=MagicMock(return_value=render_product),
            )
        )
        omni_module = ModuleType("omni")
        graph_package = ModuleType("omni.graph")
        graph_package.core = graph_module
        replicator_package = ModuleType("omni.replicator")
        replicator_package.core = replicator_module
        omni_module.graph = graph_package
        omni_module.replicator = replicator_package
        sensor = SensorHandle(
            name="front_lidar",
            sensor_type="rtx_lidar",
            mount_prim_path="/World/Spot/body/front_3d_lidar",
            sensor_prim_path="/World/Spot/body/front_3d_lidar/lidar",
            frame_id="front_3d_lidar",
            topic="/front_3d_lidar/lidar_points",
            metadata={
                "ros": {"output": "point_cloud"},
                "parent_frame_id": "base_link",
                "translation": [0.0, 0.0, 0.1],
                "orientation_quat_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
        )
        robot = RobotContext(
            name="spot_1",
            root_prim_path="/World/Spot",
        )
        provider = GeneratedRosProvider(package_root=Path(__file__).parents[1])
        provider.bridge = "spot_mobile_bridge"

        with patch.dict(
            "sys.modules",
            {
                "omni": omni_module,
                "omni.graph": graph_package,
                "omni.graph.core": graph_module,
                "omni.replicator": replicator_package,
                "omni.replicator.core": replicator_module,
            },
        ):
            provider.setup_sensors(robot, [sensor], {})

        controller.edit.assert_called_once()
        graph_target, edits = controller.edit.call_args.args
        self.assertEqual(graph_target, "/World/Spot/ROS2_ActionGraph")
        self.assertIsInstance(graph_target, str)
        self.assertIn(
            ("PublishFront_Lidar", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            edits[keys.CREATE_NODES],
        )
        self.assertIn(
            (
                "/World/Spot/ROS2_ActionGraph/OnPlaybackTick.outputs:tick",
                "PublishFront_Lidar.inputs:execIn",
            ),
            edits[keys.CONNECT],
        )

    def test_mobile_odometry_uses_configured_base_link(self):
        keys = SimpleNamespace(
            CREATE_NODES="create_nodes",
            SET_VALUES="set_values",
            CONNECT="connect",
        )
        controller = SimpleNamespace(Keys=keys, edit=MagicMock())
        graph_module = SimpleNamespace(Controller=controller)
        omni_module = ModuleType("omni")
        graph_package = ModuleType("omni.graph")
        graph_package.core = graph_module
        omni_module.graph = graph_package
        robot = RobotContext(
            name="a2_1",
            root_prim_path="/World/A2",
            links={
                "base": SimpleNamespace(prim_path="/World/A2/base_link"),
            },
            metadata={
                "namespace": "",
                "controller_config": {"cmd_vel_topic": "/cmd_vel"},
            },
        )
        provider = GeneratedRosProvider(package_root=Path(__file__).parents[1])

        with patch.dict(
            "sys.modules",
            {
                "omni": omni_module,
                "omni.graph": graph_package,
                "omni.graph.core": graph_module,
            },
        ):
            provider.setup_robot(
                robot,
                {"robot_bridge": "a2_mobile_bridge"},
            )

        _, edits = controller.edit.call_args.args
        self.assertIn(
            ("ComputeOdometry.inputs:chassisPrim", ["/World/A2/base_link"]),
            edits[keys.SET_VALUES],
        )
        self.assertIn(
            ("PublishOdomTF.inputs:topicName", "/tf"),
            edits[keys.SET_VALUES],
        )

    def test_a2_policy_observation_matches_training_contract(self):
        joint_names = [
            f"{leg}_{joint}_joint"
            for leg in ("FL", "FR", "RL", "RR")
            for joint in ("hip", "thigh", "calf")
        ]
        default_position, stiffness, damping, max_effort = _joint_parameters(
            joint_names
        )
        observation = build_observation(
            linear_velocity_world=[1.0, 2.0, 3.0],
            angular_velocity_world=[0.1, 0.2, 0.3],
            orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
            command=[0.5, -0.25, 0.1],
            joint_position=default_position,
            joint_velocity=[0.0] * 12,
            default_position=default_position,
            last_action=[0.0] * 12,
        )

        self.assertEqual(observation.shape, (48,))
        self.assertEqual(observation[0:3].tolist(), [1.0, 2.0, 3.0])
        self.assertEqual(observation[6:9].tolist(), [0.0, 0.0, -1.0])
        self.assertEqual(default_position.tolist()[0:3], [-0.1, 0.9, -1.8])
        self.assertEqual(stiffness.tolist()[0:3], [100.0, 100.0, 150.0])
        self.assertEqual(damping.tolist()[0:3], [4.0, 4.0, 6.0])
        self.assertEqual(max_effort.tolist()[0:3], [120.0, 120.0, 180.0])


if __name__ == "__main__":
    unittest.main()
