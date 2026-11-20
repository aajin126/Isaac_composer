from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from isaac_composer.config import (
    ConfigError,
    ROBOT_SECTIONS,
    load_navigation_map,
    load_navigation_profile,
    load_robot_config,
    load_scenario_bundle,
    load_sensor_preset,
)
from isaac_composer.navigation import build_navigation_plan
from isaac_composer.registry import ProviderRegistry, RegistryError


def test_scenario_loading_does_not_import_simulation_dependencies():
    import subprocess
    import sys

    # A fresh interpreter catches dependencies hidden by pytest's import cache.
    script = '''
import importlib.abc
import sys

class BlockSimulationImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('numpy', 'scipy', 'omni', 'isaacsim') or fullname == 'isaac_composer.robot_task':
            raise AssertionError('Configuration imported simulation dependency: ' + fullname)

sys.meta_path.insert(0, BlockSimulationImports())
from isaac_composer.config import load_scenario_bundle
for name in ('warehouse_a2_random', 'hospital_a2_random', 'warehouse_a2_fixed'):
    bundle = load_scenario_bundle(name)
    assert bundle['robot_task']['min_goal_distance_m'] > 0
    assert bundle['robot_task']['max_goal_distance_m'] >= bundle['robot_task']['min_goal_distance_m']
assert load_scenario_bundle('warehouse_a2_random')['robot_task']['enabled']
'''
    result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


class RobotConfigTests(unittest.TestCase):
    def test_all_robot_configs_use_the_same_required_sections(self):
        for robot_name in ("a2", "spot", "nova_carter"):
            with self.subTest(robot=robot_name):
                config = load_robot_config(robot_name)
                self.assertTrue(set(ROBOT_SECTIONS).issubset(config))
                self.assertNotIn("pose", config["robot"])

    def test_generated_lidars_have_explicit_mounts_and_ros_outputs(self):
        for robot_name in ("a2", "spot"):
            config = load_robot_config(robot_name)
            for sensor in config["sensors"]["items"]:
                self.assertEqual(sensor["source"], "generated")
                self.assertIn("parent", sensor["mount"])
                self.assertEqual(len(sensor["mount"]["translation"]), 3)
                self.assertEqual(len(sensor["mount"]["orientation_quat_wxyz"]), 4)
                self.assertIn("topic", sensor["ros"])
                self.assertIn("frame_id", sensor["ros"])

    def test_a2_lidar_mounts_match_the_vendor_transform_chain(self):
        config = load_robot_config("a2")
        front_lidar = config["sensors"]["items"][0]
        rear_lidar = config["sensors"]["items"][1]
        self.assertEqual(front_lidar["name"], "front_lidar")
        self.assertEqual(
            front_lidar["mount"]["orientation_quat_wxyz"],
            [0.5, 0.5, 0.5, 0.5],
        )
        self.assertEqual(rear_lidar["name"], "rear_lidar")
        self.assertEqual(
            rear_lidar["mount"]["parent"],
            "sensor_mount:front_lidar",
        )
        self.assertEqual(
            rear_lidar["mount"]["translation"],
            [0.0, 0.005997, -0.617643765],
        )
        self.assertEqual(
            rear_lidar["mount"]["orientation_quat_wxyz"],
            [0.0, 0.0, 1.0, 0.0],
        )
        front_translation = front_lidar["mount"]["translation"]
        rear_relative = rear_lidar["mount"]["translation"]
        # lidar2baselink_R maps [x, y, z] to [z, x, y].
        rear_in_base = [
            front_translation[0] + rear_relative[2],
            front_translation[1] + rear_relative[0],
            front_translation[2] + rear_relative[1],
        ]
        self.assertAlmostEqual(rear_in_base[0], -0.279973765)
        self.assertAlmostEqual(rear_in_base[1], 0.0)
        self.assertAlmostEqual(rear_in_base[2], 0.087337)

    def test_nova_carter_declares_embedded_ownership(self):
        config = load_robot_config("nova_carter")
        self.assertEqual(config["controller"]["provider"], "embedded")
        self.assertEqual(config["sensors"]["provider"], "embedded")
        self.assertEqual(config["ros"]["provider"], "embedded")
        self.assertEqual(config["sensors"]["items"], [])

    def test_hesai_preset_resolves(self):
        preset = load_sensor_preset("hesai_jt128")
        self.assertEqual(preset["type"], "rtx_lidar")
        self.assertEqual(
            preset["implementation"]["provider"],
            "isaac_sim_5_legacy_profile",
        )

    def test_local_robot_and_sensor_assets_stay_inside_package(self):
        package_root = Path(__file__).resolve().parents[1]
        config_root = package_root / "config"

        a2 = load_robot_config("a2", config_root)
        a2_usd = (package_root / a2["robot"]["usd_path"]).resolve()

        hesai = load_sensor_preset("hesai_jt128", config_root)
        hesai_json = (
            package_root
            / hesai["implementation"]["source_json"]
        ).resolve()

        for asset in (a2_usd, hesai_json):
            with self.subTest(asset=asset):
                self.assertIn(package_root.resolve(), asset.parents)
                self.assertTrue(asset.is_file())

        for layer in (
            "a2_base.usd",
            "a2_physics.usd",
            "a2_robot.usd",
            "a2_sensor.usd",
        ):
            with self.subTest(layer=layer):
                self.assertTrue((a2_usd.parent / "configuration" / layer).is_file())

    def test_unknown_robot_is_rejected(self):
        with self.assertRaises(ConfigError):
            load_robot_config("does_not_exist")


class ScenarioConfigTests(unittest.TestCase):
    def test_every_scenario_resolves_its_complete_bundle(self):
        expected = {
            "warehouse_a2": "a2",
            "warehouse_spot": "spot",
            "warehouse_nova_carter": "nova_carter",
        }
        for scenario_name, robot_name in expected.items():
            with self.subTest(scenario=scenario_name):
                bundle = load_scenario_bundle(scenario_name)
                self.assertEqual(bundle["environment"]["name"], "warehouse")
                self.assertEqual(bundle["simulation"]["name"], "default")
                self.assertEqual(bundle["pedestrians"]["name"], "default")
                self.assertEqual(bundle["robots"][0]["preset"]["name"], robot_name)
                self.assertIn("pose", bundle["robots"][0]["instance"])

    def test_navigation_assets_resolve_for_single_robot_scenarios(self):
        expected = {
            "warehouse_spot": ("spot", 5.0),
            "warehouse_nova_carter": ("nova_carter", 10.0),
        }
        for scenario_name, (profile_name, delay) in expected.items():
            with self.subTest(scenario=scenario_name):
                bundle = load_scenario_bundle(scenario_name)
                plan = build_navigation_plan(bundle)
                self.assertTrue(plan.enabled)
                self.assertEqual(plan.startup_delay_sec, delay)
                self.assertEqual(len(plan.instances), 1)
                instance = plan.instances[0]
                self.assertEqual(
                    bundle["robots"][0]["navigation_profile"]["name"],
                    profile_name,
                )
                self.assertTrue(instance.map_file.is_file())
                self.assertTrue(instance.params_file.is_file())
                self.assertTrue(instance.rviz_file.is_file())
                self.assertFalse(instance.use_namespace)

    def test_a2_scenario_uses_policy_navigation_contract(self):
        plan = build_navigation_plan(load_scenario_bundle("warehouse_a2"))
        self.assertTrue(plan.enabled)
        self.assertEqual(len(plan.instances), 1)
        profile = load_navigation_profile("a2")
        self.assertTrue(profile["supported"])
        self.assertFalse(profile["lidar_fusion"]["enabled"])
        self.assertEqual(profile["lidar_fusion"]["target_frame"], "base_link")
        self.assertEqual(
            profile["pointcloud_to_laserscan"]["cloud_topic"],
            "unitree/slam_lidar/points1",
        )
        self.assertEqual(
            profile["pointcloud_to_laserscan"]["cloud_topics"],
            [
                "unitree/slam_lidar/points1",
                "unitree/slam_lidar/points2",
            ],
        )
        self.assertEqual(
            profile["pointcloud_to_laserscan"]["parameters"]["target_frame"],
            "base_link",
        )

        instance = plan.instances[0]
        self.assertEqual(
            instance.cloud_topics,
            profile["pointcloud_to_laserscan"]["cloud_topics"],
        )
        self.assertEqual(instance.converter_scan_topics, ["scan_front", "scan_rear"])
        self.assertAlmostEqual(instance.scan_fusion_parameters["first_angle_max"], 1.5707963267948966)

        package_root = Path(__file__).resolve().parents[1]
        robot = load_robot_config("a2")
        policy_path = package_root / robot["controller"]["policy_path"]
        self.assertEqual(robot["spawn"]["provider"], "a2_policy")
        self.assertEqual(robot["controller"]["provider"], "from_spawner")
        self.assertEqual(robot["ros"]["robot_bridge"], "a2_mobile_bridge")
        self.assertTrue(policy_path.is_file())

    def test_multi_robot_scenarios_share_one_relative_topic_profile(self):
        cases = {
            "hospital_multi_nova_carter": 7,
            "office_multi_nova_carter": 10,
        }
        for scenario_name, costmap_size in cases.items():
            with self.subTest(scenario=scenario_name):
                plan = build_navigation_plan(load_scenario_bundle(scenario_name))
                self.assertEqual(len(plan.instances), 3)
                self.assertEqual(
                    [item.namespace for item in plan.instances],
                    ["carter1", "carter2", "carter3"],
                )
                self.assertEqual(len({item.params_file for item in plan.instances}), 1)
                for instance in plan.instances:
                    self.assertFalse(instance.cloud_topic.startswith("/"))
                    self.assertFalse(instance.scan_topic.startswith("/"))
                    self.assertEqual(
                        instance.parameter_overrides[
                            "local_costmap.local_costmap.ros__parameters.width"
                        ],
                        costmap_size,
                    )

    def test_all_map_presets_point_to_existing_assets(self):
        for map_name in ("warehouse", "full_warehouse", "hospital", "office"):
            with self.subTest(map=map_name):
                map_config = load_navigation_map(map_name)
                root = load_scenario_bundle("warehouse_spot")["config_root"].parent
                self.assertTrue((root / map_config["map_file"]).is_file())

    def test_navigation_assets_work_through_symlink_install_share(self):
        bundle = load_scenario_bundle("warehouse_spot")
        package_root = Path(bundle["config_root"]).parent
        with TemporaryDirectory() as temporary_directory:
            symlink_share = Path(temporary_directory) / "isaac_composer"
            symlink_share.symlink_to(package_root, target_is_directory=True)
            plan = build_navigation_plan(bundle, package_root=symlink_share)
            self.assertTrue(plan.instances[0].map_file.is_file())


class RegistryTests(unittest.TestCase):
    def test_registry_constructs_registered_provider(self):
        registry = ProviderRegistry("test")

        @registry.register("example-provider")
        class Example:
            def __init__(self, value):
                self.value = value

        instance = registry.create("example_provider", 7)
        self.assertEqual(instance.value, 7)

    def test_duplicate_registration_is_rejected(self):
        registry = ProviderRegistry("test")

        @registry.register("duplicate")
        class First:
            pass

        with self.assertRaises(RegistryError):

            @registry.register("duplicate")
            class Second:
                pass


if __name__ == "__main__":
    unittest.main()
