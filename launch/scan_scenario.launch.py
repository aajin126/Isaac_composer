"""Run scan processing for scenario robots without a Nav2 stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from isaac_composer.config import load_navigation_profile, load_scenario_bundle
from isaac_composer.scan_launch import create_scan_actions


def _launch_scans(context):
    root = Path(get_package_share_directory("isaac_composer")) / "config"
    bundle = load_scenario_bundle(LaunchConfiguration("scenario").perform(context), root)
    use_navigation = LaunchConfiguration("use_navigation").perform(context).lower() == "true"
    actions = []
    for robot in bundle["robots"]:
        instance = robot["instance"]
        profile_name = instance.get("scan_profile")
        if not profile_name or (use_navigation and instance["navigation"]["enabled"]):
            continue
        profile = load_navigation_profile(profile_name, root)
        if not profile["supported"]:
            raise ValueError(f"Unsupported scan profile: {profile_name}")
        actions.extend(create_scan_actions(
            profile,
            namespace=instance.get("namespace", "").strip("/"),
            use_sim_time=bundle["navigation"]["settings"].get("use_sim_time", True),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="warehouse_a2_random"),
        DeclareLaunchArgument("use_navigation", default_value="false"),
        OpaqueFunction(function=_launch_scans),
    ])
