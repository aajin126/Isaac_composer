from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
from isaac_composer.config import load_navigation_profile, load_scenario_bundle
from isaac_composer.navmesh import NavMeshScene
from isaac_composer.motion_events import scheduled_speed, validate_motion_events
from isaac_composer.pedestrians import _install_motion_schedule
from isaac_composer.scan_launch import create_scan_actions
ROOT = Path(__file__).resolve().parents[1]


def test_navmesh_uses_installed_radius_height_api():
    calls = []

    class Mesh:
        def query_closest_point(self, *, target, agent_radius, agent_height):
            calls.append((agent_radius, agent_height))
            return target, 0

        def query_shortest_path(self, *, start_pos, end_pos, agent_radius, agent_height, straighten):
            calls.append((agent_radius, agent_height))
            return SimpleNamespace(get_point_count=lambda: 2, get_points=lambda: [start_pos, end_pos])

    scene = NavMeshScene({"agent_radius": 0.4, "agent_height": 1.8})
    scene.navmesh = Mesh()
    with patch.dict("sys.modules", {"carb": SimpleNamespace(Float3=lambda *xyz: xyz)}):
        assert scene.project_world((1, 2, 0), agent_radius=0.45, agent_height=0.8) == (1, 2, 0)
        assert scene.shortest_path((0, 0, 0), (1, 2, 0)) == [(0, 0, 0), (1, 2, 0)]
    assert calls == [(0.45, 0.8), (0.4, 1.8)]



def test_timed_events_restore_speed_and_resolve_overlaps():
    events = validate_motion_events([
        {"action": "speed", "time_s": 3, "duration_s": 3, "speed_mps": 0.5},
        {"action": "stop", "time_s": 1, "duration_s": 3},
    ])
    assert [scheduled_speed(events, t, 1.2) for t in (0, 1, 3, 5, 6)] == [1.2, 0, 0.5, 0.5, 1.2]



@pytest.mark.parametrize("event", [
    {"action": "unknown", "time_s": 0, "duration_s": 1},
    {"action": "stop", "time_s": -1, "duration_s": 1},
    {"action": "stop", "time_s": 0, "duration_s": 0},
    {"action": "speed", "time_s": 0, "duration_s": 1, "speed_mps": float("nan")},
])
def test_invalid_events_fail_before_execution(event):
    with pytest.raises(ValueError):
        validate_motion_events([event])



def _person():
    targets = []
    person = SimpleNamespace(
        state=SimpleNamespace(position=np.array([0, 0, 0])),
        _stage_prefix="/World/Pedestrian", _target_speed=0.0,
        update_target_position=lambda points, walk_speed: targets.append(points),
    )
    callbacks = []
    world = SimpleNamespace(add_physics_callback=lambda name, fn: callbacks.append(fn))
    return world, person, targets, callbacks



def test_repeat_reaches_original_spawn_and_delay_is_respected():
    world, person, targets, callbacks = _person()
    _install_motion_schedule(world, person, [[1, 0, 0], [2, 0, 0]], 1, 1, True)
    tick = callbacks[0]
    tick(0.5)
    assert not targets
    tick(0.5)
    assert targets[0][0] == [0, 0, 0]
    person.state.position = np.array([2, 0, 0])
    person._current_point_index = len(targets[-1]) - 1
    tick(0.1)
    assert targets[-1][-1] == [0, 0, 0]
    person.state.position = np.array([0, 0, 0])
    person._current_point_index = len(targets[-1]) - 1
    tick(1)
    assert targets[-1][-1] == [2, 0, 0]



def test_stop_event_runs_after_delay_and_walk_resumes():
    world, person, targets, callbacks = _person()
    _install_motion_schedule(world, person, [[2, 0, 0]], 1.2, 2, False,
                             events=[{"action": "stop", "time_s": 1, "duration_s": 2}])
    tick = callbacks[0]
    tick(2)
    assert person._target_speed == 1.2
    tick(1)
    assert person._target_speed == 0
    tick(2)
    assert person._target_speed == 1.2
    person.state.position = np.array([2, 0, 0])
    person._current_point_index = len(targets[-1]) - 1
    tick(0.1)
    assert person._target_speed == 0
    tick(1)
    assert person._target_speed == 0



@pytest.mark.parametrize("name", ["warehouse_a2_random", "hospital_a2_random"])
def test_pedestrian_scan_launch_without_navigation(name):
    bundle = load_scenario_bundle(name, ROOT / "config")
    profile = load_navigation_profile(bundle["robots"][0]["instance"]["scan_profile"], ROOT / "config")
    with patch("isaac_composer.scan_launch.Node", side_effect=lambda **kwargs: kwargs):
        actions = create_scan_actions(profile)
    assert len(actions) == 3
    assert actions[0]["remappings"] == [("cloud_in", "unitree/slam_lidar/points1"), ("scan", "scan_front")]
    assert actions[1]["remappings"] == [("cloud_in", "unitree/slam_lidar/points2"), ("scan", "scan_rear")]
    assert actions[2]["parameters"][0]["output_topic"] == "scan"
    assert actions[0]["package"] == actions[1]["package"] == "pointcloud_to_laserscan_reliable"
    assert actions[0]["parameters"][0]["queue_size"] == 10
    assert actions[2]["parameters"][0]["reliable_input"]
    assert actions[2]["parameters"][0]["synchronize_inputs"]
    assert all(action["parameters"][0]["use_sim_time"] for action in actions)
    assert not bundle["navigation"]["settings"]["enabled"]


def test_stock_scan_converter_remains_available_for_other_sensor_profiles():
    from isaac_composer.scan_launch import converter_package

    assert converter_package({}) == "pointcloud_to_laserscan"
    assert converter_package({"reliable_input": False}) == "pointcloud_to_laserscan"



def test_interior_mask_excludes_exterior_and_other_floors():
    inside = [(-1, -1, 0), (1, -1, 0), (0, 1, 0)]
    outside = [(4, 4, 0), (5, 4, 0), (4, 5, 0)]
    roof = [(-1, -1, 9), (1, -1, 9), (0, 1, 9)]
    mesh = SimpleNamespace(get_area_count=lambda: 1, get_area_name=lambda index: 'walkable',
                           get_draw_triangles=lambda index: inside + outside + roof)
    scene = NavMeshScene({}, resolution_m=0.1, floor_z=0, floor_tolerance_m=0.2,
                         allowed_region_xy=[-0.5, -0.5, 0.5, 0.5])
    scene.navmesh = mesh
    assert scene._all_triangles() == [tuple(inside)]
    raster = scene.rasterize(margin_m=10)
    assert raster.bounds.to_dict() == dict(x_min=-0.5, y_min=-0.5, x_max=0.5, y_max=0.5, z_min=0, z_max=0)
    assert raster.mask.any()
    assert scene.is_allowed_world((0, 0, 0))
    assert not scene.is_allowed_world((4, 4, 0))
    assert not scene.is_allowed_world((0, 0, 9))
