import math
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import yaml

from isaac_composer.occupancy_region import OccupancyRegion
from isaac_composer.navmesh import NavMeshScene
from isaac_composer.runtime import SimulationRunner


def map_fixture(tmp_path, origin=(0, 0, 0), clearance=0):
    pixels = np.full((8, 8), 255, dtype=np.uint8)
    pixels[0, :] = pixels[-1, :] = pixels[:, 0] = pixels[:, -1] = 205
    pixels[3, 4] = 0
    Image.fromarray(pixels).save(tmp_path / 'map.png')
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump({'image': 'map.png', 'resolution': 1,
                                   'origin': list(origin), 'free_thresh': 0.196,
                                   'occupied_thresh': 0.65}))
    return OccupancyRegion(path, clearance)


def test_map_coordinates_unknown_obstacles_and_segment_crossing(tmp_path):
    region = map_fixture(tmp_path)
    assert region.allows_xy(2.5, 4.5)
    assert not region.allows_xy(4.5, 4.5)
    assert not region.allows_xy(0.5, 4.5)
    assert not region.allows_xy(-1, 4.5)
    assert not region.allows_segment((2.5, 4.5, 0), (6.5, 4.5, 0))
    assert region.allows_segment((2.5, 2.5, 0), (6.5, 2.5, 0))


def test_rotated_map_origin_and_clearance(tmp_path):
    region = map_fixture(tmp_path, origin=(10, -10, math.pi / 2), clearance=1)
    assert region.allows_xy(7.5, -7.5)  # local (2.5, 2.5)
    assert not region.allows_xy(8.5, -8.5)  # local (1.5, 1.5), too close to unknown


def test_map_is_intersected_with_navmesh_after_raster_closing(tmp_path):
    region = map_fixture(tmp_path)
    mesh = SimpleNamespace(get_area_count=lambda: 1, get_area_name=lambda _: 'walkable',
                           get_draw_triangles=lambda _: [(0,0,0),(8,0,0),(8,8,0),
                                                        (0,0,0),(8,8,0),(0,8,0)])
    scene = NavMeshScene({}, resolution_m=1, occupancy_region=region)
    scene.navmesh = mesh
    raster = scene.rasterize(margin_m=0)
    assert not raster.mask[3, 4]
    assert not raster.mask[0].any()
    assert raster.mask[5, 2]
    assert not scene.is_allowed_world((4.5, 4.5, 0))


def test_episode_duration_uses_world_time_instead_of_frame_count():
    fake = SimpleNamespace(simulation_app=SimpleNamespace(is_running=lambda: True),
                           world=SimpleNamespace(current_time=0), frame_count=0,
                           episode_start_sim_time=0, robot_task=None, setup=lambda: None)
    def step():
        fake.world.current_time += 0.25
        fake.frame_count += 1
    fake.step = step
    assert SimulationRunner.run(fake, duration_sec=1) == 'duration'
    assert fake.frame_count == 4
    assert fake.world.current_time == 1
    with pytest.raises(ValueError):
        SimulationRunner.run(fake, duration_sec=float('nan'))
