import copy
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from isaac_composer.config import ConfigError, load_scenario_bundle, validate_pedestrian_config
from isaac_composer.navmesh import RasterizedNavMesh, SceneBounds
from isaac_composer.pedestrian_routes import PedestrianRoutePlanner, prepare_pedestrian_routes


class GridScene:
    """Walkable grid with a wall and a doorway, used as a path-query test double."""
    def __init__(self):
        self.mask = np.ones((20, 20), dtype=bool)
        self.mask[:, 10] = False
        self.mask[2, 10] = True
        self.raster = RasterizedNavMesh(self.mask, np.ones_like(self.mask), SceneBounds(0, 20, 0, 20, 0, 0), 1)

    def bake(self):
        pass

    def rasterize(self, margin_m=0):
        return self.raster

    def cell(self, point):
        return 19 - int(point[1]), int(point[0])

    def project_world(self, point):
        r, c = self.cell(point)
        if not (0 <= r < 20 and 0 <= c < 20 and self.mask[r, c]):
            raise RuntimeError('Point is not walkable')
        return tuple(point)

    def shortest_path(self, start, goal):
        source, target = self.cell(start), self.cell(goal)
        previous = {source: None}
        queue = deque([source])
        while queue:
            current = queue.popleft()
            if current == target:
                break
            r, c = current
            for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
                if 0 <= nr < 20 and 0 <= nc < 20 and self.mask[nr, nc] and (nr, nc) not in previous:
                    previous[nr, nc] = current
                    queue.append((nr, nc))
        if target not in previous:
            raise RuntimeError('No connected route')
        cells = []
        current = target
        while current is not None:
            cells.append(current)
            current = previous[current]
        return [self.raster.pixel_to_world(r, c) for r, c in reversed(cells)]


def test_fixed_endpoints_use_obstacle_avoiding_path_and_requested_speed():
    scene = GridScene()
    config = {'mode': 'fixed', 'start': [4.5, 5.5, 0], 'goal': [15.5, 5.5, 0], 'speed_mps': 0.8}
    person, = PedestrianRoutePlanner(scene, config).plan()
    assert person['pose']['position'] == config['start']
    assert person['waypoints'][-1] == config['goal']
    assert person['walk_speed'] == 0.8
    # The route must reach the doorway above the wall, rather than move straight through it.
    assert [10.5, 17.5, 0] in person['waypoints']
    assert all(scene.mask[scene.cell(p)] for p in person['waypoints'])
    assert not person['repeat']


def test_random_people_are_reproducible_separated_and_cover_both_halves():
    config = {'mode': 'random', 'count': 8, 'speed_range_mps': [0.6, 1.4], 'repeat': True}
    first = PedestrianRoutePlanner(GridScene(), config, seed=43, robot_positions=[[1.5, 1.5, 0]]).plan()
    assert first == PedestrianRoutePlanner(GridScene(), config, seed=43, robot_positions=[[1.5, 1.5, 0]]).plan()
    assert first != PedestrianRoutePlanner(GridScene(), config, seed=44).plan()
    assert len(first) == 8
    starts = [p['pose']['position'] for p in first]
    assert min(p[1] for p in starts) < 10 < max(p[1] for p in starts)
    assert len(set(p['walk_speed'] for p in first)) == 8
    scene = GridScene()
    for i, person in enumerate(first):
        assert 0.6 <= person['walk_speed'] <= 1.4
        assert person['repeat']
        assert np.linalg.norm(np.array(starts[i])[:2] - np.array(person['waypoints'][-1])[:2]) >= 3
        assert np.linalg.norm(np.array(starts[i])[:2] - [1.5, 1.5]) >= 1.5
        assert all(np.linalg.norm(np.array(starts[i])[:2] - np.array(p)[:2]) >= 1 for p in starts[:i])
        assert all(scene.mask[scene.cell(p)] for p in person['waypoints'])


def test_random_disconnected_routes_are_rejected_with_bounded_retries():
    scene = GridScene()
    scene.shortest_path = lambda *args: (_ for _ in ()).throw(RuntimeError('No connected route'))
    config = {'mode': 'random', 'count': 1, 'max_attempts_per_person': 10}
    with pytest.raises(RuntimeError, match='after 10 attempts'):
        PedestrianRoutePlanner(scene, config).plan()


def test_fixed_invalid_point_fails_instead_of_moving_it_elsewhere():
    scene = GridScene()
    scene.project_world = lambda point: (point[0] + 1, point[1], 0)
    with pytest.raises(RuntimeError, match='too far'):
        PedestrianRoutePlanner(scene, {'mode': 'fixed', 'start': [1, 1, 0], 'goal': [5, 5, 0]}).plan()


def test_preparation_keeps_robot_pose_and_saves_actual_routes(tmp_path):
    bundle = load_scenario_bundle('warehouse_a2_fixed')
    robots = copy.deepcopy(bundle['robots'])
    bundle['pedestrians']['generation'] = {
        'mode': 'fixed', 'start': [4.5, 5.5, 0], 'goal': [15.5, 5.5, 0],
        'output_dir': str(tmp_path),
    }
    with patch('isaac_composer.pedestrian_routes.NavMeshScene', return_value=GridScene()):
        assert prepare_pedestrian_routes(bundle, Path('.')) is not None
    assert bundle['robots'] == robots
    assert len(bundle['pedestrians']['people']) == 1
    assert (tmp_path / 'pedestrian_routes.json').is_file()
    assert (tmp_path / '01_pedestrian_routes.png').is_file()
    assert not (tmp_path / '01_topology_overlay.png').exists()


@pytest.mark.parametrize('change', [
    {'mode': 'unknown'}, {'count': 0}, {'count': 1.5}, {'speed_range_mps': [1.4, 0.6]},
    {'speed_range_mps': [0, 1]}, {'radius': float('nan')}, {'characters': []},
])
def test_invalid_generation_parameters_fail_before_simulator_start(change):
    config = load_scenario_bundle('warehouse_a2_random')['pedestrians']
    config['generation'].update(change)
    with pytest.raises(ConfigError):
        validate_pedestrian_config(config)


@pytest.mark.parametrize('scenario', ['warehouse_a2_fixed', 'warehouse_a2_random', 'hospital_a2_random'])
def test_collection_presets_need_no_cosmos_or_topology(scenario):
    bundle = load_scenario_bundle(scenario)
    assert 'cosmos_generation' not in bundle['pedestrians']
    assert bundle['pedestrians']['generation']['mode'] in ('fixed', 'random')
    assert not bundle['navigation']['settings']['enabled']


def test_target_population_uses_walkable_area_and_maximum_cap():
    scene = GridScene()
    config = {'mode': 'random', 'count': 99, 'target_density_per_m2': .01, 'max_count': 10}
    # 381 square meters are actually walkable: round(381 * .01) = 4.
    assert len(PedestrianRoutePlanner(scene, config, 43).plan()) == 4
    config.update(target_density_per_m2=1, max_count=6)
    assert len(PedestrianRoutePlanner(scene, config, 43).plan()) == 6


def test_pedestrian_bake_and_query_radius_are_not_enlarged_by_robot_task(tmp_path):
    class RadiusScene(GridScene):
        agent_radius = 0.45
        config = {'agent_radius': 0.45}
        def bake(self):
            self.baked_radius = self.agent_radius
    scene = RadiusScene()
    bundle = load_scenario_bundle('warehouse_a2_random')
    bundle['pedestrians']['generation'].update(count=1, output_dir=str(tmp_path))
    bundle['pedestrians']['generation']['scene'] = {}
    bundle['robot_task']['output_dir'] = str(tmp_path)
    with patch('isaac_composer.pedestrian_routes.NavMeshScene',return_value=scene):
        prepare_pedestrian_routes(bundle,Path('.'))
    assert scene.baked_radius == .3
    assert scene.routing_radius_m == pytest.approx(.35)
    assert scene.config['agent_max_radius'] >= .55
    assert bundle['robot_task']['robot_radius_m'] == .55


def test_episode_mu_is_reproducible_and_does_not_change_route_randomization(tmp_path):
    results = []
    for name, mu_range in [('first', [0, 1]), ('second', [0, 1]), ('constant', [.8, .8])]:
        bundle = load_scenario_bundle('warehouse_a2_random')
        bundle['robot_task']['enabled'] = False
        bundle['pedestrians']['generation'].update(count=3, output_dir=str(tmp_path / name))
        bundle['pedestrians']['generation']['scene'] = {}
        bundle['pedestrians']['social_navigation']['mu_range'] = mu_range
        with patch('isaac_composer.pedestrian_routes.NavMeshScene', return_value=GridScene()):
            prepare_pedestrian_routes(bundle, Path('.'))
        results.append(bundle['pedestrians']['people'])
    assert results[0] == results[1]
    assert all(p['mu'] == .8 for p in results[2])
    for original, constant in zip(results[0], results[2]):
        assert {k: v for k, v in original.items() if k != 'mu'} == {k: v for k, v in constant.items() if k != 'mu'}
