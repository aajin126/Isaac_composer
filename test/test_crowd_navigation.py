"""Exercise passing with a kinematic walker independently of animation assets."""
import numpy as np
import pytest

from isaac_composer.crowd_safety import CrowdNavigator
from isaac_composer.navmesh import RasterizedNavMesh, SceneBounds


class Scene:
    def __init__(self, corridor=False, bay=True):
        self.corridor, self.bay = corridor, bay
        self.paths = []

    def allows(self, point):
        x, y = point[:2]
        return (not self.corridor or abs(y) <= 0.1
                or self.bay and 2.5 <= x <= 4.5 and abs(y) <= 2)

    def project_world(self, point):
        if not self.allows(point):
            raise RuntimeError('Point is outside walkable space')
        return tuple(point)

    def shortest_path(self, start, goal):
        def clear(points):
            return all(self.allows(point) for a, b in zip(points[:-1], points[1:])
                       for point in np.linspace(a, b, 100))

        start, goal = np.asarray(start), np.asarray(goal)
        options = [[start, goal]]
        if self.corridor:
            options.append([start, np.array([start[0], 0, 0]), np.array([goal[0], 0, 0]), goal])
        for path in options:
            if clear(path):
                self.paths.append(path)
                return [tuple(point) for point in path]
        raise RuntimeError('No walkable path')


def simulate(scene, routes, speed=1.0, duration=24, repeat=False):
    navigator = CrowdNavigator(scene)
    positions = np.asarray([route[0] for route in routes], dtype=float)
    routes = [np.asarray(route, dtype=float) for route in routes]
    targets = [None] * len(routes)
    indices = [0] * len(routes)
    minimum = np.inf
    dt = 0.04
    for _ in range(round(duration / dt)):
        if repeat:
            for i, route in enumerate(routes):
                if np.linalg.norm(positions[i, :2] - route[-1, :2]) <= 0.3:
                    routes[i] = route[::-1].copy()
        desired = [speed if np.linalg.norm(p[:2] - route[-1, :2]) > 0.3 else 0
                   for p, route in zip(positions, routes)]
        commands, paths = navigator.step(dt, positions, routes, desired, [.3] * len(routes))
        for i, (command, path) in enumerate(zip(commands, paths)):
            if command <= 1e-6:
                continue
            if targets[i] is not path:
                targets[i] = path
                indices[i] = 0
            while indices[i] < len(path) - 1 and np.linalg.norm(positions[i, :2] - path[indices[i], :2]) < 0.06:
                indices[i] += 1
            delta = path[indices[i]] - positions[i]
            distance = np.linalg.norm(delta[:2])
            if distance > 1e-6:
                positions[i] += delta * min(1, command * dt / distance)
        for i in range(len(positions)):
            for j in range(i):
                minimum = min(minimum, np.linalg.norm(positions[i, :2] - positions[j, :2]))
        assert all(scene.allows(p) for p in positions), (positions, navigator.states, navigator.events)
    return positions, minimum, navigator


def test_head_on_encounter_uses_a_bypass_and_both_reach_their_goals():
    routes = [[[-3, 0, 0], [3, 0, 0]], [[3, 0, 0], [-3, 0, 0]]]
    positions, minimum, navigator = simulate(Scene(), routes)
    assert minimum >= 0.79
    assert all(np.linalg.norm(position - route[-1]) <= .3 for position, route in zip(positions, routes))
    assert any(event['action'] == 'bypass' for event in navigator.events)
    assert any(event['action'] == 'resume' for event in navigator.events)


def test_crossing_encounter_gives_one_person_priority_without_stopping_both():
    routes = [[[-3, 0, 0], [3, 0, 0]], [[0, -3, 0], [0, 3, 0]]]
    positions, minimum, navigator = simulate(Scene(), routes)
    assert minimum >= .79
    assert all(np.linalg.norm(position - route[-1]) <= .3 for position, route in zip(positions, routes))
    assert any(event['action'] == 'yield' for event in navigator.events)


def test_stationary_person_is_bypassed():
    routes = [[[-3, 0, 0], [3, 0, 0]], [[0, 0, 0], [0, 0, 0]]]
    positions, minimum, navigator = simulate(Scene(), routes)
    assert np.linalg.norm(positions[0] - routes[0][-1]) <= .3
    np.testing.assert_equal(positions[1], [0, 0, 0])
    assert minimum >= .79


def test_prediction_stops_at_goal_instead_of_colliding_with_person_beyond_it():
    routes = [[[-3, 0, 0], [0, 0, 0]], [[1, 0, 0], [1, 0, 0]]]
    positions, minimum, navigator = simulate(Scene(), routes)
    assert np.linalg.norm(positions[0] - routes[0][-1]) <= .3
    assert minimum >= .79


def test_grid_fallback_can_replan_a_curving_temporary_route():
    scene = Scene()
    scene.rasterize = lambda margin_m: RasterizedNavMesh(
        np.ones((80, 80), dtype=bool), np.ones((80, 80)), SceneBounds(-8, 8, -8, 8, 0, 0), .2)
    routes = [[[-1, 2, 0], [-1, -.2, 0], [3, -.2, 0], [3, 3, 0]],
              [[-1, 0, 0], [-1, 0, 0]]]
    positions, minimum, navigator = simulate(scene, routes)
    assert navigator.grid is not None
    assert np.linalg.norm(positions[0] - routes[0][-1]) <= .3
    assert minimum >= .79


def test_three_way_encounter_does_not_block_an_existing_passing_pair():
    routes = [[[-3, 0, 0], [3, 0, 0]], [[0, -3, 0], [0, 3, 0]],
              [[0, 3, 0], [0, -3, 0]]]
    positions, minimum, navigator = simulate(Scene(), routes, duration=40)
    assert minimum >= .79
    assert all(np.linalg.norm(position - route[-1]) <= .3 for position, route in zip(positions, routes))


def test_five_people_can_back_out_and_pass_a_dense_waiting_group():
    angles = np.arange(5) * 2 * np.pi / 5
    starts = np.c_[4 * np.cos(angles), 4 * np.sin(angles), np.zeros(5)]
    routes = [[point.tolist(), (-point).tolist()] for point in starts]
    positions, minimum, navigator = simulate(Scene(), routes, duration=60)
    assert minimum >= .79
    assert all(np.linalg.norm(position - route[-1]) <= .3 for position, route in zip(positions, routes))


def test_narrow_corridor_uses_waiting_bay_then_resumes():
    routes = [[[-3, 0, 0], [6, 0, 0]], [[1, 0, 0], [-6, 0, 0]]]
    positions, minimum, navigator = simulate(Scene(corridor=True), routes, duration=40)
    assert minimum >= .79
    assert all(np.linalg.norm(position - route[-1]) <= .3 for position, route in zip(positions, routes))
    assert any(event['action'] == 'step_aside' for event in navigator.events)
    assert any(event['action'] == 'waiting_place_reached' for event in navigator.events)


def test_repeating_winner_releases_person_waiting_close_to_its_goal():
    routes = [[[-3, 0, 0], [4, 0, 0]], [[4, 0, 0], [-3, 0, 0]]]
    _, minimum, navigator = simulate(Scene(corridor=True), routes, duration=15, repeat=True)
    assert minimum >= .79
    assert any(event['action'] == 'waiting_place_reached' for event in navigator.events)
    assert any(event['action'] == 'resume' for event in navigator.events)
    assert navigator.states == ['walking', 'walking']


def test_unpassable_corridor_does_not_send_people_into_walls_or_each_other():
    routes = [[[-3, 0, 0], [3, 0, 0]], [[3, 0, 0], [-3, 0, 0]]]
    positions, minimum, navigator = simulate(Scene(corridor=True, bay=False), routes)
    assert minimum >= .79
    assert all(abs(p[1]) <= .1 for p in positions)
    assert not navigator.encounters


def test_priority_is_stable_while_passing():
    navigator = CrowdNavigator(Scene())
    routes = [np.array([[-1, 0, 0], [4, 0, 0]]), np.array([[1, 0, 0], [-4, 0, 0]])]
    positions = np.array([[-1, 0, 0], [1, 0, 0]])
    for _ in range(10):
        navigator.step(.04, positions, routes, [1, 1], [.3, .3])
    assert len(navigator.encounters) == 1
    assert navigator.encounters[0]['winner'] == 0
    assert len(navigator.events) == 1
