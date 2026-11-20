"""Subgoal cell membership, native query authority and stable addresses."""

import json
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

from isaac_composer.scene_grid import build_scene_grid, save_scene_grid


class Rectangles:
    """Native-query test double with disconnected, already-eroded floor patches."""

    agent_radius = 0.3
    routing_radius_m = 0.35
    floor_z = 0.0
    occupancy_region = None

    def __init__(self, rectangles):
        self.rectangles = rectangles

    def bounds(self, margin_m=0):
        return SimpleNamespace(
            x_min=min(r[0] for r in self.rectangles),
            y_min=min(r[1] for r in self.rectangles),
            x_max=max(r[2] for r in self.rectangles),
            y_max=max(r[3] for r in self.rectangles),
            z_min=0,
        )

    def walkable_triangles(self):
        for x0, y0, x1, y1 in self.rectangles:
            yield [(x0, y0, 0), (x1, y0, 0), (x1, y1, 0)]
            yield [(x0, y0, 0), (x1, y1, 0), (x0, y1, 0)]

    def project_world(self, point):
        points = [
            np.array([np.clip(point[0], a, c), np.clip(point[1], b, d), 0])
            for a, b, c, d in self.rectangles
        ]
        return min(points, key=lambda p: np.linalg.norm(p - point))

    def is_allowed_world(self, point):
        return True

    def rasterize(self, **kwargs):
        raise AssertionError("Subgoal grids must not require a fine raster")


def test_disconnected_floor_keeps_only_occupied_cells_and_does_not_renumber():
    scene = Rectangles([(0.2, 0.2, 1.8, 1.8), (4.2, 0.2, 5.8, 1.8)])
    grid = build_scene_grid(scene, 2)
    assert [c.id for c in grid.cells] == ["A1", "C1"]
    assert grid.to_dict()["removed_cell_count"] == 1
    for cell in grid.cells:
        a, b, c, d = cell.bounds_xy
        x, y, z = cell.target_world
        assert a <= x < c and b <= y < d and z == 0
        np.testing.assert_allclose(scene.project_world(cell.target_world), cell.target_world)


def test_partial_cell_with_blocked_center_is_removed():
    # A free strip exists in A1, but its center is blocked. Do not fall back to
    # another free point in that cell. C1 has a valid center and stays C1.
    grid = build_scene_grid(Rectangles([(0.1, 0.8, 0.3, 1.2), (4.2, 0.2, 5.8, 1.8)]))
    assert [cell.id for cell in grid.cells] == ["C1"]
    np.testing.assert_allclose(grid.cells[0].target_world, [5, 1, 0])


def test_projection_shift_within_same_cell_is_rejected():
    class Shifted(Rectangles):
        def project_world(self, point):
            return (point[0] + 0.05, point[1], 0) if point[0] < 2 else point

    grid = build_scene_grid(Shifted([(0.2, 0.2, 1.8, 1.8), (2.2, 0.2, 3.8, 1.8)]))
    assert [cell.id for cell in grid.cells] == ["B1"]


def test_small_native_roundoff_keeps_exact_center_as_target():
    class Roundoff(Rectangles):
        def project_world(self, point):
            return (point[0] + 1e-5, point[1] - 1e-5, 0.00001)

    grid = build_scene_grid(Roundoff([(0.2, 0.2, 1.8, 1.8)]))
    assert grid.cells[0].target_world == (1.0, 1.0, 0.00001)


def test_rejected_native_destinations_remove_cell_even_with_draw_triangles():
    class Excluded(Rectangles):
        def is_allowed_world(self, point):
            return point[0] < 2

    grid = build_scene_grid(Excluded([(0.2, 0.2, 1.8, 1.8), (2.2, 0.2, 3.8, 1.8)]))
    assert [c.id for c in grid.cells] == ["A1"]


def test_nearest_point_in_another_cell_is_not_assigned_to_blocked_cell():
    class Shifted(Rectangles):
        def project_world(self, point):
            return (3.0, 1.0, 0.0)

    grid = build_scene_grid(Shifted([(0.2, 0.2, 1.8, 1.8), (2.2, 0.2, 3.8, 1.8)]))
    assert [c.id for c in grid.cells] == ["B1"]


def test_floor_query_failures_do_not_become_valid_cells():
    class Missing(Rectangles):
        def project_world(self, point):
            raise RuntimeError("No permitted point on this floor")

    with pytest.raises(RuntimeError, match="No cells"):
        build_scene_grid(Missing([(0.2, 0.2, 1.8, 1.8)]))


def test_projection_on_another_floor_is_rejected():
    class WrongFloor(Rectangles):
        def project_world(self, point):
            return (1.0, 1.0, 3.0)

        def is_allowed_world(self, point):
            return abs(point[2] - self.floor_z) <= 0.2

    with pytest.raises(RuntimeError, match="No cells"):
        build_scene_grid(WrongFloor([(0.2, 0.2, 1.8, 1.8)]))


@pytest.mark.parametrize("size", [0, -1, float("nan"), float("inf")])
def test_cell_size_must_be_positive_and_finite(size):
    with pytest.raises(ValueError):
        build_scene_grid(Rectangles([(0.2, 0.2, 1.8, 1.8)]), size)


def test_addresses_handle_negative_world_coordinates_and_more_than_26_columns():
    grid = build_scene_grid(Rectangles([(-1.8, -1.8, -0.2, -0.2), (50.2, 0.2, 51.8, 1.8)]))
    assert [c.id for c in grid.cells] == ["AA1", "A2"]
    assert grid.bounds_xy == (-2, -2, 52, 2)


def test_export_records_only_valid_cells_and_saves_matching_image(tmp_path):
    grid = build_scene_grid(Rectangles([(0.2, 0.2, 1.8, 1.8), (4.2, 0.2, 5.8, 1.8)]))
    save_scene_grid(grid, tmp_path)
    data = json.loads((tmp_path / "grid_cells.json").read_text())
    assert [c["id"] for c in data["cells"]] == ["A1", "C1"]
    assert "polygons" not in data["cells"][0]
    with Image.open(tmp_path / "grid_navmesh.png") as image:
        assert image.size == (300, 165)
