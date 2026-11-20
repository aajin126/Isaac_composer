"""Addressable subgoal cells from native NavMesh geometry and point queries.

This does not compute paths or use the fine raster used by crowd diagnostics.
Only cells whose geometric centers pass the native query are retained.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path

import numpy as np


@dataclass
class GridCell:
    id: str
    row: int
    column: int
    bounds_xy: tuple[float, float, float, float]
    target_world: tuple[float, float, float]
    polygons: list = field(default_factory=list, repr=False)

    def to_dict(self):
        result = asdict(self)
        del result["polygons"]
        return result


@dataclass
class SceneGrid:
    cell_size_m: float
    bounds_xy: tuple[float, float, float, float]
    rows: int
    columns: int
    cells: list[GridCell]
    agent_radius_m: float
    floor_z: float

    def to_dict(self):
        return {
            "schema_version": 1,
            "frame_id": "world",
            "units": "m",
            "cell_size_m": self.cell_size_m,
            "bounds_xy": self.bounds_xy,
            "rows": self.rows,
            "columns": self.columns,
            "id_convention": "columns A,B,...,AA from left; rows 1,2,... from top (+Y)",
            "agent_query_radius_m": self.agent_radius_m,
            "floor_z": self.floor_z,
            "retained_cell_count": len(self.cells),
            "removed_cell_count": self.rows * self.columns - len(self.cells),
            "cell_definition": "geometric center is a valid pedestrian destination",
            "center_projection_tolerance_m": 0.001,
            "connectivity": "not inferred from cell adjacency; query native paths for each route",
            "cells": [cell.to_dict() for cell in self.cells],
        }


def _column_name(index):
    name = ""
    while index >= 0:
        index, digit = divmod(index, 26)
        name = chr(65 + digit) + name
        index -= 1
    return name


def _clip_polygon(points, bounds):
    """Intersect one native triangle with a cell rectangle, preserving floor Z."""
    polygon = [np.asarray(p, dtype=float) for p in points]
    x0, y0, x1, y1 = bounds
    for axis, edge, direction in ((0, x0, 1), (0, x1, -1), (1, y0, 1), (1, y1, -1)):
        source, polygon = polygon, []
        if not source:
            break
        previous = source[-1]
        before = direction * (previous[axis] - edge) >= 0
        for current in source:
            inside = direction * (current[axis] - edge) >= 0
            if inside != before:
                alpha = (edge - previous[axis]) / (current[axis] - previous[axis])
                polygon.append(previous + alpha * (current - previous))
            if inside:
                polygon.append(current)
            previous, before = current, inside
    return polygon


def _area(polygon):
    if len(polygon) < 3:
        return 0.0
    xy = np.asarray(polygon)[:, :2]
    return (
        abs(float(np.sum(xy[:, 0] * np.roll(xy[:, 1], -1) - xy[:, 1] * np.roll(xy[:, 0], -1)))) / 2
    )


def build_scene_grid(scene, cell_size_m=2.0):
    """Keep cells only when their geometric centers are valid subgoal points."""
    size = float(cell_size_m)
    if not math.isfinite(size) or size <= 0:
        raise ValueError("cell_size_m must be positive and finite")
    bounds = scene.bounds(margin_m=0)
    if not all(math.isfinite(v) for v in (bounds.x_min, bounds.y_min, bounds.x_max, bounds.y_max)):
        raise ValueError("NavMesh bounds must be finite")
    x0, y0 = math.floor(bounds.x_min / size) * size, math.floor(bounds.y_min / size) * size
    x1, y1 = math.ceil(bounds.x_max / size) * size, math.ceil(bounds.y_max / size) * size
    columns, rows = round((x1 - x0) / size), round((y1 - y0) / size)
    if min(columns, rows) <= 0:
        raise ValueError("NavMesh has no floor area")
    fragments = {}
    for triangle in scene.walkable_triangles():
        tri = np.asarray(triangle, dtype=float)
        c0 = max(0, math.floor((tri[:, 0].min() - x0) / size))
        c1 = min(columns - 1, math.floor((tri[:, 0].max() - x0) / size))
        r0 = max(0, math.floor((y1 - tri[:, 1].max()) / size))
        r1 = min(rows - 1, math.floor((y1 - tri[:, 1].min()) / size))
        for row in range(r0, r1 + 1):
            for col in range(c0, c1 + 1):
                box = (
                    x0 + col * size,
                    y1 - (row + 1) * size,
                    x0 + (col + 1) * size,
                    y1 - row * size,
                )
                polygon = _clip_polygon(tri, box)
                if _area(polygon) > 1e-10:
                    fragments.setdefault((row, col), []).append(polygon)
    cells = []
    for (row, col), polygons in sorted(fragments.items()):
        box = (x0 + col * size, y1 - (row + 1) * size, x0 + (col + 1) * size, y1 - row * size)
        center = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2, bounds.z_min])
        try:
            point = tuple(map(float, scene.project_world(center)))
        except RuntimeError:
            continue
        # query_closest_point can snap a blocked center to a free point even
        # inside this same cell. Such a cell must be rejected, not relocated.
        if (
            len(point) != 3
            or not all(math.isfinite(v) for v in point)
            or np.linalg.norm(np.asarray(point[:2]) - center[:2]) > 0.001
        ):
            continue
        target = (float(center[0]), float(center[1]), point[2])
        if not scene.is_allowed_world(target):
            continue
        cells.append(GridCell(f"{_column_name(col)}{row + 1}", row, col, box, target, polygons))
    if not cells:
        raise RuntimeError("No cells contain a valid pedestrian destination")
    return SceneGrid(
        size,
        (x0, y0, x1, y1),
        rows,
        columns,
        cells,
        float(getattr(scene, "routing_radius_m", scene.agent_radius)),
        float(scene.floor_z if scene.floor_z is not None else bounds.z_min),
    )


def save_scene_grid(grid, output_dir, *, scene=None, pixels_per_meter=50):
    """Save address data and an overhead NavMesh diagram, without USD geometry."""
    from PIL import Image, ImageDraw, ImageFont

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / "grid_cells.json").write_text(json.dumps(grid.to_dict(), indent=2, allow_nan=False))
    x0, y0, x1, y1 = grid.bounds_xy
    width = max(1, math.ceil((x1 - x0) * pixels_per_meter))
    height = max(1, math.ceil((y1 - y0) * pixels_per_meter))
    image = Image.new("RGB", (width, height), "#17202d")
    draw = ImageDraw.Draw(image)

    def pixel(point):
        return ((point[0] - x0) / (x1 - x0) * width, (y1 - point[1]) / (y1 - y0) * height)

    for cell in grid.cells:
        for polygon in cell.polygons:
            draw.polygon([pixel(p) for p in polygon], fill="#8bc6d3")
    # The optional prebuilt map's unknown/blocked areas remain blocked visually.
    region = getattr(scene, "occupancy_region", None)
    if region is not None:
        xs = x0 + (np.arange(width) + 0.5) / width * (x1 - x0)
        ys = y1 - (np.arange(height) + 0.5) / height * (y1 - y0)
        allowed = region.allows_xy(*np.meshgrid(xs, ys))
        pixels = np.asarray(image).copy()
        pixels[~allowed] = [23, 32, 45]
        image = Image.fromarray(pixels)
        draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype(
            "DejaVuSans.ttf", max(12, round(grid.cell_size_m * pixels_per_meter * 0.16))
        )
    except OSError:
        font = ImageFont.load_default()
    for cell in grid.cells:
        a, b, c, d = cell.bounds_xy
        draw.rectangle([pixel((a, d)), pixel((c, b))], outline="#e9f0f4", width=1)
        x, y = pixel(cell.target_world)
        draw.ellipse((x - 3, y - 3, x + 3, y + 3), fill="#e64545")
        label_box = draw.textbbox((0, 0), cell.id, font=font)
        tw, th = label_box[2], label_box[3]
        tx = max(1, min(width - tw - 3, x + 5))
        ty = max(1, min(height - th - 3, y - th - 3))
        draw.rectangle((tx - 2, ty - 1, tx + tw + 2, ty + th + 1), fill="#17202d")
        draw.text((tx, ty), cell.id, font=font, fill="white")
    canvas = Image.new("RGB", (width, height + 65), "#17202d")
    canvas.paste(image)
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (8, height + 5),
        f"NavMesh grid | {grid.cell_size_m:g} m | kept {len(grid.cells)}/{grid.rows * grid.columns}",
        fill="white",
    )
    draw.text(
        (8, height + 23),
        "Blue: baked NavMesh floor   Red: validated target   Dark: blocked/excluded",
        fill="white",
    )
    draw.text(
        (8, height + 41),
        "+Y up, +X right | Center must be walkable; route connectivity checked separately",
        fill="white",
    )
    canvas.save(output / "grid_navmesh.png")
    return output
