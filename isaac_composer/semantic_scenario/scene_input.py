"""Actual world RGB plus existing grid addresses; no USD object extraction."""

from __future__ import annotations

import json
from pathlib import Path

from .schema import PlanError, english_json, save_json


def load_context(
    directory, user_request="Generate plausible activities appropriate to this environment."
):
    directory = Path(directory).resolve()
    status_path = directory / "export_status.json"
    if status_path.is_file() and json.loads(status_path.read_text()).get("status") != "complete":
        raise PlanError("Scene export did not complete; regenerate the scene context")
    grid = json.loads((directory / "grid_cells.json").read_text())
    image = directory / "grid_scene.png"
    if not image.is_file():
        raise PlanError(
            "Missing actual world image grid_scene.png; export the grid with --scene-image"
        )
    context = {
        "environment": grid.get("environment", "unknown_environment"),
        "user_request": user_request,
        "grid": grid,
        "valid_cells": [c["id"] for c in grid["cells"]],
        "scene_image": str(image),
    }
    if (directory / "world_scene.png").is_file():
        context["world_image"] = str(directory / "world_scene.png")
    english_json(context)
    return context


def _pixel(grid, point, width, height):
    x0, y0, x1, y1 = grid["bounds_xy"]
    return ((point[0] - x0) / (x1 - x0) * width, (y1 - point[1]) / (y1 - y0) * height)


def render_route_preview(context, plan, output_path):
    """Draw intended subgoal connections, explicitly not obstacle-avoiding paths."""
    from PIL import Image, ImageDraw

    image = Image.open(context["scene_image"]).convert("RGB")
    draw = ImageDraw.Draw(image)
    cells = {c["id"]: c["target_world"] for c in context["grid"]["cells"]}
    colors = ["#ff3030", "#32ff48", "#ffdc18", "#ca59ff", "#ff8e26", "#13e3ff"]
    for queue in plan.get("queues", []):
        points = [_pixel(context["grid"], cells[cid], *image.size) for cid in queue["spots"]]
        if len(points) > 1:
            draw.line(points, fill="white", width=5)
        x, y = points[0]
        draw.text(
            (x + 5, y - 15),
            queue["queue_id"] + " front",
            fill="white",
            stroke_width=1,
            stroke_fill="black",
        )
    for scene in plan["scenes"]:
        for actor in scene["actor_plans"]:
            color = colors[actor["actor_id"] % len(colors)]
            ids = (
                [actor["start_cell"]]
                + [s["cell_id"] for s in actor["subgoals"]]
                + [actor["goal_cell"]]
            )
            points = [_pixel(context["grid"], cells[c], *image.size) for c in ids]
            draw.line(points, fill=color, width=4)
            for index, (x, y) in enumerate(points):
                draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill=color)
                draw.text(
                    (x + 5, y + 4),
                    f"P{actor['actor_id']}:{index}",
                    fill=color,
                    stroke_width=1,
                    stroke_fill="black",
                )
    draw.rectangle((0, 0, image.width, 20), fill="black")
    draw.text(
        (5, 4),
        "Intended subgoal order; lines are not NavMesh paths or executed trajectories",
        fill="white",
    )
    image.save(output_path)
    return str(output_path)


def render_robot_context(context, output_path):
    """Show the robot's static reference path and interaction band, with cell IDs."""
    from PIL import Image, ImageDraw
    from .spatial import placement_from_context

    image = Image.open(context["scene_image"]).convert("RGBA")
    overlay = Image.new("RGBA", image.size)
    draw = ImageDraw.Draw(overlay)
    robot = context["spatial_guidance"]["robot"]
    placement = placement_from_context(context)
    start = _pixel(context["grid"], robot["start_world"], *image.size)
    goal = _pixel(context["grid"], robot["goal_world"], *image.size)
    x0, _, x1, _ = context["grid"]["bounds_xy"]
    width = max(1, round(placement.width * 2 * image.width / (x1 - x0)))
    path = [_pixel(context["grid"], p, *image.size) for p in placement.robot_path]
    draw.line(path, fill=(255, 210, 0, 35), width=width, joint="curve")
    radius = width / 2
    for x, y in path:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=(255, 210, 0, 35))
    draw.line(path, fill=(255, 210, 0, 230), width=3)
    for point, color, label in ((start, "lime", "ROBOT START"), (goal, "red", "ROBOT GOAL")):
        x, y = point
        draw.ellipse((x - 6, y - 6, x + 6, y + 6), fill=color)
        draw.text((x + 8, y + 8), label, fill="white", stroke_width=1, stroke_fill="black")
    Image.alpha_composite(image, overlay).convert("RGB").save(output_path)
    return str(output_path)


def capture_scene_image(app, scene, grid, directory):
    """Orthographic, floor-aligned RGB; camera-only clipping removes roofs."""
    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    from pxr import Gf, UsdGeom
    from PIL import Image, ImageDraw, ImageFont

    directory = Path(directory)
    stage = omni.usd.get_context().get_stage()
    x0, y0, x1, y1 = grid.bounds_xy
    scale = UsdGeom.GetStageMetersPerUnit(stage)
    if abs(scale - 1.0) > 1e-6:
        raise PlanError("Scene image export currently requires a meter-based stage")
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    z = grid.floor_z
    camera = UsdGeom.Camera.Define(stage, "/World/SemanticScenarioCamera")
    camera.CreateProjectionAttr().Set("orthographic")
    camera.CreateHorizontalApertureAttr().Set((x1 - x0) * 10)
    camera.CreateVerticalApertureAttr().Set((y1 - y0) * 10)
    height = max(30.0, x1 - x0, y1 - y0)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(height - 2.5, height + 2))
    camera.MakeMatrixXform().Set(
        Gf.Matrix4d()
        .SetLookAt(Gf.Vec3d(cx, cy, z + height), Gf.Vec3d(cx, cy, z), Gf.Vec3d(0, 1, 0))
        .GetInverse()
    )
    ppm = min(50, 1800 / max(x1 - x0, y1 - y0))
    # Preserve the exact bounds/aspect ratio for world-to-image mapping.
    resolution = (max(1, round((x1 - x0) * ppm)), max(1, round((y1 - y0) * ppm)))
    product = rep.create.render_product(str(camera.GetPath()), resolution)
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach([product])
    try:
        pixels = None
        for _ in range(3):
            rep.orchestrator.step(rt_subframes=8, delta_time=0.0)
            candidate = annotator.get_data()
            if isinstance(candidate, np.ndarray) and candidate.ndim == 3 and candidate.size:
                pixels = candidate[:, :, :3].copy()
        if pixels is None:
            raise PlanError("RGB annotator returned no scene image")
        image = Image.fromarray(pixels)
        image.save(directory / "world_scene.png")
        draw = ImageDraw.Draw(image, "RGBA")
        try:
            font = ImageFont.truetype(
                "DejaVuSans.ttf", max(12, round(grid.cell_size_m * ppm * 0.15))
            )
        except OSError:
            font = ImageFont.load_default()
        grid_data = grid.to_dict()
        for cell in grid.cells:
            a, b, c, d = cell.bounds_xy
            draw.rectangle(
                [_pixel(grid_data, (a, d), *image.size), _pixel(grid_data, (c, b), *image.size)],
                outline=(255, 255, 255, 120),
                width=1,
            )
            x, y = _pixel(grid_data, cell.target_world, *image.size)
            draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=(255, 70, 40, 255))
            draw.text(
                (x + 3, y + 2),
                cell.id,
                font=font,
                fill="white",
                stroke_width=1,
                stroke_fill="black",
            )
        image.save(directory / "grid_scene.png")
        save_json(
            directory / "scene_image_metadata.json",
            {
                "source": "Isaac Sim RGB annotator",
                "projection": "orthographic",
                "bounds_xy": grid.bounds_xy,
                "resolution": list(image.size),
                "camera_path": str(camera.GetPath()),
                "roof_cutaway_height_m": 2.5,
            },
        )
    finally:
        annotator.detach([product.path])
        product.destroy()
