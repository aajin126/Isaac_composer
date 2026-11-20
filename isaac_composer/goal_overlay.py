"""Show a goal in the viewport without adding geometry to sensor observations."""

from __future__ import annotations

import math


def annotation_projection(matrix, meters_per_unit=1.0):
    """Preserve screen placement but give annotations their own near plane."""
    values = [float(matrix[row][col]) for row in range(4) for col in range(4)]
    near, far = 0.1 / meters_per_unit, 1000.0 / meters_per_unit
    if abs(values[11]) > 0.5:  # Perspective camera.
        values[10] = -(far + near) / (far - near)
        values[14] = -2 * far * near / (far - near)
    else:  # Orthographic camera.
        values[10] = -2 / (far - near)
        values[14] = -(far + near) / (far - near)
    return values


class GoalOverlay:
    def __init__(self, goal_world, radius_m=0.5, route=None):
        import omni.ui as ui
        from omni.ui import scene as sc
        from omni.kit.viewport.utility import get_active_viewport_window
        from pxr import UsdGeom
        import omni.usd

        self.goal = list(goal_world)
        self.route = route
        self.route_signature = None
        self.route_container = None
        self.window = get_active_viewport_window()
        self.scene_view = None
        self.label = None
        self.last_projection = None
        if self.window is None:
            return
        self.frame = self.window.get_frame("robot_task_goal")
        scale = UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage())
        self.meters_per_unit = scale
        x, y, z = [float(v) / scale for v in goal_world]
        color = ui.color(0.25, 1.0, 0.35, 1.0)
        with self.frame:
            with ui.ZStack():
                self.scene_view = sc.SceneView()
                with self.scene_view.scene:
                    self.route_container = sc.Transform()
                    points = [
                        [
                            x + radius_m / scale * math.cos(i * math.tau / 32),
                            y + radius_m / scale * math.sin(i * math.tau / 32),
                            z + 0.04 / scale,
                        ]
                        for i in range(33)
                    ]
                    for a, b in zip(points[:-1], points[1:]):
                        sc.Line(a, b, color=color, thickness=3)
                    sc.Line(
                        [x, y, z + 0.04 / scale], [x, y, z + 1.3 / scale], color=color, thickness=3
                    )
                    with sc.Transform(
                        transform=sc.Matrix44.get_translation_matrix(x, y, z + 1.5 / scale)
                    ):
                        with sc.Transform(
                            look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.SCREEN
                        ):
                            sc.Label("GOAL", color=color, size=22, alignment=ui.Alignment.CENTER)
                with ui.VStack(spacing=4):
                    # Leave room for the viewport's camera/rendering toolbar.
                    ui.Spacer(height=32)
                    with ui.HStack(height=58):
                        ui.Spacer(width=12)
                        with ui.ZStack(width=380):
                            ui.Rectangle(
                                style={
                                    "background_color": ui.color(0.04, 0.07, 0.04, 0.82),
                                    "border_radius": 5,
                                }
                            )
                            with ui.VStack(spacing=3):
                                ui.Label(
                                    "  DRIVE TO THE GOAL",
                                    height=22,
                                    style={"color": color, "font_size": 17},
                                )
                                self.label = ui.Label(
                                    "  Preparing goal...",
                                    height=23,
                                    style={"color": ui.color.white, "font_size": 17},
                                )
                        ui.Spacer()
                    ui.Spacer()
        self.window.viewport_api.add_scene_view(self.scene_view)

    def _draw_route(self):
        from omni.ui import scene as sc
        import omni.ui as ui

        points = self.route.points if self.route is not None else []
        stale = bool(self.route and self.route.error)
        signature = (stale, tuple(tuple(p) for p in points))
        if signature == self.route_signature:
            return
        self.route_container.clear()
        scale = self.meters_per_unit
        with self.route_container:
            for a, b in zip(points, points[1:]):
                sc.Line(
                    [a[0] / scale, a[1] / scale, (a[2] + 0.06) / scale],
                    [b[0] / scale, b[1] / scale, (b[2] + 0.06) / scale],
                    color=(
                        ui.color(1.0, 0.7, 0.15, 0.8) if stale else ui.color(0.25, 1.0, 0.35, 0.8)
                    ),
                    thickness=3,
                )
        self.route_signature = signature

    def update(self, robot_pose, reached=False, simulation_time=0.0):
        if self.label is None:
            return
        # Screen-only static route: no USD geometry or sensor changes.
        if self.route is not None:
            self.route.update(robot_pose, simulation_time)
        self._draw_route()
        # Top-down rendering clips away the roof with a large near plane.
        # SceneView would also clip the goal; only its depth projection changes.
        projection = self.window.viewport_api.projection
        signature = tuple(float(projection[r][c]) for r in range(4) for c in range(4))
        if signature != self.last_projection:
            self.scene_view.model.set_floats(
                "projection", annotation_projection(projection, self.meters_per_unit)
            )
            self.last_projection = signature
        points = self.route.points if self.route is not None else []
        distance, target = (
            self.route.guidance(robot_pose)
            if points
            else (math.dist(self.goal[:2], robot_pose[:2]), self.goal)
        )
        dx, dy = target[0] - robot_pose[0], target[1] - robot_pose[1]
        bearing = math.atan2(dy, dx) - robot_pose[2]
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        turn = "LEFT" if bearing >= 0 else "RIGHT"
        text = f"  PATH {distance:.1f} m | NEXT {turn} {abs(math.degrees(bearing)):.0f} deg"
        if self.route is None or not points:
            text = "  NO ROUTE TO GOAL"
        elif self.route.error:
            text = "  LAST ROUTE | UPDATE PENDING"
        if reached:
            text = "  GOAL REACHED"
        if text != self.label.text:
            self.label.text = text

    def destroy(self):
        if self.scene_view is not None:
            self.window.viewport_api.remove_scene_view(self.scene_view)
            self.frame.clear()
            self.scene_view = None
            self.label = None
            self.route_container = None
        self.window = None
