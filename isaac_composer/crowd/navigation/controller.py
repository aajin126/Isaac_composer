"""Compose preference, context, yielding and official RVO2 layers."""

import math
import numpy as np
from ..config import validate_social_config
from ..avoidance.rvo2 import RVO2Solver
from ..behavior.preference import pedestrian_preferences
from ..behavior.yielding import YieldCoordinator
from ..behavior.progress import ProgressMonitor
from ..behavior.stiffness import SocialStiffness
from ..behavior.responsibility import DirectionalResponsibility


class SocialNavigator:
    def __init__(self, scene, context, people, config, seed=42):
        self.config = validate_social_config(config)
        self.context = context
        self.stiffness = SocialStiffness(people, self.config)
        self.responsibility = DirectionalResponsibility(people, self.config)
        self.recovery = YieldCoordinator(scene, context, people, self.config, seed, self.stiffness)
        self.solver = RVO2Solver(context, self.config, self.stiffness)
        self.progress = ProgressMonitor(len(people), self.config)
        self.infeasible_count = 0
        self.local_context = []
        self.last_feasible = [True] * len(people)

    @property
    def states(self):
        return self.recovery.states

    @property
    def yield_to(self):
        return self.recovery.yield_to

    @property
    def events(self):
        return self.recovery.events

    def step(self, dt, positions, velocities, routes, speeds, radii, robots=(), headings=None):
        self.context.begin_tick()
        positions = np.asarray(positions, float)
        velocities = np.asarray(velocities, float)
        cfg = self.config
        shares = self.responsibility.update(dt, positions, velocities, radii, headings)
        routes = self.recovery.routes_for_progress(positions, routes)
        preferred = pedestrian_preferences(
            positions, routes, speeds, radii, robots, cfg, self.stiffness.buffers
        )
        self.local_context = [
            self.context.local(p, positions, cfg["density_radius_m"]) for p in positions
        ]
        # Always assess the original route intent first, using current observed
        # velocities. Existing yielding must not manufacture a new stall trigger.
        output, feasible = self.solver.compute(
            dt, positions, velocities, preferred, speeds, radii, robots, responsibilities=shares
        )
        self.progress.update(
            dt, positions, preferred, speeds, routes, output, feasible, self.solver
        )
        output, feasible = self.progress.velocity, self.progress.feasible
        planned = self.recovery.update(
            dt,
            positions,
            preferred,
            routes,
            speeds,
            radii,
            robots,
            self.local_context,
            self.progress.available,
        )
        if not np.array_equal(planned, preferred):
            output, feasible = self.solver.compute(
                dt, positions, velocities, planned, speeds, radii, robots, responsibilities=shares
            )
            output, feasible, _, _ = self.progress.resolve(
                positions, planned, speeds, output, feasible, self.solver
            )
        preferred = planned
        self.last_feasible = feasible.tolist()
        for i, ok in enumerate(feasible):
            if not ok:
                self.infeasible_count += 1
                self.states[i] = "infeasible"
        return output, preferred

    def diagnostics(self):
        finite = lambda x: float(x) if math.isfinite(x) else None
        return {
            "controller": "official_rvo2",
            "social_stiffness": self.stiffness.diagnostics(),
            "directional_responsibility": self.responsibility.diagnostics(),
            "deadlock_recovery_enabled": self.config["yielding_enabled"],
            "events": self.events,
            "progress": self.progress.diagnostics(),
            "infeasible_agent_ticks": self.infeasible_count,
            "minimum_pedestrian_separation_m": finite(self.recovery.minimum_separation),
            "minimum_robot_separation_m": finite(self.recovery.minimum_robot_separation),
        }
