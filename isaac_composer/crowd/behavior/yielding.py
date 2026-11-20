"""Optional deadlock recovery after persistent ORCA blockage; not normal avoidance."""

import random

import numpy as np

from ..avoidance.prediction import encounter_risk
from ..avoidance.clearance import pair_clearance
from .preference import preferred_velocity
from .stiffness import SocialStiffness
from ..navigation.paths import projected_path

NO_AGENT = -999


def yielding_probability(mu_i, mu_j):
    """Conditional probability that i yields once a yielding encounter is chosen."""
    return mu_i / (mu_i + mu_j) if mu_i + mu_j > 1e-12 else 0.5


class YieldCoordinator:
    def __init__(self, scene, context, people, config, seed, stiffness=None):
        self.scene, self.context, self.config = scene, context, config
        self.random = random.Random(seed ^ 0x51A1)
        self.stiffness = stiffness or SocialStiffness(people, config)
        self.mus = self.stiffness.mus
        self.buffers = self.stiffness.buffers
        self.time = 0.0
        self.minimum_separation = float("inf")
        self.minimum_robot_separation = float("inf")
        self.encounters, self.events = {}, []
        self.rejoin_routes = {}
        self.states = ["walking"] * len(people)
        self.yield_to = [NO_AGENT] * len(people)

    def _waiting_path(
        self,
        index,
        winner_position,
        winning_route,
        positions,
        radii,
        winner_radius=0.3,
        robot=False,
        winner_buffer=0.0,
    ):
        """Choose a reachable waiting point that leaves the winner's route open."""
        p = positions[index]
        route = np.asarray(winning_route)
        forward = route[-1, :2] - winner_position[:2]
        forward /= max(np.linalg.norm(forward), 1e-12)
        side = np.array([-forward[1], forward[0]])
        clearance = (
            radii[index]
            + winner_radius
            + pair_clearance(self.config, robot, self.buffers[index], winner_buffer)
        )

        def distance_to_route(point):
            a, b = winner_position[:2], route[-1, :2]
            d = b - a
            t = np.clip(np.dot(point[:2] - a, d) / max(np.dot(d, d), 1e-12), 0, 1)
            return np.linalg.norm(point[:2] - (a + t * d))

        if distance_to_route(p) >= clearance:
            return None
        candidates = []
        # A stalled encounter may begin past the nearby passing bay. Search
        # along both directions so the yielder can retreat to that bay.
        for back in (0, -0.7, 0.7, -1.4, 1.4, -2.1, 2.1):
            for offset in (0.7, 1.1, 1.6, 2.0):
                for sign in (-1, 1):
                    xy = p[:2] - forward * back + side * offset * sign
                    target = np.array([*xy, p[2]])
                    if (
                        not self.context.allows(target)
                        or distance_to_route(target) < clearance + 0.35
                    ):
                        continue
                    if any(
                        np.linalg.norm(target[:2] - q[:2])
                        < radii[index]
                        + radii[j]
                        + pair_clearance(
                            self.config, buffer_i=self.buffers[index], buffer_j=self.buffers[j]
                        )
                        for j, q in enumerate(positions)
                        if j != index
                    ):
                        continue
                    candidates.append(target)
        best, best_length = None, float("inf")
        for target in sorted(candidates, key=lambda q: np.linalg.norm(q - p)):
            if np.linalg.norm(target - p) >= best_length:
                break
            try:
                path = projected_path(self.scene, p, target)
                if len(path) > 1 and distance_to_route(path[-1]) < clearance + 0.35:
                    continue
                if len(path) > 1 and all(
                    self.context.motion_allowed(a, b, radii[index], self.config["safety_margin_m"])
                    for a, b in zip(path[:-1], path[1:])
                ):
                    # A retreat must not require walking through another
                    # stationary participant who is being told to wait.
                    blocked = False
                    for a, b in zip(path[:-1], path[1:]):
                        delta = b[:2] - a[:2]
                        for j, q in enumerate(positions):
                            if j == index:
                                continue
                            t = np.clip(
                                np.dot(q[:2] - a[:2], delta) / max(np.dot(delta, delta), 1e-12),
                                0,
                                1,
                            )
                            if np.linalg.norm(q[:2] - (a[:2] + t * delta)) < radii[index] + radii[
                                j
                            ] + pair_clearance(
                                self.config, buffer_i=self.buffers[index], buffer_j=self.buffers[j]
                            ):
                                blocked = True
                    if blocked:
                        continue
                    length = float(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1).sum())
                    if length < best_length:
                        best, best_length = path, length
            except RuntimeError:
                continue
        return best if best is not None else np.empty((0, 3))

    def routes_for_progress(self, positions, routes):
        result = list(routes)
        for index, path in list(self.rejoin_routes.items()):
            if np.linalg.norm(path[-1] - np.asarray(routes[index])[-1]) > 0.3:
                del self.rejoin_routes[index]
            else:
                result[index] = path
        return result

    def _rejoin(self, encounter, positions, routes):
        index = encounter["yielder"]
        if index >= 0 and encounter["waiting"] is not None and len(encounter["waiting"]):
            try:
                path = np.asarray(
                    projected_path(self.scene, positions[index], np.asarray(routes[index])[-1]),
                    float,
                )
                if len(path) > 1:
                    self.rejoin_routes[index] = path
            except RuntimeError:
                self._event("rejoin_unavailable", (index, encounter["winner"]))

    def _event(self, action, key, **values):
        self.events.append({"time_s": self.time, "action": action, "agents": list(key), **values})

    def _start_encounter(self, key, other_route, routes, local_context):
        i, j = key
        encounter = {
            "start": self.time,
            "clear_time": 0,
            "blocked_time": 0,
            "yielder": NO_AGENT,
            "winner": NO_AGENT,
            "waiting": None,
            "winner_axis": np.zeros(2),
            "yield_considered": False,
            "goal_i": np.asarray(routes[i])[-1].copy(),
            "goal_j": np.asarray(other_route)[-1].copy(),
        }
        self.encounters[key] = encounter
        self._event(
            "mutual_avoidance",
            key,
            density=local_context[i][0],
            passage_width_m=local_context[i][2],
        )
        return encounter

    def _select_yielder(self, encounter, key, q, w, other_route, positions, routes, radii):
        i, j = key
        # Never make a pedestrian wait for a parked externally controlled robot.
        if j < 0 and np.linalg.norm(w) < 0.15:
            return
        reserved = {
            agent
            for other in self.encounters.values()
            if other["yielder"] >= 0
            for agent in (other["yielder"], other["winner"])
            if agent >= 0
        }
        if i in reserved or (j >= 0 and j in reserved):
            return
        encounter["yield_considered"] = True
        if j < 0:
            if self.random.random() >= self.mus[i]:
                self._event("keep_orca", key, reason="robot_yield_probability")
                return
            yielder = i
        else:
            yielder = (
                i if self.random.random() < yielding_probability(self.mus[i], self.mus[j]) else j
            )
        winner = j if yielder == i else i
        wp = q if yielder == i else positions[i]
        wr = other_route if yielder == i else routes[i]
        winner_radius = radii[winner] if winner >= 0 else self.config["robot_radius_m"]
        waiting = self._waiting_path(
            yielder,
            wp,
            wr,
            positions,
            radii,
            winner_radius,
            robot=winner < 0,
            winner_buffer=self.buffers[winner] if winner >= 0 else 0.0,
        )
        adjusted = False
        if j >= 0 and waiting is not None and len(waiting) == 0:
            alternative = winner
            ap = positions[yielder]
            ar = routes[yielder]
            alternative_wait = self._waiting_path(
                alternative,
                ap,
                ar,
                positions,
                radii,
                radii[yielder],
                winner_buffer=self.buffers[yielder],
            )
            if alternative_wait is None or len(alternative_wait):
                yielder, winner = alternative, yielder
                wp, wr = ap, ar
                waiting, adjusted = alternative_wait, True
        axis = np.asarray(wr)[-1, :2] - wp[:2]
        axis /= max(np.linalg.norm(axis), 1e-12)
        encounter.update(yielder=yielder, winner=winner, waiting=waiting, winner_axis=axis)
        self._event(
            "yield",
            key,
            yielder=yielder,
            reason="orca_progress_blocked",
            blocked_for_s=encounter["blocked_time"],
            adjusted_for_reachable_space=adjusted,
            waiting_goal_world=(
                waiting[-1].tolist() if waiting is not None and len(waiting) else None
            ),
        )

    def _can_release(self, encounter, key, q, w, risk, dt, positions, routes):
        i, j = key
        cfg = self.config
        changed = np.linalg.norm(np.asarray(routes[i])[-1] - encounter["goal_i"]) > 0.3
        if j >= 0:
            changed |= np.linalg.norm(np.asarray(routes[j])[-1] - encounter["goal_j"]) > 0.3
        if changed:
            return True
        clear = not risk
        k = encounter["yielder"]
        if k >= 0:
            winner = encounter["winner"]
            wp = positions[winner] if winner >= 0 else q
            # Moving aside removes predicted conflict before the winner passes.
            # Keep the role until the winner has actually crossed the yielder.
            clear &= np.dot(wp[:2] - positions[k, :2], encounter["winner_axis"]) > 0
            if winner >= 0 and np.linalg.norm(wp[:2] - np.asarray(routes[winner])[-1, :2]) <= 0.31:
                clear = True
            elif winner < 0 and np.linalg.norm(w) < 0.15:
                clear = True
        encounter["clear_time"] = encounter["clear_time"] + dt if clear else 0
        return (
            self.time - encounter["start"] >= cfg["yield_min_hold_s"]
            and encounter["clear_time"] >= cfg["release_clear_s"]
        )

    def _apply_strategy(self, encounter, positions, speeds, preferred):
        k, winner, waiting = encounter["yielder"], encounter["winner"], encounter["waiting"]
        if k < 0:
            return
        self.yield_to[k] = winner
        preferred[k] = 0
        self.states[k] = "yielding"
        if waiting is None:
            return
        if len(waiting) == 0:
            self.states[k] = "waiting_for_space"
            if winner >= 0:
                preferred[winner] = 0
                self.states[winner] = "waiting_for_space"
        else:
            # Idle root motion can move a reached waiting pose a few centimeters.
            # Latch arrival so the winner is not stopped again on each sway.
            if np.linalg.norm(positions[k, :2] - waiting[-1, :2]) <= 0.35:
                encounter["waiting_arrived"] = True
            if encounter.get("waiting_arrived", False):
                return

            preferred[k] = preferred_velocity(positions[k], waiting, speeds[k])
            self.states[k] = "stepping_aside"
            if winner >= 0:
                preferred[winner] = 0
                self.states[winner] = "yielding"

    def update(
        self,
        dt,
        positions,
        preferred,
        routes,
        speeds,
        radii,
        robots,
        local_context,
        progress_available,
    ):
        self.time += dt
        cfg = self.config
        preferred = preferred.copy()
        original = preferred.copy()
        self.states = ["walking" if np.linalg.norm(v) > 1e-6 else "stopped" for v in preferred]
        self.yield_to = [NO_AGENT] * len(positions)
        pairs = [
            (i, j, positions[j], original[j], radii[j], routes[j])
            for i in range(len(positions))
            for j in range(i + 1, len(positions))
        ]
        for i in range(len(positions)):
            for j, (p, v, radius) in enumerate(robots):
                pairs.append(
                    (
                        i,
                        -j - 1,
                        np.asarray(p),
                        np.asarray(v),
                        radius,
                        [np.asarray(p), np.asarray(p) + np.r_[np.asarray(v) * 3, 0]],
                    )
                )
        for i, j, q, w, other_radius, other_route in pairs:
            key = (i, j)
            separation = float(np.linalg.norm(positions[i, :2] - q[:2]))
            if j >= 0:
                self.minimum_separation = min(self.minimum_separation, separation)
            else:
                self.minimum_robot_separation = min(self.minimum_robot_separation, separation)
            risk, _ = encounter_risk(
                positions[i],
                q,
                original[i],
                w,
                radii[i]
                + other_radius
                + pair_clearance(
                    cfg,
                    robot=j < 0,
                    buffer_i=self.buffers[i],
                    buffer_j=self.buffers[j] if j >= 0 else 0.0,
                ),
                cfg["encounter_horizon_s"],
            )
            encounter = self.encounters.get(key)
            if encounter is not None and self._can_release(
                encounter, key, q, w, risk, dt, positions, routes
            ):
                self._rejoin(encounter, positions, routes)
                self._event("resume", key)
                del self.encounters[key]
                encounter = None
            if encounter is None and risk:
                encounter = self._start_encounter(key, other_route, routes, local_context)
            if encounter is not None:
                unable = not progress_available[i] or (j >= 0 and not progress_available[j])
                encounter["blocked_time"] = encounter["blocked_time"] + dt if risk and unable else 0
                moving = speeds[i] > 0 and (j < 0 or speeds[j] > 0)
                if (
                    cfg["yielding_enabled"]
                    and moving
                    and not encounter["yield_considered"]
                    and encounter["blocked_time"] >= cfg["progress_hold_s"] - 1e-9
                ):
                    self._select_yielder(
                        encounter, key, q, w, other_route, positions, routes, radii
                    )
                k = encounter["yielder"]
                waiting = encounter["waiting"]
                if (
                    k >= 0
                    and waiting is not None
                    and len(waiting) == 0
                    and self.time - encounter.get("last_waiting_retry", encounter["start"])
                    >= cfg["yield_min_hold_s"]
                ):
                    winner = encounter["winner"]
                    wp = positions[winner] if winner >= 0 else q
                    wr = routes[winner] if winner >= 0 else other_route
                    winner_radius = radii[winner] if winner >= 0 else cfg["robot_radius_m"]
                    encounter["waiting"] = self._waiting_path(
                        k,
                        wp,
                        wr,
                        positions,
                        radii,
                        winner_radius,
                        robot=winner < 0,
                        winner_buffer=self.buffers[winner] if winner >= 0 else 0.0,
                    )
                    encounter["last_waiting_retry"] = self.time
                    if encounter["waiting"] is None or len(encounter["waiting"]):
                        self._event("waiting_space_available", key, yielder=k)
                self._apply_strategy(encounter, positions, speeds, preferred)
        return preferred
