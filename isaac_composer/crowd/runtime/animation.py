"""Native navigation paths for the animation graph's locomotion preview."""

import numpy as np
import math
from ..navigation.paths import projected_path
from ..behavior.preference import preferred_velocity


def locomotion_path(scene, position, velocity, accept=None):
    speed = float(np.linalg.norm(velocity))
    if speed < 1e-6:
        return np.asarray([position])
    direction = np.r_[np.asarray(velocity) / speed, 0.0]
    # Stay beyond the animation graph's arrival/braking zone. Reusing tiny
    # 0.7m goals makes locomotion repeatedly brake toward temporary endpoints.
    for distance in (max(1.2, speed * 1.5), 0.7, 0.5, 0.35):
        try:
            path = projected_path(scene, position, np.asarray(position) + direction * distance)
            if len(path) > 1 and (accept is None or accept(path)):
                return path
        except RuntimeError:
            continue
    return np.asarray([position])


class AnimationCommands:
    """Stateful locomotion input: bounded references and persistent path targets.

    Smooth the reference, then project it through the SAME official ORCA
    constraints when that region exists. Safety can override the comfort rate
    limits; this is counted. Empty ORCA regions remain explicitly infeasible:
    their bounded fallback is checked against the physical navigation boundary.
    """

    def __init__(self, people, config):
        self.config = config
        self.velocity = np.zeros((len(people), 2))
        self.heading = np.array([p.get("pose", {}).get("yaw", 0.0) for p in people], dtype=float)
        self.paths = [None] * len(people)
        self.path_heading = self.heading.copy()
        self.age = np.full(len(people), np.inf)
        self.walk = np.zeros(len(people))
        self.playback_rate = np.ones(len(people))
        self.walking = np.zeros(len(people), dtype=bool)
        self.action_walking = np.zeros(len(people), dtype=bool)
        self.idle_elapsed = np.zeros(len(people))
        self.path_changed = np.zeros(len(people), dtype=bool)
        self.action_changed = np.ones(len(people), dtype=bool)
        self.initialized = np.zeros(len(people), dtype=bool)
        self.path_updates = 0
        self.safety_overrides = 0
        self.rejected = 0
        self.infeasible_references = 0
        self.recovered_paths = 0
        self.action_switches = 0

    def _check_path(self, index, position, path, speed, solver, unresolved):
        """Validate the velocity the graph's path actually asks it to follow."""
        tangent = self._path_velocity(position, path, speed)
        checked = solver.project_velocity(index, np.asarray(position), tangent)
        if unresolved:
            context = getattr(solver, "context", None)
            valid = context is not None and context.motion_allowed(
                np.asarray(position)[:2],
                np.asarray(position)[:2] + tangent * 0.12,
                solver.radii[index],
                self.config["safety_margin_m"],
            )
        else:
            valid = checked is not None and np.linalg.norm(checked - tangent) <= 0.02
        return bool(valid and len(path) > 1 and np.linalg.norm(tangent) > 1e-6), tangent, checked

    def _find_path(self, index, position, velocity, scene, solver, unresolved):
        """Try native previews, correcting their direction with the same ORCA LP.

        No new passing policy: all trials reuse the current constraints. A
        failed long preview must not discard an executable shorter preview.
        """
        candidate = np.asarray(velocity)
        rejected = 0
        for _ in range(3):
            correction = None

            def accept(path):
                nonlocal correction, rejected
                valid, _, checked = self._check_path(
                    index, position, path, float(np.linalg.norm(candidate)), solver, unresolved
                )
                if not valid:
                    rejected += 1
                    if checked is not None and np.linalg.norm(checked) > 0.02:
                        correction = checked
                return valid

            path = locomotion_path(scene, position, candidate, accept)
            if len(path) > 1:
                if rejected:
                    self.recovered_paths += 1
                return path, float(np.linalg.norm(candidate))
            if correction is None or unresolved:
                break
            candidate = correction
        return np.asarray([position]), float(np.linalg.norm(velocity))

    def _reference(self, index, requested, dt):
        cfg = self.config
        target_speed = float(np.linalg.norm(requested))
        heading = self.heading[index]
        if target_speed > 1e-6:
            desired_heading = math.atan2(requested[1], requested[0])
            difference = math.atan2(
                math.sin(desired_heading - heading), math.cos(desired_heading - heading)
            )
            heading += np.clip(
                difference,
                -cfg["animation_max_turn_rate_rps"] * dt,
                cfg["animation_max_turn_rate_rps"] * dt,
            )
            # Do not keep charging forward while turning to a retreat command.
            remaining = desired_heading - heading
            target_speed *= max(0.0, math.cos(remaining))
        speed = float(np.linalg.norm(self.velocity[index]))
        rate = (
            cfg["animation_max_acceleration_mps2"]
            if target_speed > speed
            else cfg["animation_max_deceleration_mps2"]
        )
        speed += np.clip(target_speed - speed, -rate * dt, rate * dt)
        self.heading[index] = heading
        return speed * np.array([math.cos(heading), math.sin(heading)])

    def update(self, dt, positions, requested, scene, solver):
        cfg = self.config
        applied = np.zeros_like(requested)
        paths, feasible = [], []
        self.path_changed[:] = False
        for i, (position, desired) in enumerate(zip(positions, requested)):
            was_walking = bool(self.walking[i])
            was_action_walking = bool(self.action_walking[i])
            reference = self._reference(i, desired, dt)
            selected = solver.project_velocity(i, np.asarray(position), reference)
            ok = selected is not None
            unresolved = not ok
            if not ok:
                # The upstream solver also returns a fallback when its region
                # is empty. Conditioning must not turn every such tick into a
                # permanent stop at an aliased center-mesh boundary. Retain a
                # bounded reference only when the physical NavMesh check allows
                # it; keep this explicitly marked infeasible in the recording.
                context = getattr(solver, "context", None)
                allowed = context is not None and context.motion_allowed(
                    np.asarray(position)[:2],
                    np.asarray(position)[:2] + reference * 0.12,
                    solver.radii[i],
                    cfg["safety_margin_m"],
                )
                selected = reference if allowed else np.zeros(2)
                self.infeasible_references += 1
            elif np.linalg.norm(selected - reference) > 1e-3:
                self.safety_overrides += 1
            speed = float(np.linalg.norm(selected))
            heading = math.atan2(selected[1], selected[0]) if speed > 1e-6 else self.heading[i]
            self.age[i] += dt
            path = self.paths[i]
            turn = abs(
                math.atan2(
                    math.sin(heading - self.path_heading[i]),
                    math.cos(heading - self.path_heading[i]),
                )
            )
            near = path is None or np.linalg.norm(np.asarray(position)[:2] - path[-1, :2]) < 0.65
            # Keep a fixed world-space target while its direction is useful.
            # A materially different feasible avoidance direction retargets at
            # most once per interval; obsolete targets are never blindly reused.
            refresh = (
                path is None
                or near
                or (turn > math.radians(12) and self.age[i] >= cfg["animation_retarget_interval_s"])
            )
            if speed > 0.02:
                if not refresh:
                    valid_path, _, _ = self._check_path(
                        i, position, path, speed, solver, unresolved
                    )
                    if not valid_path:
                        refresh = True
                if refresh:
                    path, speed = self._find_path(i, position, selected, scene, solver, unresolved)
                    self.paths[i] = path
                    self.path_heading[i] = heading
                    self.age[i] = 0
                    self.path_updates += 1
                    self.path_changed[i] = True
                valid_path, tangent, _ = self._check_path(
                    i, position, path, speed, solver, unresolved
                )
                if not valid_path:
                    selected = np.zeros(2)
                    ok = False
                    self.paths[i] = None
                    self.rejected += 1
                else:
                    selected = tangent
            else:
                path = np.asarray([position])
                self.paths[i] = None
            applied[i] = selected
            self.velocity[i] = selected
            self.heading[i] = heading
            # Walk is a normalized animation input, not a velocity in m/s.
            ratio = np.linalg.norm(selected) / cfg["animation_walk_reference_mps"]
            self.walk[i] = np.clip(ratio, 0, 1)
            # Above full Walk, advance BOTH the legs and root motion faster.
            # Idle and LookAround keep their normal clock when movement stops.
            self.playback_rate[i] = max(1.0, float(ratio))
            if self.walk[i] >= 0.05:
                self.walking[i] = True
            elif self.walk[i] <= 0.02:
                self.walking[i] = False
            if self.walking[i]:
                self.idle_elapsed[i] = 0
                self.action_walking[i] = True
            else:
                self.idle_elapsed[i] += dt
                if self.idle_elapsed[i] >= cfg["animation_idle_delay_s"] - 1e-9:
                    self.action_walking[i] = False
            self.action_changed[i] = (
                not self.initialized[i] or was_action_walking != self.action_walking[i]
            )
            if self.initialized[i] and self.action_changed[i]:
                self.action_switches += 1
            self.initialized[i] = True
            if self.walking[i] and not was_walking:
                self.path_changed[i] = True
            if not self.walking[i] and was_walking:
                # Stop input is immediate even while Action remains Walk for
                # the debounce window. Never leave an old forward path active.
                self.path_changed[i] = True
                path = np.asarray([position])
            paths.append(path)
            feasible.append(ok)
        return applied, paths, feasible

    @staticmethod
    def _path_velocity(position, path, speed):
        # A preview endpoint is not the pedestrian's destination. Applying the
        # route's 0.3m arrival test here creates false zero-speed commands.
        tangent = preferred_velocity(position, path, 1.0, goal_tolerance_m=0.0)
        norm = np.linalg.norm(tangent)
        return tangent / norm * speed if norm > 1e-9 else np.zeros(2)

    def diagnostics(self):
        return {
            "path_updates": self.path_updates,
            "safety_overrides": self.safety_overrides,
            "rejected_commands": self.rejected,
            "infeasible_reference_ticks": self.infeasible_references,
            "walk_blend": self.walk.tolist(),
            "playback_rate": self.playback_rate.tolist(),
            "action_walking": self.action_walking.tolist(),
            "action_switches": self.action_switches,
            "recovered_paths": self.recovered_paths,
        }
