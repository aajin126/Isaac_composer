"""NavMesh path following and preferred velocities."""

import numpy as np

from ..avoidance.prediction import encounter_risk
from ..avoidance.clearance import pair_clearance


def preferred_velocity(position, route, speed, bias=0.0, goal_tolerance_m=0.3):
    route = np.asarray(route, float)
    if (
        len(route) < 2
        or speed <= 0
        or np.linalg.norm(position[:2] - route[-1, :2]) <= goal_tolerance_m
    ):
        return np.zeros(2)
    delta = route[1:, :2] - route[:-1, :2]
    alpha = np.clip(
        np.sum((position[:2] - route[:-1, :2]) * delta, axis=1)
        / np.maximum(np.sum(delta * delta, axis=1), 1e-12),
        0,
        1,
    )
    closest = route[:-1, :2] + alpha[:, None] * delta
    index = int(np.argmin(np.linalg.norm(closest - position[:2], axis=1)))
    target = closest[index].copy()
    budget = max(0.7, speed * 0.8)
    for point in route[index + 1 :, :2]:
        distance = float(np.linalg.norm(point - target))
        if distance >= budget:
            target += (point - target) * budget / max(distance, 1e-12)
            break
        budget -= distance
        target = point
    direction = target - position[:2]
    distance = float(np.linalg.norm(direction))
    if distance < 1e-9:
        return np.zeros(2)
    direction /= distance
    preferred = direction * min(speed, np.linalg.norm(route[-1, :2] - position[:2]) * 1.5)
    # A tiny deterministic right-hand preference breaks perfectly symmetric
    # head-on deadlocks without assigning any person priority.
    preferred += bias * np.array([direction[1], -direction[0]])
    return preferred * min(1.0, speed / max(np.linalg.norm(preferred), 1e-12))


def pedestrian_preferences(positions, routes, speeds, radii, robots, config, buffers=None):
    """Route intent with a right-side bypass for stationary external agents.

    This changes only intent. RVO2 still constrains every returned preference.
    A parked robot cannot reciprocate, and a straight preferred velocity can
    otherwise settle at a zero-velocity local minimum directly in front of it.
    """
    result = np.array(
        [
            preferred_velocity(p, path, speed, config["passing_bias_mps"])
            for p, path, speed in zip(positions, routes, speeds)
        ]
    )
    buffers = np.zeros(len(positions)) if buffers is None else buffers
    if not config["stationary_robot_bypass_enabled"]:
        return result
    for i, p in enumerate(positions):
        for q, velocity, radius in robots:
            risk, _ = encounter_risk(
                p,
                np.asarray(q),
                result[i],
                np.asarray(velocity),
                radii[i] + radius + pair_clearance(config, robot=True, buffer_i=buffers[i]),
                config["encounter_horizon_s"],
            )
            if risk and np.linalg.norm(velocity) < 0.15 and np.linalg.norm(result[i]) > 1e-6:
                direction = result[i] / np.linalg.norm(result[i])
                result[i] += np.array([direction[1], -direction[0]]) * speeds[i] * 0.7
                result[i] *= min(1.0, speeds[i] / np.linalg.norm(result[i]))
    return result
