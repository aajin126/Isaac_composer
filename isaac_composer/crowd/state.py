"""Measured world-frame agent states, independent of movement commands."""

import numpy as np

PEDESTRIAN_FIELDS = [
    "id",
    "x",
    "y",
    "vx",
    "vy",
    "mu",
    "local_density",
    "walkable_area_m2",
    "passage_width_m",
    "state",
    "yield_to",
]
STATE_CODES = {
    "walking": 0,
    "yielding": 1,
    "stepping_aside": 2,
    "stopped": 3,
    "infeasible": 4,
    "waiting_for_space": 5,
}


class MeasuredVelocities:
    """Finite differences from one shared observation per control tick."""

    def __init__(self):
        self.previous = None
        self.time = None

    def update(self, stamp, positions):
        positions = np.asarray(positions, dtype=float)[:, :2].copy()
        if self.previous is None or self.time is None or stamp <= self.time:
            velocity = np.zeros_like(positions)
        else:
            velocity = (positions - self.previous) / (stamp - self.time)
        self.previous, self.time = positions, float(stamp)
        return velocity


def body_velocity_to_world(body_velocity, world_yaw):
    c, s = np.cos(world_yaw), np.sin(world_yaw)
    return np.array([[c, -s], [s, c]]) @ np.asarray(body_velocity)[:2]
