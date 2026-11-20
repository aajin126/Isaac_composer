"""Directional sharing of pedestrian avoidance, independent of preferred velocity.

This is a local ORCA extension, not the PRISM social-stiffness interpolation.
The physical body remains a disc. Both sides use complementary pair shares.
"""

import math
import numpy as np


def character_body_yaws(orientations):
    """World heading of the character's LOCAL -Y front, quaternion order xyzw.

    The People locomotion assets face -Y, unlike the robot's +X convention.
    Rotate the actual front vector, rather than treating quaternion yaw as body
    heading. This also handles small root pitch/roll without a planar shortcut.
    """
    q = np.asarray(orientations, dtype=float).reshape(-1, 4)
    norms = np.linalg.norm(q, axis=1)
    if not np.isfinite(q).all() or not np.isfinite(norms).all() or np.any(norms < 1e-9):
        raise ValueError("Character orientations must be finite nonzero quaternions")
    x, y, z, w = (q / norms[:, None]).T
    front_x = 2 * (w * z - x * y)
    front_y = 2 * (x * x + z * z) - 1
    return np.arctan2(front_y, front_x)


def responsibility_settings(config):
    return {
        "enabled": config["directional_responsibility_enabled"],
        "rear_weight": config["directional_rear_weight"],
        "smoothing_s": config["directional_smoothing_s"],
        "emergency_gap_m": config["directional_emergency_gap_m"],
        "emergency_transition_m": config["directional_emergency_transition_m"],
        "heading_source": "observed_world_body_orientation",
        "character_local_forward_axis": "negative_y",
        "weight_formula": "rear_weight + (1 - rear_weight) * (1 + cos(theta)) / 2",
        "share_formula": "own_weight / (own_weight + other_weight)",
        "close_contact_policy": "smoothly_restore_equal_sharing_using_physical_body_gap",
        "one_sided_detection_policy": "detecting_pedestrian_takes_full_responsibility",
        "robot_avoidance_responsibility": 1.0,
    }


class DirectionalResponsibility:
    def __init__(self, people, config):
        self.config = config
        self.heading = np.array([p.get("pose", {}).get("yaw", 0.0) for p in people], float)
        self.smoothed = np.full((len(people), len(people)), 0.5)
        self.shares = self.smoothed.copy()
        self.initialized = False

    def update(self, dt, positions, velocities, radii, headings=None):
        """Update once per observation; repeated ORCA solves reuse these shares.

        Isaac always supplies measured body orientation, including while idle.
        Non-Isaac callers may omit it: use moving velocity, retaining the last
        heading at rest. Missing observations must never reset an idle heading.
        """
        cfg = self.config
        if not cfg["directional_responsibility_enabled"]:
            return None
        n = len(self.heading)
        p = np.asarray(positions, float).reshape(n, -1)[:, :2] if n else np.empty((0, 2))
        if headings is None:
            v = np.asarray(velocities, float).reshape(n, 2)
            observed = np.where(
                np.linalg.norm(v, axis=1) > 0.05, np.arctan2(v[:, 1], v[:, 0]), self.heading
            )
        else:
            observed = np.asarray(headings, float)
        r = np.asarray(radii, float)
        if observed.shape != (n,) or r.shape != (n,) or not np.isfinite(observed).all():
            raise ValueError(
                "Directional responsibility needs one finite body heading per pedestrian"
            )
        if not np.isfinite(p).all() or not np.isfinite(r).all() or np.any(r < 0):
            raise ValueError("Directional responsibility needs finite positions and body radii")
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("Directional responsibility timestep must be positive and finite")
        blend = -math.expm1(-dt / cfg["directional_smoothing_s"])
        if not self.initialized:
            self.heading = observed.copy()
        else:
            delta = observed - self.heading
            self.heading += blend * np.arctan2(np.sin(delta), np.cos(delta))
            self.heading = np.arctan2(np.sin(self.heading), np.cos(self.heading))
        delta = p[None, :, :] - p[:, None, :]
        distance = np.linalg.norm(delta, axis=2)
        forward = np.c_[np.cos(self.heading), np.sin(self.heading)]
        cosine = np.clip(
            np.einsum("ijk,ik->ij", delta, forward) / np.maximum(distance, 1e-9), -1, 1
        )
        rear = cfg["directional_rear_weight"]
        weight = rear + (1 - rear) * (1 + cosine) / 2
        target = weight / (weight + weight.T)
        if not self.initialized:
            self.smoothed = target
        else:
            self.smoothed += blend * (target - self.smoothed)
        # Restore equal responsibility near the physical bodies, independently
        # of mu's personal buffer. Emergency response must not wait for the filter.
        gap = distance - r[:, None] - r[None, :]
        amount = np.clip(
            (gap - cfg["directional_emergency_gap_m"]) / cfg["directional_emergency_transition_m"],
            0,
            1,
        )
        amount = amount * amount * (3 - 2 * amount)
        self.shares = 0.5 + amount * (self.smoothed - 0.5)
        np.fill_diagonal(self.shares, 0.5)
        self.initialized = True
        return self.shares

    def diagnostics(self):
        return {
            **responsibility_settings(self.config),
            "body_heading_rad": self.heading.tolist(),
        }
