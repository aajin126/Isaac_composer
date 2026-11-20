"""Episode-fixed social stiffness for ordinary ORCA encounters."""

import numpy as np

from ..config import MU_DEFINITION


class SocialStiffness:
    """Map one continuous trait to anticipation time and extra personal space.

    PRISM Section 4.1 linearly interpolates all three ORCA parameters.
    This changes ORCA agent parameters, not physical colliders or NavMesh bake
    radii. Deadlock recovery is a separate
    optional policy and may still use the same trait as its yield propensity.
    """

    def __init__(self, people, config):
        self.people, self.config = people, config
        self.mus = np.asarray([float(p.get("mu", 0.5)) for p in people])
        if not np.all(np.isfinite(self.mus) & (self.mus >= 0) & (self.mus <= 1)):
            raise ValueError("Pedestrian mu must be finite and within [0, 1]")
        self.enabled = config["social_stiffness_enabled"]
        if self.enabled:
            low = config["stiffness_time_horizon_min_s"]
            high = config["stiffness_time_horizon_max_s"]
            self.horizons = low + (high - low) * self.mus
            self.buffers = config["stiffness_buffer_max_m"] * self.mus
            near = config["stiffness_neighbor_distance_min_m"]
            far = config["stiffness_neighbor_distance_max_m"]
            self.neighbor_distances = near + (far - near) * self.mus
        else:
            self.horizons = np.full(len(people), config["time_horizon_s"])
            self.buffers = np.zeros(len(people))
            self.neighbor_distances = np.full(len(people), config["neighbor_distance_m"])

    def records(self):
        return [
            {
                "actor_id": p.get("id", i),
                "mu": float(self.mus[i]),
                "time_horizon_s": float(self.horizons[i]),
                "personal_buffer_m": float(self.buffers[i]),
                "neighbor_distance_m": float(self.neighbor_distances[i]),
            }
            for i, p in enumerate(self.people)
        ]

    def diagnostics(self):
        return {
            "enabled": self.enabled,
            "mu_definition": MU_DEFINITION,
            "reference": {
                "url": "https://arxiv.org/pdf/2609.18125",
                "version": "v1",
                "section": "4.1",
                "interpolation": "Phi_i = mu_i * Phi_cautious + (1 - mu_i) * Phi_aggressive",
            },
            "buffer_application": "Add each pedestrian buffer to its own ORCA radius; pair buffers sum",
            "baseline_neighbor_distance_m": self.config["neighbor_distance_m"],
            "additional_policies": {
                "deadlock_recovery_enabled": self.config["yielding_enabled"],
                "passing_bias_mps": self.config["passing_bias_mps"],
                "stationary_robot_bypass_enabled": self.config["stationary_robot_bypass_enabled"],
                "safety_margin_m": self.config["safety_margin_m"],
                "tracking_margin_m": self.config["tracking_margin_m"],
                "robot_safety_margin_m": self.config["robot_safety_margin_m"],
            },
            "actors": self.records(),
        }
