"""Dependency-free configuration for ORCA progress and pedestrian yielding."""

import math

MU_DEFINITION = (
    "Episode-fixed social stiffness: anticipation time, personal buffer and neighbor range; "
    "also used as yield propensity by optional deadlock recovery"
)

DEFAULTS = {
    "enabled": False,
    "record_states_enabled": True,
    "yielding_enabled": True,
    "social_stiffness_enabled": True,
    "directional_responsibility_enabled": False,
    "directional_rear_weight": 0.1,
    "directional_smoothing_s": 0.2,
    "directional_emergency_gap_m": 0.15,
    "directional_emergency_transition_m": 0.3,
    "stiffness_time_horizon_min_s": 2.0,
    "stiffness_time_horizon_max_s": 10.0,
    "stiffness_buffer_max_m": 0.15,
    "stiffness_neighbor_distance_min_m": 2.0,
    "stiffness_neighbor_distance_max_m": 10.0,
    "mu_range": [0.0, 1.0],
    "time_horizon_s": 2.5,
    "neighbor_distance_m": 6.0,
    "safety_margin_m": 0.02,
    "robot_safety_margin_m": 0.33,
    "robot_radius_m": 0.55,
    "obstacle_horizon_s": 0.6,
    "obstacle_clearance_m": 0.1,
    "animation_path_clearance_m": 0.05,
    "animation_retarget_interval_s": 0.12,
    "animation_max_acceleration_mps2": 1.5,
    "animation_max_deceleration_mps2": 3.0,
    "animation_max_turn_rate_rps": 3.14,
    "animation_walk_reference_mps": 1.0,
    "animation_idle_delay_s": 0.2,
    "tracking_margin_m": 0.02,
    "density_radius_m": 1.5,
    "progress_hold_s": 0.5,
    "progress_min_mps": 0.08,
    "progress_min_fraction": 0.15,
    "encounter_horizon_s": 3.0,
    "yield_min_hold_s": 0.5,
    "release_clear_s": 0.5,
    "passing_bias_mps": 0.015,
    "stationary_robot_bypass_enabled": True,
    "state_topic": "/pedestrian_states",
}


def validate_social_config(value):
    if not isinstance(value, dict):
        raise ValueError("pedestrians.social_navigation must be a mapping")
    unknown = set(value) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"Unknown social_navigation settings: {sorted(unknown)}")
    result = {**DEFAULTS, **value}
    boolean_keys = (
        "enabled",
        "record_states_enabled",
        "yielding_enabled",
        "social_stiffness_enabled",
        "directional_responsibility_enabled",
        "stationary_robot_bypass_enabled",
    )
    for key in boolean_keys:
        if not isinstance(result[key], bool):
            raise ValueError(f"social_navigation.{key} must be a boolean")
    for key in DEFAULTS:
        if key in (*boolean_keys, "mu_range", "state_topic"):
            continue
        x = result[key]
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x < 0:
            raise ValueError(f"social_navigation.{key} must be finite and non-negative")
        if (
            key
            not in (
                "passing_bias_mps",
                "safety_margin_m",
                "robot_safety_margin_m",
                "tracking_margin_m",
                "obstacle_clearance_m",
                "animation_path_clearance_m",
                "stiffness_buffer_max_m",
            )
            and x == 0
        ):
            raise ValueError(f"social_navigation.{key} must be positive")
    values = result["mu_range"]
    if (
        not isinstance(values, (list, tuple))
        or len(values) != 2
        or any(
            isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
            for x in values
        )
        or not 0 <= values[0] <= values[1] <= 1
    ):
        raise ValueError("social_navigation.mu_range must be ordered within [0, 1]")
    if result["progress_min_fraction"] >= 1:
        raise ValueError("social_navigation.progress_min_fraction must be less than 1")
    if result["directional_rear_weight"] > 1:
        raise ValueError("social_navigation.directional_rear_weight must be in (0, 1]")
    if result["stiffness_time_horizon_min_s"] > result["stiffness_time_horizon_max_s"]:
        raise ValueError("Social stiffness time horizons must be ordered")
    if result["stiffness_neighbor_distance_min_m"] > result["stiffness_neighbor_distance_max_m"]:
        raise ValueError("Social stiffness neighbor distances must be ordered")
    if not isinstance(result["state_topic"], str) or not result["state_topic"]:
        raise ValueError("social_navigation.state_topic must be a topic name")
    return result
