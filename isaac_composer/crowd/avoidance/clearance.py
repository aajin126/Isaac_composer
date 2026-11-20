"""Pair clearance shared by official RVO2 and yielding intent."""


def pedestrian_padding(config):
    return config["safety_margin_m"] / 2 + config["tracking_margin_m"]


def pair_clearance(config, robot=False, buffer_i=0.0, buffer_j=0.0):
    padding = pedestrian_padding(config)
    base = max(padding, config["robot_safety_margin_m"]) if robot else 2 * padding
    return base + buffer_i + buffer_j


def interaction_radii(radii, robots, config, buffers=None):
    """Encode pair gaps as RVO2 radii without changing physical body sizes.

    External robots do not solve their own constraints. Assign the remaining
    robot-pedestrian margin to the external radius so reducing pedestrian
    padding does not also reduce teleop robot clearance.
    """
    padding = pedestrian_padding(config)
    robot_padding = pair_clearance(config, robot=True) - padding
    buffers = [0.0] * len(radii) if buffers is None else buffers
    return [r + padding + b for r, b in zip(radii, buffers)] + [
        robot[2] + robot_padding for robot in robots
    ]
