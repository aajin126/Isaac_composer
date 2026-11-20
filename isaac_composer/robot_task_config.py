"""Robot task defaults shared by ROS configuration and Isaac Sim runtime."""

TASK_DEFAULTS = {
    'enabled': False, 'min_goal_distance_m': 5.0, 'max_goal_distance_m': 15.0,
    'robot_radius_m': 0.55, 'clearance_m': 0.65, 'spawn_height_m': 0.65,
    'goal_radius_m': 0.5, 'stopped_speed_mps': 0.1,
    'stopped_yaw_rate_rps': 0.2, 'arrival_hold_sec': 0.5, 'max_attempts': 500,
}
GOAL_FRAME = 'episode_goal'
