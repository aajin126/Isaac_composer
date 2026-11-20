"""Project both endpoints before asking native NavMesh for a path."""

import numpy as np


def projected_path(scene, start, goal):
    start = np.asarray(start, float)
    goal = np.asarray(goal, float)
    if hasattr(scene, "project_world"):
        source = np.asarray(scene.project_world(start), float)
        target = np.asarray(scene.project_world(goal), float)
    else:
        source, target = start, goal
    path = np.asarray(scene.shortest_path(source, target), float)
    if len(path) > 1 and np.linalg.norm(path[0] - start) > 1e-6:
        path = np.vstack([start, path])
    return path
