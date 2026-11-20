"""Persistent lack of safe forward/sideways movement, independent of density."""

import numpy as np


class ProgressMonitor:
    def __init__(self, count, config):
        self.config = config
        self.elapsed = np.zeros(count)
        self.available = np.ones(count, dtype=bool)
        self.blocked = np.zeros(count, dtype=bool)
        self.candidate_forward = np.zeros(count)
        self.goals = None

    def resolve(self, positions, preferred, speeds, output, feasible, solver):
        result = np.asarray(output).copy()
        valid = np.asarray(feasible, dtype=bool).copy()
        available = np.ones(len(preferred), dtype=bool)
        tested_forward = np.zeros(len(preferred))
        for i, intent in enumerate(preferred):
            norm = float(np.linalg.norm(intent))
            if speeds[i] <= 0 or norm < 1e-9:
                continue
            forward = intent / norm
            threshold = max(
                self.config["progress_min_mps"], speeds[i] * self.config["progress_min_fraction"]
            )
            candidates = [result[i]] if valid[i] else []
            useful = lambda v: np.linalg.norm(v) >= threshold and np.dot(v, forward) >= -1e-5
            if not any(useful(v) for v in candidates):
                candidates += solver.alternatives(i, np.asarray(positions[i]), intent)
            eligible = [v for v in candidates if useful(v)]
            available[i] = bool(eligible)
            tested_forward[i] = max((float(np.dot(v, forward)) for v in candidates), default=0.0)
            if eligible and not (valid[i] and useful(result[i])):
                result[i] = min(eligible, key=lambda v: np.linalg.norm(v - intent))
                valid[i] = True
        return result, valid, available, tested_forward

    def update(self, dt, positions, preferred, speeds, routes, output, feasible, solver):
        goals = np.array([np.asarray(route)[-1, :2] for route in routes])
        if self.goals is not None:
            self.elapsed[np.linalg.norm(goals - self.goals, axis=1) > 0.3] = 0
        self.goals = goals
        self.velocity, self.feasible, self.available, self.candidate_forward = self.resolve(
            positions, preferred, speeds, output, feasible, solver
        )
        for i, intent in enumerate(preferred):
            idle = speeds[i] <= 0 or np.linalg.norm(intent) < 1e-9
            self.elapsed[i] = 0 if idle or self.available[i] else self.elapsed[i] + dt
        self.blocked = self.elapsed >= self.config["progress_hold_s"] - 1e-9
        return self.blocked

    def diagnostics(self):
        return {
            "available": self.available.tolist(),
            "blocked_for_s": self.elapsed.tolist(),
            "candidate_forward_mps": self.candidate_forward.tolist(),
        }
