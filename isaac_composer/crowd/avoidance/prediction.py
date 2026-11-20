"""Constant-velocity encounter prediction."""

import math


def encounter_risk(p, q, v, w, radius, horizon):
    # This is a scalar two-agent query, called for every pair. Avoid allocating
    # several tiny NumPy arrays and dispatching clip/norm for each pair.
    px, py = float(q[0]) - float(p[0]), float(q[1]) - float(p[1])
    vx, vy = float(w[0]) - float(v[0]), float(w[1]) - float(v[1])
    speed_sq = vx * vx + vy * vy
    t = min(float(horizon), max(0.0, -(px * vx + py * vy) / max(speed_sq, 1e-12)))
    return math.hypot(px + t * vx, py + t * vy) < radius, t
