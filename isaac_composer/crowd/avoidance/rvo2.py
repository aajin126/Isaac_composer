"""ctypes adapter to pinned official RVO2; no Python ORCA reimplementation."""

import ctypes
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from .clearance import interaction_radii


def native_library():
    root = Path(__file__).resolve().parents[3] / "third_party/rvo2"
    if not root.is_dir():
        from ament_index_python.packages import get_package_share_directory

        root = Path(get_package_share_directory("isaac_composer")) / "third_party/rvo2"
    sources = sorted((root / "src").glob("*.cc")) + [root / "bridge.cc"]
    digest = hashlib.sha256()
    for path in sources + sorted((root / "src").glob("*.h")):
        digest.update(path.read_bytes())
    directory = Path(tempfile.gettempdir()) / "isaac_composer_rvo2" / digest.hexdigest()[:16]
    target = directory / "libcrowd_rvo2.so"
    if not target.exists():
        directory.mkdir(parents=True, exist_ok=True)
        temporary = target.with_suffix(f".{os.getpid()}.tmp.so")
        command = [
            "g++",
            "-std=c++17",
            "-O2",
            "-fPIC",
            "-shared",
            "-I",
            str(root / "src"),
            *map(str, sources),
            "-o",
            str(temporary),
        ]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError("Official RVO2 native build failed:\n" + result.stderr)
        temporary.replace(target)
    lib = ctypes.CDLL(str(target))
    ptr = ctypes.c_void_p
    f = ctypes.c_float
    i = ctypes.c_int
    lib.crowd_rvo_create.argtypes = [f]
    lib.crowd_rvo_create.restype = ptr
    lib.crowd_rvo_destroy.argtypes = [ptr]
    lib.crowd_rvo_destroy.restype = None
    lib.crowd_rvo_agent.argtypes = [ptr, f, f, f, f, f, i]
    lib.crowd_rvo_agent.restype = i
    lib.crowd_rvo_segment.argtypes = [ptr, f, f, f, f]
    lib.crowd_rvo_segment.restype = None
    lib.crowd_rvo_process.argtypes = [ptr]
    lib.crowd_rvo_process.restype = None
    array = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    ints = np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS")
    lib.crowd_rvo_step.argtypes = [ptr, i, f, array, array, array, array, array, array, ints]
    lib.crowd_rvo_step.restype = None
    lib.crowd_rvo_query.argtypes = [ptr, i, array, i, array]
    lib.crowd_rvo_query.restype = i
    lib.crowd_rvo_parameters.argtypes = [ptr, i, array]
    lib.crowd_rvo_parameters.restype = None
    lib.crowd_rvo_neighbors.argtypes = [ptr, i, ints, i]
    lib.crowd_rvo_neighbors.restype = i
    lib.crowd_rvo_responsibilities.argtypes = [ptr, i, ctypes.POINTER(ctypes.c_double)]
    lib.crowd_rvo_responsibilities.restype = None
    lib.crowd_rvo_responsibility.argtypes = [ptr, i, i]
    lib.crowd_rvo_responsibility.restype = f
    return lib


def boundary_segments(context):
    """Merged edges between free and blocked NavMesh cells, plus outer edges."""
    mask = np.pad(context.mask, 1)
    b = context.raster.bounds

    def runs(row):
        edges = np.diff(np.r_[False, row, False].astype(int))
        return zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))

    for r in range(context.raster.height):
        free = mask[r + 1, 1:-1]
        for adjacent, offset in ((mask[r, 1:-1], 0), (mask[r + 2, 1:-1], 1)):
            y = b.y_max - (r + offset) * context.dy
            for a, z in runs(free & ~adjacent):
                yield (b.x_min + a * context.dx, y, b.x_min + z * context.dx, y)
    for c in range(context.raster.width):
        free = mask[1:-1, c + 1]
        for adjacent, offset in ((mask[1:-1, c], 0), (mask[1:-1, c + 2], 1)):
            x = b.x_min + (c + offset) * context.dx
            for a, z in runs(free & ~adjacent):
                yield (x, b.y_max - a * context.dy, x, b.y_max - z * context.dy)


class RVO2Solver:
    def __init__(self, context, config, stiffness=None):
        self.context, self.config = context, config
        self.stiffness = stiffness
        self.lib = native_library()
        self.handle = self.lib.crowd_rvo_create(0.02)
        self.counts = None
        for edge in boundary_segments(context):
            self.lib.crowd_rvo_segment(self.handle, *edge)
        self.lib.crowd_rvo_process(self.handle)

    def compute(
        self, dt, positions, velocities, preferred, speeds, radii, robots, responsibilities=None
    ):
        n = len(positions)
        counts = (n, len(robots))
        cfg = self.config
        self.radii = list(radii)
        if self.stiffness is not None and len(self.stiffness.mus) != n:
            raise ValueError("Social stiffness roster must match the ORCA pedestrian roster")
        if self.counts is None:
            for k in range(n + len(robots)):
                self.lib.crowd_rvo_agent(
                    self.handle,
                    (
                        float(self.stiffness.horizons[k])
                        if self.stiffness is not None and k < n
                        else cfg["time_horizon_s"]
                    ),
                    cfg["obstacle_horizon_s"],
                    (
                        float(self.stiffness.neighbor_distances[k])
                        if self.stiffness is not None and k < n
                        else cfg["neighbor_distance_m"]
                    ),
                    0.3,
                    cfg["obstacle_clearance_m"],
                    int(k >= n),
                )
            self.counts = counts
        if self.counts != counts:
            raise ValueError("Agent roster cannot change inside an episode")
        shares = None
        if responsibilities is not None:
            pair = np.asarray(responsibilities, float)
            if (
                pair.shape != (n, n)
                or not np.isfinite(pair).all()
                or np.any(pair < 0)
                or np.any(pair > 1)
                or not np.allclose(pair + pair.T, 1, atol=1e-6, rtol=0)
            ):
                raise ValueError(
                    "Pedestrian responsibility shares must be complementary and in [0, 1]"
                )
            shares = np.full((sum(counts), sum(counts)), 0.5)
            shares[:n, :n] = pair
            shares[:n, n:] = 1
            shares[n:, :n] = 0
            shares = np.ascontiguousarray(shares, dtype=np.float64)
        self.lib.crowd_rvo_responsibilities(
            self.handle,
            sum(counts),
            shares.ctypes.data_as(ctypes.POINTER(ctypes.c_double)) if shares is not None else None,
        )
        p = np.ascontiguousarray(
            [list(x[:2]) for x in positions] + [list(x[0][:2]) for x in robots], dtype=np.float64
        )
        v = np.ascontiguousarray(
            [list(x[:2]) for x in velocities] + [list(x[1][:2]) for x in robots], dtype=np.float64
        )
        pref = np.ascontiguousarray(list(preferred) + [x[1] for x in robots], dtype=np.float64)
        buffers = self.stiffness.buffers if self.stiffness is not None else None
        radius = np.ascontiguousarray(
            interaction_radii(radii, robots, cfg, buffers), dtype=np.float64
        )
        maximum = np.ascontiguousarray(
            [max(float(s), float(np.linalg.norm(u)), 0.4) for s, u in zip(speeds, velocities)]
            + [max(float(np.linalg.norm(x[1])), 0.01) for x in robots],
            dtype=np.float64,
        )
        output = np.empty_like(p)
        feasible = np.empty(n + len(robots), dtype=np.int32)
        self.lib.crowd_rvo_step(
            self.handle, len(p), dt, p, v, pref, radius, maximum, output, feasible
        )
        # Mark violations; do not silently project ORCA output into a new velocity.
        valid = np.array(
            [
                self.context.motion_allowed(
                    x[:2], x[:2] + u * max(dt, 0.12), radius, cfg["safety_margin_m"]
                )
                for x, u, radius in zip(positions, output[:n], radii)
            ]
        )
        return output[:n].copy(), feasible[:n].astype(bool) & valid

    def close(self):
        if self.handle:
            self.lib.crowd_rvo_destroy(self.handle)
            self.handle = None

    def agent_parameters(self, index):
        """Read actual upstream state for audits, not a copy of Python settings."""
        if not self.handle or self.counts is None or not 0 <= index < sum(self.counts):
            raise ValueError("ORCA agent must exist before reading its parameters")
        values = np.empty(3, dtype=np.float64)
        self.lib.crowd_rvo_parameters(self.handle, index, values)
        return dict(
            zip(("time_horizon_s", "effective_radius_m", "neighbor_distance_m"), map(float, values))
        )

    def agent_neighbors(self, index):
        """Read neighbors actually selected by the upstream kd-tree query."""
        if not self.handle or self.counts is None or not 0 <= index < sum(self.counts):
            raise ValueError("ORCA agent must exist before reading its neighbors")
        ids = np.empty(sum(self.counts), dtype=np.int32)
        count = self.lib.crowd_rvo_neighbors(self.handle, index, ids, len(ids))
        return ids[:count].tolist()

    def agent_responsibility(self, index, other):
        """Actual native ORCA line share; None when the neighbor is not detected."""
        if (
            not self.handle
            or self.counts is None
            or not 0 <= index < sum(self.counts)
            or not 0 <= other < sum(self.counts)
        ):
            raise ValueError("ORCA agents must exist before reading their responsibility")
        result = float(self.lib.crowd_rvo_responsibility(self.handle, index, other))
        return result if result >= 0 else None

    def project_velocity(self, index, position, desired):
        """Closest executable velocity under the current upstream ORCA lines."""
        objective = np.ascontiguousarray(desired, dtype=np.float64)
        result = np.empty(2, dtype=np.float64)
        if not self.lib.crowd_rvo_query(self.handle, index, objective, 0, result):
            return None
        if not self.context.motion_allowed(
            position[:2],
            position[:2] + result * 0.12,
            self.radii[index],
            self.config["safety_margin_m"],
        ):
            return None
        return result

    def alternatives(self, index, position, preferred):
        """Query upstream LP on the last observation's ORCA region.

        Maximize forward progress exactly, then probe left/right intent through
        the same official LP. This is read-only: no simulation steps or forecasts.
        Each returned velocity must also stay in the NavMesh center mask.
        """
        norm = float(np.linalg.norm(preferred))
        if norm < 1e-9:
            return []
        forward = np.asarray(preferred, float) / norm
        candidates = []
        for angle in (0, -30, 30, -60, 60, -90, 90):
            radians = np.deg2rad(angle)
            c, s = np.cos(radians), np.sin(radians)
            direction = np.array([[c, -s], [s, c]]) @ forward
            for maximize in (True, False):
                objective = np.ascontiguousarray(direction if maximize else direction * norm)
                result = np.empty(2, dtype=np.float64)
                valid = self.lib.crowd_rvo_query(
                    self.handle, index, objective, int(maximize), result
                )
                if valid and self.context.motion_allowed(
                    position[:2],
                    position[:2] + result * 0.12,
                    self.radii[index],
                    self.config["safety_margin_m"],
                ):
                    candidates.append(result)
        return candidates

    def __del__(self):
        if getattr(self, "handle", None):
            self.close()
