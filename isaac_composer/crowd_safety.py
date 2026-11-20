"""NavMesh-checked pedestrian passing, yielding, and emergency stopping."""

import heapq

import numpy as np


def _remaining(route, position):
    """Return the forward suffix at the nearest point on a route polyline."""
    route = np.asarray(route, dtype=float).reshape(-1, 3)
    position = np.asarray(position, dtype=float)
    if len(route) < 2:
        return np.asarray([position, route[-1]])
    delta = route[1:, :2] - route[:-1, :2]
    length2 = np.sum(delta * delta, axis=1)
    alpha = np.clip(np.sum((position[:2] - route[:-1, :2]) * delta, axis=1)
                    / np.maximum(length2, 1e-12), 0, 1)
    projected = route[:-1] + alpha[:, None] * (route[1:] - route[:-1])
    index = int(np.argmin(np.linalg.norm(projected[:, :2] - position[:2], axis=1)))
    return np.vstack([position, projected[index], route[index + 1:]])


def _ahead(route, position, distance):
    remaining = _remaining(route, position)
    for start, end in zip(remaining[:-1], remaining[1:]):
        length = np.linalg.norm(end[:2] - start[:2])
        if length > distance:
            return start + (end - start) * distance / length
        distance -= length
    return remaining[-1]


def _point_path_distance(point, path):
    path = np.asarray(path, dtype=float)
    delta = path[1:, :2] - path[:-1, :2]
    length2 = np.sum(delta * delta, axis=1)
    alpha = np.clip(np.sum((point[:2] - path[:-1, :2]) * delta, axis=1)
                    / np.maximum(length2, 1e-12), 0, 1)
    closest = path[:-1, :2] + alpha[:, None] * delta
    return float(np.min(np.linalg.norm(closest - point[:2], axis=1)))


class CrowdNavigator:
    """Reserve a passing encounter so only one pedestrian proceeds at a time.

    Try the priority pedestrian's original route, then a lateral bypass. If a
    narrow corridor prevents passing, move the yielding pedestrian to a nearby
    walkable waiting place. All temporary paths are checked on the same NavMesh
    and map as the original routes. Emergency stopping remains a final guard.
    """

    def __init__(self, scene, horizon=1.0, margin=0.2):
        self.scene = scene
        self.horizon = horizon
        self.margin = margin
        self.time = 0.0
        self.encounters = []
        self.extra_yielders = {}
        self.local_paths = {}
        self.wait_credit = None
        self.retry_after = {}
        self.events = []
        self.states = []
        self.grid = None

    def _grid_bypass(self, mover, positions, routes, radii):
        """Find a wider route when simple side turns cannot clear a crowd."""
        if not hasattr(self.scene, 'rasterize'):
            return None
        if self.grid is None:
            self.grid = self.scene.rasterize(margin_m=0)
        grid = self.grid
        height, width = grid.mask.shape
        bounds = grid.bounds
        xs = bounds.x_min + (np.arange(width) + .5) / width * (bounds.x_max - bounds.x_min)
        ys = bounds.y_max - (np.arange(height) + .5) / height * (bounds.y_max - bounds.y_min)
        mask = grid.mask.copy()
        for other, point in enumerate(positions):
            if other == mover:
                continue
            required = radii[mover] + radii[other] + self.margin + .3
            required = min(required, max(.01, np.linalg.norm(point[:2] - positions[mover, :2]) - .15))
            mask &= (xs[None, :] - point[0]) ** 2 + (ys[:, None] - point[1]) ** 2 >= required ** 2

        def node(point):
            col = int((point[0] - bounds.x_min) / (bounds.x_max - bounds.x_min) * width)
            row = int((bounds.y_max - point[1]) / (bounds.y_max - bounds.y_min) * height)
            nearby = [(r, c) for r in range(max(0, row - 2), min(height, row + 3))
                      for c in range(max(0, col - 2), min(width, col + 3)) if mask[r, c]]
            if not nearby:
                return None
            r, c = min(nearby, key=lambda rc: (xs[rc[1]] - point[0]) ** 2 + (ys[rc[0]] - point[1]) ** 2)
            return (r, c) if np.hypot(xs[c] - point[0], ys[r] - point[1]) <= .3 else None

        start = node(positions[mover])
        goal_point = np.asarray(routes[mover][-1], dtype=float)
        goal = node(goal_point)
        if start is None or goal is None:
            return None
        costs = np.full(mask.shape, np.inf)
        costs[start] = 0.0
        parents = {}
        queue = [(0.0, 0.0, start)]
        found = False
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost > costs[current]:
                continue
            if current == goal:
                found = True
                break
            r, c = current
            for dr, dc in ((-1, 0), (0, -1), (0, 1), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)):
                nr, nc = r + dr, c + dc
                if not (0 <= nr < height and 0 <= nc < width and mask[nr, nc]):
                    continue
                if dr and dc and (not mask[r, nc] or not mask[nr, c]):
                    continue
                next_cost = cost + np.hypot(dr, dc)
                if next_cost < costs[nr, nc]:
                    costs[nr, nc] = next_cost
                    parents[nr, nc] = current
                    heuristic = np.hypot(goal[0] - nr, goal[1] - nc)
                    heapq.heappush(queue, (next_cost + heuristic, next_cost, (nr, nc)))
        if not found:
            return None
        cells = [goal]
        while cells[-1] != start:
            cells.append(parents[cells[-1]])
        cells.reverse()

        def line_clear(a, b):
            count = max(2, 2 * max(abs(a[0] - b[0]), abs(a[1] - b[1])) + 1)
            rc = np.linspace(a, b, count).round().astype(int)
            return bool(mask[rc[:, 0], rc[:, 1]].all())

        # Keep only visible bends, then have native NavMesh validate every leg.
        kept = [cells[0]]
        index = 0
        while index < len(cells) - 1:
            target = min(index + 40, len(cells) - 1)
            while target > index + 1 and not line_clear(cells[index], cells[target]):
                target -= 1
            kept.append(cells[target])
            index = target
        points = [positions[mover], *[np.array(grid.pixel_to_world(r, c)) for r, c in kept], goal_point]
        return self._path(points, mover, positions, radii)

    def _event(self, action, winner, yielder):
        self.events.append({'time_sec': round(self.time, 3), 'action': action,
                            'priority_person': int(winner), 'yielding_person': int(yielder)})

    def _clear(self, path, mover, positions, radii):
        for other, point in enumerate(positions):
            if other == mover:
                continue
            required = radii[mover] + radii[other] + self.margin + 0.15
            # Permit moving away when already closer than the planning buffer.
            required = min(required, np.linalg.norm(path[0][:2] - point[:2]))
            if _point_path_distance(point, path) < required - 1e-5:
                return False
        return True

    def _path(self, points, mover, positions, radii):
        result = []
        try:
            for point in points:
                projected = np.asarray(self.scene.project_world(point), dtype=float)
                if np.linalg.norm(projected[:2] - point[:2]) > 0.15:
                    return None
                if not result:
                    result.append(projected)
                else:
                    if np.linalg.norm(result[-1][:2] - projected[:2]) <= 1e-6:
                        continue
                    part = self.scene.shortest_path(result[-1], projected)
                    result.extend(np.asarray(part, dtype=float)[1:])
        except RuntimeError:
            return None
        path = np.asarray(result)
        return path if len(path) >= 2 and self._clear(path, mover, positions, radii) else None

    def _bypass(self, mover, blocker, positions, routes, radii, forward):
        side = np.asarray([forward[1], -forward[0], 0.0])
        center = positions[blocker]
        clearance = radii[mover] + radii[blocker] + self.margin + 0.5
        # Rejoin beyond the waiting person, retaining the original destination.
        on_route = _remaining(routes[mover], center)[1]
        join = _ahead(routes[mover], on_route, clearance + 0.7)
        if np.dot(join[:2] - center[:2], forward[:2]) < 0.6:
            return self._grid_bypass(mover, positions, routes, radii)
        for sign in (1, -1):
            for scale in (1.0, 1.4, 2.0, 2.6):
                offset = side * clearance * scale * sign
                before = center - forward * 1.1 + offset
                after = center + forward * 1.1 + offset
                for retreat in (0.0, 1.5, 3.0):
                    # A dense group may prevent an immediate lateral turn.
                    # Back out first, then pass around the whole waiting group.
                    back = positions[mover] - forward * retreat
                    points = ([positions[mover], before, after, join] if retreat == 0
                              else [positions[mover], back, back + offset, after, join])
                    prefix = self._path(points, mover, positions, radii)
                    if prefix is not None:
                        return np.vstack([prefix, _remaining(routes[mover], join)[1:]])
        return self._grid_bypass(mover, positions, routes, radii)

    def _waiting_place(self, yielder, winner, positions, routes, radii, direction):
        side = np.asarray([direction[1], -direction[0], 0.0])
        clearance = radii[yielder] + radii[winner] + self.margin + 0.5
        winning_path = _remaining(routes[winner], positions[winner])
        # Search behind the yielding person as well, allowing retreat to a bay.
        for back in (0.0, 1.5, 3.0, 5.0, 8.0):
            for sign in (1, -1):
                point = positions[yielder] - direction * back + side * clearance * sign
                if _point_path_distance(point, winning_path) < clearance - 0.05:
                    continue
                path = self._path([positions[yielder], point], yielder, positions, radii)
                if path is not None:
                    return path
        return None

    def _velocities(self, positions, paths, speeds):
        velocities = []
        for position, path, speed in zip(positions, paths, speeds):
            target = _ahead(path, position, max(0.7, speed * self.horizon))
            delta = target[:2] - position[:2]
            distance = np.linalg.norm(delta)
            # Prediction ends at the destination; a person will not keep moving
            # through it for the rest of the horizon. Keep command speed separate.
            predicted_speed = min(speed, distance / self.horizon)
            velocities.append(delta / distance * predicted_speed if distance > 1e-6 else np.zeros(2))
        return np.asarray(velocities)

    def step(self, dt, positions, routes, speeds, radii):
        self.time += float(dt)
        positions = np.asarray(positions, dtype=float)
        speeds = np.asarray(speeds, dtype=float).copy()
        radii = np.asarray(radii, dtype=float)
        if self.wait_credit is None:
            self.wait_credit = np.zeros(len(positions))
        # A temporary route ends at the original goal (or at a waiting place).
        occupied = {item[key] for item in self.encounters for key in ('winner', 'yielder')}
        occupied.update(self.extra_yielders)
        for index, path in list(self.local_paths.items()):
            changed_goal = np.linalg.norm(np.asarray(routes[index])[-1, :2] - path[-1, :2]) > 0.3
            if index not in occupied and (changed_goal or np.linalg.norm(positions[index, :2] - path[-1, :2]) <= 0.3):
                del self.local_paths[index]
        paths = [self.local_paths.get(i, np.asarray(route, dtype=float)) for i, route in enumerate(routes)]
        states = ['walking' if speed > 0 else 'idle' for speed in speeds]

        for item in list(self.encounters):
            winner, yielder = item['winner'], item['yielder']
            changed_goal = not np.allclose(np.asarray(routes[winner])[-1], item['winner_goal'])
            if changed_goal:
                # Repetition can reverse a route before a parked person is a
                # full meter behind the winner. Retire the old passing route.
                self.local_paths.pop(winner, None)
                paths[winner] = np.asarray(routes[winner], dtype=float)
            if item['phase'] == 'parking' and not changed_goal:
                if np.linalg.norm(positions[yielder, :2] - item['waiting_point'][:2]) > 0.3:
                    speeds[winner] = 0.0
                    states[winner], states[yielder] = 'waiting_for_space', 'stepping_aside'
                    continue
                item['phase'] = 'passing'
                self._event('waiting_place_reached', winner, yielder)
            separation = np.linalg.norm(positions[winner, :2] - positions[yielder, :2])
            passed = np.dot(positions[winner, :2] - positions[yielder, :2], item['forward'][:2]) > 1.0
            stopped = speeds[winner] <= 0.0 or changed_goal
            if separation > radii[winner] + radii[yielder] + self.margin + 0.5 and (passed or stopped):
                if item.get('waiting_point') is not None:
                    if self.time < item.get('rejoin_retry_after', 0.0):
                        speeds[yielder] = 0.0
                        states[yielder] = 'waiting_to_rejoin'
                        continue
                    on_route = _remaining(routes[yielder], positions[yielder])[1]
                    join = _ahead(routes[yielder], on_route, 1.8)
                    prefix = self._path([positions[yielder], join], yielder, positions, radii)
                    if prefix is None:
                        item['rejoin_retry_after'] = self.time + 0.5
                        speeds[yielder] = 0.0
                        states[yielder] = 'waiting_to_rejoin'
                        continue
                    self.local_paths[yielder] = np.vstack([prefix, _remaining(routes[yielder], join)[1:]])
                    paths[yielder] = self.local_paths[yielder]
                self.encounters.remove(item)
                self._event('resume', winner, yielder)
                continue
            speeds[yielder] = 0.0
            states[winner], states[yielder] = 'passing', 'yielding'

        for yielder, item in list(self.extra_yielders.items()):
            winner = item['winner']
            separation = np.linalg.norm(positions[winner, :2] - positions[yielder, :2])
            passed = np.dot(positions[winner, :2] - positions[yielder, :2], item['forward'][:2]) > 1.0
            changed_goal = not np.allclose(np.asarray(routes[winner])[-1], item['winner_goal'])
            if separation > radii[winner] + radii[yielder] + self.margin + 0.5 and (passed or speeds[winner] <= 0 or changed_goal):
                del self.extra_yielders[yielder]
                self._event('resume', winner, yielder)
            else:
                speeds[yielder] = 0.0
                states[yielder] = 'yielding'
        occupied = {item[key] for item in self.encounters for key in ('winner', 'yielder')}
        occupied.update(self.extra_yielders)
        occupied.update(item['winner'] for item in self.extra_yielders.values())
        velocities = self._velocities(positions, paths, speeds)
        conflicts = []
        for i in range(len(positions)):
            for j in range(i):
                delta = positions[i, :2] - positions[j, :2]
                velocity = velocities[i] - velocities[j]
                vv = float(velocity @ velocity)
                if vv <= 1e-12 or float(delta @ velocity) >= 0:
                    continue
                time = min(self.horizon, -float(delta @ velocity) / vv)
                required = radii[i] + radii[j] + self.margin
                if np.linalg.norm(delta + velocity * time) < required:
                    conflicts.append((float(np.linalg.norm(delta)), j, i))
        for _, a, b in sorted(conflicts):
            key = (a, b)
            if self.time < self.retry_after.get(key, 0):
                continue
            velocities = self._velocities(positions, paths, speeds)
            delta = positions[a, :2] - positions[b, :2]
            relative = velocities[a] - velocities[b]
            vv = float(relative @ relative)
            if vv <= 1e-12 or float(delta @ relative) >= 0:
                continue
            predicted = delta + relative * min(self.horizon, -float(delta @ relative) / vv)
            if np.linalg.norm(predicted) >= radii[a] + radii[b] + self.margin:
                continue
            moving = [i for i in (a, b) if np.linalg.norm(velocities[i]) > 1e-6]
            if a in occupied or b in occupied:
                # A third person must yield to an ongoing encounter. Otherwise
                # the emergency guard can stop the reserved pair indefinitely.
                reserved = [i for i in moving if i in occupied]
                winner = min(reserved or moving, key=lambda i: (-self.wait_credit[i], i))
                yielder = b if winner == a else a
                if any(item['winner'] == winner and item['yielder'] == yielder for item in self.encounters):
                    continue
                direction = np.r_[velocities[winner] / np.linalg.norm(velocities[winner]), 0.0]
                required = radii[winner] + radii[yielder] + self.margin + 0.15
                clear = _point_path_distance(positions[yielder], _remaining(paths[winner], positions[winner])) >= required
                bypass = None if clear else self._bypass(winner, yielder, positions, paths, radii, direction)
                if not clear and bypass is None:
                    self.retry_after[key] = self.time + 0.5
                    continue
                if bypass is not None:
                    self.local_paths[winner] = paths[winner] = bypass
                if speeds[yielder] > 0:
                    self.extra_yielders[yielder] = {'winner': winner, 'forward': direction,
                                                   'winner_goal': np.asarray(routes[winner])[-1].copy()}
                    speeds[yielder] = 0.0
                    states[yielder] = 'yielding'
                    self._event('yield_to_passing', winner, yielder)
                states[winner] = 'passing'
                occupied.update((winner, yielder))
                continue
            winner = min(moving, key=lambda i: (-self.wait_credit[i], i))
            yielder = b if winner == a else a
            direction = velocities[winner]
            direction = np.r_[direction / np.linalg.norm(direction), 0.0]
            required = radii[winner] + radii[yielder] + self.margin + 0.3
            clear = _point_path_distance(positions[yielder], _remaining(paths[winner], positions[winner])) >= required
            bypass = None if clear else self._bypass(winner, yielder, positions, paths, radii, direction)
            waiting = None
            if not clear and bypass is None and speeds[yielder] > 0:
                desired = velocities[yielder]
                desired = np.r_[desired / max(np.linalg.norm(desired), 1e-6), 0.0]
                waiting = self._waiting_place(yielder, winner, positions, paths, radii, desired)
            if not clear and bypass is None and waiting is None:
                self.retry_after[key] = self.time + 0.5
                states[a] = states[b] = 'blocked_no_space'
                continue
            item = {'winner': winner, 'yielder': yielder, 'forward': direction,
                    'winner_goal': np.asarray(routes[winner])[-1].copy(),
                    'phase': 'parking' if waiting is not None else 'passing',
                    'waiting_point': waiting[-1] if waiting is not None else None}
            self.encounters.append(item)
            occupied.update((winner, yielder))
            if waiting is not None:
                self.local_paths[yielder] = paths[yielder] = waiting
                speeds[winner] = 0.0
                states[winner], states[yielder] = 'waiting_for_space', 'stepping_aside'
                self._event('step_aside', winner, yielder)
            else:
                if bypass is not None:
                    self.local_paths[winner] = paths[winner] = bypass
                speeds[yielder] = 0.0
                states[winner], states[yielder] = 'passing', 'yielding'
                self._event('bypass' if bypass is not None else 'yield', winner, yielder)

        desired = self._velocities(positions, paths, speeds)
        safe = yielding_velocities(positions, desired, radii, self.horizon, self.margin)
        output = np.where(np.linalg.norm(safe, axis=1) > 1e-6, speeds, 0.0)
        for i, (command, requested) in enumerate(zip(output, speeds)):
            if command <= 1e-6 and requested > 0:
                states[i] = 'safety_stop' if states[i] != 'blocked_no_space' else states[i]
            if output[i] <= 1e-6 and requested >= 0:
                self.wait_credit[i] += dt
            else:
                self.wait_credit[i] = max(0.0, self.wait_credit[i] - dt)
        self.states = states
        return output, paths

    def diagnostics(self):
        return {'states': self.states, 'events': self.events}


def yielding_velocities(positions, velocities, radii, horizon=1.0, margin=0.20):
    """Stop conflicting motions, including conflicts caused by another agent stopping.

    Use closest approach over the entire prediction interval, not only its endpoint.
    Narrow head-on encounters can stop both agents; safety takes priority over progress.
    """
    positions = np.asarray(positions, dtype=float)[:, :2]
    result = np.asarray(velocities, dtype=float)[:, :2].copy()
    radii = np.asarray(radii, dtype=float)
    for _ in range(len(result) + 1):
        changed = False
        for i in range(len(result)):
            for j in range(i):
                relative_pos = positions[i] - positions[j]
                relative_vel = result[i] - result[j]
                vv = float(relative_vel @ relative_vel)
                if vv < 1e-12:
                    continue
                approaching = float(relative_pos @ relative_vel)
                if approaching >= 0:
                    continue
                t = min(horizon, -approaching / vv)
                closest = np.linalg.norm(relative_pos + relative_vel * t)
                if closest < radii[i] + radii[j] + margin:
                    for k in (i, j):
                        if np.any(result[k]):
                            result[k] = 0.0
                            changed = True
        if not changed:
            break
    return result
