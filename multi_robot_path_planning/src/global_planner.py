"""
Global planner: A* on an occupancy grid built from the pillars, then
shortcut pruning, Chaikin smoothing and resampling. ReferencePath turns the
smoothed path into the [x, y, psi, v] reference the NMPC tracks.
"""

import heapq
import math

import numpy as np

from config import N, dt, x_dim, V_CRUISE, A_REF_BRAKE


class OccupancyGrid:
    def __init__(self, bounds, resolution, pillars, clearance):
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.res = resolution
        self.nx = int(np.floor((self.x_max - self.x_min) / resolution)) + 1
        self.ny = int(np.floor((self.y_max - self.y_min) / resolution)) + 1

        xs = self.x_min + np.arange(self.nx) * resolution
        ys = self.y_min + np.arange(self.ny) * resolution
        XX, YY = np.meshgrid(xs, ys, indexing="ij")

        self.occ = np.zeros((self.nx, self.ny), dtype=bool)
        for px, py, pr in pillars:
            self.occ |= (XX - px) ** 2 + (YY - py) ** 2 <= (pr + clearance) ** 2

    def to_cell(self, x, y):
        return (int(round((x - self.x_min) / self.res)),
                int(round((y - self.y_min) / self.res)))

    def to_world(self, i, j):
        return np.array([self.x_min + i * self.res, self.y_min + j * self.res])

    def is_free(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny and not self.occ[i, j]

    def point_free(self, p):
        return self.is_free(*self.to_cell(p[0], p[1]))

    def segment_free(self, p, q):
        n = max(2, int(np.ceil(np.linalg.norm(q - p) / (0.5 * self.res))) + 1)
        return all(self.point_free(p + t * (q - p)) for t in np.linspace(0.0, 1.0, n))


def astar(grid, start_xy, goal_xy):
    """8-connected A* with Euclidean heuristic and no corner cutting."""
    start = grid.to_cell(*start_xy)
    goal = grid.to_cell(*goal_xy)

    if not grid.is_free(*start):
        raise ValueError(f"A*: start {start_xy} is inside an inflated obstacle or off the map")
    if not grid.is_free(*goal):
        raise ValueError(f"A*: goal {goal_xy} is inside an inflated obstacle or off the map")

    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
             (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]

    def h(c):
        return grid.res * math.hypot(c[0] - goal[0], c[1] - goal[1])

    g = {start: 0.0}
    parent = {start: None}
    open_heap = [(h(start), 0.0, start)]
    closed = set()

    while open_heap:
        _, g_cur, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:
            cells = []
            while cur is not None:
                cells.append(cur)
                cur = parent[cur]
            pts = np.array([grid.to_world(i, j) for i, j in reversed(cells)])
            pts[0], pts[-1] = start_xy, goal_xy
            return pts
        closed.add(cur)

        for di, dj, step_cost in moves:
            nb = (cur[0] + di, cur[1] + dj)
            if nb in closed or not grid.is_free(*nb):
                continue
            if di != 0 and dj != 0 and not (grid.is_free(cur[0] + di, cur[1])
                                            and grid.is_free(cur[0], cur[1] + dj)):
                continue
            g_new = g_cur + step_cost * grid.res
            if g_new < g.get(nb, math.inf):
                g[nb] = g_new
                parent[nb] = cur
                heapq.heappush(open_heap, (g_new + h(nb), g_new, nb))

    raise RuntimeError(f"A*: no path from {start_xy} to {goal_xy}")


def shortcut_path(grid, pts):
    """Line-of-sight pruning: keep only the waypoints A* really needs."""
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not grid.segment_free(pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return np.array(out)


def chaikin(pts, iterations):
    """Corner cutting: rounds the corners left by the shortcut step."""
    for _ in range(iterations):
        new = [pts[0]]
        for p, q in zip(pts[:-1], pts[1:]):
            new.append(0.75 * p + 0.25 * q)
            new.append(0.25 * p + 0.75 * q)
        new.append(pts[-1])
        pts = np.array(new)
    return pts


def resample(pts, ds):
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    s_new = np.arange(0.0, s[-1], ds)
    s_new = np.append(s_new, s[-1])
    return np.column_stack([np.interp(s_new, s, pts[:, 0]), np.interp(s_new, s, pts[:, 1])])


def smooth_path(raw, plan_grid, check_grid, ds=0.05):
    """Shortcut, then the strongest Chaikin smoothing that stays collision-free."""
    sparse = shortcut_path(plan_grid, raw)
    for iterations in (4, 3, 2, 1, 0):
        smooth = chaikin(sparse, iterations)
        if all(check_grid.segment_free(p, q) for p, q in zip(smooth[:-1], smooth[1:])):
            return resample(smooth, ds), sparse
    return resample(sparse, ds), sparse


class ReferencePath:
    """Turns the A* path into an NMPC reference [x, y, psi, v] over the horizon."""

    def __init__(self, pts):
        self.pts = pts
        seg = np.diff(pts, axis=0)
        self.s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(seg, axis=1))])
        heading = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
        self.psi = np.append(heading, heading[-1])
        self.length = self.s[-1]
        self.idx = 0

    def project(self, xy):
        """Arc length of the closest path point, searched in a window around the last one."""
        lo = max(0, self.idx - 10)
        hi = min(len(self.pts), self.idx + 120)
        d = np.linalg.norm(self.pts[lo:hi] - xy, axis=1)
        self.idx = lo + int(np.argmin(d))
        return self.s[self.idx]

    def speed(self, s):
        return min(V_CRUISE, math.sqrt(2.0 * A_REF_BRAKE * max(self.length - s, 0.0)))

    def reference(self, state):
        s = self.project(state[:2])
        X_ref = np.zeros((x_dim, N + 1))
        for k in range(N + 1):
            v_ref = self.speed(s)
            X_ref[:, k] = [np.interp(s, self.s, self.pts[:, 0]),
                           np.interp(s, self.s, self.pts[:, 1]),
                           np.interp(s, self.s, self.psi),
                           v_ref]
            s = min(s + v_ref * dt, self.length)

        # Put the reference heading in the same 2*pi branch as the car heading
        X_ref[2, :] += 2.0 * np.pi * np.round((state[2] - X_ref[2, 0]) / (2.0 * np.pi))
        return X_ref
