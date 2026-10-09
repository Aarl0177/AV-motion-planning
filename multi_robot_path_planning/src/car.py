"""
Car agent: holds its state, its A* global plan and its NMPC controller, and
broadcasts its state (and predicted plan) to the other cars over V2V.
"""

import numpy as np

from config import N, dt, u_dim, GOAL_TOL, PARK_DECEL, SHARE_PLANS
from dynamics import dynamics
from global_planner import astar, smooth_path, ReferencePath


class Car:
    def __init__(self, car_id, start, goal, color, plan_grid, check_grid):
        self.id = car_id
        self.state = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.color = color

        # Global plan (A*), computed once
        self.raw_path = astar(plan_grid, self.state[:2], self.goal)
        self.path, self.waypoints = smooth_path(self.raw_path, plan_grid, check_grid)
        self.ref = ReferencePath(self.path)

        self.controller = None        # set once the number of obstacle slots is known
        self.plan = None              # latest NMPC predicted trajectory (x_dim, N+1)
        self.u_prev = np.zeros(u_dim)
        self.arrived = False
        self.arrival_time = None
        self.solver_failures = 0

    def broadcast(self):
        """Message every other car receives at the next step."""
        return {
            "id": self.id,
            "state": self.state.copy(),
            "plan": None if (not SHARE_PLANS or self.plan is None) else self.plan.copy(),
        }

    def compute_control(self, obs_x, obs_y, obs_r):
        if self.arrived:
            # Parked: brake gently to a stop and tell the others we are standing still
            self.plan = np.tile(self.state.reshape(-1, 1), (1, N + 1))
            self.plan[3, :] = 0.0
            return np.array([max(-PARK_DECEL, -self.state[3] / dt), 0.0])

        x_ref = self.ref.reference(self.state)
        u, plan, ok = self.controller.solve(self.state, x_ref, obs_x, obs_y, obs_r, self.u_prev)
        if not ok:
            self.solver_failures += 1
        self.plan = plan
        return u

    def apply(self, u, t):
        self.u_prev = np.asarray(u, dtype=float).copy()
        self.state = dynamics(self.state, u)
        self.state[3] = max(self.state[3], 0.0)    # remove tiny negative speeds from solver tolerance

        if not self.arrived and np.linalg.norm(self.state[:2] - self.goal) < GOAL_TOL:
            self.arrived = True
            self.arrival_time = t
