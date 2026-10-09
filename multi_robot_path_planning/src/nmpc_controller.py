"""
Local planner: NMPC (CasADi + IPOPT), built once per car and solved with
parameters. Tracks the car's own A* path (error in the path frame) and keeps
soft distance constraints to every obstacle slot.
"""

import numpy as np
import casadi as ca

from config import (N, dt, x_dim, u_dim, Q, R, Qf, R_DU, SLACK_WEIGHT,
                    a_min, a_max, steer_min, steer_max, v_min, v_max)
from dynamics import dynamics_casadi


def path_frame_error(xk, rk):
    """Tracking error rotated into the reference frame: [longitudinal, lateral, heading, speed]."""
    ex = xk[0] - rk[0]
    ey = xk[1] - rk[1]
    c, s = ca.cos(rk[2]), ca.sin(rk[2])
    return ca.vertcat(c * ex + s * ey, -s * ex + c * ey, xk[2] - rk[2], xk[3] - rk[3])


class NMPCController:
    """
    The optimisation problem is built once; at every step only the parameters
    (initial state, reference, obstacle predictions, radii) change. This is much
    faster than rebuilding a new ca.Opti() every step and allows warm starting.
    """

    def __init__(self, n_slots):
        self.n_slots = n_slots
        opti = ca.Opti()

        X = opti.variable(x_dim, N + 1)
        U = opti.variable(u_dim, N)
        S = opti.variable(n_slots, N + 1)     # one slack per obstacle per step

        P_x0 = opti.parameter(x_dim)
        P_ref = opti.parameter(x_dim, N + 1)
        P_ox = opti.parameter(n_slots, N + 1)
        P_oy = opti.parameter(n_slots, N + 1)
        P_r = opti.parameter(n_slots)          # combined safe radius per obstacle
        P_u_prev = opti.parameter(u_dim)       # control applied at the previous step

        Qm, Rm, Qfm, Rdm = ca.DM(Q), ca.DM(R), ca.DM(Qf), ca.DM(R_DU)

        opti.subject_to(X[:, 0] == P_x0)
        opti.subject_to(ca.vec(S) >= 0)
        opti.subject_to(opti.bounded(v_min, X[3, 1:], v_max))

        cost = 0
        for k in range(N + 1):
            # Obstacle avoidance (other cars, dynamic obstacles, pillars) with slacks
            dist_sq = (X[0, k] - P_ox[:, k]) ** 2 + (X[1, k] - P_oy[:, k]) ** 2
            opti.subject_to(dist_sq >= P_r ** 2 - S[:, k])
            cost += SLACK_WEIGHT * ca.sumsqr(S[:, k])

            e = path_frame_error(X[:, k], P_ref[:, k])
            if k < N:
                u_k = U[:, k]
                opti.subject_to(X[:, k + 1] == dynamics_casadi(X[:, k], u_k))
                opti.subject_to(opti.bounded(a_min, u_k[0], a_max))
                opti.subject_to(opti.bounded(steer_min, u_k[1], steer_max))
                du = u_k - (P_u_prev if k == 0 else U[:, k - 1])
                cost += ca.bilin(Qm, e, e) + ca.bilin(Rm, u_k, u_k) + ca.bilin(Rdm, du, du)
            else:
                cost += ca.bilin(Qfm, e, e)

        opti.minimize(cost)
        opti.solver("ipopt", {
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "print_time": 0,
            "ipopt.max_iter": 1000,
            "ipopt.tol": 1e-4,
            "ipopt.acceptable_tol": 1e-3,
        })

        self.opti = opti
        self.X, self.U, self.S = X, U, S
        self.P_x0, self.P_ref, self.P_ox, self.P_oy, self.P_r = P_x0, P_ref, P_ox, P_oy, P_r
        self.P_u_prev = P_u_prev

        self.X_prev = None
        self.U_prev = None

    def solve(self, x0, x_ref, obs_x, obs_y, obs_r, u_prev):
        """Returns (u0, predicted state trajectory, success flag)."""
        opti = self.opti
        opti.set_value(self.P_x0, x0)
        opti.set_value(self.P_ref, x_ref)
        opti.set_value(self.P_ox, obs_x)
        opti.set_value(self.P_oy, obs_y)
        opti.set_value(self.P_r, obs_r)
        opti.set_value(self.P_u_prev, u_prev)

        # Warm start from the previous solution shifted by one step
        if self.X_prev is not None:
            X_init = np.hstack([self.X_prev[:, 1:], self.X_prev[:, -1:]])
            U_init = np.hstack([self.U_prev[:, 1:], self.U_prev[:, -1:]])
        else:
            X_init = x_ref.copy()
            U_init = np.zeros((u_dim, N))
        X_init[:, 0] = x0

        opti.set_initial(self.X, X_init)
        opti.set_initial(self.U, U_init)
        opti.set_initial(self.S, 0.0)

        try:
            sol = opti.solve()
            X_opt = np.asarray(sol.value(self.X)).reshape(x_dim, N + 1)
            U_opt = np.asarray(sol.value(self.U)).reshape(u_dim, N)
            self.X_prev, self.U_prev = X_opt, U_opt
            return U_opt[:, 0].copy(), X_opt, True

        except RuntimeError:
            print("  NMPC solver failed:", opti.debug.return_status())
            if self.U_prev is not None:
                # Fall back to the next control of the last good plan
                self.X_prev, self.U_prev = X_init, U_init
                return U_init[:, 0].copy(), X_init, False
            # No plan yet: brake gently, keep the wheels straight
            return np.array([max(a_min, -x0[3] / dt), 0.0]), X_init, False
