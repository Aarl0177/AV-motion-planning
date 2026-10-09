"""
Kinematic bicycle model, in CasADi (for the NMPC) and NumPy (for the simulation).
state = [x, y, psi, v], u = [a, delta]
"""

import numpy as np
import casadi as ca

from config import dt, L


def dynamics_casadi(state, u):
    """CasADi symbolic kinematic bicycle model. state = [x, y, psi, v], u = [a, delta]."""
    x, y, psi, v = state[0], state[1], state[2], state[3]
    a, delta = u[0], u[1]

    return ca.vertcat(
        x + dt * v * ca.cos(psi),
        y + dt * v * ca.sin(psi),
        psi + dt * v / L * ca.tan(delta),
        v + dt * a,
    )


def dynamics(state, u):
    """Numeric kinematic bicycle model."""
    x, y, psi, v = state
    a, delta = u

    return np.array([
        x + dt * v * np.cos(psi),
        y + dt * v * np.sin(psi),
        psi + dt * v / L * np.tan(delta),
        v + dt * a,
    ])
