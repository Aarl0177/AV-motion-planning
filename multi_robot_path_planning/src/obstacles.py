"""
Obstacle motion and prediction, and packing everything a car must avoid
into the fixed-size parameter arrays the NMPC expects:
    - other cars        -> moving obstacles, predicted from their V2V messages
    - dynamic obstacles -> moving obstacles, constant-velocity prediction
    - pillars           -> static obstacles
"""

import numpy as np

from config import N, dt, vehicle_radius, obs_radius, margin, car_margin
from scenarios import PILLARS


def obstacle_dynamics_all(obs_states, dt):
    """Constant-velocity update. obs_states[j] = [ox, oy, ovx, ovy]."""
    obs_next = obs_states.copy()
    obs_next[:, 0] = obs_states[:, 0] + dt * obs_states[:, 2]
    obs_next[:, 1] = obs_states[:, 1] + dt * obs_states[:, 3]
    return obs_next


def predict_obstacle(obs_state):
    """Constant-velocity prediction over the horizon, shape (2, N+1)."""
    ox, oy, ovx, ovy = obs_state
    k = np.arange(N + 1)
    return np.vstack([ox + ovx * dt * k, oy + ovy * dt * k])


def predict_from_message(msg):
    """
    Predicted (x, y) of another car over this car's horizon, shape (2, N+1).

    The shared plan was computed one step ago, so plan[:, k+1] is where that car
    expects to be k steps from now. Index 0 uses the freshly shared position.
    Without a plan, fall back to constant velocity along the shared heading.
    """
    x, y, psi, v = msg["state"]
    plan = msg["plan"]
    pred = np.zeros((2, N + 1))

    if plan is not None:
        pred[:, :N] = plan[:2, 1:]
        xN, yN, psiN, vN = plan[:, N]
        pred[:, N] = [xN + dt * vN * np.cos(psiN), yN + dt * vN * np.sin(psiN)]
        pred[:, 0] = [x, y]
    else:
        k = np.arange(N + 1)
        pred[0] = x + v * np.cos(psi) * dt * k
        pred[1] = y + v * np.sin(psi) * dt * k

    return pred


def build_obstacle_slots(car, inbox, dyn_obs, n_slots):
    """Stack every obstacle this car must avoid into fixed-size parameter arrays."""
    xs, ys, rs = [], [], []

    # Other cars -> moving obstacles, from the shared messages
    for msg in inbox:
        if msg["id"] == car.id:
            continue
        pred = predict_from_message(msg)
        xs.append(pred[0])
        ys.append(pred[1])
        rs.append(2.0 * vehicle_radius + car_margin)

    # Non-communicating moving obstacles -> constant-velocity prediction
    for ob in dyn_obs:
        pred = predict_obstacle(ob)
        xs.append(pred[0])
        ys.append(pred[1])
        rs.append(obs_radius + vehicle_radius + margin)

    # Static pillars
    for px, py, pr in PILLARS:
        xs.append(np.full(N + 1, px))
        ys.append(np.full(N + 1, py))
        rs.append(pr + vehicle_radius + margin)

    # Pad unused slots with far-away, zero-radius dummies
    while len(xs) < n_slots:
        xs.append(np.full(N + 1, 1e4))
        ys.append(np.full(N + 1, 1e4))
        rs.append(0.0)

    return np.array(xs), np.array(ys), np.array(rs)
