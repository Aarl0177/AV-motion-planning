"""
Safety metrics computed from the simulation history, and the printed summary.
"""

import numpy as np

from config import vehicle_radius, obs_radius, margin, car_margin
from scenarios import PILLARS


def compute_clearance(x_hist, dyn_hist):
    """
    Surface-to-surface clearance from each car to its nearest other car,
    dynamic obstacle or pillar (0 = contact). Shape (T+1, n_cars).
    """
    T, n_cars, _ = x_hist.shape
    clr = np.full((T, n_cars), np.inf)
    for t in range(T):
        for i in range(n_cars):
            p = x_hist[t, i, :2]
            for j in range(n_cars):
                if j != i:
                    clr[t, i] = min(clr[t, i], np.linalg.norm(p - x_hist[t, j, :2]) - 2 * vehicle_radius)
            for ob in dyn_hist[t]:
                clr[t, i] = min(clr[t, i], np.linalg.norm(p - ob[:2]) - obs_radius - vehicle_radius)
            for px, py, pr in PILLARS:
                clr[t, i] = min(clr[t, i], np.hypot(p[0] - px, p[1] - py) - pr - vehicle_radius)
    return clr


def car_to_car_gap(x_hist):
    """Surface-to-surface gap from each car to its nearest other car. Shape (T+1, n_cars)."""
    pos = x_hist[:, :, :2]
    d = np.linalg.norm(pos[:, :, None, :] - pos[:, None, :, :], axis=-1)
    n = pos.shape[1]
    d[:, np.arange(n), np.arange(n)] = np.inf
    return d.min(axis=2) - 2 * vehicle_radius


def print_summary(result):
    cars = result["cars"]
    clr = compute_clearance(result["x_hist"], result["dyn_hist"])
    gap = car_to_car_gap(result["x_hist"])
    st = result["solve_times"]

    print("\n------------- Summary -------------")
    for c in cars:
        arrival = f"{c.arrival_time:.1f} s" if c.arrived else "not reached"
        print(f"car {c.id}: goal {arrival}, min clearance {clr[:, c.id].min():.3f} m, "
              f"min gap to another car {gap[:, c.id].min():.3f} m, "
              f"solver failures {c.solver_failures}")
    print(f"NMPC solve time: mean {1e3 * st.mean():.1f} ms, max {1e3 * st.max():.1f} ms")
    print(f"(targets: {margin:.2f} m to obstacles, {car_margin:.2f} m between cars, contact = 0 m)")
    print("-----------------------------------\n")
