"""
Scenario: map, static pillars, cars (start / goal) and non-communicating
moving obstacles.
"""

import numpy as np


# A* map bounds [x_min, x_max, y_min, y_max] and grid resolution [m]
MAP_BOUNDS = (-2.0, 22.0, -4.0, 8.0)
GRID_RES = 0.10

# Static obstacles (pillars): [x, y, radius]
PILLARS = np.array([
    [6.0, -0.1, 0.40],     # blocks car 2's straight diagonal
    [15.0, 2.1, 0.35],     # splits car 0 (passes below) and car 1 (passes above)
    [4.0, 2.9, 0.30],      # pushes car 0 down and car 1 up near the left side
])

# Cars: start [x, y, psi, v], goal [x, y]
CARS = [
    dict(start=[0.0, 2.0, 0.0, 1.0],    goal=[20.0, 2.0], color=(0.10, 0.25, 0.90)),
    dict(start=[20.0, 2.6, np.pi, 1.0], goal=[0.0, 2.6],  color=(0.95, 0.55, 0.05)),
    dict(start=[2.0, -2.0, 0.45, 1.0],  goal=[18.0, 5.5], color=(0.15, 0.70, 0.30)),
]

# Non-communicating moving obstacles (e.g. pedestrians): [x, y, vx, vy]
DYN_OBS = np.array([
    [12.0, 7.0, -0.05, -0.30],
]).reshape(-1, 4)
