"""
Configuration: NMPC settings, vehicle limits, cost weights, safety margins,
communication mode and output settings.
"""

import numpy as np


# -----------------------------
# NMPC settings
# -----------------------------
N = 60              # prediction horizon [steps]
dt = 0.1            # control / simulation step [s]
MAX_STEPS = 350     # hard stop; the sim ends earlier once every car is parked

# Bicycle model wheelbase
L = 1.0

a_min, a_max = -2.0, 2.0
steer_min, steer_max = -np.pi / 10, np.pi / 10
v_min, v_max = 0.0, 2.5          # no reversing

V_CRUISE = 1.0      # reference speed along the A* path [m/s]
A_REF_BRAKE = 0.5   # deceleration used to bring v_ref to 0 at the goal [m/s^2]
GOAL_TOL = 0.30     # distance at which a car counts as arrived [m]
PARK_DECEL = 1.0    # braking used once a car has arrived [m/s^2]

# Tracking weights in the PATH frame: [longitudinal, lateral, heading, speed]
# (same numbers as before: the old x/y weights assumed a path along +x)
Q = np.diag([3, 30, 20, 2])
R = np.diag([5, 1])
Qf = np.diag([5, 5, 1, 1])
R_DU = np.diag([2, 50])     # input-rate penalty [Δa, Δδ]: stops the car from flipping
                            # the side it passes another car on from one step to the next
SLACK_WEIGHT = 1e6

x_dim = Q.shape[0]
u_dim = R.shape[0]


# -----------------------------
# Geometry / safety
# -----------------------------
vehicle_radius = 0.42        # circle enclosing the rendered car (body corner 0.39 m, LiDAR tip 0.41 m)
obs_radius = 0.30            # non-communicating moving obstacles
margin = 0.40                # gap kept to pillars and dynamic obstacles
car_margin = 0.60            # larger gap kept between cars (their predictions are less certain)
PATH_EXTRA_CLEARANCE = 0.15  # A* keeps a little more room than the NMPC limit


# -----------------------------
# Communication
# -----------------------------
SHARE_PLANS = True   # True : cars share their NMPC predicted trajectory
                     # False: cars share only [x, y, psi, v] (constant-velocity prediction)


# -----------------------------
# Output settings
# -----------------------------
SAVE_VIDEO = True
SHOW_VIEWER = True
SHOW_PLOTS = True
VIDEO_PATH = "stage3_multi_robot_astar_nmpc.mp4"
VIDEO_FPS = 10
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
