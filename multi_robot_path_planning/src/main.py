"""
Multi-robot NMPC with A* global planning and shared-state avoidance
===================================================================

Each car runs the same three-layer stack:

1. Global planner (A*)                                  -> global_planner.py
   Plans a collision-free route on an occupancy grid built from the static
   obstacles (pillars), inflated by vehicle radius + safety margin. The route
   is planned once at start, then shortcut-pruned, Chaikin-smoothed and
   resampled so the NMPC gets a smooth reference with a well-defined heading.

2. Communication (V2V)                                  -> car.py, obstacles.py
   After every control step each car broadcasts a message:
       {id, state = [x, y, psi, v], plan = NMPC predicted trajectory}
   Every other car reads it at the next step (one-step delay, like a real
   network). With SHARE_PLANS = False only the state is shared and the other
   cars are predicted with constant velocity.

3. Local planner (NMPC, CasADi + IPOPT)                 -> nmpc_controller.py
   Tracks the car's own A* path (error expressed in the path frame) and
   treats everything else as obstacles, all through soft distance constraints:
       - other cars        -> moving obstacles, predicted from their messages
       - dynamic obstacles -> moving obstacles, constant-velocity prediction
                              (they are sensed, they do not communicate)
       - pillars           -> static obstacles

Other modules:
    config.py            NMPC settings, limits, weights, margins, output flags
    scenarios.py         map, pillars, cars, dynamic obstacles
    dynamics.py          kinematic bicycle model (CasADi + NumPy)
    metrics.py           clearance / car-gap metrics and the printed summary
    plotting.py          matplotlib plots
    mujoco_visualizer.py MuJoCo replay viewer and video export

Run:  python main.py
"""

import time

import numpy as np

from config import (MAX_STEPS, dt, vehicle_radius, margin, PATH_EXTRA_CLEARANCE,
                    SAVE_VIDEO, SHOW_VIEWER, VIDEO_PATH)
from scenarios import MAP_BOUNDS, GRID_RES, PILLARS, CARS, DYN_OBS
from global_planner import OccupancyGrid
from nmpc_controller import NMPCController
from car import Car
from obstacles import obstacle_dynamics_all, build_obstacle_slots
from metrics import print_summary
from plotting import plot_results
from mujoco_visualizer import save_mujoco_video, visualize_in_mujoco


def run_multi_robot_simulation():
    plan_grid = OccupancyGrid(MAP_BOUNDS, GRID_RES, PILLARS,
                              vehicle_radius + margin + PATH_EXTRA_CLEARANCE)
    check_grid = OccupancyGrid(MAP_BOUNDS, GRID_RES, PILLARS, vehicle_radius + margin)

    print("Planning global paths with A* ...")
    cars = []
    for i, cfg in enumerate(CARS):
        car = Car(i, cfg["start"], cfg["goal"], cfg["color"], plan_grid, check_grid)
        print(f"  car {i}: {len(car.raw_path)} grid cells -> {len(car.waypoints)} waypoints, "
              f"length {car.ref.length:.2f} m")
        cars.append(car)

    n_slots = max(1, (len(cars) - 1) + len(DYN_OBS) + len(PILLARS))
    for car in cars:
        car.controller = NMPCController(n_slots)

    dyn_obs = DYN_OBS.copy()
    x_hist = [np.array([c.state for c in cars])]
    u_hist = []
    dyn_hist = [dyn_obs.copy()]
    solve_times = []

    for step in range(MAX_STEPS):
        t = (step + 1) * dt

        # 1) Communication: everyone reads the messages sent at the end of the last step
        inbox = [car.broadcast() for car in cars]

        # 2) Every car solves its own NMPC (decentralised, same information time)
        controls = []
        for car in cars:
            obs_x, obs_y, obs_r = build_obstacle_slots(car, inbox, dyn_obs, n_slots)
            t0 = time.perf_counter()
            u = car.compute_control(obs_x, obs_y, obs_r)
            if not car.arrived:
                solve_times.append(time.perf_counter() - t0)
            controls.append(u)

        # 3) Apply all controls simultaneously, move the obstacles
        for car, u in zip(cars, controls):
            car.apply(u, t)
        dyn_obs = obstacle_dynamics_all(dyn_obs, dt)

        x_hist.append(np.array([c.state for c in cars]))
        u_hist.append(np.array(controls))
        dyn_hist.append(dyn_obs.copy())

        if (step + 1) % 20 == 0:
            status = ", ".join(
                f"car{c.id}: ({c.state[0]:5.2f}, {c.state[1]:5.2f}) v={c.state[3]:.2f}"
                + (" [parked]" if c.arrived else "") for c in cars)
            print(f"t = {t:5.1f} s | {status}")

        if all(c.arrived for c in cars):
            print(f"All cars reached their goals at t = {t:.1f} s")
            break

    result = {
        "cars": cars,
        "x_hist": np.array(x_hist),     # (T+1, n_cars, 4)
        "u_hist": np.array(u_hist),     # (T,   n_cars, 2)
        "dyn_hist": np.array(dyn_hist), # (T+1, n_dyn, 4)
        "solve_times": np.array(solve_times),
    }
    print_summary(result)
    return result


if __name__ == "__main__":
    result = run_multi_robot_simulation()

    plot_results(result)

    if SAVE_VIDEO:
        save_mujoco_video(result, VIDEO_PATH)

    if SHOW_VIEWER:
        visualize_in_mujoco(result)
