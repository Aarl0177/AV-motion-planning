"""
Matplotlib plots: x-y trajectories, speeds and controls, and clearance.
"""

import numpy as np
import matplotlib.pyplot as plt

from config import dt, vehicle_radius, obs_radius, margin, car_margin, SHOW_PLOTS
from scenarios import PILLARS
from metrics import compute_clearance, car_to_car_gap


def plot_results(result):
    cars = result["cars"]
    x_hist, u_hist, dyn_hist = result["x_hist"], result["u_hist"], result["dyn_hist"]
    t_state = np.arange(len(x_hist)) * dt
    t_control = np.arange(len(u_hist)) * dt

    # -----------------------------
    # x-y trajectories
    # -----------------------------
    fig, ax = plt.subplots(figsize=(12, 6.5))

    for px, py, pr in PILLARS:
        ax.add_patch(plt.Circle((px, py), pr, color="0.35"))
        ax.add_patch(plt.Circle((px, py), pr + vehicle_radius + margin,
                                fill=False, linestyle=":", color="0.35"))

    for c in cars:
        ax.plot(c.raw_path[:, 0], c.raw_path[:, 1], color=c.color, linewidth=1, alpha=0.35)
        ax.plot(c.path[:, 0], c.path[:, 1], "--", color=c.color, linewidth=1.5,
                label=f"Car {c.id} A* path")
        ax.plot(x_hist[:, c.id, 0], x_hist[:, c.id, 1], color=c.color, linewidth=3,
                label=f"Car {c.id} trajectory")
        ax.scatter(*x_hist[0, c.id, :2], s=70, marker="o", color=c.color, edgecolor="k", zorder=5)
        ax.scatter(*c.goal, s=140, marker="*", color=c.color, edgecolor="k", zorder=5)

    for j in range(dyn_hist.shape[1]):
        ax.plot(dyn_hist[:, j, 0], dyn_hist[:, j, 1], "--", color="crimson", linewidth=2,
                label="Dynamic obstacle" if j == 0 else None)
        ax.add_patch(plt.Circle(dyn_hist[-1, j, :2], obs_radius, fill=False,
                                color="crimson", linewidth=2))

    ax.set_xlabel("x position [m]")
    ax.set_ylabel("y position [m]")
    ax.set_title("Multi-robot A* + NMPC: trajectories (o start, * goal; thin line = raw A*)")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend(loc="upper left", fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig("xy_trajectory_plot.png", dpi=200)

    # -----------------------------
    # Speeds and controls
    # -----------------------------
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for c in cars:
        axs[0].plot(t_state, x_hist[:, c.id, 3], color=c.color, linewidth=2, label=f"Car {c.id}")
        axs[1].plot(t_control, u_hist[:, c.id, 0], color=c.color, linewidth=2)
        axs[2].plot(t_control, u_hist[:, c.id, 1], color=c.color, linewidth=2)
    axs[0].set_ylabel("v [m/s]")
    axs[1].set_ylabel("a [m/s²]")
    axs[2].set_ylabel("δ [rad]")
    axs[2].set_xlabel("Time [s]")
    axs[0].set_title("Vehicle speeds and NMPC control inputs")
    axs[0].legend()
    for a in axs:
        a.grid(True)
    fig.tight_layout()
    fig.savefig("controls_plot.png", dpi=200)

    # -----------------------------
    # Safety: clearance to nearest car / obstacle / pillar
    # -----------------------------
    clr = compute_clearance(x_hist, dyn_hist)
    fig, ax = plt.subplots(figsize=(10, 4))
    for c in cars:
        ax.plot(t_state, clr[:, c.id], color=c.color, linewidth=2, label=f"Car {c.id}")
    gap = car_to_car_gap(x_hist)
    for c in cars:
        ax.plot(t_state, gap[:, c.id], ":", color=c.color, linewidth=1.5)
    ax.axhline(margin, linestyle="--", color="0.4", label="Obstacle margin")
    ax.axhline(car_margin, linestyle="-.", color="0.2", label="Car-to-car margin (dotted = car gap)")
    ax.axhline(0.0, color="red", label="Contact")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Clearance [m]")
    ax.set_title("Clearance to the nearest car, dynamic obstacle or pillar")
    ax.set_ylim(bottom=-0.1, top=min(4.0, np.nanmax(clr) + 0.2))
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig("clearance_plot.png", dpi=200)

    if SHOW_PLOTS:
        plt.show()
    else:
        plt.close("all")
