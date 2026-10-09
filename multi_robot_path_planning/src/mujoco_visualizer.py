"""
MuJoCo visualisation: builds a kinematic/visual scene from the cars, pillars
and dynamic obstacles, then replays the simulated states in the passive viewer
or renders them to an mp4.
"""

import time

import numpy as np

from config import (dt, vehicle_radius, obs_radius, margin,
                    VIDEO_PATH, VIDEO_FPS, VIDEO_WIDTH, VIDEO_HEIGHT)
from scenarios import PILLARS, MAP_BOUNDS


def yaw_to_quat_z(yaw):
    """Rotation around z. MuJoCo quaternion order: [w, x, y, z]."""
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def quat_x(angle):
    """Rotation around x. MuJoCo quaternion order: [w, x, y, z]."""
    return np.array([np.cos(angle / 2), np.sin(angle / 2), 0.0, 0.0])


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _rgba(color, alpha=1.0, lighten=0.0):
    r, g, b = (min(1.0, ch + lighten) for ch in color)
    return f"{r:.3f} {g:.3f} {b:.3f} {alpha}"


def car_xml(i, color):
    wheels = "\n".join(
        f"""
            <body name="car{i}_{w}" pos="{px} {py} -0.07">
                <geom type="cylinder" size="0.055 0.025" material="wheel_black"/>
            </body>"""
        for w, px, py in [("fl", 0.23, 0.21), ("fr", 0.23, -0.21),
                          ("rl", -0.23, 0.21), ("rr", -0.23, -0.21)])
    return f"""
        <body name="car{i}" pos="0 0 0.30">
            <geom type="box" size="0.35 0.18 0.08" rgba="{_rgba(color)}"/>
            <geom type="box" pos="0.22 0 0.03" size="0.12 0.15 0.05" rgba="{_rgba(color, lighten=0.25)}"/>
            <geom type="box" pos="-0.06 0 0.11" size="0.14 0.13 0.07" material="car_dark"/>
            <geom type="box" pos="0.38 0 0.08" size="0.025 0.05 0.025" rgba="0 0 0 1"/>
            <geom type="capsule" fromto="0.39 0.02 0.08 3.20 0.80 0.08" size="0.012" material="lidar_blue"/>
            <geom type="capsule" fromto="0.39 -0.02 0.08 3.20 -0.80 0.08" size="0.012" material="lidar_blue"/>
            {wheels}
        </body>"""


def path_xml(car, spacing=0.4):
    """A* path drawn as thin capsules on the ground."""
    step = max(1, int(round(spacing / 0.05)))
    pts = car.path[::step]
    if np.linalg.norm(pts[-1] - car.path[-1]) > 1e-6:
        pts = np.vstack([pts, car.path[-1]])
    segs = [
        f'<geom type="capsule" fromto="{p[0]:.3f} {p[1]:.3f} 0.02 {q[0]:.3f} {q[1]:.3f} 0.02" '
        f'size="0.025" rgba="{_rgba(car.color, 0.55)}"/>'
        for p, q in zip(pts[:-1], pts[1:]) if np.linalg.norm(q - p) > 1e-3
    ]
    goal = (f'<geom type="cylinder" pos="{car.goal[0]} {car.goal[1]} 0.02" size="0.25 0.01" '
            f'rgba="{_rgba(car.color, 0.8)}"/>')
    return "\n        ".join(segs + [goal])


def build_mujoco_xml(cars, num_dyn):
    """Kinematic/visual scene: bodies are moved directly from the simulated states."""
    pillars = "\n".join(
        f"""
        <geom type="cylinder" pos="{px} {py} 0.35" size="{pr} 0.35" rgba="0.55 0.55 0.6 1"/>
        <geom type="cylinder" pos="{px} {py} 0.012" size="{pr + vehicle_radius + margin} 0.012" rgba="1 0.6 0 0.18"/>"""
        for px, py, pr in PILLARS)

    dyn = "\n".join(
        f"""
        <body name="obs_{j}" pos="0 0 0.30">
            <geom type="sphere" size="{obs_radius}" rgba="0.9 0.1 0.1 1"/>
        </body>
        <body name="safe_{j}" pos="0 0 0.015">
            <geom type="cylinder" size="{obs_radius + vehicle_radius + margin} 0.015" rgba="1 0 0 0.18"/>
        </body>"""
        for j in range(num_dyn))

    paths = "\n        ".join(path_xml(c) for c in cars)
    car_bodies = "\n".join(car_xml(c.id, c.color) for c in cars)
    cx = 0.5 * (MAP_BOUNDS[0] + MAP_BOUNDS[1])
    cy = 0.5 * (MAP_BOUNDS[2] + MAP_BOUNDS[3])
    hx = 0.5 * (MAP_BOUNDS[1] - MAP_BOUNDS[0])
    hy = 0.5 * (MAP_BOUNDS[3] - MAP_BOUNDS[2])

    return f"""
<mujoco model="multi_robot_astar_nmpc">
    <option timestep="{dt}" gravity="0 0 -9.81"/>

    <visual>
        <quality shadowsize="2048"/>
        <global offwidth="{VIDEO_WIDTH}" offheight="{VIDEO_HEIGHT}"/>
        <map znear="0.01" zfar="100"/>
    </visual>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <asset>
        <texture name="grid" type="2d" builtin="checker" width="512" height="512"
                 rgb1="0.2 0.2 0.2" rgb2="0.3 0.3 0.3"/>
        <material name="grid_mat" texture="grid" texrepeat="24 12" reflectance="0.1"/>
        <material name="car_dark" rgba="0.03 0.05 0.12 1"/>
        <material name="wheel_black" rgba="0.01 0.01 0.01 1"/>
        <material name="lidar_blue" rgba="0.1 0.8 1.0 0.35"/>
    </asset>

    <worldbody>
        <light pos="{cx} {cy} 10" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
        <light pos="{cx} {cy - 8} 8" dir="0 0.6 -0.8" diffuse="0.3 0.3 0.3" castshadow="false"/>

        <geom name="ground" type="plane" pos="{cx} {cy} 0" size="{hx + 2} {hy + 2} 0.1" material="grid_mat"/>

        {paths}
        {pillars}
        {dyn}
        {car_bodies}
    </worldbody>
</mujoco>
"""


def get_body_ids(model, n_cars, num_dyn):
    return {
        "cars": [{
            "body": model.body(f"car{i}").id,
            "fl": model.body(f"car{i}_fl").id,
            "fr": model.body(f"car{i}_fr").id,
            "rl": model.body(f"car{i}_rl").id,
            "rr": model.body(f"car{i}_rr").id,
        } for i in range(n_cars)],
        "obs": [model.body(f"obs_{j}").id for j in range(num_dyn)],
        "safe": [model.body(f"safe_{j}").id for j in range(num_dyn)],
    }


def set_replay_frame(model, data, ids, x_hist, dyn_hist, u_hist, frame):
    import mujoco

    wheel_base_quat = quat_x(np.pi / 2)    # cylinder axis -> wheel axle

    for i, car_ids in enumerate(ids["cars"]):
        x_pos, y_pos, psi, _ = x_hist[frame, i]
        if len(u_hist) == 0:
            delta = 0.0
        else:
            delta = u_hist[min(frame, len(u_hist) - 1), i, 1]

        model.body_pos[car_ids["body"]] = [x_pos, y_pos, 0.30]
        model.body_quat[car_ids["body"]] = yaw_to_quat_z(psi)

        front = quat_mul(yaw_to_quat_z(delta), wheel_base_quat)
        model.body_quat[car_ids["fl"]] = front
        model.body_quat[car_ids["fr"]] = front
        model.body_quat[car_ids["rl"]] = wheel_base_quat
        model.body_quat[car_ids["rr"]] = wheel_base_quat

    for j in range(len(ids["obs"])):
        ox, oy = dyn_hist[frame, j, :2]
        model.body_pos[ids["obs"][j]] = [ox, oy, 0.30]
        model.body_pos[ids["safe"][j]] = [ox, oy, 0.015]

    mujoco.mj_forward(model, data)


def _load_scene(result):
    import mujoco

    cars = result["cars"]
    num_dyn = result["dyn_hist"].shape[1]
    model = mujoco.MjModel.from_xml_string(build_mujoco_xml(cars, num_dyn))
    data = mujoco.MjData(model)
    ids = get_body_ids(model, len(cars), num_dyn)
    return model, data, ids


def _set_camera(cam):
    cam.lookat[:] = [0.5 * (MAP_BOUNDS[0] + MAP_BOUNDS[1]) - 1.0,
                     0.5 * (MAP_BOUNDS[2] + MAP_BOUNDS[3]) + 0.5, 0.0]
    cam.distance = 19.0
    cam.azimuth = 90       # camera south of the map, +x to the right
    cam.elevation = -58


def save_mujoco_video(result, output_path=VIDEO_PATH):
    import mujoco
    import imageio.v2 as imageio

    model, data, ids = _load_scene(result)
    x_hist, dyn_hist, u_hist = result["x_hist"], result["dyn_hist"], result["u_hist"]

    renderer = mujoco.Renderer(model, height=VIDEO_HEIGHT, width=VIDEO_WIDTH)
    writer = imageio.get_writer(output_path, fps=VIDEO_FPS, macro_block_size=1)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    _set_camera(cam)

    print("Saving video to:", output_path)
    for frame in range(len(x_hist)):
        set_replay_frame(model, data, ids, x_hist, dyn_hist, u_hist, frame)
        renderer.update_scene(data, camera=cam)
        writer.append_data(renderer.render())

    writer.close()
    renderer.close()
    print("Video saved:", output_path)


def visualize_in_mujoco(result):
    import mujoco.viewer

    model, data, ids = _load_scene(result)
    x_hist, dyn_hist, u_hist = result["x_hist"], result["dyn_hist"], result["u_hist"]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        _set_camera(viewer.cam)
        frame = 0
        while viewer.is_running():
            set_replay_frame(model, data, ids, x_hist, dyn_hist, u_hist, frame)
            viewer.sync()
            frame = (frame + 1) % len(x_hist)
            time.sleep(dt)
