#!/usr/bin/env python3
"""Replay MATLAB PID, MPC, and PPO CartPole trajectories in PyBullet."""

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np
import pybullet as bullet
import pybullet_data
from PIL import Image


PROJECT_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_DIR = PROJECT_ROOT / "results" / "trajectories"
OUTPUT_DIR = PROJECT_ROOT / "docs" / "images"

TRAJECTORIES = {
    "PID": TRAJECTORY_DIR / "pid_result.csv",
    "MPC": TRAJECTORY_DIR / "mpc_result.csv",
    "PPO": TRAJECTORY_DIR / "ppo_result.csv",
}

COLORS = {
    "PID": [0.10, 0.55, 0.95, 1.0],
    "MPC": [1.00, 0.45, 0.05, 1.0],
    "PPO": [0.95, 0.80, 0.10, 1.0],
}

LANES = {"PID": 1.05, "MPC": 0.0, "PPO": -1.05}


def read_trajectory(path: Path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))

    required = {"time", "position", "angle", "force"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"Invalid trajectory CSV: {path}")

    return [
        {
            "time": float(row["time"]),
            "position": float(row["position"]),
            "angle": float(row["angle"]),
            "force": float(row["force"]),
        }
        for row in rows
    ]


def create_box(half_extents, color, position):
    visual = bullet.createVisualShape(
        bullet.GEOM_BOX, halfExtents=half_extents, rgbaColor=color
    )
    return bullet.createMultiBody(
        baseMass=0, baseVisualShapeIndex=visual, basePosition=position
    )


def create_cylinder(radius, length, color, position, orientation=None):
    visual = bullet.createVisualShape(
        bullet.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=color
    )
    return bullet.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual,
        basePosition=position,
        baseOrientation=orientation or [0, 0, 0, 1],
    )


def create_sphere(radius, color, position):
    visual = bullet.createVisualShape(
        bullet.GEOM_SPHERE, radius=radius, rgbaColor=color
    )
    return bullet.createMultiBody(
        baseMass=0, baseVisualShapeIndex=visual, basePosition=position
    )


def configure_world(show_all):
    bullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    bullet.loadURDF("plane.urdf")
    bullet.configureDebugVisualizer(bullet.COV_ENABLE_GUI, 0)
    bullet.configureDebugVisualizer(bullet.COV_ENABLE_SHADOWS, 1)
    bullet.resetDebugVisualizerCamera(
        cameraDistance=5.6 if show_all else 4.0,
        cameraYaw=35,
        cameraPitch=-22,
        cameraTargetPosition=[0, 0, 0.45],
    )


def build_cartpole(controller, lane_y):
    create_box(
        [2.8, 0.045, 0.035],
        [0.25, 0.27, 0.30, 1],
        [0, lane_y, 0.035],
    )
    for rail_x in np.linspace(-2.5, 2.5, 17):
        create_box(
            [0.025, 0.32, 0.018],
            [0.35, 0.22, 0.12, 1],
            [rail_x, lane_y, 0.018],
        )

    cart = create_box(
        [0.24, 0.20, 0.11], COLORS[controller], [0, lane_y, 0.20]
    )
    wheel_orientation = bullet.getQuaternionFromEuler([math.pi / 2, 0, 0])
    wheel_color = [0.08, 0.08, 0.09, 1]
    wheels = [
        create_cylinder(
            0.075,
            0.045,
            wheel_color,
            [0, lane_y - 0.20, 0.085],
            wheel_orientation,
        ),
        create_cylinder(
            0.075,
            0.045,
            wheel_color,
            [0, lane_y + 0.20, 0.085],
            wheel_orientation,
        ),
    ]
    pole = create_cylinder(
        0.026, 1.0, [0.88, 0.12, 0.10, 1], [0, lane_y, 0.75]
    )
    bob = create_sphere(0.065, [0.95, 0.18, 0.12, 1], [0, lane_y, 1.25])
    pivot = create_sphere(0.055, [0.10, 0.10, 0.12, 1], [0, lane_y, 0.25])

    return {
        "controller": controller,
        "lane_y": lane_y,
        "cart": cart,
        "wheels": wheels,
        "pole": pole,
        "bob": bob,
        "pivot": pivot,
        "text_ids": [-1, -1],
        "force_line": -1,
    }


def update_cartpole(scene, sample):
    controller = scene["controller"]
    lane_y = scene["lane_y"]
    x = sample["position"]
    theta = sample["angle"]
    force = sample["force"]
    pivot_height = 0.31
    half_pole_length = 0.50
    direction = np.array([math.sin(theta), 0.0, math.cos(theta)])
    pivot_position = np.array([x, lane_y, pivot_height])
    pole_center = pivot_position + half_pole_length * direction
    pole_top = pivot_position + 2 * half_pole_length * direction
    pole_orientation = bullet.getQuaternionFromEuler([0, theta, 0])

    bullet.resetBasePositionAndOrientation(
        scene["cart"], [x, lane_y, 0.20], [0, 0, 0, 1]
    )
    bullet.resetBasePositionAndOrientation(
        scene["pivot"], pivot_position, [0, 0, 0, 1]
    )
    bullet.resetBasePositionAndOrientation(
        scene["pole"], pole_center, pole_orientation
    )
    bullet.resetBasePositionAndOrientation(scene["bob"], pole_top, [0, 0, 0, 1])

    wheel_orientation = bullet.getQuaternionFromEuler([math.pi / 2, 0, 0])
    for wheel, offset_y in zip(scene["wheels"], [-0.20, 0.20]):
        bullet.resetBasePositionAndOrientation(
            wheel, [x, lane_y + offset_y, 0.085], wheel_orientation
        )

    scene["force_line"] = bullet.addUserDebugLine(
        [x, lane_y - 0.29, 0.20],
        [x + 0.055 * force, lane_y - 0.29, 0.20],
        lineColorRGB=COLORS[controller][:3],
        lineWidth=5,
        replaceItemUniqueId=scene["force_line"],
    )

    labels = [
        controller,
        (
            f"t={sample['time']:.2f}s  x={x:+.2f}m  "
            f"angle={math.degrees(theta):+.2f}deg  u={force:+.2f}N"
        ),
    ]
    for index, label in enumerate(labels):
        scene["text_ids"][index] = bullet.addUserDebugText(
            label,
            [-2.30, lane_y, 1.48 - 0.13 * index],
            textColorRGB=COLORS[controller][:3],
            textSize=1.25 if index == 0 else 1.0,
            replaceItemUniqueId=scene["text_ids"][index],
        )


def save_headless_screenshot(display_name, show_all):
    view = bullet.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=[0, 0, 0.45],
        distance=5.6 if show_all else 4.0,
        yaw=35,
        pitch=-22,
        roll=0,
        upAxisIndex=2,
    )
    projection = bullet.computeProjectionMatrixFOV(
        fov=55, aspect=16 / 9, nearVal=0.1, farVal=20
    )
    _, _, rgba, _, _ = bullet.getCameraImage(
        1280,
        720,
        viewMatrix=view,
        projectionMatrix=projection,
        renderer=bullet.ER_TINY_RENDERER,
    )
    image = Image.fromarray(np.asarray(rgba, dtype=np.uint8)[:, :, :3])
    output_path = OUTPUT_DIR / f"pybullet_{display_name.lower()}_3d.png"
    image.save(output_path)
    print(f"Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--controller", choices=["ALL", *TRAJECTORIES], default="ALL"
    )
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    if args.speed <= 0:
        parser.error("--speed must be positive")

    show_all = args.controller == "ALL"
    controllers = list(TRAJECTORIES) if show_all else [args.controller]
    trajectories = {
        controller: read_trajectory(TRAJECTORIES[controller])
        for controller in controllers
    }
    frame_count = min(len(value) for value in trajectories.values())

    connection_mode = bullet.DIRECT if args.headless else bullet.GUI
    client = bullet.connect(connection_mode)
    if client < 0:
        raise RuntimeError("Could not connect to PyBullet")

    configure_world(show_all)
    scenes = {
        controller: build_cartpole(
            controller, LANES[controller] if show_all else 0.0
        )
        for controller in controllers
    }

    try:
        while True:
            for frame_index in range(frame_count):
                for controller in controllers:
                    update_cartpole(
                        scenes[controller], trajectories[controller][frame_index]
                    )
                bullet.stepSimulation()

                if args.headless and frame_index == min(25, frame_count - 1):
                    save_headless_screenshot(args.controller, show_all)

                if not args.headless and frame_index + 1 < frame_count:
                    current_time = trajectories[controllers[0]][frame_index]["time"]
                    next_time = trajectories[controllers[0]][frame_index + 1]["time"]
                    time.sleep(max((next_time - current_time) / args.speed, 0))

            if not args.loop or args.headless:
                break

        if not args.headless:
            time.sleep(1.0)
    finally:
        bullet.disconnect()


if __name__ == "__main__":
    main()
