"""Play a full policy chunk in the Piper base frame.

Position deltas are meters in the arm base frame (+X forward, +Y left, +Z up).
Orientation deltas are axis-angle radians about those same axes. Every returned
step is sent in order. The gripper uses the verified millimeter command.
Base and control-mode values are ignored.
"""

import sys

import numpy as np

from piper_client import PiperClient, format_state
from policy_link import infer_once

STEP_SECONDS = 0.05
POSE_SPEED = 50


def _rx(deg):
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(deg):
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(deg):
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_to_matrix(rx, ry, rz):
    """Match piper_fk: R = Rz(rz) Ry(ry) Rx(rx), degrees."""
    return _rz(rz) @ _ry(ry) @ _rx(rx)


def matrix_to_euler(rotation):
    r20 = rotation[2, 0]
    if r20 < -1 + 1e-4:
        pitch = 90.0
        yaw = 0.0
        roll = np.rad2deg(np.arctan2(rotation[0, 1], rotation[1, 1]))
    elif r20 > 1 - 1e-4:
        pitch = -90.0
        yaw = 0.0
        roll = -np.rad2deg(np.arctan2(rotation[0, 1], rotation[1, 1]))
    else:
        pitch = np.rad2deg(np.arctan2(-r20, np.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)))
        cosine = np.cos(np.deg2rad(pitch))
        yaw = np.rad2deg(np.arctan2(rotation[1, 0] / cosine, rotation[0, 0] / cosine))
        roll = np.rad2deg(np.arctan2(rotation[2, 1] / cosine, rotation[2, 2] / cosine))
    return float(roll), float(pitch), float(yaw)


def _axis_angle_matrix(rotation_vector):
    angle = np.linalg.norm(rotation_vector)
    if angle < 1e-9:
        return np.eye(3)
    axis = rotation_vector / angle
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return c * np.eye(3) + s * cross + (1 - c) * np.outer(axis, axis)


def action_waypoints(actions, pose):
    """Integrate every policy step. Position is millimeters in the arm base frame."""
    position = np.array([pose["x"], pose["y"], pose["z"]], dtype=float)
    start = position.copy()
    rotation = euler_to_matrix(pose["rx"], pose["ry"], pose["rz"])
    recovered = matrix_to_euler(rotation)
    euler_error = max(
        abs(recovered[0] - pose["rx"]),
        abs(recovered[1] - pose["ry"]),
        abs(recovered[2] - pose["rz"]),
    )
    use_rotation = euler_error <= 1.0
    waypoints = []
    for step in actions:
        position = position + np.asarray(step[0:3], dtype=float) * 0.05 * 1000.0
        if use_rotation:
            rotation = _axis_angle_matrix(np.asarray(step[3:6], dtype=float) * 0.5) @ rotation
            rx, ry, rz = matrix_to_euler(rotation)
        else:
            rx, ry, rz = pose["rx"], pose["ry"], pose["rz"]
        waypoints.append(
            {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
                "rx": rx,
                "ry": ry,
                "rz": rz,
                "gripper_close": float(step[6]) >= 0.5,
            }
        )
    return waypoints, float(np.linalg.norm(position - start)), float(euler_error)


def apply_actions(actions):
    """Enable the arm and play every step in the returned chunk."""
    actions = np.asarray(actions)
    with PiperClient() as robot:
        before = robot.read_state()
        if not before["complete"]:
            raise RuntimeError("arm feedback is incomplete")
        waypoints, shift_mm, euler_error = action_waypoints(actions, before["pose_mm_deg"])
        if not waypoints:
            raise RuntimeError("policy returned no actions")
        final = waypoints[-1]
        print(
            f"command_steps={len(waypoints)} shift_mm={shift_mm:.2f} "
            f"euler_roundtrip_deg={euler_error:.3f} gripper_close={final['gripper_close']}",
            flush=True,
        )
        print(
            "target_mm_deg="
            f"x:{final['x']:.2f}, y:{final['y']:.2f}, z:{final['z']:.2f}, "
            f"rx:{final['rx']:.2f}, ry:{final['ry']:.2f}, rz:{final['rz']:.2f}",
            flush=True,
        )
        maximum = robot.gripper_max_mm()
        print("exit_teach", flush=True)
        robot.exit_drag_teach()
        print("enabling", flush=True)
        robot.enable_arm()
        print("moving", flush=True)
        for waypoint in waypoints:
            gripper_target = 0.0 if waypoint["gripper_close"] else maximum
            robot.move_pose(
                waypoint["x"],
                waypoint["y"],
                waypoint["z"],
                waypoint["rx"],
                waypoint["ry"],
                waypoint["rz"],
                duration=STEP_SECONDS,
                speed=POSE_SPEED,
                gripper_mm=gripper_target,
            )
        print("pose_sent", flush=True)
        reached = robot.move_gripper(0.0 if final["gripper_close"] else maximum)
        after = robot.read_state()
    print(f"gripper_target_mm={0.0 if final['gripper_close'] else maximum:.1f} gripper_reached_mm={reached}")
    print("after")
    print(format_state(after))


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "apply":
        apply_actions(np.load(sys.argv[2]))
        return
    infer_once("127.0.0.1", 8000)


if __name__ == "__main__":
    main()
