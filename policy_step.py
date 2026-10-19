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
from team_config import load as load_config

STEP_SECONDS = 0.05
POSE_SPEED = 50
DEFAULT_MAX_CHUNK_SHIFT_MM = 30.0
DEFAULT_JOINT_SPEED = 60
DEFAULT_JOINT_UPDATE_HZ = 100.0
# 0.05 s per step, so 3 degrees per step caps joint speed near 60 deg/s.
MAX_JOINT_STEP_DEG = 3.0
DEFAULT_MAX_CHUNK_JOINT_DEG = 100.0


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


def check_limits(waypoints, pose, config):
    """Raise before any frame is sent if a waypoint leaves the allowed region.

    max_chunk_shift_mm bounds every waypoint's distance from the pose read before
    the chunk. workspace_mm is an absolute base-frame box {"x": [min, max], ...};
    leave it empty to skip the box until it has been measured on site.
    """
    limit = float(config.get("max_chunk_shift_mm") or DEFAULT_MAX_CHUNK_SHIFT_MM)
    start = np.array([pose["x"], pose["y"], pose["z"]], dtype=float)
    reach = max(
        float(np.linalg.norm(np.array([w["x"], w["y"], w["z"]]) - start)) for w in waypoints
    )
    if reach > limit:
        raise RuntimeError(f"chunk reaches {reach:.1f} mm from the start pose; max_chunk_shift_mm={limit:.1f}. Nothing sent")
    box = config.get("workspace_mm") or {}
    for index, waypoint in enumerate(waypoints):
        for axis, bounds in box.items():
            if not bounds:
                continue
            low, high = bounds
            if not low <= waypoint[axis] <= high:
                raise RuntimeError(
                    f"step {index} {axis}={waypoint[axis]:.1f} mm is outside workspace_mm {axis}=[{low}, {high}]. Nothing sent"
                )
    return reach


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
        reach_mm = check_limits(waypoints, before["pose_mm_deg"], load_config())
        final = waypoints[-1]
        print(
            f"command_steps={len(waypoints)} shift_mm={shift_mm:.2f} reach_mm={reach_mm:.2f} "
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


def check_joint_chunk(actions, start, config):
    """Raise before any frame is sent if a joint chunk jumps, or travels too far from the start."""
    if actions.ndim != 2 or actions.shape[1] < 7 or not len(actions):
        raise RuntimeError(f"joint actions must be (N, 7): six angles in degrees and a gripper value, got {actions.shape}")
    limit = float(config.get("max_chunk_joint_deg") or DEFAULT_MAX_CHUNK_JOINT_DEG)
    previous = np.asarray(start, dtype=float)
    for index, step in enumerate(actions[:, :6]):
        jump = float(np.max(np.abs(step - previous)))
        if jump > MAX_JOINT_STEP_DEG:
            raise RuntimeError(f"step {index} moves a joint {jump:.1f} deg in one step (max {MAX_JOINT_STEP_DEG}). Nothing sent")
        previous = step
    reach = float(np.max(np.abs(actions[:, :6] - np.asarray(start, dtype=float))))
    if reach > limit:
        raise RuntimeError(f"chunk moves a joint {reach:.1f} deg from the start; max_chunk_joint_deg={limit:.1f}. Nothing sent")
    return reach


def apply_joint_actions(actions, speed=None, update_hz=None):
    """Play a chunk of absolute joint targets: columns 0-5 degrees, column 6 gripper (>= 0.5 closes).

    speed (percent) falls back to joint_speed in team_config.json, then 60.
    update_hz is how often the target changes within a step; 20 holds one
    target per 0.05 s step.
    """
    actions = np.asarray(actions, dtype=float)
    with PiperClient() as robot:
        before = robot.read_state()
        if not before["complete"]:
            raise RuntimeError("arm feedback is incomplete")
        config = load_config()
        reach = check_joint_chunk(actions, before["joints_deg"], config)
        speed = int(speed or config.get("joint_speed") or DEFAULT_JOINT_SPEED)
        update_hz = float(update_hz or DEFAULT_JOINT_UPDATE_HZ)
        final = actions[-1]
        print(
            f"command_steps={len(actions)} joint_reach_deg={reach:.1f} speed={speed} freq={update_hz:g} "
            "target_joints_deg=" + ", ".join(f"J{i + 1}:{v:.1f}" for i, v in enumerate(final[:6])),
            flush=True,
        )
        maximum = robot.gripper_max_mm()
        robot.exit_drag_teach()
        robot.enable_arm()
        previous = before["joints_deg"]
        for step in actions:
            robot.move_joints(
                step[:6],
                duration=STEP_SECONDS,
                speed=speed,
                gripper_mm=0.0 if step[6] >= 0.5 else maximum,
                start_deg=previous,
                update_hz=update_hz,
            )
            previous = step[:6]
        print("joints_sent", flush=True)
        # Hold the last target briefly so the arm settles before the state is read.
        robot.move_joints(final[:6], duration=0.5, speed=speed, gripper_mm=0.0 if final[6] >= 0.5 else maximum)
        after = robot.read_state()
    print("after")
    print(format_state(after))


def main():
    if len(sys.argv) > 2 and sys.argv[1] == "apply":
        import argparse

        parser = argparse.ArgumentParser()
        parser.add_argument("command")
        parser.add_argument("path")
        parser.add_argument("action_space", nargs="?", default="ee_delta", choices=("ee_delta", "joints"))
        parser.add_argument("--speed", type=int)
        parser.add_argument("--freq", type=float)
        args = parser.parse_args()
        if args.action_space == "joints":
            apply_joint_actions(np.load(args.path), speed=args.speed, update_hz=args.freq)
        else:
            apply_actions(np.load(args.path))
        return
    infer_once("127.0.0.1", 8000)


if __name__ == "__main__":
    main()
