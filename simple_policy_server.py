"""Scripted policy server for testing the whole link without a model.

Speaks the same websocket protocol as OpenPI's serve_policy.py: metadata
first, then one action chunk per observation.

--mode demo (default) replies with absolute joint targets ("action_space":
"joints", shape (N, 7): six angles in degrees and a gripper value). It unfolds
from wherever the arm is to READY_DEG, then cycles through a wave, a nod, and
a wrist twist with the gripper. Joint targets avoid the end-pose IK, which
rejects targets near the folded rest pose (TARGET_EXCEEDS_LIMIT).

--mode nudge replies with the (50, 12) end-effector delta chunk from
policy_step.action_waypoints(): +X by AMPLITUDE_MM and back.

    python simple_policy_server.py --port 8000
"""

import argparse
import socket

import numpy as np
import websockets.sync.server

from policy_link import packb, unpackb

CHUNK_STEPS = 50
ACTION_DIM = 12
AMPLITUDE_MM = 10.0
# action_waypoints() multiplies translation by 0.05 m, i.e. 50 mm per unit.
MM_PER_UNIT = 50.0


def scripted_chunk(cycle):
    """Out along +X and back to the start; gripper closes on odd cycles."""
    half = CHUNK_STEPS // 2
    step = AMPLITUDE_MM / half / MM_PER_UNIT
    actions = np.zeros((CHUNK_STEPS, ACTION_DIM), dtype=np.float32)
    actions[:half, 0] = step
    actions[half:, 0] = -step
    actions[:, 6] = 1.0 if cycle % 2 else 0.0
    return actions


READY_DEG = np.array([0.0, 45.0, -45.0, 0.0, 30.0, 0.0])
MIN_STEPS_PER_SEGMENT = 30
# Smoothstep peaks at 1.5x the average speed; keep the peak under policy_step's 3 deg/step cap.
PEAK_STEP_DEG = 2.4
OPEN, CLOSED = 0.0, 1.0


def _offset(j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=0.0, j6=0.0):
    return READY_DEG + np.array([j1, j2, j3, j4, j5, j6])


# Each routine is a list of (joint targets, gripper) keyframes, starting from READY.
ROUTINES = [
    ("wave", [(_offset(j1=30), OPEN), (_offset(j1=-30), OPEN), (_offset(j1=30), OPEN), (READY_DEG, OPEN)]),
    ("nod", [(_offset(j5=35), OPEN), (_offset(j5=-30), OPEN), (_offset(j2=15, j3=-15), OPEN), (READY_DEG, OPEN)]),
    ("twist", [(_offset(j6=90), CLOSED), (_offset(j6=-90), OPEN), (_offset(j6=60), CLOSED), (READY_DEG, OPEN)]),
]


def keyframe_chunk(current, keyframes, peak_step_deg=PEAK_STEP_DEG):
    """Smoothstep from the current joints through each keyframe. Returns (N, 7).

    Each segment gets enough 0.05 s steps that no joint moves more than
    peak_step_deg in one step.
    """
    rows = []
    start = np.asarray(current, dtype=float)
    for target, gripper in keyframes:
        target = np.asarray(target, dtype=float)
        distance = float(np.max(np.abs(target - start)))
        steps = max(MIN_STEPS_PER_SEGMENT, int(np.ceil(1.5 * distance / peak_step_deg)))
        for step in range(1, steps + 1):
            t = step / steps
            blend = t * t * (3 - 2 * t)
            rows.append(np.concatenate([start + (target - start) * blend, [gripper]]))
        start = target
    return np.asarray(rows, dtype=np.float32)


def demo_chunk(cycle, joints):
    """Cycle 1 unfolds to READY_DEG; later cycles play the routines in turn."""
    if cycle == 1:
        # Unfold from the rest pose at half speed.
        return "ready", keyframe_chunk(joints, [(READY_DEG, OPEN)], peak_step_deg=PEAK_STEP_DEG / 2)
    name, keyframes = ROUTINES[(cycle - 2) % len(ROUTINES)]
    return name, keyframe_chunk(joints, keyframes)


def handle_connection(websocket, mode="demo"):
    websocket.send(
        packb(
            {
                "protocol": "openpi-websocket",
                "model": "simple-scripted",
                "mode": mode,
            }
        )
    )
    cycle = 0
    for message in websocket:
        try:
            observation = unpackb(message)
            state = np.asarray(observation["observation/state"])
            image = np.asarray(observation["observation/image"])
            cycle += 1
            print(
                f"cycle={cycle} state={np.array2string(state, precision=3, separator=',')} "
                f"image_shape={image.shape} image_mean={float(image.mean()):.1f} "
                f"prompt={observation.get('prompt')!r}",
                flush=True,
            )
            if mode == "demo":
                name, actions = demo_chunk(cycle, observation["observation/joints_deg"])
                print(f"routine={name} steps={len(actions)}", flush=True)
                reply = {"actions": actions, "action_space": "joints"}
            else:
                reply = {"actions": scripted_chunk(cycle), "action_space": "ee_delta"}
        except Exception as exc:
            websocket.send(str(exc))
            continue
        websocket.send(packb(reply))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--mode", choices=("demo", "nudge"), default="demo")
    args = parser.parse_args()

    def handler(websocket):
        handle_connection(websocket, args.mode)

    with websockets.sync.server.serve(handler, args.host, args.port, compression=None, max_size=None) as server:
        print(f"listening on {args.host}:{args.port} host={socket.gethostname()} mode={args.mode}", flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
