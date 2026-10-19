"""Read the Piper arm and exchange one observation with a policy server.

The wire format matches OpenPI's websocket policy server: the server sends
msgpack metadata first, then each client message is one msgpack observation
and each reply is one msgpack dict. NumPy arrays use the same ``__ndarray__``
encoding as ``openpi_client.msgpack_numpy``.

The single RealSense color frame fills every image field. Each reply is one
action chunk. The client plays that chunk, then sends a new observation.
Ctrl+C stops the loop. Operators use USER_GUIDE.md; protocol and code notes are in DEVELOPER_GUIDE.md.
"""

import argparse
import time

import msgpack
import numpy as np
import websockets.sync.client
import websockets.sync.server

from piper_client import PiperClient, format_state

HOST = "127.0.0.1"
PORT = 8765
IMAGE_SHAPE = (224, 224, 3)


def pack_array(obj):
    if isinstance(obj, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": obj.tobytes(),
            b"dtype": obj.dtype.str,
            b"shape": obj.shape,
        }
    if isinstance(obj, np.generic):
        return {b"__npgeneric__": True, b"data": obj.item(), b"dtype": obj.dtype.str}
    return obj


def unpack_array(obj):
    if b"__ndarray__" in obj:
        return np.array(np.ndarray(buffer=obj[b"data"], dtype=np.dtype(obj[b"dtype"]), shape=obj[b"shape"]))
    if b"__npgeneric__" in obj:
        return np.dtype(obj[b"dtype"]).type(obj[b"data"])
    return obj


def packb(value):
    return msgpack.packb(value, default=pack_array)


def unpackb(value):
    return msgpack.unpackb(value, object_hook=unpack_array)


def black_image():
    return np.zeros(IMAGE_SHAPE, dtype=np.uint8)


def resize_with_pad(image, height, width):
    import cv2

    source_h, source_w = image.shape[:2]
    scale = min(width / source_w, height / source_h)
    resized_w = max(1, int(round(source_w * scale)))
    resized_h = max(1, int(round(source_h * scale)))
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    y0 = (height - resized_h) // 2
    x0 = (width - resized_w) // 2
    canvas[y0 : y0 + resized_h, x0 : x0 + resized_w] = resized
    return canvas


def capture_realsense_color():
    """Return one RGB frame from the connected RealSense, padded to 224x224."""
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    try:
        frame = None
        for _ in range(20):
            frames = pipeline.wait_for_frames(5000)
            color = frames.get_color_frame()
            if color:
                frame = np.asanyarray(color.get_data())
        if frame is None:
            raise RuntimeError("RealSense returned no color frame")
    finally:
        pipeline.stop()
    rgb = np.ascontiguousarray(frame[:, :, ::-1])
    return resize_with_pad(rgb, IMAGE_SHAPE[0], IMAGE_SHAPE[1])


def _euler_deg_to_quat_wxyz(rx, ry, rz):
    """Intrinsic XYZ Euler degrees to a wxyz quaternion."""
    rx, ry, rz = np.deg2rad([rx, ry, rz])
    cx, sx = np.cos(rx / 2), np.sin(rx / 2)
    cy, sy = np.cos(ry / 2), np.sin(ry / 2)
    cz, sz = np.cos(rz / 2), np.sin(rz / 2)
    return np.array(
        [
            cx * cy * cz + sx * sy * sz,
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz,
        ],
        dtype=np.float32,
    )


def robocasa_state(state):
    """16-d vector in the RoboCasa eval order.

    eef position (3, m), eef quaternion (4, wxyz), base position (3),
    base quaternion (4), gripper qpos (2, m). Piper has no mobile base, so the
    base pose is the identity. The arm pose is absolute, while this checkpoint
    was trained on a relative Panda-Omron state.
    """
    pose = state["pose_mm_deg"]
    finger_m = np.float32(state["gripper_mm"] / 2000.0)
    return np.concatenate(
        [
            np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float32) / 1000.0,
            _euler_deg_to_quat_wxyz(pose["rx"], pose["ry"], pose["rz"]),
            np.zeros(3, dtype=np.float32),
            np.array([1, 0, 0, 0], dtype=np.float32),
            np.array([finger_m, finger_m], dtype=np.float32),
        ]
    )


def build_observation(state, image):
    """Package live arm feedback. Replace the keys when the checkpoint expects a different layout."""
    if not state["complete"]:
        raise RuntimeError("arm feedback is incomplete")
    if image.shape != (*IMAGE_SHAPE,) or image.dtype != np.uint8:
        raise RuntimeError(f"image must be uint8 {IMAGE_SHAPE}")
    return {
        "observation/image": image,
        "observation/wrist_image": image.copy(),
        "observation/right_image": image.copy(),
        "observation/state": robocasa_state(state),
        "prompt": "reach the pen",
    }


def _require_image(image, name):
    if not isinstance(image, np.ndarray) or image.shape != IMAGE_SHAPE or image.dtype != np.uint8:
        raise ValueError(f"{name} must be uint8 {IMAGE_SHAPE}")


def handle_connection(websocket):
    websocket.send(
        packb(
            {
                "protocol": "openpi-websocket",
                "model": "black-image-placeholder",
                "image_shape": list(IMAGE_SHAPE),
                "execute": False,
            }
        )
    )
    for message in websocket:
        try:
            observation = unpackb(message)
            image = observation["observation/image"]
            _require_image(image, "observation/image")
            _require_image(observation["observation/wrist_image"], "observation/wrist_image")
            _require_image(observation["observation/right_image"], "observation/right_image")
            proprio = observation["observation/state"]
            if getattr(proprio, "shape", None) != (16,):
                raise ValueError(f"observation/state shape is {getattr(proprio, 'shape', None)}")
            print(
                "received "
                f"state={np.array2string(proprio, precision=3, separator=',')} "
                f"image_mean={float(image.mean()):.1f}",
                flush=True,
            )
            # Seven zeros stand in for six joint commands plus one gripper command.
            # The checkpoint's real action meaning is still unknown, so execute stays false.
            reply = {
                "actions": np.zeros((1, 7), dtype=np.float32),
                "execute": False,
            }
        except Exception as exc:
            websocket.send(str(exc))
            continue
        websocket.send(packb(reply))


def serve(host, port):
    with websockets.sync.server.serve(handle_connection, host, port) as server:
        print(f"listening ws://{host}:{port}", flush=True)
        server.serve_forever()


def _apply_in_subprocess(actions):
    """Run the CAN command in a fresh process. RealSense and WinUSB crash if combined."""
    import subprocess
    import sys
    import tempfile
    from pathlib import Path

    path = Path(tempfile.gettempdir()) / "piper_policy_actions.npy"
    np.save(path, np.asarray(actions))
    subprocess.check_call([sys.executable, str(Path(__file__).with_name("policy_step.py")), "apply", str(path)])


def _connect_policy(host, port):
    uri = f"ws://{host}:{port}"
    last_error = None
    for _ in range(50):
        try:
            return websockets.sync.client.connect(uri, compression=None, max_size=None, open_timeout=2)
        except OSError as exc:
            last_error = exc
            time.sleep(0.1)
    raise ConnectionError(f"policy server {uri} is not accepting connections: {last_error}")


def _recv_actions(websocket):
    reply = websocket.recv(timeout=600)
    if isinstance(reply, str):
        raise RuntimeError(reply)
    result = unpackb(reply)
    return result, np.asarray(result["actions"])


def infer_once(host, port):
    """Read, infer, and execute until Ctrl+C. The server connection stays open."""
    connection = _connect_policy(host, port)
    with connection as websocket:
        metadata = unpackb(websocket.recv(timeout=30))
        print("server_connected", flush=True)
        print(f"server_metadata={metadata}", flush=True)
        print("continuous; Ctrl+C stops", flush=True)
        cycle = 0
        try:
            while True:
                cycle += 1
                print(f"cycle={cycle}", flush=True)
                with PiperClient() as robot:
                    state = robot.read_state()
                if not state["complete"]:
                    raise RuntimeError("arm feedback is incomplete")
                image = capture_realsense_color()
                observation = build_observation(state, image)
                print(f"image_mean={float(image.mean()):.1f}", flush=True)
                print(format_state(state), flush=True)
                websocket.send(packb(observation))
                print("waiting_for_inference", flush=True)
                _result, actions = _recv_actions(websocket)
                print(f"actions_shape={tuple(actions.shape)}", flush=True)
                print("applying", flush=True)
                _apply_in_subprocess(actions)
        except KeyboardInterrupt:
            print("stopped", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("serve", "once"))
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    port = args.port
    if port is None:
        port = PORT
        if args.mode == "once":
            from team_config import configured_port

            port = configured_port()
    if args.mode == "serve":
        serve(args.host, port)
    else:
        infer_once(args.host, port)


if __name__ == "__main__":
    main()
