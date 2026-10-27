"""Read the Piper arm and exchange one observation with a policy server.

The wire format matches OpenPI's websocket policy server: the server sends
msgpack metadata first, then each client message is one msgpack observation
and each reply is one msgpack dict. NumPy arrays use the same ``__ndarray__``
encoding as ``openpi_client.msgpack_numpy``.

The single Astra Pro color frame is sent as ``observation/image``. Each reply is one
action chunk. The client plays that chunk, then sends a new observation.
Ctrl+C stops the loop. Operators use USER_GUIDE.md; protocol and code notes are in DEVELOPER_GUIDE.md.
"""

import argparse
import ctypes
from ctypes import POINTER, Structure, byref, c_void_p
import sys
import time

import msgpack
import numpy as np
import websockets.sync.client
import websockets.sync.server

from piper_client import PiperClient, format_state

HOST = "127.0.0.1"
PORT = 8765
IMAGE_SHAPE = (224, 224, 3)
COLOR_CAMERA_ID = "vid_2bc5&pid_0501"
# The only string in meta/tasks.jsonl. The tokenizer strips and replaces underscores;
# it does not lower-case, so this must match the dataset character for character.
PROMPT = "put the pen into the box"
# The fine-tune config returns 50 steps of 12 numbers: 6 delta, gripper, 5 unused.
ACTION_HORIZON = 50
ACTION_DIM = 12


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


def _directshow_names():
    """Return DirectShow video-device display names in OpenCV's index order."""
    ole32 = ctypes.windll.ole32
    ole32.CoInitialize(None)

    class GUID(Structure):
        _fields_ = [
            ("Data1", ctypes.c_ulong),
            ("Data2", ctypes.c_ushort),
            ("Data3", ctypes.c_ushort),
            ("Data4", ctypes.c_ubyte * 8),
        ]

    def guid(text):
        value = GUID()
        if ole32.CLSIDFromString(text, byref(value)) != 0:
            raise RuntimeError(text)
        return value

    def com_method(pointer, slot, prototype):
        vtable = ctypes.cast(pointer, POINTER(c_void_p))[0]
        return prototype(ctypes.cast(vtable, POINTER(c_void_p))[slot])

    def release(pointer):
        if pointer:
            com_method(pointer, 2, ctypes.WINFUNCTYPE(ctypes.c_ulong, c_void_p))(pointer)

    system_enum = guid("{62BE5D10-60EB-11d0-BD3B-00A0C911CE86}")
    video_category = guid("{860BB310-5D01-11d0-BD3B-00A0C911CE86}")
    create_enum_iid = guid("{29840822-5B84-11D0-BD3B-00A0C911CE86}")
    enumerator = c_void_p()
    hr = ole32.CoCreateInstance(byref(system_enum), None, 1, byref(create_enum_iid), byref(enumerator))
    if hr != 0:
        raise OSError(f"camera enumeration failed hr={hr:#x}")
    create = com_method(
        enumerator,
        3,
        ctypes.WINFUNCTYPE(ctypes.c_long, c_void_p, POINTER(GUID), POINTER(c_void_p), ctypes.c_ulong),
    )
    devices = c_void_p()
    hr = create(enumerator, byref(video_category), byref(devices), 0)
    if hr != 0:
        release(enumerator)
        raise OSError(f"camera enumeration failed hr={hr:#x}")
    next_device = com_method(
        devices,
        3,
        ctypes.WINFUNCTYPE(ctypes.c_long, c_void_p, ctypes.c_ulong, POINTER(c_void_p), POINTER(ctypes.c_ulong)),
    )
    bind_ctx = c_void_p()
    ole32.CreateBindCtx.argtypes = [ctypes.c_ulong, POINTER(c_void_p)]
    ole32.CreateBindCtx.restype = ctypes.c_long
    if ole32.CreateBindCtx(0, byref(bind_ctx)) != 0:
        release(devices)
        release(enumerator)
        raise OSError("camera enumeration failed while creating a bind context")
    names = []
    while True:
        moniker = c_void_p()
        fetched = ctypes.c_ulong()
        hr = next_device(devices, 1, byref(moniker), byref(fetched))
        if hr != 0 or not fetched.value:
            break
        display = c_void_p()
        get_name = com_method(
            moniker,
            20,
            ctypes.WINFUNCTYPE(ctypes.c_long, c_void_p, c_void_p, c_void_p, POINTER(c_void_p)),
        )
        text = ""
        if get_name(moniker, bind_ctx, None, byref(display)) == 0 and display.value:
            text = ctypes.wstring_at(display.value)
            ole32.CoTaskMemFree(display)
        names.append(text)
        release(moniker)
    release(bind_ctx)
    release(devices)
    release(enumerator)
    return names


def color_camera_index():
    """Index of the Astra Pro color camera. The IR and laptop cameras are skipped."""
    names = _directshow_names()
    for index, name in enumerate(names):
        if COLOR_CAMERA_ID in name.lower():
            return index
    raise RuntimeError("Astra Pro color camera was not found (USB VID_2BC5 PID_0501)")


def _avfoundation_devices():
    """Video devices in OpenCV's macOS index order (sorted by uniqueID), or [] without pyobjc."""
    try:
        import AVFoundation
    except ImportError:
        return []
    devices = AVFoundation.AVCaptureDevice.devicesWithMediaType_(AVFoundation.AVMediaTypeVideo)
    devices = sorted(devices, key=lambda device: str(device.uniqueID()))
    return [(str(device.localizedName()), str(device.modelID())) for device in devices]


def _configured_camera_index():
    """camera_index in team_config.json wins; on macOS the Astra Pro is otherwise found by USB ID."""
    from team_config import load

    value = load().get("camera_index")
    if value not in ("", None):
        return int(value)
    # The UVC model ID carries the USB IDs in decimal: 0x2BC5 = 11205, 0x0501 = 1281.
    for index, (_name, model) in enumerate(_avfoundation_devices()):
        if "VendorID_11205 ProductID_1281" in model:
            return index
    raise RuntimeError(
        "Astra Pro color camera not found. Plug it in, or set camera_index in team_config.json "
        "(python policy_link.py cameras lists the indexes)"
    )


def policy_camera_index():
    """Astra Pro index on this platform: DirectShow by USB ID on Windows, AVFoundation (or camera_index) elsewhere."""
    if sys.platform == "win32":
        return color_camera_index()
    return _configured_camera_index()


def open_policy_camera():
    """Open the Astra Pro with this platform's capture backend. Returns (capture, index)."""
    index = policy_camera_index()
    return _open_capture(index), index


def _open_capture(index):
    import cv2

    if sys.platform == "win32":
        return cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if sys.platform == "darwin":
        return cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    return cv2.VideoCapture(index)


def list_cameras(count=6):
    """Print every camera index that returns a frame, so the Astra Pro index can be chosen."""
    if sys.platform == "win32":
        names = _directshow_names()
    else:
        names = [name for name, _model in _avfoundation_devices()]
    opened = 0
    for index in range(count):
        capture = _open_capture(index)
        try:
            if not capture.isOpened():
                continue
            opened += 1
            frame = None
            for _ in range(10):
                ok, frame = capture.read()
                if not ok:
                    frame = None
            name = names[index] if index < len(names) else ""
            if frame is None:
                print(f"camera={index} opened, no frame {name}", flush=True)
            else:
                print(f"camera={index} shape={frame.shape} mean={float(frame.mean()):.1f} {name}", flush=True)
        finally:
            capture.release()
    if not opened and sys.platform == "darwin":
        print(
            "No camera opened. Allow camera access for the app running this command "
            "(System Settings > Privacy & Security > Camera), then reopen that app.",
            flush=True,
        )


def capture_color_bgr():
    """Return one BGR frame from the Astra Pro color camera."""
    import cv2

    index = policy_camera_index()
    capture = _open_capture(index)
    if not capture.isOpened():
        raise RuntimeError(f"could not open Astra Pro color camera at index {index}")
    try:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        frame = None
        for _ in range(15):
            ok, frame = capture.read()
            if not ok:
                frame = None
        if frame is None:
            raise RuntimeError("Astra Pro returned no color frame")
        return frame
    finally:
        capture.release()


def preview(index=None):
    """Live window of the camera the policy loop uses. q or Esc closes it."""
    import cv2

    if index is None:
        index = policy_camera_index()
    capture = _open_capture(index)
    if not capture.isOpened():
        raise RuntimeError(f"could not open camera {index}")
    title = f"camera {index} (q to quit)"
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                continue
            rgb = np.ascontiguousarray(frame[:, :, ::-1])
            model_view = resize_with_pad(rgb, IMAGE_SHAPE[0], IMAGE_SHAPE[1])[:, :, ::-1]
            height = frame.shape[0]
            inset = cv2.resize(model_view, (height // 3, height // 3), interpolation=cv2.INTER_NEAREST)
            frame[: inset.shape[0], : inset.shape[1]] = inset
            cv2.putText(frame, "model input 224x224", (8, inset.shape[0] + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow(title, frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


def capture_color():
    """Return one RGB frame from the Astra Pro, padded to 224x224."""
    rgb = np.ascontiguousarray(capture_color_bgr()[:, :, ::-1])
    return resize_with_pad(rgb, IMAGE_SHAPE[0], IMAGE_SHAPE[1])


def piper_train_state(state):
    """16-d state in the order ``prepare_dataset.piper_train_state()`` writes.

    End-effector position in the arm base frame in meters, quaternion **xyzw**, a
    static base (position zero, quaternion ``0 0 0 1``), then the gripper opening
    split into two fingers of opposite sign in meters. This is the vector the
    pi05_piper_finetune weights were trained on, not the older RoboCasa one.

    Dims 7:13 must be exactly zero: their std in norm_stats is zero, so the
    normalizer's ``(x - mean) / (std + 1e-6)`` would amplify any drift a million
    times and change the discretized state tokens.
    """
    # Imported here because policy_step and dataset_export both import this module,
    # so a module-level import would be circular. These are the exact functions that
    # wrote the training set, which is what keeps evaluation and training identical.
    from dataset_export import _quat_from_matrix
    from policy_step import euler_to_matrix

    pose = state["pose_mm_deg"]
    w, x, y, z = _quat_from_matrix(euler_to_matrix(pose["rx"], pose["ry"], pose["rz"]))
    half_m = np.float32(state["gripper_mm"] / 2000.0)
    vector = np.zeros(16, dtype=np.float32)
    vector[0] = float(pose["x"]) / 1000.0
    vector[1] = float(pose["y"]) / 1000.0
    vector[2] = float(pose["z"]) / 1000.0
    vector[3:7] = np.array([x, y, z, w], dtype=np.float32)
    vector[13] = 1.0
    vector[14] = half_m
    vector[15] = -half_m
    return vector


def build_observation(state, image):
    """Package live arm feedback for the pi05_piper_finetune server.

    Only ``observation/image`` is sent. The config feeds it to base_0_rgb and masks
    both wrist slots, so the frame must not be copied into them. The frame is
    already RGB: capture_color flips the BGR one, and the server does not flip.
    """
    if not state["complete"]:
        raise RuntimeError("arm feedback is incomplete")
    if image.shape != (*IMAGE_SHAPE,) or image.dtype != np.uint8:
        raise RuntimeError(f"image must be uint8 {IMAGE_SHAPE}")
    return {
        "observation/image": image,
        "observation/state": piper_train_state(state),
        # Extra key for scripted servers that command joints; model servers ignore it.
        "observation/joints_deg": np.asarray(state["joints_deg"], dtype=np.float32),
        "prompt": PROMPT,
    }


def _require_image(image, name):
    if not isinstance(image, np.ndarray) or image.shape != IMAGE_SHAPE or image.dtype != np.uint8:
        raise ValueError(f"{name} must be uint8 {IMAGE_SHAPE}")


def handle_connection(websocket):
    websocket.send(
        packb(
            {
                "protocol": "openpi-websocket",
                "model": "zero-action-placeholder",
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
            # The fine-tune config masks the wrist slots, so these keys are optional.
            for name in ("observation/wrist_image", "observation/right_image"):
                if name in observation:
                    _require_image(observation[name], name)
            proprio = observation["observation/state"]
            if getattr(proprio, "shape", None) != (16,):
                raise ValueError(f"observation/state shape is {getattr(proprio, 'shape', None)}")
            print(
                "received "
                f"state={np.array2string(proprio, precision=3, separator=',')} "
                f"image_mean={float(image.mean()):.1f} "
                f"prompt={observation.get('prompt')!r}",
                flush=True,
            )
            # A zero chunk of the real shape: no motion and an open gripper.
            # execute stays false, so this is a protocol check, not a controller.
            reply = {
                "actions": np.zeros((ACTION_HORIZON, ACTION_DIM), dtype=np.float32),
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


def _apply_in_subprocess(actions, action_space="ee_delta", speed=None, freq=None):
    """Run the CAN command in a fresh process. The camera and WinUSB crash if combined."""
    import subprocess
    import tempfile
    from pathlib import Path

    path = Path(tempfile.gettempdir()) / "piper_policy_actions.npy"
    np.save(path, np.asarray(actions))
    command = [sys.executable, str(Path(__file__).with_name("policy_step.py")), "apply", str(path), action_space]
    if speed is not None:
        command += ["--speed", str(speed)]
    if freq is not None:
        command += ["--freq", str(freq)]
    code = subprocess.call(command)
    # policy_step.REFUSED_EXIT: a limit check refused the chunk and nothing was sent; observe again.
    if code == 3:
        return False
    if code != 0:
        raise subprocess.CalledProcessError(code, command)
    return True


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


# Piper's zero position is its folded rest pose.
HOME_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def go_to(target_deg, label, speed=None, freq=None):
    """Move to target_deg with joint targets at half the demo speed, gripper open."""
    from simple_policy_server import OPEN, PEAK_STEP_DEG, keyframe_chunk

    with PiperClient() as robot:
        state = robot.read_state()
    if not state["complete"]:
        raise RuntimeError("arm feedback is incomplete")
    actions = keyframe_chunk(state["joints_deg"], [(target_deg, OPEN)], peak_step_deg=PEAK_STEP_DEG / 2)
    print(f"{label} steps={len(actions)}", flush=True)
    if not _apply_in_subprocess(actions, "joints", speed=speed, freq=freq):
        raise RuntimeError(f"the move to the {label} pose was refused; move the arm closer by hand first")


def go_ready(speed=None, freq=None):
    """Unfold to simple_policy_server.READY_DEG.

    End-pose chunks are rejected near the folded rest pose (TARGET_EXCEEDS_LIMIT),
    so run this before an ee_delta model.
    """
    from simple_policy_server import READY_DEG

    go_to(READY_DEG, "ready", speed=speed, freq=freq)


def go_home(speed=None, freq=None):
    """Fold back to HOME_DEG, the rest pose. The arm stays enabled and holds there."""
    go_to(HOME_DEG, "home", speed=speed, freq=freq)


def infer_once(host, port, speed=None, freq=None, ready=False):
    """Read, infer, and execute until Ctrl+C or the server says done. The server connection stays open."""
    if ready:
        go_ready(speed=speed, freq=freq)
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
                image = capture_color()
                observation = build_observation(state, image)
                print(f"image_mean={float(image.mean()):.1f}", flush=True)
                print(format_state(state), flush=True)
                websocket.send(packb(observation))
                print("waiting_for_inference", flush=True)
                result, actions = _recv_actions(websocket)
                if result.get("done"):
                    print("server_done", flush=True)
                    break
                action_space = str(result.get("action_space", "ee_delta"))
                print(f"actions_shape={tuple(actions.shape)} action_space={action_space}", flush=True)
                print("applying", flush=True)
                _apply_in_subprocess(actions, action_space, speed=speed, freq=freq)
        except KeyboardInterrupt:
            print("stopped", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("serve", "once", "cameras", "preview", "ready", "home"))
    parser.add_argument("--index", type=int, help="camera index for preview; default is the policy camera")
    parser.add_argument("--speed", type=int, help="joint speed percent for joint actions; default joint_speed in team_config.json")
    parser.add_argument("--freq", type=float, help="joint target updates per second; 20 holds one target per step; default 100")
    parser.add_argument("--ready", action="store_true", help="once: unfold to the ready pose before the first observation")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    if args.mode == "cameras":
        list_cameras()
        return
    if args.mode == "preview":
        preview(args.index)
        return
    if args.mode == "ready":
        go_ready(speed=args.speed, freq=args.freq)
        return
    if args.mode == "home":
        go_home(speed=args.speed, freq=args.freq)
        return
    port = args.port
    if port is None:
        port = PORT
        if args.mode == "once":
            from team_config import configured_port

            port = configured_port()
    if args.mode == "serve":
        serve(args.host, port)
    else:
        infer_once(args.host, port, speed=args.speed, freq=args.freq, ready=args.ready)


if __name__ == "__main__":
    main()
