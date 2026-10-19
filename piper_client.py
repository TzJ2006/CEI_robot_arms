"""Local Piper client over a candleLight (gs_usb) adapter.

Reads joint angles, gripper opening, end-effector pose, and 0x2A1 status.
Gripper motion is separate and does not enable the arm joints.

Windows talks to the adapter through WinUSB; macOS and Linux use libusb via
pyusb. Both send the same gs_usb control transfers and 20-byte frames.
"""
import ctypes
import math
import queue
import struct
import sys
import threading
import time
from ctypes import (
    POINTER,
    Structure,
    byref,
    c_byte,
    c_ubyte,
    c_ulong,
    c_ushort,
    c_void_p,
    sizeof,
    wintypes,
)
from uuid import UUID

INTERFACE_GUID = "{c15b4308-04d3-11e6-b3ea-6057189e6443}"
USB_VENDOR_ID = 0x1D50
USB_PRODUCT_ID = 0x606F
CTRL_MODE = {
    0x00: "STANDBY",
    0x01: "CAN_CTRL",
    0x02: "TEACHING",
    0x03: "ETHERNET",
    0x04: "WIFI",
    0x05: "REMOTE",
    0x06: "LINKAGE_TEACH",
    0x07: "OFFLINE_TRAJECTORY",
}
ARM_STATUS = {
    0x00: "NORMAL",
    0x01: "EMERGENCY_STOP",
    0x02: "NO_SOLUTION",
    0x03: "SINGULARITY",
    0x04: "TARGET_EXCEEDS_LIMIT",
    0x05: "JOINT_COMMUNICATION_ERR",
    0x06: "JOINT_BRAKE_NOT_RELEASED",
    0x07: "COLLISION",
    0x08: "TEACH_OVERSPEED",
    0x09: "JOINT_STATUS_ERR",
    0x0A: "OTHER_ERR",
}
# Piper joint limits in degrees, from piper_sdk. J2 and J3 rest near 0 in the folded pose.
JOINT_LIMITS_DEG = (
    (-150.0, 150.0),
    (0.0, 180.0),
    (-170.0, 0.0),
    (-100.0, 100.0),
    (-70.0, 70.0),
    (-120.0, 120.0),
)
MODE_FEED = {
    0x00: "MOVE_P",
    0x01: "MOVE_J",
    0x02: "MOVE_L",
    0x03: "MOVE_C",
    0x04: "MOVE_M",
    0x05: "MOVE_CPV",
}


class GUID(Structure):
    _fields_ = [("Data1", wintypes.DWORD), ("Data2", wintypes.WORD), ("Data3", wintypes.WORD), ("Data4", c_byte * 8)]


class IFACE(Structure):
    _fields_ = [("cbSize", wintypes.DWORD), ("InterfaceClassGuid", GUID), ("Flags", wintypes.DWORD), ("Reserved", c_void_p)]


class DETAIL(Structure):
    _fields_ = [("cbSize", wintypes.DWORD), ("DevicePath", wintypes.WCHAR * 1024)]


class SETUP(Structure):
    _fields_ = [("RequestType", c_ubyte), ("Request", c_ubyte), ("Value", c_ushort), ("Index", c_ushort), ("Length", c_ushort)]


class PIPE(Structure):
    _fields_ = [("PipeType", c_ulong), ("PipeId", c_ubyte), ("MaximumPacketSize", c_ushort), ("Interval", c_ubyte)]


class OVERLAPPED(Structure):
    _fields_ = [
        ("Internal", c_void_p),
        ("InternalHigh", c_void_p),
        ("Offset", wintypes.DWORD),
        ("OffsetHigh", wintypes.DWORD),
        ("hEvent", c_void_p),
    ]


def _device_path():
    parsed = UUID(INTERFACE_GUID)
    setup = ctypes.WinDLL("setupapi", use_last_error=True)
    setup.SetupDiGetClassDevsW.restype = c_void_p
    setup.SetupDiGetClassDevsW.argtypes = [POINTER(GUID), wintypes.LPCWSTR, wintypes.HWND, wintypes.DWORD]
    setup.SetupDiEnumDeviceInterfaces.argtypes = [c_void_p, c_void_p, POINTER(GUID), wintypes.DWORD, POINTER(IFACE)]
    setup.SetupDiEnumDeviceInterfaces.restype = wintypes.BOOL
    setup.SetupDiGetDeviceInterfaceDetailW.argtypes = [
        c_void_p, POINTER(IFACE), c_void_p, wintypes.DWORD, POINTER(wintypes.DWORD), c_void_p
    ]
    setup.SetupDiGetDeviceInterfaceDetailW.restype = wintypes.BOOL
    setup.SetupDiDestroyDeviceInfoList.argtypes = [c_void_p]
    guid = GUID()
    guid.Data1, guid.Data2, guid.Data3 = parsed.time_low, parsed.time_mid, parsed.time_hi_version
    for index, value in enumerate(parsed.bytes[8:]):
        guid.Data4[index] = value
    device_set = setup.SetupDiGetClassDevsW(byref(guid), None, None, 0x12)
    iface = IFACE()
    iface.cbSize = sizeof(IFACE)
    if not setup.SetupDiEnumDeviceInterfaces(device_set, None, byref(guid), 0, byref(iface)):
        setup.SetupDiDestroyDeviceInfoList(device_set)
        raise OSError(f"candleLight interface missing err={ctypes.get_last_error()}")
    required = wintypes.DWORD()
    setup.SetupDiGetDeviceInterfaceDetailW(device_set, byref(iface), None, 0, byref(required), None)
    detail = DETAIL()
    detail.cbSize = 8
    ok = setup.SetupDiGetDeviceInterfaceDetailW(device_set, byref(iface), byref(detail), sizeof(detail), byref(required), None)
    setup.SetupDiDestroyDeviceInfoList(device_set)
    if not ok:
        raise OSError(f"device path failed err={ctypes.get_last_error()}")
    return detail.DevicePath


def _be32(data):
    return int.from_bytes(data[:4], "big", signed=True)


def _be16(data):
    return int.from_bytes(data[:2], "big", signed=True)


def _can_frame(arbitration_id, data):
    payload = bytes(data[:8]).ljust(8, b"\x00")
    return struct.pack("<2I12B", 0, arbitration_id, 8, 0, 0, 0, *payload)


class _WinUsbTransport:
    """candleLight through WinUSB with overlapped I/O."""

    def __init__(self):
        self._kernel = None
        self._winusb = None
        self._device = None
        self._interface = None
        self._read_event = None
        self._write_event = None
        self._in_pipe = 0x81
        self._out_pipe = 0x02
        self._buffer = ctypes.create_string_buffer(64)
        self._write_buffer = None
        self._write_overlapped = None

    def open(self):
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        winusb = ctypes.WinDLL("winusb", use_last_error=True)
        kernel.CreateFileW.restype = c_void_p
        kernel.CreateFileW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, c_void_p, wintypes.DWORD, wintypes.DWORD, c_void_p]
        kernel.CloseHandle.argtypes = [c_void_p]
        kernel.CreateEventW.restype = c_void_p
        kernel.CreateEventW.argtypes = [c_void_p, wintypes.BOOL, wintypes.BOOL, wintypes.LPCWSTR]
        kernel.ResetEvent.argtypes = [c_void_p]
        kernel.WaitForSingleObject.argtypes = [c_void_p, wintypes.DWORD]
        kernel.WaitForSingleObject.restype = wintypes.DWORD
        kernel.GetOverlappedResult.argtypes = [c_void_p, POINTER(OVERLAPPED), POINTER(wintypes.DWORD), wintypes.BOOL]
        kernel.GetOverlappedResult.restype = wintypes.BOOL
        kernel.CancelIoEx.argtypes = [c_void_p, c_void_p]
        kernel.CancelIoEx.restype = wintypes.BOOL
        winusb.WinUsb_Initialize.argtypes = [c_void_p, POINTER(c_void_p)]
        winusb.WinUsb_Initialize.restype = wintypes.BOOL
        winusb.WinUsb_Free.argtypes = [c_void_p]
        winusb.WinUsb_ControlTransfer.argtypes = [c_void_p, SETUP, c_void_p, c_ulong, POINTER(c_ulong), c_void_p]
        winusb.WinUsb_ControlTransfer.restype = wintypes.BOOL
        winusb.WinUsb_QueryPipe.argtypes = [c_void_p, c_ubyte, c_ubyte, POINTER(PIPE)]
        winusb.WinUsb_QueryPipe.restype = wintypes.BOOL
        winusb.WinUsb_ReadPipe.argtypes = [c_void_p, c_ubyte, c_void_p, c_ulong, POINTER(c_ulong), POINTER(OVERLAPPED)]
        winusb.WinUsb_ReadPipe.restype = wintypes.BOOL
        winusb.WinUsb_WritePipe.argtypes = [c_void_p, c_ubyte, c_void_p, c_ulong, POINTER(c_ulong), POINTER(OVERLAPPED)]
        winusb.WinUsb_WritePipe.restype = wintypes.BOOL

        device = kernel.CreateFileW(_device_path(), 0xC0000000, 3, None, 3, 0x40000000, None)
        invalid = c_void_p(-1).value
        if not device or device == invalid:
            raise OSError(f"CreateFile failed err={ctypes.get_last_error()}")
        interface = c_void_p()
        if not winusb.WinUsb_Initialize(device, byref(interface)):
            kernel.CloseHandle(device)
            raise OSError(f"WinUsb_Initialize failed err={ctypes.get_last_error()}")
        self._kernel = kernel
        self._winusb = winusb
        self._device = device
        self._interface = interface
        try:
            self._read_event = kernel.CreateEventW(None, True, False, None)
            self._write_event = kernel.CreateEventW(None, True, False, None)
            self._query_pipes()
        except Exception:
            self.close()
            raise

    def _query_pipes(self):
        winusb = self._winusb
        interface = self._interface
        self._in_pipe = 0x81
        self._out_pipe = 0x02
        for index in range(8):
            info = PIPE()
            if not winusb.WinUsb_QueryPipe(interface, 0, index, byref(info)):
                break
            if info.PipeType == 2 and info.PipeId & 0x80:
                self._in_pipe = info.PipeId
            elif info.PipeType == 2:
                self._out_pipe = info.PipeId

    def close(self):
        kernel = self._kernel
        if self._device and kernel:
            kernel.CancelIoEx(self._device, None)
        if self._interface and self._winusb:
            self._winusb.WinUsb_Free(self._interface)
            self._interface = None
        if self._device and kernel:
            kernel.CloseHandle(self._device)
            self._device = None
        for name in ("_read_event", "_write_event"):
            handle = getattr(self, name, None)
            if handle and kernel:
                kernel.CloseHandle(handle)
                setattr(self, name, None)

    def control(self, request_type, request, payload=b"", length=0):
        size = length or len(payload)
        packet = SETUP(request_type, request, 0, 0, size)
        buffer = ctypes.create_string_buffer(max(size, 1))
        if payload:
            buffer.raw = payload
        transferred = c_ulong(0)
        ok = self._winusb.WinUsb_ControlTransfer(
            self._interface, packet, buffer, max(size, 1), byref(transferred), None
        )
        if not ok:
            raise OSError(f"ctrl {request:#x} failed err={ctypes.get_last_error()}")
        return buffer.raw[: transferred.value]

    def read(self, wait_ms=30):
        """Return one raw 20-byte gs_usb frame, or None."""
        overlapped = OVERLAPPED()
        overlapped.hEvent = self._read_event
        self._kernel.ResetEvent(self._read_event)
        transferred = c_ulong(0)
        ok = self._winusb.WinUsb_ReadPipe(
            self._interface, self._in_pipe, self._buffer, 20, byref(transferred), byref(overlapped)
        )
        if not ok:
            if ctypes.get_last_error() != 997:
                return None
            if self._kernel.WaitForSingleObject(self._read_event, wait_ms) != 0:
                self._kernel.CancelIoEx(self._device, ctypes.addressof(overlapped))
                self._kernel.WaitForSingleObject(self._read_event, 50)
                return None
            got = wintypes.DWORD()
            if not self._kernel.GetOverlappedResult(self._device, byref(overlapped), byref(got), False):
                return None
            transferred = c_ulong(got.value)
        return bytes(self._buffer.raw[: transferred.value])

    def write(self, frame):
        """Write one gs_usb frame and wait until the driver finishes with the buffer."""
        self._write_buffer = ctypes.create_string_buffer(frame)
        self._write_overlapped = OVERLAPPED()
        self._write_overlapped.hEvent = self._write_event
        self._kernel.ResetEvent(self._write_event)
        transferred = c_ulong(0)
        ok = self._winusb.WinUsb_WritePipe(
            self._interface,
            self._out_pipe,
            self._write_buffer,
            len(frame),
            byref(transferred),
            byref(self._write_overlapped),
        )
        if not ok and ctypes.get_last_error() != 997:
            raise OSError(f"CAN write failed err={ctypes.get_last_error()}")
        if not ok:
            deadline = time.perf_counter() + 0.5
            finished = False
            while time.perf_counter() < deadline:
                if self._kernel.WaitForSingleObject(self._write_event, 0) == 0:
                    finished = True
                    break
                self.read(wait_ms=5)
            if not finished:
                self._kernel.CancelIoEx(self._device, ctypes.addressof(self._write_overlapped))
                self._kernel.WaitForSingleObject(self._write_event, 200)
                raise OSError("CAN write timed out")
            got = wintypes.DWORD()
            if not self._kernel.GetOverlappedResult(self._device, byref(self._write_overlapped), byref(got), False):
                raise OSError(f"CAN write result failed err={ctypes.get_last_error()}")


class _LibUsbTransport:
    """candleLight through libusb (pyusb). Used on macOS and Linux.

    Feedback arrives at about 2,700 frames/s. If the IN endpoint is not drained
    continuously, the adapter runs out of frame buffers and stops accepting
    OUT transfers (writes time out with errno 60). A background thread reads
    every frame into a bounded queue; read() takes from that queue.
    """

    # About 0.1 s of feedback, so read_state() never sees a stale backlog.
    QUEUE_FRAMES = 256

    def __init__(self):
        self._usb = None
        self._device = None
        self._in_pipe = 0x81
        self._out_pipe = 0x02
        self._frames = None
        self._reader = None
        self._running = False

    def open(self):
        try:
            import usb.core
            import usb.util
        except ImportError as exc:
            raise OSError("pyusb is missing: pip install pyusb, and brew install libusb on macOS") from exc
        device = usb.core.find(idVendor=USB_VENDOR_ID, idProduct=USB_PRODUCT_ID)
        if device is None:
            raise OSError(f"candleLight {USB_VENDOR_ID:04x}:{USB_PRODUCT_ID:04x} not found on USB")
        try:
            if device.is_kernel_driver_active(0):
                device.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        try:
            device.get_active_configuration()
        except usb.core.USBError:
            device.set_configuration()
        usb.util.claim_interface(device, 0)
        interface = device.get_active_configuration()[(0, 0)]
        for endpoint in interface:
            if usb.util.endpoint_type(endpoint.bmAttributes) != usb.util.ENDPOINT_TYPE_BULK:
                continue
            if usb.util.endpoint_direction(endpoint.bEndpointAddress) == usb.util.ENDPOINT_IN:
                self._in_pipe = endpoint.bEndpointAddress
            else:
                self._out_pipe = endpoint.bEndpointAddress
        self._usb = usb
        self._device = device
        self._frames = queue.Queue(maxsize=self.QUEUE_FRAMES)

    def start_reader(self):
        """Begin draining the IN endpoint. Called once the bus is started."""
        self._running = True
        self._reader = threading.Thread(target=self._drain, name="candlelight-reader", daemon=True)
        self._reader.start()

    def _drain(self):
        usb = self._usb
        while self._running:
            try:
                raw = bytes(self._device.read(self._in_pipe, 64, timeout=50))
            except usb.core.USBTimeoutError:
                continue
            except usb.core.USBError:
                if not self._running:
                    break
                time.sleep(0.01)
                continue
            try:
                self._frames.put_nowait(raw)
            except queue.Full:
                # Keep the newest feedback: drop the oldest frame.
                try:
                    self._frames.get_nowait()
                except queue.Empty:
                    pass
                self._frames.put_nowait(raw)

    def close(self):
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=0.5)
            self._reader = None
        if self._device is not None:
            self._usb.util.release_interface(self._device, 0)
            self._usb.util.dispose_resources(self._device)
            self._device = None

    def control(self, request_type, request, payload=b"", length=0):
        try:
            if request_type & 0x80:
                return bytes(self._device.ctrl_transfer(request_type, request, 0, 0, length, timeout=1000))
            self._device.ctrl_transfer(request_type, request, 0, 0, payload, timeout=1000)
            return b""
        except self._usb.core.USBError as exc:
            raise OSError(f"ctrl {request:#x} failed: {exc}") from exc

    def read(self, wait_ms=30):
        """Return one raw 20-byte gs_usb frame, or None."""
        if self._reader is None:
            try:
                return bytes(self._device.read(self._in_pipe, 64, timeout=max(1, int(wait_ms))))
            except self._usb.core.USBError:
                return None
        try:
            return self._frames.get(timeout=max(0.001, wait_ms / 1000))
        except queue.Empty:
            return None

    def write(self, frame):
        try:
            self._device.write(self._out_pipe, frame, timeout=500)
        except self._usb.core.USBError as exc:
            raise OSError(f"CAN write failed: {exc}") from exc


def open_transport():
    """Open the adapter and start the bus at 1 Mbps. The caller closes it."""
    transport = _WinUsbTransport() if sys.platform == "win32" else _LibUsbTransport()
    transport.open()
    try:
        _feature, clock = struct.unpack_from("<2I", transport.control(0xC1, 4, length=40))
        prescaler = 3 if clock == 48_000_000 else 5
        transport.control(0x41, 0, struct.pack("<I", 0x0000BEEF))
        transport.control(0x41, 1, struct.pack("<5I", 1, 12, 2, 1, prescaler))
        transport.control(0x41, 2, struct.pack("<II", 1, 0))
        if hasattr(transport, "start_reader"):
            transport.start_reader()
    except Exception:
        transport.close()
        raise
    transport.clock = clock
    return transport


class PiperClient:
    """Read Piper feedback and command the gripper over candleLight."""

    def __init__(self):
        self._transport = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()

    def connect(self):
        self._transport = open_transport()

    def close(self):
        """Release the adapter. The bus-stop control transfer is not sent; it hung the process."""
        if self._transport is not None:
            self._transport.close()
            self._transport = None

    def read_state(self, timeout=0.6):
        """Return the latest feedback. Missing fields stay None."""
        needed = {0x2A1, 0x2A2, 0x2A3, 0x2A4, 0x2A5, 0x2A6, 0x2A7, 0x2A8}
        frames = {}
        started = time.perf_counter()
        idle_deadline = None
        while time.perf_counter() - started < timeout:
            message = self._read_frame(wait_ms=15)
            if message and message[0] in needed:
                frames[message[0]] = message[1]
                if needed.issubset(frames):
                    idle_deadline = time.perf_counter() + 0.05
            elif idle_deadline is not None and time.perf_counter() > idle_deadline:
                break
        return _decode_state(frames)

    def gripper_max_mm(self, timeout=0.4):
        """Query the configured stroke. Falls back to 70 mm when the reply is absent."""
        self._write_frame(_can_frame(0x477, bytes([0x04, 0x00, 0x00, 0x00, 0x03, 0, 0, 0])))
        started = time.perf_counter()
        while time.perf_counter() - started < timeout:
            message = self._read_frame()
            if message and message[0] == 0x47E and len(message[1]) >= 2 and message[1][1] in (70, 100):
                return float(message[1][1])
        return 70.0

    def move_gripper(self, target_mm, effort=1000, timeout=4.0, tolerance_mm=1.0):
        """Command the gripper only. Effort is 0.001 N·m. Does not change the zero point."""
        if not 0 <= target_mm <= 100:
            raise ValueError(f"gripper target {target_mm} mm is outside 0-100")
        if not 0 <= effort <= 5000:
            raise ValueError(f"gripper effort {effort} is outside 0-5000")
        frame = _can_frame(0x159, struct.pack(">iHBB", int(round(target_mm * 1000)), effort, 0x01, 0))
        latest = None
        started = time.perf_counter()
        while time.perf_counter() - started < timeout:
            self._write_frame(frame)
            deadline = time.perf_counter() + 0.01
            while time.perf_counter() < deadline:
                message = self._read_frame(wait_ms=5)
                if message and message[0] == 0x2A8 and len(message[1]) >= 4:
                    latest = _be32(message[1]) / 1000
            if latest is not None and abs(latest - target_mm) <= tolerance_mm:
                break
        return latest

    def exit_drag_teach(self):
        """Leave drag-teach mode so CAN pose commands are accepted. Does not stop or zero the arm."""
        frame = _can_frame(0x150, bytes([0x00, 0x00, 0x02, 0, 0, 0, 0, 0]))
        for _ in range(5):
            self._write_frame(frame)
            time.sleep(0.02)

    def enable_arm(self):
        """Enable all joint motors. Does not change the zero point."""
        self._write_frame(_can_frame(0x471, bytes([7, 0x02, 0, 0, 0, 0, 0, 0])))

    def set_pose_mode(self, speed=10):
        """CAN control, MOVE P, speed 0-100. Installation byte stays 0."""
        speed = int(max(0, min(100, speed)))
        self._write_frame(_can_frame(0x151, bytes([0x01, 0x00, speed, 0x00, 0, 0, 0, 0])))

    def set_joint_mode(self, speed=10):
        """CAN control, MOVE J, speed 0-100. Installation byte stays 0."""
        speed = int(max(0, min(100, speed)))
        self._write_frame(_can_frame(0x151, bytes([0x01, 0x01, speed, 0x00, 0, 0, 0, 0])))

    def move_joints(
        self, joints_deg, duration=0.05, speed=30, gripper_mm=None, start_deg=None, update_hz=100.0, tolerance_deg=2.0
    ):
        """Send six absolute joint angles (degrees) through 0x155-0x157 for duration seconds.

        With start_deg, the target moves from start_deg to joints_deg over
        duration, changing update_hz times per second (frames are resent every
        few ms either way). MOVE J plans to each new target, so update_hz=20
        with 0.05 s steps holds one target per step and the heavy base joints
        start and stop on every step; around 100 Hz the motion is continuous.

        Raises before sending if a target is more than tolerance_deg outside
        JOINT_LIMITS_DEG; smaller excursions are clamped to the limit.
        """
        end = self._checked_joints(joints_deg, tolerance_deg)
        begin = end if start_deg is None else self._checked_joints(start_deg, tolerance_deg)
        gripper_frame = None
        if gripper_mm is not None:
            gripper_frame = _can_frame(0x159, struct.pack(">iHBB", int(round(gripper_mm * 1000)), 1000, 0x01, 0))
        self.set_joint_mode(speed=speed)
        period = 1.0 / max(1.0, float(update_hz))
        started = time.perf_counter()
        last_mode = started
        while True:
            now = time.perf_counter()
            elapsed = now - started
            if elapsed >= duration:
                break
            if now - last_mode >= 0.05:
                self.set_joint_mode(speed=speed)
                last_mode = now
            # The target advances once per update period, to where it should be at the end of that period.
            alpha = min(1.0, (math.floor(elapsed / period) + 1) * period / duration) if duration > 0 else 1.0
            raw = [int(round((b + (e - b) * alpha) * 1000)) for b, e in zip(begin, end)]
            self._write_frame(_can_frame(0x155, struct.pack(">ii", raw[0], raw[1])))
            self._write_frame(_can_frame(0x156, struct.pack(">ii", raw[2], raw[3])))
            self._write_frame(_can_frame(0x157, struct.pack(">ii", raw[4], raw[5])))
            if gripper_frame is not None:
                self._write_frame(gripper_frame)
            time.sleep(0.003)

    @staticmethod
    def _checked_joints(joints_deg, tolerance_deg):
        if len(joints_deg) != 6:
            raise ValueError("move_joints needs six angles")
        targets = []
        for index, (value, (low, high)) in enumerate(zip(joints_deg, JOINT_LIMITS_DEG)):
            if not low - tolerance_deg <= value <= high + tolerance_deg:
                raise ValueError(f"J{index + 1}={value:.1f} deg is outside [{low}, {high}]")
            targets.append(min(high, max(low, float(value))))
        return targets

    def move_pose(self, x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg, duration=1.5, speed=50, gripper_mm=None):
        """Send one absolute end pose in the arm base frame. Units are mm and degrees."""
        xy = struct.pack(">ii", int(round(x_mm * 1000)), int(round(y_mm * 1000)))
        zrx = struct.pack(">ii", int(round(z_mm * 1000)), int(round(rx_deg * 1000)))
        ryrz = struct.pack(">ii", int(round(ry_deg * 1000)), int(round(rz_deg * 1000)))
        frames = [
            _can_frame(0x152, xy),
            _can_frame(0x153, zrx),
            _can_frame(0x154, ryrz),
        ]
        if gripper_mm is not None:
            frames.append(_can_frame(0x159, struct.pack(">iHBB", int(round(gripper_mm * 1000)), 1000, 0x01, 0)))
        started = time.perf_counter()
        while time.perf_counter() - started < duration:
            self.set_pose_mode(speed=speed)
            for frame in frames:
                self._write_frame(frame)
            time.sleep(0.01)

    def _read_frame(self, wait_ms=30):
        raw = self._transport.read(wait_ms=wait_ms)
        if raw is None or len(raw) < 20:
            return None
        echo, can_id, dlc = struct.unpack_from("<IIB", raw)
        if echo != 0xFFFFFFFF or can_id & 0x20000000:
            return None
        return can_id & 0x7FF, raw[12:12 + min(dlc, 8)]

    def _write_frame(self, frame):
        """Write one frame, then drain the transmit echo so later writes do not stall."""
        self._transport.write(frame)
        for _ in range(4):
            if self._transport.read(wait_ms=2) is None:
                break
        return True


def _decode_state(frames):
    state = {
        "joints_deg": None,
        "gripper_mm": None,
        "gripper_effort": None,
        "gripper_status": None,
        "pose_mm_deg": None,
        "status": None,
        "complete": False,
    }
    if 0x2A5 in frames and 0x2A6 in frames and 0x2A7 in frames:
        state["joints_deg"] = [
            _be32(frames[0x2A5][0:4]) / 1000,
            _be32(frames[0x2A5][4:8]) / 1000,
            _be32(frames[0x2A6][0:4]) / 1000,
            _be32(frames[0x2A6][4:8]) / 1000,
            _be32(frames[0x2A7][0:4]) / 1000,
            _be32(frames[0x2A7][4:8]) / 1000,
        ]
    if 0x2A8 in frames and len(frames[0x2A8]) >= 7:
        gripper = frames[0x2A8]
        state["gripper_mm"] = _be32(gripper[0:4]) / 1000
        state["gripper_effort"] = _be16(gripper[4:6])
        state["gripper_status"] = gripper[6]
    if 0x2A2 in frames and 0x2A3 in frames and 0x2A4 in frames:
        state["pose_mm_deg"] = {
            "x": _be32(frames[0x2A2][0:4]) / 1000,
            "y": _be32(frames[0x2A2][4:8]) / 1000,
            "z": _be32(frames[0x2A3][0:4]) / 1000,
            "rx": _be32(frames[0x2A3][4:8]) / 1000,
            "ry": _be32(frames[0x2A4][0:4]) / 1000,
            "rz": _be32(frames[0x2A4][4:8]) / 1000,
        }
    if 0x2A1 in frames and len(frames[0x2A1]) >= 8:
        raw = frames[0x2A1]
        state["status"] = {
            "ctrl_mode": raw[0],
            "ctrl_mode_name": CTRL_MODE.get(raw[0], str(raw[0])),
            "arm_status": raw[1],
            "arm_status_name": ARM_STATUS.get(raw[1], str(raw[1])),
            "mode_feed": raw[2],
            "mode_feed_name": MODE_FEED.get(raw[2], str(raw[2])),
            "teach_status": raw[3],
            "motion_status": raw[4],
            "trajectory_num": raw[5],
            "err_code": int.from_bytes(raw[6:8], "big"),
        }
    state["complete"] = all(state[key] is not None for key in ("joints_deg", "gripper_mm", "pose_mm_deg", "status"))
    return state


def format_state(state):
    lines = []
    if state["joints_deg"] is None:
        lines.append("joints_deg=missing")
    else:
        lines.append("joints_deg=" + ", ".join(f"J{index + 1}:{value:.3f}" for index, value in enumerate(state["joints_deg"])))
    if state["gripper_mm"] is None:
        lines.append("gripper=missing")
    else:
        lines.append(
            f"gripper_mm={state['gripper_mm']:.3f} effort={state['gripper_effort']} status=0x{state['gripper_status']:02x}"
        )
    pose = state["pose_mm_deg"]
    if pose is None:
        lines.append("pose=missing")
    else:
        lines.append(
            "pose_mm_deg="
            + ", ".join(f"{name}:{pose[name]:.3f}" for name in ("x", "y", "z", "rx", "ry", "rz"))
        )
    status = state["status"]
    if status is None:
        lines.append("status=missing")
    else:
        lines.append(
            "status="
            f"{status['ctrl_mode_name']} {status['arm_status_name']} {status['mode_feed_name']} "
            f"teach={status['teach_status']} motion={status['motion_status']} err={status['err_code']}"
        )
    lines.append(f"complete={state['complete']}")
    return "\n".join(lines)


def main():
    with PiperClient() as robot:
        before = robot.read_state()
        print("before")
        print(format_state(before))
        if before["gripper_mm"] is None:
            raise RuntimeError("gripper feedback missing; no motion sent")
        maximum = robot.gripper_max_mm()
        print(f"target_open_mm={maximum:.1f}")
        opened = robot.move_gripper(maximum)
        print(f"opened_mm={opened:.3f}" if opened is not None else "opened_mm=missing")
        closed = robot.move_gripper(0.0)
        print(f"closed_mm={closed:.3f}" if closed is not None else "closed_mm=missing")
        after = robot.read_state()
        print("after")
        print(format_state(after))


if __name__ == "__main__":
    main()
