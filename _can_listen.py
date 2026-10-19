"""Passive candleLight capture. Opens the adapter only; sends no CAN frames."""
import ctypes
import struct
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

OUT = str(__import__("pathlib").Path(__file__).with_name("_can_listen_log.txt"))
SECONDS = 25
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x7FF


class GUID(Structure):
    _fields_ = [
        ("Data1", wintypes.DWORD),
        ("Data2", wintypes.WORD),
        ("Data3", wintypes.WORD),
        ("Data4", c_byte * 8),
    ]


class IFACE(Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("InterfaceClassGuid", GUID),
        ("Flags", wintypes.DWORD),
        ("Reserved", c_void_p),
    ]


class DETAIL(Structure):
    _fields_ = [("cbSize", wintypes.DWORD), ("DevicePath", wintypes.WCHAR * 1024)]


class SETUP(Structure):
    _fields_ = [
        ("RequestType", c_ubyte),
        ("Request", c_ubyte),
        ("Value", c_ushort),
        ("Index", c_ushort),
        ("Length", c_ushort),
    ]


class PIPE(Structure):
    _fields_ = [
        ("PipeType", c_ulong),
        ("PipeId", c_ubyte),
        ("MaximumPacketSize", c_ushort),
        ("Interval", c_ubyte),
    ]


class OVERLAPPED(Structure):
    _fields_ = [
        ("Internal", c_void_p),
        ("InternalHigh", c_void_p),
        ("Offset", wintypes.DWORD),
        ("OffsetHigh", wintypes.DWORD),
        ("hEvent", c_void_p),
    ]


def device_path():
    u = UUID("{c15b4308-04d3-11e6-b3ea-6057189e6443}")
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
    guid.Data1, guid.Data2, guid.Data3 = u.time_low, u.time_mid, u.time_hi_version
    for i, b in enumerate(u.bytes[8:]):
        guid.Data4[i] = b
    handle = setup.SetupDiGetClassDevsW(byref(guid), None, None, 0x12)
    iface = IFACE()
    iface.cbSize = sizeof(IFACE)
    if not setup.SetupDiEnumDeviceInterfaces(handle, None, byref(guid), 0, byref(iface)):
        raise OSError(f"candleLight interface missing err={ctypes.get_last_error()}")
    req = wintypes.DWORD()
    setup.SetupDiGetDeviceInterfaceDetailW(handle, byref(iface), None, 0, byref(req), None)
    detail = DETAIL()
    detail.cbSize = 8
    if not setup.SetupDiGetDeviceInterfaceDetailW(handle, byref(iface), byref(detail), sizeof(detail), byref(req), None):
        raise OSError(f"device path failed err={ctypes.get_last_error()}")
    setup.SetupDiDestroyDeviceInfoList(handle)
    return detail.DevicePath


def main():
    path = device_path()
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
    winusb.WinUsb_Initialize.argtypes = [c_void_p, POINTER(c_void_p)]
    winusb.WinUsb_Initialize.restype = wintypes.BOOL
    winusb.WinUsb_Free.argtypes = [c_void_p]
    winusb.WinUsb_ControlTransfer.argtypes = [c_void_p, SETUP, c_void_p, c_ulong, POINTER(c_ulong), c_void_p]
    winusb.WinUsb_ControlTransfer.restype = wintypes.BOOL
    winusb.WinUsb_QueryPipe.argtypes = [c_void_p, c_ubyte, c_ubyte, POINTER(PIPE)]
    winusb.WinUsb_QueryPipe.restype = wintypes.BOOL
    winusb.WinUsb_ReadPipe.argtypes = [c_void_p, c_ubyte, c_void_p, c_ulong, POINTER(c_ulong), c_void_p]
    winusb.WinUsb_ReadPipe.restype = wintypes.BOOL

    dev = kernel.CreateFileW(path, 0xC0000000, 3, None, 3, 0x40000000, None)
    invalid = c_void_p(-1).value
    if not dev or dev == invalid:
        raise OSError(f"CreateFile failed err={ctypes.get_last_error()}")
    wh = c_void_p()
    if not winusb.WinUsb_Initialize(dev, byref(wh)):
        kernel.CloseHandle(dev)
        raise OSError(f"WinUsb_Initialize failed err={ctypes.get_last_error()}")

    def ctrl(req_type, request, payload=b"", length=0):
        n = length or len(payload)
        packet = SETUP(req_type, request, 0, 0, n)
        buf = ctypes.create_string_buffer(max(n, 1))
        if payload:
            buf.raw = payload
        transferred = c_ulong(0)
        if not winusb.WinUsb_ControlTransfer(wh, packet, buf, max(n, 1), byref(transferred), None):
            raise OSError(f"ctrl {request:#x} failed err={ctypes.get_last_error()}")
        return buf.raw[: transferred.value]

    try:
        in_pipe = 0x81
        for idx in range(8):
            info = PIPE()
            if not winusb.WinUsb_QueryPipe(wh, 0, idx, byref(info)):
                break
            if info.PipeType == 2 and info.PipeId & 0x80:
                in_pipe = info.PipeId
        _feature, fclk = struct.unpack_from("<2I", ctrl(0xC1, 4, length=40))
        timing = (1, 12, 2, 1, 3 if fclk == 48_000_000 else 5)
        ctrl(0x41, 0, struct.pack("<I", 0x0000BEEF))
        ctrl(0x41, 1, struct.pack("<5I", *timing))
        ctrl(0x41, 2, struct.pack("<II", 1, 0))
        event = kernel.CreateEventW(None, True, False, None)
        buf = ctypes.create_string_buffer(64)
        t0 = time.perf_counter()
        rows = []
        read_errors = {}
        while time.perf_counter() - t0 < SECONDS:
            ov = OVERLAPPED()
            ov.hEvent = event
            kernel.ResetEvent(event)
            transferred = c_ulong(0)
            ok = winusb.WinUsb_ReadPipe(wh, in_pipe, buf, 20, byref(transferred), byref(ov))
            if not ok:
                err = ctypes.get_last_error()
                if err == 997:
                    if kernel.WaitForSingleObject(event, 200) != 0:
                        continue
                    got = wintypes.DWORD()
                    if not kernel.GetOverlappedResult(dev, byref(ov), byref(got), False):
                        read_errors[ctypes.get_last_error()] = read_errors.get(ctypes.get_last_error(), 0) + 1
                        continue
                    transferred = c_ulong(got.value)
                else:
                    read_errors[err] = read_errors.get(err, 0) + 1
                    continue
            raw = bytes(buf.raw[: transferred.value])
            ts = time.perf_counter() - t0
            if len(raw) < 20:
                rows.append(f"{ts:.6f}\tSHORT\t{raw.hex()}")
                continue
            echo, can_id, dlc, channel, flags, _reserved = struct.unpack_from("<IIBBBB", raw)
            extended = bool(can_id & CAN_EFF_FLAG)
            remote = bool(can_id & CAN_RTR_FLAG)
            error = bool(can_id & CAN_ERR_FLAG)
            arb = can_id & (CAN_EFF_MASK if extended else CAN_SFF_MASK)
            is_rx = echo == 0xFFFFFFFF
            data = raw[12:12 + min(dlc, 8)]
            rows.append(
                f"{ts:.6f}\t{arb:#x}\t{int(extended)}\t{int(remote)}\t{int(error)}\t{int(is_rx)}\t{echo:#x}\t{dlc}\t{channel}\t{flags:#x}\t{data.hex()}"
            )
        try:
            ctrl(0x41, 2, struct.pack("<II", 0, 0))
        except OSError as exc:
            print(exc)
        header = "timestamp_s\tarbitration_id\tis_extended_id\tis_remote_frame\tis_error_frame\tis_rx\techo_id\tdlc\tchannel\tflags\tdata"
        with open(OUT, "w", encoding="utf-8") as f:
            f.write(f"# path={path}\n")
            f.write(f"# fclk={fclk} bitrate=1000000 seconds={SECONDS} frames={len(rows)} read_errors={read_errors}\n")
            f.write(f"# no CAN transmit\n")
            f.write(header + "\n")
            f.write("\n".join(rows))
            if rows:
                f.write("\n")
        print(f"frames={len(rows)} read_errors={read_errors} log={OUT}")
    finally:
        winusb.WinUsb_Free(wh)
        kernel.CloseHandle(dev)


if __name__ == "__main__":
    main()
