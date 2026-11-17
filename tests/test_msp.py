"""MSP codec tests (firmware/core/msp.c)."""
import ctypes as C
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "sim"))
import navi_core as nc   # noqa: E402


class Parser(C.Structure):
    _fields_ = [("state", C.c_uint8), ("v2", C.c_uint8), ("crc", C.c_uint8),
                ("cmd", C.c_uint16), ("size", C.c_uint16), ("idx", C.c_uint16),
                ("payload", C.c_uint8 * 64)]


def parse(data):
    lib, p, frames = nc.lib(), Parser(), []
    for b in data:
        if lib.msp_parse(C.byref(p), C.c_uint8(b)):
            frames.append((p.cmd, bytes(p.payload[:p.size])))
    return frames


def encode_rc(ch):
    buf = (C.c_uint8 * 64)()
    n = nc.lib().msp_encode_rc((C.c_uint16 * len(ch))(*ch), len(ch), buf, 64)
    return bytes(buf[:n])


def test_v1_request_matches_reference_bytes():
    buf = (C.c_uint8 * 16)()
    n = nc.lib().msp_encode_v1(108, None, 0, buf, 16)
    # "$M<", size 0, cmd 108, checksum = 0 ^ 108
    assert bytes(buf[:n]) == b"$M<\x00\x6c\x6c"


def test_set_raw_rc_frame():
    f = encode_rc([1500, 1500, 1000, 1500])
    assert f[:5] == b"$M<\x08\xc8"
    assert struct.unpack("<4H", f[5:13]) == (1500, 1500, 1000, 1500)
    x = 0
    for b in f[3:-1]:
        x ^= b
    assert f[-1] == x
    assert parse(f) == [(200, f[5:13])]


def test_v2_sensor_frames_roundtrip_with_noise():
    lib = nc.lib()
    rng = struct.pack("<Bi", 255, 1234)          # quality, distance mm
    flow = struct.pack("<Bii", 180, -42, 17)     # quality, motion x, y
    buf = (C.c_uint8 * 64)()
    stream = b"\x00garbage$X"
    for cmd, pl in ((0x1F01, rng), (0x1F02, flow)):
        n = lib.msp_encode_v2(cmd, pl, len(pl), buf, 64)
        stream += bytes(buf[:n]) + b"\xff$"
    assert parse(stream) == [(0x1F01, rng), (0x1F02, flow)]


def test_corrupt_frame_is_dropped():
    f = bytearray(encode_rc([1500, 1500, 1000, 1500]))
    f[7] ^= 0x10
    assert parse(bytes(f)) == []


def test_crc8_dvb_s2_known_value():
    lib = nc.lib()
    lib.msp_crc8_dvb_s2.restype = C.c_uint8
    crc = 0
    for b in b"123456789":
        crc = lib.msp_crc8_dvb_s2(C.c_uint8(crc), C.c_uint8(b))
    assert crc == 0xBC      # CRC-8/DVB-S2 check value


def test_stick_scaling():
    lib = nc.lib()
    lib.msp_stick_us.restype = C.c_uint16
    lib.msp_throttle_us.restype = C.c_uint16
    lib.msp_stick_us.argtypes = [C.c_float]
    lib.msp_throttle_us.argtypes = [C.c_float]
    assert [lib.msp_stick_us(v) for v in (-2, -1, 0, 1, 2)] == [1000, 1000, 1500, 2000, 2000]
    assert [lib.msp_throttle_us(v) for v in (-1, 0, 0.5, 1, 3)] == [1000, 1000, 1500, 2000, 2000]
