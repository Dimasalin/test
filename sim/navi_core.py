"""ctypes binding to the firmware core (firmware/core/*.c).

The simulator does not re-implement the autopilot: it compiles the same C
sources that go onto the ESP32 into a shared library and calls navi_step().
"""
import ctypes as C
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CORE = ROOT / "firmware" / "core"
BUILD = ROOT / "sim" / "_build"
SOURCES = ["navi_est.c", "navi_ctrl.c", "navi_behavior.c", "msp.c"]

F, I = C.c_float, C.c_int32

LANDED, TAKEOFF, FOLLOW, TARGET, ATTENTION, LAND, FAILSAFE = range(7)
STATE_NAMES = ["LANDED", "TAKEOFF", "FOLLOW", "TARGET", "ATTENTION", "LAND", "FAILSAFE"]
CMD_NONE, CMD_TAKEOFF, CMD_LAND, CMD_ATTENTION, CMD_TARGET, CMD_RELEASE, CMD_KILL = range(7)
FAULT_ATT, FAULT_FLOW, FAULT_TOF, FAULT_UWB, FAULT_VBAT = 1, 2, 4, 8, 16


class In(C.Structure):
    _fields_ = [
        ("dt", F), ("roll", F), ("pitch", F), ("yaw", F),
        ("gyro_x", F), ("gyro_y", F), ("att_fresh", I),
        ("flow_x", F), ("flow_y", F), ("flow_quality", I), ("flow_fresh", I),
        ("tof_range", F), ("tof_fresh", I),
        ("uwb_range", F), ("uwb_fresh", I),
        ("vbat", F), ("cmd", I), ("cmd_x", F), ("cmd_y", F), ("cmd_z", F),
    ]


class Out(C.Structure):
    _fields_ = [
        ("rc_roll", F), ("rc_pitch", F), ("rc_yaw", F), ("rc_throttle", F), ("arm", I),
        ("led_r", I), ("led_g", I), ("led_b", I), ("buzzer_hz", F),
        ("state", I), ("fault", I),
        ("est_x", F), ("est_y", F), ("est_z", F),
        ("est_vx", F), ("est_vy", F), ("est_vz", F),
        ("tgt_x", F), ("tgt_y", F), ("tgt_vx", F), ("tgt_vy", F), ("tgt_sigma", F),
        ("sp_x", F), ("sp_y", F), ("sp_z", F), ("hover_thr", F),
    ]


class Cfg(C.Structure):
    _fields_ = [
        ("bf_angle_limit", F), ("hover_thr_init", F), ("thr_min", F), ("thr_max", F),
        ("kp_pos", F), ("kv_pos", F), ("ki_vel", F), ("acc_max", F),
        ("kp_z", F), ("kv_z", F), ("ki_hover", F), ("vz_max", F),
        ("flow_lpf", F), ("flow_min_quality", I),
        ("z_alpha", F), ("z_beta", F), ("floor_step", F),
        ("uwb_enabled", I), ("uwb_sigma", F), ("tgt_acc_sigma", F), ("tgt_vel_tau", F),
        ("tgt_init_x", F), ("tgt_init_y", F), ("tgt_init_sigma", F), ("beacon_z", F),
        ("hover_z", F), ("safe_z", F), ("z_max", F), ("orbit_radius", F), ("keepout_radius", F),
        ("leash", F), ("v_follow", F), ("v_dart", F),
        ("attention_secs", F), ("target_secs", F), ("idle_attention_secs", F),
        ("lead_enabled", I), ("vel_ff", F),
        ("vbat_warn", F), ("vbat_land", F),
    ]


_lib = None


def lib():
    """Build (if stale) and load the core as a shared library."""
    global _lib
    if _lib is not None:
        return _lib
    BUILD.mkdir(exist_ok=True)
    so = BUILD / "libnavi.so"
    srcs = [CORE / s for s in SOURCES]
    deps = srcs + list(CORE.glob("*.h"))
    if not so.exists() or so.stat().st_mtime < max(p.stat().st_mtime for p in deps):
        subprocess.check_call(
            ["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-fPIC", "-shared",
             "-o", str(so)] + [str(s) for s in srcs] + ["-lm"])
    l = C.CDLL(str(so))
    l.navi_sizeof.restype = C.c_uint32
    l.navi_sizeof_cfg.restype = C.c_uint32
    assert l.navi_sizeof_cfg() == C.sizeof(Cfg), "Cfg mirror out of sync with navi.h"
    _lib = l
    return l


class Navi:
    def __init__(self, **overrides):
        l = lib()
        self.cfg = Cfg()
        l.navi_default_cfg(C.byref(self.cfg))
        for k, v in overrides.items():
            if not hasattr(self.cfg, k):
                raise AttributeError(k)
            setattr(self.cfg, k, v)
        self._mem = C.create_string_buffer(l.navi_sizeof())
        l.navi_init(self._mem, C.byref(self.cfg))
        self.out = Out()

    def step(self, inp: In) -> Out:
        lib().navi_step(self._mem, C.byref(inp), C.byref(self.out))
        return self.out
