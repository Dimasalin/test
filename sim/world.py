"""Physics, sensors and the person being followed.

Truth model
  * 6-DOF rigid body, four motors with first-order lag and saturation.
  * An emulated Betaflight-style angle mode (angle P -> rate P -> mixer), so
    the companion code sees realistic attitude lag and thrust coupling.
  * 1S battery with internal resistance; thrust falls as voltage sags.
  * Linear drag against a gusting indoor draught.

Sensor models (all delayed by one 20 ms control tick)
  * optical flow: angular rate incl. body-rotation term, noise, scale error,
    random quality drop-outs
  * time-of-flight range: slant range to whatever is underneath, noise
  * UWB range to the beacon: noise, bias, non-line-of-sight spikes, drop-outs
  * attitude/gyro from the flight controller, with slow yaw drift
"""
from dataclasses import dataclass, field
import math
import numpy as np

import navi_core as nc

G = 9.80665
HEAD_CLEAR_Z = 1.85   # m: above this the drone passes over a standing person


@dataclass
class Airframe:
    """75 mm ducted 1S whoop + companion payload. See docs/DESIGN.md."""
    mass: float = 0.056            # kg, all-up
    thrust_max: float = 0.40       # N per motor at 4.2 V (vendor ~40-46 g; derated)
    arm: float = 0.0265            # m, motor offset in x and y (75 mm diagonal)
    J: tuple = (2.1e-5, 2.1e-5, 3.6e-5)   # kg m^2 (estimate)
    kq: float = 0.006              # m, yaw torque per unit thrust
    motor_tau: float = 0.030       # s
    drag: float = 0.45             # 1/s, linear drag / mass (ducts are draggy)
    # battery: 1S 550 mAh
    capacity_mah: float = 550.0
    r_int: float = 0.070           # ohm, cell + connector + wiring
    hover_amps: float = 5.2        # A at nominal mass (estimate from flight times)
    # emulated flight controller
    angle_limit: float = math.radians(30)
    k_angle: float = 6.0
    k_rate: float = 30.0


def rot_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def euler_from_rot(R):
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def _expm_so3(w):
    th = float(np.linalg.norm(w))
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if th < 1e-9:
        return np.eye(3) + K
    return np.eye(3) + math.sin(th) / th * K + (1 - math.cos(th)) / th**2 * (K @ K)


class Quad:
    SPIN = np.array([1.0, -1.0, 1.0, -1.0])

    def __init__(self, af: Airframe, soc=1.0):
        self.af = af
        a = af.arm
        self.mx = np.array([a, -a, -a, a])
        self.my = np.array([a, a, -a, -a])
        self.J = np.array(af.J)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.R = np.eye(3)
        self.w = np.zeros(3)
        self.thrust = np.zeros(4)
        self.soc = soc
        self.vbat = self.ocv()
        self.amps = 0.0
        self.sat_time = 0.0
        self.yaw_rate_bias = 0.0
        self.touchdown_speed = 0.0

    def ocv(self):
        s = max(0.0, min(1.0, self.soc))
        if s > 0.1:
            return 3.55 + 0.65 * (s - 0.1) / 0.9
        return 3.0 + 0.55 * s / 0.1

    def step(self, dt, rc, armed, wind, floor_z):
        af = self.af
        roll, pitch, _ = euler_from_rot(self.R)
        tmax = af.thrust_max * (self.vbat / 4.2) ** 2

        if armed:
            # --- emulated angle mode ---
            w_sp = np.array([
                af.k_angle * (rc[0] * af.angle_limit - roll),
                af.k_angle * (rc[1] * af.angle_limit - pitch),
                rc[2] * math.radians(200) + self.yaw_rate_bias,
            ])
            w_sp = np.clip(w_sp, -8.0, 8.0)
            tau = self.J * af.k_rate * (w_sp - self.w)
            coll = 4.0 * tmax * rc[3] ** 2
            cmd = (coll / 4.0 + tau[0] / (4 * self.my) - tau[1] / (4 * self.mx)
                   + tau[2] * self.SPIN / (4 * af.kq))
            if rc[3] < 0.02:
                cmd = np.zeros(4)
            if np.any(cmd > tmax) or (rc[3] > 0.05 and np.any(cmd < 0)):
                self.sat_time += dt
            cmd = np.clip(cmd, 0.0, tmax)
        else:
            cmd = np.zeros(4)
        self.thrust += (cmd - self.thrust) * min(1.0, dt / af.motor_tau)

        T = float(self.thrust.sum())
        tau_b = np.array([
            float(self.my @ self.thrust),
            float(-self.mx @ self.thrust),
            float(af.kq * (self.SPIN @ self.thrust)),
        ])
        acc = self.R[:, 2] * (T / af.mass) - np.array([0, 0, G]) \
            - af.drag * (self.vel - wind)
        self.w += dt * (tau_b - np.cross(self.w, self.J * self.w)) / self.J
        self.R = self.R @ _expm_so3(self.w * dt)
        self.vel += acc * dt
        self.pos += self.vel * dt

        if self.pos[2] <= floor_z:
            self.pos[2] = floor_z
            if self.vel[2] < 0:
                if self.vel[2] < -0.05:
                    self.touchdown_speed = max(self.touchdown_speed, -self.vel[2])
                self.vel[:] = 0.0
            if T < af.mass * G:
                yaw = math.atan2(self.R[1, 0], self.R[0, 0])
                self.R = rot_from_euler(0, 0, yaw)
                self.w[:] = 0.0
                self.vel[:2] = 0.0

        # --- battery ---
        # hover_amps is quoted at the nominal 56 g; induced power ~ T^1.5
        self.amps = 0.15 + af.hover_amps * (T / (0.056 * G)) ** 1.5
        self.soc -= self.amps * dt / 3600.0 / (af.capacity_mah / 1000.0)
        self.vbat = self.ocv() - self.amps * af.r_int


class Person:
    """Walks through timed waypoints: [(t_arrive_start, (x, y)), ...].
    Between entries they walk in a straight line at `speed`, then wait."""

    def __init__(self, start, legs=(), speed=1.1, beacon_z=1.25):
        self.p = np.array(start, float)
        self.cmd = self.p.copy()
        self.legs = sorted(legs, key=lambda l: l[0])
        self.speed = speed
        self.beacon_z = beacon_z
        self.goal = self.p.copy()
        self.vel = np.zeros(2)

    def step(self, t, dt):
        for t0, wp in self.legs:
            if t >= t0:
                self.goal = np.array(wp, float)
        d = self.goal - self.cmd
        dist = float(np.linalg.norm(d))
        stepmax = self.speed * dt
        self.cmd = self.goal.copy() if dist <= stepmax else self.cmd + d / dist * stepmax
        new = self.p + (self.cmd - self.p) * min(1.0, dt / 0.35)   # human-ish accel
        self.vel = (new - self.p) / dt
        self.p = new


@dataclass
class Scenario:
    name: str = "story"
    duration: float = 86.0
    seed: int = 1
    person_start: tuple = (-0.7, 0.0)
    legs: tuple = ()
    commands: tuple = ()            # (t, cmd, x, y, z)
    airframe: Airframe = field(default_factory=Airframe)
    cfg: dict = field(default_factory=dict)
    soc0: float = 1.0
    flow_scale: float = 1.0
    flow_dead_after: float = 1e9    # s: optical flow quality drops to zero
    uwb_dead: tuple = (1e9, 1e9)    # (from, to) s: beacon silent
    table: tuple = ()               # (x0, x1, y0, y1, height)
    wind_sigma: float = 0.12
    pad: tuple = ()                 # (x, y) of the charging pad, if any
    pad_dead_after: float = 1e9     # s: pad beacon goes silent


def story(seed=1, **kw):
    """Stand, walk a loop of the room, get nagged, inspect something, walk
    back, land."""
    return Scenario(
        name="story", seed=seed,
        legs=((13.0, (4.0, 0.0)), (18.5, (4.0, 3.0)), (22.5, (0.5, 3.0)),
              (52.0, (0.5, 0.5))),
        commands=((1.0, nc.CMD_TAKEOFF, 0, 0, 0),
                  (32.0, nc.CMD_ATTENTION, 0, 0, 0),
                  (38.0, nc.CMD_TARGET, 2.6, 1.6, 1.0),
                  (74.0, nc.CMD_LAND, 0, 0, 0)),
        **kw)


class Sim:
    PHYS_DT = 0.002
    CTRL_DT = 0.02

    def __init__(self, sc: Scenario):
        self.sc = sc
        self.rng = np.random.default_rng(sc.seed)
        self.quad = Quad(sc.airframe, soc=sc.soc0)
        self.quad.yaw_rate_bias = math.radians(self.rng.normal(0, 0.15))
        self.person = Person(sc.person_start, sc.legs)
        cfg = dict(sc.cfg)
        if sc.pad:
            cfg.setdefault("pad_enabled", 1)
        self.navi = nc.Navi(**cfg)
        self.pad_bias = self.rng.normal(0.03, 0.03)
        self.wind = np.zeros(3)
        self.rc = np.zeros(4)
        self.armed = False
        self.t = 0.0
        self.tick = 0
        self.yaw_ref = 0.0          # what the FC believes yaw is
        self.uwb_bias = self.rng.normal(0.03, 0.03)
        self.pending = None         # one-tick sensor delay
        self.cmds = sorted(sc.commands, key=lambda c: c[0])
        self.log = {k: [] for k in (
            "t pos vel rpy est tgt_est person sp state fault led buzzer rc vbat "
            "amps sigma hover_thr wind yaw_ref pad_est pad_sigma home_phase").split()}

    def floor(self, x, y):
        tb = self.sc.table
        if tb and tb[0] <= x <= tb[1] and tb[2] <= y <= tb[3]:
            return tb[4]
        return 0.0

    def _sense(self):
        q, sc, rng = self.quad, self.sc, self.rng
        roll, pitch, _ = euler_from_rot(q.R)
        s = nc.In()
        s.dt = self.CTRL_DT
        s.roll = roll + rng.normal(0, 0.003)
        s.pitch = pitch + rng.normal(0, 0.003)
        s.yaw = self.yaw_ref
        s.gyro_x = q.w[0] + rng.normal(0, 0.02)
        s.gyro_y = q.w[1] + rng.normal(0, 0.02)
        s.att_fresh = 1

        agl = q.pos[2] - self.floor(q.pos[0], q.pos[1])
        tilt = max(0.3, q.R[2, 2])
        slant = agl / tilt + 0.02            # sensor sits ~2 cm above the skids
        s.tof_range = slant * (1 + rng.normal(0, 0.012)) + rng.normal(0, 0.006)
        s.tof_fresh = 1
        if slant > 7.5:
            s.tof_range = -1.0

        vb = q.R.T @ q.vel
        s.flow_x = (vb[0] / slant - q.w[1]) * sc.flow_scale + rng.normal(0, 0.05)
        s.flow_y = (vb[1] / slant + q.w[0]) * sc.flow_scale + rng.normal(0, 0.05)
        s.flow_quality = 0 if (rng.random() < 0.03 or self.t > sc.flow_dead_after) \
            else int(rng.integers(90, 200))
        s.flow_fresh = 1

        s.uwb_fresh = 0
        if self.tick % 5 == 0 and not (sc.uwb_dead[0] <= self.t < sc.uwb_dead[1]) \
                and rng.random() > 0.03:
            d = math.sqrt((q.pos[0] - self.person.p[0]) ** 2 +
                          (q.pos[1] - self.person.p[1]) ** 2 +
                          (q.pos[2] - self.person.beacon_z) ** 2)
            r = d + self.uwb_bias + rng.normal(0, 0.08)
            if rng.random() < 0.04:
                r += rng.uniform(0.3, 1.0)   # body-blocked path
            s.uwb_range = r
            s.uwb_fresh = 1
        s.pad_fresh = 0
        if sc.pad and self.tick % 5 == 2 and self.t < sc.pad_dead_after \
                and rng.random() > 0.03:
            d = math.sqrt((q.pos[0] - sc.pad[0]) ** 2 + (q.pos[1] - sc.pad[1]) ** 2 +
                          (q.pos[2] - self.navi.cfg.pad_z) ** 2)
            r = d + self.pad_bias + rng.normal(0, 0.08)
            if rng.random() < 0.02:
                r += rng.uniform(0.3, 1.0)
            s.pad_range = r
            s.pad_fresh = 1
        s.vbat = q.vbat + rng.normal(0, 0.01)
        return s

    def step(self):
        sc, q = self.sc, self.quad
        s = self._sense()
        inp = self.pending if self.pending is not None else s
        self.pending = s
        inp.cmd = nc.CMD_NONE
        while self.cmds and self.cmds[0][0] <= self.t:
            _, c, x, y, z = self.cmds.pop(0)
            inp.cmd, inp.cmd_x, inp.cmd_y, inp.cmd_z = c, x, y, z
        out = self.navi.step(inp)
        self.rc = np.array([out.rc_roll, out.rc_pitch, out.rc_yaw, out.rc_throttle])
        self.armed = bool(out.arm)

        L = self.log
        L["t"].append(self.t)
        L["pos"].append(q.pos.copy())
        L["vel"].append(q.vel.copy())
        L["rpy"].append(euler_from_rot(q.R))
        L["est"].append((out.est_x, out.est_y, out.est_z))
        L["tgt_est"].append((out.tgt_x, out.tgt_y))
        L["person"].append(self.person.p.copy())
        L["sp"].append((out.sp_x, out.sp_y, out.sp_z))
        L["state"].append(out.state)
        L["fault"].append(out.fault)
        L["led"].append((out.led_r, out.led_g, out.led_b))
        L["buzzer"].append(out.buzzer_hz)
        L["rc"].append(self.rc.copy())
        L["vbat"].append(q.vbat)
        L["amps"].append(q.amps)
        L["sigma"].append(out.tgt_sigma)
        L["hover_thr"].append(out.hover_thr)
        L["wind"].append(self.wind.copy())
        L["yaw_ref"].append(self.yaw_ref)
        L["pad_est"].append((out.pad_x, out.pad_y))
        L["pad_sigma"].append(out.pad_sigma)
        L["home_phase"].append(out.home_phase)

        n = int(round(self.CTRL_DT / self.PHYS_DT))
        for _ in range(n):
            dt = self.PHYS_DT
            self.wind[:2] += (-self.wind[:2] / 2.0) * dt + \
                sc.wind_sigma * math.sqrt(2 * dt / 2.0) * self.rng.normal(size=2)
            q.step(dt, self.rc, self.armed, self.wind,
                   self.floor(q.pos[0], q.pos[1]))
            # the FC integrates its (biased) gyro: it never sees the bias
            self.yaw_ref += (q.w[2] - q.yaw_rate_bias) * dt
            self.person.step(self.t, dt)
            self.t += dt
        self.tick += 1

    def run(self):
        while self.t < self.sc.duration:
            self.step()
        self.log = {k: np.array(v) for k, v in self.log.items()}
        return self.log


def metrics(sim: Sim):
    """Numbers that say whether the flight was acceptable."""
    L = sim.log
    t, pos, per, st = L["t"], L["pos"], L["person"], L["state"]
    flying = pos[:, 2] > 0.5
    cruise = np.isin(st, [nc.FOLLOW, nc.ATTENTION])
    d_h = np.linalg.norm(pos[:, :2] - per, axis=1)

    # The odom frame slowly rotates against the world (yaw drift), so compare
    # the *relative* vector drone->person, rotated back by the drift angle.
    true_yaw = L["rpy"][:, 2]
    drift = true_yaw - L["yaw_ref"]
    rel_est = L["tgt_est"] - L["est"][:, :2]
    c, s = np.cos(drift), np.sin(drift)
    rel_est_w = np.stack([c * rel_est[:, 0] - s * rel_est[:, 1],
                          s * rel_est[:, 0] + c * rel_est[:, 1]], axis=1)
    rel_err = np.linalg.norm(rel_est_w - (per - pos[:, :2]), axis=1)

    tilt = np.degrees(np.arccos(np.clip(
        np.cos(L["rpy"][:, 0]) * np.cos(L["rpy"][:, 1]), -1, 1)))
    m = {
        "final_state": nc.STATE_NAMES[int(st[-1])],
        "airborne_s": float(flying.sum() * sim.CTRL_DT),
        "z_rms_cm": float(100 * np.sqrt(np.mean(
            (pos[cruise, 2] - L["sp"][cruise, 2]) ** 2))) if cruise.any() else float("nan"),
        "min_dist_m": float(d_h[flying].min()) if flying.any() else float("nan"),
        # closest horizontal approach while low enough to touch someone
        "min_dist_low_m": float(d_h[flying & (pos[:, 2] < HEAD_CLEAR_Z)].min())
        if (flying & (pos[:, 2] < HEAD_CLEAR_Z)).any() else float("nan"),
        "mean_dist_m": float(d_h[cruise].mean()) if cruise.any() else float("nan"),
        "max_dist_follow_m": float(d_h[cruise].max()) if cruise.any() else float("nan"),
        "track_rms_m": float(np.sqrt(np.mean(rel_err[cruise] ** 2))) if cruise.any() else float("nan"),
        "track_max_m": float(rel_err[cruise].max()) if cruise.any() else float("nan"),
        "max_tilt_deg": float(tilt[flying].max()) if flying.any() else 0.0,
        "max_alt_m": float(pos[:, 2].max()),
        "motor_sat_s": float(sim.quad.sat_time),
        "touchdown_mps": float(sim.quad.touchdown_speed),
        "end_vbat": float(L["vbat"][-1]),
        "hover_thr": float(np.median(L["hover_thr"][cruise])) if cruise.any() else float("nan"),
        "hover_amps": float(np.median(L["amps"][cruise])) if cruise.any() else float("nan"),
        "faults": int(np.bitwise_or.reduce(L["fault"].astype(int))),
    }
    if sim.sc.pad:
        pad = np.array(sim.sc.pad)
        home = st == nc.HOME
        m["pad_miss_m"] = float(np.linalg.norm(pos[-1, :2] - pad))
        m["home_secs"] = float(home.sum() * sim.CTRL_DT)
        m["reached_settle"] = bool((L["home_phase"] == 3).any())
    return m
