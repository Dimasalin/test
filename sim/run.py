#!/usr/bin/env python3
"""Run simulated flights of the fairy drone.

    python3 sim/run.py                 # story flight: metrics + plots
    python3 sim/run.py standing        # any scenario from scenarios.py
    python3 sim/run.py --all           # every scenario, one line each
    python3 sim/run.py --mc 30         # Monte Carlo over build tolerances
    python3 sim/run.py --mc-home 40    # Monte Carlo of landing on the pad
    python3 sim/run.py story --gif     # also write an animation
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import navi_core as nc           # noqa: E402
import scenarios                 # noqa: E402
from world import Sim, metrics   # noqa: E402

OUT = Path(__file__).resolve().parent / "out"
STATE_COLORS = ["#8a8f98", "#9ecbff", "#4f9dff", "#e3b62f", "#3fbf6f", "#b08cff", "#e5484d",
                "#ff8ad8"]


def fly(sc):
    sim = Sim(sc)
    sim.run()
    return sim, metrics(sim)


def led_colors(L, floor=0.25):
    c = L["led"].astype(float) / 255.0
    return np.clip(c * (1 - floor) + floor * (c > 0).any(axis=1, keepdims=True), 0, 1)


def plot(sim, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    L, sc = sim.log, sim.sc
    t = L["t"]
    fig = plt.figure(figsize=(13, 8.5), facecolor="#0e1420")
    gs = fig.add_gridspec(3, 2, width_ratios=[1.15, 1], hspace=0.32, wspace=0.16)
    axes = []

    def style(ax, title):
        ax.set_facecolor("#131b2b")
        ax.set_title(title, color="#e8eefc", fontsize=10, loc="left")
        ax.tick_params(colors="#9fb0cc", labelsize=8)
        for s in ax.spines.values():
            s.set_color("#2a3650")
        ax.grid(color="#22304a", lw=0.5)
        axes.append(ax)

    ax = fig.add_subplot(gs[:, 0])
    style(ax, f"Top view, {sc.name} flight (glow colour = what the LED is doing)")
    if sc.table:
        x0, x1, y0, y1, _ = sc.table
        ax.fill([x0, x1, x1, x0], [y0, y0, y1, y1], color="#5b4630", alpha=0.7, label="table")
    ax.plot(L["person"][:, 0], L["person"][:, 1], color="#ffffff", lw=2.5, alpha=0.8,
            label="person")
    air = L["pos"][:, 2] > 0.05
    ax.scatter(L["pos"][air, 0], L["pos"][air, 1], c=led_colors(L)[air], s=5, lw=0)
    ax.plot([], [], "o", color="#9ecbff", ms=4, label="fairy")
    ax.plot(L["tgt_est"][air, 0][::10], L["tgt_est"][air, 1][::10], ".", color="#ff8fa3",
            ms=2, alpha=0.6, label="where it thinks you are")
    for c in sc.commands:
        if c[1] == nc.CMD_TARGET:
            ax.plot(c[2], c[3], "*", color="#ffd84a", ms=14, label="point of interest")
    ax.plot(0, 0, "x", color="#9fb0cc", ms=8, label="take-off")
    if sc.pad:
        ax.add_patch(plt.Circle(sc.pad, 0.15, fill=False, color="#ff8ad8", lw=2,
                                label="charging pad (30 cm)"))
        ax.plot(L["pos"][-1, 0], L["pos"][-1, 1], "v", color="#ff8ad8", ms=7,
                label="touchdown")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]", color="#9fb0cc", fontsize=8)
    ax.set_ylabel("y [m]", color="#9fb0cc", fontsize=8)
    ax.legend(fontsize=7, facecolor="#131b2b", edgecolor="#2a3650", labelcolor="#e8eefc",
              loc="lower right")

    def bands(ax):
        st = L["state"]
        edges = np.flatnonzero(np.diff(st)) + 1
        for a, b in zip(np.r_[0, edges], np.r_[edges, len(st)]):
            ax.axvspan(t[a], t[b - 1], color=STATE_COLORS[st[a]], alpha=0.13, lw=0)

    ax = fig.add_subplot(gs[0, 1])
    style(ax, "Altitude [m] (bands: flight state)")
    bands(ax)
    ax.plot(t, L["sp"][:, 2], color="#9fb0cc", lw=0.8, ls="--", label="setpoint")
    ax.plot(t, L["pos"][:, 2], color="#9ecbff", lw=1.2, label="true")
    ax.axhline(1.85, color="#e5484d", lw=0.6, ls=":")
    ax.set_ylim(0, 2.4)
    ax.legend(fontsize=7, facecolor="#131b2b", edgecolor="#2a3650", labelcolor="#e8eefc")

    ax = fig.add_subplot(gs[1, 1])
    style(ax, "Distance to person [m] and how sure the tracker is")
    bands(ax)
    d = np.linalg.norm(L["pos"][:, :2] - L["person"], axis=1)
    ax.plot(t, d, color="#9ecbff", lw=1.2, label="horizontal distance")
    ax.plot(t, L["sigma"], color="#ff8fa3", lw=0.9, label="tracker uncertainty (1σ)")
    ax.axhline(sim.navi.cfg.keepout_radius, color="#e5484d", lw=0.6, ls=":")
    ax.set_ylim(0, max(4.0, float(d.max()) * 1.05))
    ax.legend(fontsize=7, facecolor="#131b2b", edgecolor="#2a3650", labelcolor="#e8eefc")

    ax = fig.add_subplot(gs[2, 1])
    style(ax, "Battery [V] and throttle")
    bands(ax)
    ax.plot(t, L["vbat"], color="#ffd84a", lw=1.0, label="battery under load [V]")
    ax.plot(t, L["rc"][:, 3] * 4.0, color="#3fbf6f", lw=0.6, alpha=0.8,
            label="throttle ×4")
    ax.set_ylim(0, 4.4)
    ax.set_xlabel("time [s]", color="#9fb0cc", fontsize=8)
    ax.legend(fontsize=7, facecolor="#131b2b", edgecolor="#2a3650", labelcolor="#e8eefc",
              loc="lower left")

    handles = [plt.Rectangle((0, 0), 1, 1, color=STATE_COLORS[i], alpha=0.5)
               for i in range(8)]
    fig.legend(handles, nc.STATE_NAMES, ncol=8, fontsize=7, loc="lower center",
               facecolor="#0e1420", edgecolor="#0e1420", labelcolor="#e8eefc")
    fig.savefig(path, dpi=130, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def gif(sim, path, fps=8, speed=2.0):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    L = sim.log
    step = int(round(speed / fps / sim.CTRL_DT))
    idx = np.arange(0, len(L["t"]), step)
    cols = led_colors(L, floor=0.15)
    allxy = np.vstack([L["pos"][:, :2], L["person"]])
    lo, hi = allxy.min(0) - 0.8, allxy.max(0) + 0.8

    fig, ax = plt.subplots(figsize=(5.6, 5.0), facecolor="#0e1420")
    ax.set_facecolor("#131b2b")
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect("equal")
    ax.tick_params(colors="#9fb0cc", labelsize=7)
    for s in ax.spines.values():
        s.set_color("#2a3650")
    for c in sim.sc.commands:
        if c[1] == nc.CMD_TARGET:
            ax.plot(c[2], c[3], "*", color="#ffd84a", ms=12)
    person, = ax.plot([], [], "o", color="#ffffff", ms=16, alpha=0.9)
    trail = ax.scatter([], [], s=6, lw=0)
    halo = ax.scatter([], [], s=420, lw=0, alpha=0.25)
    body = ax.scatter([], [], s=70, lw=0)
    txt = ax.text(0.02, 0.97, "", transform=ax.transAxes, color="#e8eefc", fontsize=9,
                  va="top", family="monospace")

    def draw(k):
        i = idx[k]
        j0 = max(0, i - int(4.0 / sim.CTRL_DT))
        person.set_data([L["person"][i, 0]], [L["person"][i, 1]])
        trail.set_offsets(L["pos"][j0:i:3, :2]); trail.set_color(cols[j0:i:3])
        size = 30 + 45 * L["pos"][i, 2]
        for a in (halo, body):
            a.set_offsets(L["pos"][i:i + 1, :2]); a.set_color(cols[i:i + 1])
        body.set_sizes([size]); halo.set_sizes([size * 6])
        chirp = "  ♪" if L["buzzer"][i] > 0 else ""
        txt.set_text(f"t={L['t'][i]:5.1f}s  {nc.STATE_NAMES[L['state'][i]]:9s} "
                     f"z={L['pos'][i, 2]:.2f}m{chirp}")
        return person, trail, halo, body, txt

    FuncAnimation(fig, draw, frames=len(idx), blit=False).save(
        path, writer=PillowWriter(fps=fps), dpi=80)
    plt.close(fig)


def monte_carlo(n):
    rows = []
    for seed in range(1, n + 1):
        _, m = fly(scenarios.varied(seed))
        rows.append(m)
        print(f"  run {seed:3d}: {m['final_state']:8s} track {m['track_rms_m']:.2f} m  "
              f"closest-low {m['min_dist_low_m']:.2f} m  farthest {m['max_dist_follow_m']:.2f} m  "
              f"tilt {m['max_tilt_deg']:.0f}°  hover thr {m['hover_thr']:.2f}", flush=True)
    keys = ["track_rms_m", "min_dist_low_m", "max_dist_follow_m", "z_rms_cm",
            "max_tilt_deg", "max_alt_m", "touchdown_mps", "hover_thr", "motor_sat_s"]
    print(f"\nMonte Carlo, {n} story flights with mass -7/+10 %, thrust -18/+15 %, "
          "drag, flow scale, draughts and battery resistance varied")
    print(f"  {'metric':20s} {'median':>8s} {'p5':>8s} {'p95':>8s} {'worst':>8s}")
    for k in keys:
        v = np.array([r[k] for r in rows], float)
        worst = np.nanmin(v) if k == "min_dist_low_m" else np.nanmax(v)
        print(f"  {k:20s} {np.nanmedian(v):8.2f} {np.nanpercentile(v, 5):8.2f} "
              f"{np.nanpercentile(v, 95):8.2f} {worst:8.2f}")
    landed = sum(r["final_state"] == "LANDED" for r in rows)
    print(f"  landed under control: {landed}/{n}")
    return rows


def monte_carlo_home(n):
    miss, secs, fails = [], [], 0
    for seed in range(1, n + 1):
        _, m = fly(scenarios.home_varied(seed))
        ok = m["final_state"] == "LANDED" and m["reached_settle"]
        fails += not ok
        miss.append(m["pad_miss_m"]); secs.append(m["home_secs"])
        print(f"  run {seed:3d}: {'on pad ' if ok else 'ABORTED'} miss {100 * m['pad_miss_m']:5.1f} cm  "
              f"took {m['home_secs']:4.1f} s  touchdown {m['touchdown_mps']:.2f} m/s", flush=True)
    miss, secs = 100 * np.array(miss), np.array(secs)
    print(f"\nGo-home Monte Carlo, {n} flights, pad 1.6-4 m away in a random direction, "
          "airframe and sensors varied")
    print(f"  completed the approach: {n - fails}/{n}")
    print(f"  miss distance [cm]: median {np.median(miss):.1f}  p95 {np.percentile(miss, 95):.1f}  "
          f"worst {miss.max():.1f}")
    for r in (10, 15, 20):
        print(f"  inside a {2 * r} cm pad: {(miss <= r).sum()}/{n}")
    print(f"  time from command to touchdown [s]: median {np.median(secs):.0f}  "
          f"worst {secs.max():.0f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", nargs="?", default="story", choices=sorted(scenarios.ALL))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--mc", type=int, default=0, metavar="N")
    ap.add_argument("--mc-home", type=int, default=0, metavar="N")
    ap.add_argument("--gif", action="store_true")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)

    if a.mc:
        monte_carlo(a.mc)
        return
    if a.mc_home:
        monte_carlo_home(a.mc_home)
        return
    names = sorted(scenarios.ALL) if a.all else [a.scenario]
    for name in names:
        sim, m = fly(scenarios.ALL[name](seed=a.seed))
        if a.all:
            pad = f"  pad miss {100 * m['pad_miss_m']:.0f} cm" if "pad_miss_m" in m else ""
            print(f"{name:17s} {m['final_state']:8s} airborne {m['airborne_s']:5.0f} s  "
                  f"track {m['track_rms_m']:.2f} m  closest-low {m['min_dist_low_m']:.2f} m  "
                  f"touchdown {m['touchdown_mps']:.2f} m/s  faults {m['faults']:#04x}{pad}")
        else:
            print(json.dumps(m, indent=2))
        if not a.no_plot:
            plot(sim, OUT / f"{name}.png")
            if not a.all:
                print("plot:", OUT / f"{name}.png")
        if a.gif:
            gif(sim, OUT / f"{name}.gif")
            print("animation:", OUT / f"{name}.gif")


if __name__ == "__main__":
    main()
