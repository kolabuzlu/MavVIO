"""Where did the plane really fly after the GPS loss? Runs of tools/gz_gps_loss.py side by side.

Reads results/<run>/log_data.csv (from analyze_log.py): the true path (SIM) from the GPS loss until the GPS
came back, on a map around home, and the true distance from home against time - with what ArduPilot
believed as a thin line. Marks the moment the wind changed (--shift-s after the GPS loss).

Usage:  python plot_rtl_compare.py RUN=LABEL [RUN=LABEL ...] [--shift-s 60] [--title T] [--out FILE]
"""
import argparse
import csv
import math
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]


def load(run):
    with open(ROOT / "results" / run / "log_data.csv") as f:
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    t, gps = col("t_s"), col("gps_on")
    off = np.where(gps < 0.5)[0]
    t0, t1 = t[off[0]], t[off[-1]]
    keep = (t >= t0) & (t <= t1)
    return (t[keep] - t0, col("truth_n")[keep], col("truth_e")[keep], col("est_n")[keep], col("est_e")[keep])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="RUN=LABEL")
    ap.add_argument("--shift-s", type=float, help="seconds after the GPS loss when the wind changed")
    ap.add_argument("--title", default="Where the plane really flew without GPS")
    ap.add_argument("--out", default=str(ROOT / "results" / "rtl_compare.png"))
    ap.add_argument("--ellipse", help="E,N,RADIUS_E,RADIUS_N,LABEL: draw an area on the map (e.g. the lake)")
    a = ap.parse_args()

    fig, (ax_map, ax_d) = plt.subplots(1, 2, figsize=(14, 6.4), facecolor=SURFACE,
                                       gridspec_kw=dict(width_ratios=[1, 1.3]))
    fig.subplots_adjust(left=0.06, right=0.88, top=0.84, bottom=0.11, wspace=0.2)
    fig.text(0.06, 0.94, a.title, fontsize=15, color=INK, weight="bold")
    reach = 150.0
    for k, spec in enumerate(a.runs):
        run, label = spec.split("=", 1)
        t, tn, te, en, ee = load(run)
        c = SERIES[k % len(SERIES)]
        d_true, d_est = np.hypot(tn, te), np.hypot(en, ee)
        reach = max(reach, d_true.max())
        ax_map.plot(te, tn, color=c, lw=1.6, label=label)
        ax_map.plot(te[-1], tn[-1], "o", ms=6, color=c)
        ax_map.plot(te[0], tn[0], "x", ms=9, mew=2, color=c)
        if k == 0:
            ax_map.annotate("GPS lost", (te[0], tn[0]), xytext=(8, 6), textcoords="offset points", fontsize=9.5,
                            color=INK2)
        ax_d.plot(t, d_true, color=c, lw=2)
        ax_d.plot(t, d_est, color=c, lw=1, ls=(0, (4, 3)), alpha=0.8)
        ax_d.annotate(f"{label}  {d_true[-1]:.0f} m", (t[-1], d_true[-1]), xytext=(6, 0), textcoords="offset points",
                      va="center", fontsize=10, color=INK)
        err = np.hypot(tn - en, te - ee)
        print(f"{label:28} without GPS for {t[-1]:.0f} s: really {d_true.min():.0f}-{d_true.max():.0f} m from home "
              f"(at the end {d_true[-1]:.0f} m); ArduPilot's position off by up to {err.max():.0f} m")
    if a.ellipse:
        e0, n0, re, rn, name = a.ellipse.split(",", 4)
        ang = np.linspace(0, 2 * np.pi, 200)
        ax_map.fill(float(e0) + float(re) * np.cos(ang), float(n0) + float(rn) * np.sin(ang), color="#cde2fb",
                    zorder=0)
        ax_map.annotate(name, (float(e0), float(n0)), ha="center", va="center", fontsize=9.5, color=INK2, zorder=1)
        reach = max(reach, math.hypot(float(e0) + float(re), float(n0) + float(rn)))
    ax_map.plot(0, 0, marker="^", ms=11, color=INK, mfc=SURFACE, mew=1.8)
    ax_map.annotate("home", (0, 0), xytext=(8, -12), textcoords="offset points", fontsize=10, color=INK2)
    lim = reach * 1.08
    ax_map.set_xlim(-lim, lim)
    ax_map.set_ylim(-lim, lim)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("east (m)", color=INK2)
    ax_map.set_ylabel("north (m)", color=INK2)
    ax_map.set_title("True path while the GPS was off", loc="left", fontsize=12, color=INK)
    ax_map.legend(frameon=False, fontsize=10, loc="upper left")
    ax_d.set_xlabel("time since the GPS was lost (s)", color=INK2)
    ax_d.set_ylabel("distance from home (m)", color=INK2)
    ax_d.set_title("Distance from home: really (solid) and what ArduPilot believed (dashed)", loc="left",
                   fontsize=12, color=INK)
    ax_d.set_ylim(bottom=0)
    ax_d.set_xlim(left=0)
    if a.shift_s is not None:
        ax_d.axvline(a.shift_s, color=MUTED, lw=1, ls=":")
        ax_d.annotate("wind changes", (a.shift_s, ax_d.get_ylim()[1]), xytext=(4, -14), textcoords="offset points",
                      fontsize=9.5, color=INK2)
    for ax in (ax_map, ax_d):
        ax.set_facecolor(SURFACE)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(AXIS)
        ax.tick_params(colors=INK2, labelsize=9.5)
        ax.grid(True, color=GRID, lw=0.8)
        ax.set_axisbelow(True)
    fig.savefig(a.out, dpi=130, facecolor=SURFACE)
    print(f"saved {a.out}")


if __name__ == "__main__":
    main()
