"""Compare VIO runs that were started from the same known state (tools/bag_gt_init.py).

Because each run starts from the true position, orientation and speed, nothing is lined up afterwards:
the error is exactly what the navigation would get - how far the VIO's position is from the true one,
against the distance flown since the start. Draws the tracks on a map and the error over distance.

Usage:  python compare_cameras.py STATE_CSV RUN [RUN ...] [--labels A,B,C] [--title T] [--out FILE]
  STATE_CSV  the true state file the runs started from (results/.../state_gt.csv)
  RUN        results/RUN/ov_estimate.txt
"""
import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]


def load_state(path):
    d = np.loadtxt(path, delimiter=",", comments="#")
    return d[:, 0] * 1e-9, d[:, 1:4]


def load_estimate(path):
    rows = [l.split() for l in Path(path).read_text().splitlines() if l.strip() and not l.startswith("#")]
    d = np.array([[float(v) for v in r[:11]] for r in rows])
    return d[:, 0], d[:, 5:8]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("state")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--labels")
    ap.add_argument("--title", default="VIO position error, started from the true state")
    ap.add_argument("--out")
    a = ap.parse_args()
    labels = a.labels.split(",") if a.labels else a.runs
    t_gt, p_gt = load_state(a.state)

    fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(14, 6.2), facecolor=SURFACE,
                                         gridspec_kw=dict(width_ratios=[1, 1.35]))
    fig.subplots_adjust(left=0.05, right=0.9, top=0.83, bottom=0.11, wspace=0.18)
    fig.text(0.05, 0.94, a.title, fontsize=15, color=INK, weight="bold")
    results = []
    for k, (run, label) in enumerate(zip(a.runs, labels)):
        t, p = load_estimate(ROOT / "results" / run / "ov_estimate.txt")
        keep = (t >= t_gt[0]) & (t <= t_gt[-1])
        t, p = t[keep], p[keep]
        g = np.column_stack([np.interp(t, t_gt, p_gt[:, i]) for i in range(3)])
        err = np.linalg.norm(p[:, :2] - g[:, :2], axis=1)
        dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(g[:, :2], axis=0), axis=1))])
        at = lambda km: err[np.searchsorted(dist, km * 1000)] if dist[-1] >= km * 1000 else np.nan
        results.append((label, dist[-1], err[-1], err.max(), at(1), at(2), at(3), np.abs(p[-1, 2] - g[-1, 2])))
        c = COLORS[k % len(COLORS)]
        if k == 0:
            ax_map.plot(g[:, 0], g[:, 1], color=INK, lw=2.5, alpha=0.25, label="true path", solid_capstyle="round")
            ax_map.plot(g[0, 0], g[0, 1], "o", ms=8, mfc=SURFACE, mec=INK, mew=1.5)
            ax_map.annotate("VIO starts", (g[0, 0], g[0, 1]), xytext=(8, -14), textcoords="offset points",
                            fontsize=9.5, color=INK2)
        ax_map.plot(p[:, 0], p[:, 1], color=c, lw=1.6, label=label)
        ax_err.plot(dist / 1000, err, color=c, lw=2)
        ax_err.annotate(f"{label}  {err[-1]:.0f} m", (dist[-1] / 1000, err[-1]), xytext=(6, 0),
                        textcoords="offset points", va="center", fontsize=10, color=INK)
    for ax in (ax_map, ax_err):
        ax.set_facecolor(SURFACE)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(AXIS)
        ax.tick_params(colors=INK2, labelsize=9.5)
        ax.grid(True, color=GRID, lw=0.8)
        ax.set_axisbelow(True)
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.set_xlabel("east (m)", color=INK2)
    ax_map.set_ylabel("north (m)", color=INK2)
    ax_map.set_title("Where the VIO thinks the plane is", loc="left", color=INK, fontsize=12)
    ax_map.legend(frameon=False, fontsize=9.5, loc="best")
    ax_err.set_xlabel("distance flown since the VIO started (km)", color=INK2)
    ax_err.set_ylabel("horizontal position error (m)", color=INK2)
    ax_err.set_title("Position error", loc="left", color=INK, fontsize=12)
    ax_err.set_ylim(bottom=0)
    ax_err.set_xlim(left=0)
    out = Path(a.out) if a.out else ROOT / "results" / f"{a.runs[0]}_compare.png"
    fig.savefig(out, dpi=130, facecolor=SURFACE)
    print(f"{'run':24} {'flown':>8} {'error at end':>13} {'max':>8} {'at 1 km':>8} {'at 2 km':>8} {'at 3 km':>8}  height error at end")
    for label, flown, end, mx, e1, e2, e3, dz in results:
        f = lambda v: "     -" if np.isnan(v) else f"{v:6.0f} m"
        print(f"{label:24} {flown / 1000:6.2f} km {end:9.0f} m {mx:6.0f} m {f(e1):>8} {f(e2):>8} {f(e3):>8}  {dz:6.0f} m"
              f"   ({end / flown * 100:.1f} % of the distance)")
    print(f"saved {out}")


if __name__ == "__main__":
    main()
