"""How good was the VIO in runs of tools/gz_gps_loss.py? Two panels, several runs side by side.

Top: the VIO's speed error against the flight controller (GPS) while the GPS was on, from the companion's log
(results/<run>/vio_bridge.csv; 5 s rolling median), with the companion's VIO restarts marked. Bottom:
ArduPilot's position error after the GPS loss, from the flight log (results/<run>/log_data.csv, made by
analyze_log.py).

Usage:  python plot_vio_runs.py RUN=LABEL [RUN=LABEL ...] [--title T] [--out FILE]
"""
import argparse
import csv
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]


def columns(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return lambda k: np.array([float(r[k]) for r in rows])


def speed_error(run):
    """Minutes since the VIO first started, 5 s rolling median of |VIO - flight controller| speed while on GPS,
    and the minutes at which the VIO (re)started."""
    col = columns(ROOT / "results" / run / "vio_bridge.csv")
    t, fix, runs = col("t"), col("gps_fix"), col("run")
    err = np.hypot(col("vio_vn") - col("fc_vn"), col("vio_ve") - col("fc_ve"))
    lost = np.where(fix < 3)[0]
    end = lost[0] if len(lost) else len(t)
    t, err, runs = t[:end], err[:end], runs[:end]
    smooth = np.array([np.nanmedian(err[max(0, i - 50):i + 50]) for i in range(len(err))])   # ~5 s at 20/s
    starts = t[np.r_[0, np.where(np.diff(runs) > 0)[0] + 1]]
    return (t - t[0]) / 60, smooth, (starts - t[0]) / 60


def position_error(run):
    col = columns(ROOT / "results" / run / "log_data.csv")
    t, gps, err = col("t_s"), col("gps_on"), col("pos_err_m")
    off = np.where(gps < 0.5)[0]
    keep = slice(off[0], off[-1] + 1)
    return t[keep] - t[off[0]], err[keep]


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=INK2, labelsize=9)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="RUN=LABEL")
    ap.add_argument("--title", default="The VIO in the long GPS-loss flight")
    ap.add_argument("--out", default=str(ROOT / "results" / "vio_runs.png"))
    a = ap.parse_args()
    runs = [r.split("=", 1) for r in a.runs]

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 7.2), facecolor=SURFACE,
                                      gridspec_kw=dict(height_ratios=[1, 1], hspace=0.42))
    for ax in (top, bottom):
        style(ax)
    for i, (run, label) in enumerate(runs):
        c = SERIES[i]
        m, e, starts = speed_error(run)
        top.plot(m, e, color=c, linewidth=1.4, label=label)
        top.plot(starts[1:], np.full(len(starts) - 1, 1.12 + 0.07 * i), linestyle="none", marker="v",
                 markersize=7, color=c, markeredgecolor=SURFACE, markeredgewidth=1.5)
        s, pe = position_error(run)
        bottom.plot(s, pe, color=c, linewidth=1.6, label=f"{label}: mean {pe.mean():.0f} m, max {pe.max():.0f} m")
    top.axhline(0.7, color=MUTED, linewidth=1, linestyle=(0, (4, 3)))
    top.text(0.2, 0.72, "restart line 0.7 m/s (for 5 s)", color=INK2, fontsize=8.5, va="bottom")
    top.set_ylim(0, 1.3)
    top.set_xlabel("minutes since the VIO first started (GPS on)", color=INK2, fontsize=9.5)
    top.set_ylabel("VIO speed error, m/s", color=INK2, fontsize=9.5)
    top.set_title("VIO speed error against GPS while the GPS was on  (triangles: the companion restarted the VIO)",
                  color=INK, fontsize=10.5, loc="left")
    fig.legend(*top.get_legend_handles_labels(), frameon=False, fontsize=9.5, loc="upper right",
               ncol=len(runs), labelcolor=INK2, bbox_to_anchor=(0.99, 1.0))
    bottom.set_xlabel("seconds since the GPS was lost", color=INK2, fontsize=9.5)
    bottom.set_ylabel("position error, m", color=INK2, fontsize=9.5)
    bottom.set_ylim(bottom=0)
    bottom.set_title("ArduPilot's position error while flying home on the VIO (true position from the simulator)",
                     color=INK, fontsize=10.5, loc="left")
    bottom.legend(frameon=False, fontsize=9, loc="upper left", labelcolor=INK2)
    fig.suptitle(a.title, color=INK, fontsize=12.5, x=0.06, ha="left", y=0.985)
    fig.savefig(a.out, dpi=130, bbox_inches="tight", facecolor=SURFACE)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
