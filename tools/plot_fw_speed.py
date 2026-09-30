"""Chart for the InGVIO fixed-wing tests: true ground speed vs OpenVINS's speed over time, turns shaded.

Usage:  python plot_fw_speed.py RUN_NAME [GT_FILE] [CLOCK_OFFSET]
"""
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
EST_COLOR, BAND = "#2a78d6", "#f0efec"
REC_START = 1657679585.0   # first IMU/camera stamp of fw_gvi_easy


def speed_series(t, p, window=1.0):
    """Speed averaged over +-window/2 seconds, sampled every 0.5 s."""
    grid = np.arange(t[0] + window, t[-1] - window, 0.5)
    i = np.searchsorted(t, grid - window / 2)
    j = np.clip(np.searchsorted(t, grid + window / 2), 0, len(t) - 1)
    return grid, np.linalg.norm(p[j] - p[i], axis=1) / np.maximum(t[j] - t[i], 1e-3)


def main(run, gt_file="datasets/ingvio/fw_gvi_easy_gt.txt", offset=970047.2):
    est = np.array([[float(v) for v in l.split()[:8]] for l in open(ROOT / "results" / run / "ov_estimate.txt")
                    if l.strip() and not l.startswith("#")])
    gt = np.array([[float(v) for v in l.split()[:4]] for l in open(ROOT / gt_file) if not l.startswith("#")])
    t_gt = gt[:, 0] - float(offset) - REC_START
    t_est = est[:, 0] - REC_START
    g_t, g_v = speed_series(t_gt, gt[:, 1:4])
    e_t, e_v = speed_series(t_est, est[:, 5:8])

    vel = np.gradient(gt[:, 1:3], t_gt, axis=0)
    turn_rate = np.degrees(np.gradient(np.unwrap(np.arctan2(vel[:, 0], vel[:, 1])), t_gt))
    kernel = np.ones(25) / 25                                   # ~3 s moving average at ~8.5 Hz
    turn_rate = np.convolve(turn_rate, kernel, mode="same")
    speed = np.linalg.norm(vel, axis=1)
    turning = (np.abs(turn_rate) > 8) & (speed > 15)
    # merge turns separated by less than 3 s, drop blips shorter than 3 s
    idx = np.flatnonzero(turning)
    bands = []
    for i in idx:
        if bands and t_gt[i] - t_gt[bands[-1][1]] < 3:
            bands[-1][1] = i
        else:
            bands.append([i, i])
    bands = [(a, b) for a, b in bands if t_gt[b] - t_gt[a] >= 3]

    fig, ax = plt.subplots(figsize=(11, 5), facecolor=SURFACE)
    fig.subplots_adjust(left=0.08, right=0.97, top=0.8, bottom=0.12)
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for a, b in bands:
        ax.axvspan(t_gt[a], t_gt[b], color=BAND, zorder=0)
        ax.text((t_gt[a] + t_gt[b]) / 2, 0.97, "turn", transform=ax.get_xaxis_transform(), ha="center", va="top",
                color=INK2, fontsize=8.5)
    ax.plot(g_t, g_v, color=INK2, linewidth=1.6, linestyle=(0, (4, 2)), label="true ground speed (RTK GPS)")
    ax.plot(e_t, e_v, color=EST_COLOR, linewidth=2, label="OpenVINS estimate")
    ax.set_ylim(0, 60)
    ax.set_xlim(20, 175)
    for x, label in ((33, "takeoff roll"), (70, "lake in view")):
        ax.annotate(label, (x, 2), xytext=(0, 0), textcoords="offset points", ha="center", color=INK2, fontsize=8.5)
    ax.set_xlabel("time in the recording (s)", color=INK2, fontsize=9)
    ax.set_ylabel("ground speed (m/s), cut off at 60", color=INK2, fontsize=9)
    ax.legend(loc="lower right", bbox_to_anchor=(1, 1.01), ncol=2, fontsize=9, frameon=False, labelcolor=INK2)
    fig.text(0.08, 0.93, f"OpenVINS on the InGVIO fixed-wing flight - run '{run}'", color=INK, fontsize=13, fontweight="bold")
    out = ROOT / "results" / run / "speed_plot.png"
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(out)


if __name__ == "__main__":
    main(*sys.argv[1:])
