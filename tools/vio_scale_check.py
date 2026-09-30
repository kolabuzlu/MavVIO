"""Does the VIO's scale error show up in its height as well as its speed?

For a downward camera the ground is at a distance equal to the height, which the barometer knows. If
(VIO speed / true speed) follows (VIO height / true height), the height ratio could correct the speed.
Compares both ratios over time and plots them.

Usage:  python vio_scale_check.py RUN_NAME   (results/RUN_NAME/ov_estimate.txt and truth.txt)
"""
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"


def load(path):
    rows = [l.split() for l in Path(path).read_text().splitlines() if l.strip() and not l.startswith("#")]
    return np.array([[float(v) for v in r] for r in rows])


def main(run):
    d = ROOT / "results" / run
    est, gt = load(d / "ov_estimate.txt"), load(d / "truth.txt")
    t = est[:, 0]
    p_est, v_est = est[:, 5:8], est[:, 8:11]
    t_gt, p_gt = gt[:, 0], gt[:, 1:4]
    # true velocity from the true path, smoothed over ~1 s
    v_gt_all = np.gradient(p_gt, t_gt, axis=0)
    k = 50
    v_gt_all = np.column_stack([np.convolve(v_gt_all[:, i], np.ones(k) / k, mode="same") for i in range(3)])
    keep = (t > t_gt[0] + 1) & (t < t_gt[-1] - 1)
    t, p_est, v_est = t[keep], p_est[keep], v_est[keep]
    v_gt = np.column_stack([np.interp(t, t_gt, v_gt_all[:, i]) for i in range(3)])
    z_gt = np.interp(t, t_gt, p_gt[:, 2]) - p_gt[0, 2]
    z_est = p_est[:, 2] - p_est[0, 2]
    speed_gt, speed_est = np.linalg.norm(v_gt[:, :2], axis=1), np.linalg.norm(v_est[:, :2], axis=1)
    fly = (z_gt > 30) & (speed_gt > 5)
    r_speed, r_height = speed_est / np.maximum(speed_gt, 1e-3), z_est / np.maximum(z_gt, 1e-3)
    tt = t - t[0]
    corrected = speed_est / r_height
    print(f"in flight (above 30 m): VIO speed / true speed from {r_speed[fly].min():.2f} to {r_speed[fly].max():.2f}; "
          f"VIO height / true height from {r_height[fly].min():.2f} to {r_height[fly].max():.2f}")
    print(f"correlation of the two ratios: {np.corrcoef(r_speed[fly], r_height[fly])[0, 1]:.2f}")
    err_before = np.abs(speed_est[fly] - speed_gt[fly])
    err_after = np.abs(corrected[fly] - speed_gt[fly])
    print(f"speed error: {err_before.mean():.2f} m/s on average as the VIO gives it, "
          f"{err_after.mean():.2f} m/s after dividing by the height ratio")

    fig, ax = plt.subplots(figsize=(11, 4.8), facecolor=SURFACE)
    fig.subplots_adjust(left=0.08, right=0.97, top=0.8, bottom=0.13)
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8)
    ax.plot(tt[fly], r_speed[fly], ".", markersize=2, color="#2a78d6", label="VIO speed / true speed")
    ax.plot(tt[fly], r_height[fly], ".", markersize=2, color="#eb6834", label="VIO height / true height")
    ax.axhline(1.0, color=INK2, linewidth=1, linestyle=(0, (3, 3)))
    ax.set_ylim(0, 3)
    ax.set_xlabel("time since OpenVINS started (s)", color=INK2, fontsize=9)
    ax.set_ylabel("ratio (1 = correct)", color=INK2, fontsize=9)
    ax.legend(loc="lower right", bbox_to_anchor=(1, 1.01), ncol=2, fontsize=9, frameon=False, labelcolor=INK2)
    fig.text(0.08, 0.93, f"Does the VIO's height reveal its scale error? - run '{run}'", color=INK, fontsize=13,
             fontweight="bold")
    out = d / "scale_check.png"
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(out)


if __name__ == "__main__":
    main(sys.argv[1])
