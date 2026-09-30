"""Score a VIO run against ground truth and plot it.

Two measures:
  * ATE (absolute trajectory error): the whole estimated path is lined up with the true path
    (shift + turn about the vertical), then the RMS distance is taken - the usual research score.
  * Drift after a start line-up: paths are lined up once, over the first --align-s seconds, then the
    growing distance is followed - this is what happens after a GPS loss, when ArduPilot lines up
    the VIO frame at the switch-over and then relies on it.

Usage:  python eval_vio.py ESTIMATE GROUNDTRUTH NAME [--time-offset S | --time-offset auto:S] [--align-s 5]
  ESTIMATE       OpenVINS ov_estimate.txt (timestamp, quaternion, position, ...)
  GROUNDTRUTH    timestamp x y z ... text file (from bag_groundtruth.py)
  NAME           results/NAME/ receives vio_summary.txt and vio_plot.png
  --time-offset  ground-truth clock minus estimate clock, in seconds; "auto:S" refines the guess S
                 within +-2 s by finding the offset where the two paths match best
"""
import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
EST_COLOR = "#2a78d6"


def load(path, pos_cols):
    """(times, Nx3 positions) from a whitespace text file, skipping comment lines."""
    rows = [l.split() for l in Path(path).read_text().splitlines() if l.strip() and not l.startswith("#")]
    data = np.array([[float(r[0])] + [float(r[c]) for c in pos_cols] for r in rows])
    return data[:, 0], data[:, 1:4]


def align_yaw(est, gt):
    """Rotation about z and translation that best map est onto gt (least squares)."""
    me, mg = est.mean(axis=0), gt.mean(axis=0)
    e, g = est - me, gt - mg
    yaw = np.arctan2(np.sum(e[:, 0] * g[:, 1] - e[:, 1] * g[:, 0]), np.sum(e[:, 0] * g[:, 0] + e[:, 1] * g[:, 1]))
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return rot, mg - rot @ me


def matched(t_est, p_est, t_gt, p_gt, offset):
    """Estimate samples inside the ground-truth time span, with ground truth interpolated to them."""
    t_gt = t_gt - offset
    keep = (t_est >= t_gt[0]) & (t_est <= t_gt[-1])
    gt_at = np.column_stack([np.interp(t_est[keep], t_gt, p_gt[:, i]) for i in range(3)])
    return t_est[keep], p_est[keep], gt_at


def ate_rms(p_est, gt_at):
    rot, shift = align_yaw(p_est, gt_at)
    return np.sqrt(np.mean(np.sum(((p_est @ rot.T + shift) - gt_at) ** 2, axis=1)))


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("estimate")
    ap.add_argument("groundtruth")
    ap.add_argument("name")
    ap.add_argument("--time-offset", default="0")
    ap.add_argument("--align-s", type=float, default=5.0)
    args = ap.parse_args()

    t_est_all, p_est_all = load(args.estimate, (5, 6, 7))      # OpenVINS: t, qx qy qz qw, px py pz, ...
    t_gt, p_gt = load(args.groundtruth, (1, 2, 3))
    if args.time_offset.startswith("auto:"):
        guess = float(args.time_offset[5:])
        candidates = np.arange(guess - 2.0, guess + 2.0 + 1e-9, 0.02)
        scores = [ate_rms(*matched(t_est_all, p_est_all, t_gt, p_gt, o)[1:]) for o in candidates]
        offset = float(candidates[int(np.argmin(scores))])
    else:
        offset = float(args.time_offset)
    t_est, p_est, gt_at = matched(t_est_all, p_est_all, t_gt, p_gt, offset)
    t_rel = t_est - t_est[0]

    rot, shift = align_yaw(p_est, gt_at)
    ate = np.linalg.norm((p_est @ rot.T + shift) - gt_at, axis=1)
    first = t_rel <= args.align_s
    rot0, shift0 = align_yaw(p_est[first], gt_at[first])
    aligned0 = p_est @ rot0.T + shift0
    drift = np.linalg.norm(aligned0 - gt_at, axis=1)
    dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(gt_at, axis=0), axis=1))])

    lines = [f"VIO run '{args.name}': {len(t_est)} estimates over {t_rel[-1]:.0f} s, true path length {dist[-1]:.1f} m"
             + (f", clock offset {offset:.2f} s" if offset else ""),
             f"ATE (whole path lined up): RMS {np.sqrt(np.mean(ate ** 2)):.3f} m, max {ate.max():.3f} m "
             f"= {100 * np.sqrt(np.mean(ate ** 2)) / dist[-1]:.2f} % of the distance",
             f"drift after lining up over the first {args.align_s:.0f} s: {drift[-1]:.2f} m at the end, max {drift.max():.2f} m "
             f"= {100 * drift[-1] / dist[-1]:.2f} % of the distance flown"]
    out = ROOT / "results" / args.name
    out.mkdir(parents=True, exist_ok=True)
    (out / "vio_summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))

    fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(13, 5.6), facecolor=SURFACE,
                                          gridspec_kw={"width_ratios": [1, 1.25]})
    fig.subplots_adjust(left=0.06, right=0.98, top=0.8, bottom=0.12, wspace=0.22)
    style(ax_map)
    ax_map.plot(gt_at[:, 0], gt_at[:, 1], color=INK2, linewidth=1.2, linestyle=(0, (3, 2)), label="true path")
    ax_map.plot(aligned0[:, 0], aligned0[:, 1], color=EST_COLOR, linewidth=2, label="OpenVINS estimate")
    ax_map.scatter([gt_at[0, 0]], [gt_at[0, 1]], s=60, color=INK, zorder=5)
    ax_map.annotate("start", (gt_at[0, 0], gt_at[0, 1]), xytext=(8, -12), textcoords="offset points", color=INK2, fontsize=9)
    ax_map.set_aspect("equal")
    ax_map.set_xlabel("x (m)", color=INK2, fontsize=9)
    ax_map.set_ylabel("y (m)", color=INK2, fontsize=9)
    ax_map.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=2, fontsize=8.5, frameon=False, labelcolor=INK2)
    ax_map.set_title(f"seen from above (lined up over the first {args.align_s:.0f} s)", color=INK2, fontsize=9.5, loc="left")

    style(ax_err)
    ax_err.plot(t_rel, drift, color=EST_COLOR, linewidth=2)
    ax_err.set_xlabel("time since OpenVINS started (s)", color=INK2, fontsize=9)
    ax_err.set_ylabel("distance between estimate and truth (m)", color=INK2, fontsize=9)
    ax_err.set_ylim(bottom=0)
    ax_err.set_xlim(0, t_rel[-1])
    ax_err.set_title("drift after lining up at the start", color=INK2, fontsize=9.5, loc="left")
    fig.text(0.06, 0.94, f"OpenVINS on '{args.name}'", color=INK, fontsize=14, fontweight="bold")
    fig.text(0.06, 0.885, lines[1] + "\n" + lines[2], color=INK2, fontsize=9.5, va="center")
    fig.savefig(out / "vio_plot.png", dpi=150, facecolor=SURFACE)
    print(out / "vio_plot.png")


if __name__ == "__main__":
    main()
