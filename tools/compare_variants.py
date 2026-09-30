"""Scores OpenVINS variants that ran side by side (setup/run_vio_variants.sh) against the simulator's truth.

Every VIO start of a variant is one segment (from LOG_DIR/odom_NAME.csv, the live estimates logged by
tools/ros_truth_log.py; without those, from OpenVINS's own files est_NAME_N.txt). For each: how far its path drifts from
the true path, both counted from the segment's start (while the GPS is on, ArduPilot keeps lining the VIO up
with its own position, so the drift since the start is what the plane would fly with after a GPS loss) - after
30 s, 1, 2, 3, 4 and 5 minutes - and its ground speed error. All variants get the same start states at the same
moments, so they face the same flight. Truth: LOG_DIR/truth.csv (tools/ros_truth_log.py, 100 per second).

With --split-turning each segment is labelled by how much the true heading varies over it: "straight"
(under 6 deg), "zig-zag" (8-40 deg), other segments (circling, the climb) are left out - for
gz_gps_loss.py --mission weave --restart-at-legs, where every leg is flown by a fresh VIO.

Usage:  python compare_variants.py LOG_DIR [--horizons 30,60,120,180,240,300] [--split-turning]
                                   [--out results/variants.png] [--title T]
        (LOG_DIR as seen from Windows, e.g. \\\\wsl.localhost\\Ubuntu-22.04\\home\\kolabuzlu\\sitl_gazebo\\variants)
"""
import argparse
import csv
import re
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
LABELS = {   # variant: label, in the order they are listed and coloured (the two references always first)
    "hd15": "1280x800, 15/s",
    "sd15": "848x480, 15/s",
    "hd30": "1280x800, 30/s",
    "sd30": "848x480, 30/s",
    "hd10": "1280x800, 10/s",
    "hd15c30": "1280x800, 15/s, 30 clones",
    "hd15_blur": "1280x800, 15/s, blurred",
    "sd15_blur": "848x480, 15/s, blurred",
    "hd15_noise": "1280x800, 15/s, noisy",
    "sd15_noise": "848x480, 15/s, noisy",
}
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]   # fixed order
HORIZONS = [30, 60, 120, 180, 240, 300]      # seconds after each start at which the drift is read (--horizons)


def load_truth(path):
    with open(path) as f:
        rows = [r for r in csv.DictReader(f)]
    t = np.array([float(r["t"]) for r in rows])
    p = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    keep = np.r_[True, np.diff(t) > 0]
    t, p = t[keep], p[keep]
    v = np.gradient(p, t, axis=0)
    return t, p, v


def load_est(path):
    """OpenVINS's saved state: time, q (4), p (3), v (3), ... - position and velocity in its global frame."""
    a = np.loadtxt(path, comments="#", ndmin=2)
    if len(a) == 0:
        return None
    return a[:, 0], a[:, 5:7], a[:, 8:10]


def load_odom(path):
    """The live estimates of one variant, split into its VIO starts (column run)."""
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return [(a[a[:, 1] == r, 0], a[a[:, 1] == r][:, 2:4], a[a[:, 1] == r][:, 5:7]) for r in np.unique(a[:, 1])]


def segment_scores(truth, est):
    tt, tp, tv = truth
    t, p, v = est
    ok = (t >= tt[0]) & (t <= tt[-1])
    t, p, v = t[ok], p[ok], v[ok]
    if len(t) < 50:
        return None
    ptrue = np.stack([np.interp(t, tt, tp[:, i]) for i in range(2)], 1)
    vtrue = np.stack([np.interp(t, tt, tv[:, i]) for i in range(2)], 1)
    since = t - t[0]
    drift = np.linalg.norm((p - p[0]) - (ptrue - ptrue[0]), axis=1)
    speed_err = np.linalg.norm(v - vtrue, axis=1)
    at = {h: float(np.interp(h, since, drift)) for h in HORIZONS if since[-1] >= h}
    heading = np.unwrap(np.arctan2(vtrue[:, 0], vtrue[:, 1]))
    return dict(since=since, drift=drift, at=at, speed=float(np.median(speed_err)), rate=len(t) / max(since[-1], 1),
                start_offset=float(np.linalg.norm(p[0] - ptrue[0])), length=float(since[-1]),
                turning=float(np.degrees(np.std(heading))),
                km=float(np.sum(np.linalg.norm(np.diff(ptrue, axis=0), axis=1)) / 1000))


def kind_of(seg):
    return "straight" if seg["turning"] < 6 else "zig-zag" if 8 <= seg["turning"] <= 40 else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log_dir")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "results" / "variants.png"))
    ap.add_argument("--title", default="OpenVINS settings side by side on one simulated flight")
    ap.add_argument("--horizons", default="30,60,120,180,240,300", help="seconds after each start to read the drift at")
    ap.add_argument("--split-turning", action="store_true", help="score straight and zig-zag segments apart")
    ap.add_argument("--skip-first", action="store_true", help="leave out each variant's first segment (take-off, transit)")
    a = ap.parse_args()
    HORIZONS[:] = [int(h) for h in a.horizons.split(",")]
    log = Path(a.log_dir)
    truth = load_truth(log / "truth.csv")

    results = {}
    odom = sorted(log.glob("odom_*.csv"))
    if odom:
        for f in odom:
            for est in load_odom(f):
                s = segment_scores(truth, est)
                if s:
                    results.setdefault(f.stem[5:], []).append(s)
    else:
        for f in sorted(log.glob("est_*_*.txt"),
                        key=lambda f: (f.name.rsplit("_", 1)[0], int(re.findall(r"_(\d+)\.txt$", f.name)[0]))):
            name = f.name[4:].rsplit("_", 1)[0]
            est = load_est(f)
            s = segment_scores(truth, est) if est is not None else None
            if s:
                results.setdefault(name, []).append(s)

    if a.skip_first:
        results = {n: segs[1:] for n, segs in results.items() if len(segs) > 1}
    if a.split_turning:
        split = {}
        for n, segs in results.items():
            for seg in segs:
                k = kind_of(seg)
                if k:
                    split.setdefault(f"{n} {k}", []).append(seg)
        results = split
        for n, segs in results.items():
            LABELS.setdefault(n, LABELS.get(n.split()[0], n.split()[0]) + ", " + n.split()[1])
        for n in sorted(results):
            segs = results[n]
            print(f"{n}: {len(segs)} legs, turning {np.median([g['turning'] for g in segs]):.0f} deg, "
                  f"{np.median([g['km'] for g in segs]):.2f} km each, drift per km flown "
                  f"{np.median([g['drift'][-1] / max(g['km'], 0.1) for g in segs]):.1f} m")
    names = [n for n in LABELS if n in results] + [n for n in results if n not in LABELS]
    colours = {n: SERIES[i % len(SERIES)] for i, n in enumerate(names)}
    print(f"{'variant':28} segs  rate/s  speed err  " + "  ".join(f"drift@{h // 60}:{h % 60:02d}" for h in HORIZONS))
    for n in names:
        segs = results[n]
        label = LABELS.get(n, n)
        cells = []
        for h in HORIZONS:
            vals = [s["at"][h] for s in segs if h in s["at"]]
            cells.append(f"{np.median(vals):7.1f} m ({len(vals)})" if vals else f"{'-':>7}     ")
        print(f"{label:28} {len(segs):4d}  {np.median([s['rate'] for s in segs]):6.1f}  "
              f"{np.median([s['speed'] for s in segs]):6.2f} m/s  " + "  ".join(cells))
    print("start offsets (VIO start vs truth, m):",
          {n: round(float(np.median([s["start_offset"] for s in results[n]])), 1) for n in names})

    fig, ax = plt.subplots(figsize=(10, 5.6), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=INK2, labelsize=9)
    grid_t = np.arange(0, max(HORIZONS) + 1, 5.0)
    for n in names:
        colour, label = colours[n], LABELS.get(n, n)
        curves = []
        for s in results[n]:
            ax.plot(s["since"], s["drift"], color=colour, linewidth=0.6, alpha=0.35)
            c = np.interp(grid_t, s["since"], s["drift"], right=np.nan)
            curves.append(c)
        med = np.nanmedian(np.array(curves), axis=0)
        n_seg = np.sum(~np.isnan(np.array(curves)), axis=0)
        med[n_seg < max(2, (len(curves) + 1) // 2)] = np.nan       # only while at least half the segments run
        ax.plot(grid_t, med, color=colour, linewidth=2.2, label=f"{label} (median of {len(results[n])})")
    ax.set_xlim(0, max(HORIZONS))
    ax.set_ylim(bottom=0)
    ax.set_xlabel("seconds since the VIO (re)started", color=INK2, fontsize=9.5)
    ax.set_ylabel("drift from the true path, m", color=INK2, fontsize=9.5)
    ax.set_title(a.title, color=INK, fontsize=11.5, loc="left")
    ax.legend(frameon=False, fontsize=9, loc="upper left", labelcolor=INK2)
    fig.savefig(a.out, dpi=130, bbox_inches="tight", facecolor=SURFACE)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
