"""Compare test runs: position error against time since the GPS was lost, one line per run.

Usage:  python compare_runs.py wind_no_vio="No VIO" wind_real_vio="Imperfect VIO" wind_perfect_vio="Perfect VIO"
        python compare_runs.py runaway_old="Old script" runaway_new="New script" --out runaway_comparison.png
               --title "VIO runs away 40 s into a 3-minute GPS outage" --mark "40=VIO runs away"
Writes results/comparison.png (or --out) and prints a small table.
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
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]   # categorical slots 1-4, fixed order
OFF_BAND = "#f0efec"
LABEL_BOX = dict(boxstyle="round,pad=0.2", facecolor=SURFACE, edgecolor="none", alpha=0.9)


def load(name):
    with open(ROOT / "results" / name / "log_data.csv", newline="") as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    t_off = next(r["t_s"] for r in rows if r["gps_on"] == 0)
    t_on = next(r["t_s"] for r in rows if r["t_s"] > t_off and r["gps_on"] == 1)
    return rows, t_off, t_on


def main(pairs, title, out_name, marks, dashed=(), slots=None):
    fig, ax = plt.subplots(figsize=(11, 5.6), facecolor=SURFACE)
    fig.subplots_adjust(left=0.08, right=0.84, top=0.82, bottom=0.12)
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8, which="major")
    ax.set_axisbelow(True)

    table = []
    labels = []   # (x, y, text) of the end-of-outage labels, placed after all lines are drawn
    outage = None
    worst = 0.0
    for i, (name, label) in enumerate(pairs):
        color = SERIES[(slots or {}).get(name, i + 1) - 1]
        rows, t_off, t_on = load(name)
        outage = t_on - t_off
        sel = [r for r in rows if t_off - 30 <= r["t_s"] <= t_on + 30]
        x = [r["t_s"] - t_off for r in sel]
        y = [max(r["pos_err_m"], 0.05) for r in sel]
        ax.plot(x, y, color=color, linewidth=2, label=label, zorder=3,
                linestyle=(0, (5, 2)) if name in dashed else "solid")
        during = [r["pos_err_m"] for r in rows if t_off <= r["t_s"] < t_on]
        end_err = next(r["pos_err_m"] for r in reversed(rows) if r["t_s"] < t_on)
        # direct label at the end of the outage, where the lines are furthest apart
        idx = min(range(len(x)), key=lambda i: abs(x[i] - outage + 1))
        ax.scatter([x[idx]], [y[idx]], s=40, color=color, edgecolor=SURFACE, linewidth=2, zorder=4)
        labels.append((x[idx], y[idx], f"{label}: {end_err:.0f} m off" if end_err >= 1 else f"{label}: {end_err:.1f} m off"))
        table.append((label, sum(during) / len(during), max(during), end_err))
        worst = max(worst, max(y))
    # keep the labels at least ~0.2 decades apart on the log axis so they do not overlap
    placed = []
    for x, y, text in sorted(labels, key=lambda l: l[1]):
        log_y = np.log10(y)
        if placed and log_y - placed[-1] < 0.2:
            log_y = placed[-1] + 0.2
        placed.append(log_y)
        ax.annotate(text, (x, 10 ** log_y), xytext=(10, 0), textcoords="offset points", va="center", color=INK,
                    fontsize=9, bbox=LABEL_BOX, zorder=5)

    ax.axvspan(0, outage, color=OFF_BAND, zorder=0)
    for t, text in marks:
        ax.axvline(t, color=INK2, linewidth=1, linestyle=(0, (3, 3)), zorder=1)
        ax.annotate(text, (t, 0.93), xycoords=ax.get_xaxis_transform(), xytext=(4, 0), textcoords="offset points",
                    color=INK2, fontsize=8.5, bbox=LABEL_BOX)
    ax.annotate("GPS off", (0, 0.99), xycoords=ax.get_xaxis_transform(), xytext=(4, 0), textcoords="offset points",
                va="top", color=INK2, fontsize=9, fontweight="bold")
    ax.annotate("GPS back", (outage, 0.99), xycoords=ax.get_xaxis_transform(), xytext=(4, 0), textcoords="offset points",
                va="top", color=INK2, fontsize=9, fontweight="bold")
    ax.set_yscale("log")
    big = worst > 2000
    ax.set_ylim(0.05, 20000 if big else 2000)
    ax.set_yticks([0.1, 1, 10, 100, 1000] + ([10000] if big else []))
    ax.set_yticklabels(["0.1 m", "1 m", "10 m", "100 m", "1 km"] + (["10 km"] if big else []))
    ax.set_xlabel("time since GPS was lost (s)", color=INK2, fontsize=9)
    ax.set_ylabel("position error: where ArduPilot thinks it is vs. where it is (log scale)", color=INK2, fontsize=9)
    ax.legend(loc="lower right", bbox_to_anchor=(1, 1.01), ncol=len(pairs) if len(pairs) <= 3 else 2, fontsize=9,
              frameon=False, labelcolor=INK2)
    fig.text(0.08, 0.93, title, color=INK, fontsize=14, fontweight="bold")
    out = ROOT / "results" / out_name
    fig.savefig(out, dpi=150, facecolor=SURFACE)

    print(f"{'run':24} {'mean error':>11} {'worst':>9} {'at GPS return':>14}")
    for label, mean, worst_err, end in table:
        print(f"{label:24} {mean:9.1f} m {worst_err:7.1f} m {end:12.1f} m")
    print(out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help='RUN_NAME="label"')
    ap.add_argument("--title", default="3 minutes without GPS, in wind that shifts after 1 minute")
    ap.add_argument("--out", default="comparison.png", help="file name in results/")
    ap.add_argument("--mark", action="append", help='"SECONDS=text": a dashed line at that time since GPS loss '
                                                    '(default: 60=wind shift)')
    ap.add_argument("--dashed", action="append", default=[], help="RUN_NAME to draw dashed")
    ap.add_argument("--color", action="append", default=[], help="RUN_NAME=SLOT: colour slot 1-4 (default: run order)")
    args = ap.parse_args()
    marks = [(float(m.split("=", 1)[0]), m.split("=", 1)[1]) for m in (args.mark or ["60=wind shift"])]
    slots = {c.split("=")[0]: int(c.split("=")[1]) for c in args.color}
    main([(a.split("=", 1)[0], a.split("=", 1)[1]) for a in args.runs], args.title, args.out, marks, args.dashed, slots)
