"""Plot a test run from results/<name>/log_data.csv (made by analyze_log.py) into results/<name>/plot.png.

Left: the true flight track, coloured by how ArduPilot navigates (GPS, VIO or dead reckoning), with its
own estimate dashed on top. Right: position error over time with the GPS-off period shaded, then - when
the log has them - the switch-over script's two VIO checks against their reject line, and the bank
angle against the bank limit (otherwise the altitude).

Usage:  python plot_run.py vio_switch
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

ROOT = Path(__file__).resolve().parent.parent
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
MODE_COLOR = {"GPS": "#2a78d6", "VIO": "#eb6834", "DR": "#eda100"}
MODE_LABEL = {"GPS": "GPS", "VIO": "VIO", "DR": "dead reckoning"}
MODE_Z = {"GPS": 2, "VIO": 3, "DR": 3}        # GPS-free parts drawn on top of later GPS laps
OFF_BAND = "#f0efec"
CORNERS_NE = [(300, -300), (300, 500), (-300, 500), (-300, -300)]
LABEL_BOX = dict(boxstyle="round,pad=0.2", facecolor=SURFACE, edgecolor="none", alpha=0.9)
CHECK_TOP = 3.0   # the check panel is cut off at 3x the reject line


def load(name):
    with open(ROOT / "results" / name / "log_data.csv", newline="") as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    for r in rows:
        r["mode"] = "VIO" if r["src_set"] == 2 else ("DR" if r["gps_on"] == 0 else "GPS")
    return rows


def load_events(name):
    """(time, text) pairs from results/<name>/log_events.txt."""
    path = ROOT / "results" / name / "log_events.txt"
    if not path.exists():
        return []
    events = []
    for line in path.read_text(encoding="utf-8").splitlines():
        t, _, text = line.strip().partition(" s  ")
        events.append((float(t), text))
    return events


def load_params(name):
    path = ROOT / "results" / name / "log_params.txt"
    if not path.exists():
        return {}
    return {k: float(v) for k, v in (line.split() for line in path.read_text().splitlines() if line.strip())}


def runs_by_mode(rows):
    """Split rows into consecutive runs with the same navigation mode (sharing the boundary point)."""
    runs, start = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]["mode"] != rows[start]["mode"]:
            runs.append((rows[start]["mode"], rows[start:min(i + 1, len(rows))]))
            start = i
    return runs


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def main(name):
    rows = load(name)
    events = load_events(name)
    params = load_params(name)
    off = [r["t_s"] for r in rows if r["gps_on"] == 0]
    runs = runs_by_mode(rows)
    has_bank = "roll_deg" in rows[0]                                   # logs analysed with the newer analyze_log.py
    has_checks = has_bank and any(r["src_set"] == 2 for r in rows)      # the VIO checks only matter if VIO was used
    panels = ["err"] + (["chk"] if has_checks else []) + (["bank"] if has_bank else ["alt"])
    first = lambda key: next((t for t, text in events if key in text), None)
    t_run, t_rej, t_rtl = first("running away"), first("VIO rejected"), first("flight mode -> RTL")
    drop_start, drop_end, wind_t = first("dropout starts"), first("dropout ends"), first("wind shift")

    n_right = len(panels)
    fig = plt.figure(figsize=(13, 7.4 if n_right == 3 else 6.6), facecolor=SURFACE)
    gs = fig.add_gridspec(n_right, 2, width_ratios=[1.05, 1], hspace=0.42, wspace=0.2, left=0.07, right=0.98,
                          top=0.84, bottom=0.13)
    ax_map = fig.add_subplot(gs[:, 0])
    right = [fig.add_subplot(gs[i, 1]) for i in range(n_right)]
    for ax in right[1:]:
        ax.sharex(right[0])

    # Map: mission rectangle, true track by navigation mode, ArduPilot's estimate
    style(ax_map)
    rect = CORNERS_NE + CORNERS_NE[:1]
    ax_map.plot([e for n, e in rect], [n for n, e in rect], color=AXIS, linewidth=1, linestyle=(0, (4, 3)), zorder=1)
    ax_map.scatter([e for n, e in CORNERS_NE], [n for n, e in CORNERS_NE], s=64, marker="s", facecolor=SURFACE,
                   edgecolor=MUTED, linewidth=1.5, zorder=5)
    for mode, seg in runs:
        ax_map.plot([r["truth_e"] for r in seg], [r["truth_n"] for r in seg], color=MODE_COLOR[mode], linewidth=2,
                    solid_capstyle="round", zorder=MODE_Z[mode])
    ax_map.plot([r["est_e"] for r in rows], [r["est_n"] for r in rows], color=INK2, linewidth=0.9,
                linestyle=(0, (2, 2)), zorder=4)
    ax_map.scatter([0], [0], s=90, marker="^", color=INK, zorder=6)
    ax_map.annotate("home", (0, 0), xytext=(8, 4), textcoords="offset points", color=INK2, fontsize=9)
    marks = [("GPS off", off[0] if off else None, (12, -18)), ("GPS back", off[-1] if off else None, (-66, -4)),
             ("VIO rejected", t_rej, (10, 8)), ("RTL", t_rtl, (10, -16))]
    for label, t, offset in marks:
        if t is None:
            continue
        r = min(rows, key=lambda row: abs(row["t_s"] - t))
        ax_map.scatter([r["truth_e"]], [r["truth_n"]], s=70, color=INK, edgecolor=SURFACE, linewidth=2, zorder=7)
        ax_map.annotate(label, (r["truth_e"], r["truth_n"]), xytext=offset, textcoords="offset points", color=INK,
                        fontsize=9, fontweight="bold", bbox=LABEL_BOX, zorder=8)
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.set_xlabel("east of home (m)", color=INK2, fontsize=9)
    ax_map.set_ylabel("north of home (m)", color=INK2, fontsize=9)
    modes = [m for m in ("GPS", "VIO", "DR") if any(mode == m for mode, _ in runs)]
    handles = [Line2D([], [], color=MODE_COLOR[m], linewidth=2, label=f"true track, {MODE_LABEL[m]}") for m in modes]
    handles.append(Line2D([], [], color=INK2, linewidth=0.9, linestyle=(0, (2, 2)), label="ArduPilot's own estimate"))
    ax_map.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2, fontsize=8.5,
                  frameon=False, labelcolor=INK2)

    # Right column
    for ax in right:
        style(ax)
        if off:
            ax.axvspan(off[0], off[-1], color=OFF_BAND, zorder=0)
        if drop_start is not None and drop_end is not None:
            ax.axvspan(drop_start, drop_end, facecolor="none", edgecolor=MUTED, hatch="////", linewidth=0, zorder=1)
        for t in (wind_t, t_run):
            if t is not None:
                ax.axvline(t, color=INK2, linewidth=1, linestyle=(0, (3, 3)), zorder=1)
        for t in (t_rej, t_rtl):
            if t is not None:
                ax.axvline(t, color=INK, linewidth=1.2, zorder=1)
        ax.set_xlim(rows[0]["t_s"], rows[-1]["t_s"])
    ax_err = right[0]
    for mode, seg in runs:
        ax_err.plot([r["t_s"] for r in seg], [r["pos_err_m"] for r in seg], color=MODE_COLOR[mode], linewidth=1.6, zorder=2)
    ax_err.set_ylabel("position error (m)", color=INK2, fontsize=9)
    ax_err.set_ylim(bottom=0)
    if off:
        ax_err.annotate("GPS off", (off[0], 0.97), xycoords=ax_err.get_xaxis_transform(), xytext=(4, 0),
                        textcoords="offset points", va="top", color=INK2, fontsize=9, fontweight="bold")
    for t, label, y in ((drop_end, "VIO dropout", 0.60), (wind_t, "wind shift", 0.78), (t_run, "VIO runs away", 0.78),
                        (t_rej, "VIO rejected", 0.55), (t_rtl, "RTL", 0.35)):
        if t is not None:
            ax_err.annotate(label, (t, y), xycoords=ax_err.get_xaxis_transform(), xytext=(4, 0),
                            textcoords="offset points", color=INK if label in ("VIO rejected", "RTL") else INK2, fontsize=8.5,
                            bbox=LABEL_BOX, zorder=5)
    patches = [Patch(color=MODE_COLOR[m], label=MODE_LABEL[m]) for m in modes]
    ax_err.legend(handles=patches, loc="lower right", bbox_to_anchor=(1, 1.02), ncol=len(patches), fontsize=8.5,
                  frameon=False, labelcolor=INK2)

    ax = dict(zip(panels, right))
    if has_checks:
        ax_chk = ax["chk"]
        innov, vel = params.get("VSW_INNOV", 1.0), params.get("VSW_VEL_ERR", 12.0)
        on_vio = [r for r in rows if r["src_set"] == 2 and r["gps_on"] == 0]

        def series(key, limit):
            """Check value relative to its reject line, only while the VIO is in use without GPS (gaps elsewhere)."""
            return [min(r[key] / limit, CHECK_TOP) if (r["src_set"] == 2 and r["gps_on"] == 0 and r[key] >= 0)
                    else float("nan") for r in rows]
        ax_chk.plot([r["t_s"] for r in rows], series("imu_check", innov), color=INK, linewidth=1.3, zorder=3,
                    label=f"IMU check (EKF velocity test ratio / {innov:g})")
        if any(r["airspeed_check"] >= 0 for r in on_vio):
            ax_chk.plot([r["t_s"] for r in rows], series("airspeed_check", vel), color=MUTED, linewidth=1.3,
                        linestyle=(0, (4, 2)), zorder=3, label=f"airspeed check (mismatch / {vel:g} m/s)")
        ax_chk.axhline(1.0, color=INK2, linewidth=1, linestyle=(0, (1, 2)), zorder=2)
        ax_chk.annotate("reject line", (1.0, 1.0), xycoords=("axes fraction", "data"), xytext=(-4, 3),
                        textcoords="offset points", ha="right", color=INK2, fontsize=8.5)
        ax_chk.set_ylim(0, CHECK_TOP * 1.05)
        ax_chk.set_ylabel("VIO checks\n(1 = reject line)", color=INK2, fontsize=9)
        ax_chk.legend(loc="lower right", bbox_to_anchor=(1, 1.0), ncol=2, fontsize=8, frameon=False, labelcolor=INK2)
    if has_bank:
        ax_bank = ax["bank"]
        for mode, seg in runs:
            ax_bank.plot([r["t_s"] for r in seg], [abs(r["roll_deg"]) for r in seg], color=MODE_COLOR[mode],
                         linewidth=1.0, zorder=2)
        ax_bank.step([r["t_s"] for r in rows], [r["roll_limit"] for r in rows], where="post", color=INK,
                     linewidth=1.2, linestyle=(0, (4, 2)), zorder=3)
        ax_bank.annotate("bank limit", (rows[-1]["t_s"], rows[-1]["roll_limit"]), xytext=(-4, 3),
                         textcoords="offset points", ha="right", va="bottom", color=INK, fontsize=8.5)
        ax_bank.set_ylim(0, max(70, max(r["roll_limit"] for r in rows) + 5))
        ax_bank.set_ylabel("bank angle (deg)", color=INK2, fontsize=9)
        ax_bank.set_xlabel("time since arming (s)", color=INK2, fontsize=9)
    else:
        ax_alt = ax["alt"]
        for mode, seg in runs:
            ax_alt.plot([r["t_s"] for r in seg], [r["alt_rel"] for r in seg], color=MODE_COLOR[mode], linewidth=1.6, zorder=2)
        ax_alt.set_ylabel("altitude above home (m)", color=INK2, fontsize=9)
        ax_alt.set_xlabel("time since arming (s)", color=INK2, fontsize=9)

    t_off = off[0] if off else float("inf")
    parts = []
    for label, test in (("before GPS loss", lambda r: 60 < r["t_s"] < t_off),
                        ("on VIO", lambda r: r["mode"] == "VIO" and r["gps_on"] == 0),
                        ("dead reckoning", lambda r: r["mode"] == "DR")):
        errs = [r["pos_err_m"] for r in rows if test(r)]
        seconds = sum(b["t_s"] - a["t_s"] for a, b in zip(rows, rows[1:]) if test(a) and test(b))
        if seconds >= 5:
            parts.append(f"{label}: average {sum(errs) / len(errs):.1f} m, worst {max(errs):.1f} m ({seconds:.0f} s)")
    lowest = min((r["alt_rel"] for r in rows if r["gps_on"] == 0), default=None)
    fig.text(0.05, 0.955, f"GPS lost in AUTO flight - run '{name}'", color=INK, fontsize=14, fontweight="bold")
    fig.text(0.05, 0.915, "Position error " + "   |   ".join(parts), color=INK2, fontsize=9.5)
    if lowest is not None:
        fig.text(0.05, 0.885, f"Lowest altitude without GPS: {lowest:.0f} m above home", color=INK2, fontsize=9.5)
    out = ROOT / "results" / name / "plot.png"
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(out)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "vio_switch")
