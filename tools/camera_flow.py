"""What the VIO camera sees move, depending on where it points.

Level flight over flat ground: for every pixel of the VIO camera (640x480, 90 deg wide, like the Gazebo one)
works out how far away the ground is and how fast it slides across the picture (pixels per second, from
the plane's travel only - turning is left out because the gyro measures it). The camera measures speed
through that sliding, so where the ground barely moves (far away, or straight ahead) the picture says
little about speed. Draws a camera looking straight down, one looking straight ahead and one tilted
part way down.

Usage:  python camera_flow.py [--height 100] [--speed 10] [--tilt 40]   (defaults = the Gazebo flight)
"""
import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

ROOT = Path(__file__).resolve().parent.parent
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SKY = "#ecebe6"
BLUES = LinearSegmentedColormap.from_list("blues", ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"])
W, H, F = 640, 480, 320.0          # pixels, focal length in pixels (90 deg across)


def camera_axes(tilt_deg):
    """Camera right / down / viewing direction in the world (x forward, y left, z up)."""
    a = np.radians(tilt_deg)
    look = np.array([np.cos(a), 0.0, -np.sin(a)])
    right = np.array([0.0, -1.0, 0.0])
    return np.column_stack([right, np.cross(look, right), look])


def ground_flow(tilt_deg, height, speed):
    """Per pixel: distance to the ground (nan = sky) and how fast the ground point moves in the picture."""
    R = camera_axes(tilt_deg)
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    rays = np.stack([(u - W / 2) / F, (v - H / 2) / F, np.ones_like(u)], axis=-1) @ R.T
    down = -rays[..., 2]
    ground = down > 1e-6
    t = np.where(ground, height / np.where(ground, down, 1.0), np.nan)
    q = (t[..., None] * rays) @ R                     # ground point in camera coordinates
    q_dot = -(R.T @ np.array([speed, 0.0, 0.0]))      # it moves back past the camera at the flight speed
    du = F * (q_dot[0] * q[..., 2] - q[..., 0] * q_dot[2]) / q[..., 2] ** 2
    dv = F * (q_dot[1] * q[..., 2] - q[..., 1] * q_dot[2]) / q[..., 2] ** 2
    dist = t * np.linalg.norm(rays, axis=-1)
    return dist, du, dv


def time_in_view(tilt_deg, height, speed, ahead=1000.0):
    """Seconds a spot on the ground straight ahead (starting at most `ahead` m away) spends in the picture."""
    R = camera_axes(tilt_deg)
    seen = 0
    for x in np.arange(ahead, -ahead, -speed * 0.1):  # the spot relative to the plane, every 0.1 s
        q = R.T @ np.array([x, 0.0, -height])
        seen += q[2] > 0 and 0 <= F * q[0] / q[2] + W / 2 < W and 0 <= F * q[1] / q[2] + H / 2 < H
    return seen * 0.1


def describe(name, tilt, height, speed, dist, flow):
    ground = ~np.isnan(dist)
    sky = 1 - ground.mean()
    mean_all = np.nanmean(np.where(ground, flow, 0.0))
    near, far = np.nanmin(dist), np.nanmax(dist)
    stay = time_in_view(tilt, height, speed)
    print(f"{name:16}  sky {sky * 100:4.0f} %  ground {near:5.0f} m to {'horizon' if far > 2e4 else f'{far:.0f} m'}  "
          f"motion: average over the picture {mean_all:4.1f} px/s, max {np.nanmax(flow):4.1f}  "
          f"share moving > 10 px/s {np.nanmean(np.where(ground, flow > 10, False)) * 100:3.0f} %  "
          f"a spot ahead (from 1 km) stays in view {stay:4.0f} s")
    return sky, near, far, mean_all, stay


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--height", type=float, default=100.0, help="height above the ground, m")
    ap.add_argument("--speed", type=float, default=10.0, help="ground speed, m/s")
    ap.add_argument("--tilt", type=float, default=40.0, help="tilt of the third camera below the horizon, deg")
    ap.add_argument("--out", default=str(ROOT / "results" / "camera_flow.png"))
    a = ap.parse_args()

    cams = [("Straight down (now)", 90.0), ("Straight ahead", 0.0), (f"Tilted {a.tilt:.0f}° down", a.tilt)]
    data = []
    for name, tilt in cams:
        dist, du, dv = ground_flow(tilt, a.height, a.speed)
        flow = np.hypot(du, dv)
        data.append((name, tilt, dist, du, dv, flow, describe(name, tilt, a.height, a.speed, dist, flow)))
    vmax = np.ceil(max(np.nanmax(d[5]) for d in data) / 5) * 5

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 6.4), facecolor=SURFACE)
    fig.subplots_adjust(left=0.02, right=0.96, top=0.77, bottom=0.27, wspace=0.16)
    fig.text(0.02, 0.95, "What the VIO camera sees move, depending on where it points", fontsize=15, color=INK,
             weight="bold")
    fig.text(0.02, 0.905, f"Level flight {a.height:.0f} m above flat ground at {a.speed:.0f} m/s; 640 x 480 camera, "
             f"90° wide (as in the simulator).", fontsize=10.5, color=INK2)
    fig.text(0.02, 0.87, "Colour: how fast each bit of ground slides across the picture. Arrows: where it slides in "
             "2 seconds. Turning is left out (the gyro measures it).", fontsize=10.5, color=INK2)
    for ax, (name, tilt, dist, du, dv, flow, (sky, near, far, mean_all, stay)) in zip(axes, data):
        ax.set_facecolor(SKY)
        img = ax.imshow(np.ma.masked_invalid(flow), cmap=BLUES, vmin=0, vmax=vmax, extent=(0, W, H, 0),
                        interpolation="bilinear")
        gu, gv = np.meshgrid(np.linspace(50, W - 50, 7).astype(int), np.linspace(48, H - 48, 5).astype(int))
        ok = dist[gv, gu] < 2e4                          # skip the sky and the horizon itself
        ax.quiver(gu[ok], gv[ok], 2 * du[gv, gu][ok], 2 * dv[gv, gu][ok], angles="xy", scale_units="xy", scale=1,
                  pivot="mid", color=INK, width=0.005, headwidth=3.5, headlength=4, headaxislength=3.5,
                  minshaft=2, minlength=0.8)
        if sky > 0:
            sky_rows = np.where(np.isnan(dist[:, W // 2]))[0]
            ax.text(W / 2, sky_rows.mean(), "sky - nothing to track", ha="center", va="center", color=INK2,
                    fontsize=11)
            ax.axhline(sky_rows.max() + 1, color=MUTED, lw=1)
        v_ahead = H / 2 - F * np.tan(np.radians(tilt))    # the point the plane flies towards
        if tilt < 60 and v_ahead > 0:
            ax.plot(W / 2, v_ahead, "o", ms=9, mfc="none", mec="#eb6834", mew=2)
            ax.annotate("flying towards here:\nno motion at all", (W / 2, v_ahead), xytext=(W / 2 + 60, v_ahead + 60),
                        color=INK, fontsize=9.5, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
        # ground distance along the middle column, as ticks on the right
        col = dist[:, W // 2]
        ticks = [(np.where(np.abs(col - d) < d * 0.02)[0].mean(), f"{d} m" if d < 1000 else f"{d / 1000:g} km")
                 for d in (150, 300, 1000) if np.nanmin(col) * 1.05 < d < np.nanmax(col)]
        ax.set_xlim(0, W)
        ax.set_ylim(H, 0)
        ax.set_xticks([])
        ax.yaxis.tick_right()
        ax.set_yticks([r for r, _ in ticks], [t for _, t in ticks])
        ax.tick_params(axis="y", colors=INK2, labelsize=8.5, length=4)
        for s in ax.spines.values():
            s.set_color(AXIS)
        ax.set_title(name, loc="left", fontsize=12.5, color=INK, pad=6)
        far_txt = "the horizon" if far > 2e4 else f"{far / 1000:.1f} km" if far >= 1000 else f"{far:.0f} m"
        ax.text(0, -0.06, f"ground from {near:.0f} m to {far_txt}" + (f", {sky * 100:.0f} % sky" if sky > 0.005 else ""),
                transform=ax.transAxes, fontsize=10, color=INK, va="top")
        ax.text(0, -0.13, f"average motion over the picture: {mean_all:.0f} px/s", transform=ax.transAxes,
                fontsize=10, color=INK, va="top")
        ax.text(0, -0.2, f"a spot straight ahead stays in view {stay:.0f} s" + (" (from 1 km out)" if tilt < 60 else ""),
                transform=ax.transAxes, fontsize=10, color=INK, va="top")
    cax = fig.add_axes([0.35, 0.065, 0.28, 0.028])
    cb = fig.colorbar(img, cax=cax, orientation="horizontal")
    cb.outline.set_visible(False)
    cb.ax.tick_params(colors=INK2, labelsize=9, length=0)
    cb.set_label("ground motion in the picture, pixels per second", color=INK2, fontsize=9.5)
    fig.savefig(a.out, dpi=130, facecolor=SURFACE)
    print(f"saved {a.out}")


if __name__ == "__main__":
    main()
