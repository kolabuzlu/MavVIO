#!/usr/bin/env python3
"""Build the Gazebo world for the camera / VIO tests (Gazebo step 2).

A 5.1 km x 5.1 km landscape around the simulator's home (ArduPilot's CMAC airfield coordinates; x = east,
y = north, metres): fields with crop rows and tramlines, ploughed and stubble fields, meadows, orchards,
forests, hedges, seven villages, roads and dirt tracks, five lakes, a mown model-airfield strip at home with
a big N and E painted next to it (one lake lies under the north-east corner of the test rectangle). The
central 2 km x 2 km is painted as 64 aerial-photo-like texture tiles at 25 cm per pixel, the ring around it
out to 2.56 km from home as 84 tiles at 50 cm per pixel (the graphics memory would not hold all of it at
25 cm), plain grass beyond; plus 3D trees and houses so the camera also sees depth.
Everything follows from one random seed, so the world is the same every time.

Runs inside Ubuntu (numpy + Pillow):  python3 /mnt/c/Users/funfo/vio/tools/make_scenery.py
Writes ~/vio_gazebo (models/vio_ground, vio_objects, zephyr_vio and worlds/vio_world.sdf) and a map
preview to vio/results/scenery_map.jpg.
"""
import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

SEED = 7
HALF = 1024.0                      # detailed area: -HALF..HALF metres east and north of home
TILES = 8                          # 8 x 8 texture tiles there
TILE_M = 2 * HALF / TILES          # 256 m per tile
TILE_PX = 1024                     # pixels per tile side: 25 cm per pixel in the detailed area
PAD = 8                            # each tile image also paints 8 px of its neighbours, against seams
TEX_PX = TILE_PX + 2 * PAD
WIDE = 2560.0                      # the ring around it out to -WIDE..WIDE: 512 m tiles, 50 cm per pixel
WIDE_TILE_M = 512.0
OUTER = 8000.0                     # plain ground out to this distance
SUN = np.array([-0.35, 0.45, -0.82])            # direction the sunlight travels (east, north, up)
SUN = SUN / np.linalg.norm(SUN)
SHADOW = SUN[:2] / -SUN[2]                       # shadow offset per metre of height
ROT = math.radians(8)                            # the field grid is turned 8 degrees
LAKES = [((500.0, 400.0), (230.0, 140.0)),       # under the NE corner (north 300, east 500) of the rectangle
         ((-1750.0, 1350.0), (380.0, 230.0)), ((1600.0, -1650.0), (300.0, 200.0)),
         ((-900.0, -2050.0), (240.0, 160.0)), ((2050.0, 1950.0), (190.0, 150.0))]
VILLAGES = [((0.0, -470.0), 150.0), ((-560.0, 170.0), 130.0),
            ((1850.0, 350.0), 180.0), ((-1950.0, -650.0), 160.0), ((450.0, 1900.0), 150.0),
            ((900.0, -1950.0), 170.0), ((-1300.0, 2150.0), 140.0)]
FORESTS = [((-1700.0, -1550.0), 550.0), ((2150.0, -500.0), 420.0), ((-450.0, 1650.0), 380.0),
           ((1500.0, 1450.0), 330.0)]
HOME_AREA = (70.0, 200.0)          # mown grass, |x| < 70, |y| < 200
STRIP = (15.0, 110.0)              # the model-airfield strip, |x| < 15, |y| < 110
HOME_CLEAR = 90.0                  # no 3D objects this close to home (take-off)
CORNERS_NE = [(300, -300), (300, 500), (-300, 500), (-300, -300)]   # test rectangle (north, east)

CROP, PLOUGHED, STUBBLE, MEADOW, ORCHARD, FOREST, VILLAGE = range(7)
PALETTE = {
    CROP: [(0.30, 0.42, 0.16), (0.36, 0.47, 0.18), (0.26, 0.38, 0.15), (0.40, 0.50, 0.22)],
    PLOUGHED: [(0.42, 0.33, 0.24), (0.36, 0.28, 0.20), (0.47, 0.38, 0.28)],
    STUBBLE: [(0.74, 0.66, 0.42), (0.68, 0.60, 0.36), (0.78, 0.70, 0.48)],
    MEADOW: [(0.36, 0.48, 0.22), (0.40, 0.52, 0.25), (0.32, 0.44, 0.20)],
    ORCHARD: [(0.38, 0.48, 0.24)],
    FOREST: [(0.13, 0.20, 0.10)],
    VILLAGE: [(0.40, 0.47, 0.26)],
}
GRASS = np.array([0.34, 0.45, 0.20], np.float32)
HEDGE = np.array([0.16, 0.26, 0.11], np.float32)
WATER = np.array([0.16, 0.24, 0.27], np.float32)
REEDS = np.array([0.30, 0.36, 0.18], np.float32)
CROWNS = [(0.16, 0.28, 0.12), (0.20, 0.33, 0.14), (0.13, 0.24, 0.11), (0.24, 0.36, 0.16)]


# ---------------------------------------------------------------- noise, the same at the same place everywhere
def _hash(i, j, seed):
    n = (i * 374761393 + j * 668265263 + seed * 144269504) & 0xFFFFFFFF
    n = ((n ^ (n >> 13)) * 1274126177) & 0xFFFFFFFF
    return ((n ^ (n >> 16)) & 0xFFFF) / 65535.0


def value_noise(x, y, scale, seed):
    """Smooth random values 0..1 on a grid of `scale` metres."""
    gx, gy = x / scale, y / scale
    ix, iy = np.floor(gx).astype(np.int64), np.floor(gy).astype(np.int64)
    fx, fy = gx - ix, gy - iy
    fx, fy = fx * fx * (3 - 2 * fx), fy * fy * (3 - 2 * fy)
    a, b = _hash(ix, iy, seed), _hash(ix + 1, iy, seed)
    c, d = _hash(ix, iy + 1, seed), _hash(ix + 1, iy + 1, seed)
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def fbm(x, y, octaves, seed):
    """Sum of noise layers [(scale m, amplitude)], centred on 0."""
    return sum(amp * (value_noise(x, y, scale, seed + k) - 0.5) * 2 for k, (scale, amp) in enumerate(octaves))


def to_uv(x, y):
    """World (east, north) -> the field grid's turned axes."""
    return x * math.cos(ROT) + y * math.sin(ROT), -x * math.sin(ROT) + y * math.cos(ROT)


def from_uv(u, v):
    return u * math.cos(ROT) - v * math.sin(ROT), u * math.sin(ROT) + v * math.cos(ROT)


# ---------------------------------------------------------------- the layout: where everything is
class Layout:
    def __init__(self, seed):
        rng = np.random.default_rng(seed)
        self.lake_phase = rng.uniform(0, 2 * math.pi, (len(LAKES), 3))
        self.forest_phase = rng.uniform(0, 2 * math.pi, (len(FORESTS), 2))
        self.streets = []                             # (village index, points, width) - houses line them
        self.roads = self._roads(rng)
        self.parcels = self._parcels(rng)
        self.crowns = self._crowns(rng)
        self.trees, self.houses = self._objects(rng)

    # lakes: ellipses with a wavy shore; returns the radius ratio to the nearest lake (< 1 inside)
    def lake_ratio(self, x, y):
        best = None
        for ((cx, cy), (rx, ry)), p in zip(LAKES, self.lake_phase):
            dx, dy = (x - cx) / rx, (y - cy) / ry
            t = np.arctan2(dy, dx)
            shore = 1 + 0.08 * np.sin(3 * t + p[0]) + 0.05 * np.sin(5 * t + p[1]) + 0.03 * np.sin(9 * t + p[2])
            r = np.hypot(dx, dy) / shore
            best = r if best is None else np.minimum(best, r)
        return best

    def in_forest(self, x, y):
        """The big forests of the outer ring: round blobs with a wavy edge."""
        for ((cx, cy), radius), p in zip(FORESTS, self.forest_phase):
            t = math.atan2(y - cy, x - cx)
            if math.hypot(x - cx, y - cy) < radius * (1 + 0.15 * math.sin(3 * t + p[0]) + 0.08 * math.sin(7 * t + p[1])):
                return True
        return False

    @staticmethod
    def main_road_y(x):
        return -120 + 35 * np.sin(x / 420) + 10 * np.sin(x / 97)

    @staticmethod
    def side_road_x(y):
        return 190 + 25 * np.sin(y / 310)

    def _roads(self, rng):
        x = np.linspace(-WIDE - 150, WIDE + 150, 1100)
        main = np.c_[x, self.main_road_y(x)]
        y = np.linspace(-WIDE - 150, WIDE + 150, 1100)
        side = np.c_[self.side_road_x(y), y]
        roads = [(main, 7.0, "asphalt"), (side, 5.0, "asphalt")]
        for vi, ((cx, cy), radius) in enumerate(VILLAGES):     # village streets
            a = rng.uniform(0, math.pi)
            for turn in (0, math.pi / 2):
                d = np.array([math.cos(a + turn), math.sin(a + turn)])
                s = np.linspace(-radius, radius, 60)[:, None]
                street = np.array([cx, cy]) + s * d
                roads.append((street, 5.0, "asphalt"))
                self.streets.append((vi, street, 5.0))
            if vi >= 2:                                         # the outer villages: a road to the nearer main road
                to_main, to_side = abs(cy - self.main_road_y(cx)), abs(cx - self.side_road_x(cy))
                end = np.array([cx, self.main_road_y(cx)]) if to_main < to_side else np.array([self.side_road_x(cy), cy])
                s = np.linspace(0, 1, 120)[:, None]
                line = np.array([cx, cy]) + s * (end - np.array([cx, cy]))
                normal = np.array([-(end - (cx, cy))[1], (end - (cx, cy))[0]]) / (np.linalg.norm(end - (cx, cy)) or 1)
                line += 25 * np.sin(np.pi * 3 * s) * normal                     # a gentle wiggle
                roads.append((line, 5.0, "asphalt"))
        for _ in range(30):                          # dirt tracks off the main road
            x0 = rng.uniform(-WIDE + 150, WIDE - 150)
            if abs(x0) < 120 or abs(x0 - 190) < 60:
                continue
            p = np.array([x0, float(self.main_road_y(x0))])
            heading = math.pi / 2 if rng.random() < 0.5 else -math.pi / 2
            pts = [p.copy()]
            for _ in range(int(rng.uniform(40, 130))):
                heading += rng.normal(0, 0.06)
                p = p + 8 * np.array([math.cos(heading), math.sin(heading)])
                if self.lake_ratio(p[0], p[1]) < 1.15 or max(abs(p)) > WIDE + 60 or \
                        (abs(p[0]) < HOME_AREA[0] + 10 and abs(p[1]) < HOME_AREA[1] + 10):
                    break
                pts.append(p.copy())
            if len(pts) > 5:
                roads.append((np.array(pts), 3.0, "dirt"))
        return roads

    def road_distance(self, x, y):
        """Distance to the nearest road edge (negative on the road); inf if no road within 100 m."""
        x, y = np.atleast_1d(x).astype(float), np.atleast_1d(y).astype(float)
        best = np.full(x.shape, np.inf)
        for pts, width, _ in self.roads:
            near = np.nonzero((x > pts[:, 0].min() - 100) & (x < pts[:, 0].max() + 100) &
                              (y > pts[:, 1].min() - 100) & (y < pts[:, 1].max() + 100))[0]
            if len(near) == 0:
                continue
            a, b = pts[:-1], pts[1:]
            ab = b - a
            L2 = np.maximum((ab ** 2).sum(1), 1e-9)
            for k in range(0, len(near), 2000):
                idx = near[k:k + 2000]
                px, py = x[idx, None], y[idx, None]
                t = np.clip(((px - a[:, 0]) * ab[:, 0] + (py - a[:, 1]) * ab[:, 1]) / L2, 0, 1)
                d = np.hypot(px - (a[:, 0] + t * ab[:, 0]), py - (a[:, 1] + t * ab[:, 1])).min(1) - width / 2
                best[idx] = np.minimum(best[idx], d)
        return best

    def _parcels(self, rng):
        parcels = []

        def split(u0, v0, u1, v1):
            w, h = u1 - u0, v1 - v0
            if max(w, h) < rng.uniform(80, 190):
                parcels.append([u0, v0, u1, v1])
                return
            if w >= h:
                cut = u0 + w * rng.uniform(0.3, 0.7)
                split(u0, v0, cut, v1)
                split(cut, v0, u1, v1)
            else:
                cut = v0 + h * rng.uniform(0.3, 0.7)
                split(u0, v0, u1, cut)
                split(u0, cut, u1, v1)

        split(-WIDE - 700, -WIDE - 700, WIDE + 700, WIDE + 700)
        out = []
        for u0, v0, u1, v1 in parcels:
            cx, cy = from_uv((u0 + u1) / 2, (v0 + v1) / 2)
            if max(abs(cx), abs(cy)) > WIDE + 250:
                continue
            if any(math.hypot(cx - vx, cy - vy) < r for (vx, vy), r in VILLAGES):
                kind = VILLAGE
            elif (-1150 < cx < -380 and -1150 < cy < -480 + 60 * math.sin(cx / 150)) or self.in_forest(cx, cy):
                kind = FOREST
            else:
                kind = rng.choice([CROP, PLOUGHED, STUBBLE, MEADOW, ORCHARD, FOREST],
                                  p=[0.30, 0.14, 0.15, 0.22, 0.07, 0.12])
            base = np.array(PALETTE[kind][rng.integers(len(PALETTE[kind]))], np.float32)
            base = base * rng.uniform(0.92, 1.08)
            spacing = {CROP: rng.uniform(1.6, 2.8), PLOUGHED: rng.uniform(0.9, 1.6),
                       STUBBLE: rng.uniform(0.7, 1.2), ORCHARD: rng.uniform(6.0, 7.0)}.get(kind, 0)
            hedge = rng.uniform(1.5, 3.0) if rng.random() < 0.35 else 0.0
            corners = [from_uv(u, v) for u in (u0, u1) for v in (v0, v1)]
            xs, ys = [p[0] for p in corners], [p[1] for p in corners]
            out.append(dict(u0=u0, v0=v0, u1=u1, v1=v1, kind=kind, base=base, spacing=spacing,
                            along_u=rng.random() < 0.5, hedge=hedge, seed=int(rng.integers(1_000_000)),
                            bbox=(min(xs), min(ys), max(xs), max(ys))))
        return out

    def _keep(self, x, y, road_margin, lake_margin=1.12):
        """Mask of points clear of the home area, the lake and the roads."""
        home = (np.abs(x) < HOME_AREA[0]) & (np.abs(y) < HOME_AREA[1])
        return ~home & (self.lake_ratio(x, y) > lake_margin) & (self.road_distance(x, y) > road_margin)

    def _crowns(self, rng):
        """Tree crowns painted into the texture: x, y, radius, height, colour index."""
        pts = []
        for p in self.parcels:
            u0, v0, u1, v1 = p["u0"], p["v0"], p["u1"], p["v1"]
            if p["kind"] == FOREST:
                gu, gv = np.meshgrid(np.arange(u0 + 2, u1 - 2, 4.5), np.arange(v0 + 2, v1 - 2, 4.5))
                n = gu.size
                u, v = gu.ravel() + rng.uniform(-1.8, 1.8, n), gv.ravel() + rng.uniform(-1.8, 1.8, n)
                pts.append(np.c_[u, v, rng.uniform(1.8, 3.8, n), rng.uniform(12, 22, n), rng.integers(0, 4, n)])
            elif p["kind"] == ORCHARD:
                s = p["spacing"]
                gu, gv = np.meshgrid(np.arange(u0 + 4, u1 - 4, s), np.arange(v0 + 4, v1 - 4, s * 0.8))
                n = gu.size
                u, v = gu.ravel() + rng.uniform(-0.4, 0.4, n), gv.ravel() + rng.uniform(-0.4, 0.4, n)
                pts.append(np.c_[u, v, rng.uniform(1.5, 2.2, n), rng.uniform(4, 5, n), np.full(n, 1)])
            elif p["kind"] == MEADOW:                  # a few bushes
                n = int(rng.integers(0, 9))
                u, v = rng.uniform(u0 + 3, u1 - 3, n), rng.uniform(v0 + 3, v1 - 3, n)
                pts.append(np.c_[u, v, rng.uniform(1.0, 2.5, n), rng.uniform(2, 4, n), rng.integers(0, 4, n)])
            if p["hedge"] >= 2.0:                     # bushes and small trees along the hedge (two edges)
                for along_u in (True, False):
                    along = np.arange(u0, u1, 4.0) if along_u else np.arange(v0, v1, 4.0)
                    n = len(along)
                    jitter = rng.uniform(-0.6, 0.6, n)
                    if along_u:
                        u, v = along + rng.uniform(-1, 1, n), v0 + p["hedge"] / 2 + jitter
                    else:
                        u, v = u0 + p["hedge"] / 2 + jitter, along + rng.uniform(-1, 1, n)
                    keep = rng.random(n) < 0.7
                    pts.append(np.c_[u, v, rng.uniform(1.2, 2.6, n), rng.uniform(3, 8, n), rng.integers(0, 4, n)][keep])
        c = np.vstack(pts)
        x, y = from_uv(c[:, 0], c[:, 1])
        c[:, 0], c[:, 1] = x, y
        c = c[(np.abs(x) < WIDE + 20) & (np.abs(y) < WIDE + 20)]
        return c[self._keep(c[:, 0], c[:, 1], road_margin=1.0)]

    def _objects(self, rng):
        """3D trees (x, y, height, crown radius, conifer?) and houses (x, y, width, depth, wall h, roof h, yaw, kind)."""
        houses = []
        for vi, ((cx, cy), radius) in enumerate(VILLAGES):
            for street_vi, pts, width in self.streets:
                if street_vi != vi:
                    continue
                d = pts[-1] - pts[0]
                d = d / np.linalg.norm(d)
                n = np.array([-d[1], d[0]])
                for s in np.arange(-radius + 15, radius - 10, rng.uniform(17, 24)):
                    for side in (-1, 1):
                        if rng.random() < 0.15:
                            continue
                        w, dp = rng.uniform(8, 14), rng.uniform(7, 11)
                        c = np.array([cx, cy]) + s * d + side * (width / 2 + rng.uniform(4, 8) + dp / 2) * n
                        houses.append([c[0], c[1], w, dp, rng.uniform(3, 6.5), rng.uniform(2, 4),
                                       math.atan2(d[1], d[0]), int(rng.integers(3))])
        for _ in range(40):                           # farm barns in the fields
            p = self.parcels[rng.integers(len(self.parcels))]
            if p["kind"] not in (CROP, STUBBLE, PLOUGHED, MEADOW):
                continue
            x, y = from_uv(p["u0"] + 20, p["v0"] + 15)
            houses.append([x, y, rng.uniform(18, 30), rng.uniform(10, 14), rng.uniform(5, 8), rng.uniform(1.5, 2.5),
                           ROT, 3])
        houses = np.array(houses)
        ok = self._keep(houses[:, 0], houses[:, 1], road_margin=0) & (np.hypot(houses[:, 0], houses[:, 1]) > HOME_CLEAR)
        ok &= self.road_distance(houses[:, 0], houses[:, 1]) > np.maximum(houses[:, 2], houses[:, 3]) / 2 + 1
        houses = houses[ok]

        trees = []
        for x in np.arange(-WIDE + 60, WIDE - 60, 26.0):   # avenue north of the main road
            if 700 < abs(x) < 1300:
                continue
            y = float(self.main_road_y(x)) + 9 + rng.uniform(-1, 1)
            trees.append([x + rng.uniform(-3, 3), y, rng.uniform(8, 14), rng.uniform(2.5, 4), 0])
        for y in np.arange(-WIDE + 60, WIDE - 60, 25.0):   # along the north-south road
            if -850 < y < 0 or abs(y) > 900:
                trees.append([float(self.side_road_x(y)) + 8, y + rng.uniform(-3, 3), rng.uniform(7, 12),
                              rng.uniform(2, 3.5), 0])
        for p in self.parcels:
            if p["kind"] == MEADOW and rng.random() < 0.6:        # solitary trees
                u, v = rng.uniform(p["u0"] + 10, p["u1"] - 10), rng.uniform(p["v0"] + 10, p["v1"] - 10)
                x, y = from_uv(u, v)
                trees.append([x, y, rng.uniform(9, 16), rng.uniform(3, 5), 0])
            if p["kind"] == FOREST:                               # edge trees of the forest
                for along in np.arange(p["u0"], p["u1"], rng.uniform(10, 14)):
                    x, y = from_uv(along, p["v0"] + rng.uniform(0, 4))
                    trees.append([x, y, rng.uniform(12, 20), rng.uniform(2.5, 4), float(rng.random() < 0.35)])
        for (cx, cy), radius in VILLAGES:            # garden trees
            for _ in range(14):
                a, r = rng.uniform(0, 2 * math.pi), rng.uniform(20, radius)
                trees.append([cx + r * math.cos(a), cy + r * math.sin(a), rng.uniform(6, 11), rng.uniform(2, 3.5),
                              float(rng.random() < 0.3)])
        trees = np.array(trees)
        ok = self._keep(trees[:, 0], trees[:, 1], road_margin=2.0) & (np.hypot(trees[:, 0], trees[:, 1]) > HOME_CLEAR)
        ok &= (np.abs(trees[:, 0]) < WIDE) & (np.abs(trees[:, 1]) < WIDE)
        for h in houses:                              # not inside a house
            ok &= np.hypot(trees[:, 0] - h[0], trees[:, 1] - h[1]) > max(h[2], h[3]) / 2 + 3
        return trees[ok], houses


# ---------------------------------------------------------------- painting a texture tile
def paint_tile(lay, x_west, y_north, size_m):
    """Texture of one tile (west edge, north edge, side in metres) plus a PAD-pixel border of its
    surroundings (the mesh only uses the inside)."""
    PX = size_m / TILE_PX                                            # metres per pixel of this tile
    x0, y1 = x_west - PAD * PX, y_north + PAD * PX                   # image corner (north-west)
    xs = x0 + (np.arange(TEX_PX) + 0.5) * PX
    ys = y1 - (np.arange(TEX_PX) + 0.5) * PX
    X, Y = np.meshgrid(xs.astype(np.float32), ys.astype(np.float32))
    U, V = to_uv(X, Y)
    img = np.empty((TEX_PX, TEX_PX, 3), np.float32)
    img[:] = GRASS
    tx0, tx1, ty0, ty1 = x0, x0 + TEX_PX * PX, y1 - TEX_PX * PX, y1

    for p in lay.parcels:
        bx0, by0, bx1, by1 = p["bbox"]
        if bx1 < tx0 or bx0 > tx1 or by1 < ty0 or by0 > ty1:
            continue
        m = (U >= p["u0"]) & (U < p["u1"]) & (V >= p["v0"]) & (V < p["v1"])
        if not m.any():
            continue
        idx = np.nonzero(m)
        u, v, x, y = U[idx], V[idx], X[idx], Y[idx]
        a = u if p["along_u"] else v                          # across the rows
        k, s, base = p["kind"], p["seed"], p["base"]
        blotch = fbm(x, y, [(18, 0.08), (55, 0.07)], s)
        if k == CROP:
            f = 1 + 0.10 * np.cos(2 * np.pi * a / p["spacing"]) + blotch
            f = np.where((a % (p["spacing"] * 9)) < 0.5, f * 0.78, f)             # tramlines
        elif k == PLOUGHED:
            f = 1 + 0.16 * np.cos(2 * np.pi * a / p["spacing"]) + 1.5 * blotch + fbm(x, y, [(0.6, 0.06)], s + 5)
        elif k == STUBBLE:
            t = a % 20.0
            f = 1 + 0.06 * np.cos(2 * np.pi * a / p["spacing"]) + blotch
            f = np.where((np.abs(t - 3) < 0.3) | (np.abs(t - 4.8) < 0.3), f * 0.75, f)
        elif k == MEADOW:
            f = 1 + fbm(x, y, [(6, 0.12), (20, 0.10)], s)
        elif k == ORCHARD:
            f = 1 + 0.06 * np.cos(2 * np.pi * a / p["spacing"]) + blotch
        elif k == FOREST:
            f = 1 + fbm(x, y, [(3, 0.15)], s)
        else:                                                  # VILLAGE: gardens, gravel, paving
            q = np.floor(value_noise(x, y, 9.0, s) * 4).astype(int)
            garden = np.array([[0.36, 0.48, 0.22], [0.34, 0.33, 0.22], [0.55, 0.53, 0.48], [0.45, 0.44, 0.42]],
                              np.float32)
            col = garden[np.clip(q, 0, 3)]
            img[idx] = col * (1 + fbm(x, y, [(1.5, 0.08)], s))[:, None]
            f = None
        if f is not None:
            img[idx] = base * f[:, None]
        edge = np.minimum.reduce([u - p["u0"], p["u1"] - u, v - p["v0"], p["v1"] - v])
        if p["hedge"] > 0:
            h = edge < p["hedge"]
            img[idx[0][h], idx[1][h]] = HEDGE * (1 + fbm(x[h], y[h], [(1.2, 0.2)], s + 9))[:, None]
        g = (edge < 0.6) & (edge >= p["hedge"])
        img[idx[0][g], idx[1][g]] = GRASS * (1 + fbm(x[g], y[g], [(1.0, 0.1)], s + 11))[:, None]

    # home: mown grass and the strip
    home = (np.abs(X) < HOME_AREA[0]) & (np.abs(Y) < HOME_AREA[1])
    if home.any():
        img[home] = (GRASS * 1.08) * (1 + fbm(X[home], Y[home], [(5, 0.06)], 3))[:, None]
        strip = (np.abs(X) < STRIP[0]) & (np.abs(Y) < STRIP[1])
        band = np.where((np.floor((X[strip] + STRIP[0]) / 5) % 2) == 0, 1.06, 0.94)
        img[strip] = np.array([0.44, 0.56, 0.27], np.float32) * band[:, None]

    # lake and its reedy shore
    ratio = lay.lake_ratio(X, Y)
    shore = (ratio < 1.07) & (ratio >= 1.0)
    if shore.any():
        img[shore] = REEDS * (1 + fbm(X[shore], Y[shore], [(1.5, 0.15), (6, 0.1)], 21))[:, None]
    water = ratio < 1.0
    if water.any():
        img[water] = WATER * (1 + fbm(X[water], Y[water], [(80, 0.03), (8, 0.008)], 22))[:, None]

    img *= (1 + fbm(X, Y, [(0.5, 0.03), (1.4, 0.03)], 99))[:, :, None]   # fine grain everywhere
    im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))

    def to_px(x, y):
        return (x - x0) / PX, (y1 - y) / PX

    # tree crowns: shadow first, then the crown with a sunlit side
    c = lay.crowns
    sel = (c[:, 0] > tx0 - 30) & (c[:, 0] < tx1 + 30) & (c[:, 1] > ty0 - 30) & (c[:, 1] < ty1 + 30)
    shade = Image.new("RGBA", im.size, (0, 0, 0, 0))
    ds = ImageDraw.Draw(shade)
    for x, y, r, h, _ in c[sel]:
        sx, sy = x + SHADOW[0] * h * 0.6, y + SHADOW[1] * h * 0.6
        px, py, pr = *to_px(sx, sy), r / PX
        ds.ellipse([px - pr, py - pr * 0.9, px + pr, py + pr * 0.9], fill=(10, 20, 5, 120))
    im = Image.alpha_composite(im.convert("RGBA"), shade).convert("RGB")
    d = ImageDraw.Draw(im)
    for x, y, r, h, ci in c[sel]:
        col = np.array(CROWNS[int(ci)])
        px, py, pr = *to_px(x, y), r / PX
        d.ellipse([px - pr, py - pr, px + pr, py + pr], fill=tuple((col * 255).astype(int)))
        lx, ly = px - SHADOW[0] / np.hypot(*SHADOW) * pr * 0.3, py + SHADOW[1] / np.hypot(*SHADOW) * pr * 0.3
        d.ellipse([lx - pr * 0.55, ly - pr * 0.55, lx + pr * 0.55, ly + pr * 0.55],
                  fill=tuple((np.minimum(col * 1.35, 1) * 255).astype(int)))

    # roads on top
    for pts, width, kind in lay.roads:
        if pts[:, 0].max() < tx0 - 20 or pts[:, 0].min() > tx1 + 20 or pts[:, 1].max() < ty0 - 20 or \
                pts[:, 1].min() > ty1 + 20:
            continue
        line = [to_px(x, y) for x, y in pts]
        if kind == "asphalt":
            d.line(line, fill=(78, 80, 76), width=int((width + 1.2) / PX), joint="curve")   # verge
            d.line(line, fill=(92, 92, 90), width=int(width / PX), joint="curve")
        else:
            d.line(line, fill=(148, 128, 98), width=int(width / PX), joint="curve")
            d.line(line, fill=(120, 104, 80), width=int(0.5 / PX), joint="curve")        # grass ridge
    main = lay.roads[0][0]
    seg = np.hypot(*np.diff(main, axis=0).T)
    s = np.r_[0, np.cumsum(seg)]
    for start in np.arange(0, s[-1], 9.0):                       # dashed centre line
        a, b = np.interp([start, start + 3], s, main[:, 0]), np.interp([start, start + 3], s, main[:, 1])
        if tx0 - 5 < a[0] < tx1 + 5 and ty0 - 5 < b[0] < ty1 + 5:
            d.line([to_px(a[0], b[0]), to_px(a[1], b[1])], fill=(215, 215, 205), width=max(1, int(0.3 / PX)))

    # the N and E markers next to the strip (also show which way the images are turned)
    white = (232, 232, 225)
    for p0, p1 in [((-13, 150), (-13, 190)), ((-13, 190), (13, 150)), ((13, 150), (13, 190)),
                   ((100, -20), (100, 20)), ((100, 20), (126, 20)), ((100, 0), (120, 0)), ((100, -20), (126, -20))]:
        d.line([to_px(*p0), to_px(*p1)], fill=white, width=int(4 / PX))
    return im


# ---------------------------------------------------------------- meshes and models
class Obj:
    """A small OBJ writer: flat-shaded faces grouped by material."""

    def __init__(self):
        self.v, self.vt, self.vn, self.faces = [], [], [], {}

    def face(self, mat, pts, uvs=None):
        pts = [np.asarray(p, float) for p in pts]
        n = np.cross(pts[1] - pts[0], pts[2] - pts[0])
        n = n / (np.linalg.norm(n) or 1)
        self.vn.append(n)
        ni = len(self.vn)
        refs = []
        for k, p in enumerate(pts):
            self.v.append(p)
            vi = len(self.v)
            if uvs is not None:
                self.vt.append(uvs[k])
                refs.append(f"{vi}/{len(self.vt)}/{ni}")
            else:
                refs.append(f"{vi}//{ni}")
        self.faces.setdefault(mat, []).append(refs)

    def write(self, path, mtl_name):
        with open(path, "w") as f:
            f.write(f"mtllib {mtl_name}\n")
            f.writelines(f"v {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n" for p in self.v)
            f.writelines(f"vt {t[0]:.5f} {t[1]:.5f}\n" for t in self.vt)
            f.writelines(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n" for n in self.vn)
            for mat, faces in self.faces.items():
                f.write(f"usemtl {mat}\n")
                f.writelines("f " + " ".join(r) + "\n" for r in faces)


def add_tree(obj, x, y, h, r, conifer, k):
    seg = 7
    trunk_h, trunk_r = h * (0.25 if conifer else 0.35), 0.18 + h * 0.02
    ang = np.linspace(0, 2 * np.pi, seg + 1)[:-1] + k
    ring = lambda z, rad: [(x + rad * math.cos(a), y + rad * math.sin(a), z) for a in ang]
    lo, hi = ring(0, trunk_r), ring(trunk_h + 0.3, trunk_r * 0.7)
    for i in range(seg):
        j = (i + 1) % seg
        obj.face("trunk", [lo[i], lo[j], hi[j], hi[i]])
    if conifer:
        profile = [(0.0, 1.0), (0.35, 0.75), (0.7, 0.4), (1.0, 0.0)]
    else:
        profile = [(0.0, 0.45), (0.25, 0.95), (0.55, 1.0), (0.8, 0.75), (1.0, 0.0)]
    mat = f"leaf{int(k * 10) % 3 + 1}" if not conifer else "conifer"
    z0, crown_h = trunk_h, h - trunk_h
    rings = [ring(z0 + f * crown_h, max(rr * r, 0.01)) for f, rr in profile]
    bottom = ring(z0, 0.01)
    rings = [bottom] + rings
    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(seg):
            j = (i + 1) % seg
            obj.face(mat, [a[i], a[j], b[j], b[i]])


def add_house(obj, x, y, w, dp, wall, roof, yaw, kind):
    c, s = math.cos(yaw), math.sin(yaw)
    loc = lambda u, v, z: (x + u * c - v * s, y + u * s + v * c, z)
    hw, hd = w / 2, dp / 2
    base = [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]
    wall_mat, roof_mat = (f"wall{kind + 1}", f"roof{kind + 1}") if kind < 3 else ("barn", "barnroof")
    for i in range(4):
        (u0, v0), (u1, v1) = base[i], base[(i + 1) % 4]
        obj.face(wall_mat, [loc(u0, v0, 0), loc(u1, v1, 0), loc(u1, v1, wall), loc(u0, v0, wall)])
    ridge = wall + roof                           # gable roof, ridge along the long (u) side, small eaves
    e = 0.5
    obj.face(roof_mat, [loc(-hw - e, -hd - e, wall - 0.3), loc(hw + e, -hd - e, wall - 0.3), loc(hw + e, 0, ridge),
                        loc(-hw - e, 0, ridge)])
    obj.face(roof_mat, [loc(hw + e, hd + e, wall - 0.3), loc(-hw - e, hd + e, wall - 0.3), loc(-hw - e, 0, ridge),
                        loc(hw + e, 0, ridge)])
    obj.face(wall_mat, [loc(hw, -hd, wall), loc(hw, hd, wall), loc(hw, 0, ridge)])
    obj.face(wall_mat, [loc(-hw, hd, wall), loc(-hw, -hd, wall), loc(-hw, 0, ridge)])


MATERIALS = {"trunk": (0.35, 0.25, 0.15), "leaf1": (0.18, 0.32, 0.12), "leaf2": (0.22, 0.38, 0.14),
             "leaf3": (0.15, 0.26, 0.13), "conifer": (0.11, 0.22, 0.13), "wall1": (0.86, 0.83, 0.76),
             "wall2": (0.78, 0.70, 0.58), "wall3": (0.70, 0.45, 0.35), "roof1": (0.60, 0.22, 0.16),
             "roof2": (0.45, 0.30, 0.22), "roof3": (0.42, 0.44, 0.46), "barn": (0.55, 0.35, 0.25),
             "barnroof": (0.55, 0.58, 0.60)}


def model_files(folder, name, body):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "model.config").write_text(
        f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n'
        f'  <sdf version="1.9">model.sdf</sdf>\n  <description>made by vio/tools/make_scenery.py</description>\n</model>\n')
    (folder / "model.sdf").write_text(f'<?xml version="1.0"?>\n<sdf version="1.9">\n{body}</sdf>\n')


CAMERA_LINK = """
    <!-- VIO camera module (added by vio/tools/make_scenery.py): downward camera (optical axis down, image top = nose),
         cameras tilted towards the nose (the tilts option) and the VIO IMU, all in one frame. The cameras draw in colour:
         Gazebo's grey format (L8) comes out several times too dark (no gamma, unlike a real camera), so
         setup/record_gazebo_flight.sh turns the colour pictures into grey ones the usual way. -->
    <link name="camera_link">
      <pose relative_to="zephyr::wing">0 -0.10 -0.03 0 1.5708 -1.5708</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="vio_camera" type="camera">
        <always_on>1</always_on>
        <update_rate>VIO_RATE</update_rate>
        <visualize>false</visualize>
        <topic>vio/camera</topic>
        <camera>
          <horizontal_fov>VIO_HFOV</horizontal_fov>
          <image><width>VIO_WIDTH</width><height>VIO_HEIGHT</height><format>R8G8B8</format></image>
          <clip><near>0.2</near><far>3000</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.007</stddev></noise>
        </camera>
      </sensor>
TILTED_CAMERAS
      <!-- the VIO's own IMU, in the camera module like on a real VIO camera: 400 Hz with realistic noise
           (white noise and a random constant bias per axis). ArduPilot keeps using the plane's IMU. -->
      <sensor name="vio_imu" type="imu">
        <always_on>1</always_on>
        <update_rate>400</update_rate>
        <topic>vio/imu</topic>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><bias_mean>0</bias_mean><bias_stddev>0.001</bias_stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><bias_mean>0</bias_mean><bias_stddev>0.001</bias_stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><bias_mean>0</bias_mean><bias_stddev>0.001</bias_stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.04</stddev><bias_mean>0</bias_mean><bias_stddev>0.02</bias_stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.04</stddev><bias_mean>0</bias_mean><bias_stddev>0.02</bias_stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.04</stddev><bias_mean>0</bias_mean><bias_stddev>0.02</bias_stddev></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>zephyr::wing</parent>
      <child>camera_link</child>
    </joint>

    <!-- chase camera for watching (tools/gz_web_view.py): on a virtual boom 10 m behind and 3 m above,
         looking forward and 15 deg down; weightless, no collision, not used by the VIO. 10 pictures a
         second: under WSL every picture drawn leaks ~0.2 MB of graphics memory, which limits a session -->
    <link name="chase_link">
      <pose relative_to="zephyr::wing">0 10 3 0 0.2618 -1.5708</pose>
      <inertial>
        <mass>0.000001</mass>
        <inertia><ixx>1e-9</ixx><iyy>1e-9</iyy><izz>1e-9</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="chase_camera" type="camera">
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <visualize>false</visualize>
        <topic>vio/chase_camera</topic>
        <camera>
          <camera_info_topic>vio/chase_camera_info</camera_info_topic>
          <horizontal_fov>1.22</horizontal_fov>
          <image><width>960</width><height>540</height><format>R8G8B8</format></image>
          <clip><near>0.5</near><far>6000</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="chase_joint" type="fixed">
      <parent>zephyr::wing</parent>
      <child>chase_link</child>
    </joint>

    <!-- the true pose of the plane (the wing frame = where its IMU sits), for scoring the VIO -->
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_model_pose>true</publish_model_pose>
      <publish_link_pose>false</publish_link_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <publish_sensor_pose>false</publish_sensor_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>100</update_frequency>
    </plugin>
"""

WORLD = """<?xml version="1.0" ?>
<sdf version="1.9">
  <!-- made by vio/tools/make_scenery.py -->
  <world name="vio_world">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- wind: the Zephyr's lift and drag use the air's speed, so it drifts with the wind. None at start;
         setup/gz_set_wind.sh sets and changes it while flying (topic /world/vio_world/wind). The wind
         system itself adds no forces here (scaling 0) - it only carries the wind to the rest. -->
    <wind><linear_velocity>0 0 0</linear_velocity></wind>
    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>0</force_approximation_scaling_factor>
    </plugin>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.72 0.80 0.90 1</background>
      <sky></sky>
      <shadows>true</shadows>
    </scene>
    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>{sun}</direction>
    </light>
    <include><uri>model://vio_ground</uri></include>
    <include><uri>model://vio_objects</uri></include>
    <include>
      <uri>model://zephyr_vio</uri>
      <pose degrees="true">0 0 0.422 -90 0 180</pose>
    </include>
  </world>
</sdf>
"""


# the downward VIO camera: width, height, pictures per second, horizontal field of view (rad)
CAMERA_PROFILES = {
    # Intel RealSense D455 colour camera (global shutter) as OpenVINS's rs_d455 config uses it:
    # 848 x 480 at 30 per second, fx = 416.85 px -> 91 deg across
    "d455": (848, 480, 30, 2 * math.atan(424 / 416.85)),
    # the same camera at its full 1280 x 800 (the 848 x 480 mode cuts off top and bottom: 91 x 60 deg instead of
    # 91 x 65) - 1.5x finer angles; at 30 per second, so OpenVINS can take every picture, every 2nd or every 3rd
    "d455-800": (1280, 800, 30, 2 * math.atan(424 / 416.85)),
    "sim640": (640, 480, 20, math.pi / 2),       # the camera of steps 2-2 to 3 (640 x 480, 20 per second, 90 deg)
}


def tilted_camera(tilt_deg):
    """A VIO camera like the downward one in the same module (same IMU), turned from straight down
    towards the nose, so in level flight it looks tilt_deg below the horizon; image top still = nose side."""
    turn = math.radians(90 - tilt_deg)   # about the module's left axis; x (optical axis) goes from down to forward
    return f"""      <sensor name="vio_camera_t{tilt_deg}" type="camera">
        <pose>0 0 0 0 {-turn:.6f} 0</pose>
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <visualize>false</visualize>
        <topic>vio/camera_t{tilt_deg}</topic>
        <camera>
          <camera_info_topic>vio/camera_t{tilt_deg}_info</camera_info_topic>
          <horizontal_fov>1.5708</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.2</near><far>8000</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.007</stddev></noise>
        </camera>
      </sensor>
"""


def build_landscape(out, keep_tiles=False):
    """The ground (painted tiles and the plain frame) and the 3D trees and houses; returns the map preview.

    Each ground tile and each 512 m block of trees and houses is its own mesh and visual: Gazebo then draws
    only the pieces a camera can see. As one big mesh it drew all 149 tiles in every picture, and under WSL
    each piece drawn leaks graphics memory - the session ran out after a few minutes."""
    lay = Layout(SEED)
    print(f"layout: {len(lay.parcels)} parcels, {len(lay.roads)} roads, {len(lay.crowns)} painted tree crowns, "
          f"{len(lay.trees)} 3D trees, {len(lay.houses)} 3D houses")

    # ground: 64 detailed tiles, the 84 tiles of the ring around them, and a plain frame out to OUTER
    mesh_dir = out / "models" / "vio_ground" / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for old in mesh_dir.glob("*.obj"):
        old.unlink()
    if not keep_tiles:
        for old in mesh_dir.glob("*.jpg"):
            old.unlink()
    scale = 0.25                                                   # preview map: pixels per metre
    mtl, visuals, preview = [], [], Image.new("RGB", (int(2 * WIDE * scale), int(2 * WIDE * scale)))
    tiles = [(f"tile_{r}_{c}", -HALF + c * TILE_M, HALF - r * TILE_M, TILE_M) for r in range(TILES) for c in range(TILES)]
    n_wide = int(2 * WIDE / WIDE_TILE_M)
    for r in range(n_wide):
        for c in range(n_wide):
            xw, yn = -WIDE + c * WIDE_TILE_M, WIDE - r * WIDE_TILE_M
            if -HALF <= xw < HALF and -HALF < yn <= HALF:              # the detailed area covers it
                continue
            tiles.append((f"wide_{r}_{c}", xw, yn, WIDE_TILE_M))
    for k, (name, xw, yn, size) in enumerate(tiles):
        jpg = mesh_dir / f"{name}.jpg"
        if keep_tiles and jpg.exists():
            im = Image.open(jpg)
        else:
            im = paint_tile(lay, xw, yn, size)
            im.save(jpg, quality=92)
        inner = im.crop((PAD, PAD, PAD + TILE_PX, PAD + TILE_PX))
        side = int(size * scale)
        preview.paste(inner.resize((side, side), Image.LANCZOS), (int((xw + WIDE) * scale), int((WIDE - yn) * scale)))
        y0 = yn - size
        a, b = PAD / TEX_PX, (PAD + TILE_PX) / TEX_PX                  # the inside of the padded image
        tile = Obj()
        tile.face(name, [(xw, y0, 0), (xw + size, y0, 0), (xw + size, yn, 0), (xw, yn, 0)],
                  uvs=[(a, a), (b, a), (b, b), (a, b)])
        tile.write(mesh_dir / f"{name}.obj", "ground.mtl")
        mtl.append(f"newmtl {name}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nNs 1\nmap_Kd {name}.jpg\n")
        visuals.append(name)
        print(f"  {name} done ({k + 1}/{len(tiles)})", flush=True)
    rng = np.random.default_rng(SEED + 1)
    xs = np.linspace(-OUTER, OUTER, 2048)
    X, Y = np.meshgrid(xs[:512], xs[:512])
    outer_tex = GRASS * (1 + fbm(X, Y, [(3, 0.12), (12, 0.12), (40, 0.1)], 55))[:, :, None]
    Image.fromarray((np.clip(outer_tex, 0, 1) * 255).astype(np.uint8)).save(mesh_dir / "outer.jpg", quality=90)
    rep = 60.0                                                     # the plain texture repeats every 60 m
    frame = Obj()
    for (ax0, ay0, ax1, ay1) in [(-OUTER, WIDE, OUTER, OUTER), (-OUTER, -OUTER, OUTER, -WIDE),
                                 (-OUTER, -WIDE, -WIDE, WIDE), (WIDE, -WIDE, OUTER, WIDE)]:
        frame.face("outer", [(ax0, ay0, 0), (ax1, ay0, 0), (ax1, ay1, 0), (ax0, ay1, 0)],
                   uvs=[(ax0 / rep, ay0 / rep), (ax1 / rep, ay0 / rep), (ax1 / rep, ay1 / rep), (ax0 / rep, ay1 / rep)])
    frame.write(mesh_dir / "outer.obj", "ground.mtl")
    visuals.append("outer")
    mtl.append("newmtl outer\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nNs 1\nmap_Kd outer.jpg\n")
    (mesh_dir / "ground.mtl").write_text("\n".join(mtl))
    ground_visuals = "".join(
        f'      <visual name="{v}">\n        <cast_shadows>false</cast_shadows>\n'
        f'        <geometry><mesh><uri>model://vio_ground/meshes/{v}.obj</uri></mesh></geometry>\n      </visual>\n'
        for v in visuals)
    model_files(out / "models" / "vio_ground", "vio_ground", f"""  <model name="vio_ground">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <pose>0 0 -0.5 0 0 0</pose>
        <geometry><box><size>18000 18000 1</size></box></geometry>
      </collision>
{ground_visuals}    </link>
  </model>
""")

    # 3D trees and houses, in 512 m blocks
    blocks = {}
    cell = lambda x, y: (int((x + WIDE) // WIDE_TILE_M), int((y + WIDE) // WIDE_TILE_M))
    for k, (x, y, h, r, conifer) in enumerate(lay.trees):
        add_tree(blocks.setdefault(cell(x, y), Obj()), x, y, h, r, bool(conifer), (k * 0.618) % 1)
    for x, y, w, dp, wall, roof, yaw, kind in lay.houses:
        add_house(blocks.setdefault(cell(x, y), Obj()), x, y, w, dp, wall, roof, yaw, int(kind))
    obj_dir = out / "models" / "vio_objects" / "meshes"
    obj_dir.mkdir(parents=True, exist_ok=True)
    for old in obj_dir.glob("*.obj"):
        old.unlink()
    for (i, j), obj in blocks.items():
        obj.write(obj_dir / f"objects_{i}_{j}.obj", "objects.mtl")
    (obj_dir / "objects.mtl").write_text("\n".join(
        f"newmtl {m}\nKa {c[0]} {c[1]} {c[2]}\nKd {c[0]} {c[1]} {c[2]}\nKs 0 0 0\nNs 1\n" for m, c in MATERIALS.items()))
    object_visuals = "".join(
        f'      <visual name="objects_{i}_{j}">\n'
        f'        <geometry><mesh><uri>model://vio_objects/meshes/objects_{i}_{j}.obj</uri></mesh></geometry>\n'
        f'      </visual>\n' for (i, j) in sorted(blocks))
    model_files(out / "models" / "vio_objects", "vio_objects", f"""  <model name="vio_objects">
    <static>true</static>
    <link name="link">
{object_visuals}    </link>
  </model>
""")
    print(f"  {len(visuals)} ground pieces, {len(blocks)} blocks of trees and houses")
    return preview


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(Path.home() / "vio_gazebo"))
    ap.add_argument("--preview", default=str(Path(__file__).resolve().parent.parent / "results" / "scenery_map.jpg"))
    ap.add_argument("--zephyr", default=str(Path.home() / "ardupilot_gazebo" / "models" / "zephyr_with_ardupilot"))
    ap.add_argument("--camera", default="d455-800", choices=sorted(CAMERA_PROFILES),
                    help="the downward VIO camera: d455-800 (1280 x 800, 30/s, 91 deg - the D455's whole sensor, best in "
                         "the side-by-side test), d455 (848 x 480, 30/s) or sim640 (640 x 480, 20/s, 90 deg)")
    ap.add_argument("--tilts", default="",
                    help="extra VIO cameras looking this many degrees below the horizon, e.g. 30,45 (step 2-3)")
    ap.add_argument("--no-chase", action="store_true",
                    help="leave out the chase camera (fewer pictures to draw; the step 2-3 recording was made so)")
    ap.add_argument("--model-only", action="store_true",
                    help="only rewrite the plane model and the world file, keep the painted landscape")
    ap.add_argument("--keep-tiles", action="store_true",
                    help="reuse the painted tile images already in the output folder (much faster)")
    args = ap.parse_args()
    out = Path(args.out)
    if not args.model_only:
        preview = build_landscape(out, args.keep_tiles)
    # the Zephyr with a downward camera
    sdf = (Path(args.zephyr) / "model.sdf").read_text()
    sdf = sdf.replace('<model name="zephyr_with_ardupilot">', '<model name="zephyr_vio">', 1)
    cut = sdf.rindex("</model>")
    tilts = [int(t) for t in args.tilts.split(",") if t]
    width, height, rate, hfov = CAMERA_PROFILES[args.camera]
    camera_link = CAMERA_LINK.replace("TILTED_CAMERAS\n", "".join(tilted_camera(t) for t in tilts))
    camera_link = (camera_link.replace("VIO_RATE", str(rate)).replace("VIO_HFOV", f"{hfov:.6f}")
                   .replace("VIO_WIDTH", str(width)).replace("VIO_HEIGHT", str(height)))
    if args.no_chase:
        start = camera_link.index("    <!-- chase camera for watching")
        end = camera_link.index("</joint>", camera_link.index('<joint name="chase_joint"')) + len("</joint>\n")
        camera_link = camera_link[:start] + camera_link[end:]
    sdf = sdf[:cut] + camera_link + "  " + sdf[cut:]
    zdir = out / "models" / "zephyr_vio"
    zdir.mkdir(parents=True, exist_ok=True)
    (zdir / "model.sdf").write_text(sdf)
    (zdir / "model.config").write_text(
        '<?xml version="1.0"?>\n<model>\n  <name>zephyr_vio</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.9">model.sdf</sdf>\n  <description>ArduPilot Zephyr with VIO cameras (down, tilted) and a VIO IMU</description>\n</model>\n')

    (out / "worlds").mkdir(parents=True, exist_ok=True)
    (out / "worlds" / "vio_world.sdf").write_text(WORLD.format(sun=" ".join(f"{v:.3f}" for v in SUN)))

    if not args.model_only:
        # preview map with the test rectangle, the detailed area and home
        d = ImageDraw.Draw(preview)
        s = preview.size[0] / (2 * WIDE)
        to_px = lambda n, e: ((e + WIDE) * s, (WIDE - n) * s)
        d.rectangle([to_px(HALF, -HALF), to_px(-HALF, HALF)], outline=(255, 255, 255), width=1)
        rect = [to_px(n, e) for n, e in CORNERS_NE + CORNERS_NE[:1]]
        d.line(rect, fill=(255, 60, 200), width=3)
        hx, hy = to_px(0, 0)
        d.polygon([(hx, hy - 9), (hx - 7, hy + 6), (hx + 7, hy + 6)], fill=(255, 255, 255), outline=(0, 0, 0))
        Path(args.preview).parent.mkdir(parents=True, exist_ok=True)
        preview.save(args.preview, quality=90)
    print(f"world: {out / 'worlds' / 'vio_world.sdf'}" + ("" if args.model_only else f"\npreview: {args.preview}"))


if __name__ == "__main__":
    main()
