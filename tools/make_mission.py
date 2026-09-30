"""Write a Mission Planner mission file: take off, then laps of a 600 x 800 m rectangle at 100 m.

Usage:  python make_mission.py LAT LON HOME_ALT_M [output.waypoints]
Load it in Mission Planner: FLIGHT PLAN > Load WP File, then Write WPs.
"""
import math
import sys
from pathlib import Path

CORNERS_NE = [(300, -300), (300, 500), (-300, 500), (-300, -300)]  # metres north/east of home
TAKEOFF_ALT, CRUISE_ALT, LAPS = 80, 100, 50


def main(lat, lon, home_alt, out):
    def offset(north, east):
        return (lat + math.degrees(north / 6371000),
                lon + math.degrees(east / (6371000 * math.cos(math.radians(lat)))))

    # index, current, frame, command, p1, p2, p3, p4, lat, lon, alt, autocontinue
    items = [(0, 16, 0, 0, lat, lon, home_alt),        # home (absolute altitude)
             (3, 22, 15, 0, lat, lon, TAKEOFF_ALT)]    # NAV_TAKEOFF, 15 deg pitch, relative altitude
    items += [(3, 16, 0, 0, *offset(n, e), CRUISE_ALT) for n, e in CORNERS_NE]  # NAV_WAYPOINT
    items.append((3, 177, 2, LAPS, 0, 0, 0))           # DO_JUMP back to item 2, LAPS times
    lines = ["QGC WPL 110"]
    for i, (frame, cmd, p1, p2, la, lo, alt) in enumerate(items):
        lines.append("\t".join(str(v) for v in (i, int(i == 0), frame, cmd, p1, p2, 0, 0,
                                                 f"{la:.8f}", f"{lo:.8f}", f"{alt:.2f}", 1)))
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    Path(out).write_text("\n".join(lines) + "\n")
    print(out)


if __name__ == "__main__":
    root = Path(__file__).resolve().parent.parent
    default_out = root / "missions" / "rectangle_loop.waypoints"
    main(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4] if len(sys.argv) > 4 else default_out)
