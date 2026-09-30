"""Save camera images from a ROS 2 (sqlite3) recording at chosen times, as one contact sheet.

Usage:  python bag_frames.py BAG_DIR TOPIC OUT.png T1 [T2 ...]
  T = seconds from the start of the recording. Reads only the chosen images (indexed by time).
"""
import sqlite3
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from rosbags.typesys import Stores, get_typestore


def main(bag_dir, topic, out, *times):
    db = next(Path(bag_dir).glob("*.db3"))
    # immutable=1: read-only without file locks (locks fail across the \\wsl.localhost file bridge).
    # A UNC path (//server/share/...) needs an empty URI authority: file:////server/share/...
    path = db.as_posix()
    uri = f"file://{path}" if path.startswith("//") else f"file:/{path}"
    con = sqlite3.connect(f"{uri}?mode=ro&immutable=1", uri=True)
    topic_id, msgtype = con.execute("SELECT id, type FROM topics WHERE name = ?", (topic,)).fetchone()
    start = con.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    store = get_typestore(Stores.ROS2_HUMBLE)
    tiles = []
    for t in times:
        row = con.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? AND timestamp >= ? "
                          "ORDER BY timestamp LIMIT 1", (topic_id, start + int(float(t) * 1e9))).fetchone()
        msg = store.deserialize_cdr(row[1], msgtype)
        channels = {"mono8": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}[msg.encoding]
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * channels]
        img = img.reshape(msg.height, msg.width, channels) if channels > 1 else img
        if msg.encoding.startswith("bgr"):
            img = img[:, :, 2::-1]
        tile = Image.fromarray(img[:, :, :3] if channels > 1 else img).convert("RGB").resize((640, 360))
        ImageDraw.Draw(tile).text((10, 8), f"t = {float(t):.0f} s", fill=(255, 255, 0))
        tiles.append(tile)
    cols = 2
    sheet = Image.new("RGB", (640 * cols, 360 * ((len(tiles) + cols - 1) // cols)), "white")
    for i, tile in enumerate(tiles):
        sheet.paste(tile, ((i % cols) * 640, (i // cols) * 360))
    sheet.save(out)
    print(f"{len(tiles)} images from {topic} ({msg.width}x{msg.height}, {msg.encoding}) -> {out}")


if __name__ == "__main__":
    main(*sys.argv[1:])
