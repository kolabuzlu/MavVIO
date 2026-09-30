#!/usr/bin/env python3
"""Save frames from a Gazebo camera as JPEG files (runs inside Ubuntu, with Gazebo's Python bindings).

Usage:  python3 /mnt/c/Users/funfo/vio/tools/gz_camera_grab.py [--topic /vio/camera] [--count 6] [--every 5]
Writes frame_000.jpg, frame_001.jpg ... into --out (default: vio/results/gazebo_camera).
"""
import argparse
import threading
import time
from pathlib import Path

import numpy as np
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node
from PIL import Image

RGB_INT8, L_INT8 = 3, 1     # gz.msgs PixelFormatType values used here


def to_array(msg):
    if msg.pixel_format_type == RGB_INT8:
        channels = 3
    elif msg.pixel_format_type == L_INT8:
        channels = 1
    else:
        raise ValueError(f"pixel format {msg.pixel_format_type} not handled")
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, -1)[:, :msg.width * channels]
    return rows.reshape(msg.height, msg.width, channels).squeeze()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/vio/camera")
    ap.add_argument("--count", type=int, default=6)
    ap.add_argument("--every", type=float, default=5.0, help="seconds between saved frames")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "results" / "gazebo_camera"))
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    latest, lock, received = [None], threading.Lock(), [0]

    def on_image(msg):
        with lock:
            latest[0] = msg
            received[0] += 1

    node = Node()
    if not node.subscribe(GzImage, args.topic, on_image):
        raise SystemExit(f"could not subscribe to {args.topic}")
    t0 = time.time()
    while latest[0] is None:
        if time.time() - t0 > 20:
            raise SystemExit(f"no images on {args.topic} within 20 s")
        time.sleep(0.1)
    for k in range(args.count):
        with lock:
            msg = latest[0]
        Image.fromarray(to_array(msg)).save(out / f"frame_{k:03d}.jpg", quality=92)
        rate = received[0] / (time.time() - t0)
        print(f"saved {out / f'frame_{k:03d}.jpg'}  ({msg.width}x{msg.height}, ~{rate:.1f} frames/s)", flush=True)
        if k + 1 < args.count:
            time.sleep(args.every)


if __name__ == "__main__":
    main()
