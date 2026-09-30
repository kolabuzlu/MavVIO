#!/usr/bin/env python3
"""Live view of the Gazebo cameras in your Windows browser - no Linux window needed.

Serves the chase camera (behind the plane, 10 frames/s), the plane's downward VIO camera (20 frames/s) and
the tilted VIO cameras (10 frames/s, if the world has them) as live pictures on http://localhost:8080. Both are rendered inside the simulation, in step with the
physics, so the view is smooth - unlike a Gazebo window under WSL. Runs inside Ubuntu with Gazebo's
Python bindings; run_gazebo_vio.sh starts it.

Usage:  python3 /mnt/c/Users/funfo/vio/tools/gz_web_view.py [--port 8080]
"""
import argparse
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node
from PIL import Image

STREAMS = {"chase": "/vio/chase_camera", "vio": "/vio/camera", "t45": "/vio/camera_t45", "t30": "/vio/camera_t30"}
MAX_FPS = {"chase": 10, "vio": 20, "t45": 10, "t30": 10}
PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>VIO simulation - live</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { margin: 0; background: #16181b; color: #d9dadb; font: 14px system-ui, sans-serif; }
  h1 { font-size: 16px; font-weight: 600; margin: 12px 16px 0; }
  .row { display: flex; flex-wrap: wrap; gap: 12px; padding: 12px 16px; align-items: flex-start; }
  figure { margin: 0; }
  .chase { flex: 3 1 560px; max-width: 960px; }
  .vio { flex: 1 1 320px; max-width: 640px; }
  .tilt { flex: 0 1 480px; }
  img { width: 100%; display: block; background: #25282c; border-radius: 4px; }
  figcaption { padding: 6px 2px; color: #a4a7ab; }
</style></head>
<body><h1>VIO simulation - live from Gazebo</h1>
<div class="row">
FIGURES
</div></body></html>
"""
FIGURES = {
    "chase": '<figure class="chase"><img src="/chase.mjpg" alt="chase camera"><figcaption>Chase camera: 10 m behind and 3 m above the plane</figcaption></figure>',
    "vio": '<figure class="vio"><img src="/vio.mjpg" alt="VIO camera"><figcaption>VIO camera: straight down, top of the picture = nose</figcaption></figure>',
    "t45": '<figure class="tilt"><img src="/t45.mjpg" alt="VIO camera tilted 45 degrees"><figcaption>VIO camera tilted 45&deg; down (from the nose)</figcaption></figure>',
    "t30": '<figure class="tilt"><img src="/t30.mjpg" alt="VIO camera tilted 30 degrees"><figcaption>VIO camera tilted 30&deg; down (from the nose)</figcaption></figure>',
}


class Latest:
    """The newest JPEG of one camera, and a way to wait for the next one."""

    def __init__(self, max_fps):
        self.jpeg, self.stamp, self.cond, self.period = None, 0.0, threading.Condition(), 1.0 / max_fps

    def on_image(self, msg):
        now = time.time()
        if now - self.stamp < self.period * 0.9:
            return
        channels = 3 if msg.pixel_format_type == 3 else 1          # RGB_INT8 or L_INT8
        rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, -1)[:, :msg.width * channels]
        buf = io.BytesIO()
        Image.fromarray(rows.reshape(msg.height, msg.width, channels).squeeze()).save(buf, "JPEG", quality=85)
        with self.cond:
            self.jpeg, self.stamp = buf.getvalue(), now
            self.cond.notify_all()


LATEST = {name: Latest(MAX_FPS[name]) for name in STREAMS}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        name = self.path.lstrip("/").replace(".mjpg", "")
        if name not in LATEST:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        latest, last = LATEST[name], 0.0
        try:
            while True:
                with latest.cond:
                    latest.cond.wait_for(lambda: latest.stamp > last, timeout=5)
                    jpeg, last = latest.jpeg, latest.stamp
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--streams", default="chase,vio",
                    help="cameras to show: chase, vio, and t45, t30 if the world has the tilted cameras "
                         "(make_scenery.py --tilts 30,45)")
    args = ap.parse_args()
    global PAGE
    PAGE = PAGE.replace("FIGURES", "\n".join(FIGURES[name] for name in args.streams.split(",")))
    node = Node()
    for name in args.streams.split(","):
        if not node.subscribe(GzImage, STREAMS[name], LATEST[name].on_image):
            print(f"could not subscribe to {STREAMS[name]}")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.daemon_threads = True
    print(f"live view on http://localhost:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
