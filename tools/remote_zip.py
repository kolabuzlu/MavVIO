"""List or extract single files from a large remote .zip without downloading the whole archive.

Works with any server that supports HTTP range requests. Only the archive's table of contents and
the chosen files are transferred. A file stored uncompressed in the archive (the usual case for
recordings) is fetched with one continuous request, and resumed where it stopped if the server
interrupts it or asks us to slow down (HTTP 429).

Usage:
  python remote_zip.py URL                          # list the archive's files and sizes
  python remote_zip.py URL MEMBER [MEMBER ...] -o DIR   # extract the named files into DIR
"""
import argparse
import io
import json
import shutil
import struct
import sys
import time
import urllib.request
import zipfile
from pathlib import Path

UA = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)"}


def polite_wait(error, attempt):
    """Seconds to wait after a failed request: the server's Retry-After if given, else 60 s doubling to 5 min."""
    headers = getattr(error, "headers", None)
    retry_after = headers.get("Retry-After") if headers else None
    if retry_after and retry_after.isdigit():
        return int(retry_after) + 5
    return min(60 * 2 ** attempt, 300)


def get_range(url, first, last, attempts=12):
    """Bytes first..last of url, waiting and retrying when the server refuses or the connection drops."""
    for attempt in range(attempts):
        try:
            req = urllib.request.Request(url, headers={**UA, "Range": f"bytes={first}-{last}"})
            with urllib.request.urlopen(req, timeout=120) as r:
                return r.read(), r.headers
        except OSError as e:
            if attempt == attempts - 1:
                raise
            pause = polite_wait(e, attempt)
            print(f"  request refused ({e}); retrying in {pause} s", flush=True)
            time.sleep(pause)


class HttpRangeFile(io.RawIOBase):
    """Seekable read-only view of a remote file, fetched with HTTP range requests."""

    def __init__(self, url):
        self.url, self.pos = url, 0
        _, headers = get_range(url, 0, 0)
        self.size = int(headers["Content-Range"].split("/")[-1])

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = {0: off, 1: self.pos + off, 2: self.size + off}[whence]
        return self.pos

    def readinto(self, b):
        if self.pos >= self.size:
            return 0
        end = min(self.pos + len(b), self.size) - 1
        data, _ = get_range(self.url, self.pos, end)
        b[: len(data)] = data
        self.pos += len(data)
        return len(data)


def open_remote_zip(url):
    return zipfile.ZipFile(io.BufferedReader(HttpRangeFile(url), buffer_size=8 << 20))


def stored_member_range(z, name):
    """(first byte, length) of an uncompressed member's data inside the archive."""
    info = z.getinfo(name)
    if info.compress_type != zipfile.ZIP_STORED:
        raise ValueError(f"{name} is compressed; only uncompressed members can be fetched directly")
    z.fp.seek(info.header_offset)
    header = z.fp.read(30)
    name_len, extra_len = struct.unpack("<HH", header[26:30])
    return info.header_offset + 30 + name_len + extra_len, info.file_size


def fetch_range(url, start, length, target, attempts=20):
    """Download bytes start..start+length-1 of url into target, resuming after interruptions."""
    failures = 0
    while True:
        done = target.stat().st_size if target.exists() else 0
        if done >= length:
            return
        req = urllib.request.Request(url, headers={**UA, "Range": f"bytes={start + done}-{start + length - 1}"})
        try:
            with urllib.request.urlopen(req, timeout=120) as r, open(target, "ab") as f:
                while chunk := r.read(1 << 20):
                    f.write(chunk)
            failures = 0
        except OSError as e:
            failures += 1
            if failures >= attempts:
                raise
            pause = polite_wait(e, failures - 1)
            now = target.stat().st_size if target.exists() else 0
            print(f"  interrupted at {now / 1e6:.0f} of {length / 1e6:.0f} MB ({e}); continuing in {pause} s", flush=True)
            time.sleep(pause)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("url")
    ap.add_argument("members", nargs="*")
    ap.add_argument("-o", "--out", default=".")
    args = ap.parse_args()
    if not args.members:
        z = open_remote_zip(args.url)
        for i in z.infolist():
            if not i.is_dir():
                print(f"{i.file_size / 1e6:10.1f} MB  {i.filename}")
        return
    out = Path(args.out)
    z = None
    for name in args.members:
        target = out / Path(name).name
        target.parent.mkdir(parents=True, exist_ok=True)
        where = target.with_name(target.name + ".where.json")   # remembered position inside the archive
        t = time.time()
        known = json.loads(where.read_text()) if where.exists() else None
        if not known or known.get("url") != args.url or known.get("member") != name:
            z = z or open_remote_zip(args.url)
            try:
                start, length = stored_member_range(z, name)
            except ValueError:
                with z.open(name) as src, open(target, "wb") as dst:
                    shutil.copyfileobj(src, dst, 8 << 20)
                print(f"extracted {name} -> {target} ({target.stat().st_size / 1e6:.0f} MB in {time.time() - t:.0f} s)")
                continue
            known = {"url": args.url, "member": name, "start": start, "length": length}
            where.write_text(json.dumps(known))
        print(f"{name}: {known['length'] / 1e6:.0f} MB stored uncompressed; fetching directly "
              f"({(target.stat().st_size if target.exists() else 0) / 1e6:.0f} MB already here)", flush=True)
        fetch_range(args.url, known["start"], known["length"], target)
        where.unlink()
        print(f"extracted {name} -> {target} ({target.stat().st_size / 1e6:.0f} MB in {time.time() - t:.0f} s)")


if __name__ == "__main__":
    sys.exit(main())
