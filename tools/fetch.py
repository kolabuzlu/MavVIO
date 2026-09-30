"""Resumable download of one URL into a file (continues a partial file; waits out rate limits).

Usage:  python fetch.py URL OUTPUT_FILE
If it stops, run it again with the same OUTPUT_FILE (a fresh URL for the same file is fine).
"""
import sys
from pathlib import Path

from remote_zip import fetch_range, get_range


def main(url, out):
    target = Path(out)
    target.parent.mkdir(parents=True, exist_ok=True)
    _, headers = get_range(url, 0, 0)
    length = int(headers["Content-Range"].split("/")[-1])
    have = target.stat().st_size if target.exists() else 0
    print(f"{target.name}: {length / 1e9:.2f} GB, {have / 1e9:.2f} GB already here", flush=True)
    fetch_range(url, 0, length, target)
    print(f"done: {target} ({target.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
