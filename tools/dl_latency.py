"""Step DL -- CheapStage latency, contour channel ON vs OFF. OFFLINE,
isolated (LogFrameSource, no sim), matching how the ~8 ms baseline was
measured. Reports mean/median/p95 per-infer() wall time over a
recording, both ways.
"""
import json
import time
from pathlib import Path

import numpy as np

from obst_avoidance.frame_source import LogFrameSource
from obst_avoidance.perception import CheapStage


def load_odom(rec):
    o = {}
    for l in open(Path(rec) / "packets.jsonl"):
        l = l.strip()
        if l:
            d = json.loads(l)
            o[d["seq"]] = (d.get("position"), d.get("velocity"), d.get("attitude_quat"), d.get("odom_valid", True))
    return o


def measure(rec, use_contours):
    src = LogFrameSource(rec)
    intr = src.intrinsics
    odom = load_odom(rec)
    c = CheapStage(intr, use_contours=use_contours)
    prev = None
    lat = []
    while True:
        p = src.read()
        if p is None:
            break
        if prev is None:
            prev = p; continue
        t0 = time.perf_counter()
        c.infer(p, prev, odom.get(p.seq, (None, None, None, False)))
        lat.append((time.perf_counter() - t0) * 1000.0)
        prev = p
    src.close()
    return np.array(lat)


def main():
    # corridor_segment2 -- long, texture-rich, representative of live perception load
    rec = "/home/saurabh/ardu_ws/recordings/corridor_segment2"
    print(f"latency on {Path(rec).name} (isolated, LogFrameSource)\n")
    for use in (False, True):
        lat = measure(rec, use)
        tag = "ON " if use else "OFF"
        print(f"[contours {tag}] n={len(lat)}  mean={lat.mean():.2f}ms  median={np.median(lat):.2f}ms  "
              f"p95={np.percentile(lat,95):.2f}ms  max={lat.max():.2f}ms")


if __name__ == "__main__":
    main()
