"""Step DM -- why is contour tau_area uncorrelated with distance?
OFFLINE. boxB1_approach actually flies DEAD-AHEAD into wallB1 (a
0.4x1.0x1.5 m panel at (55,0,3)); boxB1 itself is 6 m off to the side
(a fly-by), so DK's -0.076 correlation was measured against the WRONG
obstacle. Here we (1) log region identity/area per frame, (2) check
area-vs-distance-to-wallB1 over the APPROACH phase, (3) count region-ID
changes and detection dropout, (4) compare measured area to the
geometric prediction for wallB1's 1.0 m x 1.5 m front face.
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np

from obst_avoidance.frame_source import LogFrameSource
from obst_avoidance.perception import geometry
from obst_avoidance.perception.contours import ContourLoomingChannel

REC = "/home/saurabh/ardu_ws/recordings/boxB1_approach"
WALL_X = 55.0
WALL_W_M, WALL_H_M = 1.0, 1.5   # front face the camera sees (size y,z of the 0.4x1.0x1.5 panel)


def load_odom(rec):
    o = {}
    for l in open(Path(rec) / "packets.jsonl"):
        l = l.strip()
        if l:
            d = json.loads(l)
            o[d["seq"]] = d.get("position")
    return o


def main():
    src = LogFrameSource(REC)
    intr = src.intrinsics
    fx, fy = intr["fx"], intr["fy"]
    pos_by_seq = load_odom(REC)
    ch = ContourLoomingChannel(intr, geometry.N_SECTORS)

    frames = []  # (x, dist_wall, [(id, cx, cy, area)], central_region or None)
    while True:
        p = src.read()
        if p is None:
            break
        gray = cv2.cvtColor(p.image, cv2.COLOR_RGB2GRAY)
        regions = ch.update(gray, p.t_capture)
        pos = pos_by_seq.get(p.seq)
        if pos is None:
            continue
        x = pos[0]
        dist = WALL_X - x  # + while approaching, ~0 at the wall
        # "the obstacle region" = region whose centroid is nearest image centre-x
        central = None
        if regions:
            central = min(regions, key=lambda r: abs(r.cx - intr["cx"]))
        frames.append((p.seq, x, dist, regions, central))
    src.close()

    # -- approach phase only: dist decreasing from ~7 m to ~0 (x < 55) --
    appr = [f for f in frames if 0.3 < f[2] < 9.0]
    print(f"boxB1_approach -> wallB1 at x={WALL_X}. total frames={len(frames)}, "
          f"approach-phase frames (0.3<dist<9m)={len(appr)}\n")

    # (3) dropout + ID changes for the central region over the approach
    have_central = [f for f in appr if f[4] is not None]
    dropout = 1.0 - len(have_central) / len(appr) if appr else float("nan")
    ids = [f[4].region_id for f in have_central]
    id_changes = sum(1 for a, b in zip(ids, ids[1:]) if a != b)
    distinct_ids = len(set(ids))
    print(f"(3) central-region detection dropout over approach: {100*dropout:.1f}% of frames")
    print(f"(2) central-region ID: {distinct_ids} distinct IDs across {len(have_central)} frames; "
          f"{id_changes} frame-to-frame ID changes")

    # (1) area vs distance correlation over approach
    d = np.array([f[2] for f in have_central])
    a = np.array([f[4].area for f in have_central])
    if len(d) >= 2:
        corr = np.corrcoef(d, a)[0, 1]
        print(f"(1) corr(distance_to_wall, central area) = {corr:+.3f}  "
              f"(expect STRONG NEGATIVE: area grows as distance falls)")

    # (4) measured vs geometric-predicted area at 6/4/2 m
    print("\n(4) measured vs geometric-predicted front-face area (1.0m x 1.5m):")
    print(f"  {'dist':>5} {'predicted_px2':>14} {'measured_px2':>13} {'ratio':>7}")
    for target in (6.0, 4.0, 2.0):
        pred = fx * WALL_W_M * fy * WALL_H_M / (target ** 2)
        # nearest approach-frame to this distance that HAS a central region
        near = min(have_central, key=lambda f: abs(f[2] - target)) if have_central else None
        if near is not None and abs(near[2] - target) < 0.6:
            meas = near[4].area
            print(f"  {target:>5.1f} {pred:>14.0f} {meas:>13.0f} {meas/pred:>7.2f}  "
                  f"(actual dist {near[2]:.2f}m, id {near[4].region_id})")
        else:
            print(f"  {target:>5.1f} {pred:>14.0f} {'no region':>13}")

    # raw dump around the closest approach for eyeballing
    print("\nper-frame dump (approach phase, every ~5th frame): dist | #regions | central(id,area,sector)")
    for i, f in enumerate(appr):
        if i % 5 != 0:
            continue
        seq, x, dist, regions, central = f
        cstr = f"id{central.region_id} a={central.area:.0f} s{central.sector}" if central else "NONE"
        print(f"  d={dist:4.2f}m  regions={len(regions):2d}  central: {cstr}")


if __name__ == "__main__":
    main()
