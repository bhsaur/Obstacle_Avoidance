"""Step DP dataset -- build a 168-feature, labelled gate dataset by
REPLAYING the image recordings through the DN CheapStage (contours on).
OFFLINE.

WHY recordings and not the Phase-5 logs: the Phase-5 gate-training logs
(flow08/run*/zoneC/zoneD) are FEATURE-ONLY -- no images -- so the
contour channel cannot be retrofitted to them. The only image data is
recordings/ (~13k frames spanning zones A-D). So DP/DQ/DR run on the
recordings, with a matched 102-vs-168 comparison on IDENTICAL frames
(the 102 baseline = the same vectors with the 66 contour columns
dropped). This means the absolute AUC is NOT directly the Phase-5 0.900
(different data); the load-bearing number is the 102-vs-168 delta on the
same frames, and the leave-one-zone-out generalisation (Step DR).

LABEL (ground-truth geometry, no perception, no disagreement labels):
label=1 if the vehicle's logged trajectory reaches CONTACT with any
obstacle within LABEL_HORIZON_S seconds of this frame's t_capture, where
contact = horizontal centre-distance < (obstacle characteristic size +
COLLISION_RADIUS_M). Time-based (t_capture), matching Step CS. Frames
whose recording never contacts an obstacle are all label 0 -- giving
genuine NEGATIVE runs the Phase-5 data lacked.
"""
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np

from obst_avoidance.frame_source import LogFrameSource
from obst_avoidance.perception import CheapStage, geometry
from obst_avoidance.perception.features import feature_names, build_feature_spec

LABEL_HORIZON_S = 2.0
COLLISION_RADIUS_M = 0.35
REC_BASE = "/home/saurabh/ardu_ws/recordings"
CACHE = "/home/saurabh/ardu_ws/eval_results/gate_recordings_168.npz"

# obstacle map (orchestrator.OBSTACLES) -- (x, y, characteristic_size)
OBSTACLES = [
    (17.0, -3.8, 0.25), (20.0, 3.8, 0.25), (23.0, -2.8, 0.75), (24.0, 4.2, 0.22),
    (55.0, 0.0, 0.5), (58.0, -6.0, 1.0),
    (86.0, -1.5, 0.25), (87.0, 1.5, 0.25), (89.0, -3.0, 0.75), (89.0, 0.0, 0.75),
    (91.0, 2.5, 0.22), (93.0, -1.0, 0.65), (94.0, 1.0, 0.25),
    (108.0, 0.0, 1.75), (116.0, -3.5, 0.25), (119.0, 2.0, 0.9), (122.0, 0.0, 5.0),
]
ZONE_X = [("zone_A", 10, 30), ("zone_B", 40, 70), ("zone_C", 75, 100), ("zone_D", 100, 140)]

RECORDINGS = [
    "textured_approach_treeA2", "boxB1_approach", "panelB1_approach",
    "wallB1_close_approach", "wall_approach_1p0", "wall_approach_1p5",
    "wall_approach_2p0", "narrow_approach_D1", "corridor_segment1",
    "corridor_segment2", "corridor_segment3", "corridor_segment4",
    "corridor_zonec_fill", "corridor_zonec_fill2", "empty_gap1",
    "odom_sync_check1", "roll_pitch_wiggle1",
]


def zone_of_x(x):
    for z, lo, hi in ZONE_X:
        if lo <= x < hi:
            return z
    return "other"


def contact_dist(pos):
    """min over obstacles of (centre-dist - char - collision_radius); <=0 means contact."""
    best = 1e9
    for ox, oy, ch in OBSTACLES:
        d = math.hypot(pos[0] - ox, pos[1] - oy) - ch - COLLISION_RADIUS_M
        best = min(best, d)
    return best


def build():
    names = feature_names(geometry.N_SECTORS)
    X, y, runs, zones, ts = [], [], [], [], []
    for rec in RECORDINGS:
        path = os.path.join(REC_BASE, rec)
        if not os.path.isdir(path):
            continue
        # odom by seq
        pos_by_seq, t_by_seq = {}, {}
        for l in open(Path(path) / "packets.jsonl"):
            l = l.strip()
            if not l:
                continue
            d = json.loads(l)
            pos_by_seq[d["seq"]] = d.get("position")
            t_by_seq[d["seq"]] = d["t_capture"]

        # first pass: per-frame contact distance + t, for labelling
        seqs = sorted(t_by_seq)
        contact = {s: (contact_dist(pos_by_seq[s]) if pos_by_seq.get(s) else 1e9) for s in seqs}

        # replay features
        src = LogFrameSource(path)
        intr = src.intrinsics
        cheap = CheapStage(intr, use_contours=True)
        prev = None
        while True:
            p = src.read()
            if p is None:
                break
            if prev is None:
                prev = p; continue
            odom = (pos_by_seq.get(p.seq), None, None, True)
            _, fv = cheap.infer(p, prev, odom)
            pos = pos_by_seq.get(p.seq)
            # label: contact within next 2s
            t0 = p.t_capture
            lab = 0
            for s in seqs:
                if 0 <= (t_by_seq[s] - t0) <= LABEL_HORIZON_S and contact[s] <= 0.0:
                    lab = 1
                    break
            X.append(fv)
            y.append(lab)
            runs.append(rec)
            zones.append(zone_of_x(pos[0]) if pos else "other")
            ts.append(t0)
            prev = p
        src.close()
        print(f"  {rec}: done ({sum(1 for r in runs if r==rec)} frames, "
              f"{sum(1 for r,l in zip(runs,y) if r==rec and l==1)} positive)")

    X = np.array(X, dtype=np.float64)
    y = np.array(y, dtype=np.int64)
    runs = np.array(runs); zones = np.array(zones); ts = np.array(ts)
    np.savez(CACHE, X=X, y=y, runs=runs, zones=zones, ts=ts, names=np.array(names))
    print(f"\nsaved {CACHE}: X={X.shape}, {int(y.sum())} pos ({100*y.mean():.2f}%)")
    return X, y, runs, zones, names


if __name__ == "__main__":
    print("building 168-feature recordings dataset (replaying through DN CheapStage)...")
    build()
