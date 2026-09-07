"""Step DH.3 -- row band ON vs OFF on the textured approach.
Reports (a) correlation(distance_to_obstacle, min_tau) -- positive means
tau falls as the obstacle nears, the property the estimator is supposed
to have; (b) per-sector valid-rate change. OFFLINE.

Odom (position/velocity/attitude/odom_valid) is read from the
recording's packets.jsonl keyed by seq and passed to CheapStage.infer()
exactly as in flight, so both runs see identical inputs -- only
use_row_band differs.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from obst_avoidance.frame_source import LogFrameSource
from obst_avoidance.perception import CheapStage, geometry
from obst_avoidance.perception.features import feature_names

MIN_TAU_IDX = feature_names().index("min_tau")
N = geometry.N_SECTORS

# obstacle world positions (orchestrator.OBSTACLES)
OBSTACLES = {
    "textured_approach_treeA2": (20.0, 3.8),
    "boxB1_approach": (58.0, -6.0),
    "wall_approach_1p0": (55.0, 0.0),
    "narrow_approach_D1": (100.0, 0.0),
}


def load_odom_by_seq(recording):
    rows = {}
    for line in open(Path(recording) / "packets.jsonl"):
        line = line.strip()
        if not line:
            continue
        d = json.loads(line)
        rows[d["seq"]] = (d.get("position"), d.get("velocity"),
                          d.get("attitude_quat"), d.get("odom_valid", True))
    return rows


def run(recording, use_row_band, obstacle_xy):
    src = LogFrameSource(recording)
    intr = src.intrinsics
    odom_by_seq = load_odom_by_seq(recording)
    cheap = CheapStage(intr, use_row_band=use_row_band)
    prev = None
    dists, min_taus = [], []
    valid_count = np.zeros(N)
    n_frames = 0
    while True:
        pkt = src.read()
        if pkt is None:
            break
        if prev is None:
            prev = pkt
            continue
        odom = odom_by_seq.get(pkt.seq, (None, None, None, False))
        belief, feats = cheap.infer(pkt, prev, odom)
        pos = odom[0]
        if pos is not None:
            d = math.hypot(pos[0] - obstacle_xy[0], pos[1] - obstacle_xy[1])
            dists.append(d)
            min_taus.append(feats[MIN_TAU_IDX])
        valid_count += belief.valid.astype(float)
        n_frames += 1
        prev = pkt
    src.close()
    return np.array(dists), np.array(min_taus), valid_count, n_frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recording", default="/home/saurabh/ardu_ws/recordings/textured_approach_treeA2")
    args = ap.parse_args()
    name = Path(args.recording).name
    obs = OBSTACLES.get(name)
    if obs is None:
        raise SystemExit(f"no obstacle position known for {name}")

    print(f"=== DH.3: row band ON vs OFF on {name} (obstacle at {obs}) ===\n")
    results = {}
    for use in (False, True):
        d, mt, vc, nf = run(args.recording, use, obs)
        ok = ~np.isnan(mt)
        corr = float(np.corrcoef(d[ok], mt[ok])[0, 1]) if ok.sum() >= 2 else float("nan")
        results[use] = (corr, vc, nf, int(ok.sum()))
        tag = "ON " if use else "OFF"
        print(f"[band {tag}] correlation(distance, min_tau) = {corr:+.3f}   "
              f"(valid min_tau frames: {int(ok.sum())}/{nf})")
    print("\nper-sector valid-rate (fraction of frames the sector was valid):")
    corr_off, vc_off, nf, _ = results[False]
    corr_on, vc_on, nf_on, _ = results[True]
    print(f"  {'sec':>3} {'OFF%':>7} {'ON%':>7} {'delta':>8}")
    for s in range(N):
        off = 100 * vc_off[s] / nf
        on = 100 * vc_on[s] / nf_on
        print(f"  {s:>3} {off:>7.1f} {on:>7.1f} {on-off:>+8.1f}")
    print(f"\n  overall valid-rate: OFF {100*vc_off.sum()/(nf*N):.1f}%  "
          f"ON {100*vc_on.sum()/(nf_on*N):.1f}%")
    print(f"\nSUMMARY: correlation OFF={results[False][0]:+.3f} -> ON={results[True][0]:+.3f}")


if __name__ == "__main__":
    main()
