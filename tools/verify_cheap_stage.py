#!/usr/bin/env python3
"""CheapStage VERIFY harness -- THROWAWAY DIAGNOSTIC, not production
code. Replays an existing recording through CheapStage via
LogFrameSource (no sim). Reports what CheapStage actually does on that
recording -- does NOT tune anything to make TTC look good. See the
CheapStage Phase 2 task's VERIFY section for what each --check is
supposed to demonstrate (or fail to).

LogFrameSource's FramePacket does not carry position/velocity/
attitude_quat/odom_valid (frame_source/packet.py is intentionally
source-agnostic) -- those live only in packets.jsonl, written by
record_pass.py. This harness reads packets.jsonl directly, keyed by
seq, to reconstruct the `odom` tuple CheapStage.infer() expects.
Recordings made before Step V (odometry interpolation) have no
odom_valid column at all -- treated as odom_valid=True with a printed
warning, since those recordings can't distinguish stale odometry from
fresh.

Usage:
  tools/verify_cheap_stage.py --recording <dir> --check {yaw_only,wall_approach,generic} \\
      [--limit N] [--wall-x 55.0] [--out-dir <dir>]
"""
import argparse
import json
from pathlib import Path

import numpy as np

from obst_avoidance.frame_source import LogFrameSource
from obst_avoidance.perception import CheapStage
from obst_avoidance.perception.features import feature_names
from obst_avoidance.perception.geometry import N_SECTORS

MIN_TAU_IDX = feature_names().index("min_tau")  # looked up, not hardcoded -- see features.py


def load_odom_by_seq(recording_dir):
    path = Path(recording_dir) / "packets.jsonl"
    rows = {}
    has_odom_valid = True
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if "odom_valid" not in row:
                has_odom_valid = False
            rows[row["seq"]] = (
                row.get("position"),
                row.get("velocity") or (0.0, 0.0, 0.0),
                row.get("attitude_quat"),
                row.get("odom_valid", True),
            )
    if not has_odom_valid:
        print("WARNING: this recording predates Step V -- no odom_valid column, "
              "treating every frame as odom_valid=True")
    return rows


def replay(recording, limit):
    """Runs CheapStage over every consecutive frame pair in `recording`,
    up to `limit` pairs. Returns a list of dicts, one per pair, each
    with seq, t_capture, position, belief (SectorBelief), features
    (np.ndarray)."""
    source = LogFrameSource(recording)
    intr = source.intrinsics
    odom_by_seq = load_odom_by_seq(recording)
    stage = CheapStage(intr)

    prev = None
    results = []
    while True:
        if len(results) >= limit:
            break
        p = source.read()
        if p is None:
            break
        if prev is not None and len(results) < limit:
            odom = odom_by_seq.get(p.seq, (None, (0.0, 0.0, 0.0), None, False))
            belief, feats = stage.infer(p, prev, odom)
            results.append({
                "seq": p.seq,
                "t_capture": p.t_capture,
                "position": odom[0],
                "belief": belief,
                "features": feats,
            })
        prev = p
    return results


def report_valid_rate(results, label):
    n = len(results)
    if n == 0:
        print(f"{label}: no frame pairs processed")
        return
    valid_counts = np.zeros(N_SECTORS)
    for r in results:
        valid_counts += r["belief"].valid.astype(float)
    print(f"{label}: {n} frame pairs, valid rate per sector:")
    for s in range(N_SECTORS):
        print(f"  sector {s}: {valid_counts[s]/n*100:.1f}% valid "
              f"({int(valid_counts[s])}/{n})")


def check_yaw_only(recording, limit):
    """Zero commanded translation -> true TTC is infinite everywhere.
    ANY finite, LOW tau here means rotational flow is leaking into the
    divergence estimate (the sharpest test CheapStage has -- de-rotation
    is supposed to remove exactly this)."""
    results = replay(recording, limit)
    report_valid_rate(results, f"yaw_only ({recording})")

    all_valid_taus = []
    low_tau_frames = 0
    for r in results:
        belief = r["belief"]
        names = feature_names(len(belief.scores))
        valid_taus = np.array([
            r["features"][names.index(f"tau_sector_{s}")]
            for s in range(len(belief.scores)) if belief.valid[s]
        ])
        if len(valid_taus):
            all_valid_taus.extend(valid_taus.tolist())
            if np.any(valid_taus < 3.0):  # 3s is well inside "about to hit something", never true here
                low_tau_frames += 1
    if all_valid_taus:
        from obst_avoidance.perception.cheap import TAU_CAP_S
        arr = np.array(all_valid_taus)
        print(f"  valid-sector tau: n={len(arr)} mean={arr.mean():.2f}s "
              f"median={np.median(arr):.2f}s min={arr.min():.2f}s "
              f"(tau_cap={TAU_CAP_S:.0f}s -- capped, so 'near-infinite' shows as "
              f"this cap, not literal inf)")
        print(f"  frames with ANY valid sector tau < 3.0s (spurious "
              f"'about to hit something' signal): {low_tau_frames}/{len(results)} "
              f"({low_tau_frames/max(len(results),1)*100:.1f}%)")
    else:
        print("  no sector was ever valid -- cannot evaluate tau leakage "
              "(every sector had too few points, see valid-rate above)")


def check_wall_approach(recording, limit, wall_x, out_dir):
    """min tau should DECREASE as ground-truth distance to the wall
    decreases (from interpolated odometry position, not commanded
    anything). Plots distance-to-wall vs. min_tau."""
    results = replay(recording, limit)
    report_valid_rate(results, f"wall_approach ({recording})")

    distances = []
    min_taus = []
    for r in results:
        pos = r["position"]
        if pos is None:
            continue
        dist = wall_x - pos[0]
        min_tau = r["features"][MIN_TAU_IDX]
        distances.append(dist)
        min_taus.append(min_tau)
    distances = np.array(distances)
    min_taus = np.array(min_taus)
    valid_mask = ~np.isnan(min_taus)
    n_valid = int(valid_mask.sum())
    print(f"  {n_valid}/{len(results)} frames had at least one valid sector "
          f"(min_tau defined)")
    if n_valid >= 3:
        corr = float(np.corrcoef(distances[valid_mask], min_taus[valid_mask])[0, 1])
        print(f"  correlation(distance_to_wall, min_tau) = {corr:.3f} "
              f"(positive = min_tau falls as distance falls, as expected)")
    else:
        print("  too few valid frames to compute a correlation")

    if out_dir:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        fig, ax = plt.subplots(figsize=(7, 5))
        ax.scatter(distances[valid_mask], min_taus[valid_mask], s=10, alpha=0.6, label="valid")
        invalid_mask = ~valid_mask
        if np.any(invalid_mask):
            ax.scatter(distances[invalid_mask], np.zeros(invalid_mask.sum()) - 1,
                       s=10, alpha=0.3, color="red", marker="x", label="no valid sector (min_tau undefined)")
        ax.set_xlabel("distance to wall (m, from interpolated odometry)")
        ax.set_ylabel("min_tau (s, capped)")
        ax.set_title(f"wall_approach: min_tau vs. distance -- {Path(recording).name}")
        ax.legend()
        ax.invert_xaxis()  # approaching -> distance decreasing left-to-right
        out_path = Path(out_dir) / f"wall_approach_tau_vs_distance_{Path(recording).name}.png"
        fig.savefig(out_path, dpi=120)
        plt.close(fig)
        print(f"  plot saved: {out_path}")


def check_generic(recording, limit):
    """No specific ground-truth expectation -- just report per-zone
    feature-vector statistics and valid=False rate per sector."""
    results = replay(recording, limit)
    report_valid_rate(results, f"generic ({recording})")
    if results:
        confidences = np.array([r["belief"].confidence for r in results])
        print(f"  confidence: mean={confidences.mean():.3f} min={confidences.min():.3f} "
              f"max={confidences.max():.3f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--recording", type=str, required=True)
    parser.add_argument("--check", type=str, default="generic",
                         choices=["yaw_only", "wall_approach", "generic"])
    parser.add_argument("--limit", type=int, default=1000)
    parser.add_argument("--wall-x", type=float, default=55.0, help="wall_approach only: wall's x position (m)")
    parser.add_argument("--out-dir", type=str, default=None, help="wall_approach only: where to save the plot")
    args = parser.parse_args()

    if args.check == "yaw_only":
        check_yaw_only(args.recording, args.limit)
    elif args.check == "wall_approach":
        check_wall_approach(args.recording, args.limit, args.wall_x, args.out_dir)
    else:
        check_generic(args.recording, args.limit)


if __name__ == "__main__":
    main()
