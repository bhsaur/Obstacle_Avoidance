"""Phase 5 / Step CS -- build the labelled gate-training dataset from
logged closed-loop runs. OFFLINE, no sim.

The gate is the learning-to-defer decision: given ONLY CheapStage's
per-frame feature vector (never depth, never ground truth, never
HeavyStage's output), predict whether this frame is on a
soon-to-collide trajectory -- i.e. whether the cheap stage should
defer to the expensive one.

LABEL (Step CS): label=1 if the run collides within LABEL_HORIZON_S
seconds of THIS frame's t_capture, else 0. Time-based, NOT a frame
count -- runs were logged at different perception rates (CheapStage
~25-30Hz here), so a fixed frame count would mean different amounts of
real lead-time per run. Collision time is the run's LAST logged frame's
t_capture: verified (gate_dataset self-check + README Step CS) that the
last logged vehicle_position equals the run summary's collision_xy to
0.00 m in every collided run, i.e. the run terminates AT collision.

USABLE DATA (an honest constraint found in Step CS, not designed
around): only runs whose logged `source` is "cheap" AND whose logged
feature dict has the expected 54 keys (n_sectors=5 layout) are usable.
  - HEAVY-only runs log an EMPTY feature dict ({}) -- HeavyStage does
    not compute CheapStage's feature vector, so despite the task's
    framing there is nothing for the gate to read in them. Excluded.
  - The newer n_sectors=11 cheap runs (flow_co/flow_cr) log a
    102-element vector with a DIFFERENT per-sector layout, not
    concatenable with the 54-element one. The task specifies "the
    54-element cheap-stage vector, unchanged", so those are excluded
    too (reported, not silently dropped).

This module only BUILDS and labels the matrix; training/eval lives in
gate_train.py. Kept separate so the dataset definition is auditable on
its own and reused unchanged by the heuristic baseline (Step CV).
"""
import json
import glob
import os
from typing import List, Tuple

import numpy as np

from obst_avoidance.perception.features import feature_names, extract_features

N_SECTORS_CANONICAL = 5
N_FEATURES_CANONICAL = 54
LABEL_HORIZON_S = 2.0  # Step CS: ~2 s of flight. ONE setting, not tuned (task constraint).

EVAL_DIR = "/home/saurabh/ardu_ws/eval_results"


def _iter_frames(path: str):
    for line in open(path):
        line = line.strip()
        if line:
            yield json.loads(line)


def _first_frame(path: str):
    for d in _iter_frames(path):
        return d
    return None


def discover_usable_runs(eval_dir: str = EVAL_DIR):
    """Returns (usable, excluded) where usable is a list of frame-log
    paths with source=cheap and 54-key features, and excluded is a list
    of (path, reason) for everything else -- so Step CS can report
    exactly what was and wasn't included and why."""
    usable, excluded = [], []
    for path in sorted(glob.glob(os.path.join(eval_dir, "*_frames.jsonl"))):
        f = _first_frame(path)
        if f is None:
            excluded.append((path, "empty log"))
            continue
        src = f.get("source")
        nfeat = len(f.get("features", {}))
        if src == "cheap" and nfeat == N_FEATURES_CANONICAL:
            usable.append(path)
        elif src == "heavy":
            excluded.append((path, "heavy source -- empty cheap feature dict"))
        elif src == "cheap" and nfeat != N_FEATURES_CANONICAL:
            excluded.append((path, f"cheap but {nfeat}-element features (n_sectors!=5 layout)"))
        else:
            excluded.append((path, f"source={src} nfeat={nfeat}"))
    return usable, excluded


def _run_zone(tag: str) -> str:
    """Collision zone from the run's summary JSON (fallback 'unknown')."""
    sj = os.path.join(EVAL_DIR, tag + ".json")
    if os.path.exists(sj) and os.path.getsize(sj) > 0:
        try:
            return str(json.load(open(sj)).get("collision_zone"))
        except Exception:
            return "unknown"
    return "unknown"


def build_dataset(eval_dir: str = EVAL_DIR, horizon_s: float = LABEL_HORIZON_S):
    """Returns X (n_frames, 54), y (n_frames,), run_ids (n_frames,),
    zones (n_frames,), plus meta lists. Raw features WITH NaN preserved
    -- imputation/standardization is the trainer's job (fit per fold to
    avoid leakage), not baked in here."""
    usable, excluded = discover_usable_runs(eval_dir)
    names = feature_names(N_SECTORS_CANONICAL)

    X_list, y_list, run_list, zone_list = [], [], [], []
    per_run = []  # (tag, n_frames, n_pos, zone, collided)
    for ri, path in enumerate(usable):
        tag = os.path.basename(path).replace("_frames.jsonl", "")
        frames = list(_iter_frames(path))
        if not frames:
            continue
        t_last = frames[-1]["t_capture"]
        # collided is asserted by the summary; all usable runs collided,
        # but read it rather than assume, so a future safe run labels 0.
        sj = os.path.join(eval_dir, tag + ".json")
        collided = True
        if os.path.exists(sj) and os.path.getsize(sj) > 0:
            try:
                collided = bool(json.load(open(sj)).get("collided"))
            except Exception:
                pass
        zone = _run_zone(tag)
        npos = 0
        for d in frames:
            X_list.append(extract_features(d["features"], N_SECTORS_CANONICAL))
            if collided and (t_last - d["t_capture"]) <= horizon_s:
                y = 1
                npos += 1
            else:
                y = 0
            y_list.append(y)
            run_list.append(tag)
            zone_list.append(zone)
        per_run.append((tag, len(frames), npos, zone, collided))

    X = np.array(X_list, dtype=np.float64)
    y = np.array(y_list, dtype=np.int64)
    run_ids = np.array(run_list)
    zones = np.array(zone_list)
    return X, y, run_ids, zones, names, per_run, excluded


if __name__ == "__main__":
    X, y, run_ids, zones, names, per_run, excluded = build_dataset()
    print(f"=== Step CS dataset ===")
    print(f"total frames: {X.shape[0]}   features: {X.shape[1]}")
    print(f"distinct runs: {len(per_run)}")
    print(f"label balance: {int(y.sum())} positive ({100*y.mean():.2f}%), "
          f"{int((y==0).sum())} negative ({100*(1-y.mean()):.2f}%)")
    print(f"label horizon: {LABEL_HORIZON_S} s")

    print("\n--- per zone ---")
    for z in sorted(set(zones)):
        m = zones == z
        print(f"  {z:<10} frames={int(m.sum()):<7} pos={int(y[m].sum()):<6} ({100*y[m].mean():.1f}%)  "
              f"runs={len(set(run_ids[m]))}")

    print("\n--- per run ---")
    for tag, nf, npos, zone, collided in per_run:
        print(f"  {tag:<14} frames={nf:<6} pos={npos:<5} ({100*npos/nf:4.1f}%)  zone={zone:<8} collided={collided}")

    print(f"\n--- EXCLUDED ({len(excluded)}) ---")
    from collections import Counter
    reasons = Counter(r for _, r in excluded)
    for reason, c in reasons.most_common():
        print(f"  {c:>3}  {reason}")
