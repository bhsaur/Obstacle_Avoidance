#!/usr/bin/env python3
"""Step CN -- offline replay of a logged belief sequence through the
FIXED SectorController (Step CM's world-bearing fix + the n_sectors=5
vs 11 fix found while preparing this replay), to verify the commanded
yaw-rate trace now converges instead of accumulating. No sim, no ROS.

Input: eval_results/heavy_cl_run{N}_frames.jsonl -- per-frame belief
(scores/valid/confidence/source), recorded under the BROKEN (Step CL)
controller. Only the BELIEF SEQUENCE is replayed (scores/valid/
confidence/source, in logged order, at the logged t_capture spacing);
vehicle_position/vehicle_velocity from that log are NOT used as the
yaw input.

Why not: an earlier version of this script derived yaw from the
LOGGED vehicle_velocity (atan2(vy, vx), same proxy Step CL's diagnosis
used) and fed that into the fixed controller. That is NOT an honest
convergence test: the logged velocity trace was produced by the
BROKEN controller spinning continuously (to -177 deg and beyond), so
feeding those already-diverged yaw values into the FIXED controller
just shows it correctly applying MORE correction as the (externally
imposed, ever-growing) error grows -- saturating at yaw_rate_max, not
converging, because the input trajectory itself never approaches the
target. That is expected and correct given that input, but it answers
the wrong question.

Instead, yaw here is SELF-INTEGRATED from the fixed controller's own
output: yaw[0] = 0.0, yaw[i] = wrap_to_pi(yaw[i-1] + yaw_rate[i-1] *
dt) where dt is the logged t_capture spacing and yaw_rate[i-1] is what
THIS controller commanded on the previous step. This is still fully
offline (no sim, no dynamics model beyond a single yaw integrator) and
answers Step CN's actual question: given this real logged belief
sequence (the box entering frame, sector scores dropping, exactly as
HeavyStage actually saw it), if the fixed controller's own commands
were the thing turning the vehicle, does yaw settle onto the target
bearing instead of accumulating past it. It does not reproduce the
logged run's actual flight path (forward progress, lateral drift) --
that requires Step CO's closed-loop sim -- but it is a real, honest,
self-consistent test of the one thing Step CN asks for: whether the
commanded yaw rate converges when it is the thing driving yaw.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from obst_avoidance.control import ControlState, SectorController, SectorControllerConfig
from obst_avoidance.perception import geometry
from obst_avoidance.perception.types import SectorBelief

# Same camera intrinsics used throughout this project's sim sessions
# (geometry.py's own docstring, Step BG) -- these frames.jsonl logs did
# not store per-frame intrinsics, so they're hardcoded here rather than
# guessed from image size alone.
CAMERA_WIDTH = 640
CAMERA_FX = 205.4696
CAMERA_CX = 320.0
N_SECTORS = 11


def load_frames(path: Path):
    frames = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    frames.sort(key=lambda r: r["seq"])
    return frames


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def replay(path: Path):
    frames = load_frames(path)
    bearings = tuple(geometry.sector_bearings_rad(CAMERA_WIDTH, CAMERA_FX, CAMERA_CX, N_SECTORS))
    cfg = SectorControllerConfig(sector_bearings_rad=bearings, n_sectors=N_SECTORS)
    ctrl = SectorController(cfg)

    state = ControlState(target_world_bearing=None, seconds_on_target=0.0, last_source=None, last_t=None)
    yaw = 0.0  # self-integrated from THIS controller's own yaw_rate output, see module docstring
    prev_t = None
    rows = []
    for i, row in enumerate(frames):
        if prev_t is not None:
            dt = row["t_capture"] - prev_t
            yaw = wrap_to_pi(yaw + rows[-1]["new_yaw_rate"] * dt)
        prev_t = row["t_capture"]
        goal_heading = wrap_to_pi(-yaw)  # _goal_heading_body(yaw), GOAL_WORLD_BEARING_RAD=0.0 (Step CF)

        belief = SectorBelief(
            scores=np.array(row["scores"], dtype=np.float64),
            valid=np.array(row["valid"], dtype=bool),
            confidence=row["confidence"],
            source=row["source"],
            latency_ms=row["perception_latency_ms"],
        )
        cmd, state = ctrl.step(belief, goal_heading, yaw, row["t_capture"], state)
        rows.append({
            "frame": i,
            "seq": row["seq"],
            "t_capture": row["t_capture"],
            "yaw_deg": math.degrees(yaw),
            "old_logged_yaw_rate": row["cmd"]["yaw_rate"],
            "old_logged_target_sector": row["cmd"]["target_sector"],
            "new_yaw_rate": cmd.yaw_rate,
            "new_target_sector": cmd.target_sector,
            "new_target_world_bearing_deg": math.degrees(state.target_world_bearing) if state.target_world_bearing is not None else None,
            "mode": cmd.mode,
        })
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", type=Path)
    args = parser.parse_args()

    for path in args.logs:
        print(f"\n=== {path.name} ===")
        rows = replay(path)
        header = f"{'frame':>5} {'t':>7} {'yaw(deg)':>9} {'OLD yaw_rate':>13} {'OLD sector':>10}   {'NEW yaw_rate':>13} {'NEW sector':>10} {'NEW target world bearing(deg)':>30}"
        print(header)
        for r in rows:
            bearing = r["new_target_world_bearing_deg"]
            bearing_str = "" if bearing is None else f"{bearing:.1f}"
            print(f"{r['frame']:>5} {r['t_capture']:>7.2f} {r['yaw_deg']:>9.1f} "
                  f"{r['old_logged_yaw_rate']:>13.4f} {str(r['old_logged_target_sector']):>10} | "
                  f"{r['new_yaw_rate']:>13.4f} {str(r['new_target_sector']):>10} "
                  f"{bearing_str:>30}")

        old_rates = [r["old_logged_yaw_rate"] for r in rows]
        new_rates = [r["new_yaw_rate"] for r in rows]
        new_yaws = [r["yaw_deg"] for r in rows]
        old_unique = len(set(round(v, 6) for v in old_rates))
        print(f"\nOLD (Step CL) trace: {old_unique} distinct yaw_rate value(s) across {len(old_rates)} frames "
              f"({'CONSTANT -- open-loop, never converges' if old_unique == 1 else 'varies'})")
        print(f"NEW (Step CM) trace: max|yaw_rate|={max(abs(v) for v in new_rates):.4f}, "
              f"final yaw_rate={new_rates[-1]:.4f}, self-integrated yaw settled at {new_yaws[-1]:.1f} deg "
              f"-- {'SETTLED, did not accumulate' if abs(new_yaws[-1]) < 30 else 'DID NOT SETTLE'}")


if __name__ == "__main__":
    main()
