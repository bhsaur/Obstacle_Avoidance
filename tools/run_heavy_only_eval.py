#!/usr/bin/env python3
"""Drives ONE closed-loop HEAVY-ONLY run (Step AW) against an
already-launched live sim -- HeavyStage driving SectorController instead
of CheapStage, via HeavyStageAdapter (see runtime/orchestrator.py's
docstring for why the adapter exists: HeavyStage.infer(packet) takes one
argument, CheapStage.infer(packet, prev_packet, odom) takes three, and
Orchestrator always calls the three-argument form).

Mirrors tools/run_flow_only_eval.py's retry logic and output schema
(RunResult's field names are shared across both arms, see
orchestrator.py's `perception_latency_ms`). Flight speed comes from
SectorControllerConfig.fwd_vel's own default (0.8 m/s as of Step BK,
changed from 1.5 -- see control/sector.py's docstring and README's Step
BK for the arithmetic) -- ONE shared value, not a per-script override,
so this arm and run_flow_only_eval.py's cheap arm stay a controlled
comparison at whatever speed that default currently is.

HeavyStage's per-frame cost is ~1.5s on this machine (Step AU, isolated;
Step AT's live/contended measurement applies here too) -- roughly two
orders of magnitude slower than CheapStage. A run covering the same
ground distance will take similar WALL-CLOCK time for the flight itself
(governed by RTF * speed, not perception rate, since VehicleInterface's
15Hz republish timer keeps flying on the last command between the much
rarer perception updates -- see platform/vehicle.py), but each
individual perception update costs far more, and --max-wall-time should
be set generously to avoid the run's own safety cap cutting off a flight
that's still making progress just on infrequent updates.

Usage:
  tools/run_heavy_only_eval.py --spawn-x -3.0 --spawn-y 0.0 --altitude 3.0 \\
      --max-wall-time 300 --out result.json [--log-path frames.jsonl]
"""
import argparse
import json
import time

import rclpy

from obst_avoidance.control import SectorController, SectorControllerConfig
from obst_avoidance.frame_source import SimFrameSource
from obst_avoidance.perception import HeavyStage, geometry
from obst_avoidance.platform import MavrosVehicle
from obst_avoidance.runtime import HeavyStageAdapter, Orchestrator, OrchestratorConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--max-wall-time", type=float, default=300.0)
    parser.add_argument("--out", type=str, required=True)
    parser.add_argument("--log-path", type=str, default=None)
    parser.add_argument("--goal-x", type=float, default=None,
                         help="override OrchestratorConfig.goal_x_m -- for zone-segment runs "
                              "(Step AX/BF), pair with the LAUNCH's spawn_x/spawn_y so both "
                              "match the same runtime.ZONE_SEGMENTS entry; default (None) uses "
                              "OrchestratorConfig's own default (end of full corridor)")
    args = parser.parse_args()

    rclpy.init()
    vehicle = MavrosVehicle()
    source = SimFrameSource()

    try:
        source.wait_for_intrinsics()
        intr = source.intrinsics
        print("loading HeavyStage (Depth Anything V2 Small)...")
        heavy = HeavyStage(intr)
        print("HeavyStage loaded")
        adapter = HeavyStageAdapter(heavy)
        # sector_bearings_rad from REAL intrinsics, not the linear-fov fallback --
        # see control/sector.py's SectorControllerConfig docstring and Step BJ.
        bearings = tuple(geometry.sector_bearings_rad(intr["width"], intr["fx"], intr["cx"]))
        # n_sectors MUST match len(bearings) -- SectorControllerConfig's own
        # n_sectors default (5) does NOT auto-track sector_bearings_rad's length
        # (Step CN finding: every closed-loop run before this fix silently
        # restricted candidates to range(cfg.n_sectors)=range(5), i.e. only the
        # LEFT HALF of the 11-sector belief/bearings arrays, regardless of score --
        # confirmed empirically via target_sector never exceeding 4 in any prior
        # logged run). Deriving it from len(bearings) here instead of a bare
        # literal makes this mismatch structurally impossible to reintroduce.
        controller = SectorController(SectorControllerConfig(sector_bearings_rad=bearings, n_sectors=len(bearings)))

        max_arm_attempts = 4
        for attempt in range(1, max_arm_attempts + 1):
            try:
                vehicle.guided_arm_takeoff(args.altitude)
                break
            except RuntimeError as e:
                if attempt == max_arm_attempts:
                    raise
                wait_s = 10.0 * attempt
                print(f"arm/takeoff attempt {attempt}/{max_arm_attempts} failed ({e}); "
                      f"waiting {wait_s:.0f}s and retrying")
                time.sleep(wait_s)

        cfg_kwargs = dict(max_wall_time_s=args.max_wall_time, log_path=args.log_path)
        if args.goal_x is not None:
            cfg_kwargs["goal_x_m"] = args.goal_x
        cfg = OrchestratorConfig(**cfg_kwargs)
        orch = Orchestrator(source, adapter, controller, vehicle, cfg)
        result = orch.run()

        summary = {
            "collided": result.collided,
            "collision_zone": result.collision_zone,
            "collision_xy": result.collision_xy,
            "completed": result.completed,
            "n_frames": result.n_frames,
            "duration_s": result.duration_s,
            "mode_counts": result.mode_counts,
            "n_target_switches": result.n_target_switches,
            "perception_latency_ms": result.perception_latency_ms,
            "controller_latency_ms": result.controller_latency_ms,
        }
        with open(args.out, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"RESULT: collided={result.collided} zone={result.collision_zone} "
              f"completed={result.completed} frames={result.n_frames} "
              f"duration={result.duration_s:.1f}s modes={result.mode_counts} "
              f"switches={result.n_target_switches}")
    finally:
        vehicle.close()
        source.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
