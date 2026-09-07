#!/usr/bin/env python3
"""Drives ONE closed-loop flow-only run against an already-launched live
sim (see README's Step AS for the launch commands used to produce each
run's sim session). Not itself the orchestrator -- runtime/orchestrator.py
is the reusable library logic; this is the thin executable entry point
that wires real SimFrameSource/MavrosVehicle/CheapStage/SectorController
together and drives one Orchestrator.run() to completion, writing the
result as JSON.

Retries guided_arm_takeoff() once on a transient EKF/GPS-not-settled arm
failure (mavros_msgs CommandBool result=4) -- this project has hit that
exact transient repeatedly across many prior sim sessions; a fixed short
wait + one retry has reliably resolved it every time so far.

Usage:
  tools/run_flow_only_eval.py --spawn-x -3.0 --spawn-y 0.0 --altitude 3.0 \\
      --max-wall-time 220 --out result.json [--log-path frames.jsonl]
"""
import argparse
import json
import time

import rclpy

from obst_avoidance.control import SectorController, SectorControllerConfig
from obst_avoidance.frame_source import SimFrameSource
from obst_avoidance.perception import CheapStage, geometry
from obst_avoidance.platform import MavrosVehicle
from obst_avoidance.runtime import Orchestrator, OrchestratorConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--max-wall-time", type=float, default=220.0)
    parser.add_argument("--out", type=str, required=True)
    parser.add_argument("--log-path", type=str, default=None)
    parser.add_argument("--goal-x", type=float, default=None,
                         help="override OrchestratorConfig.goal_x_m -- for zone-segment runs "
                              "(Step AX), pair with the LAUNCH's spawn_x/spawn_y so both match "
                              "the same runtime.ZONE_SEGMENTS entry; default (None) uses "
                              "OrchestratorConfig's own default (end of full corridor)")
    args = parser.parse_args()

    rclpy.init()
    vehicle = MavrosVehicle()
    source = SimFrameSource()

    try:
        source.wait_for_intrinsics()  # blocks until camera_info has arrived; raw-form return value
                                       # discarded -- CheapStage needs the DECODED .intrinsics
                                       # property (fx/fy/cx/cy/width/height), not this raw K/D form.
        intr = source.intrinsics
        cheap = CheapStage(intr)
        # sector_bearings_rad from REAL intrinsics, not the linear-fov fallback --
        # see control/sector.py's SectorControllerConfig docstring and Step BJ.
        bearings = tuple(geometry.sector_bearings_rad(intr["width"], intr["fx"], intr["cx"]))
        # n_sectors MUST match len(bearings) -- see run_heavy_only_eval.py's matching
        # comment / Step CN finding: SectorControllerConfig's own n_sectors default
        # (5) does not auto-track sector_bearings_rad's length, and every closed-loop
        # run before this fix silently restricted candidates to the left half of the
        # 11-sector arrays regardless of score.
        controller = SectorController(SectorControllerConfig(sector_bearings_rad=bearings, n_sectors=len(bearings)))

        max_arm_attempts = 4
        for attempt in range(1, max_arm_attempts + 1):
            try:
                vehicle.guided_arm_takeoff(args.altitude)
                break
            except RuntimeError as e:
                if attempt == max_arm_attempts:
                    raise
                wait_s = 10.0 * attempt  # 10s, 20s, 30s -- give the EKF more room each retry
                print(f"arm/takeoff attempt {attempt}/{max_arm_attempts} failed ({e}); "
                      f"waiting {wait_s:.0f}s and retrying")
                time.sleep(wait_s)

        cfg_kwargs = dict(max_wall_time_s=args.max_wall_time, log_path=args.log_path)
        if args.goal_x is not None:
            cfg_kwargs["goal_x_m"] = args.goal_x
        cfg = OrchestratorConfig(**cfg_kwargs)
        orch = Orchestrator(source, cheap, controller, vehicle, cfg)
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
