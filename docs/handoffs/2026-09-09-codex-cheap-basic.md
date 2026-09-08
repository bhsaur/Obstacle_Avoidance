# CHEAP-BASIC — Codex implementation and simulation evidence

Baseline: package Git `2d2a8bd`. User priority: CheapStage alone must dodge one
large visible obstacle. No depth was used. Claude peer review is pending.

## Mechanism

The old LK-only belief was often invalid on a plain obstacle. Lowering its
validity threshold would not create missing visual evidence. The opt-in
`cheap_visible` variant therefore adds a separate `SectorBelief.obstacle_spans`
detection cue, while keeping all LK validity/NaN, scores, and 168 features.
No contour TTC enters the belief. `None` means cue unavailable; empty tuple
means no detected silhouette, NOT certified free space.

`perception/visible.py` balances LK corner quotas across sectors and vertical
tiles, checks forward/backward track consistency, and retains coherent fast
foreground tracks. Large closed contours and long upright edges provide
angular occupancy. Upright edges support shapes clipped by the frame/rotor;
one edge conservatively extends toward the nearer image edge. No color class,
world coordinates, neural model or depth drives detection.

The shared SectorController dispatches an explicitly opt-in visual policy.
It expands silhouette bounds by 0.55 rad, chooses a boundary near the route
heading with a continuity cost, limits yaw to 0.6 rad/s and slew to 1 rad/s²,
and slows translation with heading error (maximum 0.5 m/s). It turns in place
when the route goal is outside its steering window. Detected angular bounds
persist in world orientation for up to 2 capture-time seconds through missing
detections. No translation compensation or metric range is claimed.

The isolated world contains one plain gray 2 x 3 x 5 m box centered at (17,0).
Routes run from (10,y) to (25,y), altitude command 3 m, endpoint tolerance
0.75 m, modeled collision radius 0.35 m. Obstacle geometry is supplied only
to the evaluator and recording metadata, never to perception/controller.
The trajectory plotter now honors recorded geometry; mixed maps need --grid.

See [launch guide](../BASIC_AVOIDANCE.md). Default baseline and other worlds
remain available. This variant is a new slower detection-driven experiment,
not an interchangeable historical flow-only/gating arm.

## Rejected hypotheses and preserved attempts

- Ideal angular-projection test with 0.35 rad padding cut the centered box
  corner (minimum modeled margin -0.054 m). Padding 0.55 passed the centered
  and ±0.7 m ideal starts before flight. This tuning was not a safety proof.
- First live center run, `eval_results/cheap_basic_center_jr85nV/run`, collided
  near (15.683,-0.598). Closed contours missed clipped/rotor-connected outlines;
  immediate route recovery turned back toward the obstacle. Preserve this
  failed attempt. Only 69/187 control frames detected an outline. Fixed with
  upright edges and short world-orientation memory; see source hashes per run.
- `eval_results/cheap_basic_offsets_v2_4q1AzA/offset_-0.7` failed before takeoff:
  GUIDED telemetry confirmation timed out. Result records disarmed/LAND,
  cleanup errors empty. This is a startup failure, not an avoidance trial.

## Validation

Offline command from the package:

```bash
PYTHONDONTWRITEBYTECODE=1 MPLCONFIGDIR=/tmp/obst-mpl python3 -m pytest -q -p no:cacheprovider \
 test/test_visible_obstacle.py test/test_controller_recovery.py \
 test/test_controller_v2.py test/test_reference_path.py test/test_sector_controller.py \
 test/test_orchestrator.py test/test_score_normalization.py test/test_vehicle_lifecycle.py \
 test/test_owned_processes.py test/test_cheap_stage.py test/test_live_viewer.py \
 test/test_sim_startup.py test/test_trajectory_review.py test/test_physical_belief.py
```

136 passed, 18 Matplotlib/pyparsing deprecation warnings. One additional
custom-scene collision/metadata integration test was then added; rerunning
`test/test_orchestrator.py` passed 26 tests, bringing distinct passing tests
to 137. Tests include plain
and clipped silhouettes, horizon rejection, spatial LK translation, missing
outline memory, ideal box avoidance at three starts, and custom-map clearance.
`git diff --check` passes. Package was built successfully with colcon during
implementation. Live ROS/log-source and ament lint suites were not run.

Centered corrected run: `eval_results/cheap_basic_center_v2_wgShJe/run`.
Endpoint reached, collision false, 448 control frames, 48.147 capture-time
seconds, minimum sampled airframe margin 0.249 m. Landed/disarmed.
`comparison.png` preserves failed/successful trajectories; `avoidance.mp4`
contains the annotated autonomous phase (25 fps, faster than real flight).
Raw AVI, frame/video indexes, config, simulator log and source hashes retained.

Offset repeats: `eval_results/cheap_basic_offsets_retry_danjmv/`.
Both used the same frozen runtime source hashes as the corrected center run.

| Start y | Outcome | Min sampled airframe margin | Endpoint distance | Sim seconds |
|---|---|---|---|---|
| 0 | endpoint reached | 0.249 m | 0.671 m | 48.147 |
| -0.7 | endpoint reached | 0.521 m | 0.743 m | 35.607 |
| +0.7 | endpoint reached | 0.537 m | 0.730 m | 35.079 |

All three completed flights were collision-free and landed/disarmed. These
three successes follow the preserved initial collision and one disarmed
startup failure; do not combine them into an unqualified overall 100% rate.
`three_trials.png` and `summary.json` in the offset directory compare them.

Final `colcon build --packages-select obst_avoidance --symlink-install` from
the workspace succeeded (one package). Host inspection after all runs found
no autonomous_demo, Gazebo, ArduCopter, MAVROS, bridge or MAVProxy processes;
UDP14550/14551/9002/9003 and TCP5760 were free. Unrelated applications were
not stopped. Implementation is ready for Claude review.

## Limits and review focus

This only establishes the basic visible-box milestone if the recorded trials
succeed. It does not establish reliability in the old multi-obstacle world,
real-world flight safety, or the learned gate research claim. No-detection
cruise assumes a visible, upright obstacle; low contrast or single-edge side
ambiguity can violate that assumption. Memory is angular and expires; it
cannot replace depth. Steering still has unnecessary reorientation turns.

The prototype perception version string is `cheap_balanced_visible_v1` in
both initial and corrected attempts; source hashes and distinct run directories
identify the actual flown revision. Latency measurement now includes the
silhouette detector. Historical initial-attempt latency omitted that added CV
cost. Target-switch counts mean per-frame world-bearing updates, not physical
side changes. Seeds are recorded as null/unapplied; these are not paired-seed
statistical claims.

Claude should review edge clipping and memory expiration, blindness behavior,
and the evaluator separation before extending the policy to more scenes.
