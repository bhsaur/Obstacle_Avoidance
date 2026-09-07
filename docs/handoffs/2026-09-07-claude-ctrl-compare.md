# Handoff: controller comparison + debugging upgrade (Claude, CTRL-COMPARE)

Builds on Codex's 2026-09-07 trajectory-review handoff. Baseline controller
PRESERVED and unchanged; the variant adds a selectable goal-heading source
only. Edit-lock workflow followed (claimed CTRL-COMPARE; before-copies in
.collaboration/backups/claude-ctrl-compare-20260907/).

## Changes
NEW obst_avoidance/control/reference_path.py
  - WorldBearingGoal (baseline: fixed world +x, == _goal_heading_body)
  - ReferencePathGoal (variant: pure-pursuit lookahead on straight A->B route)
  - GoalInfo(goal_heading_body, cross_track_error, lookahead_xy, progress_m, reference)
control/types.py: + ControlTelemetry (candidate costs, best/current target,
  target_world_bearing, heading_error, switch_reason, is_new_commitment,
  min_score_valid); ControlCommand gains telemetry: Optional[...] = None
  (diagnostic-only; never affects fwd_vel/yaw_rate/mode -> determinism intact).
control/sector.py: sets switch_reason per branch and attaches telemetry.
  NO behavioural change (verified: existing sector tests pass unchanged).
  Reasons: blind_no_valid_sectors | adopt_no_prior | adopt_source_switch |
  adopt_target_lost | hold_reinforce | hold_challenge_pending |
  switch_hysteresis_met | hold_no_challenger.
runtime/orchestrator.py:
  - OrchestratorConfig: goal_provider (None=baseline), controller_version,
    velocity_frame, scene_version.
  - goal_heading now comes from the provider (position passed in).
  - measured yaw rate via finite-diff of yaw over t_capture (frame-agnostic).
  - per-frame log EXPANDED: yaw, measured_yaw_rate, commanded_yaw_rate(signed),
    target_world_bearing, heading_error, switch_reason, candidate_costs,
    min_score_valid, cross_track_error, lookahead_xy, obs_age_ms.
  - FIRST log line = {"meta": {...}} (controller_version, velocity_frame,
    scene_version, goal_reference, goal_x_m, collision_radius_m).
  - stop_reason already distinguished collision/goal_reached/user_stop/
    camera_timeout/wall_timeout (unchanged) -> used by the plotter for status.
autonomous_demo.py: --controller {baseline,pathtrack} + --lookahead-m.
  pathtrack builds ReferencePathGoal(A=spawn, B=(goal_x, spawn_y)). Perception,
  speed, sectors, gains IDENTICAL across arms (one variable changed).
tools/plot_trajectories.py (per Codex review):
  - parses meta; skips non-"seq" rows; no run dict from zero positions.
  - result matching: named sibling before generic result.json; bad JSON ->
    status "unknown", never guessed; never coerces missing mode to cruise.
  - status from stop_reason (completed/COLLIDED/timed_out/user_stopped/unknown).
  - timeseries v2: SIGNED commanded + measured yaw rate vs t_capture
    (steps-post), heading-error panel, clearance-minus-radius + cross-track.
  - draws reference route (A->B) + goal-x line; shows controller_version.
test/test_reference_path.py: NEW, 11 tests (heading/frame signs, cross-track
  recovery from EITHER side, symmetry, diagonal route, yaw composition,
  missing-odom fallback, lookahead clamp).
test/test_orchestrator.py: updated log-fields test for the meta line + new fields.

## Tests (all from src/obst_avoidance, PYTHONDONTWRITEBYTECODE=1)
  pytest test/test_reference_path.py test/test_sector_controller.py
         test/test_orchestrator.py test/test_score_normalization.py  -> 50 passed
  pytest test/test_collab.py                                          -> 14 passed
  (test_log_frame_source_identity.py NOT run: needs live sim/ROS logging.)

## Matched Gazebo trial (zone_A, spawn_y=-3.8, headless, identical scene)
Runner: tools/run_ctrl_compare.sh zone_A -3.8
Run dirs: eval_results/cmp_baseline_zone_A , eval_results/cmp_pathtrack_zone_A
Plots:    eval_results/cmp_overlay.png , eval_results/cmp_timing.png

| arm       | status    | path_len | dur_s | min_margin | cte_rms | cte_max | steering_reversals |
|-----------|-----------|----------|-------|------------|---------|---------|--------------------|
| baseline  | completed | 22.14 m  | 28.4  | +0.29 m    | n/a     | n/a     | 31                 |
| pathtrack | completed | 21.98 m  | 28.2  | +0.64 m    | 5.05 m  | 6.87 m  | 20                 |

Both reach the goal. Path-tracking keeps ~2x the clearance margin and ~35%
fewer steering reversals (smoother), at similar length/duration -- a real,
if single-seed, improvement. min_margin = centre-to-surface distance minus
the 0.35 m airframe radius (evaluation only; NOT metric perception).
No owned sim processes remained after the batch (verified).

## Unresolved / caveats
1. The through-spawn reference route (y=-3.8) runs straight THROUGH treeA1
   (17,-3.8), so pathtrack's cross-track (rms 5.05 m) is dominated by an
   UNAVOIDABLE detour, not controller error. For a fair path-tracking
   comparison pick a reference the scene doesn't block (e.g. y=0 corridor,
   or a spawn whose straight route is clear). The variant + tooling are ready;
   only the reference choice needs revisiting.
2. Single seed / single zone so far. Need a matched multi-seed/zone batch
   (run_ctrl_compare.sh per (zone,spawn_y)) before any general claim.
3. Guardrails kept: AVOID != detection and normalized scores != metric
   clearance are NOT presented as such; ground-truth OBSTACLES used ONLY in
   the evaluator (plotter/collision), never in the goal provider/controller.
4. Only goal_heading source changed this round (no normalization/gain/speed/
   target-selection changes), per the one-variable rule.
