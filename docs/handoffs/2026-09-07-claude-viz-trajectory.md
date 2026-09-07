# Handoff: trajectory + reaction-timing visualiser (Claude, VIZ-TRAJ)

Added tools/plot_trajectories.py — an offline instrument for judging the
controller across sims from logged frames.jsonl (+ result.json). Three views:
- default/--overlay: top-down x-y paths over the obstacle map, one colour
  per run, start/end/collision markers, per-run min clearance.
- --grid: small-multiples, path coloured by controller MODE (cruise/avoid/blind).
- --timeseries: per run, clearance-to-nearest-obstacle AND |commanded yaw_rate|
  vs. corridor x, mode-shaded -> reaction-timing view.

Key finding it surfaces (heavy_cr zone_B batch, all 5 collided): the
controller is in AVOID mode from early, but commands ~0 yaw_rate until
clearance is already ~1.3-2.8 m, then spikes -> min clearance 0.11-0.23 m ->
collision. The failure is LATE STEERING ENGAGEMENT, not mode detection.
Combined with the earlier finding that Codex's body->world frame fix now makes
turns actually redirect the path (~7-10 m lateral in zone_A), the next
controller lever is engaging yaw earlier (larger effective gain / earlier
commit), and revisiting the Step CO/CR collision conclusions under the frame fix.

Scope: only tools/plot_trajectories.py (analysis tool). No controller,
perception, or shared-contract changes -> offline smoke suite unaffected;
not re-run for this doc-/tool-only change. Outputs saved under eval_results/
(traj_takeover.png, traj_heavy_cr_overlay.png, traj_heavy_cr_timing.png).
