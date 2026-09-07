# Trajectory plotter review and controller priorities

Author: Codex, 2026-09-07. Reviewed Claude's VIZ-TRAJ and flight-takeover
handoffs, plot_trajectories.py, controller, normalization, orchestrator logs,
the saved trajectory/timing figures, and six actual run logs. No controller
or plotter code changed, and no flight was launched during this review.

## Verdict and reproduced evidence

The plotter is useful for visual diagnosis and comparing path shape. Its
mode shading and command plots are not sufficient to establish causality.

Recomputed using the tool's load_run/clearance_series functions:

- takeover_demo_4: 242 logged control frames, 22.968 simulation seconds;
  position (10.0004,-3.8058) to (24.2832,6.1886); 17.794m path length;
  minimum centre-to-obstacle-surface distance 0.6595m, giving only 0.3095m
  margin after the evaluator's assumed 0.35m airframe radius. End reason
  user_stop; goal x30 not reached. Logged control phase, not later landing
  position, is the trajectory being plotted. All these rows have valid odom.
- The same run: cruise 9, avoid 173, blind 60 frames; commanded yaw sign
  changed 28 times after excluding magnitudes <=0.05 rad/s. This is a
  diagnostic command-reversal count, not measured physical oscillation.
- heavy_cr_run0..4: first logged |yaw_rate|>0.05 at surface distance
  1.9907, 2.1518, 2.3002, 1.3380, 2.7699m, respectively. Each log has just
  14 control samples. These historical runs precede the velocity-frame fix;
  they cannot establish how the current actuation would have avoided.

## Findings

1. **AVOID does not mean an avoidance action engaged.** sector.py computes
   target and yaw command before assigning mode. Both cruise/avoid use the
   same speed and steering law. Nonuniform valid scores are normalized per
   frame to a minimum of zero; min_score_valid<0.1 therefore labels these
   frames AVOID regardless of absolute danger. All 126 multi-valid frames
   in takeover_demo_4 had minimum score zero. Claude's late-command finding
   is real; its interpretation as evidence for simply increasing gain is
   not established. Gain times a zero heading error still gives zero yaw.
2. **Timing view discards direction and uses progress as time.** The tool
   loads t_capture but plots abs(yaw_rate) against x, with interpolated
   lines. This hides left/right reversals and misrepresents time when
   movement stops or reverses. Plot signed commands with step holds against
   capture time, alongside actual yaw/yaw rate and velocity direction.
3. **Geometry is approximate and sampled.** Current global OBSTACLES is
   imported for every run; it is not the recorded scene. Box rotations and
   altitude are ignored. Distances are from vehicle centre to obstacle
   surface, not airframe clearance. Sparse samples can miss closer passes
   between points. Rectangle inflation corners are drawn square, unlike
   the Euclidean-radius check. Preserve scene/config versions and label
   sampled geometry estimates; add swept-path checks/contact evidence.
4. **Comparison metadata and failures need attention.** Old/new frame
   conventions must be separated. Plotter loads config but doesn't show
   version/goal; the advertised goal line is not drawn. Missing modes
   silently become cruise; bad result JSON is silently ignored; rows with
   no usable positions still produce a run dict and can crash bounds().
   For *_frames.jsonl, generic sibling result.json takes precedence over
   the matching named result if both exist, potentially attaching the wrong
   verdict. None of those should silently become a controller result.

## Proposed order of work

1. Extend logs/plots with actual attitude and yaw rate, measured velocity,
   target world bearing/error, all sector scores/validity, candidate costs,
   switch reason, capture-to-command age, and per-run controller/scene/frame
   versions. Add completion, clearance margin, path length and path error.
2. Establish a current-controller baseline on matched starting conditions.
   Separate low-level velocity/yaw tracking, controller tests with synthetic
   known beliefs, and actual-perception closed-loop flights. An oracle
   obstacle map may be used only in a separately labelled diagnostic arm.
3. Replace the fixed world +x goal bearing with tracking of an explicit
   reference path/lookahead point. Pick the intended reference explicitly
   (e.g. y=0 corridor or original y=-3.8 route); both imply different errors.
   Current code has no cross-track recovery objective, so a sideways detour
   is not penalized by distance back to a route.
4. Separate absolute urgency/uncertainty from relative sector preference.
   Current min-max scores cannot supply metric clearance or calibrated TTC.
   Define shared confidence/validity and urgency semantics before adding
   geometry-dependent swept-arc safety checks. Keep unknown separate from
   known free space. Then consider contiguous opening selection, reachable
   short steering arcs, and consistent challenger identity in hysteresis.
5. Add the same stop/slow policy for stale/unknown observations and tight
   turns to every experimental arm; version this change because current
   constant-speed results cease to be directly comparable. A stopped robot
   is not a completed mission; evaluate progress and completion as well.

Dynamic feasibility and curvature-aware tracking are established design
ideas, not a reason to drop a ground-robot controller unchanged into this
drone. Primary references consulted:
- https://publications.ri.cmu.edu/the-dynamic-window-approach-to-collision-avoidance
- https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/

Immediate next change recommended: diagnostic plot/log improvements plus
reference-path recovery, with baseline preserved. Tune yaw gain only after
target selection and measured command tracking can be distinguished.
