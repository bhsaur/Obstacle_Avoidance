# Controller source review: reproduced behavior failures

Codex, 2026-09-07. User requested analysis of where the controller lags
and how to improve it. No runtime/controller source changed; no flight run.
Read sector.py/types.py, orchestrator, vehicle publication, belief contract,
existing tests, Claude's handoffs and current recorded flight results.

## Reproductions against the actual controller

Configuration: default weights/margins/speed, 11 sectors with bearings from
width=640, fx=205.4696, cx=320. Fresh state unless specified. Cheap source,
zero inference latency, valid mask all true unless specified. Invoke step
directly with deterministic beliefs; these are controller checks, not flight
predictions.

1. All scores 0, goal/yaw 0 -> forward=0.8, yaw_rate=0, mode=avoid,
   target=5. No unacceptable-candidate rejection or stop fallback exists.
2. All sectors invalid/NaN -> forward=0.8, yaw_rate=0, mode=blind. This is
   the documented baseline speed policy, not an accidental implementation.
3. All scores 1, confidence=0 -> forward=0.8, yaw_rate=0, mode=cruise.
   Confidence does not influence this controller at all.
4. Clear scores all 1; held world bearing=0.3 rad, current yaw=0.3 rad,
   goal_heading=-0.3 (request world +x), last_t=0. Calls at t=.1,1,10
   all retain world bearing=.3 and yaw_rate=0. The neighboring sector
   bearing is -.27594; its goal-cost improvement is .13797, below the
   .15 switching margin. Waiting longer cannot overcome a failed margin.
   At commanded .8 m/s that heading implies about .236 m/s lateral motion
   if perfectly tracked. This is distinct from missing cross-track recovery.
5. Held world bearing=2 rad, yaw=0, only leftmost sector valid and scores=1
   -> target_sector=0, held world bearing=2, yaw_rate=.8. A target outside
   the camera is aliased to a visible edge sector by nearest-centre lookup;
   its actual direction has no observation supporting it.
6. Held world bearing=0; scores .1 except winner=1; winning indices 4,6,4
   at t=.04,.08,.12 accumulate one challenge and commit index4. No one
   challenger has remained preferred for the configured .12s. Missing
   challenger identity defeats intended persistence under alternating noise.

## Where the delay originates

takeover_demo_4 result.json: controller latency median .106 ms, p95 .136 ms;
perception/orchestrator measurement median 80.50 ms, p95 122.95 ms. Logged
capture intervals median .099s, maximum .165s; visible valid sectors median
2 of 11 (range 0..4). Algorithm compute time is not the observed steering
delay. Target selection, commitment rules, sparse/uncertain observations,
and unmeasured command-to-response latency need separating.

The margin is a permanent selection barrier, not merely a dwell timer.
Increasing k_yaw does nothing while target heading error is zero. The .12s
debounce also credits the interval preceding the first newly winning sample;
at sparse update rates one sample can immediately satisfy it. This is an
explicit baseline tradeoff, not evidence that a challenger persisted unseen.

## Ranked improvements

1. Separate direction ranking from safety admission. A valid observation is
   not necessarily traversable. Add an explicit all-blocked/unknown fallback,
   shared across cheap/heavy arms. Current per-frame normalized scores do
   not retain absolute urgency, so define/version the belief semantics before
   interpreting thresholds as TTC or clearance. Keep speed-policy changes
   separate from steering-only comparisons.
2. Separate stable obstacle-side commitment from continuous goal tracking.
   Keep world-referenced commitments, but allow continuous goal correction
   within a selected traversable opening. Use an explicit path/lookahead
   target and cross-track recovery. Do not just remove all hysteresis.
3. Reject held targets outside actual camera angular support; do not let
   edge-sector validity authorize an unseen world bearing. Track challenger
   identity and first-observed time; reset on challenger changes and time
   discontinuities. Define source-switch and stale-observation policies.
4. Check an opening/trajectory against vehicle width and achievable motion,
   rather than selecting isolated valid sectors solely on scalar cost.
   Metric swept-path checks need an appropriate shared geometric input;
   normalized rankings alone cannot provide them. Do not feed ground-truth
   obstacles into the deployed controller as an evaluation shortcut.
5. Preserve the 15Hz republisher but expire stale commands independently of
   perception. Current adapter has no command-age deadline, and orchestrator
   frame timeout is optional. A watchdog tied only to the main perception
   loop cannot intervene if that loop itself stalls.

The mode label is assigned after steering and does not switch its law.
Minimum valid normalized score is zero for every nonuniform belief, so AVOID
mode prevalence is not an obstacle-detection success metric. See the earlier
trajectory review for numeric evidence and the pre-frame-fix comparison caveat.

## Validation and next work

Existing command:
`PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider
test/test_sector_controller.py test/test_orchestrator.py
test/test_score_normalization.py` -> 39 passed. Several problematic defaults
are explicitly asserted by those tests. Add behavior-level regressions for
the cases above when implementing a versioned candidate controller.

Next controlled comparison: corrected baseline versus path/heading-recovery
variant, fixed perception/speed/scene/start poses. Separately introduce and
evaluate common safety admission/speed policy. Log target/cost/commit reasons,
actual yaw and velocity, and capture-to-command age before tuning yaw gain.
