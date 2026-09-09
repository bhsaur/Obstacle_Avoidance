# Handoff: HeavyStage + latest-controller test (Claude, HEAVY-CTRL-TEST)

User asked to test the latest controller with HeavyStage perception.
Lock recovered from Codex (CHEAP-MULTI) with explicit user authorization
after confirming Codex idle (no workers/sim ~30 min); before-copies in
.collaboration/backups/claude-heavy-test-20260909/. Lock released after.

## Changes to autonomous_demo.py
1. NEW `--perception {cheap,heavy}` (default cheap). heavy builds
   HeavyStageAdapter(HeavyStage(intrinsics)). Guard: cheap_visible + heavy
   is rejected (needs CheapStage obstacle_spans). show() now tolerates an
   empty cheap-feature vector (heavy) -- annotates the raw frame with the
   belief summary instead of the LK sector panel; stage.reset()/last_tracks
   accessed via getattr so the adapter (no such attrs) works.
2. BUG FIX (pre-existing, Codex code, not mine): world_file was
   Path(__file__).resolve().parents[1]/'worlds'/... which points into
   site-packages/worlds/ when run from the INSTALLED copy (colcon makes a
   REAL copy here), so `config['world_sha256']=sha256(world_file.read_bytes())`
   crashed EVERY installed-package run (cheap too) with FileNotFoundError.
   Now resolves via ament share dir fallback and tolerates absence
   (world_sha256='unavailable'). Codex should confirm this matches intent.

## Test results (zone_A, spawn_y=-3.8, headless, CPU, no GPU)
- HeavyStage load OK; depth inference ~4-5 s/frame live on this CPU
  (matches heavy_validation's 3.7-5.5 s and the offline probe's ~1.2 s warm).
- pathtrack_v5 + heavy -> RESULT: perception_timeout, frames=0. v5's
  bounded perception-failure watchdog (unsupported_timeout_s=5) treats
  HeavyStage's normal ~5 s inference as "perception unavailable" and aborts
  BEFORE the first depth belief arrives. This is the same "constant assumes
  the perception rate" class of issue; v5's watchdog needs to be
  perception-latency-aware (or scaled by expected infer time) to run on heavy.
- pathtrack_v3 + heavy (no watchdog) -> started flying but user stopped it:
  too slow to be worth watching (each control update ~5 s apart).

## Conclusion
HeavyStage closed-loop is CPU-latency-BLOCKED on this machine (~5 s/frame,
~0.2 Hz). This is the documented heavy-arm wall; it needs the GPU/Jetson
(30-50 ms) to be a real closed-loop arm. The CAPABILITY is separately
confirmed offline: scratchpad/cheap_vs_heavy.py shows heavy produces
11/11 valid sectors and flags danger on the untextured wall where cheap is
fully blind (centre never valid) -- so the two-stage/gate premise holds;
only the closed-loop cost is the blocker.

## Tests / cleanliness
- 69 offline tests pass (test_reference_path/test_sector_controller/
  test_orchestrator). autonomous_demo has no unit test; verified import +
  --help + the world-file fix (share path resolves).
- No owned sim processes/ports remain.
