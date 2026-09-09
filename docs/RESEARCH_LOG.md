# obst_avoidance

**Start here for current work:** [Project status](docs/PROJECT_STATUS.md),
[shared Codex/Claude workflow](docs/COLLABORATION.md), and
[task board](docs/TASKS.md). The material below is a chronological research
log; later steps supersede earlier status statements. The supplied
[complete reference](docs/reference/project_complete_reference.md) is archived
unchanged, with corrections recorded in the current status document.

### Camera with CheapStage sector scores

```bash
cd ~/ardu_ws
colcon build --packages-select obst_avoidance --symlink-install
source install/setup.bash
ros2 launch obst_avoidance cheap_viewer.launch.py
```

This opens the live camera monitor and starts the existing zone simulator.
Add `use_gui:=true` to also open Gazebo's 3D view, or `start_sim:=false` to
attach to an existing simulation. The monitor does not arm or command flight.
Close its window (or press Q/Esc) to request shutdown of this launch.
Avoid a second recorder/viewer using MAVLink port 14551 at the same time.

The workspace now includes a MAVROS 2.14.0 duplicate-ACK crash backport.
Source `install/setup.bash` so the patched workspace package is selected.
For a fresh rebuild, follow [MAVROS backport build instructions](../mavros/BACKPORT.md).
The [crash diagnosis](docs/handoffs/2026-09-06-codex-mavros-crash.md)
records the core-dump evidence and validation.

The bottom panel shows each of the 11 sectors' **relative score**, LK TTC in
seconds, and tracked-point count. `? / UNKNOWN` means insufficient signal.
Scores are rankings within the current frame, not probabilities of safety;
optical flow needs motion and trackable texture.

Replay a moving camera recording without running ROS or the simulator:

```bash
cd ~/ardu_ws/src/obst_avoidance
python3 -m obst_avoidance.live_viewer --recording ~/ardu_ws/recordings/textured_approach_treeA2
```

Use `--headless --max-frames 220 --output /tmp/cheap-view.avi` to export a
finite preview (choose a new output filename). See
[the CheapStage review](docs/handoffs/2026-09-06-codex-cheap-review.md) for the
fixes, checks, and remaining limitations.

Monocular obstacle-avoidance groundwork for the ArduPilot/Gazebo drone
stack. The flow-only baseline arm is now complete and flying
closed-loop: **FrameSource** (time-synchronized image/gyro/odometry
capture) -> **CheapStage** (LK-flow per-sector time-to-contact) ->
**SectorController** (VFH) -> **MavrosVehicle**, orchestrated end-to-end
with collision detection and per-frame logging. First real
closed-loop numbers exist (see "Step AS" below): 10/10 runs collide,
10/10 at Zone B (the untextured wall CheapStage structurally cannot
see) — the expected, correct result and the compute floor the eventual
gate is measured against. **HeavyStage and the learning-to-defer gate
don't exist yet** — that's the next work, plus two open items flagged
below (the tau estimator's sector-selection gap, and live latency
running ~7x higher than the isolated benchmark).

## What's here

```
worlds/env_forest.sdf, env_zones.sdf   Obstacle-course worlds (env_zones replaces env_forest, see Step X)
models/gimbal_small_3d_640x480/        Local override: fixes upstream's 640x4800 camera
models/iris_with_gimbal_640x480/       Local override: gimbal locked rigid at (0,0,0)
launch/env_forest.launch.py, env_zones.launch.py   Gazebo + ArduPilot SITL + MAVROS, one command each
obst_avoidance/frame_source/
    packet.py                      FramePacket(image, gyro, t_capture, seq)
    base.py                        Abstract FrameSource (read(), dropped_frames)
    sim.py                         SimFrameSource -- the only concrete live implementation
    log.py                         LogFrameSource -- replays a recording, same interface
obst_avoidance/record_pass.py      Scripted arm/takeoff/forward-fly/yaw/wiggle + record to disk
obst_avoidance/perception/
    types.py                       SectorBelief -- shared output type, cheap AND heavy stages
    geometry.py                    Sectoring, FOE, miss distance, cone weight -- shared, not per-stage
    cheap.py                       CheapStage; also the canonical track_pair()/rotational_flow()
    features.py                    54-element feature vector, FEATURE_SPEC as the one source of truth
obst_avoidance/control/
    types.py                       ControlCommand, ControlState -- caller-owned, no wall clock
    sector.py                      SectorController -- VFH (Borenstein & Koren 1991), one impl for both stages
obst_avoidance/platform/
    vehicle.py                     VehicleInterface (Protocol), MavrosVehicle -- 15Hz decoupled setpoint publisher
obst_avoidance/runtime/
    orchestrator.py                Orchestrator -- FrameSource -> CheapStage -> SectorController -> VehicleInterface,
                                    collision/goal detection, per-frame logging
tools/                              flow_explore.py, ttc_sectors.py, verify_cheap_stage.py, diagnose_tracking.py,
                                    run_flow_only_eval.py, check_depth.py, gen_textures.py -- diagnostics/eval driver, not library code
```

## Running it

```bash
# Launch the sim (add use_gui:=false to run headless, which roughly doubles RTF)
ros2 launch obst_avoidance env_forest.launch.py

# Fly a scripted forward pass and record frames
ros2 run obst_avoidance record_pass --out-dir ~/ardu_ws/recordings/run1 \
    --altitude 5.0 --speed 1.0 --duration 15.0
```

Output per recording: `frames/frame_NNNNNN.png` (BGR, correct for viewing --
see gotchas), `packets.jsonl` (one row of `seq, t_capture, gyro, gyro_valid,
frame_file` per frame), `intrinsics.json` (camera K matrix, saved once via
`SimFrameSource.wait_for_intrinsics()`).

## Using FrameSource in code

```python
from obst_avoidance.frame_source import SimFrameSource

source = SimFrameSource()  # rclpy.init() first
packet = source.read()     # None if no new frame since last read()
# packet.image: np.ndarray (rgb8 -- NOT BGR, see gotcha below)
# packet.gyro:  np.ndarray(3,) -- body-frame NED rad/s (see gotcha below)
# packet.gyro_valid: bool -- False if no fresh-enough gyro sample existed;
#                             gyro is still zeros in that case, check this
#                             flag rather than trusting a zero rate
# packet.t_capture: float seconds, sim-time
# packet.seq: int
source.dropped_frames          # frames overwritten before being read()
source.get_intrinsics()        # None until camera_info has arrived
source.wait_for_intrinsics()   # blocks (up to timeout_s) until available
source.close()
```

`RealFrameSource` / `LogFrameSource` can be added later against the same
`FrameSource` interface without touching any caller.

## Key facts and gotchas (learned the hard way)

- **Camera Hz is throttled by real-time factor, not resolution.** The
  sensor is configured for 30 Hz sim-time (raised from an original 10 --
  see below); wall-clock Hz ≈ 30 × RTF. RTF fluctuates with system load
  (observed 0.23–1.25 across sessions). Tested whether reduced resolution
  (320x240 vs 640x480) would help, on the theory it might be a render
  bottleneck -- it made **no measurable difference** (same ~10Hz at
  either resolution, same RTF), ruling that out; reverted to 640x480
  since there's no reason to sacrifice image quality for no benefit.
  Typical achieved rate in this environment: ~20-26 Hz.
- **ATTITUDE (gyro) stream rate needed two separate fixes, empirically
  found, not assumed.** (1) The legacy `SR0_EXTRA1` parameter from most
  ArduPilot tutorials doesn't exist in this build -- it's been renamed
  `MAV1_EXTRA1`, defaulting to 4Hz. `PARAM_REQUEST_READ` for the old name
  just times out silently with no error, which is what made this hard to
  spot -- `param_fetch_all()` (dumps everything) was needed to find the
  real name. (2) Even after requesting a higher rate via both
  `MAV1_EXTRA1` and `MAV_CMD_SET_MESSAGE_INTERVAL`, the achieved rate
  measurably **decayed** from ~28Hz to ~3Hz within 5-10s of startup and
  stayed there -- a one-time request doesn't stick. Most likely cause:
  mavros shares the same underlying FC link (via mavproxy's single master
  connection) and periodically re-asserts its own lower stream-rate
  config; ArduPilot tracks message-interval state per (message, channel),
  not per remote client, so mavros's request silently overwrites ours.
  Fixed by having `SimFrameSource` re-send its rate request every 0.5s
  from the background MAVLink thread instead of once at startup.
- **Gyro is body-frame NED, not ROS's ENU.** A +0.6 rad/s commanded yaw
  rate (mavros/ROS convention) shows up as ≈ -0.6 rad/s in the raw MAVLink
  `ATTITUDE.yawspeed` that `SimFrameSource` reads. Verified, not a bug —
  just don't mix it with ENU-convention topics without converting.
- **The gimbal is rigid, not stabilizing.** Confirmed empirically: joint
  angles didn't move under a real ~59° airframe yaw. It's locked to
  forward/level (0,0,0) by deleting ArduPilot's mount-control channels
  (8/9/10) from the SDF override, so nothing re-drives it. Practical
  upshot: the camera image rotates 1:1 with the airframe, so de-rotation
  only needs one body-frame gyro sample per frame — no gimbal joint angle
  tracking required.
- **`SimFrameSource` must not use port 14550 for its MAVLink connection.**
  That's mavros's port. A second long-lived socket bound there silently
  starves mavros of further packets (looks exactly like an unrelated
  executor-starvation bug — cost real time to track down). Default is
  port 14551, which SITL already relays telemetry to and mavros doesn't
  use.
- **`rclpy.spin_once(node, ...)` without an explicit executor spins a
  shared *global* executor.** Any node ever passed to it stays registered
  there permanently, so unrelated code spinning a different node the same
  way ends up competing with it. Both `SimFrameSource` and
  `record_pass.FlightControl` use their own private
  `SingleThreadedExecutor` to avoid this.
- **`node.get_clock().now()` silently returns exactly 0** until that
  node's `/clock` subscription has processed its first message (with
  `use_sim_time=True`). Reading it before that produces a plausible-looking
  but wrong timestamp. `SimFrameSource` blocks on a valid non-zero reading
  before calibrating its MAVLink-to-sim-time offset.
- **`mode_sent=True` from mavros's `/set_mode` service only means the
  request was sent**, not that the FCU has switched yet. Arming
  immediately after can be rejected if the old mode forbids it (e.g.
  leftover `LAND`). `record_pass` waits for `/state.mode` to actually
  confirm `GUIDED` first.
- **Image↔gyro sync delta, after all the fixes below, lands around
  mean ~30ms / max several hundred ms (rare outliers) over a 30s
  capture**, with `gyro_valid=False` on a small minority (~5-7%) of
  frames at the real 0.05s (50ms) threshold. Started far worse (mean
  120-180ms, ~65% invalid) through two separate wrong designs -- see the
  history in `SimFrameSource`'s docstring. Mean is close to but not
  quite under a 20ms target; the remaining gap is occasional gyro-stream
  hiccups, not a structural problem -- see "What's left" for the further
  fix if that's needed.
- **A single `rclpy.spin_once()` call processes at most one queued
  callback, and only advances the cached sim clock if it happens to
  process a /clock message.** Calling it only from `read()` (an earlier
  version) meant both stale images (if a caller's loop stalled while
  several arrived, `read()` would hand out the *oldest* queued one) and
  a stale clock (biasing the time-offset calibration, see below). Fixed
  by moving ALL spinning to a dedicated daemon thread started in
  `__init__` before anything else -- `read()` no longer spins at all, it
  only reads state the thread keeps fresh continuously. QoS depth 1
  (`KEEP_LAST`) on the image subscription additionally guarantees the
  transport itself never buffers more than the single latest frame.
- **The MAVLink time-offset estimate is a rolling minimum over a ~5s
  window, updated on every ATTITUDE message** (not a one-time startup
  calibration -- that was tried twice and failed twice). Attempt 1:
  average several startup samples -- produced deltas up to ~500ms.
  Attempt 2: minimum instead of average (the standard NTP one-sided-
  latency trick, since `t_sim` is read after `recv_match()` returns and
  transport latency can only push it high, never low) -- still produced
  a calibration ~240ms off from the true value measured moments later.
  Root cause: `get_clock().now()` was stale by however long the
  *previous* `recv_match()` call happened to block, because nothing was
  spinning the node between calibration samples except the calibration
  loop itself -- exactly the same disease as the freshness bug above,
  different symptom. Once the dedicated spin thread (above) keeps the
  clock continuously fresh, a rolling minimum actually filters transport
  latency instead of picking out the most-stale sample.
- **Gyro matching is linear interpolation between the two buffered
  samples bracketing each frame's timestamp**, not nearest-neighbor --
  meaningfully tighter once the gyro rate fixes below landed. Falls back
  to nearest-sample (with the normal staleness check) only when
  `t_capture` isn't bracketed, e.g. right at startup or during a stream
  hiccup.
- **Image color order is pinned to `rgb8` explicitly, not left as
  `"passthrough"`.** This actually caught a real bug: `record_pass.py`
  was feeding that raw array straight into `cv2.imwrite()`, which expects
  BGR -- every saved PNG had red and blue channels swapped (visible as a
  wrong-colored sky) until `record_pass.py` was fixed to convert
  explicitly with `cv2.cvtColor(..., cv2.COLOR_RGB2BGR)` before writing.
  `FramePacket.image` itself stays `rgb8` (unconverted) -- consumers that
  need BGR must convert themselves.

## What's left

- **Perception: both CheapStage and HeavyStage exist now, both flown
  closed-loop, neither is the safety ceiling it needs to be yet.**
  CheapStage's `MIN_POINTS_PER_SECTOR` is set from real data (10, was
  20); its textured-approach spike-at-closest-range failure is fixed
  (percentile estimator, Step AI). Its sector-*selection* gap (Step AE:
  the target-facing sector often supplies no data at all on wall/narrow
  approaches) is still open. HeavyStage (`perception/heavy.py`, Depth
  Anything V2 Small, Step AU) is valid nearly 100% of the time as
  expected, but its per-frame relative normalization has a CONFIRMED
  live failure mode (Step AW: score reads SAFER as a flat wall gets
  closer, 5/5 closed-loop runs collided at the same Zone B CheapStage
  fails at) and costs ~74x CheapStage's latency in the same loop (Step
  AT/AW) -- not the ~3x figure earlier steps had assumed before
  HeavyStage existed to actually measure. Step AV's metric-scale
  question is resolved for now (rank-only, option (c), NOT metric --
  see that step for why, including the real evidence for option (b)
  kept on record for a future phase).
- **Control / avoidance logic — the flow-only AND heavy-only arms are
  both built, wired, and measured closed-loop.** `control/` (VFH
  `SectorController`), `platform/` (`MavrosVehicle`), `runtime/`
  (`Orchestrator`, `HeavyStageAdapter`, `ZONE_SEGMENTS`) all exist
  (Steps AN-AR, AW). Closed-loop numbers now cover the WHOLE corridor,
  not just the approach to Zone B (Step AX): flow-only fails 100% in
  Zone B (blind), Zone C (92.4% avoid-mode but still collides -- too
  dense to react in time), AND Zone D (mixed blind/avoid, collides at
  narrowD1 every time) -- three distinguishable failure mechanisms, not
  one repeated result. Heavy-only fails 100% at Zone B too, for a
  different, now well-understood reason (see above).
- **Gating / decision logic — nothing built yet, and now has a real,
  measured picture to be trained against instead of an assumed one.**
  `SectorBelief.source`/`.latency_ms` exist for exactly this. Building
  it is no longer blocked on missing numbers -- Steps AT-AX produced
  real collision-rate, mode, and latency data for both stages across
  the whole corridor -- but IS blocked on HeavyStage's normalization
  problem: a gate can't usefully learn "defer to heavy near flat
  surfaces" if heavy's own score inverts exactly there. That -- not the
  gate itself -- is the next thing to fix.
- **`RealFrameSource`** (physical camera + real FC) — not implemented.
  `LogFrameSource` (replay from a saved recording, `frame_source/log.py`)
  is implemented: same `read()` contract as `SimFrameSource`, fast or
  realtime-paced replay, `gyro_valid` preserved from the recording
  (never recomputed). Array identity with the live source is verified by
  `test/test_log_frame_source_identity.py` (byte-identical `image`
  arrays after the record→PNG→replay round trip).
- **Tighter image↔gyro sync**, if the current ~30ms mean / occasional
  several-hundred-ms outliers are too coarse for whatever consumes
  `FramePacket` next (target was mean <20ms, max <50ms; not quite there).
  The remaining gap looks like occasional gyro-stream hiccups rather than
  a structural issue -- next things to try: shrink the rolling offset
  window below 5s, or bridge a `sensor_msgs/Imu` topic on the same sim
  clock as images and match via `message_filters` instead of raw MAVLink
  over UDP (removes the mavros rate-contention problem entirely, since
  IMU would ride the same ROS transport as images).
- **Landing/disarm at the end of a recording pass** — `record_pass`
  currently just stops forward velocity; it doesn't land or disarm.

## Pre-CheapStage fixes (Steps U-X, R2, S)

- **Step U -- wx_cam/wz_cam axis ambiguity resolved.** A new `record_pass.py
  --mode roll_pitch_wiggle` (alternating lateral then fore/aft velocity,
  near-zero net translation) induces large roll/pitch rate that yaw_only
  couldn't. Rerunning the 48-candidate sweep (now reusable via
  `flow_explore.py --sweep`) against it: **wx_cam=+pitch beats
  wx_cam=+roll by 66.4% lower median residual** (0.728px vs. 2.168px,
  334 frame pairs) -- decisive, not a tie. The existing mapping
  (wx=pitch, wy=yaw, wz=roll) is confirmed, not just assumed by
  elimination. Details in `flow_explore.py`'s module docstring.
- **Step V -- odometry is now time-aligned to each frame, not "whatever
  arrived last."** `record_pass.FlightControl` buffers `/odometry`
  samples with their header timestamps and interpolates
  (position/velocity linear, attitude NLERP) to each frame's
  `t_capture`, exactly like `SimFrameSource` already does for gyro.
  `odom_valid` is logged per frame, same convention as `gyro_valid`.
  Measured over a dedicated 30s recording: mean sync delta 33.4ms, max
  793ms (one outlier), odom_valid=False on 17/505 frames (3.4%) --
  comparable to gyro's own documented ~5-7% invalid rate, not
  structurally different.
- **Step W -- altitude default corrected to 3.0m** (was 5.0, inconsistent
  with the world/horizon/ground-TTC design). No prior diagnostic
  recordings existed on disk to re-make at the old altitude -- this
  package had produced no persisted recordings before this pass.
- **Step X -- `launch/env_zones.launch.py` added**, defaulting to
  `env_zones.sdf` / `world_name=env_zones` (confirmed to match
  `<world name="env_zones">` in the SDF) instead of silently inheriting
  `env_forest.launch.py`'s stale `env_forest` defaults. Verified live:
  world loads, Iris spawns, `/camera/image` and `/odometry` publish.
- **Step R2 -- ground TTC contamination is real but sector-dependent, and
  smaller than expected.** Per-sector TTC (all tracked points vs.
  above-horizon-only, row<cy=240; see `tools/ttc_sectors.py`) on an
  empty-gap recording: in the three well-sampled sectors (>500 points
  each), all-points vs. above-horizon TTC differ by only 5-15% -- mild
  contamination, not a false alarm from nothing. The two edge sectors
  showed a much bigger swing, but with only 9-16 above-horizon points
  each -- that's a small-sample noise artifact from sky being nearly
  featureless at the frame edges, not contamination; it reinforces the
  existing "prefer centre sectors, not a global min" guidance below
  rather than adding a new finding. Separately, on `wall_approach_1p0`:
  **the centre sector had ZERO trackable points** -- the wall is
  untextured enough that `goodFeaturesToTrack` found nothing there at
  all, at any distance recorded. This is the same "few features,
  something untrackable is there" failure mode already documented below
  for Zone D, now directly confirmed against Zone B's wall.
- **Step S -- flow SNR (mean/std of residual-flow magnitude,
  `tools/`-level ad hoc measurement, not yet a permanent script) across
  wall-approach speeds: 1.0 m/s -> SNR 0.82 (mean 1.59px); 1.5 m/s ->
  SNR 1.38 (mean 2.83px); 2.0 m/s -> SNR 1.32 (mean 2.90px).** Both 1.5
  and 2.0 clear 1.0 by a wide margin; 1.5 edges out 2.0 on this metric.
  Tentative recommendation: **1.5 m/s** as the default approach speed --
  best measured SNR, without 2.0's reduced reaction margin. Caveat:
  this session's real-time factor was unusually low (~0.15-0.6,
  fluctuating) and each approach only covered a few meters of real
  displacement rather than a long straight run -- treat this as
  directional, not a validated final number; worth re-measuring on a
  longer approach when RTF cooperates.

## CheapStage Phase 2 -- built, VERIFY run, one real calibration problem found

`obst_avoidance/perception/` now exists: `types.py` (`SectorBelief`,
shared by CheapStage and the future HeavyStage -- see its docstring for
why the `valid` mask is load-bearing, not optional), `geometry.py`
(sectoring/FOE/miss-distance/cone-weight, shared so the two stages can't
silently diverge in how they reason about trajectory), `cheap.py`
(`CheapStage`, plus `track_pair()`/`rotational_flow()` moved here as the
canonical implementations -- `tools/flow_explore.py` and
`tools/ttc_sectors.py` now import them from here instead of defining
their own copies), `features.py` (54-element feature vector, one
documented `FEATURE_SPEC` as the single source of truth for every
index). `tools/verify_cheap_stage.py` replays existing recordings
through `CheapStage` via `LogFrameSource` -- no sim needed for any of
this.

**yaw_only VERIFY: PASSED.** Zero commanded translation means true TTC
is infinite everywhere -- the sharpest test, since any finite low tau
here means rotational flow is leaking through de-rotation. Result: 0/504
frames showed a spurious low (<3s) tau in any sector; the few sectors
that did register a value stayed high (median 28.3s, min 5.5s, against
a 30s cap). De-rotation holds up under this test.

**wall_approach VERIFY: COULD NOT RUN AS SPECIFIED -- a real
calibration problem, not a bug.** `min_tau` was undefined on 100% of
frames across all three wall_approach recordings (0/241, 0/128, 0/130),
so the "does min_tau decrease with distance" correlation couldn't be
computed at all. Root cause, measured directly (not guessed):
`CheapStage.MIN_POINTS_PER_SECTOR=20` was set from R2's numbers, but
those were AGGREGATED across ~200 frames (e.g. "787 points in sector
0") -- the actual PER-FRAME per-sector count on wall_approach_1p0 never
exceeds 14 (mean 5.3-8.0 depending on sector), because
`goodFeaturesToTrack`'s 300-corner budget is split across 5 sectors and
then thinned further by vignette/LK-error/magnitude rejection. The
threshold is calibrated against the wrong denominator. **Not changed**
-- lowering it now, right after finding it makes the demo look worse,
is exactly the "tune for good-looking TTC" the task explicitly forbids.
Left for whoever calibrates CheapStage next, with the real numbers
above rather than a guess.

**narrow_approach / full-corridor-pass VERIFY: NOT RUN.** No such
recordings exist in this workspace, and per the task's "no sim" VERIFY
constraint, none were created to force these checks to run.

**valid=False rate per sector, per available recording** (5 sectors,
0=far left .. 4=far right):
| recording | 0 | 1 | 2 | 3 | 4 |
|---|---|---|---|---|---|
| yaw_only (odom_sync_check1) | 98.0% | 99.0% | 97.2% | 93.5% | 99.2% |
| empty_gap1 | 100% | 100% | 88.2% | 81.8% | 78.8% |
| roll_pitch_wiggle1 | 100% | 99.7% | 68.5% | 86.4% | 100% |
| wall_approach (all 3 speeds) | 100% | 100% | 100% | 100% | 100% |

No sector is reliably valid across recordings -- which one clears
threshold depends entirely on scene content, reinforcing (again) the
existing "prefer centre sectors, not a global min" guidance is
necessary but not sufficient; the real fix is the threshold above.

## CheapStage Phase 2 -- point-count diagnosis (Steps Y-AB)

`tools/diagnose_tracking.py` (`--check cascade|sweep|estimators|cleaning`)
instruments the tracking pipeline without touching production code --
everything here is a parameterized clone used only for measurement.

**Step Y -- survival cascade.** Ran on all 6 available recordings; the
biggest single-stage drop is the **vignette filter**, not LK-error or
magnitude rejection, on every single recording -- consistently removing
~50% of what LK status left standing (e.g. wall_approach_1p0:
86.09 -> 33.97 points/frame mean, a -52.12 drop, vs. the next stage's
-7.74 and the last stage's -0.14). The other structural finding: on all
three wall_approach recordings, sector 2's count is already ~0 at the
RAW `goodFeaturesToTrack` stage, before any rejection runs -- confirms
(again, independently) that the wall is genuinely textureless, not that
cleaning is too aggressive there.

**Step Z -- detector budget sweep.** 18 combinations (maxCorners x
minDistance x qualityLevel) on wall_approach_1p0, with real
`CheapStage.infer()` latency measured for each (GF_PARAMS temporarily
monkeypatched for the measurement, restored immediately after).
**maxCorners has ZERO effect** -- 300/600/1000 give byte-identical
per-sector counts at fixed minDistance/qualityLevel, because
`minDistance`'s spatial-spacing constraint was already capping the
achievable count well under 300; raising the budget past the actual
bottleneck does nothing. minDistance and qualityLevel ARE the real
levers: loosest tested (minDistance=2, qualityLevel=0.005) nearly
doubles total yield (50.5 vs. 26.9 points/frame at the shipped
300/7/0.01). **Sector 2 stays at exactly 0 across all 18 combinations**
-- no detector setting fixes a lack of texture; this is not a
budget-tuning problem. Latency stayed in the 8.8-15.3ms mean / 12-23ms
max range across every combination -- above the "a few ms" target
mentioned for the paper's compute comparison, but not by an order of
magnitude; report the real numbers rather than the target.

**Step AA -- estimator comparison** (current signed-radial-with-sign-filter
vs. an alternative tau=r/|flow magnitude| with no sign filter), swept
across min_points thresholds since fixing it at 20 would mask the effect
being tested (neither estimator clears 20 on this data at all):
alternative (b) DOES raise valid-rate at every threshold, as
hypothesized (e.g. 74.7% vs. 56.7% at min_points=10) -- but ALSO makes
`correlation(distance, min_tau)` MORE wrong-signed, not less (-0.459 vs.
-0.111 at min_points=5). Both estimators show a NEGATIVE correlation
where a working signal should be positive (tau should fall as the wall
approaches; both show it rising instead) -- a real, unresolved,
concerning finding on its own, not just an estimator-choice issue. Not
adopted -- reported, per the task's scope.

**Step AB -- cleaning-stage revisit**: dropping the unconditional
LK-error top-quartile-rejection stage. Valid-rate improves at every
threshold (e.g. 72.8% vs. 56.7% at min_points=10), AND at the lowest,
most-achievable thresholds it actually FIXES the sign problem Step AA
surfaced: at min_points=1, correlation flips from -0.108 (current,
wrong sign) to +0.098 (correct sign); at min_points=3, -0.220 -> +0.091.
At higher thresholds (5, 10) the two configs are close and mixed (no
longer a clean win). **This is the most promising lead found across all
four steps** -- not adopted in this pass (only `MIN_POINTS_PER_SECTOR`
was in scope), flagged for whoever revisits the cleaning stages next.

**Final step -- MIN_POINTS_PER_SECTOR set from data: 20 -> 10.** The
previous value (20) was based on R2's counts AGGREGATED across ~200
frames, not the actual per-frame yield -- it never cleared on any
wall_approach recording (0% valid-rate everywhere). Measured on the
CURRENT shipped pipeline (with the quartile filter, estimator (a)) on
wall_approach_1p0, the hardest available recording: valid-rate holds at
88.0% down to min_points=5, drops to 56.7% at 10, hits exactly 0.0% at
15. 10 is the largest value that still clears a majority of frames on
the hardest recording, while still requiring double-digit points for a
median to mean something. Confirmed working end-to-end after rebuild:
wall_approach_1p0 sectors 0/4 (higher-yield sectors on this recording)
now go valid 34-38% of the time, sectors 1/2/3 stay at 0% (2 and
structurally so; 1 and 3's ~4-point-per-frame mean on this specific
recording still doesn't clear even 10) -- not a bug, matches the
measured per-sector heterogeneity exactly.

## CheapStage Phase 2 -- wrong-signed correlation diagnosis (Steps AC-AF)

**The Step AB LK-error-filter finding from the previous pass was
correctly rejected and NOT adopted.** A correlation swinging
-0.108 -> +0.098 on a preprocessing choice, on this little data, is
noise, not signal -- selecting the preprocessing that yields the
desired sign would have been the same error the threshold-tuning
discipline was built to avoid. Nothing about track_pair()'s cleaning
stages changed as a result of that finding.

**Step AC -- what does the vignette filter actually remove?** Annotated
frames (kept vs. vignette-rejected points, `tools/diagnose_tracking.py
--check vignette`) on wall_approach_1p0, visually inspected directly:
across all 3 sampled frames, rejected (red) points sit tightly on the
visible rotor-blade/gimbal-housing silhouettes in the frame's top
corners -- genuinely drone-body, not real scene. Loosening the
threshold (y>100 -> y>50 -> disabled) INCREASES valid-rate (56.7% ->
94.0% -> 97.7%) but makes the distance/tau correlation MORE
wrong-signed, not less (-0.076 -> -0.148 -> -0.292) -- consistent with
the rejected points being genuinely bad, not falsely-excluded signal.
VIGNETTE_Y_MIN=100 is doing what it was built to do.

**Step AD -- above-horizon only (the ground-contamination hypothesis):**
does NOT cleanly resolve the sign problem. Sweeping min_points, the
above-horizon-only correlation is positive at 1/3/10 and negative at 5
-- unstable, not a fix -- and at the thresholds where it looks positive,
valid-rate has collapsed to single digits (2.8% at min_points=10, ~6
frames), too small a sample to trust. Applying the same skepticism the
LK-filter finding got: this is not evidence the ground hypothesis is
confirmed.

**Step AE -- is wall_approach validatable at all? NO, structurally.**
Across all three wall_approach recordings, sector 2 (the one pointed at
the wall) supplies min_tau on exactly 0/123, 0/43, and 0/61 frames.
min_tau always comes from sectors 0/4 (the far edges -- peripheral
scenery, not the obstacle ahead). Every correlation number measured
against wall_approach in this and the previous diagnostic pass was
never actually measuring the wall -- it was measuring how peripheral
ground/scenery flow happens to correlate with forward progress. This
reframes the earlier "wrong-signed correlation" finding: the estimator
wasn't necessarily broken for obstacles, wall_approach just never tested
that.

**Step AF -- textured (treeA2) and untextured-narrow (narrowD1)
approaches, recorded to actually test obstacle response** (`ros2 run
obst_avoidance record_pass --mode forward --altitude 3.0 --speed 1.5`,
targets from `worlds/env_zones.sdf`). Both flights ended in a collision
with the target (altitude dropping sharply from ~3.4m to <0.5m right at
the target's x-position in both cases) -- analysis restricted to the
pre-collision segment (odometry z>2.5m) in both.

- **textured_approach_treeA2** (bark-textured trunk, x=20): sector 1
  (not a peripheral edge sector) supplies most min_tau winners (83-91 of
  ~244 frames depending on threshold) -- qualitatively different from
  wall_approach, a real obstacle-adjacent sector is actually
  contributing. Raw per-frame correlation is positive at achievable
  thresholds (0.30-0.39 at min_points=1/3/5) but goes negative at the
  shipped min_points=10 (-0.061) where a single wild outlier (~460s
  tau) dominates an unstable median. **CheapStage's actual production
  output (EMA-smoothed, min_points=10, plotted)**: correlation=0.115 --
  weak positive, and NOT monotonic. Critically, tau **spikes to its
  highest recorded value (19.4s) at the closest distance to the tree,
  moments before collision** -- the worst possible failure mode for a
  safety signal: "all clear" exactly when danger is highest.
  **ACCEPTANCE (min_tau must decrease monotonically with distance):
  FAILS.** Per the task's own stated consequence, this means the
  estimator has a real accuracy problem that no threshold or filter
  setting fixes -- it needs to be fixed at the estimator/smoothing
  level, not tuned around.
- **narrow_approach_D1** (untextured but only 3.5m wide, x=108): unlike
  the wide wall, sector 2 DOES supply min_tau here (32.4% of frames,
  35/108) -- being narrow enough to sit near textured background within
  the same sector matters, even without its own texture. But the
  resulting correlation is consistently NEGATIVE and gets WORSE with
  higher min_points (-0.229 -> -0.169 -> -0.365 -> -0.428 at
  min_points=1/3/5/10) -- having points in the target-facing sector
  doesn't by itself produce a correct signal.

**Net finding across AC-AF: the per-point signed-radial-tau, median-
per-sector estimator (as shipped) does not reliably track ground-truth
distance even when it has real, obstacle-relevant points to work with.**
This is a deeper problem than point-count scarcity (Steps Y-AB) or
sector-selection (this pass) can explain -- it needs to be addressed at
the estimator level before CheapStage's output can be trusted as a
distance signal, textured target or not.

**Final cheap-stage latency** (production settings, unchanged by this
diagnosis -- measured across all 8 available recordings, n=2117 frame
pairs): **mean 8.06ms, median 8.98ms, p95 10.26ms, max 13.65ms.**
Confirms the earlier 8.8-15.3ms estimate on a much larger sample. Still
3-5x the "few ms" assumption in the paper's compute arithmetic --
cheap:heavy is closer to ~1:3 than ~1:10 if the heavy stage lands in the
30-45ms range. Reported as measured; the gating story's strength depends
on where the heavy stage actually lands once it's built.

## CheapStage Phase 2 -- tau estimator fix (Steps AG-AI)

**Step AG ruled out the cheap explanations before touching anything, as
instructed.**
1. SIGN: sector 2's (the trunk's sector) median divergence stays
   POSITIVE in ~90% of frames -- only 22.7% of individual points get
   discarded via the sign check. `cheap.py` DISCARDS negative-divergence
   points entirely (not clamp, not `abs()` -- confirmed by reading the
   code, not assumed). Sign is not the primary bug. (One real pattern
   worth keeping in mind: sector 2's median divergence jumps from
   ~5px/s through most of the approach to 135px/s in the final metre --
   see Step AH for why.)
2. EMA: raw and EMA-smoothed min_tau were plotted directly overlaid.
   They track each other almost everywhere; the one place they diverge
   is the exact frame that later spikes -- RAW already shows a 465s
   outlier there, EMA damps it to "only" 19.4s. **Smoothing is not the
   bug -- it's already helping, just not enough.** The instability is in
   the raw per-frame estimate.
   Critically, that specific 465s/19.4s spike traces to **sector 0**
   (a peripheral edge sector), not sector 2 (the trunk) -- and at the
   very closest frames, sector 2 itself (when it has enough points)
   reports genuinely low, correct tau (0.099-0.524s). The estimator
   works fine when given real obstacle data; the failure is a
   peripheral sector supplying a degenerate single-frame estimate when
   the primary sector's own data is too thin.

**Step AH confirmed the main hypothesis, with a sharper mechanism than
originally framed.** Overlaid tracked points on frames at ~8m, ~4m, ~2m,
~1m (`/tmp/.../step_ah_overlays/`, inspected directly): at 8m, sector
2's 20 points sit on the tree canopy's edge, as expected. At 4m, only 8
remain -- the canopy has grown too large/low-contrast to yield corners,
so it's now essentially untrackable up close (the same texture-loss
failure mode as the wall, just triggered by proximity/blur instead of
inherent flatness). At 2m and 1m, the few surviving "sector 2" points
sit not on distant background *past* the tree (as hypothesized) but on
a **different nearby object** (a concrete box next to the trunk) and the
ground horizon. By 1m, the trunk itself -- filling most of the frame --
contributes **zero** trackable points. The mechanism: not "background
beyond the obstacle floods in," but "the obstacle itself goes dark
(untrackable) at close range, and whatever else happens to share its
sector fills the gap."

**Step AI: estimator replaced.** New default is `TAU_ESTIMATOR =
"percentile"` in `cheap.py`: per-point tau = r / |flow magnitude|
(direction-agnostic, same formula Step AA's alternative estimator used),
aggregated per sector by the **20th percentile**, not the median --
tested at exactly 20, not swept. The previous estimator (signed-radial
divergence + median) remains available via `CheapStage(estimator=
"divergence")` for comparison. VERIFY on the textured approach (244
pre-collision frame pairs, min_points=10):

| estimator | correlation(distance, min_tau) | trend as min_points 1->10 | max tau over approach |
|---|---|---|---|
| (a) divergence + median (previous default) | -0.061 | 0.390 -> -0.061 (degrades) | 465s (raw), 19.4s (EMA) |
| (b) per-point (r/\|u\|) + median | -0.115 | 0.119 -> -0.115 (degrades, worse than (a) at every threshold) | -- |
| (c) per-point (r/\|u\|) + 20th percentile (new default) | +0.462 | 0.413 -> 0.462 (IMPROVES) | 5.3s |

Confirmed end-to-end through the actual `CheapStage.infer()` (EMA
included): `estimator="percentile"` gives correlation=0.333, max
tau=10.55s; `estimator="divergence"` reproduces the old numbers exactly
(correlation=0.115, max tau=19.40s).

**Not a universal fix -- reported, not hidden.** On wall_approach and
narrow_approach, percentile does NOT flip the sign (still -0.236 and
-0.308 at min_points=10, respectively -- worse than (a) on the wall,
better but still wrong-signed on narrow). This is expected, not a
failure of Step AI: per Step AE, those recordings' target-facing sector
rarely or never supplies min_tau at all, so no per-sector aggregator
change can fix a sector with no real signal in it. Percentile fixes the
**aggregation** failure mode (median diluted by background/off-target
points when the target sector DOES have relevant data); it doesn't and
can't fix the **sector-selection** failure mode (AE) where the target
sector has no data to aggregate in the first place. Both remain live
issues; this pass only fixes the one Step AF's acceptance test actually
found.

## CheapStage close-out (Steps AJ-AM)

**Step AJ -- wall_approach/narrow_approach reframed as valid=False
checks, not tau-correlation targets.** These recordings target
obstacles CheapStage cannot see by design (failure modes 1 and 2 below)
-- they're positive cases for a future gate, not regressions to fix.
Measured directly against the ACCEPTANCE bar (report valid=False rather
than a confident wrong tau): **PASSES on all 4 recordings.**
- wall_approach (all 3 speeds): sector 2's tracked point count is
  **exactly 0 for the entire approach** (mean=0.00, max=0) -- not a
  marginal threshold effect. valid=False on 100% of frames, at every
  distance bucket tested.
- narrow_approach_D1: sector 2 IS valid sometimes (44/235 frames,
  18.7%) -- but only at safe range (35.5% valid at 5-10m) and **exactly
  0% valid inside 5m**, the range that matters most. Where it does
  report confidently, the 44 point counts backing those reports are all
  10-18 (i.e. genuinely clearing min_points_per_sector, not borderline
  noise) and the reported tau values are sane (1.87-7.06s) -- no wild
  outliers among the confident readings. The stage never produces a
  confident wrong answer here; it goes quiet exactly where it should.

**Step AL -- full corridor pass + per-zone feature statistics.** Needed
sim (none was running before this pass; fully torn down after). Flown
in segments rather than one continuous pass -- each straight-line run
ends in a collision at the first centreline obstacle it reaches, so
covering the whole ~125m corridor meant relaunching past each collision
point: `corridor_segment1` (spawn -3 -> collides with wallB1 at x~54.7),
`corridor_segment2` (60 -> collides near boxC2, x~88.2),
`corridor_zonec_fill`/`corridor_zonec_fill2` (84->88.2, then 90->107.6,
filling the rest of Zone C after the first two segments both stopped at
the same spot), `corridor_segment3` (96->107.8, collides approaching
narrowD1), `corridor_segment4` (112->121.8, collides approaching
wallD1). Combined with the existing wall_approach/narrow_approach
recordings, analysis restricted to pre-collision frames (odometry
z>2.5m) throughout, per-frame zone assigned from ground-truth x
position. Zone boundaries were adjusted from the SDF's nominal
obstacle-centre x to the actual pre-collision approach footprint --
e.g. Zone B's nominal x=55-58 was NEVER reached in stable flight in any
recording; the wall's collision surface intercepts the drone at
x~54.6-54.8, short of the nominal boundary. Widened to x=50-58 (and
Zone D similarly to x=104-122) to capture the real approach data that
exists rather than reporting a false "no data" gap.

Per-zone means, `CheapStage` default (percentile) settings, min_points=10:

| zone | n frames | global_feature_count | min_tau defined | sector-2 valid% | mean valid sectors (/5) | confidence | grad_energy (sector 2) |
|---|---|---|---|---|---|---|---|
| gap (pre-A) | 427 | 57.3 | 100.0% | 88.8% | 3.35 | 0.647 | 16.6 |
| **Zone A** (textured trees, off-axis) | 142 | 46.8 | 97.9% | 26.8% | 2.41 | 0.482 | 11.2 |
| gap (A-B) | 1023 | 25.8 | 67.8% | 0.0% | 1.15 | 0.220 | 9.9 |
| **Zone B** (wall, textureless) | 129 | **2.7** | 4.7% | 2.3% | **0.05** | **0.009** | 4.0 |
| gap (B-C) | 609 | 44.4 | 97.0% | 84.1% | 2.22 | 0.432 | 16.9 |
| **Zone C** (textured, mixed) | 243 | 53.7 | 91.4% | 66.3% | 2.12 | 0.413 | 15.5 |
| gap (C-D) | 533 | 33.7 | 86.1% | 35.8% | 1.57 | 0.306 | 13.6 |
| **Zone D** (narrow + wall, textureless) | 551 | 15.4 | 41.0% | 0.9% | 0.62 | 0.119 | 5.0 |

**The zones are dramatically distinguishable -- this is not the Phase 5
blocker the task was checking for.** Zone A/C vs. Zone B/D separate
cleanly on multiple features simultaneously, not just tau:
`global_feature_count` alone is 3-17x higher in A/C (47-54) than in B/D
(2.7-15.4); `confidence` is 4-50x higher (0.41-0.48 vs. 0.009-0.12);
mean valid-sector-count is 3-40x higher (2.1-2.4 vs. 0.05-0.62); Zone
B's confidence (0.009) and valid-sector-count (0.05/5) are both
essentially zero -- CheapStage isn't just less certain in Zone B, it's
almost completely silent there, which is itself an unambiguous,
easy-to-learn signal. A gate could plausibly key off feature-vector
SPARSITY alone (how much of the vector is real vs. NaN placeholder)
without needing tau accuracy to be perfect everywhere -- consistent
with the "defer when confidence collapses" behavior the whole
CheapStage/HeavyStage split is meant to produce.

Caveat worth keeping in mind reading this table: Zone A and most of
Zone C were flown PAST, not INTO -- their obstacles sit off the y=0
centreline (see `worlds/env_zones.sdf`), so those zones' numbers partly
reflect "textured scenery visible nearby" rather than "obstacle
directly ahead," unlike Zone B/D where the drone flew straight into a
centreline obstacle. This matches how the corridor is actually flown in
this codebase (record_pass has no lateral steering) -- not a flaw in
the measurement, just a reason the absolute numbers shouldn't be read
as "CheapStage looking directly at Zone A's trees," only as "what the
camera saw passing through Zone A's airspace."

**Step AM -- latency re-measured at final (shipping) settings.** The
percentile estimator replaced divergence+median as CheapStage's
default; re-measured rather than assumed to confirm the swap didn't
change the cost. Across all 14 available recordings (n=13,787 frame
pairs, the largest sample yet -- includes the new corridor data):
**mean 8.29ms, median 8.31ms, p95 10.28ms, max 24.27ms.** Essentially
unchanged from the pre-swap measurement (mean 8.06ms, median 8.98ms,
p95 10.26ms, max 13.65ms, n=2117) -- the two per-point formulas
(signed-radial-projection vs. plain magnitude) and aggregators
(median vs. percentile) cost about the same, as expected for two
O(n) operations of similar shape. The higher max (24.27ms vs. 13.65ms)
tracks with the much larger sample (13,787 vs. 2,117 frame pairs) and
appears in the largest corridor recordings specifically -- consistent
with occasional system-load variance during a long sequential
measurement run, not a property of the estimator itself. Still 3-5x the
paper's "few ms" assumption; cheap:heavy stays closer to ~1:3 than
~1:10 if heavy lands at 30-45ms, unchanged from the earlier finding.

## Phase 2 remainder -- SectorController + score comparability (Steps AN-AP)

`obst_avoidance/control/` added: `types.py` (`ControlCommand`,
`ControlState` -- state is caller-owned, never held on the controller,
and carries no wall clock, only `packet.t_capture`, so behaviour doesn't
vary with RTF between runs) and `sector.py` (`SectorController`, VFH per
Borenstein & Koren 1991 -- candidate masking that excludes invalid
sectors rather than scoring them zero, argmin selection with no
interpolation, hysteresis with a source-switch bypass, mode logic,
constant-speed command generation). 20 unit tests in
`test/test_sector_controller.py` and `test/test_score_normalization.py`.

**A real bug was caught by testing, not by review:** the hysteresis
streak counter, if naively incremented from the existing dwell counter,
let a challenger appearing after a long stable dwell trigger an instant
switch on a single noisy frame -- exactly what the N-consecutive-frames
requirement exists to prevent. Fixed with a sign convention on
`frames_on_target` (positive = dwelling, negative = in-progress
challenge streak) so the two regimes can't leak into each other within
`ControlState`'s single-counter shape.

**Step AP -- score comparability, and it was a real fix, not paperwork.**
CheapStage's scores were, until this step, raw tau in seconds (capped
at 30) -- NOT bounded [0,1], which would have made `SectorController`'s
`w_tau * (1 - scores[i])` term swing by up to -29 against a goal term
bounded by ~pi, silently drowning `w_goal` regardless of its configured
value. Added `cheap.tau_to_score()`: linear, `score = min(tau /
TAU_CAP_S, 1.0)`, NaN-preserving (invalid stays invalid, never becomes
a fabricated number). Deliberately not a saturating/log curve that
would give more resolution near tau=0 -- HeavyStage doesn't exist yet
to calibrate against, so a cleverer curve now would be tuning against
nothing, the same discipline the controller itself was built under.
CheapStage now asserts this bound on every `SectorBelief` it builds
(not just tested in isolation); confirmed end-to-end on a real
recording (749 valid scores across `textured_approach_treeA2`, min
0.0012, max 0.9608, assertion never fired). `control/sector.py`'s
`tau_crit` was renamed to `tau_crit_score` and its default changed
0.1 <- 3.0s (the exact equivalent under the linear mapping,
3/TAU_CAP_S=30 = 0.1) -- same tuning intent, correct units.

**Step AQ -- `obst_avoidance/platform/vehicle.py`.** `VehicleInterface`
(`@runtime_checkable` Protocol: `send(cmd)`, `state()`) and
`MavrosVehicle`, the mavros-backed implementation. Reuses the exact
dedicated-spin-thread architecture `SimFrameSource` already proved out
(and documented at length) for the identical reason: a node's own
liveness must never depend on the caller's thread or loop cadence. A
fixed 15Hz ROS timer republishes whatever `ControlCommand` `send()` last
stored -- `send()` itself never publishes anything. Verified mechanically
(no live mavros needed for this part): node creation, spin thread alive,
correct pre-connection state (`connected=False`, `position=None`),
`send()` storing the command correctly, the timer firing repeatedly over
0.5s with zero subscribers and no error, clean teardown via `close()`,
and `isinstance(vehicle, VehicleInterface)` holding true. A
`guided_arm_takeoff()` convenience method is included (reusing
`record_pass.FlightControl`'s proven logic and timeouts verbatim rather
than re-deriving it) but is explicitly NOT part of `VehicleInterface` --
arm/takeoff is a one-time startup handshake outside the closed control
loop, not something `SectorController` or the per-frame path touches,
and its wall-clock timeouts are correct there specifically because
Step AN's "no wall clock" rule governs the deterministic per-frame path,
not real-world service-call handshaking.

**Step AR -- `obst_avoidance/runtime/orchestrator.py`, the flow-only
arm.** Wires `FrameSource -> CheapStage -> SectorController ->
VehicleInterface`, logging per frame (belief scores/valid/confidence/
source, the full 54-element feature vector, `ControlCommand`, vehicle
position/velocity, `odom_valid`, and both CheapStage's and the
controller's own latency) to JSONL. Collision detection uses
ground-truth obstacle poses read directly from `worlds/env_zones.sdf`
(not guessed) rather than a Gazebo contact sensor -- only geometry that
actually intersects the 3m flight altitude is modelled (tree canopies
are spheres well above it; only trunks are collidable at cruise
altitude), box/wall yaw rotations are ignored in favor of axis-aligned
bounding half-extents, per this project's established
don't-over-engineer discipline. Goal-reached is `x >= 125m` (just past
`wallD1`). 11 tests in `test/test_orchestrator.py` -- obstacle-distance
math, yaw/heading conversion, and a full `run()` exercised with STUB
`FrameSource`/`CheapStage`/vehicle plus the REAL `SectorController`
(collision detection, goal-reached, per-frame logging, mode/switch
counting -- all verified without needing a live sim; Step AS is what
actually needs one). All 31 tests across Steps AN-AR pass together.

**Step AS -- first closed-loop runs, flow-only arm. The project's first
real collision-rate number.** `tools/run_flow_only_eval.py` drives one
live run (real `SimFrameSource` + `MavrosVehicle` + `CheapStage` +
`SectorController`, `env_zones.launch.py`, y=0 corridor, 1.5 m/s, 3m
altitude); 10 runs with seeded (`seed=42`) randomized start positions
(`x ~ Uniform(-5,0)`, `y ~ Uniform(-1.5,1.5)`, near the corridor's
start), each a fresh sim launch/teardown. Parameters were NOT tuned to
reduce Zone B collisions -- the number below is what the flow-only arm
actually does, unmodified.

| metric | result |
|---|---|
| collision rate | **10/10 (100%)** |
| collision zone | **10/10 in zone_B** (the untextured wall) |
| completion rate | 0/10 (0%) |
| mode distribution (pooled, 11,781 frames) | cruise 35.8%, avoid 52.4%, blind 11.8% |
| target switch rate | mean 8.4% of frames, range 3.8%-19.4% across runs |
| sim-time duration per run | 37.5-40.4s (mean 38.9s), tight across runs |
| CheapStage latency (LIVE, n=11,781) | mean 56.83ms, median 55.92ms, p95 77.23ms, max 114.04ms |
| controller latency (LIVE, n=11,781) | mean 0.066ms, p95 0.093ms, max 3.17ms (negligible, pure arithmetic) |

**This is the expected, correct result, exactly as predicted before
running anything:** the flow-only arm reliably avoids Zone A's textured
obstacles (confirmed independently of the headline number -- every
single collision happened at x~54.4-54.5, Zone B's wall, meaning every
run traversed all of Zone A's obstacles first) and reliably fails at
Zone B, which it structurally cannot see (Step AJ: sector 2's tracked
point count is exactly 0 there, 100% of the time). This 100%/100%
collision-at-B number is the compute floor the gate is measured
against, not a bug to fix.

**Not tested by this batch: Zone C and Zone D.** Every run terminated
at Zone B (x~55) before reaching Zone C (x~86) or Zone D (x~108) --
worth stating plainly rather than letting the table imply otherwise. A
future arm that gets PAST zone B (the gated arm, once HeavyStage
exists) would be the first to actually exercise Zone C/D in a
closed-loop run.

**A genuinely unflattering latency finding, not smoothed over:** live
CheapStage latency (mean 56.83ms) is ~7x Step AM's isolated-replay
measurement (mean 8.29ms, via `LogFrameSource`, no sim contention). The
gap is resource contention -- Gazebo, SITL, mavros, and the whole ROS
graph competing for CPU during an actual closed-loop run, which the
offline benchmark never experienced. 56.83ms is the operationally
honest number; the compute-floor discussion (cheap:heavy ~1:3 vs ~1:10)
should use THIS, not the isolated figure -- and if HeavyStage sees
similar live contention, its own real-world latency will likely be
higher than its own isolated benchmark too, pushing the ratio further
from ~1:10 than previously estimated.

**Operational note, not a code bug:** every one of the 10 runs needed
2-3 `guided_arm_takeoff()` attempts before arming succeeded (the
already-documented transient EKF-not-settled failure this project has
hit repeatedly -- see Step AS's own retry logic, `max_arm_attempts=4`,
added specifically because the original single-retry version failed
outright on run 1's first attempt). Every run's sim was launched and
torn down individually; all 10 confirmed clean afterward, nothing left
running between or after runs.

## Phase 3 -- latency audit + HeavyStage

**Step AT -- latency measurement audit, before trusting the 7x gap.**

1. **Timer basis: WALL CLOCK.** `CheapStage.infer()` uses
   `time.perf_counter()` (confirmed by reading the code, not assumed) --
   this is correct for "how long did this actually take," not sim time.
2. **RTF during measurement: 0.325**, measured directly (`/clock` vs.
   wall clock over 15s) with the drone sitting idle on the ground --
   i.e. the sim was running at under a third of real-time speed BEFORE
   any flight or perception activity started.
3. **CPU utilization during an active flight** (24 one-second samples,
   `ps -eo pid,pcpu,comm`): `mavros_node` 169.8%, `gz sim` 148.1%, and
   -- this is the important one -- **our own `python3` process itself:
   135.6%** (over 1.3 cores). That's ~564% CPU (5.6 cores) from the
   sim+perception stack alone, on a 12-core machine, plus ~95% from
   unrelated background load already running on this shared dev machine
   (VSCode's `cpptools` at 51%, Firefox at 34%) that a real deployment
   or an isolated benchmark would never carry.
4. **Headless re-run: mean 55.08ms (n=646), essentially identical to
   the original 56.83ms.** But **Step AS was already run with
   `use_gui:=false`** (confirmed by reading `do_eval_run.sh`, not
   assumed) -- so this result is expected, not a "drop": GUI rendering
   was never a contributor, ruled out by construction rather than by
   this new experiment. Worth stating plainly since the task's own
   framing assumed GUI might have been on.

**Conclusion: the gap is real, and it is NOT GUI rendering.** It's two
things layered together: (a) genuine contention from the Gazebo+SITL+
mavros simulation stack itself, which the isolated `LogFrameSource`
benchmark never had running alongside it, and (b) this project's own
LIVE process costing more than its offline counterpart on its own
terms -- `SimFrameSource` and `MavrosVehicle` each run a dedicated spin
thread (by design, see their docstrings) that don't exist in an offline
replay loop, and that threading/ROS-message overhead shows up directly
in the 135.6% CPU figure for our own process. **56.83ms is not what a
real deployed drone (no simulator, no shared dev-machine load) would
see** -- it's what THIS evaluation methodology costs. The isolated
8.29ms and the live 56.83ms are both real, honest numbers measuring
different things; neither alone is "the" answer.

**For the cheap:heavy ratio specifically: measure both stages LIVE, in
the SAME loop, on the SAME machine, in the SAME contended conditions**
(per the task's own instruction) -- Step AW does exactly this for
HeavyStage. Whatever absolute inflation this methodology adds, it adds
EQUALLY to both stages, so the ratio between them stays meaningful even
though neither absolute number should be read as a real-deployment
latency budget on its own.

**Step AU -- HeavyStage.** `perception/heavy.py`: Depth Anything V2
Small, frozen, pretrained (`depth-anything/Depth-Anything-V2-Small-hf`,
CPU -- no CUDA available on this machine, confirmed via
`torch.cuda.is_available()`). Reuses `geometry.sector_index()` directly
(imported, not reimplemented) so sector boundaries can never drift
between stages. Per sector: 80th percentile of raw (relative inverse)
depth in that sector's column range -- "nearest dominates," the same
collision-risk reasoning behind CheapStage's Step AI switch to a low
percentile of tau, just at the opposite end of the distribution since
DA-V2's convention is larger-raw-value=closer (confirmed empirically in
this project's own earlier `tools/check_depth.py` diagnostic, not
assumed from the model card -- getting this backwards would silently
invert every danger ranking). Normalised per-frame (min-max against
that frame's own raw-depth range, then flipped) to land in [0,1],
higher=safer, satisfying Step AP's hard bound by construction.

**Valid mask: 100% True across every frame tested** (580/580
sector-slots, textured_approach_treeA2) -- confirms the expected
behavior exactly: depth estimation needs no texture, so HeavyStage
never goes blind the way CheapStage does. The only coded path to
`valid=False` is a perfectly uniform depth map (`frame_max==frame_min`,
a genuine 0/0 case, not a texture problem) -- never observed in
practice on the recording tested.

**Score comparability -- confirmed on the hard bound, NOT on
distribution shape.** Ran both stages on the identical recording
(textured_approach_treeA2, 116 sampled frames, stride=3):

| | CheapStage | HeavyStage |
|---|---|---|
| valid rate | 41.4% (240/580 sector-slots) | 100% (580/580) |
| mean | 0.135 | 0.375 |
| median | 0.092 | 0.341 |
| std | 0.179 | 0.193 |
| range | [0.002, 0.923] | [0.017, 0.971] |

Both satisfy the [0,1] bound -- the literal Step AP requirement -- and
both actually USE most of that range (not clipped to a narrow band).
**But the distributions are shaped very differently, not smoothed over:**
CheapStage is sharply spiked near 0 (this recording ends in a
collision, so its scores skew dangerous as distance closes); HeavyStage
is broadly spread with a peak around 0.3-0.4 and a long tail to 1.0,
reflecting the full depth range visible in every frame rather than
concentrating near the single nearest obstacle the way CheapStage's
sparse, obstacle-only signal does. Mean score differs by nearly 3x
(0.135 vs. 0.375). **A gate switching sources could still see a
discontinuous jump in TYPICAL score level even though neither stage
ever violates the hard [0,1] bound** -- satisfying the letter of the
comparability contract doesn't yet satisfy its actual purpose. Flagged,
not resolved here; worth knowing before the gate is built on top of
both stages.

**The latency finding that changes everything downstream:**
HeavyStage's isolated (offline, `LogFrameSource`, no sim contention)
latency is **mean 1541.3ms per frame** -- CheapStage's isolated latency
on the identical sampled frames was 12.88ms. That's a ~120x ratio in
isolation, dramatically worse than the "30-45ms" figure earlier steps
speculated HeavyStage might land at. This single number reshapes the
entire compute-floor/gating story for the paper -- see Step AW for the
live, same-loop measurement this motivates.

**Step AV -- the metric-scale question, tested empirically, not
recommended from theory.** `geometry.miss_distance()` needs metric Z;
CheapStage has it (`Z = Tz * tau`), HeavyStage's relative inverse depth
does not. Collected (ground-truth distance, CheapStage Z, HeavyStage raw
depth) triples on `textured_approach_treeA2` (61 frames, position from
real odometry, target world x=20.0 known from the SDF).

**First pass (naive, full range) looked like it supported defaulting to
(c):** correlation(heavy_raw, 1/true_dist) = -0.174, R²=0.030 --
essentially nothing, and WRONG-SIGNED. Same for option (a):
correlation(cheap_Z, true_dist) = -0.083 on the 64% of frames where
CheapStage was even valid. Taken at face value, both metric-recovery
options looked dead.

**That first pass was wrong, caught by looking at the actual sequence,
not just the correlation coefficient.** Plotting heavy_raw against
distance in order (far to near) shows a real, mostly-monotonic
increase from ~5.0 (at 10m) up toward ~5.6-7.8 (0.3-0.9m) -- then a
sudden COLLAPSE in the final ~8 frames (distance <0.5m: values crash to
5.9, 5.3, 7.8, 6.3, 3.8, 1.7, 0.7) -- the same close-range model
breakdown already documented for CheapStage in Steps AF/AH, now shown
to affect HeavyStage too. That handful of collapsed points was enough
to erase the real relationship in a plain Pearson correlation over the
full range. **Excluding distance <0.5m (8/61 frames, the same
established breakdown region) recovers a real signal:**
correlation(heavy_raw, 1/dist) = 0.468 (correct sign), **R²=0.219**,
Spearman rank correlation = -0.507 with **p=0.0001** -- statistically
real, not noise, even though R² alone is moderate.

**Recommendation: (c) for now (rank/comparative sectors only, no
metric miss-distance for HeavyStage), but NOT by default -- (b) has
genuine, measured evidence behind it that a future phase could build
on.** Reasoning:
- Option (a) is strictly worse than (b) here: it has LESS data (Z only
  valid on 64% of frames vs. (b)'s ground truth available on 100%) and
  calibrates against CheapStage's OWN noisy estimate rather than true
  position -- any bias/error already documented in CheapStage's tau
  (Steps AF-AI) would get baked directly into HeavyStage's calibration,
  contaminating the two stages with CORRELATED error exactly when the
  paper needs their disagreement to reflect independent perception
  competencies, not shared calibration noise.
- Option (b) is real but not safety-grade yet: R²=0.219 means ~78% of
  the variance in raw depth is still unexplained by inverse distance --
  workable for a coarse metric estimate, not for feeding a safety
  cone-weight/miss-distance calculation with confidence, and this
  project's own explicit stance is "we do NOT improve depth
  estimation" -- engineering a tighter fit would cross into exactly
  that out-of-scope work.
- Option (c) avoids inventing metric confidence HeavyStage doesn't
  actually have, consistent with this whole project's repeated
  discipline of never fabricating a number a stage isn't sure of
  (`SectorBelief.valid`, NaN placeholders, the entire Step AP contract).
  The cost -- weakened cone reasoning for HeavyStage specifically -- is
  real and stated plainly, not hidden: `HeavyStage.infer()` does not
  compute `geometry.miss_distance()`/`cone_weight()` at all (see its
  own module docstring), only `SectorController`'s ranking-based
  argmin/cost logic, which needs comparative scores, not metric ones.
- (b)'s numbers are kept on record specifically so a future phase with
  a stronger case for metric HeavyStage output isn't starting blind --
  the fit `heavy_raw ~= 1.084*(1/dist) + 4.523` (post-exclusion) is a
  concrete starting point, not just "it might be possible."

**Step AW -- heavy-only closed-loop arm. NOT the expected result, and
the real cheap:heavy ratio.** `tools/run_heavy_only_eval.py` (mirrors
`run_flow_only_eval.py`, HeavyStage driving via `HeavyStageAdapter`
instead of CheapStage). **Scope disclosed up front: 5 of the original
10 seeded positions, not all 10** -- HeavyStage's per-frame cost (~4s
live, see below) makes each run materially more expensive, and Step AX
still needed real work afterward; the first 5 of the same `seed=42`
positions were used for a direct, paired comparison against the
matching first 5 of Step AS's runs, not a different sample.

| metric | CheapStage (Step AS, same 5 runs) | HeavyStage (Step AW, 5 runs) |
|---|---|---|
| collision rate | 5/5 (100%) | 5/5 (100%) |
| collision zone | 5/5 zone_B | **5/5 zone_B** |
| completion rate | 0/5 | 0/5 |
| mode distribution | cruise 37.3%, avoid 53.8%, blind 8.9% | **cruise 95.7%, avoid 4.3%, blind 0%** |
| switches per run | [45, 46, 176, 71, 67] | **[1, 1, 1, 1, 1]** |
| perception latency (live) | mean 57.3ms, p95 77.0ms | **mean 4257.1ms, p95 6843.3ms** |
| frames per run | 1136-1200 | 33-40 |

**EXPECTED was "Zone B is passed, depth sees the wall." That did NOT
happen -- 5/5 runs collided at Zone B, same as CheapStage.** Not
smoothed over or re-run with different parameters to get a nicer
number (that would be exactly the "tune away the failure" mistake this
whole project has been careful to avoid at every prior step). Investigated
instead: pulled the actual per-frame score log for run 0's final
approach --

| distance to wall | HeavyStage score (centre sector) |
|---|---|
| 5.26m | 0.43 (moderate) |
| **3.72m** | **0.82 (looks SAFE)** |
| 2.19m | 0.83 (still looks safe) |
| 0.61m | 0.63 |
| collision | 0.09 |

**The score got SAFER-looking as the wall got closer, from 5.26m down
to 2.19m.** This is exactly the failure Step AV's per-frame min-max
normalization was flagged as risking, now confirmed with real flight
data, not just a statistical R² concern: as a flat, textureless wall
fills more of the frame, the RELATIVE depth variation WITHIN that frame
shrinks (there's less near/far contrast left once the wall dominates
the view), compressing the normalized score toward the safe end even
as absolute distance collapses. `target_sector` stayed at 2 (centre)
for the entire approach in every one of the 5 runs (`switches=1` each
-- the one switch is the initial "no target yet -> sector 2" assignment,
never a real avoidance decision) -- the controller never even
attempted to steer away, because the score signal never gave it a
reason to. Combined with only ~35-40 perception updates over the whole
run (~4s/frame), by the time the score correctly collapsed to
dangerous (the collision frame itself, 0.09), there was no time left to
react at 1.5 m/s.

**The real cheap:heavy ratio, measured the way the task demanded --
same loop, same machine, same contention: 74.3x** (4257.1ms /
57.3ms), not the ~30-45ms/~1:3 estimate earlier steps worked from
before HeavyStage existed to actually measure. This -- combined with
the Zone B finding above -- means the current HeavyStage
implementation is not yet the "safety ceiling" the gate should be
measured against on EITHER axis: it's not obviously safer (same
collision zone, same 0% completion) and it's dramatically more
expensive (74x, not 3x). Two independent problems, not one: the
NORMALIZATION problem (score compression near large flat surfaces,
Step AV's flagged concern materializing) and the LATENCY problem (Step
AT/AU's finding, now confirmed live). Fixing the gate's incentive
structure requires knowing both numbers precisely, which is what this
step was for -- reporting them as measured, not adjusting until they
look better.

**Step AX -- Zone C/D coverage, flow-only. The other 60% of the
corridor, and it also fails, but not the same way.** Added
`runtime.ZONE_SEGMENTS` (start x/y + a local `goal_x_m` per zone, so a
segment run's outcome reflects that zone specifically rather than the
whole remaining corridor) and a `--goal-x` override to
`run_flow_only_eval.py`. 3 runs each for Zone C and Zone D (y-jittered
starts, continuing the same `seed=42` sequence rather than a fresh
one), spawned a few metres before each zone.

| | Zone C (dense textured) | Zone D (mixed) |
|---|---|---|
| collision rate | 3/3 (100%) | 3/3 (100%) |
| completion rate | 0/3 | 0/3 |
| collision x (zone starts at 86 / 104) | 87.95, 85.42, 87.95 | 107.47, 107.49, 107.46 |
| time to collision | 4.5-6.2s (fast) | 5.8s (fast, remarkably consistent) |
| mode distribution | cruise 2.3%, **avoid 92.4%**, blind 5.3% | cruise 12.9%, avoid 58.4%, blind 28.7% |

**Flow-only now has three DIFFERENT, independently-characterized
failure modes across the corridor, not one repeated result:**
- **Zone B**: fails blind (0% valid on the target sector -- Step AJ).
  CheapStage never sees the problem at all.
- **Zone C**: fails while screaming -- 92.4% of frames are "avoid" mode,
  the highest of anywhere measured in this project. CheapStage DOES
  detect danger almost constantly here; the dense obstacle cluster (7
  trees/boxes in an 8m span, `worlds/env_zones.sdf`) just doesn't leave
  enough clear room or reaction time for the controller to find a safe
  sector before something is hit. Collision lands consistently 2-6m
  into the zone (85.4-88.0, zone starts at 86) across all 3 runs.
- **Zone D**: fails via a mix -- 28.7% blind (narrowD1's own
  textureless surface, Step AK's failure mode 2) alongside 58.4% avoid.
  Collision lands remarkably consistently at x~107.5, right at
  narrowD1's approach (x=108) in all 3 runs -- tighter clustering than
  either other zone.

**This is real, useful, and previously entirely missing training
signal for the eventual gate**: "CheapStage is blind" (Zone B), "
CheapStage sees danger everywhere but can't act on it in time" (Zone
C), and "CheapStage is blind to one specific obstacle amid otherwise-
visible ones" (Zone D) are three distinguishable situations a gate
would need to tell apart, and until this step there was no closed-loop
data demonstrating any of the latter two exist at all -- every prior
closed-loop run (Step AS, Step AW) terminated at Zone B before ever
reaching them.

**Compute-ratio caveat**: the 74.3x same-loop ratio above is DA-V2
Small on CPU (1541ms isolated / ~4257ms live-loop). Both a Jetson HIL
measurement (GPU depth inference, expected ~30-50ms) and this CPU
number should be reported in the paper -- the CPU number is what was
actually measured in this dev environment, the Jetson number is the
deployment-realistic one, and neither should be presented alone.

## HeavyStage normalization -- mechanism confirmed, fixed calibration found to not generalize (Steps AY-AZ)

**Step AY -- mechanism.** `heavy.py` recomputes `frame_min`/`frame_max`
from `raw.min()/raw.max()` on every single call (lines 94-95, inside
`infer()`) -- genuine per-frame min-max normalization, confirmed by
reading the code, not inferred. This is the exact mechanism Step AW's
hypothesis named: when an obstacle fills the frame, its own sector
stops looking like an outlier against the rest of that same frame, and
`danger = (nearest_raw - frame_min) / (frame_max - frame_min)` can
collapse toward mid-range or fall as the object gets even closer and
more dominant.

VERIFY was run on a new recording, `wallB1_close_approach` (615
frames, spawned at x=48 heading into wallB1 at x=55, reached 0.33m
before collision -- the existing `wall_approach_1p0/1p5/2p0`
recordings never got closer than ~7.7m, too far to see the failure
region). Plot: `step_ay_verify_plot.png` (raw DA-V2 sector-2 value and
normalized score vs. ground-truth distance). The symptom reproduces
offline, matching Step AW's closed-loop numbers: `score` correlates
**negatively** with distance (r=-0.297, p=0.02 over the full 0.25-7m
range) -- it reads safer as the wall gets closer.

**Honest complication**: the VERIFY step's stated premise was "if raw
rises monotonically while score does not, the mechanism is confirmed."
That premise doesn't hold cleanly here -- raw itself is not
monotonic on this wall either (r=+0.257 with distance, wrong sign;
r=-0.02, not significant, after excluding the established <0.5m
breakdown region). So per-frame normalization is real and does produce
the dangerous inversion, but on this obstacle the raw signal feeding
it is already noisy, not just "correctly informative until
normalization scrambles it."

**Step AZ -- fixed calibration, fit and held out as specified.** Fit
`raw = a*(1/dist) + b` on `textured_approach_treeA2` (train, n=71,
dist>=0.5m, matching the established close-range exclusion):

```
raw = 1.122 * (1/dist) + 4.506      train R^2 = 0.245, p = 0.00001
```

Evaluated held-out, per the task's explicit requirement, on
`wallB1_close_approach` -- a *different* obstacle, never seen during
fitting:

```
held-out R^2 = -7.98   (full range, n=63)
held-out R^2 = -5.85   (excluding <0.5m, n=54)
corr(1/dist, raw) = -0.02, p = 0.88   (excluding <0.5m -- no relationship)
```

A held-out R^2 this negative means the tree-fitted line predicts the
wall's raw values *worse than the wall's own mean would*. This isn't
"weak but real" signal -- the two obstacles' raw-value scales are
genuinely different (tree-fit predicts ~4.6-6.8 over the observed
distance range; the wall's own raw values sit mostly at 0.6-3.9). This
is exactly the limitation this file's own module docstring already
names: DA-V2's relative-inverse-depth checkpoints have **no fixed
scale across frames or scenes** -- "the same physical distance can map
to different raw values in different frames, even different scenes."
A single global fixed scale/shift, as originally scoped for this step,
asks a *relative* depth model for something only a *metric* depth
model can give across untrained-for scene content. `heavy.py` was
**not modified** this step -- applying an affine map known to fail
held-out validation this badly would replace one wrong-signed score
source with another, not fix it.

**Status: blocked on a direction call, not a further measurement.**
Steps BA (re-verify score contract) and BB (10-run closed-loop,
zone C/D) both assume a working calibrated HeavyStage; running them
against a calibration already shown to fail held-out would burn the
Step BB compute budget re-confirming a known-broken config. Options on
the table, not yet decided: (1) drop metric recovery, fix the specific
Step AY failure mode with a temporal-window (multi-frame) normalization
reference instead of per-frame, which doesn't need cross-scene
generalization; (2) keep fixed calibration but scope its validity to
same-surface-type held-out sets only (e.g. wall-to-wall), and document
the narrower validity as a limitation; (3) something else. Steps BA/BB
are on hold pending that call.

## Decision: temporal-window normalization, not metric calibration (Steps BC-BE)

Option 1 chosen. Fixed calibration is dead (held-out R^2=-7.98 above) --
DA-V2 is trained to be affine-invariant per image, so a mapping fit on
one scene has no principled reason to transfer to another; that's a
model-capability mismatch, not a tuning problem. Option 2 (same-surface
calibration) was ruled out too: it needs the drone to know what it's
looking at to pick a calibration, which isn't available at runtime.
**Consequence, accepted and documented**: HeavyStage does not recover
metric Z -- `miss_distance()`/`cone_weight()` stay CheapStage-only,
permanently (see geometry.py's module docstring). This costs the
learning path nothing, since every gate feature comes from CheapStage
(Step AP).

**Step BC -- implementation.** `heavy.py`'s per-frame min-max was
replaced with a rolling window over the last `window_frames=30` frames
this HeavyStage instance actually ran inference on (a `collections.deque`
appended to once per `infer()` call -- processed-frame-indexed, not
wall-clock-indexed, so collection-mode and sparser deployment-mode runs
see the same window SEQUENCE regardless of wall-clock spacing between
frames -- see the module docstring for the full train/serve-skew
argument). Reference range is the pooled window's 5th-95th percentile,
not min/max -- a single outlier frame drove the old failure, and a
percentile of ~30 frames pooled together is far harder for one frame to
distort. `nearest_raw` can now fall outside `[ref_lo, ref_hi]` (unlike
the old per-frame min/max, which bounded by construction), so `danger`
is clipped to `[0,1]` rather than left to escape it.

Two bugs caught and fixed during this step, both worth noting since
they'd otherwise have silently invalidated later results:
- The first version called `np.percentile()` twice per frame (5th, then
  95th) on the full-resolution pooled window -- once the window filled,
  that's a ~9M-element array sorted TWICE per frame, which measurably
  dominated latency. Fixed: a single `np.percentile(pooled, [lo, hi])`
  call, and each frame is stored in the window spatially subsampled
  (`WINDOW_SUBSAMPLE_STRIDE=4`, 16x fewer values) -- still tens of
  thousands of pooled values, plenty for a stable percentile estimate.
  Verified flat latency (~1.2-1.6s/frame) from an empty window through a
  full one.
- **The colcon `install/` build was stale** (dated Aug 27, predating this
  entire normalization-fix conversation), and this project's closed-loop
  driver scripts run with a working directory that resolves the
  *installed* copy, not `src/`. Without catching this, every step from
  here on (BD, BE, and the closed-loop runs in BF) would have silently
  exercised the OLD per-frame-normalized code while every log and report
  claimed to be testing the fix. Caught by an `AttributeError` on
  `_raw_window` not existing when constructing `HeavyStage` from the
  project's actual run convention (`cd /home/saurabh/ardu_ws` then
  `python3 src/obst_avoidance/tools/...`) -- fixed by
  `colcon build --packages-select obst_avoidance` (this package's
  install is a real copy, not a symlink, even with `--symlink-install`,
  so **any future edit to `heavy.py`/`geometry.py` needs a rebuild
  before it's live** for anything invoked the project's normal way).

**Step BD -- sampling-rate stability, the acceptance gate.** On two
recordings (`wallB1_close_approach`, 150 frames; `textured_approach_treeA2`,
117 frames), ran HeavyStage twice each: every frame, and every 5th frame.
Compared the argmax-score (safest) sector at the frame indices both runs
processed:

| recording | common indices | differ |
|---|---|---|
| wallB1_close_approach | 30 | 0 (0.0%) |
| textured_approach_treeA2 | 24 | 0 (0.0%) |

**Passed cleanly, no retuning needed.** Sector choice was identical
between the two sampling rates on both recordings -- the
processed-frame-indexed window gives collection-mode and deployment-mode
runs the same decision regardless of how much wall-clock time separates
the frames in it, which was the whole point.

**Step BE -- score contract, re-verified on full recordings** (not
frame-capped, unlike BD's stability check -- 253 rows on wallB1,
`pos[2]>=2.5` filter; 245 rows on treeA2):

| recording | stage | n valid | mean | std | corr(score, dist) | p |
|---|---|---|---|---|---|---|
| wallB1_close_approach | HEAVY | 253/253 | 0.453 | 0.236 | **+0.294** | <0.0001 |
| wallB1_close_approach | CHEAP | 0/253 | -- | -- | -- | -- |
| textured_approach_treeA2 | HEAVY | 245/245 | 0.216 | 0.183 | +0.357 | <0.0001 |
| textured_approach_treeA2 | CHEAP | 152/245 | 0.100 | 0.045 | +0.134 | 0.0996 |

**The wall_approach requirement is met**: heavy's centre-sector score
now correlates POSITIVELY with distance on wallB1 (r=+0.294, p<0.0001)
-- score falls as the wall approaches, exactly reversing Step AY's
measured r=-0.297 on the same recording under the old per-frame code.
Same correct sign on treeA2 (+0.357, p<0.0001).

**Reported plainly, not smoothed over**: r=+0.294 is real and highly
significant (n=253) but not strong -- R^2~=0.086, roughly 9% of
variance explained. The underlying raw signal's own noisiness on this
wall (Step AY's finding) hasn't gone away; the window fix corrected the
SIGN of the bias (no longer systematically wrong), it did not turn a
noisy sensor into a precise one. Whether that's enough margin to avoid
Zone B in closed loop, where it matters, is exactly what Step BF tests
next -- this offline correlation is necessary but not sufficient
evidence.

CheapStage's 0/253 valid readings on wallB1 is not a regression --
it's Step AJ's already-documented "CheapStage is blind at Zone B"
finding, reproduced here on a different (closer-range) recording of the
same wall. The two stages' score distributions on treeA2 (heavy
mean=0.216/std=0.183 vs cheap mean=0.100/std=0.045) are both in [0,1]
and both correctly signed, but are not numerically close -- heavy's
window-relative ranking spans a fuller range than cheap's tau-based
score, which saturates hard on a close, fast approach. Flagged as an
open asymmetry, not silently smoothed into "comparable enough."

Proceeding to Step BF (10-run closed-loop) on the strength of Step BD's
clean pass and Step BE's correct-signed, significant (if not strong)
wall response.

## Step BF -- heavy-only closed-loop, temporal-window fix. FAILS Zone B, contingency triggered.

**7 of the planned 10 seeded runs completed (same `seed=42` methodology
as Step AS: `random.Random(42)`, `x~Uniform(-5,0)`, `y~Uniform(-1.5,1.5)`
draws consumed in order -- regenerated from the documented methodology
for this batch, not replayed from an archived script, since the literal
Step AS generator wasn't retained; noted here in case the exact draw
values ever need reconciling against Step AS's original run). The batch
was stopped after run 6, before runs 7-9 and the Zone C/D segments, for
two independent reasons that both point the same direction: (1) the
result was already unanimous and tightly clustered enough that 3 more
runs would not have changed the conclusion, and (2) the machine hit a
sustained load average of 15-19 (12 cores) partway through run 7 --
Gazebo + ArduPilot SITL + mavros + DA-V2 inference running concurrently
for an extended stretch -- and continuing to grind through the rest of
the batch for a statistically unnecessary 10/10 wasn't worth the
thermal cost. Run 7 was killed mid-flight (no result recorded); runs
0-6 completed cleanly before the stop.

| run | collision zone | collision x | frames | modes (cruise/avoid/blind) | switches |
|---|---|---|---|---|---|
| 0 | zone_B | 54.60 | 39 | 38/1/0 | 1 |
| 1 | zone_B | 54.73 | 41 | 39/2/0 | 1 |
| 2 | zone_B | 54.58 | 39 | 37/2/0 | 1 |
| 3 | zone_B | 54.57 | 39 | 38/1/0 | 1 |
| 4 | zone_B | 54.75 | 41 | 39/2/0 | 1 |
| 5 | zone_B | 54.57 | 42 | 40/2/0 | 1 |
| 6 | zone_B | 54.53 | 41 | 39/2/0 | 1 |

**7/7 (100%) collide at Zone B**, mean collision x=54.62m (range
54.53-54.75m, essentially the same point every time) -- statistically
indistinguishable from Step AW's original 5/5 result under the broken
per-frame code (collisions there landed 54.4-54.5m). `switches=1` in
every single run: `target_sector` never actually changed away from its
initial default assignment, meaning the controller never attempted a
real avoidance maneuver, in any of the 7 runs -- the identical signature
Step AW reported.

**Pulled the centre-sector score trace for run 0's final approach to see
exactly how it fails** (`eval_results/heavy_bf_corridor_run0_frames.jsonl`):

| t_capture (s) | centre-sector score |
|---|---|
| 51.9 | 0.280 |
| 54.0 | 0.288 |
| 56.1 | 0.375 |
| 58.0 | 0.426 |
| 60.0 | 0.546 |
| 61.1 | **0.770** |
| 62.1 | **0.802** |
| 63.1 | 0.462 (post-collision) |

**The score climbs from 0.28 to 0.80 in the ~10 seconds before
collision -- the same qualitative shape as Step AW's original bug**,
reproduced through the new mechanism. This directly contradicts Step
BE's offline finding (correct-signed, r=+0.294) on `wallB1_close_approach`,
and the likely reason why is itself an important, previously
unconsidered gap in Step BC's design: Step BE's offline test replayed a
recording that STARTS already close to the wall (spawned at x=48,
approaching x=55), so its entire 30-frame window is built from wall-only
content with genuine near/far variation across the approach. This
closed-loop run instead starts ~57m earlier and flies through all of
Zone A's varied scenery first; with HeavyStage processing only ~40
frames across the whole ~40s flight, a 30-frame window is nearly the
ENTIRE run's history even at the moment of collision. As the wall
starts to dominate several consecutive recent frames near the end of
the approach, those same near-uniform close-wall values increasingly
populate the window's OWN 5th-95th percentile reference range -- so the
"how extreme is this sector's raw value against my reference" question
starts collapsing again, just diffused across a window instead of
trapped inside one frame. The window widened the reference just enough
to fix an isolated, idealized wall-only replay; it did not survive
contact with a real flight's actual window contents.

**Contingency triggered, per the task's own instruction: not tuning,
reporting and stopping.** BE passed offline; BF fails in the realistic
closed-loop setting BE couldn't fully stand in for. Shrinking
`window_frames` or narrowing the percentile range might chase a better
number on this specific batch, but that's exactly the tuning-to-the-test
the task explicitly rules out, and Step BF's own mechanism finding above
suggests it wouldn't fix the underlying issue anyway -- any window still
eventually fills with the wall's own increasingly-uniform recent
frames the closer the vehicle gets, for any window short enough to
matter in ~40 total frames. Zone C and Zone D segment runs were not
attempted -- BF's own acceptance criterion (Zone B passed) already
failed unambiguously, and every prior closed-loop arm (Step AS, AW, and
now this one) has terminated at Zone B before ever reaching them
regardless.

**Where this leaves HeavyStage**: two independent normalization schemes
(per-frame min-max, Step AY; rolling multi-frame window, Step BC) have
now both failed to make Zone B survivable in closed loop, despite the
second one measurably fixing the score-vs-distance correlation SIGN in
an isolated offline test. This is consistent with the contingency
section's own stated hypothesis: a 16m-wide, flat, textureless wall
at close range may be a genuinely hard case for ANY within-scene
relative-depth normalization, not a scheme this project hasn't found
yet -- once the obstacle fills most or all of several consecutive
frames, there is no reference left (single-frame OR
multi-frame-recent-history) that still contains real near/far contrast
to normalize against. The task's own proposed fallback -- resizing
Zone B's obstacle so scene depth variation survives at close range,
rather than continuing to chase a normalization fix for a 16m-wide flat
wall -- is a world-design decision, not a code fix, and is flagged here
for deliberate sign-off rather than made unilaterally.

**Also worth recording: HeavyStage's effective closed-loop rate is
~1Hz, not the ~4257ms-per-call number's reciprocal alone would suggest.**
Step BF's own runs got only ~33-42 perception updates across a ~38-41s
flight -- roughly one HeavyStage update per second of flight, because
1541ms/frame on this machine's CPU (Step AU, isolated) dominates the
control loop; the vehicle keeps flying on the last command
(`VehicleInterface`'s 15Hz republish, see `platform/vehicle.py`) between
those far-rarer perception updates. This matters for two things: (1) any
deferral-rate arithmetic in the eventual learned gate needs to account
for HeavyStage's update rate itself being this low, not just its
per-call cost in isolation -- a gate that "defers to heavy" 10% of
frames doesn't cost 10% of heavy's per-call latency if heavy can only
physically run ~once a second regardless of how often it's asked; (2)
this is a strong, concrete argument for why the HIL Jetson measurement
matters beyond just a nicer latency number -- GPU depth inference at an
estimated 30-50ms (vs this machine's 1541ms CPU) would let HeavyStage
run close to CheapStage's own update rate instead of ~30-75x slower,
which changes the whole shape of the deferral-rate/safety-margin
tradeoff the gate has to learn, not just its cost. Both the CPU number
measured here and the eventual Jetson number should be reported
side-by-side in the paper, not the CPU one alone.

## Step BG/BH -- Zone B rebuilt, both stages re-verified. Real but late-onset heavy signal.

**Step BG -- box geometry, computed not eyeballed.** Replaced wallB1's
16m-wide/6m-tall box with a **1.0m wide x 0.4m thick x 1.5m tall** box,
same position (55, 0, 3). Sized from the camera's actual intrinsics
(fx=205.4696, cx=320, width=640) rather than the task's own rough
~23deg/sector estimate: `geometry.sector_index()`'s 5 sectors are
PIXEL-linear, not ANGLE-linear, so under the pinhole mapping
theta=atan((x-cx)/fx) the computed sector boundaries are actually
**[-57.30, -43.06, -17.30, +17.30, +43.06, +57.30] deg** -- centre
sector 2 spans 34.6deg (not the naive 115/5=23deg), the outermost
sectors only 14.24deg each. A 1.0m-wide box's angular half-width stays
under sector 2's 17.30deg half-angle down to d=1.6m (14.04deg at the
task's specified 2m checkpoint, ~3.3deg of margin), only spilling into
sectors 1/3 below that -- deepening further within the <0.5m breakdown
region Step AY/AZ already established applies to both stages. Full
reasoning is in `worlds/env_zones.sdf`'s comment above the model.
`orchestrator.py`'s ground-truth `OBSTACLES` entry updated to match
(`half_y=8.0` -> `half_y=0.5`). All 31 tests still pass; both the SDF
and the Python package were rebuilt via `colcon build` (this project's
`install/` is a real copy, not a symlink -- see Step BC's note).

**Step BH -- verify both stages, `boxB1_approach` recording** (1.5m/s,
3m altitude, spawned x=48; open-loop straight flight -- no
avoidance, so it physically collided with the box around x=55, same as
every prior open-loop wall-approach recording used for offline analysis
in this project). 165 pre-collision rows (`pos[2]>=3.0`, dist range
0.24-6.98m):

| stage | valid rate | mean | std | corr(score,dist) | p |
|---|---|---|---|---|---|
| CHEAP | 1.8% (3/165) | 0.275 | 0.124 | 0.867 | 0.333 (n=3, meaningless) |
| HEAVY | 100% (165/165) | 0.284 | 0.134 | **+0.397** | <0.00001 |

**CHEAP fails as designed** -- 1.8% valid, essentially nothing to
track on an untextured box, exactly the positive case Step BH wanted.

**HEAVY's overall correlation is real and significant, but reporting it
as a single number would hide the actual shape -- split by range**:

| range | n | corr(score,dist) | p | score mean | score std |
|---|---|---|---|---|---|
| far (>=1.5m) | 121 | -0.051 | 0.578 (n.s.) | 0.328 | 0.038 |
| near (0.5-1.5m) | 20 | **+0.454** | **0.045** | 0.082 | 0.085 |
| close (<0.5m) | 24 | 0.240 | 0.259 (n.s.) | 0.226 | 0.249 |

**This is not a gradual, monotonic ranking across the approach -- it's
closer to a step function.** From 7m down to 1.5m (121 of 165 rows,
nearly the whole approach), the score sits flat and uninformative
around 0.33 with no significant relationship to distance at all. Only
between 1.5m and 0.5m does a real, significant, correctly-signed drop
appear (mean falls from 0.328 to 0.082) -- this is what pulls the
overall correlation to +0.397. Below 0.5m it's noisy again, consistent
with the already-established close-range breakdown region.

**Qualitatively different from Step BF's failure, but with its own
practical concern to flag plainly, not smooth over.** Step BF's flat
wall showed the score moving the WRONG way as danger increased (rising
0.28->0.80 right before collision) -- an active hazard. This box shows
the RIGHT sign where it changes at all, and the drop happens exactly in
the range that matters for a collision decision, not spuriously
elsewhere. But it only starts ranking the obstacle within ~1.5m -- at
this recording's 1.5m/s closing speed, that is about **1 second of lead
time** before the signal says anything. Whether that is enough for
`SectorController` to actually execute a turn before impact is exactly
what Step BI's closed loop will show; this offline result does not by
itself say the answer either way, in either direction.

**Not treated as a trigger for the "if heavy still cannot rank it,
report and stop" clause** -- heavy DOES rank it, with a real,
significant, correctly-signed relationship in the decision-relevant
near range, which is different from "cannot rank it." The box was not
resized to chase a cleaner far-range number. Proceeding to Step BI is
the actual test of whether a ~1.5m-only signal is sufficient; flagged
to the user before launching it given the resource/thermal cost of a
10-run closed-loop batch (Step BF's batch pushed this machine to a
sustained load average of 15-19 on 12 cores).

## Step BJ -- sector bearings were wrong (pixel-linear vs angle-linear), now fixed

`control/sector.py`'s `_sector_bearing()` computed each sector's bearing
as a uniform angular split (`fov/2 - (sector+0.5)*(fov/n)`) -- correct
only if pixel columns were equal-angle, which Step BG already showed
they are not under a pinhole projection. Added
`geometry.sector_bearings_rad(width, fx, cx, n_sectors)`, which computes
each sector's PIXEL-COLUMN-CENTRE bearing trigonometrically
(`theta=atan((cx-px_centre)/fx)`, sign-corrected to match the
ENU/mavros convention `control/sector.py`'s module docstring already
establishes -- the task's literal `atan((px-cx)/fx)` has the opposite
sign, which would have silently inverted every steering decision had it
gone in unchecked).

**Corrected sector-centre bearings** (this project's camera,
fx=205.4696, cx=320, width=640):

| sector | corrected bearing (trig) | naive linear estimate* |
|---|---|---|
| 0 (far left) | **+51.25 deg** | +40.05 deg |
| 1 | **+31.92 deg** | +13.35 deg |
| 2 (centre) | 0.00 deg | 0.00 deg |
| 3 | **-31.92 deg** | -13.35 deg |
| 4 (far right) | **-51.25 deg** | -40.05 deg |

*naive linear estimate uses the REAL measured HFOV (114.59deg)/5 split
-- sectors 1 and 3 are the worst-affected, off by ~2.4x (13.35deg vs
the true 31.92deg), not just the ~50% sector-WIDTH discrepancy Step BG
found for the centre sector alone; sectors 0/4 are off in the other
direction (linear split overestimates them: 40.05deg vs true 51.25deg).

**A second, compounding bug found while fixing this**: `SectorControllerConfig.horizontal_fov_rad`
already carried a code comment saying it "should normally be
constructed with the real value derived from camera intrinsics, not
this default" -- but grepping every caller (`run_flow_only_eval.py`,
`run_heavy_only_eval.py`, `test_orchestrator.py`) showed NONE of them
ever did. Every closed-loop run in this project so far (Step AS, AW,
BF) computed its goal-heading cost term against the 80deg PLACEHOLDER
default, not the camera's real ~114.6deg FOV -- a second source of
wrong theta_i, independent of the pixel-linear issue this step targets.

**Fix**: `SectorControllerConfig` gains `sector_bearings_rad: tuple = ()`
(preferred -- real, trig-computed bearings) alongside the existing
`horizontal_fov_rad` (now explicitly documented as a fallback
approximation for synthetic unit tests only, which check qualitative
sign/ordering, not real numeric geometry). Both real eval-script
callers (`run_flow_only_eval.py`, `run_heavy_only_eval.py`) now compute
`geometry.sector_bearings_rad(intr["width"], intr["fx"], intr["cx"])`
from the SimFrameSource's actual intrinsics and pass it in. Unit tests
(`test_sector_controller.py`) untouched -- they only assert qualitative
properties (yaw_rate sign, bound) that hold under either bearing
source. All 31 tests still pass after the fix.

**Consequence for prior closed-loop numbers**: every run reported in
Step AS/AW/BF used the wrong (80deg-placeholder, linear-split) goal
bearings. Since `w_goal=0.5` vs `w_tau=1.0`, the goal term mostly
affects tie-breaking BETWEEN similarly-safe sectors, not first-order
safety decisions when one sector is clearly more dangerous than
others -- so this is unlikely to overturn the Zone B collision results
themselves (which were driven by the score term reading the whole
approach as safe, not by a goal-heading tie-break), but it does mean
any reported "target switch" / steering-preference behavior in those
prior sections should be read with this caveat. Not re-run
retroactively; flagged here rather than silently revised.

## Step BK -- lead-time analysis (offline, no closed-loop runs)

**Do not run BI yet, per the task -- compute the physics first.**

**1. Lead-time window.** BH's segmented analysis put the informative
onset at ~1.5m (matches BG's own 1.6m geometric containment guarantee).
Collision is registered at centre-distance = the box's own half_x
(0.2m, its surface). Window = 1.5 - 0.2 = **1.3m**. At v=1.5 m/s:
**T_lead = 1.3/1.5 = 0.867s.**

This 1.5 m/s and the resulting seconds are SIM-TIME, which matters here
because of a real subtlety: Step BF's OWN 7 runs show the simulation
running at RTF~=0.34-0.35 while HeavyStage inference is active (mean
wall-clock latency 2913ms/call, n=282, but n_frames/duration_s is a
remarkably consistent 0.97-1.03 across all 7 runs) -- i.e. Gazebo's
physics loop is itself starved by HeavyStage's own CPU use, so sim-time
advances slower than wall-clock during inference. Both the vehicle's
1.5 m/s forward speed AND its perception update rate are sim-time
quantities that dilate together under this contention, so using
sim-time consistently throughout (not mixing in the 2913ms wall-clock
figure) is the physically correct choice here, and happens to validate
the task's own "~1Hz" estimate: n_frames/duration_s across Step BF's 7
runs is 0.97-1.03 Hz, not the ~0.34Hz a naive 1/2913ms wall-clock
calculation would suggest.

**2. Depth frames in the window.** At ~1.0 Hz (sim-time, confirmed
above) and a 0.867s window: **~0.87 expected updates**. Perception
updates are not phase-locked to when the box crosses the 1.5m
threshold, so this means AT MOST ONE fresh update can land inside the
window, and there is a real chance of ZERO -- i.e. the controller may
still be acting on a stale, pre-threshold reading for the window's
entire duration, only getting a first informative reading after the
window has already closed.

**3. Achievable heading change.** `fwd_vel` is body-frame (see
`control/types.py`'s `ControlCommand` docstring), so a sustained
max-rate yaw traces a constant-radius arc, not a straight line with a
late lateral offset -- R = v/omega_max = 1.5/0.8 = 1.875m. Optimistic
case (a fresh update lands at the EXACT start of the window, zero
decision/republish latency, continuous max-rate turning for the whole
0.867s):

```
theta = omega_max * T_lead = 0.8 * 0.867 = 0.693 rad (39.7 deg)
lateral clearance y = R*(1-cos(theta)) = 1.875*(1-cos(0.693)) = 0.433 m
forward progress x = R*sin(theta) = 1.198 m (vs 1.30 m needed to reach the box --
  turning itself slows closure a little, a small amount of extra grace
  not counted anywhere else in this analysis)
```

**0.433m of achievable lateral clearance vs 0.5m needed just to clear
the box's bare half-width -- short by 0.067m, before any airframe
half-width or safety margin is added.** This is the BEST case: a fresh
update at the perfect moment, zero pipeline latency, and full-window
max-rate turning. Point 2 already established that a fresh update
landing at the right moment is not guaranteed at ~1Hz vs a 0.867s
window; real pipeline latency (command computation, 15Hz republish
timer) would eat further into the 0.867s; and no airframe margin is
included at all -- every one of these makes the real number worse, not
better.

**Prediction for Step BI: avoidance is not physically achievable in the
available window, even under best-case assumptions.** Per the task's
own framing, running the 10-run closed-loop batch now would very likely
just re-confirm this at the machine's cost already spent on Step BF's
batch (sustained load average 15-19). Not run.

**This changes what "too small" vs "too big" means for Zone B's
obstacle, worth stating plainly**: the ORIGINAL 16m wall failed because
it filled the view, leaving no depth contrast (Step AY/BF). This 1.0m
box's arithmetic problem is different in kind -- it is geometrically
well-contained (Step BG) and DOES rank correctly where it ranks at all
(Step BH's near-segment), but a relative-depth model's own detection
range on a 1.0m untextured object, combined with this project's actual
closed-loop physics (1.5 m/s constant speed, 0.8 rad/s max yaw, ~1Hz
sim-time perception rate), leaves too little geometric margin regardless
of score quality. A bigger box makes the normalization problem worse
(Step AY/BF); a box any smaller makes the detection-range problem
worse still. Per the task's contingency, this is flagged for a
deliberate decision among (a) reducing flight speed for the heavy arm
specifically (breaks the constant-speed experimental design, last
resort), (b) accepting ~1.5m detection range as a documented DA-V2
Small limitation and designing obstacle spacing around it, or (c) one
more (not repeated) resize attempt at an intermediate size -- not made
unilaterally here.

**Decision: option (a), reduce flight speed for the heavy arm.**
Explicitly chosen over (b)/(c) despite being the task's own stated last
resort -- documented here as a deliberate, disclosed deviation from the
constant-speed experimental design that has held cheap and heavy at the
same 1.5 m/s through every prior comparison in this project, not a
silent tuning.

Re-solving Step BK's arithmetic for v instead of holding it fixed at
1.5 m/s (window distance stays 1.3m regardless of speed; both lead time
AND achievable turn angle scale with 1/v, so slower speed helps on both
the "does a fresh update land in the window" axis and the "how much can
it turn" axis simultaneously):

| v (m/s) | T_lead (s) | expected updates | clearance (m) | margin vs 0.5m |
|---|---|---|---|---|
| 1.5 (original) | 0.867 | 0.867 | 0.433 | -0.067 |
| 1.2 | 1.083 | 1.083 | 0.529 | +0.029 |
| 1.0 | 1.300 | 1.300 | 0.617 | +0.117 |
| **0.8 (chosen)** | **1.625** | **1.625** | **0.733** | **+0.233** |
| 0.6 | 2.167 | 2.167 | 0.871 | +0.371 |

**0.8 m/s chosen**: comfortable margin on both fronts (0.733m
achievable clearance vs 0.5m needed, +0.233m buffer -- enough headroom
to absorb a real airframe half-width, which the bare-box-only
comparison above still doesn't include) without going as slow as 0.6
m/s, which would roughly double per-run sim-time duration for no
further first-order benefit (returns are diminishing past ~0.8: each
0.2 m/s step down buys progressively smaller margin gains once the
achievable-clearance curve is well past the 0.5m threshold).

**Implementation, corrected**: not a per-script flag. `fwd_vel`'s
DEFAULT in the shared `SectorControllerConfig` (`control/sector.py`)
changed from 1.5 to 0.8 -- ONE value, in ONE place, that both
`run_flow_only_eval.py` and `run_heavy_only_eval.py` pick up
identically since neither overrides it. This keeps cheap and heavy a
controlled comparison at whatever the shared speed currently is, rather
than deliberately splitting them across two speeds -- an earlier
version of this fix used a per-script `--fwd-vel` override defaulting
heavy alone to 0.8 while leaving cheap at 1.5; that was corrected here
before either arm was re-run, since it would have made the two arms'
results not directly comparable again, the exact thing Step AP's score
contract and this whole project's "controlled comparison" discipline
exists to avoid.

**Consequence: Step AS's flow-only baseline (1.5 m/s) is now stale and
must be re-run at 0.8 m/s** to re-establish the compute floor before
Step BI's heavy-only numbers can be compared against anything. All 31
tests still pass (no test constructs `SectorControllerConfig` with a
speed-sensitive assertion).

### Prediction, stated before re-running: CheapStage SNR should degrade at 0.8 m/s

Recorded here BEFORE the re-run, so the flow-only numbers aren't a
surprise either way (matching this project's standing practice --
compare Step AW's "EXPECTED was..." framing).

`cheap.py`'s per-point tau is `r_px / mag`, where
`mag = |resid_flow| / dt` -- the residual (de-rotated) optical-flow
MAGNITUDE in pixels, directly in the denominator. Translational flow
magnitude scales roughly linearly with forward speed for a given
point's depth/radial position, so dropping from 1.5 to 0.8 m/s (a
~47% cut) should shrink `mag` by a comparable fraction across the
board. Two concrete, testable predictions follow directly from that:

1. **Tau noise should increase.** `tau = r_px/mag` amplifies any FIXED
   absolute pixel-level noise (LK sub-pixel tracking error -- literally
   the same `err` field `track_pair()`'s 75th-percentile filter already
   uses) into a LARGER relative error as `mag` shrinks -- classic 1/x
   noise amplification as the signal approaches the noise floor. Same
   underlying tracking precision, smaller signal to divide by.
2. **The inlier filter's fixed floor may bite differently.**
   `track_pair()`'s outlier rejection is `mags < max(3*median, 10.0)`
   -- an ABSOLUTE 10px floor. At lower overall flow magnitude, that
   fixed floor becomes relatively MORE permissive (accepts a wider
   *relative* spread as "inlier"), which could let noisier points
   through rather than reject them -- the opposite failure mode from
   simply losing points, but degrading quality rather than count.

**Expected observable effect**: `MIN_POINTS_PER_SECTOR` (10, the
threshold below which a sector goes invalid) may or may not be crossed
more often -- that depends on corner-detection count, which doesn't
directly depend on speed -- but PER-POINT tau/score noise should be
visibly higher at 0.8 m/s than the 1.5 m/s numbers Step AS reported,
independent of whether valid-point counts themselves change much. If
the re-run's cheap-arm numbers come back noisier (wider score std,
more erratic mode-switching, possibly different collision timing/x)
that is the predicted mechanism confirming itself, not a new bug to
chase.

Step AS is being re-run at 0.8 m/s now to measure this directly, then
Step BI (10-run heavy-only closed-loop, full corridor + Zone C/D,
ACCEPTANCE: Zone B passed) follows at the same speed.

## Step AS re-run at 0.8 m/s -- compute floor changed ZONE, not just noise level

Full 10 seeded runs (same seed=42 positions as the original Step AS),
`fwd_vel=0.8` via the new shared default. One run (`flow08_run1`) had to
be discarded and re-run after the machine suspended overnight mid-run
and the live ROS2/DDS connection never recovered ("Time jump detected",
mavros stuck indefinitely in "DDS: Initialization waiting...") --
environmental, not a code issue; the driver script now wraps the eval
call in a 300s `timeout` so a repeat doesn't hang silently again.

| run | zone | frames | duration_s | cruise | avoid | blind | switches |
|---|---|---|---|---|---|---|---|
| 0 | zone_C | 3309 | 109.8 | 1750 | 1009 | 550 | 271 |
| 1 | zone_B | 2203 | 73.6 | 1154 | 1042 | 7 | 171 |
| 2 | zone_B | 2108 | 70.5 | 1276 | 829 | 3 | 129 |
| 3 | zone_C | 3248 | 108.3 | 1598 | 1005 | 645 | 299 |
| 4 | zone_C | 3336 | 111.1 | 1715 | 1049 | 572 | 320 |
| 5 | zone_B | 2208 | 73.7 | 605 | 976 | 627 | 444 |
| 6 | zone_C | 3509 | 116.8 | 1836 | 1183 | 490 | 313 |
| 7 | zone_B | 2130 | 71.0 | 1186 | 927 | 17 | 174 |
| 8 | zone_B | 2218 | 73.7 | 1423 | 788 | 7 | 148 |
| 9 | zone_C | 3271 | 108.7 | 1683 | 1079 | 509 | 283 |

**Collision rate: still 10/10 (100%)** -- unchanged from Step AS.
**Collision ZONE: 5/10 zone_B, 5/10 zone_C** -- a real split from Step
AS's 10/10 zone_B. **Pooled mode distribution: cruise 51.7%, avoid
35.9%, blind 12.4%** vs Step AS's cruise 35.8%/avoid 52.4%/blind 11.8%
-- cruise and avoid roughly swapped shares; blind stayed essentially
flat (11.8%->12.4%). **Switches: range 129-444, mean 255.2** vs Step
AS's range [45,46,176,71,67], mean ~81 -- roughly 3x higher.

**The SNR prediction is confirmed by the switch-count and mode-share
data** -- both point the same direction (more erratic target-sector
churn, less time confidently cruising per unit distance covered) and
match the predicted mechanism (`tau=r_px/mag` amplifying fixed
tracking noise as flow magnitude shrinks with speed).

**But the zone_B->zone_C split is NOT a speed effect, and reporting it
as one would be wrong -- it's a confound this comparison did not
control for.** Collision x for every zone_B run still clusters tightly
at 54.45-54.66m; every zone_C run clusters at 85.40-87.93m, an EXACT
match to Step AX's already-documented Zone C cluster (dense obstacle
group, collisions landing 85.4-88.0m). The reason: **Step BG's Zone B
world-geometry change (16m wall -> 1.0m box) was a change to
`worlds/env_zones.sdf`, which every sim launch loads -- it was never
scoped to the heavy arm alone.** These flow-only corridor runs are
therefore testing against the NEW, much-narrower box, not the wall
Step AS originally measured against. CheapStage is still
STRUCTURALLY BLIND there (Step AJ's sector-2-always-zero-points finding
has nothing to do with obstacle width, only texture) -- what changed is
that a box 1.0m wide leaves enough of the seeded ±1.5m y-jitter starts
room to clear it by lateral position alone, flying blind, where a wall
spanning y=-8 to +8m left no such room for any of them. Five of ten
seeded (x,y) draws happened to have enough lateral offset once erratic
noisy steering nudged them further; the other five didn't. This is
geometry-driven luck on top of an unchanged blind spot, not a
perceptual improvement -- and it means Step BI's own upcoming Zone B
number is being tested against the SAME box these cheap-arm runs
share, so the two arms remain comparable to EACH OTHER, just not
directly to Step AS's original wall-era numbers without this caveat.

**Compute floor updated**: the honest floor to compare a future gated
arm against is now this table (10/10 collide, 5/10 at the box, 5/10 in
Zone C), not Step AS's original wall-era 10/10-at-zone-B number, which
described an obstacle that no longer exists in this world.

## Step BI (partial) -- 5/5 valid heavy-only corridor runs, PAUSED at load 21.77

Same seed=42 positions, fwd_vel=0.8 (shared default), against the new
box. **4/5 valid runs passed Zone B** (collided zone_C instead, same
cluster Step AX/BM already characterize); **run 5 collided at the box**
(y~0.02 at closest approach -- essentially dead-centre, no lateral
margin at all). Runs 2 and 6 were discarded: both hit the same
DDS-connection hang seen in the flow08 batch (`ros2 service call
.../set_mode LAND` in the teardown step also has no timeout wrapper and
can hang the same way -- caught and killed manually both times, no
code fix attempted yet since the batch was paused before finishing).
Batch paused by explicit request after run 5, with load having reached
21.77 -- the highest of this whole session.

## Step BL/BM -- did heavy actually SEE the box, or just miss it geometrically?

**The right question, and the answer is PASSED BY GEOMETRY, both arms.**
`switches=1` in all 5 heavy runs was the tell: the controller never
re-targeted away from sector 2 in ANY of them, including the 4 that
"passed." 5/10 (flow) vs 4/5 (heavy) was never a distinguishable
difference at these sample sizes on its own -- this step checks the
actual mechanism instead of the pass count.

**Method**: for each run's logged frames (`vehicle_position`, `cmd.mode`,
`scores[2]`), checked (1) lateral offset y as the run passed the box's
x-position, (2) whether the trajectory's y ever came within the box's
0.5m half-width, (3) whether "avoid" mode triggered anywhere in the
x=[48,56] approach window and at what x, (4) the centre-sector score
trace over that window, (5) how many HeavyStage updates landed in the
x=[52,54.5] informative window Step BH established.

**Step BL (heavy, all 5 valid runs)**:

| run | y at closest approach | intersects 0.5m half-width? | first "avoid" in [48,56] | verdict |
|---|---|---|---|---|
| 0 | -1.403 | no | never | PASSED BY GEOMETRY |
| 1 | -0.897 | no | x=54.82 (at/after the box's own front face, x=54.8) | PASSED BY GEOMETRY |
| 3 | -1.226 | no | x=55.58 (past the box) | PASSED BY GEOMETRY |
| 4 | -1.390 | no | x=55.33 (past the box) | PASSED BY GEOMETRY |
| 5 | +0.016 | **YES -- collided** | never | (control: no margin, no avoid, hit it) |

In every passed run the lateral offset was already 0.81-1.4m before the
box, unchanged through the approach -- there is no proactive course
change anywhere in the data. The few "avoid" flags that appear (runs 1,
3, 4) all fire AFTER the vehicle's x has already reached or passed the
box, i.e. simultaneous with or following the close pass, not causing
it. Run 5 -- the one run with near-zero y offset, an actual collision
course -- shows the counterfactual directly: score never drops, avoid
never triggers, it hits the box. Depth updates inside the x=[52,54.5]
informative window: 3-4 per run in every case, consistent with Step
BH's ~1Hz rate over a ~1.9s window at 0.8 m/s -- there was physically
very little data for the controller to react to either way.

**Step BM (flow-only, all 5 zone_C runs)**: same check, same result.
y at closest approach: -1.42, -1.23, -1.39, -0.88, -1.46 -- every one
comfortably outside the box's half-width, every one large well before
reaching x=54.5. This is expected independently of any mechanism check,
since CheapStage already measured 0% valid centre-sector readings on
this box (Step BE) -- it cannot be steering around something it cannot
see at all. The handful of "avoid" flags in these runs' windows are not
centred on the box's approach and don't correspond to any sustained
centre-sector response.

**Verdict: neither arm has demonstrated seeing this obstacle in closed
loop.** Every pass in both arms is fully explained by the seeded
y-jitter already clearing the box's 0.5m half-width -- not by any
detected-then-avoided sequence. Per Step BH's own honest framing
("whether that's enough margin... is exactly what Step BF/BI tests" --
now answered: it hasn't been tested at all, because none of the sampled
starts required it to be), this batch cannot distinguish "HeavyStage
can avoid this box" from "HeavyStage was never asked to."

**Not proceeding further on this seed set -- per the task, more runs
here would not resolve this.** The confound is the start-position
sampling itself: `x~Uniform(-5,0), y~Uniform(-1.5,1.5)` was designed for
the ORIGINAL 16m wall, where no y-jitter in that range could possibly
miss it -- every run was necessarily a real test. Against a 1.0m box,
the same jitter range makes "missed it" the default outcome and
"tested against it" the exception (only run 5 out of 5 heavy runs
actually was on a collision course). Fixing this needs seed positions
constructed so the vehicle's approach is on (or very near) a collision
course with the box specifically -- y-jitter should be small enough
that reaching x=55 without ANY course correction would collide, so that
clearing it can only mean the controller actually changed course. Not
implemented yet -- flagged for the next step, deliberately, rather than
picked unilaterally given it changes the evaluation methodology itself.

## Step BN -- collision-course start positions, computed not guessed

`ZONE_COLLISION_JITTER_Y_M` (`runtime/orchestrator.py`, next to
`ZONE_SEGMENTS`): `j = 0.5 * (obstacle_half_y + r_air)` -- half the true
"any straight-line y in this range physically contacts the
obstacle+airframe" limit, so every drawn start keeps real margin below
the absolute worst case rather than sitting right at it. `r_air` is
`COLLISION_RADIUS_M` (0.35m) -- deliberately NOT `cheap.py`'s
`CONE_R_SAFE_M` (0.6m), which is a soft planning/awareness margin, not
a contact radius; `COLLISION_RADIUS_M` is the actual airframe radius
this project's own ground-truth ​collision detector already uses, so
reusing it keeps "collision course" meaning the same thing here as it
does everywhere else in this codebase.

Computed per zone from that zone's own CANONICAL CENTRELINE positive
case (the one obstacle each zone's own code comments already single out
as the "gate should fire" test for that zone), not every obstacle in
the zone:

| zone | canonical obstacle | half_y (m) | j (m) |
|---|---|---|---|
| zone_B | wallB1 (the new box, Step BG) | 0.5 | **0.425** |
| zone_C | boxC2 | 0.75 | **0.550** |
| zone_D | narrowD1 | 1.75 | **1.050** |

Zone A intentionally has no entry -- its trees/box are deliberately
off-centreline clutter (a different test: general avoidance amid
obstacles the vehicle isn't aimed at), not a single centreline target
this "guaranteed collision course" framing applies to.

**VERIFY (Step BN's own requirement, done before flying anything)**:
generated the 10 Zone B seeds (`random.Random(42)`,
`y~Uniform(-0.425,+0.425)`) and checked each against the true
worst-case limit (`obstacle_half_y + r_air = 0.85m`):

| run | y_start | \|y\| | margin below 0.85m limit |
|---|---|---|---|
| 0 | +0.1185 | 0.1185 | 0.7315 |
| 1 | -0.4037 | 0.4037 | 0.4463 |
| 2 | -0.1912 | 0.1912 | 0.6588 |
| 3 | -0.2353 | 0.2353 | 0.6147 |
| 4 | +0.2010 | 0.2010 | 0.6490 |
| 5 | +0.1502 | 0.1502 | 0.6998 |
| 6 | +0.3334 | 0.3334 | 0.5166 |
| 7 | -0.3511 | 0.3511 | 0.4989 |
| 8 | -0.0664 | 0.0664 | 0.7836 |
| 9 | -0.3997 | 0.3997 | 0.4503 |

All 10 intersect by construction (every \|y_start\| < 0.425 < 0.85);
none are borderline. `start_x=45.0, goal_x_m=65.0` (Zone B's existing
`ZONE_SEGMENTS` entry) for all 10 -- x doesn't affect whether a
straight-line trajectory would hit, only y does, so x was left fixed
rather than re-deriving a second jitter dimension that wouldn't change
the test.

Step BO (paired 10-run heavy-only + flow-only re-run on these seeds) is
next -- not started yet. It's 20 sim launches total (double Step BI's
already-paused batch), so flagged for the user before launching rather
than started automatically, given today's load already reached 21.77
once.

## Step BP-BS -- HeavyStage aggregation, tested before flying anything. Neither fix helped.

All offline, no sim launches, per the task.

**Step BP -- exactly how heavy.py aggregates (code-verified, not guessed)**:
`nearest_raw = np.percentile(raw[:, cols], 80)` -- the 80th percentile
of raw (relative-inverse) depth over the sector's FULL column range,
every row, not just rows the near object occupies. Not a mean, contrary
to this step's own opening hypothesis -- it was already a
"nearest-biased" percentile (Step AI's same reasoning, applied to
heavy). The real question was whether 80 is aggressive enough.

Box pixel-AREA fraction of the centre sector, computed from geometry
alone (box 1.0m wide x 1.5m tall, fx=fy=205.4696, camera and box both
at the drone's cruise altitude):

| range | horiz frac (5 sec) | vert frac | AREA frac (5 sec) | AREA frac (11 sec) |
|---|---|---|---|---|
| 4m | 40.1% | 16.1% | **6.44%** | 14.17% |
| 3m | 53.5% | 21.4% | **11.45%** | 21.40% |
| 2m | 80.3% | 32.1% | **25.77%** | 32.10% |

At 5 sectors, the box doesn't reach the 80th-percentile threshold's own
top-20%-of-pixels bar until the area fraction crosses 20%, which
geometry alone puts at **d~2.27m** -- close to Step BH's observed ~1.5m
onset, and both are far too close-in to give a controller at 0.8 m/s
useful lead time (Step BK/BL). At 90th percentile (top 10%), that
crossing moves out to **d~3.21m** (5 sectors) or **d~4.76m** (11
sectors) -- the aggregation-only fix this step set out to test.

**Step BQ -- alternative percentile, kept as a flag.**
`NEAREST_PERCENTILE_V2 = 90` added alongside the unchanged
`NEAREST_PERCENTILE = 80` default -- `nearest_percentile` was already a
`HeavyStage` constructor parameter before this step, so no new flag
mechanism was needed, just a named alternative instead of an unlabelled
number at the call site (mirrors `cheap.py` keeping "divergence"
alongside "percentile", Step AI). One value tested, not a sweep, per
the task.

**Step BR -- sector count, geometry.py's shared constant.**
`geometry.N_SECTORS`: 5 -> 11. Both `CheapStage` and `HeavyStage`
already default their own `n_sectors` from this constant, so the change
applies to both automatically -- no special-casing needed.
`control/sector.py`'s `SectorControllerConfig.n_sectors` is a SEPARATE
hardcoded `5`, not wired to `geometry.N_SECTORS` -- left alone here,
flagged as a known inconsistency, since this task is scoped to offline
perception analysis, not the closed-loop controller. All 31 tests still
pass (they construct sector counts explicitly, not via the module
default).

**Step BS -- offline verification, 4 configs, one DA-V2 pass per frame
(all four aggregations computed cheaply from the same raw depth map).**
Run against `boxB1_approach` -- **not** the task's named
`wallB1_close_approach`, which predates Step BG and shows the OLD,
now-removed 16m wall; substituted the correct current-box recording and
flagging the substitution explicitly rather than silently analyzing the
wrong obstacle. `BI run5`'s log could NOT be reanalyzed -- live
closed-loop runs never save raw camera frames, only the
already-computed `SectorBelief` scores, so there is no raw depth map to
re-aggregate; its known baseline-config behavior (score never dropped,
avoid never triggered, Step BL) is cited as context only, not
recomputed.

| config | corr(raw,dist) | corr(score,dist) | onset (mean score starts a sustained fall) |
|---|---|---|---|
| (a) 5 sectors, p80 (baseline) | +0.084, n.s. | **+0.356**, p<0.0001 | ~1.5m (0.315 -> 0.123 between the [1.5,2.0) and [1.0,1.5) bins) |
| (b) 5 sectors, p90 | **+0.603**, p<0.0001 | -0.108, n.s. | none -- noisy throughout, no sustained trend at any range |
| (c) 11 sectors, p80 | +0.088, n.s. | +0.352, p<0.0001 | ~1.5m -- nearly identical to (a) |
| (d) 11 sectors, p90 | **+0.604**, p<0.0001 | -0.110, n.s. | none -- nearly identical to (b), still noisy throughout |

**Neither intervention delivered the hoped-for improvement. Reporting
this plainly, exactly as the task asked, not tuned to look better.**

1. **Sector count made essentially no difference.** (a) vs (c) and (b)
   vs (d) are near-identical bin-by-bin. The geometric prediction said
   11 sectors should push the 80th-percentile onset out to ~3.21m
   (since an 11-way sector saturates to 100% horizontal box coverage
   at a longer range than a 5-way one) -- that did not happen
   empirically. The likely explanation: DA-V2 Small's OWN effective
   spatial resolution (patch-based ViT internals, upsampled to the
   full output grid) is probably coarser than an 11-way sector's ~58px
   width, so subdividing further doesn't expose genuinely independent
   signal -- it just re-slices an already-smoothed field. Not proven
   here (would need inspecting the model's actual patch resolution to
   confirm directly), but it is the explanation consistent with both
   the geometric prediction failing to materialize and the two
   percentile pairs behaving almost identically regardless of sector
   count.

2. **90th percentile is WORSE than 80th, not better -- wrong-signed,
   not just weaker.** This is the inconvenient result the task's own
   framing hoped wouldn't happen. `corr(raw,dist)` at p90 is much
   STRONGER than at p80 (0.60 vs 0.08) but the WRONG sign -- raw is
   often HIGHER at 7m than at 1.5m (e.g. d=6.98m: raw_p90=5.88; d=1.52m:
   raw_p90=4.17 -- checked directly against the logged rows, not just
   the correlation coefficient). The resulting score is correspondingly
   wrong-signed too (r=-0.108), worse than baseline's correctly-signed
   +0.356. Mechanism not fully diagnosed here, but the top-10%-closest
   tail is a smaller, noisier statistic than the top-20% -- more
   exposed to whatever occasional high-raw artifact (background
   texture/lighting the model misreads, not necessarily the box) shows
   up anywhere in the sector's full column, at exactly the far-range
   frames where the box itself is too small a fraction to dominate
   either percentile.

**Answering Step BS's actual question**: Step BH's flat-from-7m-to-1.5m
signal looks like it is DA-V2's real limit on this object, at THESE two
aggregation choices, not simply an artifact of averaging or coarse
sectoring -- finer sectoring didn't move the onset, and a more
aggressive percentile made the signal actively worse, not better. This
doesn't rule out every possible aggregation fix (e.g. restricting
sector_pixels to a row band matching the box's expected vertical
extent, rather than the full column, was not tested here and remains a
real, untried option), but the two specific, well-motivated
interventions this task asked for both failed to help. `heavy.py` was
NOT changed to adopt either -- `NEAREST_PERCENTILE` stays 80,
`geometry.N_SECTORS` stays at the new 11 (Step BR's change itself is
harmless/neutral per this result, so left in place, but percentile was
not bumped to 90 given it measurably hurts).

## Step BT -- required Zone B obstacle size, computed before touching the SDF

**Not fixable in heavy.py -- Step BP-BS already showed both tested
aggregation fixes fail. Working backwards from measured constraints to
size a new obstacle instead.**

**Width, computed as asked**: minimum width for the centre sector's
horizontal occupancy to fully saturate (box_px_w >= sector_w, N=11,
`sector_w=58.18px`, `fx=205.4696`):

```
min_width(d) = sector_w * d / fx
min_width(4m) = 58.18 * 4 / 205.4696 = 1.1327 m  (16.12deg angular width)
min_width(3m) = 58.18 * 3 / 205.4696 = 0.8495 m  (same 16.12deg -- angular
                                                    width is range-independent
                                                    by construction)
```

**Important correction to the task's own framing -- the failed 1.0m box
was NOT actually width-limited.** Re-checked its own numbers at N=11
(the current deployed default): `horiz_frac(4m)=88.3%`, `horiz_frac(3m)
=100%` -- both already close to or at saturation. The real bottleneck
was HEIGHT: `vert_frac(4m)=16.1%`, `vert_frac(3m)=21.4%` -- because
`sector_pixels = raw[:, cols]` aggregates the FULL image height
regardless of how tall the obstacle actually is (Step BP), a 1.5m-tall
box at 3-4m only occupies a sixth to a fifth of the column's rows no
matter how wide it is. Sizing the new obstacle from width alone, as
literally asked, would reproduce the same failure. **Minimum height**
(given width is saturated) for area_frac to clear the 20% (80th
percentile) threshold:

```
min_height(d) = 0.20 * height_px * d / fy
min_height(4m) = 0.20 * 480 * 4 / 205.4696 = 1.8689 m
min_height(3m) = 0.20 * 480 * 3 / 205.4696 = 1.4017 m
```

**Both dimensions matter; sizing to width alone would not have worked.**
Chosen design (Step BU, with real margin above both minimums, not
sized right at the threshold): **1.2m wide x 2.5m tall**, giving
horiz_frac=100% (fully saturated) at every relevant range and
area_frac=26.75% at 4m (vs the 20% bar), 35.67% at 3m, 53.51% at 2m.

**Superseded by Step BW below before ever being flown as a batch** --
the root cause identified at Step BW made this whole resize's premise
(percentile-of-pixel-fraction aggregation) moot, so it was reverted
back to the 1.0m x 1.5m box (Step BG) rather than tested. Not deleted
from the record -- if the metric-checkpoint path fails, this reasoning
is still valid and available to reapply.

## Step BW-BZ -- switch HeavyStage to the METRIC checkpoint

**Root cause reframed**: every HeavyStage failure across Steps AY-BU
was a normalization problem downstream of ONE choice -- the RELATIVE-
depth checkpoint has no fixed scale, so every fix invented was a new
way to normalize an inherently unscaled signal, and each either
collapsed (obstacle fills the frame, no local contrast left) or needed
an implausible pixel-fraction to reach the obstacle at all (Step BP).
Depth Anything V2 ships METRIC checkpoints (fine-tuned on real depth
labels, output directly in metres) as an alternative -- tested here
instead of continuing to redesign the world around the relative
checkpoint's limits.

**Step BW -- checkpoint swap.** The task's named model ID
(`depth-anything/Depth-Anything-V2-Metric-VKITTI-Small`, no `-hf` suffix)
404s -- that's the ORIGINAL, non-transformers checkpoint format. The
transformers-pipeline-compatible port of the same VKITTI-trained
outdoor model is `depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf`
(confirmed via the HF Hub API's own model listing, not guessed) --
`depth_estimation_type: "metric"`, `max_depth: 80`, matching the task's
own description. All per-frame min-max normalization, the Step BC
rolling window, and the Step AZ calibration-fit code are deleted from
`heavy.py` entirely -- none of it applies to a checkpoint that already
outputs metres.

**Sign convention verified explicitly on 3 frames before writing any
code**, per the task's own requirement: at true distance 6.98m, centre
raw depth ~8.6-77m; at true distance 0.32m (just before collision),
centre raw depth ~2.8-3.8m. Unambiguously LARGER=FARTHER -- the
opposite of the old relative checkpoint. `NEAREST_PERCENTILE` changed
from 80 to **20** to preserve the SAME "top 20% closest pixels"
semantic under the flipped sign (the task said "keep p80... as they
are" -- read as keeping the AGGREGATION STRATEGY, not the literal
number 80, since applying 80 unchanged under the new sign would
silently aggregate the FARTHEST 20% of a sector as the "nearest"
reading, exactly the kind of sign bug this file's docstrings have
warned about since Step 0b). `geometry.N_SECTORS` stays 11 (Step BR),
untouched. Score is now a direct distance cap (`score =
min(Z/Z_CAP_M, 1.0)`, `Z_CAP_M=20.0m`, a first-cut, not empirically
tuned, choice) -- no danger/1-danger flip needed any more since metric
Z is already "higher=safer" by construction.

**Step BX -- metric accuracy, measured directly (no correlation
proxies), on `boxB1_approach`** (n=165, the current 1.0m x 1.5m box):

| metric | value |
|---|---|
| corr(predicted_z, true_dist) | **+0.722**, p<0.0001 |
| corr(score, true_dist) | +0.722, p<0.0001 (same, score is a linear function of z below the cap) |
| abs error @ 4m | predicted 10.15m -- **6.16m error** |
| abs error @ 3m | predicted 9.96m -- **6.96m error** |
| abs error @ 2m | predicted 9.67m -- **7.66m error** |
| MAE, all samples | **4.66m** |

**Genuinely mixed result -- reporting both halves plainly, not picking
the flattering one.** The correlation (0.722) is by far the strongest
distance relationship any HeavyStage variant has produced in this
project -- roughly double Step BE's best relative-checkpoint number
(0.294-0.397). The mean-score-by-distance-bin table shows a real,
visibly sustained decline starting around **2.0-2.5m** (0.484 -> 0.405
-> 0.243 -> 0.166 as distance falls from 2.5m to 0.5m), cleaner than
anything the relative checkpoint produced -- but that onset is short of
the "falls monotonically from 4m+" the task's own Step BX question was
checking for, and short of the 3m+ bar this whole redesign effort has
used throughout.

**Absolute accuracy is poor, and not by a fixable constant offset.**
Predicted values are consistently 2.5-4.8x the true distance, and the
RATIO itself grows as the object gets closer (2.54x at 4m, 3.31x at
3m, 4.82x at 2m) rather than staying fixed -- not a simple linear
scale/shift error, more consistent with a real cross-domain gap
(VKITTI is real highway/street driving footage; this is a synthetic
Gazebo corridor with a flat grey box at close range, well outside what
the checkpoint saw in training). "Metric" describes the OUTPUT UNITS by
construction, not calibrated accuracy for this specific sim -- exactly
the caveat flagged in `heavy.py`'s own new docstring before this number
was measured, not added after the fact to explain it away.

**Step BY -- metric geometry restored, scoped.** Added
`geometry.miss_distance_from_z(x_norm, y_norm, z) = z *
sqrt(x_norm^2+y_norm^2)` -- the direct route to a miss distance from a
known metric Z, versus `miss_distance()`'s tau/vz route for CheapStage
(which has no direct Z, only a flow-derived time-to-contact). Both feed
the same shared `cone_weight()` -- that is what "shared, not
reimplemented" means now that the two stages measure genuinely
different things (time vs. distance) rather than one of them lacking
any usable quantity at all. `geometry.py`'s module docstring updated to
remove the old CheapStage-only asymmetry note. **Not done in this
pass**: wiring per-pixel cone-weighted miss into `HeavyStage.infer()`
itself -- CheapStage's cone-weighting operates at the per-TRACKED-POINT
level (its own `point_cone_w`/`cone_min_tau` machinery); HeavyStage's
aggregation is per-SECTOR (a single percentile value), and rearchitecting
it to point-level cone-weighting is a real, separate change, not
attempted here given Step BX's mixed result makes it unclear whether
this checkpoint is worth that investment yet. The shared capability
exists and is available; it is not yet exercised by HeavyStage's own
sector aggregation.

**Step BZ -- latency, isolated, `boxB1_approach`, n=15**: mean
**1255.9ms**, median 1246.9ms -- FASTER than the old relative Small
checkpoint's 1541ms isolated baseline (Step AU), not slower. A genuine,
unambiguous improvement on this axis regardless of how the accuracy
question above gets resolved.

**Not proceeding to the re-run batch yet.** The task gated it on "only
if BX passes" -- BX's own result is mixed enough (strong correlation,
poor absolute accuracy, onset short of 3-4m) that calling it a clean
pass or fail is a judgement call, not an automatic one, so it's
reported here rather than decided unilaterally. All 31 tests pass; sim
was launched only for the one Step BW/BX verification recording, torn
down and confirmed clean afterward.

## Step CE -- 5-run batch on the metric checkpoint. Collides again, and a deeper mechanism found. STOPPING here.

BX was judged a pass (ranking quality is what the controller consumes,
not metres; +0.722 far exceeds any relative-checkpoint number). 3 of
the planned 5 runs completed with identical outcomes -- stopped early
per the task's own instruction rather than running the remaining 2.

| run | y_start | result | frames | avoid frames | switches | target_sector used |
|---|---|---|---|---|---|---|
| 0 | +0.104 | collided zone_B | 15 | 15/15 | 1 | {4} only |
| 1 | -0.411 | collided zone_B | 15 | 15/15 | 1 | {4} only |
| 2 | -0.198 | collided zone_B | 15 | 15/15 | 1 | {4} only |

**3/3 collide. ACCEPTANCE (clears the box by course change, majority
of runs) is not met.** One run (`heavy_ce_run2`) had to be discarded
and redone after hitting the same DDS-connection hang seen twice
before in this session (`ros2 service call .../set_mode LAND` and the
main eval call both needed manual `kill -9` after their `timeout`
wrappers sent SIGTERM without the process actually exiting) --
environmental, not related to the metric checkpoint.

**Surface reading, and why it's not enough on its own.** Unlike Step
BI/BL's relative-checkpoint runs (score never dropped, "avoid" never
triggered, `switches=1`), THIS checkpoint puts the controller in
"avoid" mode for 100% of frames in all 3 runs -- a real, measurable
behavioural difference the correlation improvement predicts. But
`switches=1` still holds in every run, and `target_sector` is **locked
to sector 4 -- not the centre sector (5) -- from frame 0, at 10m out**,
before the box is anywhere near relevant. That is not a reaction to the
box; it is a decision made before there was anything box-specific to
react to.

**Mechanism, traced through the actual cost computation, not assumed:**
`goal_heading = -yaw` (`orchestrator.py`), not a fixed 0 -- so a small,
ordinary spawn-yaw misalignment breaks the exact bearing symmetry
between sectors 4 and 6 (+-15.82deg either side of centre, N_SECTORS=11).
Whichever of the two comes out marginally cheaper at frame 0 gets
picked; frame 0's raw scores actually favour sector 5 in every run
(e.g. run0: sector4=0.506, sector5=0.509 -- sector 5 is BETTER on score
alone), so this is the goal-heading term, not the perception score,
deciding the first pick. After that, `SectorController`'s hysteresis
(`hysteresis_margin=0.15`) requires a challenger to beat the current
target by that margin to take over. The fixed goal-term cost gap
between sector 4 and centre sector 5 is `w_goal * 15.82deg (rad) =
0.5*0.276 = 0.138` on its own -- already most of the 0.15 budget --
and this checkpoint's actual score spread across ALL sectors, at ANY
point before collision, stayed inside roughly **0.44-0.53** in every
run checked (confirmed directly in the logged `scores` arrays, not
inferred). No score difference that small can supply the remaining
~0.15+ combined margin needed to unseat an incumbent, so once sector 4
locked in at frame 0 it could not lose, structurally, regardless of
what the depth signal did for the rest of the approach. Depth updates
landing in the informative window (`true_dist` in [0, 2.5m]): 4 per
run in every case (matches Step BH's ~1Hz-over-the-window arithmetic),
consistent but moot given the target was already fixed before any of
them could matter.

**This is a genuine, new finding beyond "no signal reached the
controller"**: a real signal existed (Step BX's correlation, and this
batch's own universal "avoid" trigger both show it), but this
checkpoint's SCORE DYNAMIC RANGE is narrow enough, and the controller's
hysteresis/goal-term architecture rigid enough, that the two together
make target-switching close to structurally unreachable regardless of
what the obstacle does. Per the task's explicit instruction, the
controller was NOT changed to test whether loosening the hysteresis
margin or the goal-term weight would fix this -- that would be
tuning the controller to this specific finding, exactly what was ruled
out here.

**Stopping, per the task's explicit contingency.** Not resizing the
box again, not trying another checkpoint. The measured finding: even a
metric checkpoint with a real, substantially-improved (0.722 vs
~0.3-0.4) distance correlation could not translate into obstacle
avoidance in closed loop at this project's actual flight parameters
(0.8 m/s, ~1Hz CPU perception, this controller's hysteresis/goal-term
tuning) in a majority of trials -- 0 of 3, not a marginal miss. The
paper should frame around this as a measured limitation of monocular
depth (relative OR metric) as a standalone safety ceiling in this
deployment envelope, not as a normalization or checkpoint problem still
waiting on the right fix. `miss_distance_from_z()`'s own docstring
(Step BY, above) already documents why its output isn't trustworthy
enough to lean on either.

## Step CF/CG -- two controller bugs, one confirmed real, one not. Fixed the real one; documented the other honestly.

Re-reading Step CE: `switches=1` in every run despite avoid mode firing
on 100% of frames pointed at the controller, not perception -- a real
signal was reaching it and still couldn't move the needle.

**Step CF -- re-derived `goal_heading` independently, not assumed
broken.** The suspected bug was `goal_heading = -yaw`. Refactored it
into a named constant (`GOAL_WORLD_BEARING_RAD = 0.0`) and an
independently-testable function (`_goal_heading_body()`), then wrote
two unit tests that check the actual claim rather than assume it:

1. `test_goal_heading_body_matches_negative_yaw_for_zero_world_bearing`
   -- confirms the new function is numerically IDENTICAL to the old
   `-yaw` formula, for the corridor's actual goal direction (world
   bearing 0), across 6 yaw values.
2. `test_goal_heading_picks_sector_closest_to_true_world_corridor_direction`
   -- the semantic property Step CF's VERIFY actually asked for,
   checked two independent ways: which sector the goal-cost argmin
   would pick vs. which sector's WORLD-frame bearing (body bearing +
   yaw) is closest to the true corridor direction. Agrees at every
   tested yaw offset (0 to +-90deg).

**Both tests pass -- reporting plainly that `goal_heading = -yaw` was
mathematically correct already**, not claiming a math fix that isn't
there. `goal_world_bearing - yaw` IS `-yaw` when the world bearing is
0, which it always has been in this codebase. What Step CF actually
changed: the constant is now named and documented instead of folded
into a sign flip, and there are now regression tests that would catch
a future accidental sign error, which did not exist before. The
sector-4 lock in Step CE's data came from a real, ordinary, small
nonzero spawn yaw (not a bug) interacting with Step CG's actual root
cause below.

**Step CG -- confirmed real: HeavyStage's score spread is too narrow
for the controller's hysteresis to ever clear.** This IS the Step AP
comparability contract failing in practice: both stages are bounded
[0,1] by contract, but Step CE's own logged scores showed HeavyStage's
metric checkpoint compressing to ~0.44-0.53 in every frame checked,
while `SectorController`'s fixed `hysteresis_margin=0.15` plus the
fixed goal-term cost gap between adjacent sectors (~0.138 at this
project's sector spacing) is a bar no score difference that narrow can
ever clear -- structurally, regardless of what the obstacle does.

**Chose option (b)**: `geometry.normalize_scores_to_span(scores,
valid)` -- a per-frame, ACROSS-SECTORS min-max stretch, applied inside
BOTH `heavy.py` and `cheap.py`'s `infer()`, right before each
`SectorBelief` is returned. Deliberately across SECTORS within one
frame, not across TIME the way Step AY/BC's failed relative-checkpoint
normalization was -- `SectorController` compares scores across sectors
at one instant to pick a target, so that is the axis that needs
spread, and stretching within one frame cannot reintroduce the
temporal-collapse failure mode (an obstacle filling the view over
several consecutive frames) since no history is pooled at all. Chosen
over (a) (scaling the hysteresis margin itself) because (b) fixes the
underlying contract at its source -- the SAME narrow spread that broke
hysteresis also flattens the `w_tau*(1-score)` cost term everywhere
else scores are used (candidate ranking, `tau_crit_score` masking),
which (a) would have left untouched. Degenerate case (all valid scores
identical) returns unchanged, not stretched to an arbitrary spread --
a genuinely uniform reading is real information, not smoothed over.

**VERIFY, `boxB1_approach`, before vs after**:

| stage | n frames | PRE mean span | POST mean span |
|---|---|---|---|
| HEAVY | 20 sampled | 0.0992 | 1.0000 |
| CHEAP | 3 (all multi-valid-sector frames in this recording) | 0.0999 | 1.0000 |

POST is exactly 1.0 by construction whenever more than one sector is
valid with any difference at all -- that's what a min-max stretch does,
not a claim that the RESULT is somehow more informative than the
raw spread, only that it's now USABLE by a margin-based decision.
CheapStage's own n=3 here is too sparse to generalize from (it's
structurally near-blind on this specific untextured box, Step BE) --
noted honestly rather than treated as a real per-stage comparison; the
near-identical PRE spans (0.0992 vs 0.0999) between the two stages on
this recording is more likely coincidence at this sample size than a
real finding.

All 33 tests pass (2 new from Step CF). Re-running the closed-loop
batch next (Step CH) to see whether this actually changes anything.

## Step CH (partial) -- fixes deployed, collides again. Root cause is neither controller bug -- a wide-FOV artifact.

3 of the planned 5 heavy-only runs completed (BN seeds, same as Step
CE) before the batch was paused; all 3 collided at zone_B with
`switches=1` again -- **the CF/CG fixes did not change the outcome**:

| run | y_start | result | switches |
|---|---|---|---|
| 0 | +0.1185 | collided zone_B | 1 |
| 1 | -0.4037 | collided zone_B | 1 |
| 2 | -0.1912 | collided zone_B | 1 |

Confirmed both fixes were actually live in the installed package before
trusting this result (this project's install isn't symlinked, and a
stale install silently invalidated results once before, Step BC) --
`grep` on the installed `heavy.py`/`orchestrator.py` showed the new
code present.

**But the score data now shows something Step CG's fix could not have
addressed, because it isn't a spread problem any more.** Post-stretch
scores DO span the full [0,1] range now (confirmed, e.g. run0 frame0:
`[0.0, 0.741, 0.995, 0.975, 0.957, 0.969, 0.986, 1.0, 0.971, 0.403,
0.162]`) -- yet even in the LAST frame before collision (run0 frame13,
~0.43m from the box), the CENTRE sectors (looking directly at the box)
still score 0.94-0.97 (safe-looking), while the LOW scores are
concentrated in sectors 9-10 -- the far right edge, nowhere near the
box.

**Traced the mechanism using `boxB1_approach`'s saved images (the live
closed-loop runs have no saved frames to reprocess).** Extracted raw
per-sector metric Z at 6-7m out (nowhere near the box yet) across
several frames: sectors 0 and 10 (the far LEFT and RIGHT edges) are
**consistently the two lowest (closest-reading) sectors at every
sampled distance**, e.g. at 7.00m: `[8.64, 9.7, 10.2, 10.14, 10.1,
10.13, 10.2, 10.23, 10.18, 9.21, 8.56]` -- a "bowl" shape, edges low,
centre high, independent of the box entirely (this is 6-7m out, before
the box could be a meaningful factor). Checked WHERE within edge
sector 10 the low values concentrate, row-by-row: high (30-40m) in the
MIDDLE rows, but only 7-9m in the TOP and BOTTOM row-bands. The
edge sectors' full-column aggregation is being dragged down by
sky/ground rows visible at extreme viewing angles, not by any obstacle.

**Root cause: this project's camera has a genuinely wide FOV (114.6deg
HFOV, ~99deg VFOV, Step BG/BJ) -- almost certainly wider than whatever
camera produced the VKITTI training data this metric checkpoint was
fine-tuned on.** At the periphery of a FOV this wide, a column samples
extreme, oblique viewing angles top-to-bottom -- exactly the kind of
out-of-training-distribution geometry a model would be least reliable
on, and it reads them as closer than they are, consistently and
independent of scene content near the box. This is a genuine,
previously-uncharacterized domain-gap effect, more specific than Step
BX's already-documented "poor absolute accuracy" -- it isn't uniform
error, it's SPATIALLY STRUCTURED error concentrated at the image edges
regardless of what's actually there.

**Why Step CG's fix couldn't have helped, and made this specific
problem worse, not better.** `normalize_scores_to_span()` stretches
relative to the CURRENT frame's own min and max across sectors. If
sectors 9-10 are ALWAYS near the bottom of that range (a persistent
artifact, not a signal), they anchor the stretch's floor every single
frame -- burying the box's own, much weaker, centre-sector signal
inside the stretched range instead of amplifying it. The fix correctly
solved the PROBLEM IT TARGETED (narrow spread, Step CE) but the
narrowness was never really about a genuine box-detection signal being
compressed -- most of what the checkpoint reads as "sector-to-sector
variation" at this FOV is the edge artifact, not the box, and Step CG
made that artifact more decisive, not less.

**This traces back to an option flagged and left untried at Step BS**:
"restricting sector_pixels to a row band matching the box's expected
vertical extent, rather than the full column, was not tested here and
remains a real, untried option." That option would ALSO fix this,
independently of why it was originally proposed -- if the per-sector
aggregate excluded the extreme top/bottom rows (sky, near-ground) that
this wide FOV renders unreliable, sectors 9-10 would stop reading as
permanently-closest for reasons that have nothing to do with any
obstacle.

**Paused here, not resuming the batch or picking a fix unilaterally.**
Neither Step CF (confirmed not a real bug) nor Step CG (a real fix for
a real problem, just not THIS one) can address a spatially-structured
artifact from the camera's FOV interacting with the checkpoint's
training distribution. Runs 3-4 were not attempted -- rerunning them
against the same unaddressed artifact would very likely just reproduce
runs 0-2. Sim confirmed torn down and clean.

## Step CI/CJ/CK -- row-band aggregation. One half of ACCEPTANCE met decisively, the other only partially.

**Step CI -- characterised before implementing, all offline.** Horizon
row (level camera): `cy=240`, exactly. Box (1.5m tall, centred at
camera altitude) row-band by range: d=2m -> rows [163,317] (154px
tall), d=4m -> [201,279] (77px), d=6m -> [214,266] (51px) -- the band
SHRINKS with range, so the d=2m band (closest range of interest)
contains all the others and is the correct single fixed band to use.
Verified empirically on `boxB1_approach` with a per-sector, per-row
mean-Z profile (12 row-bands x 11 sectors, at 4 sampled distances):
rows below ~320 read NEARLY IDENTICAL Z across all 11 sectors (e.g. row
[440:480] at every one of 7m/4m/2m/~1m stayed within 6.7-7.2m,
regardless of the vehicle's actual distance to the box) -- a ground-
plane geometric signature, not scene content, confirming the diagnosis
before writing any aggregation code.

**Step CJ -- implemented, derived not hand-picked.** `row_lo`/`row_hi`
computed in `HeavyStage.__init__` from `fy`, `cy`,
`OBSTACLE_HALF_HEIGHT_M=0.75`, `ROW_BAND_MIN_RANGE_M=2.0` -- resolves to
rows [162, 318] for this camera, matching the Step CI derivation
exactly. Applied to HeavyStage only, per the task -- CheapStage
aggregates tracked points, not raw pixels, and already has its own
`VIGNETTE_Y_MIN` exclusion for a different reason (rotor blur, not
sky/ground). All 33 tests still pass.

**Step CK -- verified on `boxB1_approach`, full-column vs row-band**:

| | FULL-COLUMN (old) | ROW-BAND (new) |
|---|---|---|
| corr(centre raw Z, dist) | 0.722 | **0.877** |
| corr(centre stretched score, dist) | 0.569 | **0.681** |
| edge-sector-is-argmin ("bowl") fraction | 86.1% | **66.1%** |
| decline onset | ~2.0m, and not clean (score still RISES 3.5m->2.0m: 0.795->0.856->0.964->0.981) | **~3.5-4.0m, clean and monotonic** |

Row-band mean centre score by distance bin (far -> near): 1.000 held
flat from 6.5m to 4.0m, then a clean, monotonic, sustained decline --
0.979 (3.5-4.0m) -> 0.855 -> 0.613 -> 0.488 -> 0.282 -> 0.147 -> 0.009
(0.5-1.0m) -- before the already-established <0.5m breakdown noise
(0.286). This is a dramatically different shape from the old
full-column result, which stayed flat or even ROSE through the same
range before finally dropping much later (~2.0m).

**ACCEPTANCE was stated as an AND: bowl disappears AND monotonic
decline from 3m+. Only the second half is unambiguously true.** The
decline is not just "from 3m" -- it starts closer to 4m, comfortably
past the bar. The bowl fraction improved substantially (86.1% ->
66.1%) but did NOT disappear -- it's still the majority of frames.
Plausible honest explanation, not confirmed: even within the row band,
a wide-FOV edge sector may be looking at genuinely closer scene
geometry (corridor boundary/perspective) than the open centre path --
a real 3D-geometry effect the row band cannot remove, distinct from
the sky/ground artifact it WAS built to remove and did remove (the
now-flat, stable ground-row readings confirm that part worked).

**Judgement call, not decided unilaterally**: the criterion that
actually mattered for Step CE/CH's failure (does the centre sector's
own score ever drop enough to matter) is now met dramatically -- the
persisting bowl is a separate, secondary question whose practical
effect on the controller (goal-term cost may already discount a
transient edge-sector reading enough not to matter) is exactly what a
closed-loop test would show, not something more offline analysis can
resolve further. Flagged to the user before spending more sim time on
Step CL rather than deciding either way.

## Step CL -- 5/5 collide again. A fourth, different, and more fundamental mechanism found: the vehicle spins itself away from its own camera's view.

Full 5-run heavy-only batch, BN seeds, row-band fix (Step CJ) live and
confirmed present in the installed package before trusting the result
(checked after this session's machine rebooted mid-pause). All 5
collided at zone_B, `switches=1` in every run -- **the row-band fix,
despite its strong offline improvement (Step CK), did not change the
live outcome either.**

**This time the mechanism is neither perception (Step CE/CI diagnoses)
nor score-scale (Step CG) -- it's the vehicle's own attitude dynamics.**
Pulled `vehicle_velocity` (not just position) across all 5 runs and
computed the heading each frame implies (`atan2(vy, vx)`). Every run
shows the SAME systematic, near-linear rotation, remarkably consistent
run to run:

| frame | run0 | run1 | run2 | run3 | run4 |
|---|---|---|---|---|---|
| ~3 | -39.2deg | -41.3deg | -38.2deg | -33.3deg | -38.8deg |
| ~6 | -88.0deg | -88.7deg | -88.4deg | -84.0deg | -88.9deg |
| ~9 | -132.0deg | -131.6deg | -135.5deg | -122.3deg | -130.3deg |

**The vehicle is spinning, not translating.** `target_sector=4` locks
at frame 0 (Step CE's finding, unaddressed by CF/CG/CI since none of
them changed WHY the initial pick sticks, only whether score data is
usable) and NEVER changes (`switches=1`), so `SectorController` issues
the SAME body-frame yaw-rate command, unmodified, frame after frame,
for the ENTIRE ~14s approach. With no re-evaluation ever correcting it,
the commanded turn just keeps compounding: by frame 9 (still ~4m from
the box) the vehicle's actual velocity direction has rotated past
-130deg -- it is flying nearly SIDEWAYS-TO-BACKWARD relative to where
it started, not toward the box at all any more in body-frame terms.

**This directly explains Step CL's own anomalous score data.** Run0
frames 9-13 (x=51.2 to 54.2, i.e. 0.8-3.8m from the box) show EVERY
sector reading score=1.0 -- not because the box became safe, but
because by then the CAMERA has rotated far enough that it is no longer
looking anywhere near the box; it is looking at open space in whatever
direction the accumulated spin has pointed it, which genuinely reads
as "far" because it IS far in that direction. The row-band fix (Step
CJ) is doing exactly what it was built to do -- the camera just isn't
pointed at the danger any more by the time the fixed perception would
matter.

**Why the vehicle's POSITION still reaches the box despite the camera
spinning away**: `x` keeps increasing toward 55 across all 5 runs
regardless of the rotating commanded heading -- consistent with real
flight dynamics lag/momentum (the vehicle cannot instantly redirect its
actual world-frame velocity just because the commanded body-frame
velocity direction changed), so the ACTUAL trajectory is a damped,
lagged response to a continuously-changing command, not a clean
redirection. The net effect: perception stays informative for a few
frames (Step CL's frames 4-7 DO show real sector differentiation before
the spin dominates), but the vehicle's physical inertia carries it into
the collision anyway because the yaw command was never allowed to
settle on a stable heading long enough to actually redirect the
trajectory.

**Not a perception problem, and not something Steps CI/CJ/CK could
have fixed -- flagged, not addressed unilaterally.** This is a
controller/vehicle-dynamics interaction: an unswitchable sticky target
(the original Step CE finding) produces an UNCORRECTED, compounding yaw
command with no re-evaluation, which the vehicle's own inertia cannot
track cleanly, decoupling what the camera sees from where the vehicle
is actually going. Fixing THIS is a controller change (e.g. re-evaluate
target more aggressively, cap cumulative yaw per unit time, or address
why the target never re-evaluates even once real sector differentiation
existed in frames 4-7) -- explicitly out of this task's scope
("Do not change the controller" was the standing instruction through
Step CE; Step CF/CG were narrowly scoped fixes, not a general licence).
Not picked or implemented here. Sim confirmed torn down and clean after
all 5 runs. Flow-only batch NOT started -- re-running it against a
heavy-only result this inconclusive would not usefully inform the
paired comparison yet.

## Step CM/CN -- the controller fix for Step CL's spin, a third bug found while verifying it, and an honest offline replay

**Every closed-loop collision-rate/switch-count number produced before this
section -- Step AS's flow-only baseline, the 0.8 m/s re-run, Zone C/D
segment results, Steps CE, CH, and CL -- was produced under the bugs
fixed here and is SUPERSEDED.** The qualitative finding "CheapStage
cannot see the untextured box" (Zone B, CheapStage's own valid-sector
blindness) still stands -- it's about CheapStage's perception output,
unrelated to yaw control -- but every quantitative collision-rate,
switch-count, or heading number from before this fix must be treated as
invalid until re-run under the controller described below (Step CO).

### Bug 1 -- yaw_rate was an open-loop command, never recomputed

Step CL's own diagnosis (previous section) stopped at "the target
sector never re-evaluates." The deeper bug: `_sector_bearing(i, cfg)`
is BODY-FRAME-FIXED -- constant per sector index, since sectors are
slices of the CURRENT camera image, not stable world directions. So
`yaw_rate = k_yaw * _sector_bearing(target, cfg)` was IDENTICAL every
frame once a target locked, regardless of how much the vehicle had
already turned -- an open-loop rotation command that never shrank
toward zero, reissued verbatim frame after frame. This alone explains
Step CL's compounding heading (-38 -> -88 -> -132deg by frames 3/6/9):
the command never responded to its own effect.

### Bug 2 -- hysteresis compared a WORLD direction against a rotating-frame INDEX

Sector 4 at frame 0 and sector 4 at frame 9 are different world
directions once the camera has rotated between them -- sectors are
fixed slices of the CURRENT image, not stable references. Latching a
sector INDEX in `ControlState` and comparing it frame-to-frame was
therefore comparing against a moving target that happened to share a
label -- the actual cause of `switches=1` in every closed-loop run
through Step CL: the "held" index stopped meaning anything the moment
the vehicle yawed even slightly.

### The fix

`ControlState.target_sector: Optional[int]` -> `ControlState.
target_world_bearing: Optional[float]` (rad, world-frame ENU, same
convention as everything else in `control/`). `SectorController.step()`
gained a required `yaw: float` parameter -- the vehicle's current
world-frame yaw, the one piece of genuinely world-frame information the
controller accepts, needed only to project a held bearing into the
current camera frame, never to know about the corridor or any obstacle
specifically. Each step: `theta_current = wrap_to_pi(target_world_bearing
- yaw)` gives the held bearing's CURRENT body-frame position, which
`_nearest_sector_for_bearing()` maps back to a sector index for
candidacy/hysteresis checks; `yaw_rate = k_yaw * theta_current` is
computed FRESH every step from that projection, so it naturally shrinks
toward zero as the vehicle turns toward it, instead of reissuing a
constant command. `target_world_bearing` is only re-derived from a
sector's current body bearing on a GENUINE new commitment (bypass-adopt
on no-prior-target/source-switch/candidate-lost, or a hysteresis-
debounced switch actually completing); otherwise it carries through
unchanged frame to frame -- that unchanged carry-through, combined with
projecting it fresh each step, IS the convergence fix.

VERIFIED with two unit tests (`test/test_sector_controller.py`,
hysteresis neutralized in both to isolate the property under test):
- `test_yaw_rate_converges_as_vehicle_turns_toward_target` -- fixed
  belief, vehicle yaw swept toward the winning sector's own body
  bearing: commanded yaw_rate decreases monotonically and reaches
  ~0 exactly at alignment.
- `test_committed_world_bearing_stable_while_sector_index_drifts` --
  same fixed belief, yaw swept: `target_world_bearing` stays IDENTICAL
  across every step (to 1e-9) while the REPORTED sector index (which
  sector currently contains that bearing) visibly drifts from 0 to 2 as
  the camera rotates under it -- proof the two concepts (a stable
  committed direction vs. a rotating-frame label for it) were
  conflated before and are correctly separated now.

**A known, deliberately out-of-scope edge case, documented not fixed:**
cost computation (`w_tau*(1-score) + w_goal*|theta-goal|`) is
yaw-independent, so mid-convergence the sector the held bearing
currently projects into can diverge from `best_candidate`'s own sector
before true alignment is reached. If the score/goal gap between them
ever clears `hysteresis_margin` for `hysteresis_min_frames` in a row
during that window, the controller will re-anchor to a moving target
rather than finishing the original turn. Step CG's per-frame score
stretch can produce gaps large enough for this. Not exercised by either
unit test above (both neutralize hysteresis specifically to avoid it);
flagged for whoever next touches hysteresis timing, not fixed here
since it's outside Bug 1/2's stated scope.

### A third bug, found while preparing Step CN's replay -- `n_sectors` never matched the real sector count

`SectorControllerConfig.n_sectors` defaults to 5. Neither
`run_heavy_only_eval.py` nor `run_flow_only_eval.py` ever overrode it,
while both pass an 11-element `sector_bearings_rad` (from
`geometry.sector_bearings_rad()`, `N_SECTORS=11` since Step BR) and
both perception stages emit 11-element `scores`/`valid` arrays.
`candidates = [i for i in range(cfg.n_sectors) if belief.valid[i]]` and
`_nearest_sector_for_bearing()` both index by `cfg.n_sectors`, not
`len(belief.scores)` or `len(sector_bearings_rad)` -- so candidate/
target selection has silently been restricted to sectors 0-4 (the LEFT
HALF of the frame) in **every closed-loop run in this project's
history**, regardless of what score sectors 5-10 (right half) reported.
Confirmed empirically: `target_sector` never exceeds 4 across any
frame of any logged run (`heavy_cl_run0-4_frames.jsonl`, all five).

Fixed by deriving `n_sectors` from `len(bearings)` at both call sites
(`tools/run_heavy_only_eval.py`, `tools/run_flow_only_eval.py`) instead
of relying on the dataclass default, and adding a `__post_init__` guard
to `SectorControllerConfig` itself that raises if `sector_bearings_rad`
is supplied with a length that doesn't match `n_sectors` -- so this
specific mismatch cannot be silently reintroduced by any future call
site. All 35 tests (33 prior + Bug 1/2's two new ones) still pass with
the guard in place.

### Step CN -- offline replay against a real logged belief sequence

`tools/replay_cn_yaw_convergence.py` replays each `heavy_cl_run{0..4}
_frames.jsonl`'s BELIEF sequence (scores/valid/confidence/source, in
logged order and spacing) through the fixed controller, with vehicle
yaw SELF-INTEGRATED from the fixed controller's own `yaw_rate` output
(`yaw[i] = wrap_to_pi(yaw[i-1] + yaw_rate[i-1]*dt)`, `yaw[0]=0`) rather
than read from the log's `vehicle_velocity`. That choice matters and is
documented in the script's own docstring: an earlier version derived
yaw from the LOGGED velocity (`atan2(vy,vx)`, the same proxy the CL
diagnosis above used) and fed that into the fixed controller -- but
that trajectory was produced by the BROKEN controller spinning to
-177deg and beyond, so feeding those already-diverged values into the
fixed controller just showed it correctly applying more correction as
an externally-imposed, ever-growing error grew, saturating at
`yaw_rate_max` -- not converging, because the input never approached
the target. Self-integrating yaw from the FIXED controller's own output
is the honest version of the question Step CN actually asks.

**Result, all 5 runs**: OLD (logged) `yaw_rate` is a single constant
value (0.2759 rad/s, `target_sector=4`) across all 15-16 frames of
every run -- exactly Step CL's finding, reproduced here as the
before-comparison. NEW (self-integrated): `yaw_rate=0.0` at every
single frame of every run, self-integrated yaw stays at 0.0deg
throughout, `target_sector=5` (the TRUE centre under the n_sectors=11
fix) the entire time.

**This is a genuine but undramatic pass, and the reason why is worth
stating precisely rather than presenting it as more than it is.**
Sector 5's score never drops low enough, for long enough, for any
challenger to clear `hysteresis_margin` against it in this specific
belief sequence -- checked directly: at run0 frame 5 (seq 607), sector
5's score dips to 0.48 and sector 10 reads 1.0 (cost 0.50 vs sector 5's
cost 0.52 at goal_heading=0), a real but only 0.02-wide gap, well under
the 0.15 margin, so hysteresis correctly suppresses the switch -- the
system is behaving exactly as designed, not stuck. But this replay
never exercises the case the two unit tests above specifically built
for: a LARGE initial error shrinking toward zero over several frames.
The reason is structural, not a limitation of the fix: these five
belief sequences were themselves recorded WHILE the vehicle was
spinning uncontrollably (Step CL's bug), so what the camera saw at each
timestamp is an artifact of erratic rotation, not a coherent view of
one obstacle being approached under stable heading -- reusing that
sequence with a corrected yaw trajectory answers "does the fixed
controller avoid runaway/instability on real recorded perception data"
(yes, cleanly) but cannot exercise "does it converge from a large
error," because in this specific data the centre sector was already
optimal at every frame that was ever recorded. The unit tests remain
the load-bearing verification for that property; this replay's job was
a real-data sanity check for absence of runaway, which it passes.

Net effect on what Step CL actually reported: OLD accumulated to past
-177deg by frame 12-13 of every run; NEW never leaves 0.0deg. Whatever
Step CO's actual flight paths turn out to be, they cannot exhibit
Step CL's continuous-spin failure mode, since that failure mode
required the exact open-loop mechanism removed here.

## Step CO -- the fixed controller flies straight, and still collides 5/5. A new, distinct root cause.

5 heavy-only + 5 flow-only runs, BN seeds, 0.8 m/s, Zone B, the SAME
seeds as Steps CE/CH/CL, now against the Step CM/CN-fixed controller
(world-bearing yaw convergence + hysteresis, `n_sectors=11`). Driver:
`tools/run_step_co_batch.sh` (one sim launch per run, headless, fully
torn down between runs). All processes confirmed torn down cleanly
after the batch; no DDS hangs this time.

**Headline: the fix works exactly as designed -- heading settles, and
switches are no longer stuck at 1 -- but collision rate is UNCHANGED,
5/5 heavy and 5/5 flow, both at zone_B, same as every batch before it.
The failure mechanism has changed. The outcome has not.**

### Heading now settles -- the fix is real

| run | max heading deviation from initial (excl. final/collision frame) | max lateral \|y-y0\| (m) | switches |
|---|---|---|---|
| heavy 0 | 0.7deg | 0.033 | 2 |
| heavy 1 | 0.6deg | 0.034 | 2 |
| heavy 2 | 0.6deg | 0.032 | 2 |
| heavy 3 | 0.6deg | 0.014 | 1 |
| heavy 4 | 0.7deg | 0.031 | 2 |
| flow 0 | 19.3deg | 0.021 | 18 |
| flow 1 | 15.0deg | 0.022 | 24 |
| flow 2 | 15.7deg | 0.022 | 12 |
| flow 3 | 12.2deg | 0.031 | 18 |
| flow 4 | 17.0deg | 0.021 | 16 |

Every heavy run holds heading within **0.7 degrees** and lateral
position within **3.4cm** of its initial line for the ENTIRE approach
-- compare Step CL's SAME seeds, SAME zone, rotating -38 -> -88 ->
-132deg by frames 3/6/9. `switches` is now 1-2 in every heavy run and
12-24 in every flow run (CheapStage re-evaluates far more often simply
because it runs at a much higher frame rate, not because it's more
decisive) -- **never stuck at exactly 1** the way every pre-CM run was,
confirming the hysteresis/world-bearing fix (Bug 2) is genuinely
working, not just the yaw-rate convergence (Bug 1).

**The final logged frame of every heavy run shows a large heading
"jump" (0.8 to 95.5deg) -- this is a collision artifact, not
controller behavior.** `vehicle_velocity` at that frame is the
airframe's post-impact motion (bouncing/tumbling off the box), not a
commanded trajectory; excluded from the table above, and flagged
explicitly rather than silently dropped so it isn't mistaken for
Step CL's spin recurring. Flow runs show no such artifact (0.0deg
final jump in every case) -- consistent with their much higher frame
rate meaning the last LOGGED frame lands further from the actual
impact instant.

### Which sector, and when

Centre (sector 5)'s score first dropped below 0.5 at these frames --
and in every single run, the target sector reported AT THAT EXACT
FRAME is still 5 (centre itself). That's expected, not a bug: crossing
0.5 is what STARTS the hysteresis clock, not an instantaneous switch.
What matters is what happens next:

| run | frame centre score first <0.5 | target at that frame | target sector(s) AFTER, before collision |
|---|---|---|---|
| heavy 0 | 12 (of 16) | 5 | 1, then 0 |
| heavy 1 | 11 (of 14) | 5 | 7, then 10 |
| heavy 2 | 10 (of 13) | 5 | 5 (unchanged), then 10 |
| heavy 3 | 12 (of 14) | 5 | **never switches -- flies centre-committed into the box** |
| heavy 4 | 11 (of 13) | 5 | 1 |

4 of 5 heavy runs DID genuinely switch away from centre -- real,
working avoidance, something no pre-CM run ever did (target_sector was
permanently stuck at the initial pick in every one of them). Run 3 is
the clean counter-example: centre score collapses to 0.016 and the
controller still never switches, flying centre-committed straight into
the box. Even the 4 that DID switch still collided.

### The new root cause: reaction arrives too late relative to closing speed, not that reaction doesn't happen

Centre score does not cross the 0.5 threshold until 1-3 frames (HeavyStage
frames, ~0.9-1.2s apart in sim time at this batch's RTF) before the
logged collision. `hysteresis_min_frames=3` requires the challenger to
keep winning by `hysteresis_margin` for 3 CONSECUTIVE frames before a
switch commits -- at a ~1s-per-frame effective decision rate, that is
itself close to 3 seconds of sustained, unambiguous danger needed
before ANY switch can complete, and Step BK's own "0.733m achievable
clearance at 0.8 m/s" arithmetic implicitly assumed reaction begins
as soon as danger is real, not 3 seconds of confirmation after it
first crosses a threshold that itself only trips a frame or two before
impact. This is DIFFERENT from every prior diagnosis in this
project's history (not perception blindness -- Step CE/CI; not score
units -- Step CG; not FOV artifact -- Step CH; not open-loop spin --
Step CL): the controller now correctly recognizes danger and correctly
initiates a real turn, but the combination of HeavyStage's per-frame
latency and hysteresis's own debounce window leaves too little
distance/time to complete it before the airframe's own momentum
carries it into the box.

**Flagged, not fixed here, following this project's own standing
practice (Step CL's closing note) of reporting a newly found mechanism
rather than unilaterally tuning parameters to make it go away.**
Candidate levers for whoever picks this up next -- lowering
`hysteresis_min_frames`, raising `hysteresis_margin`'s sensitivity to
trip earlier, reducing `fwd_vel` further, or attacking HeavyStage's own
per-frame latency directly -- are all plausible and none were picked
here; picking one now, after just finding this mechanism, would repeat
exactly the "tune until it looks fixed" mistake this session has
consistently avoided.

### Flow-only arm: CheapStage's blindness finding stands, now with harder numbers

`blind` mode dominates every flow run (90-97% of frames: 403/417,
379/399, 390/399, 383/397, 379/390) -- CheapStage still cannot extract
usable sector scores from Zone B's untextured box, the qualitative
finding this whole controller investigation started from (Step CE) and
explicitly NOT something Step CM/CN/CO's controller fix could address,
since it's a perception-stage limitation, not a control one. All 5
flow runs still collided at zone_B.

### What this means for every number in this README

The collision-rate numbers in this section (10/10 across both arms)
are the FIRST ones in this project's history produced under a
controller that doesn't have Bug 1, Bug 2, or the `n_sectors` bug --
they supersede Steps AS, CE, CH, and CL, exactly as flagged at the top
of the Step CM/CN section above. They are also, honestly, not better:
same 100% collision rate, same zone, same qualitative CheapStage
blindness. What changed is WHY -- from an open-loop controller that
physically could not track its own target, to a correctly-behaving
controller whose reaction still arrives too close to impact to matter
at this speed, this obstacle size, and these timing constants. Both
are real findings; neither should be reported as "the" obstacle-
avoidance result without the other.

## Step CP -- diagnosing heavy_co_run3: no fourth bug, hysteresis debounce math working exactly as designed

Full offline replay of `heavy_co_run3_frames.jsonl` (the one Step CO
run where `target_sector` never left centre despite the centre score
collapsing to 0.016) through the fixed controller, reconstructing every
sector's cost, the committed world bearing, and the hysteresis streak
counter at each frame (no sim; belief sequence + real logged
velocity-derived yaw replayed exactly as flown).

**`valid` was `[True]*11` at every single frame** -- never excluded any
alternative. **`target_world_bearing` stayed at 0.00deg (dead ahead)
for the entire flight** -- never drifted, correctly tracking centre the
whole time (consistent with Step CO's own 0.6deg max-heading-deviation
finding for this run). Neither the valid mask nor a stale/rotating
bearing explains the non-switch.

The actual mechanism, frame by frame:

| frame | t | centre score | cost(centre) | best challenger | cost(challenger) | margin cleared? | streak |
|---|---|---|---|---|---|---|---|
| 10 | 32.44 | 0.844 | 0.159 | sector 6 | 0.135 | NO (needed <0.009, missed by 0.024) | -- |
| 11 | 33.46 | 0.578 | 0.426 | sector 6 | 0.278 | NO (needed <0.276, missed by 0.002) | -- |
| 12 | 34.55 | 0.140 | 0.864 | sector 7 | 0.419 | YES | starts, streak=1 |
| 13 | 35.48 | 0.016 | 0.988 | sector 9 | 0.428 | YES (different challenger -- see sector.py's own documented simplification) | streak=2, needs 3 |

Frames 10-11 show hysteresis correctly suppressing genuine but
insufficient challenges (missed the margin by 0.024, then by a mere
0.002). Frame 12 is where real, decisive danger appears and a
debounce streak legitimately starts. Frame 13 continues it. The flight
ends via collision before a 3rd confirming frame -- which, at
HeavyStage's ~1.1s/frame rate in this run, would have needed roughly
another second that the ~2s-wide danger window didn't have.

**Conclusion: this is not a fourth latching bug.** It is
`hysteresis_min_frames=3` -- a threshold picked for CheapStage's
~25Hz rate (3 frames = 0.12s of confirmation) -- silently costing
HeavyStage's ~0.8-1Hz rate ~3-3.75 REAL SECONDS of confirmation for the
exact same rule, inside a danger window barely wider than that. Same
bug class as Steps CM (world-bearing) and CN (n_sectors): a
control-layer constant silently assuming a fact about whichever
perception stage happens to be feeding it.

## Step CQ -- hysteresis made time-based

`hysteresis_min_frames: int = 3` -> `hysteresis_min_seconds: float =
0.12` (the CONVERTED value: 3 frames / 25Hz = 0.12s, not retuned).
Evaluated against `t_capture` (sim time from each frame's packet),
never wall clock, per this module's standing constraint (`types.py`'s
docstring). `SectorController.step()` gained a required `t_capture`
parameter; `ControlState.frames_on_target: int` (a frame COUNT) became
`ControlState.seconds_on_target: float` (elapsed sim TIME, same
sign convention: positive=dwelling, negative=in-progress-challenge
magnitude) -- ControlState.last_t, present but unused since Step AN,
is now genuinely read every step to compute `dt = t_capture - last_t`.

Each qualifying frame contributes its OWN `dt` to whichever streak is
active (mirroring the old code's literal "+1 per frame" with dt
substituted for 1) -- so three 25Hz frames (dt=0.04s each) accumulate
to exactly 0.12s and switch on the 3rd, identical to the pre-CQ
behaviour, while a single HeavyStage frame whose OWN interval already
exceeds 0.12s (its dt alone, e.g. ~1.1-1.25s) now correctly switches on
the FIRST confirming frame rather than being forced to wait for a 2nd
and 3rd that may not arrive before impact. Two implementation
subtleties worth recording plainly (both found by construction, not
guessed):
- **A fresh commitment must store `dt`, not `0.0`.** Storing exactly
  0.0 on the frame a target is adopted would be indistinguishable from
  "no streak at all" on the very next frame's sign check, silently
  discarding that frame's own elapsed time and under-counting true
  dwell duration by one frame-interval on every single commitment.
- **A fresh challenge must likewise start at `dt`, not `0.0`** for the
  same reason -- `0.0` negated is still `0.0`, not a distinguishable
  negative sentinel, so a challenge that "started at 0" would look
  identical to "no challenge in progress" and could never accumulate
  past its first frame.

**NOISE-REJECTION TRADE-OFF, stated plainly rather than left implicit:**
converting to real time necessarily means a slow-rate source gets less
built-in noise suppression per sample than a fast one did under the old
frame-count rule -- a single spurious HeavyStage frame can now trigger
an immediate switch where 3 confirming frames (~3.75s) were required
before. This is the direct, unavoidable consequence of removing the
rate-unfairness Step CQ exists to remove, not an oversight.

**VERIFIED** with `test_hysteresis_rate_independent_25hz_vs_0.8hz`:
at 25Hz (dt=0.04s), a single challenging frame does NOT switch (noise
rejection preserved, matches old behaviour exactly) but three
consecutive ones (cumulative 0.12s) do; at 0.8Hz (dt=1.25s), a SINGLE
challenging frame switches immediately, since its own interval already
exceeds the 0.12s threshold. Same rule, both rates -- confirmed by all
36 tests passing (33 pre-CM + 2 Step CM + this 1 new CQ test).

**Sanity check against the actual failing case (offline replay, not
yet re-flown):** replaying `heavy_co_run3`'s exact belief/yaw sequence
through the NEW time-based hysteresis, frame 12 (t=34.55, the exact
frame Step CP identified as where a real streak starts) now completes
the switch to sector 7 IMMEDIATELY -- `yaw_rate` jumps from ~0 to
-0.515 rad/s that same frame, roughly a full second earlier than the
old frame-count rule could have allowed. Frame 13 continues reacting,
switching again to sector 9 (`yaw_rate=-0.8`, saturated). Whether this
is enough real-world distance/time to avoid the box is a closed-loop
question the offline replay cannot answer -- that is exactly what
Step CR re-flies.

## Step CR -- the controller now reacts hard and in time, and STILL collides 10/10. This is the physical limit, and it is the paper's finding.

5 heavy-only + 5 flow-only runs, BN seeds, 0.8 m/s, Zone B, against the
CP/CQ time-based-hysteresis controller. (Runs heavy 0-3 were flown
before a mid-batch pause and kept; heavy 4 + all 5 flow flown on
resume. `heavy_cr_run3.json` completed normally in the ~2s between the
pause command and process teardown -- verified a complete 14-frame run,
not a truncated artifact.) All processes confirmed torn down clean.

### The fix worked. The switch behaviour is now everything CP/CQ intended.

| run | collided | switches (CO -> CR) | max heading dev (excl. collision frame) | centre <0.5 at frame | target when danger detected |
|---|---|---|---|---|---|
| heavy 0 | yes | 2 -> **4** | 20.3deg | 11 | 3 |
| heavy 1 | yes | 2 -> **3** | 11.7deg | 11 | 6 |
| heavy 2 | yes | 2 -> **5** | 10.7deg | 11 | 2 |
| heavy 3 | yes | **1 -> 3** | 0.7deg* | 11 | 2 |
| heavy 4 | yes | 2 -> **5** | 28.9deg | 10 | 4 |

Switch counts rose across the board (CO's 1-2 -> CR's 3-5), and
**heavy_cr_run3 -- the exact run Step CP diagnosed as never switching
at all -- now switches 3 times.** (*run3's velocity-derived heading
still reads ~0.7deg max because, as Step CP found, its danger arrives
so late that even an immediate, correct yaw COMMAND leaves essentially
no time for the vehicle's actual velocity vector to rotate before
impact -- the command changed, the achieved heading barely could. The
other four runs, whose danger arrives a frame earlier, show the turn
reaching the velocity vector: up to 28.9deg.) Flow arm: switches
8-32, `blind` still 362-376 of ~380 frames -- CheapStage's Zone B
blindness stands exactly as before, unchanged and unrelated to any
controller fix.

### And it still collides 10/10, for a reason no further controller work can touch

The decisive measurement, the one Step CR was added to make -- **time
and perception-update count between first danger signal and collision:**

| run | first danger -> collision | HeavyStage updates in that window | max lateral displacement achieved |
|---|---|---|---|
| heavy 0 | 1.95 s | 2 | 0.031 m |
| heavy 1 | 1.88 s | 2 | 0.027 m |
| heavy 2 | 1.85 s | 2 | 0.031 m |
| heavy 3 | 1.95 s | 2 | 0.014 m |
| heavy 4 | 2.94 s | 3 | 0.034 m |

At HeavyStage's measured ~1.1 s per frame in these runs, the centre
score does not cross 0.5 until there are only **2-3 perception updates
left before impact**. The controller now reacts on the very first of
those (CQ's whole point -- verified: it switches and commands
saturated yaw within one frame of danger), but two things no debounce
change can fix remain:

1. **~2 seconds and 2-3 updates is not enough distance to translate
   clear at 0.8 m/s.** Max lateral displacement achieved before impact
   is ~3 cm across every run; the box needs ~0.5 m of clearance (Step
   BK). The vehicle rotates its heading (up to 28.9deg) but cannot move
   its body far enough sideways in the time left -- the achieved
   displacement is two orders of magnitude short.
2. **Detection itself is late because it is update-rate-limited.** The
   box only reads as dangerous once it is close enough AND a HeavyStage
   frame happens to land -- at ~1.1 s between frames, "close enough"
   and "next frame" together consume most of the ~4 m detection range
   before a usable danger signal exists at all.

This is the constraint the task anticipated: **at ~0.8-1 Hz inference
and ~4 m effective detection range, there are simply not enough
perception updates in the approach window to avoid an obstacle at this
flight speed.** It is not a perception-quality bug (HeavyStage sees the
box -- centre score does collapse, correctly, by frame 11), not a
score-scale bug (Step CG), not a controller bug (Steps CM/CN/CP/CQ
between them removed every one found, and the switch/heading numbers
above prove the controller now does exactly the right thing with the
information it has). It is an **update-rate limit**, and it is a
genuine, quantified result rather than a failure to be tuned away.

### Stopping here, deliberately -- and why speed is NOT the lever

Per this step's own standing instruction: the resolution is NOT to
reduce flight speed (that would break the constant-0.8-m/s design that
keeps the cheap and heavy arms a controlled comparison, and it would be
tuning-to-pass, the exact thing this project has refused at every prior
step). The resolution is **more perception updates in the same spatial
window**, which is a HARDWARE/latency question, not a control one:
HeavyStage's ~1.1 s CPU inference here is the bottleneck. On the target
HIL Jetson with GPU depth at 30-50 ms/frame, the same 0.8 m/s approach
over the same ~4 m detection range yields **20-30x more perception
updates** -- tens of frames in the danger window instead of two or
three -- which is where the achievable-clearance arithmetic (Step BK)
finally has the samples it assumed all along. That measurement is the
next milestone; continuing to fly this obstacle in CPU sim at 0.8 Hz
would only keep re-measuring the same update-rate wall.

**What every collision number in this project now means, stated once,
plainly:** the sim collision rate against Zone B's untextured box is
100% in both arms, at every stage from AS through CR. The REASONS
differ and have been chased down one at a time -- CheapStage cannot see
the box at all (perception, stands); the metric checkpoint's FOV
artifact (CH, fixed); the open-loop spin (CL/CM, fixed); the
left-half-only sector bug (CN, fixed); the frame-rate-unfair debounce
(CP/CQ, fixed) -- but the HeavyStage arm's *final* reason, after every
controller bug is gone, is the update-rate limit above. That is the
finding to write up, and the HIL measurement is its intended
resolution.

# Phase 5 -- Training the learning-to-defer gate (Steps CS-CV)

The project's central question, and it does NOT depend on any arm
avoiding anything: **can a gate, reading ONLY CheapStage's per-frame
feature vector (never depth, never HeavyStage's output, never ground
truth), predict that the current frame is on a soon-to-collide
trajectory** -- i.e. learn when the cheap stage should defer to the
expensive one? Offline, no sim. Tools: `tools/gate_dataset.py` (build +
label), `tools/gate_train.py` (train + eval + baseline).

## Step CS -- the labelled dataset, and two honest data constraints

Label (per frame): **1 if the run collides within 2.0 s of this
frame's `t_capture`, else 0.** Time-based, not a frame count -- runs
were logged at different perception rates, so a fixed frame count would
mean different real lead-times per run. Collision time = the run's last
logged frame's `t_capture`: verified that the last logged
`vehicle_position` equals the summary's `collision_xy` to 0.00 m in all
26 usable runs, i.e. every run terminates AT collision.

**Two constraints found while building it, reported not designed
around:**
1. **Heavy-only runs are unusable.** They log an EMPTY feature dict --
   HeavyStage does not compute CheapStage's feature vector, so despite
   the task's framing there is literally nothing for the gate (which
   reads cheap features) to consume in them. 39 heavy runs (1338
   frames) excluded on this basis. This is not a loss of signal for the
   gate: the gate's input is defined as cheap features, and those exist
   only in cheap-source runs.
2. **The n_sectors=11 cheap runs (flow_co/flow_cr, 10 runs) use a
   102-element vector** with a different per-sector layout, not
   concatenable with the 54-element (n_sectors=5) vector the task
   specifies. Excluded (reported), leaving the 54-element layout as the
   single consistent training representation. (1 empty log also
   excluded.)

**Usable dataset: 26 runs, 40,362 frames, 3.93% positive** (1585 pos /
38777 neg -- ~61 positive frames per run, i.e. ~2 s at CheapStage's
~30 Hz). Per-zone:

| zone | frames | positive | runs |
|---|---|---|---|
| zone_B | 22,648 | 915 (4.0%) | 15 |
| zone_C | 17,185 | 488 (2.8%) | 8 |
| zone_D | 529 | 182 (34.4%) | 3 |

**A real limitation, stated up front: every usable run collided.**
There are no safe-completion runs in the logged history for the cheap
arm, so the negative class is entirely "early frames of runs that
eventually collided," not "frames of runs that stayed clear." The gate
therefore learns to rank *proximity to an imminent collision within a
collision trajectory*, which is what the defer decision needs, but it
has never seen a full clear approach as a counter-example. Worth a line
in the paper and a reason to log some obstacle-free / successful passes
before over-claiming generalization.

## Step CT -- training: LR, leave-one-run-out, asymmetric loss

Logistic regression (the task's first choice, and -- see Step CU -- the
one whose weights are the interpretability story, so no reason to reach
for an MLP once LR clears the bar). Asymmetric loss: false negatives
weighted **7x** false positives (mid of the task's 5-10x range, ONE
setting, not tuned). **Entire runs held out, never random frames** --
consecutive CheapStage frames are near-duplicates and a random split
leaks catastrophically; leave-one-run-out CV (26 folds), AUC computed
on the pooled out-of-fold predictions. Features standardized per fold
(nan-aware: `nanmean`/`nanstd` on training runs only, then NaN->0 so an
imputed value sits at the training mean; the never-NaN `valid_sector_i`
/ `gyro_valid` / `odom_valid` flags carry the "measurement missing"
signal -- the honest choice for a linear model on missing-not-at-random
features, since NaN here means "no valid signal," itself informative).

**POOLED ROC AUC on held-out runs = 0.900. This is the go/no-go, and
it is a clear GO.** Mean per-fold AUC 0.913 +/- 0.116. Per-fold ranged
from near-perfect on the `run0-9` batch (0.949-0.997) down to one fold
BELOW chance -- `flow08_run6` at 0.476 -- with two more weak
(`flow08_run4` 0.707, `flow08_run3` 0.774). The weak folds are all
long zone_C flow08 runs (3200-3500 frames); the gate generalizes least
well to those, an honest caveat the 0.116 fold-std already signals and
which the pooled 0.900 should be read alongside, not instead of.

## Step CU -- interpretability: the signal is subtle, not the valid mask

Learned weights (full-data fit, standardized -> directly comparable),
top by magnitude:

| weight | feature | reading |
|---|---|---|
| -1.844 | count_sector_1 | fewer tracked points in a near-centre sector -> danger |
| -1.264 | centre_mean_tau | lower centre time-to-contact -> danger |
| +1.149 | tau_rate_sector_0 | **tau dropping (temporal derivative) -> danger** |
| -0.939 | grad_energy_sector_2 | **gradient energy (sky-vs-wall discriminator) -> danger when low** |
| -0.882 | tau_sector_3 | |
| -0.862 | ground_frac_sector_0 | more ground-plane points -> danger signal |
| -0.823 | across_sector_tau_variance / tau_variance | |
| -0.820 | cone_effective_count | **fewer points near the flight path -> danger** |
| +0.800 | tau_rate_sector_1 | second temporal derivative |
| +0.749 | centre_min_tau | |

**The paper's interpretability finding: the learned gate does NOT lean
on the obvious flags.** `min_tau` -- the single feature a human would
reach for, and the core of the heuristic baseline below -- gets a
near-ZERO learned weight (-0.003). So do `odom_valid` (0.000),
`valid_sector_4` (-0.016), and `gyro_valid` (+0.079). The signal
instead lives in **subtler features the spec was deliberately built to
expose**: per-sector tracked-point COUNTS, centre-region tau, TEMPORAL
tau derivatives (`tau_rate_sector_*`), GRADIENT ENERGY (the
sky-vs-textureless-wall discriminator), CONE-relative point
concentration, and ground-fraction distrust signals. This is precisely
the case for a 54-feature learned gate over a two-line threshold, and
the reason to prefer LR: its weights *say* what it found.

## Step CV -- heuristic baseline: a simple threshold does NOT suffice

Hand-tuned gate a reviewer would propose: danger rises as fewer sectors
are valid AND min tau shrinks (with NaN min_tau -- no valid sector at
all -- mapped to maximum danger). Scored as a continuous danger value
so its ROC AUC is threshold-free and directly comparable.

**Heuristic pooled ROC AUC = 0.735. Learned gate = 0.900, a +0.165
improvement.** We asked the reviewer's question ourselves and the
answer is that the learned gate beats the simple threshold decisively
-- and Step CU explains WHY: the heuristic's two signals (valid count,
min tau) are exactly the ones the LR found *least* useful, while the
predictive signal sits in the temporal / gradient / count features a
hand-tuned threshold never touches. Had the learned gate merely tied
the heuristic, that would have been the finding and we would have
reported it; it does not tie, it clears it by 0.165 AUC on held-out
runs.

**Constraints honoured:** no disagreement labels (HeavyStage never
trusted as a label source); the label horizon (2.0 s) and loss weight
(7x) were each set once from their stated definitions and NOT tuned to
chase AUC; LR chosen over MLP because it cleared the bar AND carries the
interpretability story, not because an MLP was tried and rejected.

# Phase 5 validation -- adversarial checks on the 0.900 (Steps CW-CZ)

The 0.900-vs-0.735 result is the project's headline, so before building
on it we tried to BREAK it. `tools/gate_validate.py`. Bottom line: **it
survives the leakage audit decisively -- the 0.900 is real, not a clock
in disguise -- but leave-one-ZONE-out shows its generalization is
zone-specific, not universal. Both halves must be stated together.**

## Step CW -- label-leakage audit (highest priority): PASSES

Concern: every usable run ended in a collision, so "labelled 1" and
"near the end of a run" are nearly the same set -- the model might be
reading TIME ELAPSED, not danger. Four checks, all pointing the same
way:

**CW.1 -- positional-only models are near chance.** An LR trained on
ONLY `time_since_takeoff` scores pooled AUC **0.566**; on ONLY
`frame_index`, **0.568** (leave-one-run-out). If elapsed position alone
predicted the label these would be high; they are barely above 0.5.
The reason is structural and worth stating: the label is "within 2 s of
this run's END," but run lengths span 4.5 s to 117 s, so the positive
region sits at a wildly different absolute time in each run -- no single
time/index threshold can separate them across runs. Position alone
carries almost no cross-run signal.

**CW.2 -- 10 features drift monotonically with time.** Mean per-run
Spearman(feature, time-since-takeoff), flagged where |mean rho| >= 0.5
AND std <= 0.35 (consistent direction across runs): `global_feature_
count` (-0.68), `grad_energy_sector_2` (-0.64), `valid_sector_3/4/2/0`,
`odom_valid` (-0.58, std 0.02 -- essentially a pure clock),
`gyro_valid`, `frac_features_lost`, `count_sector_3`. These are the
features a time-leak would exploit.

**CW.3 -- removing ALL 10 drifters changes nothing.** Retraining on the
44 non-drifting features: pooled AUC **0.9006 vs 0.9001, delta
+0.0005.** This is the decisive check: if the 0.900 were riding on
time-correlated drift, deleting every drifting feature would gut it. It
doesn't move. The predictive signal lives in the non-drifting features
(tau values, `tau_rate` temporal derivatives, flow, centre tau).

**CW.4 -- per-zone AUC is NOT uniform, and is strongest in the hardest
zone.** zone_B (untextured-wall texture failure) **0.953**, zone_D
(narrow obstacle) 0.858, zone_C (density) 0.800. The task's own tell:
uniform-everywhere would be suspicious (a global clock); concentrated
in the genuine failure modes is evidence of real scene-grounded signal.
It is concentrated exactly where CheapStage actually fails hardest.

**Verdict: the 0.900 is not contamination.** Positional-only is chance,
drifter-removal is a no-op, per-zone is scene-shaped.

## Step CX -- weights: the richer post-O2 features carry the signal

Full-data-fit weights (standardized, comparable), top 15, tagged O =
"obvious" (valid mask, gyro/odom_valid, raw counts, global count,
min_tau) vs R = richer/post-O2 (temporal derivatives, gradient energy,
cone-relative, centre-region tau):

| weight | tag | feature |
|---|---|---|
| -1.844 | O | count_sector_1 |
| -1.264 | R | centre_mean_tau |
| +1.149 | R | tau_rate_sector_0 (temporal derivative) |
| -0.939 | R | grad_energy_sector_2 (sky-vs-wall) |
| -0.882 | R | tau_sector_3 |
| -0.862 | R | ground_frac_sector_0 |
| -0.823 | R | across_sector_tau_variance / tau_variance |
| -0.820 | R | cone_effective_count |
| +0.800 | R | tau_rate_sector_1 |
| +0.749 | R | centre_min_tau |
| -0.738 | O | global_feature_count |
| -0.674 | O | count_sector_2 |
| +0.667 | R | tau_rate_sector_4 |
| +0.622 | R | frac_features_lost |

**Total |weight| mass: obvious 5.15, richer 17.39 -- the richer,
post-O2 features carry 77% of it.** This closes the O2 loop end to end:
O2 found raw per-sector counts do NOT discriminate danger, which is
exactly why the richer features were added; those richer features are
where the learned signal sits. (`count_sector_1` is the single largest
individual weight, so counts aren't useless -- but the aggregate story
is decisively the richer features, and `min_tau`, the heuristic's whole
basis, gets ~0 weight, per Step CU.)

## Step CY -- operating point, deferral rate, implied k

AUC is threshold-free; the deployed gate is not, so we pick the
operating point the asymmetric loss implies. FN weighted 7x FP gives a
Bayes threshold p* = cost_FP/(cost_FP+cost_FN) = 1/(1+7) = **0.125**.
ROC curve (full-model OOF) is steep early -- tpr 0.95 already at fpr
0.33 -- consistent with AUC 0.90.

At p* = 0.125:
- **deferral rate = 26.5%** of frames scored >= p*
- **recall = 0.909** (catches 91% of danger-window frames)
- **precision = 0.134** (most deferred frames aren't themselves in the
  2 s window -- expected at a 3.9% base rate and a deliberately
  high-recall, FN-averse operating point; the gate is tuned to rarely
  MISS danger, at the cost of deferring liberally)
- **implied PERIODIC-baseline k = 1/0.265 = 3.8** -- i.e. to match this
  gate's compute budget, the Phase 6 periodic baseline must run
  HeavyStage every ~4th frame. This is the number Phase 6 needs.

## Step CZ -- cross-validation, including the honest cross-zone limit

**CZ.a -- leave-one-run-out:** mean AUC **0.913 +/- 0.116** (pooled
0.900, n=26). Solid, but the 0.116 spread already warned that some runs
generalize worse than others.

**CZ.b -- leave-one-ZONE-out** (train on the other zones entirely, test
on a failure mode never seen in training). No Zone A exists in the
usable cheap runs, so zones are B/C/D:

| test zone | trained on | AUC |
|---|---|---|
| zone_B (untextured wall) | C + D | **0.512** (chance) |
| zone_C (density) | B + D | 0.658 |
| zone_D (narrow) | B + C | 0.816 |

**This is the honest limit of the result, reported plainly: the gate
does NOT transfer to a failure mode it never trained on.** Held out
entirely, zone_B collapses to chance -- training on density (C) and
narrow-obstacle (D) runs teaches the model nothing about the
untextured-wall signature, where trackable features vanish altogether
rather than behaving like a closer obstacle. zone_D transfers best
(0.816), plausibly because narrow obstacles share more of their cheap-
feature signature with the other zones.

**What this means for the paper, stated without spin:** the headline
0.900 is a leave-one-RUN-out number and is genuine (Step CW rules out
the clock artifact decisively) -- it measures generalization to new
runs of failure modes the gate HAS seen. It is NOT evidence that a gate
trained on some failure modes will catch an unseen one; CZ.b shows it
largely will not, worst of all for the very texture-failure case that
motivated the project. The deployable claim is "learns the cheap-feature
signatures of failure modes present in its training distribution," and
the training distribution must therefore cover the failure modes the
deployment will face. This is exactly the result a reviewer would probe
for, so we report it up front rather than let the 0.900 stand alone.

**Constraints honoured:** nothing tuned to preserve 0.900 (the
drifter-exclusion, zone-holdout, and operating-point analyses were run
once and reported as-is); no disagreement labels; no sim.

# Phase 6 -- CheapStage add-ons: world-geometry masking + contour/looming (Steps DH-DL)

Two ADD-ONS to CheapStage. The LK path is untouched; nothing existing
is removed. (Phase 5 will need retraining afterwards -- planned, not
done here.)

## Step DH -- world-geometry row band: implemented, measured, left OFF (it doesn't help cheap)

**DH.1 -- ground contamination is far worse than R2's old 5-15%.**
Replaying the point-tau pipeline over 8 recordings
(`tools/dh_ground_contamination.py`), point-weighted across all of
them: **42.7% of tracked points fall below the horizon (row >= cy=240),
and 60.9% of the 20th-percentile-tau "tail" -- the near points that
actually SET each sector's danger score -- are below-horizon.** R2
measured 5-15% and chose to feed ground-fraction to the gate rather
than mask; the fresh numbers are 3-4x higher. Per recording (below-
horizon% / tau-tail-ground%): boxB1_approach 50.2% / **85.7%**,
textured_approach_treeA2 47.5% / 47.0%, narrow_approach_D1 76.2% /
66.7%, wall_approach_1p0 88.2% / n-a, corridor segments 27-49% /
24-97%. On boxB1 specifically, the *rare* valid CheapStage readings are
86% ground-driven -- i.e. what little the cheap stage "sees" on the
untextured box is mostly the ground plane, not the box. (Caveat stated
plainly: "below horizon" over-counts true ground contamination, because
an obstacle's own lower half is also below the horizon line -- which is
exactly why the fix is a geometry-derived BAND around the horizon, not
a simple horizon cut.)

**DH.2 -- implemented a configurable row band, default OFF**, deriving
it from the SAME geometry HeavyStage uses (Step CJ):
`half_band_px = fy * obstacle_half_height / min_range`, giving band rows
**162-318** for this camera -- identical to heavy. Applied as an add-on
filter on tracked points inside `CheapStage.infer()`; `track_pair`/LK is
untouched, and with the flag off the pipeline is byte-identical to
before (36 tests still pass).

**DH.3 -- it does NOT help, so it stays off.** On
textured_approach_treeA2, band ON vs OFF:
`correlation(distance, min_tau)` goes **+0.522 -> +0.451 (worse)** and
overall per-sector valid-rate **collapses 9.3% -> 2.5%** (valid min_tau
frames 238 -> 87). Every sector loses valid-rate; sector 5 (the
tree-bearing centre) drops 27.1% -> 4.9%. The reason is exactly R2's
original one, now confirmed with much larger contamination numbers:
**CheapStage is point-starved** (unlike HeavyStage's dense metric depth
map, which can afford to discard 2/3 of its rows), so cropping half the
image removes points it cannot spare -- including legitimate obstacle
points (the trunk extends below the band too). Masking is the wrong
remedy for a point-starved stage even though the contamination it
targets is real. Per the task's own rule ("do not enable until DH.3
shows it helps"), the band ships OFF. The honest conclusion: R2 was
right not to mask; the fix for cheap is to ADD signal (Step DI's contour
channel), not remove points.

## Step DI/DJ -- contour/looming channel: a second, LK-free TTC source

New module `perception/contours.py` (`ContourLoomingChannel`).
CheapStage constructs one and MERGES its per-sector output into the
belief and feature vector; the LK estimator (track_pair + point-tau) is
completely untouched. Default ON (`use_contours=True`); with it off the
belief is byte-identical to the pre-DI pipeline (36 tests pass either
way).

**Detection (DI):** GaussianBlur -> Canny -> morphological CLOSE ->
`findContours(RETR_EXTERNAL)` -> filter by area (drop specks <0.2% and
world-fill >60% of frame). Close-after-Canny gives closed silhouette
REGIONS, not raw edge pixels, as the task required. Regions are tracked
across frames by centroid + area proximity (a lightweight identity
match, not a full tracker), and assigned to the sector containing their
centroid.

**Looming TTC:** `tau_area = 2A / (dA/dt)`. The factor 2 is geometric
(frontal area A ~ 1/z^2 -> (dA/dt)/A = 2/tau_z). Heeding the LK-tau
lesson that raw frame-to-frame dA/dt is garbage, **dA/dt is the slope of
a line fit to (t, area) over a 0.6 s window** (>=4 samples required),
never a consecutive difference; tau is emitted only when the region is
actually growing (approaching).

**Merge policy (DJ), stated and justified:** per sector CheapStage now
has two TTC sources -- LK per-point tau and contour tau_area:
- both valid -> **MIN** (nearest thing wins, same logic as the 20th-
  percentile LK choice);
- one valid -> use it, and record WHICH via a per-sector source flag
  (1 LK-only / 2 contour-only / 3 both);
- neither -> `valid=False`, as before.
The merged tau/valid drive the SectorBelief (its own EMA state, so the
LK-only `tau_sector_*`/`valid_sector_*` features keep their exact pre-DI
meaning). Feature vector GROWS by 6 per-sector features (nothing
removed): `tau_area`, `contour_region_count`, `contour_total_area`,
`contour_growth` (largest region's dA/dt), `ttc_source`, and
`tau_agreement` = min(tau_lk,tau_area)/max(...) where both exist (their
disagreement is itself an unreliability signal the gate can learn from).
Vector length 102 -> 168 at n_sectors=11; the pre-DI features stay a
stable prefix.

## Step DK -- verification on the diagnostic recordings

**DK.1 boxB1_approach -- YES, contour detects the untextured box where
LK is blind.** Frames with any valid sector **64 -> 225 (3.5x)**; belief
valid-rate **1.7% -> 7.2%**; contour detection onset range **11.7 m ->
5.5 m**. BUT, reported honestly: the contour `tau_area` VALUE does not
track distance -- `corr(distance, min tau_area) = -0.076`, essentially
uncorrelated (and wrong-signed), with tau_area bouncing across its full
1-30 s range. So on this fly-by recording the channel RESTORES
DETECTION (which sectors are occupied) but its quantitative TTC is not
yet reliable -- likely because a box passed off-centre (y=-6) grows in
area from framing/parallax, not clean head-on looming. The value to the
gate is the detection + region-geometry features (count/area/growth) and
the agreement flag, not a trustworthy contour TTC number yet. This is a
documented weakness, not a solved TTC.

**DK.2 textured_approach_treeA2 -- LK path NOT degraded.**
`corr(distance, min_tau)` is **+0.522 with contours OFF and +0.522 with
contours ON** (identical -- the LK-only min_tau feature is untouched, as
designed), while belief valid-rate rises 9.3% -> 12.8% (contour adds
coverage without touching LK). The merged tau does not degrade the
monotonic LK signal.

**DK.3 rotation-only (odom_sync_check1, ~0.006 m/s translation, rotating
in place -- the yaw_only stand-in; no true yaw_only recording exists).**
tau_area should be invalid throughout (no approach = no looming). Result:
**3.54% of sector-slots emit a finite tau_area** -- low, but not zero.
Pure rotation slides regions across the frame and changes their apparent
area (foreshortening, frame entry/exit), producing a small rate of
FALSE looming. Reported as-is; it is exactly the failure the task warned
about, it is small, and `tau_agreement`/`ttc_source` give the gate a
handle to distrust contour-only readings.

**DK.4 corridor per-zone valid-rate (contours OFF -> ON), with dominant
source:**

| zone | valid-rate OFF -> ON | dominant new source |
|---|---|---|
| zone_A | 9.2% -> 17.4% | contour-only (366 vs 399 LK) |
| zone_B | 0.5% -> 2.5% | contour-only (458 vs 123 LK) |
| zone_C | 1.6% -> 2.7% | mixed (310 contour vs 424 LK) |
| zone_D | 0.1% -> 1.7% | contour-only (633 vs 27 LK) |

Zone B -- the untextured-box zone the whole exercise targets -- goes
**0.5% -> 2.5% (5x)**, and boxB1 specifically 1.7% -> 7.2%. Honest
framing: this is a real, several-fold improvement and contour-only is
carrying almost all the new signal in the texture-poor zones (B and D),
**but Zone B is still low-valid in absolute terms (2.5-7.2%), NOT
"substantially valid"** in the sense of 30-50%. The channel moves the
needle in the right direction and restores detection the LK path
structurally cannot provide; it does not by itself make the untextured
box a well-perceived obstacle.

## Step DL -- latency

Isolated (LogFrameSource, corridor_segment2, 2770 frames), matching the
~8-10 ms baseline measurement: contours **OFF mean 10.45 ms** (median
10.32, p95 11.50) vs **ON mean 11.69 ms** (median 11.42, p95 13.43) --
**+1.24 ms, ~12% overhead**. CheapStage stays ~12 ms against
HeavyStage's ~1100 ms, so the cheap:heavy ratio remains ~100:1 and the
contour channel does not push cheap out of the regime that is the
paper's story.

**Net Phase 6 result:** the row band (DH) was implemented but stays off
(it costs a point-starved stage too much); the contour channel (DI-DK)
is the real addition -- it restores DETECTION on the untextured box that
LK structurally cannot see (Zone B valid-rate up several-fold, box seen
from 11.7 m), at ~1.2 ms cost, without degrading the LK path -- while
honestly falling short of a reliable contour TTC value and of making
Zone B "substantially valid." Phase 5 retraining on the new 168-feature
vector is the planned next step (NOT done here).

# Phase 6 cont. -- diagnose tau_area, fix the merge, retrain (Steps DM-DR)

## Step DM -- why tau_area is uncorrelated: the TRACKER, not area noise

First, a correction: DK's -0.076 correlation was measured against the
WRONG obstacle. `boxB1_approach` flies DEAD-AHEAD into **wallB1** (a
0.4x1.0x1.5 m panel at (55,0,3)); boxB1 itself is 6 m off to the side
(a fly-by). Re-measured against wallB1:
- **Area is sound.** Within the primary track the region area grows
  smoothly and monotonically 5924 -> 28732 px^2 as distance falls
  7.0 -> 2.7 m; `corr(distance, area) = -0.462`, correctly signed.
- **It matches geometry (the decisive check).** Measured vs predicted
  front-face area (1.0x1.5 m): 6 m 7712 vs 1759 (ratio 4.4), 4 m 15632
  vs 3958 (4.0), 2 m 19690 vs 15832 (1.2) -- grows as ~1/d^2 exactly as
  predicted. Cause (a) area-noise is RULED OUT.
- **The tracker is the problem (cause b).** Near closest approach the
  region IDENTITY swaps (id0 -> id2, area 28732 -> 12338, then -> id1)
  and at d<1 m detection drops entirely (obstacle exceeds the 60%
  max-area "world" filter). dA/dt across an identity swap or fresh-track
  restart differences unrelated blobs -> garbage tau. Per the task's own
  note, identity swaps are not recoverable by window-fitting.

## Step DN -- merge rule fixed: DETECTION-ONLY

Since tau_area is not fixable to the LK path's +0.522 bar without a real
tracker (out of scope) and carries a 3.54% false-looming rate under pure
rotation (DK.3), the contour channel is switched to **detection-only**:
tau_area NEVER enters `SectorBelief.scores` or `.valid`; the belief is
LK-only (byte-identical to pre-DI). A contour present in a sector with no
LK support is flagged "detected, unmeasured" via `ttc_source=2` and
leaves that sector `valid=False` -- so the controller avoids it as
UNKNOWN, not on an unreliable number. All contour statistics still flow
to the feature vector for the gate. This is the honest split: an
unreliable signal belongs in the gate's features, not the control loop.

## Step DO -- re-verified: LK preserved, detection not regressed

boxB1 belief valid-rate now identical off/on (1.7%, LK-only) while
contour detection is unchanged (onset 11.7 -> 5.5 m; corridor
contour-only detections: zone_B 2058, zone_D 3237 sector-frames).
textured_treeA2 LK correlation **+0.522 both off and on** (LK path
provably untouched). Rotation-only false-looming still 3.54% -- now
harmless (features-only). Per-zone belief valid-rate identical off/on
(contours no longer touch the belief).

## Steps DP/DQ/DR -- retrain on 168 features: contour features do NOT help the gate

**Data constraint, reported not worked around:** the Phase-5 gate logs
are feature-only (no images), so contour features cannot be retrofitted
to them. The only image data is `recordings/` (~15k frames, zones A-D),
replayed through the DN CheapStage for 168-feature vectors, labelled by
ground-truth geometry (contact within 2 s). This is a DIFFERENT regime
from Phase 5: 68.9% positive (dense corridors sit within 2 s of some
obstacle) and **positional-only AUC 0.849** (vs Phase-5's 0.566 -- the
approach recordings are strongly temporally structured). So absolute
AUCs here are NOT comparable to the 0.900, and this data can reproduce
neither it nor Phase-5's zone-collapse. The valid signals are the
matched 102-vs-168 delta on IDENTICAL frames and leave-one-zone-out.

**DP -- adding contour features does not raise AUC; it slightly lowers
it.** Matched on identical frames: 102-feature LORO pooled **0.979**,
168-feature **0.966** (delta -0.013); per-run mean 0.944 vs 0.926.

**DQ -- the model heavily USES the contour features anyway (overfitting,
not generalization).** Top two weights are `contour_growth_sector_5/7`
(+9.1, +4.67); |weight| mass obvious 8% / richer-LK 47% / contour 45%;
post-O2 share 92%. `tau_agreement` carries real weight (total 4.47, the
interesting DQ positive -- the model does find LK-vs-contour
disagreement informative). But this heavy usage does not translate to
held-out gain -- the classic "more features fit training, don't
generalize" the task explicitly warned about (the exp-overflow during
fitting is the tell: extreme standardized contour_growth outliers the
model latches onto).

**DR -- leave-one-ZONE-out (the one that matters): the contour effort
does NOT help generalization.**

| metric | 102 | 168 |
|---|---|---|
| LORO pooled | 0.979 | 0.966 |
| LORO per-run mean | 0.944 | 0.926 |
| leave-one-zone zone_A | 0.763 | **0.668** |
| leave-one-zone zone_B | 0.976 | 0.962 |
| leave-one-zone zone_C | 0.974 | 0.968 |
| leave-one-zone zone_D | 0.964 | 0.969 |
| positional-only AUC | 0.849 | (same data) |
| drop-drifting delta | -- | +0.007 (no reliance) |

168 is worse in 3 of 4 held-out zones (A by -0.094) and essentially tied
in the 4th. **Answer to the core question of the whole contour effort:
on the data available, the new features let the gate FIT the seen zones
(with heavy contour weight) but do NOT help it GENERALISE to an unseen
one.**

**Honest bottom line of Phase 6:** the contour channel earns its place
as a DETECTION signal (it sees the untextured box LK cannot -- Steps
DK/DO) and is correctly kept OUT of the control loop (DN, detection-only)
because its TTC is tracker-limited (DM). As GATE FEATURES it does not
improve generalization on the data we can build (DP-DR). A clean verdict
on the features needs image-logged closed-loop runs matching Phase-5's
label distribution, which do not exist yet -- flagged as the prerequisite
for a real retrain, rather than claiming a gain the data does not show.
The `ttc_source`/`tau_agreement`/detection features remain in the vector
for that future retrain; nothing was tuned to manufacture a gain.

## CheapStage known failure modes

Five ways CheapStage can fail to see a real obstacle. The first two and
the fifth are independently documented in this repo's own diagnostic
history (linked below); the middle two are named here because they're
part of the same family, but their own detailed evidence lives outside
this session's scope -- listed for completeness, not asserted from data
gathered here.

1. **Textureless surfaces** (Zone B's wall). `goodFeaturesToTrack` finds
   nothing on a flat, unmarked surface regardless of distance or
   detector budget -- see R2 (per-sector feature counts) and Step Z
   (18-combination detector sweep, sector 2 stayed at exactly 0
   throughout).
2. **Thin structures** (narrowD1, Zone D). Even without its own
   texture, a narrow enough obstacle can sit close to textured
   background within the same sector and contribute SOME signal (Step
   AE: sector 2 supplied min_tau on 32.4% of narrow_approach_D1's
   frames, vs. 0% on the wide wall) -- but that signal is unreliable at
   the range that matters (Step AJ: sector 2 goes to exactly 0% valid
   in the final 5m, correctly reporting valid=False rather than a
   guess).
3. **FOE blind spot** -- referenced here for completeness; not
   independently characterized in this repo's diagnostic history.
4. **Low speed** -- referenced here for completeness; not independently
   characterized in this repo's diagnostic history.
5. **Insufficient trackable structure from proximity** (Step AH, new).
   A TEXTURED obstacle (treeA2's canopy) becomes untrackable once close
   enough -- not because it lacks texture, but because its texture
   scale exceeds the LK window and local contrast collapses at that
   distance. Overlaid tracked points at 8m/4m/2m/1m
   (`/tmp/.../step_ah_overlays/`, generated by an ad hoc script this
   session, not a permanent tool): sector-2 point count fell 20 -> 8 ->
   9 -> 2 as the canopy grew to fill the frame; by 1m the trunk itself,
   though real bark-textured geometry, contributed zero trackable
   points, and the sector's few survivors sat on an unrelated nearby
   object (a box) instead. This generalizes modes 1 and 2: the common
   failure condition is INSUFFICIENT TRACKABLE STRUCTURE within a
   sector, which a flat surface produces permanently, a narrow object
   produces except near favorable background, and proximity produces on
   ANY surface, textured or not, once it's close enough. A gate (or
   CheapStage itself) that treats "no trackable structure" as one
   failure family rather than three unrelated bugs would be reasoning
   about the actual mechanism, not a list of special cases.

## Feature-extractor requirements that shaped CheapStage's design

Now implemented — see `obst_avoidance/perception/features.py`'s
`FEATURE_SPEC` for the actual indices/definitions. Kept below as the
original reasoning trail, not a to-do list anymore. These are findings
from offline analysis of recorded passes (per-sector `goodFeaturesToTrack`
counts across Zones A-D, and the flow diagnostics in `tools/flow_explore.py`)
that constrain what the eventual CheapStage feature extractor must emit,
so they don't get lost before that code is written:

- **Raw per-sector feature COUNT alone does not discriminate danger.**
  Zone D's narrow textureless obstacle scored a *lower* feature count than
  Zone B's open sky/ground background, because "few features, nothing
  there" and "few features, something untrackable is there" look
  identical in count alone. Whatever replaces raw count needs a way to
  tell these apart.
- **Use centre-weighted statistics, not a global min across all 5
  sectors.** The outer sectors are sky/ground-dominated and legitimately
  low-feature all the time; a global min across all 5 sectors gets
  swamped by that and never isolates the danger signal. Prefer
  sectors 1-3 (centre) specifically.
- **Emit flow magnitude alongside feature count.** This is the
  disambiguator for the bullet above: "few features, low flow" is sky;
  "few features, high flow" is a close untextured obstacle filling the
  frame. Count alone can't tell these apart; count + flow can.
- **Emit temporal derivatives**, not just instantaneous values — rate of
  change of feature count and of time-to-contact (TTC), not only their
  current levels.
- **Emit per-sector gradient energy separately from corner count.** A
  flat, low-gradient region and a corner-sparse-but-high-gradient region
  (e.g. one big edge, few corners) both show low corner counts but mean
  different things; gradient energy is needed to tell them apart.
- **Global feature-count decline + low across-sector TTC variance is an
  early signal for wide walls.** A wide wall fills the whole frame fairly
  uniformly, so it's invisible to any center-vs-outer sector comparison
  until it's already close (~2m out); the earlier tell is a decline
  across *all* sectors together with TTC variance collapsing across
  sectors (everything approaching at the same rate, not just the
  center).
