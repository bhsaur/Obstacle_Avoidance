# Adaptive Perception Gating for Monocular Drone Obstacle Avoidance

**Complete project reference and handover document**

Author: Prateek
Status date: September 2026
Target venues: INDICON / InGARSS (Indian IEEE conferences)
Effort to date: ~100 hours

---

## How to read this document

Sections 1–3 explain what the project is and why it is built the way it is. Sections 4–7 are the technical core: the two perception stages, the controller, and the gate. Section 8 is the complete findings log, including every failed approach and why it failed — this is the most valuable part for anyone continuing the work, because most of the project's cost was spent discovering these things. Sections 9–12 cover current status, the forward plan, and how to extend the work.

If you are picking this project up cold, read §1, §2, §9, §10. If you are debugging something, read §8 first.

---

## 1. The research claim

### 1.1 One sentence

A tiny classifier reads optical-flow statistics and decides, per frame, whether a drone actually needs to run a monocular depth network — cutting heavy inferences substantially without raising collision rate.

### 1.2 The problem

Monocular obstacle avoidance on drones is compute-bound. Learned depth estimators (MiDaS, Depth Anything V2) give usable obstacle geometry from a single camera but cost 30–50 ms per frame on embedded GPUs, and over 1200 ms on CPU. Running them on every frame burns latency and energy on a platform where both are scarce.

Most frames do not need the heavy model. Flying down an empty corridor, a cheap classical cue is entirely sufficient. The expensive model is only necessary in specific, identifiable situations — where the cheap cue is unreliable.

### 1.3 What is novel, and what is not

**Not novel, deliberately:** the perception components. Optical flow, time-to-contact, Depth Anything V2, VFH steering — all off-the-shelf, all citable, all deliberately unimproved. Improving them would attract reviewer scrutiny to the wrong place and, worse, would shrink the measured contribution (see §1.5).

**Novel:** the gating policy — a learned decision mechanism for *when* to invoke the heavy model.

### 1.4 Positioning against the literature

Two bodies of work sit adjacent to this, and the final positioning came from finding them mid-project.

**Introspective perception.** IVOA (Introspective Vision for Obstacle Avoidance, UT Austin AMRL) and Introspective Perception (CMU, 2016) both train an introspection model to predict when a perception system will fail. IVOA even uses a textureless wall as its motivating example — the same case this project built as Zone B. CMU's work evaluates MAV obstacle avoidance by distance flown before pilot intervention, at 1.5 m/s across clutter densities.

**Model cascades.** Routing inputs by difficulty under a compute budget, ancestry going back to Viola-Jones face detection; modern work includes learning-to-defer and learning-to-reject in classification.

**The gap this work occupies:** introspection predicts failure in order to *flag* it — hand control to a pilot, slow down, express uncertainty. Nobody predicts failure in order to *route to a second, more expensive perception model under a compute budget*.

The claim, stated precisely:

> Introspective perception predicts when a vision system will fail. Model cascades route inputs by difficulty under a compute budget. We combine them: an introspection model over cheap-cue features acts as the routing policy in a two-stage perception cascade, deferring to an expensive depth model only when the cheap cue is predicted to fail.

This is narrower than "we invented failure-aware gating," but it is defensible and genuinely unoccupied. It also makes the six-arm evaluation the right experiment: the compute-safety tradeoff is exactly what introspection papers do not report, because they have no second model to route to.

### 1.5 The make-or-break experiment

**The learned gate must beat periodic invocation** — running depth every k-th frame regardless of content — at an equal compute budget. If a fixed schedule matches the learned gate, there is no paper.

Every design decision serves this comparison, especially the world design (§5.1). A heterogeneous corridor is where content-aware gating can win; a uniformly cluttered world is where it cannot.

A corollary worth internalising: **every improvement to the cheap stage shrinks the result.** A stronger classical cue defers less often, but it also has less to detect, narrowing the gap between the learned gate and periodic invocation. This is why the cheap stage is deliberately plain sparse LK and not something cleverer.

---

## 2. Theory

### 2.1 Brightness constancy and the aperture problem

Assume a point's brightness is unchanged between frames:

```
I(x, y, t) = I(x + δx, y + δy, t + δt)
```

First-order Taylor expansion gives the optical flow constraint equation:

```
Ix·u + Iy·v + It = 0
```

One equation, two unknowns per pixel. This is the **aperture problem**: from a local patch you recover flow only *along* the image gradient, never perpendicular to it.

Two consequences that shape everything downstream:

- **No gradient → no flow.** Textureless regions are unrecoverable in principle, not just in practice.
- **One-directional gradient → ambiguous flow.** Straight edges give flow across but not along, which is why thin structures (poles, wires) defeat this stage.

### 2.2 Lucas-Kanade and the structure tensor

LK assumes flow is constant over a small window, producing an overdetermined least-squares system:

```
⎡ Σ Ix²    Σ IxIy ⎤ ⎡u⎤     ⎡ Σ IxIt ⎤
⎢                 ⎥ ⎢ ⎥  = −⎢        ⎥
⎣ Σ IxIy   Σ Iy²  ⎦ ⎣v⎦     ⎣ Σ IyIt ⎦
```

That 2×2 matrix is the **structure tensor**. It is invertible only when both eigenvalues are large — gradient in two independent directions — which is the definition of a corner. `goodFeaturesToTrack` scores candidates by the smaller eigenvalue (Shi–Tomasi) and keeps the strongest.

**Consequence: per-sector feature counts are literally a map of where this matrix is well-conditioned.** That is why feature count is a meaningful confidence signal and not an arbitrary statistic — it is the reason the gate has anything to read.

Measured in this project: ~27% of tracked points come back anti-radial with magnitude comparable to the well-behaved majority. This is generic LK mistracking — a patch matched onto a similar-looking neighbour. Not position-dependent, not geometric. It is the reason every per-sector aggregation must be robust (median or percentile), never a mean.

### 2.3 The motion field

Camera translating **T**, rotating **ω**, viewing a point at depth Z, in normalised coordinates `x = (px − cx)/fx`:

```
u = (−Tx + x·Tz)/Z  +  [ xy·ωx − (1+x²)·ωy + y·ωz ]
v = (−Ty + y·Tz)/Z  +  [ (1+y²)·ωx − xy·ωy − x·ωz ]
    └── translational ──┘  └────────── rotational ──────────┘
```

Three structural facts, each driving a design decision:

**(a) The rotational term contains no Z.** Rotation displaces every pixel identically regardless of depth — zero distance information, pure contamination. But it depends only on **ω** and pixel position, both known, so it can be computed and subtracted. This is why `FramePacket` carries gyro alongside the image, why the timestamp synchronisation work mattered, and why the gimbal-rigidity question was not pedantry.

**(b) Translation appears only as T/Z.** "Close and slow" is indistinguishable from "far and fast." Monocular scale ambiguity is a property of projection, not a solvable problem.

**(c) The (1+x²) amplification.** At this camera's intrinsics the frame edge sits at x = 320/205.47 = 1.56, so (1+x²) = 3.4. Rotational flow is 3.4× stronger at the edges, so small gyro or timing errors leave disproportionate residuals in peripheral sectors.

### 2.4 Focus of expansion

With **ω** = 0 (post de-rotation), the flow field radiates from a single point:

```
FOE = (Tx/Tz, Ty/Tz)
```

This is **the heading direction projected into the image**. It depends only on **T** — not on Z, not on which surface a point sits on. Every point in the scene radiates from the same FOE. For level forward flight, Tx = Ty = 0, so the FOE is at image centre.

**A hard-won correction:** early in the project the FOE was conflated with the horizon's vanishing point, leading to a wrong hypothesis about ground points. The horizon is a property of the ground *plane*; the FOE is a property of *motion*. Measurement settled it — ground points align with the outward-radial model as well as or better than sky points (mean cos 0.465 vs 0.382).

### 2.5 Time to contact

For a point at radial distance r from the FOE, de-rotated flow magnitude is:

```
|u| = r·Tz / Z     →     τ = Z / Tz = r / |u|
```

**Z and Tz appear only as a ratio.** You recover a *time* — seconds until contact — from pure image measurements, without knowing distance or speed. This is Lee's tau theory (1976), and it is why monocular scale ambiguity is irrelevant to the controller.

Equivalently via divergence: `∇·u = 2·Tz/Z = 2/τ`.

**Two estimators, not interchangeable:**

| | Radial `r/\|u\|` | Divergence `2/(∇·u)` |
|---|---|---|
| Nature | per-point | field property, fitted over a patch |
| Near FOE | ill-conditioned (0/0) | well-conditioned |
| Sensitivity to mistracking | high per point | reduced by fitting |
| Needs how many points | few | many (3-parameter fit) |

The project initially used divergence and it failed (§8.3). Per-point τ with a low percentile is the shipped estimator.

### 2.6 Trajectory relevance and the collision cone

Low TTC is not collision risk. A tree passing 5 m to the left has low TTC and threatens nothing. The quantity that matters is **miss distance**.

A point at normalised coordinate x with depth Z has lateral offset X = x·Z from the flight axis. The drone travels along the optical axis, so when it reaches depth Z — at time τ — the point is still X to the side:

```
miss distance = x · Z = x · Tz · τ
```

Collision threat when `|x| · Tz · τ < r_safe`.

Note what this requires: Tz, forward speed, which the autopilot provides. **Flow alone is scale-free, but flow plus known speed recovers metric depth** (Z = Tz·τ) and therefore metric miss distance.

Formal framing: the collision cone (Chakravarthy & Ghose, IEEE SMC-A 1998, from IISc Bangalore; extended to 3-D in 2012). Standard machinery, cited not claimed.

**Angular half-width at depth Z:** `θ = arctan(r_safe / Z)`. With r_safe = 0.5 m: 2.9° at 10 m, 5.7° at 5 m, 14° at 2 m. Sector width in this project is ~23–34°. So at 10 m the genuinely threatening region is a small fraction of the centre sector — fixed sectors dilute the signal at exactly the range where early warning matters. Hence cone-weighted features for the gate (soft weight `w = 1/(1 + (miss/r_safe)²)`), while the controller keeps sectors.

---

## 3. Architecture

### 3.1 The organising principle

**The only thing that varies across the entire experiment matrix is one swappable object — the Gate.** Everything else is fixed infrastructure.

```
┌─ Offline (laptop, no robot) ──────────────────┐
│  labeller → gate trainer → evaluator          │
└───────────────▲───────────────────────────────┘
                │ reads logs
┌─ Runtime ─────┴───────────────────────────────┐
│  Orchestrator (perception scheduler)          │
│     ├── FrameSource        (platform)         │
│     ├── CheapStage         (always runs)      │
│     ├── FeatureExtractor   (cheap internals)  │
│     ├── Gate               ← THE VARIABLE     │
│     ├── HeavyStage         (runs if deferred) │
│     ├── Controller         (single impl)      │
│     ├── VehicleInterface   (platform)         │
│     └── Logger                                │
└───────────────────────────────────────────────┘
```

### 3.2 The two load-bearing abstractions

**A common perception output type.** Both stages return the same object, so the controller cannot tell them apart:

```python
@dataclass
class SectorBelief:
    scores: np.ndarray        # per-sector traversability, higher = safer, [0,1]
    valid: np.ndarray         # per-sector bool — False when too few points to estimate
    confidence: float
    source: str               # "cheap" | "heavy"  (logging only)
    latency_ms: float
```

The `valid` mask is structural, not cosmetic. "This sector is dangerous" and "I have no idea about this sector" are different states, and collapsing them destroys exactly the distinction the gate exists to exploit. Invalid sectors carry NaN, never a fabricated number.

**The Gate as sole experimental variable.**

```python
class Gate(Protocol):
    def decide(self, feats, state, t_capture) -> tuple[bool, GateState]: ...
```

All six evaluation arms are implementations of this one interface. Evaluation becomes a loop over gate configs rather than six forked code paths — the comparison is *structurally* fair rather than fair-because-someone-was-careful.

### 3.3 Why the controller must be shared

This is the single most important architectural constraint, and it is worth stating in full because it is not obvious.

Gate labels can be generated by comparing the two stages: if cheap chose sector 2 and heavy chose sector 4, that frame is labelled `defer = 1`. The logic that makes this meaningful is: *same controller, same input shape → any difference in output must come from the perception.*

Give heavy its own controller and this breaks. Imagine an empty corridor: both stages perceive "clear ahead" identically, but cheap's controller holds sector 2 via hysteresis while heavy's re-targets to sector 3 because of a different goal weighting. Different sectors. Labelled `defer = 1`. Nothing was wrong.

Repeat across thousands of frames and a chunk of the positive labels are "the two controllers had different hysteresis states." The gate then learns to fire on empty corridors, wasting exactly the compute it exists to save.

**And it is unrecoverable.** The label does not record *why* they disagreed, so controller-caused disagreements cannot be filtered out afterward. The contamination is baked in and would require re-collecting everything.

One controller means every disagreement is a perception disagreement, by construction.

### 3.4 Platform abstraction

```python
class FrameSource(Protocol):
    def read(self) -> FramePacket | None: ...

class VehicleInterface(Protocol):
    def send(self, cmd: ControlCommand) -> None: ...
    def state(self) -> VehicleState: ...
```

Sim and real differ *only* in which concrete class is constructed. The vehicle side barely changes at all — MAVLink to SITL and MAVLink to a real flight controller are the same protocol. The real-hardware port is essentially one new `FrameSource`.

Three implementations exist: `SimFrameSource` (Gazebo + MAVLink), `LogFrameSource` (replay from disk), and `RealFrameSource` (not yet written).

`LogFrameSource` matters more than it looks: it means offline analysis runs through the identical code path as flight, which structurally eliminates train/serve skew in the gate features. Its array-identity guarantee is explicitly tested — the array a caller receives must be byte-identical to what `SimFrameSource` produced for that frame.

### 3.5 Two operating modes, one code path

- **Collection mode** — both stages run every frame, heavy drives the vehicle, everything logs. Produces the training dataset.
- **Deployment mode** — cheap runs every frame, the gate decides, heavy runs only when deferred. Produces the evaluation.

Same orchestrator, one flag. The logged record type is identical in both, which is what lets the labeller and evaluator share code.

### 3.6 Cross-cutting rules

- **Single-source feature extraction.** The `FeatureExtractor` used at training time must be the same object used at runtime.
- **Synchronous heavy inference.** Depth could run async with stale reuse, but latency is the thing being measured — the loop stalls honestly. Async is future work, not an architecture decision.
- **Config-as-experiment.** One YAML defines an arm. A run is `run(config)`.
- **Determinism.** Seed everything. Matched seeds across arms enable paired comparison, which matters when collision rates are low and run counts modest.
- **No wall clock in control logic.** RTF has ranged 0.15–1.25 across sessions; wall-clock timing would make arms non-comparable.

### 3.7 Repository layout

```
obst_avoidance/
  frame_source/   packet.py  base.py  sim.py  log.py
  perception/     types.py  geometry.py  cheap.py  heavy.py  features.py
  control/        types.py  sector.py
  platform/       vehicle.py
  runtime/        orchestrator.py
  gating/         (Phase 6 — to build)
  record_pass.py
worlds/
  env_forest.sdf          # original, retained
  env_zones.sdf           # current evaluation world
  env_zones.launch.py
tools/
  gen_textures.py  check_depth.py  flow_explore.py
  ttc_sectors.py  verify_cheap_stage.py  diagnose_tracking.py
```

---

## 4. The cheap stage

### 4.1 Job

Every frame, at ~12 ms: produce (a) per-sector traversability with a validity mask, and (b) a 168-element feature vector describing how much that estimate can be trusted.

The stage has **two independent detection channels**: sparse LK optical flow (the original, validated path) and a contour/silhouette looming channel added later. They fail differently, which is the point — see §4.6.

Output (b) is as important as (a). The gate never sees depth — it sees only what the cheap stage knows about its own reliability. The cheap stage is therefore not merely a perception module; **it is the sensor the gate reads.**

### 4.2 Why flow

1. **Cost.** ~8 ms versus 1256 ms for depth on CPU. That ratio is the economic basis of the paper.
2. **Scale-free.** TTC is a time, recoverable without knowing distance or speed.
3. **Failures announce themselves.** Flow needs corners; a blank wall returns *fewer tracked features* rather than confident wrong answers. That self-reported degradation is what the gate learns to read. A cue that failed silently would be useless here.

### 4.3 Pipeline

```
FramePacket (image, gyro, t_capture, seq, gyro_valid)
  1. Grayscale
  2. goodFeaturesToTrack (Shi–Tomasi)
  3. calcOpticalFlowPyrLK
  4. Three-stage outlier rejection:
       (a) vignette: reject pixel-y < 100 (rotor blur / gimbal housing —
           visually confirmed to be drone body, not scene)
       (b) reject LK error top quartile
       (c) reject flow magnitude > 3× that pair's median
  5. DE-ROTATE using packet.gyro and intrinsics
       u_rot = xy·ωx − (1+x²)·ωy + y·ωz
       v_rot = (1+y²)·ωx − xy·ωy − x·ωz
       Axis mapping (body-NED → camera optical), determined empirically:
         wx_cam = +pitchspeed,  wy_cam = +yawspeed,  wz_cam = +rollspeed
  6. Bin points into sectors (N_SECTORS = 11)
  7. Per sector: per-point τ = r/|u|, FOE at image centre,
     aggregate by 20th PERCENTILE (not median — see §8.3)
     If point count < MIN_POINTS_PER_SECTOR (10): valid = False, τ = NaN
  8. Temporal smoothing (EMA) on per-sector τ
  9. geometry.py: miss distance, cone weights

  CONTOUR CHANNEL (parallel, independent of steps 2-8):
  C1. Smooth, find closed contours
  C2. Filter by area (too large = the world, not an obstacle)
  C3. Track region identity across frames (centroid + area proximity)
  C4. Windowed line fit to area → dA/dt → τ_area = 2A / (dA/dt)
  C5. Assign each region to the sector containing its centroid

  MERGE: both valid → min; one valid → use it + flag source;
         neither → valid = False
     ├─→ SectorBelief → controller
     └─→ 168-element feature vector → gate
```

### 4.4 The feature vector

`FEATURE_SPEC` is the documented source of truth. Groups:

| Group | Features |
|---|---|
| TTC | per-sector τ, per-sector valid mask, min τ, τ variance across sectors |
| Centre-weighted | statistics over centre sectors only, not a global min — outer sectors are dominated by sky and open ground |
| Cone-relative | min τ among cone-weighted points, effective point count in cone, cone-region vs full-frame feature density |
| Count + magnitude | per-sector feature count **and** mean flow magnitude (sky = few points + low flow; wall = few points + high flow — same count, opposite meaning) |
| Ground fraction | per-sector fraction of points below the horizon (see §4.7 — contamination is far higher than first measured; fed to the gate as a distrust signal rather than masked) |
| Temporal | d(feature count)/dt, d(τ)/dt |
| Structure | per-sector gradient energy, separate from corner count |
| Global | total feature count, its rate of change, across-sector τ variance |
| Tracking health | LK residual, fraction of features lost |
| Validity | gyro_valid, odom_valid |

**Why the list is this rich:** Step O2 found that raw per-sector feature count does *not* discriminate danger — Zone D (mostly textured) scored a lower per-frame minimum (7.9) than the textureless Zone B (8.3). The reason is a conflation: few features because *nothing is there* (empty sky, zero danger) looks identical to few features because *something untrackable is there* (blank wall, maximum danger). Every added feature group answers that conflation.

This paid off: the post-O2 features (τ rate, gradient energy, cone effective count, centre-region τ) carried **77% of the weight mass** of the gate trained on the 102-feature vector.

The vector later grew 102 → 168 with the contour channel's six new per-sector features. Pre-contour features are kept as a stable, unchanged prefix, so the two models can be compared feature-for-feature.

### 4.5 A note on Canny versus silhouettes

Two different ideas get conflated here, and only one works.

**Canny edge detection does not help.** Canny is thresholded gradient magnitude — a surface with no intensity gradient has no edges either, so it adds nothing where flow already fails. Worse, an edge is precisely the rank-deficient case for the structure tensor: gradient in one direction, none in the other.

**Silhouettes and looming do help.** A grey box against textured background has a *boundary*, and a closed region's area growth gives time-to-contact with no interior texture required:

```
τ_area = 2A / (dA/dt)
```

This is the published "growth in feature sizes" cue, and it is what the contour channel (§4.6) actually implements. The distinction matters: edge pixels are useless, closed regions are not.

### 4.6 The contour / looming channel

Added as a pure add-on — `perception/contours.py`, with the LK path byte-identical when disabled.

**Method.** Smooth, find closed contours, filter by area (regions covering most of the frame are the world, not obstacles), track region identity across frames by centroid and area proximity, and fit area over a short window to get `dA/dt`. The slope comes from a windowed line fit, never from raw consecutive differences — blob area is noisy, and this is the same lesson the τ estimator taught (§8.3).

**Merge rule.** Both channels valid → take the minimum (nearest thing wins, same logic as the 20th-percentile choice). One valid → use it and record which in the feature vector. Neither → `valid = False`.

**Results:**

| Metric | Effect |
|---|---|
| boxB1 frames with a valid sector | 64 → 225 (3.5×) |
| boxB1 detection onset | 11.7 → 5.5 m |
| Zone B valid rate | 0.5% → 2.5% |
| Zone A valid rate | 9.2% → 17.4% |
| Zone D valid rate | 0.1% → 1.7% |
| LK correlation on textured approach | +0.522 → +0.522 (unchanged — path not degraded) |
| Latency | 10.45 → 11.69 ms (+1.24 ms) |
| Rotation-only false looming | 3.54% |

**The honest limitation: `τ_area` does not track distance** (correlation −0.076 against ground truth). The channel restores *detection* where LK sees nothing, but its TTC *number* is not yet reliable.

This has a direct consequence for the merge rule. `MIN(τ_lk, τ_area)` treats both as comparable quantities when one is not validated, so an unreliable contour τ can be taken as the minimum and injected into the controller's scores. Two resolutions, unresolved at time of writing:

- **Fix `τ_area`** — the candidates are area-estimation noise versus contour identity swapping between frames. Identity swaps would mean `dA/dt` differences the areas of unrelated blobs, which no amount of window fitting recovers. The decisive test is comparing measured area against the *geometric* prediction: a 1.0 m box at distance d subtends a predictable pixel area given fx = 205.47.
- **Make the channel detection-only** — a contour in a sector with no LK support sets `valid = False` with an explicit "detected, unmeasured" flag; `τ_area` never enters `scores`; contour statistics stay in the feature vector for the gate. The controller then avoids the sector because it is *unknown*, not because of an unreliable number. This is more honest about what was measured, and a signal with 3.54% false looming arguably belongs in the gate's inputs rather than the control loop.

**The feature `tau_agreement` is worth watching.** Where both channels produce a τ, their disagreement is itself an unreliability signal. If the retrained gate assigns it meaningful weight, that is a result worth writing up.

### 4.7 Ground contamination — a correction

An early measurement put ground-plane contamination of TTC at 5–15% and concluded masking was unnecessary. **That figure was wrong.** Remeasured properly:

- **42.7%** of tracked points fall below the horizon
- **60.9%** of the danger-setting 20th-percentile τ tail comes from ground points
- **86%** on the boxB1 approach specifically

So the cheap stage's most safety-critical number is majority terrain, not obstacle. Flow-based TTC has no semantics — it reports time-to-contact for whatever a point sits on, and at 3 m altitude with a level camera, roughly half the frame is ground the drone will never hit.

**Masking does not fix it.** A geometry-derived row band (rows 162–318, identical to the one HeavyStage uses) was implemented and tested: correlation on the textured approach *worsened* (+0.522 → +0.451) and valid rate collapsed (9.3% → 2.5%). The cheap stage is too point-starved to afford the crop. The band ships **off**.

The response is therefore what already existed: per-sector ground fraction is a *gate feature*, so the model can learn to distrust ground-dominated sectors rather than having them removed. This is a real, measured limitation worth stating in the paper — *flow-based TTC on a low-altitude platform is majority-terrain-driven, and masking is not affordable at this point density.*

Note the asymmetry with the heavy stage, which *does* use a row band successfully (§6.2). Dense depth has pixels to spare; sparse flow does not.

### 4.8 Documented failure modes

These are not defects to fix. They are the phenomenon the gate exists to detect, and all are documented in the published literature (which means they can be cited rather than demonstrated from scratch).

| Failure | Cause | Contour channel helps? |
|---|---|---|
| **Textureless surfaces** | No gradient → structure tensor singular → no LK solution | Partly — silhouette against textured background is detectable |
| **Thin structures** | Aperture problem — gradient in one direction only | No — too small to form a stable region |
| **Head-on approach (FOE blind spot)** | Point at FOE has r = 0, so \|u\| = 0 regardless of Z. **An obstacle dead ahead produces no flow at its centre.** | Yes in principle — area growth is strongest head-on, which is exactly where flow is weakest |
| **Low speed** | \|u\| ∝ Tz — slow flight means sub-pixel flow buried in LK's ~0.1–0.3 px noise floor | Partly — area change is also proportional to speed, but is a lower-frequency measurement |
| **Proximity** (discovered here) | Textured obstacles become untrackable up close as texture scale exceeds the LK window and local contrast collapses | Yes — a close object has a large, easily segmented silhouette |
| **Terrain contamination** (§4.7) | Flow TTC has no semantics; ground points produce genuine low τ for a surface the drone never hits | No — orthogonal problem |

The complementarity in that last column is the design argument for two channels: **flow is strongest where looming is weakest (lateral, distant, textured) and looming is strongest where flow is weakest (head-on, close, untextured).**

The FOE blind spot deserves emphasis: the most trajectory-relevant points are the least measurable, because the collision cone sits at the FOE where the radial estimator degenerates. The cheap stage is structurally weakest in the direction the drone is flying. That is a *principled* argument for a gate, not merely an empirical one.

The proximity mode is a genuine contribution and generalises the others: the cheap stage does not fail on "untextured things," it fails on **insufficient trackable structure**, which proximity produces on any surface.

---

## 5. The world

### 5.1 Design rationale

The evaluation world is not scenery — it is experimental apparatus, and two properties are load-bearing.

**Heterogeneity.** Long sparse stretches punctuated by dense clutter zones. This is the condition under which content-aware gating can beat a fixed periodic schedule: periodic wastes budget in empty gaps and under-samples in clutter. A uniformly cluttered world would produce a null result caused by world design rather than by the method.

**Textured by default; textureless as deliberate exception.** Depth Anything V2 does not need texture; optical flow does. The original world had *every* surface flat-coloured, which would have produced near-zero trackable features everywhere, disagreement on nearly every frame, and a gate that learns "always defer" — a degenerate result that would have looked like method failure.

### 5.2 Current layout (`env_zones.sdf`)

~120 m corridor along +x, flight at y ≈ 0, 3 m altitude.

| Zone | x-range | Contents | Texture | Obstacles/10 m | Role |
|---|---|---|---|---|---|
| A | 15–25 | 3 trees + 1 box | bark, leaves, concrete | 4.0 | Flow works — gate mostly quiet; source of compute savings |
| — | gap ~29 m | empty | — | 0 | Periodic wastes budget here |
| B | 50–58 | 1.0×0.4×1.5 m untextured box | flat grey, deliberate | 2.5 | **Texture failure — canonical positive case** |
| — | gap ~25 m | empty | — | 0 | |
| C | 86–94 | 7 objects, ~1.5–2 m gaps | bark, leaves, concrete, stone | 8.75 | **Resolution failure — a different defer reason** |
| — | gap ~19 m | empty | — | 0 | |
| D | 116–122 | textured obstacles + untextured wall | mixed | 5.0 | Discrimination within one zone |

Textures are procedurally generated (`tools/gen_textures.py`) as multi-octave value noise with strong high-frequency grain — deliberately non-periodic, since perfectly periodic patterns cause LK aperture ambiguity and false matches.

**Two different defer reasons matters.** Zone B is perception failure (flow sees nothing). Zone C is resolution failure (flow sits in avoid mode 92.4% of frames and still cannot thread the gaps). A gate that fires in both has learned something more general than a texture detector.

### 5.3 Excluded by design

Thin structures (poles, wires) were removed from flight paths. They defeat *both* stages for related reasons — few corners for flow, poor monocular depth resolution — so **they fail into agreement**: cheap says clear, heavy says clear, disagreement is zero, and the labeller marks `defer = 0`. The gate would learn that wires are safe.

Disagreement-based labelling is structurally blind to shared failure modes. Two mitigations: exclude thin structures from evaluation worlds, and use ground-truth collision labels as a second source (which is what the shipped gate actually uses).

### 5.4 Start-position methodology

Start positions jitter as `y ~ Uniform(−j, +j)` where `j = 0.5·(obstacle_half_y + r_air)`, computed **per zone** from that zone's actual geometry:

| Zone | canonical obstacle | half_y | j |
|---|---|---|---|
| B | box | 0.5 | 0.425 m |
| C | boxC2 | 0.75 | 0.550 m |
| D | narrowD1 | 1.75 | 1.050 m |

Zone A is deliberately excluded — it is off-centreline clutter, a different kind of test.

**Why this matters:** the original fixed ±1.5 m jitter was sized for a 16 m wall that nothing could miss. Against a 1.0 m box it made "never on a collision course" the default outcome, silently converting the collision-rate metric into a measure of start-position luck. Every seed is now verified to intersect the obstacle footprint by construction.

---

## 6. The heavy stage

### 6.1 Current configuration

- Model: `depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf` (VKITTI-trained, max_depth 80)
- Output: **metric depth in metres, larger = FARTHER** (opposite of the relative checkpoints)
- No normalisation code at all — metric output needs none
- Aggregation: 20th percentile of "closeness" within a **row band** (rows 162–318), derived from camera geometry
- Latency: 1256 ms on CPU

### 6.2 Why the row band exists

The camera's 114.6° FOV is wider than VKITTI's training distribution. At the periphery, the model reads sky rows and ground rows as unreliably close — measured row-by-row in an outer sector: 30–40 m in middle rows, but only 7–9 m at the very top and bottom. This produced a "bowl" shape where edge sectors always read as closest regardless of scene content.

Rows below ~320 read a near-constant 6.7–7.2 m across all sectors regardless of the vehicle's true distance — a ground-plane artifact.

Restricting to a geometry-derived row band is not tuning: it excludes pixels the model demonstrably cannot estimate. Sky has no depth to read; ground is surface the drone never hits. Precedent exists on the cheap side, where `VIGNETTE_Y_MIN = 100` excludes rotor-blur rows for the same reason.

**Result:** centre-sector score now flat at 1.0 from 6.5 m to 4.0 m, then declining cleanly and monotonically to 0.009 by 0.5–1.0 m. The bowl shrank from 86.1% to 66.1% of frames but did not disappear — possibly partly real corridor geometry rather than pure artifact.

### 6.3 Known limitations

**Absolute accuracy is poor.** MAE 4.66 m; predictions run 2.5–4.8× true distance, with the ratio *growing* closer in. This is a real VKITTI-to-synthetic domain gap, not a fixable constant offset.

This does not block ranking-based control (the controller ranks sectors and never consumes metres), but it does mean `miss_distance_from_z()` gives unreliable metres for the heavy stage. **Cone weighting therefore stays cheap-only**, for a measured reason rather than a structural one.

**Inference rate is the binding constraint.** At 1256 ms, depth runs at ~0.8 Hz in closed loop. With danger first detectable at ~4 m and flight at 0.8 m/s, the informative window is ~5 s — about 4 depth frames total. See §8.6.

---

## 7. The controller

### 7.1 Pipeline

```
SectorBelief + goal_heading + ControlState
  1. CANDIDATE MASK   valid=False sectors excluded from candidacy
  2. COST             cost_i = w_τ·(1 − score_i) + w_goal·|θ_i − goal_heading|
  3. SELECTION        argmin cost, with hysteresis
  4. MODE             cruise / avoid / blind
  5. COMMAND          yaw_rate = clamp(K·θ_target), fwd_vel constant
       ↓
ControlCommand → 15 Hz republisher → MAVLink GUIDED
```

~100 lines, 0.066 ms per call. VFH (Borenstein & Koren, 1991), deliberately unmodified.

### 7.2 Design decisions and why

**Validity is a mask, not a score.** Scoring invalid sectors zero would make argmin silently pick sector 0 in Zone B instead of entering `blind` mode. The mask forces the unknown case to be handled explicitly.

**The goal term is not optional.** Without it, a controller that deflects 60° and flies into open space forever scores *zero collisions* while completing nothing — a degenerate arm that would look excellent on the headline safety metric.

**Argmin, never interpolation.** A score-weighted centroid seems like it would give smoother steering, but with an obstacle dead centre and clear sectors either side, the centroid of "go left" and "go right" is *straight into it* — the classic potential-field failure. The angular quantisation is a price worth paying.

**Hysteresis holds a WORLD BEARING, not a sector index.** Sector 4 at frame 0 and sector 4 at frame 9 are different world directions if the camera rotated between them. Each frame, the committed world bearing is projected into the current camera frame to find which sector now contains it.

**Hysteresis is time-based, not frame-based.** `hysteresis_min_seconds = 0.12`, evaluated against `t_capture`. Three frames means 0.12 s at 25 Hz and 3.75 s at 0.8 Hz — the same constant is reasonable for one stage and a permanent lock for the other.

**Hysteresis is bypassed when `belief.source` changes.** The whole point of deferring is to get a better answer; suppressing it because the previous cheap answer disagreed defeats the purpose.

**Constant speed across all arms.** This is experimental design, not control. If arms fly at different speeds, collision rate and completion time are both confounded — a crawling flow-only arm would look safer for reasons unrelated to perception.

**Caller-owned state, no wall clock.** State passed in and returned gives paired-seed determinism and allows replaying a logged belief sequence offline to reproduce commands exactly.

**Decoupled 15 Hz republisher.** ArduPilot GUIDED fails safe if setpoints stop arriving, and the heavy stage stalls the perception loop for over a second. Without a separate publisher, intermittent mode drops would look exactly like a control bug.

### 7.3 The through-line

Almost every decision above serves the *experiment* rather than flight performance. Interchangeable stages, identical speed, deterministic state, no wall clock — these exist so that when six arms are compared, the only thing differing is the gate.

---

## 8. Findings log

This is the most valuable section for anyone continuing the work. Most of the project's cost was spent discovering these. They are ordered by theme, not chronology.

### 8.1 Infrastructure and synchronisation

**Three sync bugs, found by measurement rather than assumption.**

1. **ATTITUDE arriving at ~13 Hz, not the requested 50 Hz.** Two compounding causes: `SR0_EXTRA1` does not exist in this ArduPilot build — the parameter is `MAV1_EXTRA1`, defaulting to 4 Hz, and `PARAM_REQUEST_READ` times out *silently* so `param_fetch_all()` plus grep was needed to find the real name. Separately, rate requests decay (~28 Hz falling to ~3 Hz within 5–10 s), most likely mavros re-asserting its own rate on the shared link. Workaround: re-send the rate request every 0.5 s.

2. **Startup clock calibration biased ~240 ms wrong.** The executor was only spun on demand, so during `recv_match`'s blocking wait nothing processed `/clock` and `get_clock()` returned a stale cached value. An earlier "take the minimum of N samples" fix — reasoned from an NTP model where transport latency biases the reading *high* — actually selected the *most stale* sample under this mechanism. Wrong error model, opposite sign. Root cause shared with an earlier freshness bug: on-demand spinning.

3. **Camera publishing at ~8 Hz** because the SDF `update_rate` was 10. Resolution reduction was tested as a render-bottleneck hypothesis and ruled out — the real constraint is RTF / physics load.

**Other infrastructure findings:**

- pymavlink must use port 14551, not 14550 — mavros binds 14550, and sharing it silently killed mavros `/state` updates. The symptom looked identical to executor starvation, which was investigated and ruled out first.
- With `use_sim_time`, a node's clock reads exactly 0 until the first `/clock` message arrives, silently.
- `mode_sent=True` from the SetMode service means only that the request was accepted for sending, not that the FCU switched.
- Nodes must use private executors; the module-level `rclpy.spin_once()` convenience functions spin a shared global executor, and coexisting nodes through it caused `/state` updates to stop entirely.
- A DDS-connection hang has killed runs three times across batches. Environmental flakiness; batch runners must be resumable.

**Final sync state:** mean ~30 ms, rare 400 ms outliers, `gyro_valid=False` on 5–7% of frames, camera 21–26 Hz. Judged against what matters — de-rotation error at fx = 205 and realistic angular accelerations — 30 ms produces well under a pixel per frame, invisible next to LK noise.

### 8.2 De-rotation validation

The axis mapping from body-NED gyro to camera optical frame was determined **empirically**, not by inspection, because the gimbal's mounted orientation involves a compound rotation across several joint frames. Method: sweep all 6 axis permutations × 8 sign combinations (48 candidates), score each by median residual flow magnitude on a pure-yaw recording.

The pure-yaw recording could not discriminate the pitch/roll axes (both near zero throughout a yaw manoeuvre). A dedicated `roll_pitch_wiggle` recording mode resolved it decisively: `wx_cam = pitch` beats `wx_cam = roll` by a 66.4% margin.

**Validation results:** on `yaw_only`, raw flow mean 24.6 px / median 19.2 px reduces to residual mean 11.0 px / median 3.4 px — an 82% median reduction. On `wall_approach`, residual magnitude grows monotonically with radial distance (0.19 → 2.61 px across quintiles, r = 0.54) with median cos to outward-radial of 0.99, confirming a genuine focus of expansion.

The mean stays higher than the median because of the ~27% mistracking population — a real, separate finding, not a sign or axis bug.

### 8.3 The τ estimator inversion — and its fix

**Symptom:** on a textured approach, production EMA-smoothed τ spiked to its *highest* value (19.4 s) at the closest distance, moments before collision. "All clear" when danger was highest.

**Ruled out first:** divergence sign (positive ~90% of the time; negative-divergence points are discarded, not clamped) and EMA smoothing (raw already showed a 465 s outlier; smoothing was helping).

**Root cause:** as the drone closes, the tree canopy becomes *untrackable* — too large, too low-contrast at that scale. The surviving points in that sector sit on a different nearby object. The sector median therefore flips from measuring the obstacle to measuring something else, precisely at closest range.

**Fix:** replace divergence-fit + median with per-point `τ = r/|u|` (FOE at image centre) aggregated by the **20th percentile**.

| Estimator | Correlation with distance | Max τ |
|---|---|---|
| divergence + median | −0.061 | 465 s |
| per-point + median | −0.115 | — |
| **per-point + 20th percentile** | **+0.462** | **5.3 s** |

The percentile, not the per-point switch, does the work. Rationale: collision risk is set by the *nearest* thing in a direction, not the typical thing. A sector with one close trunk and twelve distant background points has a median dominated by background.

### 8.4 Measurement-validity errors

Several results turned out to be measuring something other than what was intended. These are worth cataloguing as a class.

**`wall_approach` could not validate obstacle response at all.** Sector 2 (facing the wall) supplied `min_tau` on 0/123, 0/43, and 0/61 frames across three recordings. Every correlation ever measured on that recording was peripheral scenery, never the wall. The recording was structurally unable to test what it was being used to test.

**"Passed by geometry, not by seeing."** Four of five heavy runs that cleared the Zone B box had 0.81–1.4 m of lateral offset *before* reaching it, unchanged through the approach, with no course correction. The `switches = 1` signature was the tell. Flow-only "clearing" runs showed the same pattern. Root cause was the start-jitter sizing (§5.4).

**An offline test passed because its replay started already close.** Temporal-window normalisation appeared to work offline but failed in flight, because the replay recording began near the wall so its entire window contained wall-only content with real variation. In an actual flight, the window filled with the wall's own near-uniform values.

**A statistics artifact created a phantom anomaly.** A top-quartile *mean* was compared against an overall *median*, producing an apparent anomaly that vanished when redone as like-for-like quartiles.

**Global feature counts hid the per-sector picture.** Zone B's respectable mean of 101 features was ~100 features on textured ground and periphery plus ~1 on the wall directly ahead.

**Every closed-loop number before the controller fixes was produced by a spinning drone.** See §8.5.

### 8.5 Controller bugs

Four, found across two sessions, all in `SectorController`, all the same species: state that was computed once and reused without tracking what the perception layer was actually saying.

1. **Open-loop yaw.** `yaw_rate = K·θ_target` was computed once and reissued unchanged for ~14 s. Since θ_target is a bearing *in the current camera frame*, it should shrink to zero as the drone comes onto heading. Instead the command was constant, so the drone rotated continuously past its target: −38° by frame 3, −88° by frame 6, −132° by frame 9, near-identical across every run. By frames 9–13 all sectors read "safe" simply because the camera no longer pointed at the obstacle. Position kept climbing toward the box through flight-dynamics lag.

2. **Hysteresis latched a sector index across a rotating frame.** Meaningless once the camera turns; also why `switches = 1` in every run — the controller compared against a stale index that no longer referred to anything.

3. **`n_sectors` defaulted to 5 against 11-element beliefs.** Every closed-loop run in the project's history had candidate selection silently restricted to the left half of the frame. Now derived from `len(bearings)` with a `__post_init__` guard.

4. **Frame-based hysteresis applied across stages with 30× different rates.** `hysteresis_min_frames = 3` cost the heavy stage ~3.75 s of confirmation out of a ~5 s informative window. Replaced with `hysteresis_min_seconds`.

**Post-fix behaviour:** heading holds within 0.6–0.7° and 3.4 cm laterally through an entire approach; switch counts rose from 1 to 3–5; headings turn up to 29°.

**Consequence:** all closed-loop collision-rate numbers from before these fixes are superseded. The perception findings hold (they came from offline analysis on recordings), but flight results need re-running.

### 8.6 The heavy stage: four failed approaches and one physical limit

**Attempt 1 — per-frame min/max normalisation.** Scores got *safer-looking* as the wall approached (0.43 → 0.82 → 0.09). When a uniform surface fills the view, everything in frame is close, so after per-frame scaling the wall's sector looks average or far.

**Attempt 2 — temporal-window normalisation.** Also failed: 7/7 collisions. The heavy stage gets only ~40 perception updates in a ~40 s flight, so a 30-frame window is nearly the whole flight, and the wall's own near-uniform close values come to dominate the reference range.

**Attempt 3 — fixed metric calibration from relative depth.** Fitted on a tree approach, evaluated held-out on a wall approach: **R² = −7.98**, worse than predicting the mean. DA-V2 is *trained* to be affine-invariant, so scale and shift genuinely differ per image. A tree-fitted mapping has no reason to transfer to a wall. This is a model-capability mismatch, not a tuning problem.

**Attempt 4 — aggregation changes.** The aggregator was already nearest-biased (80th percentile, not a mean). The box occupies only 6–26% of the centre sector at 4–2 m, so p80 cannot reach its pixels until ~2.3 m. Raising to p90 made things *worse* (sampling noise). Raising sector count from 5 to 11 made no measurable difference — DA-V2's effective spatial resolution is coarser than an 11-way split.

**The root cause of all four:** the wrong checkpoint. Every failure was a *normalisation* failure, and normalisation only existed because the relative inverse-depth checkpoint has no fixed scale. Switching to the metric checkpoint deleted the entire problem class.

**Post-switch:** correlation +0.722 (roughly double any relative variant), latency improved to 1256 ms, and — after the row-band fix — a clean monotonic decline from 4 m.

**The remaining constraint is physical, not algorithmic.** At 0.8 Hz inference, ~4 m detection range, and 0.8 m/s flight, only 2–3 perception updates exist between first danger and impact, giving ~3 cm of achieved lateral displacement against ~0.5 m needed. That is off by a factor of ~16; no parameter recovers it.

**Framing caveat for the paper:** state this specifically. "DA-V2 Metric Small on this CPU at 0.8 Hz cannot avoid a 1 m obstacle at 0.8 m/s given a 4 m detection range" is honest and defensible. "Monocular depth cannot do obstacle avoidance" is not — plenty of published systems do exactly that on GPUs, and a reviewer will say so.

### 8.7 The wide-FOV finding

Worth writing up independently: **metric depth foundation models degrade at the periphery of camera FOVs wider than their training distribution.** Demonstrated here with row-level evidence — 30–40 m in middle rows versus 7–9 m at top and bottom of the same sector, on a 114.6° camera against VKITTI-trained weights.

Most papers deploying these models on drones would not surface this. It is a practical deployment finding and a genuine contribution to the discussion section.

---

### 8.8 A measurement that was simply wrong

Ground contamination was first measured at 5–15% and used to justify not masking. Remeasured later with a better method: **42.7% of points below the horizon, 60.9% of the danger-setting τ tail, 86% on one approach** (§4.7).

Unusually among the errors in this log, this one is not a subtle validity problem — it was a straightforwardly wrong number that stood for weeks and informed a design decision. The decision happened to survive re-examination (masking makes things worse), but that was luck, not vindication.

The lesson is narrower than "measure carefully": **a measurement that justifies *not* doing something deserves the same scrutiny as one that justifies doing it.** The first kind is easier to accept and harder to notice being wrong, because nothing downstream changes to expose it.

### 8.9 Asymmetric fixes between the two stages

The row band works for the heavy stage and fails for the cheap stage — same geometry, same derivation, opposite outcome. Dense depth has pixels to spare, so cropping costs nothing; sparse flow is point-starved, so cropping costs valid sectors (9.3% → 2.5%).

Worth internalising as a general point: **a fix validated on one stage does not transfer to the other merely because both are "perception."** Their data densities differ by orders of magnitude, and several project decisions (aggregation choice, masking, sector count) fall out differently for that reason alone.

---

## 9. The gate — Phase 5 results

> **Status note.** Everything in this section describes the model trained on the **102-feature** vector, before the contour channel existed. It is the validated result at time of writing. The retrain on 168 features (§10.2a) will supersede these numbers, and the same validation protocol applies unchanged.

### 9.1 Method

**Labels.** Ground-truth collision, not stage disagreement:

```
label = 1  if the run collided within ~2 s of flight after this frame
label = 0  otherwise
```

The N-frame window is computed from `t_capture`, not a frame count, because runs differ in perception rate.

This label source is *stronger* than disagreement. Disagreement is a proxy for "cheap is wrong"; ground-truth collision is the thing itself. It is also immune to the shared-blind-spot problem (§5.3), and — critically — it does not require the heavy stage to work, which is what unblocked Phase 5 while the heavy stage was still failing.

**Features.** The 54-element cheap-stage vector. The gate sees only cheap features — never depth, never ground truth. It must decide *before* depth would run.

**Model.** Logistic regression, asymmetric loss weighting false negatives 5–10× (a missed obstacle is a crash; a wasted depth call is milliseconds). Held out **entire runs**, never random frames — consecutive frames are near-duplicates and a random split leaks badly.

Note: superseded runs (those flown with the controller bugs) are still valid *training* data. The bugs affected trajectories, not the cheap-stage features computed per frame, and the features are what the gate reads.

### 9.2 Headline result

**ROC AUC 0.900** on held-out runs.
**Heuristic baseline** (hand-tuned threshold on valid-sector count plus min τ): **0.735**.

The gap between them is the contribution, quantified. A sensible threshold gets most of the way; learning gets substantially further.

### 9.3 Leakage audit

Every run in the dataset ended in a collision, so "frames labelled 1" and "frames near the end of a run" are nearly the same set. The model could have been detecting *time elapsed* rather than *danger*. Three independent checks say it is not:

1. **Positional-only models are near chance.** `time_since_takeoff` 0.566, `frame_index` 0.568. Structurally, run lengths span 4.5–117 s, so the positive region sits at a different absolute time in each run and no clock separates them across runs.
2. **Removing all 10 monotonically-drifting features changes AUC by +0.0005** (0.9001 → 0.9006). If the result rode on drift, this would gut it.
3. **Per-zone AUC is non-uniform and strongest in the hardest zone** — B 0.953, D 0.858, C 0.800. Scene-shaped signal, not a global clock.

### 9.4 What the gate learned

Post-O2 features — τ rate of change, gradient energy, cone effective count, centre-region τ — carry **77% of the total weight magnitude**.

This closes a narrative arc: Step O2 found raw per-sector counts do not discriminate danger, richer features were added specifically to address that conflation, and the trained model independently agrees those are where the signal lives.

### 9.5 Operating point

At the asymmetric-loss threshold p* = 0.125:

- **Deferral rate 26.5%**
- Recall 0.909, precision 0.134
- **Implied periodic baseline k = 3.8** (Phase 6 must run the heavy stage every ~4th frame to match the gate's budget)

Low precision is expected and correct given the asymmetric loss — the gate is deliberately biased toward deferring.

### 9.6 The honest limit

- **Leave-one-run-out: 0.913 ± 0.116** (pooled 0.900)
- **Leave-one-ZONE-out: collapses.** Test zone_B (trained on C+D) = **0.512 — chance**. zone_C = 0.658. zone_D = 0.816.

The gate does not transfer to a failure mode it never trained on, and it is worst on the untextured-wall case that motivated the project.

**This should be stated up front, not buried.** It is the same lesson the introspection literature carries — failure predictors learn *specific* failure signatures, not a general notion of "my perception is unreliable" — made concrete and quantified for a routing application.

The deployable claim:

> Introspective gating generalises across runs but not across failure modes. A deployed gate's training distribution must cover the failure types it will encounter.

That implies a concrete design rule: a real deployment must *deliberately collect* textureless, thin-structure, and high-density cases rather than hoping they generalise.

---

## 10. Current status and the forward plan

### 10.1 Status

| Phase | State |
|---|---|
| 0 — Depth kill switch | Complete, passed |
| 1 — World, camera, sync, recorder | Complete |
| 2 — Cheap stage + controller | Complete, validated |
| 2b — Contour channel (add-on) | Built; merge rule unresolved (§4.6) |
| 3 — Heavy stage | Built; blocked on CPU inference rate |
| 4 — Collection mode | Not needed for the shipped label source |
| **5 — The gate (102 features)** | **Complete and validated, AUC 0.900** |
| **5b — Retrain on 168 features** | **Pending — the immediate next step** |
| 6 — Six-arm evaluation | Harness to build; needs GPU |

~100 hours invested. Two of three project risks retired; the third (is deferral predictable?) is now answered affirmatively for the 102-feature model.

### 10.2 The two blockers

**(a) The gate retrain, and what it actually tests.** The contour channel changed the feature vector 102 → 168, so the validated AUC 0.900 no longer describes the running system. The retrain must repeat the *entire* CW–CZ protocol: leakage audit, weight interpretation, operating point, leave-one-run-out, leave-one-zone-out.

One caution up front: **the pooled AUC will probably rise simply from having 66 more features on the same data.** That is expected and not very informative. The number that decides whether the contour work earned its place is **leave-one-zone-out** — specifically whether Zone B moves above 0.512 (chance). If it does, the new features help the gate generalise to an *unseen* failure mode, which is a strong result. If it stays at chance, they fit the seen failure modes better without transferring, which is a different and more sobering finding.

The merge rule (§4.6) should be settled before the retrain, since it changes what `scores` contains and therefore what several features mean.

**(b) GPU access.** Everything after that is the six-arm evaluation, and three of six arms need the heavy stage running at a usable rate. On a GPU, DA-V2 Small runs at 30–50 ms rather than 1256 ms — 25–40× more perception updates in the same window, which turns 3 cm of achieved lateral displacement into metres.

A friend's machine with an NVIDIA card is the current plan. Practical questions to settle: which GPU, how much VRAM (DA-V2 Small needs little), and whether it runs Linux with ROS 2 installable — the Gazebo + SITL + mavros stack is the harder dependency, not the model.

If the full stack cannot run there, the fallback is to run only the depth benchmark on that machine and do sim elsewhere.

### 10.3 Phase 6 — the evaluation

**Six arms, one Gate interface, nothing else changing between them:**

| Arm | Purpose |
|---|---|
| Always-depth | Safety ceiling |
| Flow-only | Compute floor |
| Periodic every-k (k = 4) | **The arm that decides the paper** |
| Random at matched rate (p = 0.265) | Proves it is the *learning*, not the *rate* |
| Hand-tuned heuristic | Pre-empts "would a threshold have done?" |
| Learned gate (p* = 0.125) | The contribution |

**Metrics:** collision rate and location, mission completion, % frames deferred, mean and p95 total and per-stage latency, switch count, max lateral deviation from initial heading.

**Design requirements:** matched seeds across arms (paired comparison), constant speed across arms, deferral rate keyed to heavy-stage invocations rather than wall frames, resumable batch runner (the DDS hang has killed runs three times).

### 10.4 The work to do before GPU access

All of this is buildable and verifiable on CPU, so that GPU time is pure execution rather than debugging:

1. **`gating/` module** — all six gate implementations behind one protocol. Verify by grepping for any arm-specific conditional elsewhere in the codebase; there should be none.
2. **Orchestrator wiring** — cheap every frame → gate → heavy if deferred → controller. Log gate decision, belief source, both latencies, running deferral rate.
3. **Config-as-experiment** — one YAML per arm, everything seeded.
4. **Batch runner + aggregation** — resumable; produces the paper's comparison table directly from run rows, with no manual transcription.
5. **Device portability** — `device` config ("cpu" | "cuda"), no hardcoded `.to("cpu")`, device and measured latency printed into every run log so GPU and CPU runs are distinguishable after the fact.
6. **`tools/bench_heavy.py`** — standalone model benchmark. First thing to run on the GPU machine; verifies the environment before any sim time is spent.
7. **Reproducibility** — pinned requirements, a SETUP.md covering ROS 2 distro / Gazebo Harmonic / ArduPilot SITL / mavros / checkpoint ID, and a smoke test that exercises the full stack without flying.
8. **Dry-run the whole matrix on CPU via `LogFrameSource`** — proves the harness, config system, aggregation, and all six gates end to end. Latency numbers will be meaningless; correctness will not. Deferral rates should land near 25% (periodic), 26.5% (random), 26.5% (learned) — if they diverge, the arms are not budget-matched and Phase 6 would be invalid.

### 10.5 Writing

Most of the paper can be drafted now. Final and unblocked: the method, the world design, the cheap stage and its failure taxonomy, the wide-FOV finding, the terrain-contamination finding, and the CPU inference-rate finding.

Pending: the gate numbers (retrain on 168 features) and the six-arm comparison table.

The cheap stage should be described as **two-channel** — sparse LK flow plus contour looming — with the complementarity argument from §4.8: flow is strongest where looming is weakest and vice versa. That is a better story than a single cue with a list of failure modes, and it is honestly what the system now is.

Suggested structure:

1. Introduction — compute-bound monocular avoidance
2. Related work — introspective perception, model cascades, flow-based avoidance
3. Method — two-stage cascade, the shared `SectorBelief` contract, the gate
4. Experimental setup — world design, arms, metrics
5. Results — gate AUC, per-zone breakdown, six-arm comparison
6. Discussion — the generalisation limit, the FOV finding, the inference-rate finding
7. Limitations and future work

---

## 11. How to take this further

### 11.1 Immediate extensions (same paper or a close follow-up)

**HIL energy measurement.** Stream sim camera frames to a real Jetson, measure watts with `tegrastats`. Real millijoules rather than FLOP counts, at zero flight risk. This is what separates "student project" from "paper" in reviewers' eyes, and it also resolves the inference-rate limitation as a side effect.

**Sector-count ablation.** Zone C fails with perception working (92.4% avoid mode, gaps too tight for the current angular resolution). Sector count is a config value both stages read from `geometry.py`, so "collision rate vs. angular resolution" is a cheap extra result.

**Centre/periphery reliability weighting.** The literature (Coombs et al.; Sensors 2019) converges independently on treating central and peripheral flow differently, because an object near the FOE generates less flow than the same object off to the side. A scoring change inside `cheap.py` that acknowledges centre-sector τ is less reliable per unit of measured flow — no interface change.

**Frontal divergence channel.** Divergence over an annulus around the FOE, as an additional *gate feature* rather than a control channel. It is the strongest published signal for the head-on case, which is exactly where the LK channel is structurally weakest. Note the contour channel now partly covers this case (area growth is strongest head-on), so measure whether divergence adds anything beyond it before building it.

**Fix `τ_area`, or prove it cannot be fixed.** The contour channel detects but cannot yet time (§4.6). Making the looming TTC quantitative would turn a detection-only add-on into a genuine second estimator, and it would let the `MIN` merge rule stand on evidence rather than assumption. The decisive experiment is the geometric-area check described in §4.6.

### 11.2 Real hardware

The staged path, in order of increasing risk:

1. **Real data, offline.** Manual flights with camera and IMU logging; rerun the analysis on real footage. Zero crash risk, and the labelling pipeline works identically. Note that DA-V2 is trained on real photographs, so real footage is its *home turf* — sim was the harder domain, and the domain gap that caused MAE 4.66 m should shrink considerably.
2. **Real closed-loop flights.** Companion computer (Jetson Orin Nano or similar) on a quad, foam obstacles, low speed, geofence, safety pilot with mode-switch authority, always-depth as the fallback when the gate is uncertain. Note the failure mode is bounded: a gate that fires too often merely wastes compute, which is the direction the asymmetric loss already pushes it.

**What transfers and what does not.** The entire software stack, the gate architecture, the training procedure, and the evaluation protocol transfer unchanged — swapping `FrameSource` is the whole port. The trained gate *weights* will not transfer: real images differ in texture statistics, lighting, motion blur, and rolling shutter. But retraining is cheap, because collecting real training data requires only manual flights with logging, not a working autonomous system.

The ground-truth collision label is the one thing that does not survive the transition — you cannot crash repeatedly on real hardware. Substitutes: hand-label near-misses from video, or fly manually toward known obstacles and mark the frames where you intervened. Cruder than sim ground truth, but usable. The disagreement label works unchanged on real footage.

### 11.3 The bigger version of this project

The current system is a *measurement instrument*: one shared controller, both stages forced through the same coarse `SectorBelief`, so that disagreement measures perception. That constraint is right for the paper and wrong for a system you would actually fly.

A stronger second project separates them:

- **Cheap stage → reactive avoidance.** Fast, coarse, handles open space.
- **Heavy stage → deliberative planning.** Dense depth, gap detection, trajectory selection through constrained space — not sector scoring.
- **The gate routes between two genuinely different control regimes**, not between two versions of the same one.

That matches the original framing — "cheap handles the easy 90%, heavy exists for the constrained regions where controlled movement is required" — and *"gating between a reactive cue and a deliberative planner"* is a more interesting claim than gating between two sector-scorers. It is plausibly RA-L or ICRA territory with real flights.

One cheap thing to do now that enables it later: **log the dense depth map alongside `SectorBelief`**, or expose it as an optional field the controller ignores. Costs nothing today, and a later deliberative planner will need recordings that already contain it.

### 11.4 Addressing the generalisation limit

The leave-one-zone-out collapse (§9.6) is the most scientifically interesting open problem this project surfaced. Directions:

- **Broader training distribution.** More failure-mode types, deliberately collected. Tests whether the collapse is a data-coverage problem or something deeper.
- **Failure-mode-agnostic features.** Are there cheap-stage signals that indicate unreliability *in general* rather than signalling a specific failure? Tracking residual and confidence-calibration signals are candidates.
- **Uncertainty-based gating.** Instead of predicting failure, predict the cheap stage's *uncertainty* and defer when it is high. This might transfer better across failure modes, since uncertainty is defined without reference to any particular failure cause.
- **Cross-domain transfer.** Train the gate in sim, test on real footage. Directly answers whether sim-trained introspection is deployable.

---

## 12. Working practices worth keeping

Several habits developed during this project turned out to be decisive, and are worth stating explicitly for anyone continuing it.

**Measure before hypothesising.** Nearly every "perception limitation" in this project turned out to be a specific, traceable mechanism — the τ estimator inversion, the start-jitter confound, the wrong checkpoint, the frame-0 sector lock, the wide-FOV artifact, the open-loop yaw. Seven times. Each was found by tracing the mechanism rather than accepting the headline number. Stay suspicious of the eighth.

**Write down the rejected hypotheses, not just the conclusions.** The docstrings in this codebase record theories that turned out to be wrong (executor starvation as an explanation for the UDP port conflict; the NTP-style model behind a min-filter that solved the wrong failure mode). Those records repeatedly prevented re-investigating dead ends.

**Refuse to tune toward a desired result.** `MIN_POINTS_PER_SECTOR` was deliberately left broken rather than nudged until a plot looked good. A correlation that flipped sign based on a preprocessing choice was recognised as noise rather than adopted. This discipline is why the AUC 0.900 is trustworthy.

**Check what a test is actually measuring.** Several results measured something other than what was intended (§8.4). The question "could this recording, by construction, produce the result I am looking for?" should be asked before the analysis, not after.

**Distinguish correction from tuning.** Fixing `goal_heading` to be world-referenced, making hysteresis time-based, and restricting depth aggregation to a valid row band are all *corrections* — each was wrong on its own terms regardless of the result. Nudging a threshold until the obstacle gets avoided is tuning. The difference matters and should be argued explicitly each time.

**Separate the research artifact from the good system.** The constraints that make the experiment measurable (shared controller, constant speed, coarse shared interface) are not the constraints you would choose for a system you actually want to fly. Keeping that distinction clear prevents both crippling the science and over-engineering the instrument.
