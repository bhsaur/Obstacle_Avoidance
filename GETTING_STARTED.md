# obst_avoidance — Getting Started

Monocular obstacle avoidance for an ArduPilot + Gazebo quadrotor (ROS 2),
built as a research testbed for a **two-stage adaptive-perception** idea:
a cheap optical-flow stage runs every frame, and a learned "gate" decides
when to defer to an expensive monocular-depth stage — trading compute for
safety only when the cheap stage is unsure.

> This is a **research codebase**, not a turnkey product. `README.md` is a
> long chronological research log (historical; read top-down for the story).
> **This file is the practical entry point.**

---

## What's implemented

- **CheapStage** (`obst_avoidance/perception/cheap.py`) — Lucas–Kanade
  optical flow → per-sector time-to-contact, gyro-derotated. Emits a shared
  `SectorBelief` (per-sector traversability score in [0,1], validity mask,
  confidence) plus a 168-element feature vector. Blind on untextured
  surfaces *by design* — it reports `invalid`, never a fabricated number.
  An optional contour/looming channel adds detection-only cues.
- **HeavyStage** (`obst_avoidance/perception/heavy.py`) — Depth Anything V2
  (metric) monocular depth → the same `SectorBelief`. Sees untextured
  obstacles the cheap stage can't. **~4–5 s/frame on CPU** (needs a GPU to
  be usable closed-loop — see Limitations).
- **SectorController** (`obst_avoidance/control/sector.py`) — a deliberately
  simple VFH steering law with world-referenced target commitment and
  time-based hysteresis. Selectable goal source: fixed corridor bearing
  (`baseline`) or a pure-pursuit reference path (`pathtrack*`).
- **Learning-to-defer gate** (`tools/gate_*.py`) — logistic-regression gate
  over cheap features, trained/validated **offline** (leave-one-run-out
  ROC-AUC ≈ 0.90, beating a hand-tuned threshold ≈ 0.74).
- **Evaluation tooling** — `tools/plot_trajectories.py` (top-down paths,
  reaction-timing, clearance vs. distance), `obst_avoidance/autonomous_demo.py`
  (one bounded closed-loop flight with a live camera+sector overlay),
  `obst_avoidance/live_viewer.py` (passive monitor / recording replay).

## Limitations (read before drawing conclusions)

- **HeavyStage is CPU-latency-limited here** (~0.2 Hz). Closed-loop heavy /
  gated flight needs GPU (e.g. a Jetson at 30–50 ms/frame); on CPU it is
  only meaningful for **offline** perception comparison.
- The **gate is validated offline only** — not yet closed-loop / matched-budget.
- Closed-loop avoidance is demonstrated on **single, large obstacles**
  (see `docs/BASIC_AVOIDANCE.md`), not general navigation.
- Collision/obstacle geometry is used **only for evaluation**, never inside
  perception or control.

---

## Prerequisites

- **Ubuntu 22.04 + ROS 2 Humble**
- **Gazebo (gz) Harmonic** with `ros_gz_sim`
- **ArduPilot SITL** + **`ardupilot_gazebo`** + **`ardupilot_gz_bringup`**
- **MAVROS** (`ros-humble-mavros*`) and its GeographicLib datasets
- Python: `numpy opencv-python scipy matplotlib pymavlink`
  (offline tools) and, only for HeavyStage, `torch transformers`
  (the depth model auto-downloads on first use).

This package assumes it lives in a colcon workspace (e.g. `~/ardu_ws/src/`)
alongside the ArduPilot/ros_gz packages it depends on.

## Build

```bash
cd ~/ardu_ws
colcon build --packages-select obst_avoidance --symlink-install
source install/setup.bash            # source in every new shell
```

## Run

**1) Launch the simulator** (Gazebo world + ArduPilot SITL + MAVROS):
```bash
ros2 launch obst_avoidance env_zones.launch.py use_gui:=true spawn_x:=45.0 spawn_y:=0.0
# use_gui:=false runs headless (faster). Wait for EKF alignment before flying.
```

**2) Fly one bounded autonomous run** (own module; launches its own sim):
```bash
python3 -m obst_avoidance.autonomous_demo \
    --output-dir eval_results/my_run --zone zone_A --spawn-y -3.8 \
    --controller baseline            # or pathtrack / pathtrack_v5
    # --perception heavy             # depth stage (slow on CPU)
    # --headless                     # no live window
```
Outputs a run dir: `frames.jsonl` (per-frame log), `result.json`,
`camera.avi` (annotated) + `raw_camera.avi`, `config.json`, `simulator.log`.

**3) Judge the result visually:**
```bash
python3 tools/plot_trajectories.py eval_results/my_run --out traj.png --grid
python3 tools/plot_trajectories.py eval_results/my_run --out timing.png --timeseries
```

**4) Passive live monitor / replay a recording** (no flight commands):
```bash
python3 -m obst_avoidance.live_viewer                       # live monitor
python3 -m obst_avoidance.live_viewer --recording recordings/boxB1_approach
```

**5) Offline tests** (no sim needed):
```bash
cd ~/ardu_ws/src/obst_avoidance
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
  test/test_sector_controller.py test/test_orchestrator.py \
  test/test_score_normalization.py test/test_reference_path.py
```
(`test/test_log_frame_source_identity.py` needs a live sim; not part of the
offline suite.)

---

## Repository layout

| Path | What |
|------|------|
| `obst_avoidance/perception/` | CheapStage, HeavyStage, features, geometry, contours |
| `obst_avoidance/control/` | SectorController (VFH), reference-path goal providers, types |
| `obst_avoidance/runtime/` | Orchestrator (perception→control→vehicle loop), logging |
| `obst_avoidance/platform/` | MAVROS vehicle interface |
| `obst_avoidance/frame_source/` | live sim camera + recording replay |
| `launch/`, `worlds/`, `models/` | Gazebo worlds, drone model, textures |
| `tools/` | eval/plot/gate scripts (analysis, offline) |
| `test/` | offline unit tests |
| `docs/` | `BASIC_AVOIDANCE.md`, status, and per-change handoffs |

## Not included in the repo (git-ignored)

Raw research data and build artifacts are intentionally **not** committed:
`recordings/`, `eval_results/`, `build/`, `install/`, `log/`, `logs/`,
`terrain/`, `__pycache__/`. Recordings/eval outputs are large and
machine-specific; regenerate them with the commands above.

## License

Apache-2.0 (see `LICENSE`).
