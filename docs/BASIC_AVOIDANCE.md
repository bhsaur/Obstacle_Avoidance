# Cheap visible-obstacle demo

This opt-in simulation uses classical image edges and optical flow, without a
neural model or depth. It targets one large upright, visibly contrasting
obstacle. The controller uses silhouette angular bounds; it does not consume
the simulator obstacle map. The map is used only to evaluate collision and
plot clearance. The old optical-flow controller remains the default.

From the package directory, with no other project simulator running:

```bash
source /opt/ros/humble/setup.bash
source /home/saurabh/ardu_ws/install/setup.bash
trial_dir=$(mktemp -d /home/saurabh/ardu_ws/eval_results/basic_demo_XXXXXX)
python3 -m obst_avoidance.autonomous_demo \
  --output-dir "$trial_dir/run" --controller cheap_visible \
  --basic-scene --spawn-y 0 --endpoint-goal --max-wall-time 180
```

This starts Gazebo/SITL, waits for readiness, arms the simulated drone, flies
past the box toward the endpoint, then lands and stops its owned processes.
The camera window shows sectors, LK scores and red detection bounds. Q/Escape
requests stop and landing. Add `--headless` to record without the camera window;
add `--gazebo-gui` to also open Gazebo. Existing output directories are rejected.

The box is 2 m long, 3 m wide and 5 m tall. The route runs from x=10 to x=25
at y=`--spawn-y`; commanded cruise altitude is 3 m. Maximum forward speed is
0.5 m/s and decreases while turning. Success requires arriving within 0.75 m
of the endpoint, not merely crossing the goal line.

Each run saves `config.json`, `result.json`, `frames.jsonl`, annotated
`camera.avi`, unannotated lossy `raw_camera.avi`, and a `video_frames.jsonl`
index with capture times. AVI playback is 25 fps and is not real-time flight
speed. A result counts as success only if `completed` is true and `collided`
is false. Startup failures and timeouts remain failures, separately classified.

```bash
PYTHONPATH=. MPLCONFIGDIR=/tmp/obst-mpl python3 tools/plot_trajectories.py \
  "$trial_dir/run" --grid --out "$trial_dir/trajectory.png"
```

The plotter reads obstacle geometry from the recording metadata. Reported
margin is sampled center-to-obstacle distance minus the modeled 0.35 m drone
radius. It is not a continuous collision guarantee.

## Scope and known limits

Contours and long upright edges supply detections, not metric TTC or free-space
confidence. Invalid LK sectors retain NaN scores. The visual controller can
cruise when no silhouette is detected: that is an explicit limited-scene
assumption, not evidence that unseen space is safe. It remembers detected
angular bounds in world orientation for up to 2 seconds; this memory does not
compensate for translation or measure distance. A single upright edge occupies
the nearer image boundary conservatively, which can be wrong for other scenes.

This is a basic-obstacle milestone, not general navigation, a real-drone flight
controller, or validation of the learned-gating research experiment. Thin,
low-contrast, overhead, moving, and multiple obstacles have not been validated.
Compare this slower, detection-driven variant separately from historical flow
controller runs. See the CHEAP-BASIC handoff for successes and rejected attempts.
