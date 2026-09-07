# MAVROS startup crash — diagnosis and workspace backport

Date: 2026-09-06. Author: Codex. State: local fix verified; peer review pending.
User report: Ubuntu repeatedly reports `mavros_node` stopped unexpectedly
when the camera/simulator launch starts.

## Confirmed cause

Installed package: MAVROS 2.14.0, Debian package
`ros-humble-mavros 2.14.0-1jammy.20260804.200257`.
The first full launch and the clean restart both aborted with signal 6 and
`std::future_error: Promise already satisfied`. Recovered the actual core
from `/var/crash/_opt_ros_humble_lib_mavros_mavros_node.1000.crash`.
GDB's crashing thread shows `std::__throw_future_error` immediately called
from `mavros::std_plugins::CommandPlugin::handle_command_ack`.

In 2.14.0, the ACK callback calls `promise.set_value` without handling a
duplicate reply. The command-service thread removes the transaction later,
after reacquiring the mutex. A second ACK arriving in between tries to
complete the same promise again and aborts the whole MAVROS process.
Version-request timeouts/replies precede this in the live logs; duplicate
AUTOPILOT_VERSION messages themselves are not the throwing callback.

Primary source comparison:
- https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/plugins/command.cpp
- https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/command.cpp

Late-starting MAVROS after SITL initialization connected successfully and
reported `connected: true`, `armed: false`. That was a diagnostic observation,
not a robust fix for duplicate ACKs. All identified project processes were
subsequently stopped before building. The camera's separate Qt close bug
was already corrected; the restarted viewer closed cleanly after 934 frames.
Some upstream shell-launched Gazebo/MAVProxy descendants required explicit
process-group cleanup after ROS launch exited; that remains a shutdown limit.

## Change

Added `/home/saurabh/ardu_ws/src/mavros` from the official matching 2.14.0
release archive, with its license files. Backported only the duplicate-ACK
exception guard in `src/plugins/command.cpp`. Unlike upstream's current
guard, unexpected future errors are rethrown. No public headers/ABI changed.
System packages under `/opt/ros/humble` were not modified.
Build/provenance details: `../../../mavros/BACKPORT.md`.

Added `tools/check_mavros_duplicate_ack.py`: runs the actual MAVROS binary
against a local fake FCU, on ephemeral loopback UDP ports and isolated ROS
domain 87. Sends bursts of duplicate ACKs for informational capabilities
requests, checks service success, verifies the guard actually executed,
and checks the node stayed alive. It does not connect to SITL or hardware.

## Validation and artifacts

Artifact root:
`/home/saurabh/ardu_ws/eval_results/cheap_review_20260906_0gYo8b/`.
`restart_1/` contains failing `live_launch.log`, healthy late-start
`mavros_late_start.log`, all-thread `mavros_backtrace.txt`, and the decisive
`mavros_crashing_thread.txt`.

The first workspace build was blocked by ccache writing outside the
sandbox; rerun with `CCACHE_DISABLE=1`. The first source-download approval
review hit a usage limit; after the core confirmed the exact fault, the
bounded official-source download was approved on retry.

The matching-version overlay built successfully in 5m58s. Comparing every
original package file against the downloaded archive found only
`src/plugins/command.cpp` changed. Build emitted pre-existing warnings about
potentially uninitialized variables in setpoint_accel/setpoint_trajectory;
these unrelated upstream files were not changed. The upstream broad test
suite was disabled; the targeted executable-level probe below was run.

Focused probe command (after sourcing workspace setup):

```bash
python3 -B src/obst_avoidance/tools/check_mavros_duplicate_ack.py \
  --binary install/mavros/lib/mavros/mavros_node \
  --log eval_results/cheap_review_20260906_0gYo8b/mavros_fix_1/duplicate_ack_drained.log
```

Result: **25 requests completed, 2 duplicate ACKs caught, MAVROS alive**;
fake FCU observed 31 commands including automatic initialization requests.
The final probe drains each burst before reusing a command ID, since
COMMAND_ACK has no per-request identifier. An earlier probe without that
drain also exercised the guard, but its wire-level request count was still
in flight when it exited; use the drained result as final evidence.

Live cold start used `ros2 launch obst_avoidance cheap_viewer.launch.py
use_gui:=false` from `mavros_fix_1/`. Verified the process executable was
`install/mavros/lib/mavros/mavros_node`, and package-prefix/library lookup
selected the workspace overlay. Camera processing and connected, disarmed
MAVROS telemetry were observed. Live log: `mavros_fix_1/live_launch.log`;
initial ROS state: `mavros_fix_1/state_initial.txt`.

Final live check: more than 1,200 camera pairs processed; `/state` capture
advanced from simulation time 2.745s to 24.035s and still reported
`connected: true`, `armed: false`. `/proc/944758/maps` confirmed both
`libmavros.so` and `libmavros_plugins.so` were loaded from `install/mavros`.
No new future-error abort appeared, and the original Ubuntu crash report's
mtime remained 16:43:49 local time. MAVROS did log a time-synchronizer reset;
it continued publishing telemetry. This is separate from the fixed abort.

Demo left running for the user: launch PID/PGID 944722 (tool session 18763),
MAVROS 944758, camera viewer 944865. Recheck these IDs before signaling;
the task board records exclusive use of simulator/ports while the demo runs.
The edit lock is released for peer review, not for starting a second demo.

No autonomous flight has been commanded or validated. The duplicate-ACK
failure is corrected; this is not a claim that all upstream MAVROS issues
or optical-flow limitations are resolved.
