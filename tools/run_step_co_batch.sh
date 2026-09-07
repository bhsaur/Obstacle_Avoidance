#!/bin/bash
# Step CO -- 5 heavy-only + 5 flow-only closed-loop runs against Zone B,
# BN seeds, against the FIXED controller (Step CM: world-bearing yaw
# convergence + hysteresis; Step CN's n_sectors=5-vs-11 fix). Re-run of
# Step CL's batch, which was produced under the now-fixed bugs and is
# superseded (README).
#
# One sim launch per run (spawn_x/spawn_y set per-seed via env_zones.launch.py
# launch arguments, NOT the eval script -- run_{heavy,flow}_only_eval.py take
# no spawn args, they just connect to whatever is already spawned), headless
# (use_gui:=false, ~2x RTF per Step AT), torn down completely between runs so
# no state/DDS connection carries over -- this project's recurring DDS-hang
# failure mode (README) has consistently required a clean process teardown
# between attempts, not a shared long-lived sim across runs.
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source /home/saurabh/ardu_ws/install/setup.bash
set -u

OUT_DIR=/home/saurabh/ardu_ws/eval_results
GOAL_X=65.0
START_X=45.0
ALTITUDE=3.0
MAX_WALL_TIME=300
BOOTSTRAP_S=25

# BN seeds (README Step BN), first 5 of the 10 -- same seeds Step CE/CH/CL used.
SEEDS=(0.1185 -0.4037 -0.1912 -0.2353 0.2010)

teardown() {
    if [ -n "${LAUNCH_PGID:-}" ]; then
        kill -TERM -"$LAUNCH_PGID" 2>/dev/null
    fi
    sleep 3
    # pkill -f alone was observed to silently no-op once in this session --
    # pgrep-then-kill-by-exact-PID is the pattern that reliably worked when
    # verified manually, so do both (pkill first as a best-effort fast path,
    # pgrep+kill as the pattern actually confirmed to work).
    for pat in "env_zones.launch.py" "arducopter" "gz sim" "mavros_node"; do
        pkill -9 -f "$pat" 2>/dev/null
        for pid in $(pgrep -f "$pat" 2>/dev/null); do
            kill -9 "$pid" 2>/dev/null
        done
    done
    sleep 3
    remaining=$(pgrep -f "env_zones.launch.py|arducopter|gz sim|mavros_node" 2>/dev/null)
    if [ -n "$remaining" ]; then
        echo "WARNING: teardown left processes running: $remaining"
    fi
}

run_one() {
    local arm="$1" idx="$2" y="$3"
    local tag="${arm}_co_run${idx}"

    if [ -f "$OUT_DIR/${tag}.json" ]; then
        echo "=== $(date -Iseconds) SKIPPING $tag -- $OUT_DIR/${tag}.json already exists ==="
        return 0
    fi

    echo "=== $(date -Iseconds) starting $tag (spawn_x=$START_X spawn_y=$y) ==="

    teardown
    setsid ros2 launch obst_avoidance env_zones.launch.py \
        spawn_x:="$START_X" spawn_y:="$y" use_gui:=false \
        > "$OUT_DIR/${tag}_launch.log" 2>&1 &
    LAUNCH_PID=$!
    LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')
    echo "launch pid=$LAUNCH_PID pgid=$LAUNCH_PGID, waiting ${BOOTSTRAP_S}s for sim bootstrap"
    sleep "$BOOTSTRAP_S"

    if [ "$arm" = "heavy" ]; then
        script="tools/run_heavy_only_eval.py"
    else
        script="tools/run_flow_only_eval.py"
    fi

    timeout "$((MAX_WALL_TIME + 120))" python3 "$script" \
        --altitude "$ALTITUDE" --max-wall-time "$MAX_WALL_TIME" \
        --goal-x "$GOAL_X" \
        --out "$OUT_DIR/${tag}.json" \
        --log-path "$OUT_DIR/${tag}_frames.jsonl" \
        > "$OUT_DIR/${tag}_eval.log" 2>&1
    status=$?
    echo "=== $(date -Iseconds) $tag eval script exit=$status ==="

    teardown
}

if [ "${1:-}" = "--test-one" ]; then
    # Sanity-check the launch/bootstrap/eval/teardown mechanics on ONE run
    # before committing to the full unattended 10-run batch.
    run_one heavy 0 "${SEEDS[0]}"
    exit $?
fi

for i in "${!SEEDS[@]}"; do
    run_one heavy "$i" "${SEEDS[$i]}"
done

for i in "${!SEEDS[@]}"; do
    run_one flow "$i" "${SEEDS[$i]}"
done

echo "=== $(date -Iseconds) Step CO batch complete ==="
