#!/bin/bash
# Each autonomous_demo owns and cleans its simulator process group.
# Run only while holding the project edit lock and simulator resources.
set -e
cd /home/saurabh/ardu_ws
source /opt/ros/humble/setup.bash
source /home/saurabh/ardu_ws/install/setup.bash
set -u
cd /home/saurabh/ardu_ws/src/obst_avoidance
ZONE="${1:-zone_A}"
SPAWN_Y="${2:--3.8}"
case "$ZONE" in zone_A|zone_B|zone_C|zone_D) ;; *) echo 'Invalid zone' >&2; exit 2 ;; esac
if pgrep -x 'arducopter|mavros_node' >/dev/null; then
  echo 'Simulator resources occupied; stop the owning session first.' >&2
  exit 1
fi
OUT=$(mktemp -d "/home/saurabh/ardu_ws/eval_results/ctrl_compare_${ZONE}_XXXXXX")
echo "Comparison outputs: $OUT"
# Override to compare pathtrack against pathtrack_v2 without changing its goal.
read -r -a ARMS <<< "${CTRL_COMPARE_ARMS:-baseline pathtrack}"
for arm in "${ARMS[@]}"; do
  case "$arm" in baseline|pathtrack|pathtrack_v2|pathtrack_v3|pathtrack_v4|pathtrack_v5) ;; *) echo 'Invalid arm' >&2; exit 2 ;; esac
  python3 -m obst_avoidance.autonomous_demo --output-dir "$OUT/$arm" \
    --zone "$ZONE" --spawn-y "$SPAWN_Y" --headless --controller "$arm" \
    --endpoint-goal --max-wall-time "${CTRL_COMPARE_WALL_S:-240}" > "$OUT/$arm.log" 2>&1 || {
      echo "Arm $arm failed; preserving output and stopping comparison." >&2
      exit 1
    }
  if pgrep -x 'arducopter|mavros_node' >/dev/null; then
    echo 'Residual simulator process detected; stopping comparison.' >&2
    exit 1
  fi
done
echo "Comparison finished: $OUT"
