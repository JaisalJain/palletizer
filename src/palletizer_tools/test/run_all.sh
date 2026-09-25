#!/usr/bin/env bash
# Runs all v0 tests: build test, then starts a headless sim and runs tests 2-5.
# Usage: source install/setup.bash && src/palletizer_tools/test/run_all.sh
# Environment: ROS, then the project venv (pybullet/gymnasium), then the workspace overlay.
WS_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/.venv/bin/activate"
[[ -f $WS_ROOT/install/setup.bash ]] && source "$WS_ROOT/install/setup.bash"
set -u
DIR=$(cd "$(dirname "$0")" && pwd)
LOG=${PALLETIZER_SIM_LOG:-/tmp/palletizer_sim.log}
declare -A result
order=(test_build test_controllers test_camera test_conveyor test_suction)

"$DIR/test_build.sh"; result[test_build]=$?

"$DIR/sim_ctl.sh" start "$LOG" || { echo "sim did not start"; exit 1; }
for t in test_controllers test_camera test_conveyor test_suction; do
  python3 "$DIR/$t.py"; result[$t]=$?
done
"$DIR/sim_ctl.sh" stop

echo
echo "===== summary ====="
fail=0
for t in "${order[@]}"; do
  if [[ ${result[$t]} == 0 ]]; then echo "PASS  $t"; else echo "FAIL  $t"; fail=1; fi
done
exit $fail
