#!/usr/bin/env bash
# Start/stop a headless palletizer sim for the tests.
#   sim_ctl.sh start [log]   -> launches palletizer_bringup gui:=false spawn_rate:=0, waits until ready
#   sim_ctl.sh stop
# Environment: ROS, then the project venv (pybullet/gymnasium), then the workspace overlay.
WS_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/.venv/bin/activate"
[[ -f $WS_ROOT/install/setup.bash ]] && source "$WS_ROOT/install/setup.bash"
set -u
PIDFILE=/tmp/palletizer_sim.pid

stop() {
  if [[ -f $PIDFILE ]]; then
    local pgid
    pgid=$(cat "$PIDFILE")
    kill -INT -- "-$pgid" 2>/dev/null
    for _ in $(seq 20); do kill -0 -- "-$pgid" 2>/dev/null || break; sleep 1; done
    kill -KILL -- "-$pgid" 2>/dev/null
    rm -f "$PIDFILE"
  fi
  "$(dirname "${BASH_SOURCE[0]}")/kill_sim.sh"   # stragglers (gz, move_group) outlive the group
}

start() {
  local log=${1:-/tmp/palletizer_sim.log}
  stop
  setsid ros2 launch palletizer_bringup palletizer.launch.py gui:=false rviz:=false \
    spawn_rate:=0.0 ${SIM_ARGS:-} > "$log" 2>&1 &
  echo $! > "$PIDFILE"
  for _ in $(seq 180); do
    if grep -q "box_spawner ready" "$log" && \
       grep -q "activated joint_trajectory_controller" "$log" && \
       grep -q "motion_server ready" "$log"; then
      sleep 2
      echo "sim ready (log: $log)"
      return 0
    fi
    sleep 1
  done
  echo "sim failed to start, see $log" >&2
  stop
  return 1
}

case "${1:-}" in
  start) start "${2:-}" ;;
  stop) stop ;;
  *) echo "usage: $0 start [log] | stop" >&2; exit 2 ;;
esac
