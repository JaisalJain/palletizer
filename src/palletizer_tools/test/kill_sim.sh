#!/usr/bin/env bash
# Kill every process of a palletizer simulation stack and wait until they are gone.
# Patterns use the [x] trick so this script never matches its own command line.
PATTERNS=("[g]z sim" "[g]z_tools_vendor" "[m]ove_group" "[p]arameter_bridge"
          "[r]obot_state_publisher" "[s]tatic_transform_publisher" "[r]viz2"
          "[c]ontroller_manager/spawner" "[i]nstall/palletizer_" "[r]os2 launch palletizer")
alive() { for p in "${PATTERNS[@]}"; do pgrep -f "$p" >/dev/null && return 0; done; return 1; }
for sig in INT TERM KILL; do
  alive || exit 0
  for p in "${PATTERNS[@]}"; do pkill -"$sig" -f "$p" 2>/dev/null; done
  for _ in $(seq 20); do alive || exit 0; sleep 0.5; done
done
alive && { echo "kill_sim: processes still alive" >&2; exit 1; }
exit 0
