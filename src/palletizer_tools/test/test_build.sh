#!/usr/bin/env bash
# Test 1: clean colcon build of the palletizer packages with no compiler/setuptools warnings,
# and the Gazebo plugins link the ROS-vendored gz-sim (not the /usr gz-harmonic install).
set -u
WS=$(cd "$(dirname "$0")/../../.." && pwd)
OUT=$(mktemp -d /tmp/palletizer_build_test.XXXX)
PKGS="palletizer_description palletizer_gazebo palletizer_tools palletizer_bringup"
fail=0
check() { if [[ $1 == 0 ]]; then echo "  [PASS] $2"; else echo "  [FAIL] $2"; fail=$((fail + 1)); fi; }

echo "== test_build (fresh build in $OUT)"
cd "$WS"
colcon --log-base "$OUT/log" build --build-base "$OUT/build" --install-base "$OUT/install" \
  --packages-select $PKGS --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release > "$OUT/build.txt" 2>&1
check $? "colcon build succeeded"

warnings=$(grep -iE "warning" "$OUT/build.txt" | grep -v "^--" || true)
[[ -z $warnings ]]; check $? "no warnings in build output"
[[ -n $warnings ]] && echo "$warnings" | head -20

for lib in libpalletizer_suction_gripper.so libpalletizer_model_pose_publisher.so; do
  path="$OUT/install/palletizer_gazebo/lib/$lib"
  gzsim=$(LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-} ldd "$path" 2>/dev/null | grep "libgz-sim8" | awk '{print $3}')
  [[ $gzsim == /opt/ros/jazzy/opt/gz_sim_vendor/* ]]; check $? "$lib links $gzsim"
done

rm -rf "$OUT"
if [[ $fail == 0 ]]; then echo "test_build: PASS"; else echo "test_build: FAIL ($fail)"; fi
exit $fail
