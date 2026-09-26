# Palletizer — autonomous palletizing cell (ROS 2 Jazzy, Gazebo Harmonic)

UR10e with a suction gripper, zone conveyor with photo-eyes, overhead RGB-D camera, EUR pallet.
Perception → packing planner (stability + physics check) → MoveIt 2 pick & place, driven by a
BehaviorTree. Results: [docs/RESULTS.md](docs/RESULTS.md).

![Palletizing cell](docs/world.png)

## Setup
```bash
sudo apt install ros-jazzy-ur-description ros-jazzy-ur-simulation-gz ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit python3-scipy
cd ~/palletizer_ws
python3 -m venv --system-site-packages .venv && touch .venv/COLCON_IGNORE
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
pip install -r requirements.txt
python -m colcon build
source install/setup.bash
```

## Every new terminal
```bash
source /opt/ros/jazzy/setup.bash
source ~/palletizer_ws/.venv/bin/activate
source ~/palletizer_ws/install/setup.bash
```

## Run
```bash
# autonomous palletizing demo (Gazebo GUI + RViz)
ros2 launch palletizer_bringup palletizer.launch.py mode:=auto planner:=heuristic

# same, naive planner / fixed box sequence
ros2 launch palletizer_bringup palletizer.launch.py mode:=auto planner:=naive seed:=3

# headless, stop after 3 pallets
ros2 launch palletizer_bringup palletizer.launch.py mode:=auto gui:=false max_pallets:=3

# manual mode (everything except the orchestrator)
ros2 launch palletizer_bringup palletizer.launch.py

# view the behavior tree live: open Groot2 and connect to localhost:1667
```

## Manual commands
```bash
ros2 service call /box_spawner/spawn_box std_srvs/srv/Trigger
ros2 service call /perception/detect_box palletizer_interfaces/srv/DetectBox
ros2 service call /conveyor_controller/resume std_srvs/srv/Trigger
ros2 service call /conveyor_controller/stop std_srvs/srv/Trigger
ros2 service call /motion/park std_srvs/srv/Trigger
ros2 service call /packing/reset std_srvs/srv/Trigger
ros2 topic pub --once /suction/cmd std_msgs/msg/Bool "{data: true}"
ros2 topic echo /orchestrator/cycle_report
```

## Tests
```bash
src/palletizer_tools/test/run_all.sh                            # v0 suite (build + 4 sim tests)
python -m pytest src/palletizer_packing/test                    # packing core unit tests

SIM_ARGS="seed:=42" src/palletizer_tools/test/sim_ctl.sh start  # sim for the tests below
ros2 run palletizer_perception eval_perception --n 60           # perception vs ground truth
python3 src/palletizer_perception/test/test_perception_touching.py
ros2 run palletizer_orchestrator pick_place_test --n 20         # 20-box pick & place acceptance
src/palletizer_tools/test/sim_ctl.sh stop
```

## Benchmark
```bash
ros2 run palletizer_orchestrator benchmark --planners naive heuristic --seeds 1 2 3 4 5 6 7 8
ros2 run palletizer_orchestrator benchmark --summary-only        # rebuild docs/RESULTS.md table
```

## Offline tools
```bash
ros2 run palletizer_motion build_reachability_map                # needs move_group running
```
