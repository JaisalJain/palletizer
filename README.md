# Palletizer — autonomous palletizing cell (ROS 2 Jazzy, Gazebo Harmonic)

UR10e with a suction gripper, conveyor with random boxes, overhead RGB-D camera, EUR pallet.

![Palletizing cell](docs/world.png)

## Setup
```bash
sudo apt install ros-jazzy-ur-description ros-jazzy-ur-simulation-gz ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-moveit python3-scipy
cd ~/palletizer_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Run
```bash
# full cell: Gazebo GUI + RViz + box spawner + conveyor controller
ros2 launch palletizer_bringup palletizer.launch.py

# headless, boxes only on demand
ros2 launch palletizer_bringup palletizer.launch.py gui:=false spawn_rate:=0.0

# simulation only (no spawner / conveyor controller)
ros2 launch palletizer_gazebo sim.launch.py gui:=true
```

## Commands
```bash
ros2 service call /box_spawner/spawn_box std_srvs/srv/Trigger          # spawn a box
ros2 service call /conveyor_controller/resume std_srvs/srv/Trigger     # restart belt
ros2 service call /conveyor_controller/stop std_srvs/srv/Trigger       # stop belt
ros2 topic pub --once /suction/cmd std_msgs/msg/Bool "{data: true}"    # suction on
ros2 topic pub --once /suction/cmd std_msgs/msg/Bool "{data: false}"   # suction off
ros2 topic echo /suction/attached_model                                # grasp state
ros2 topic echo /sim/box_poses                                         # ground-truth box poses
```

## Test
```bash
src/palletizer_tools/test/run_all.sh                 # build + all sim tests (headless)

src/palletizer_tools/test/sim_ctl.sh start           # or run tests one by one
python3 src/palletizer_tools/test/test_controllers.py
python3 src/palletizer_tools/test/test_camera.py
python3 src/palletizer_tools/test/test_conveyor.py
python3 src/palletizer_tools/test/test_suction.py
src/palletizer_tools/test/sim_ctl.sh stop
```
