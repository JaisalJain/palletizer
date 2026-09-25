# Palletizer — project context

ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8) portfolio project: an autonomous palletizing cell.
A UR10e with a suction gripper stands between a conveyor and a EUR pallet. Random boxes
arrive on the conveyor; an overhead RGB-D camera above the pick zone measures each box; the
robot picks at the top-face center and a packing planner places it on the pallet after a
stability check.

## Phases
- **v0 (done)**: workspace, robot description, Gazebo world, runtime suction plugin, box
  spawner, conveyor control, integration tests.
- **v1 (in progress, one combined task)**: A spawner/world updates (done), B perception (done),
  C MoveIt pick & place, D packing, E orchestrator + metrics + benchmark.

## Ground rules (v1)
- Ground truth (`/sim/box_poses`, `/sim/box_specs`) is for evaluation, tests, the collapse
  detector and the simulated upstream process (spawner) ONLY — never an input to
  perception, planning or robot/conveyor control.
- All geometry and tunables live in cell_config.yaml. Acceptance thresholds are never loosened.
- Python deps not in apt (pybullet, gymnasium) live in `.venv` (see requirements.txt); only
  palletizer_packing imports them. Source order: ROS → `.venv/bin/activate` → `install/setup.bash`;
  build with `python -m colcon build`. ament_python packages set
  `[build_scripts] executable = /usr/bin/env python3` in setup.cfg.
- v1: perception (box size / yaw / top-face center from the overhead RGB-D).
- v2: MoveIt 2 pick & place.
- v3: packing planners + stability checker.
- v4: benchmarks. v5: RL planner.

## Packages (`src/`)
| Package | Type | Role |
|---|---|---|
| `palletizer_bringup` | ament_cmake | `config/cell_config.yaml` (**single source of truth** for every dimension/pose), `launch/palletizer.launch.py` (top level) |
| `palletizer_description` | ament_cmake | `urdf/palletizer.urdf.xacro`: world → pedestal → official `ur_robot` macro (ur10e) → `suction_gripper.xacro`; gz_ros2_control + SuctionGripper plugins; `config/controllers.yaml` |
| `palletizer_gazebo` | ament_cmake, C++ | `worlds/palletizing_cell.sdf.xacro`, gz-sim systems `SuctionGripper` and `ModelPosePublisher`, `config/bridge.yaml`, `launch/sim.launch.py`, `rviz/cell.rviz` |
| `palletizer_tools` | ament_python | `box_spawner`, `conveyor_controller`, `cloud_relay`, `boxes.py` (box SDF), `ur_ik.py` (numeric IK for tests), `test/` |

Dependency direction: bringup → gazebo → description, tools. `sim.launch.py` finds
`cell_config.yaml` in palletizer_bringup **at runtime** (no package dependency, which would
create a cycle). Both xacros take `cell_config:=<path>` and read it with `xacro.load_yaml`;
the launch file renders the world to `/tmp/palletizer/palletizing_cell.sdf`.

## Cell layout (world frame = floor under the robot base axis; +x = conveyor travel)
- Pedestal 0.25×0.25×0.70 m; UR10e `base_link` at (0, 0, 0.70).
- Conveyor: belt 3.0×0.6 m, centerline y = 0.80, x ∈ [−2.55, 0.45], top z = 0.80, end stop at x_end.
  Three driven zones (infeed / accum / pick, `conveyor.zones`) with single-ray gpu_lidar
  photo-eyes; zero-pressure accumulation keeps queued boxes >= 0.10 m apart. Boxes stop
  with their leading edge at the pick eye (x = 0.25), so their centers land in [−0.15, 0.15].
- Depth camera has Gaussian noise σ = 2 mm (`camera.depth_noise_stddev`).
- F/T sensor on `suction_cup_joint` → `/suction/wrench` (WrenchStamped, 100 Hz); reads the
  cup's own weight (≈0.98 N) when empty — tare before each grasp.
- Pick zone (0, 0.80, 0.80). Pallet (EUR 1.2×0.8×0.144) centered at (0, −0.60).
- Overhead camera at (0, 0.80, 2.30) looking straight down (1.5 m above the belt), 640×480 @ 15 Hz, hfov 1.2.
- Suction: 0.12 m body + 0.03 m cup; `suction_tcp` = cup face center, +z out of the cup.
- Home joints: [−π/2, −π/2, π/2, −π/2, −π/2, 0] = arm over the pallet side (out of the camera view).
  The same pose with pan = +π/2 is above the pick zone.

## TF frames
`world` (root, in URDF) → `pedestal` → `base_link` → … → `tool0` → `suction_body` → `suction_cup` → `suction_tcp`.
Static TFs from sim.launch.py (from cell_config): `pick_zone`, `pallet_origin` (corner of the
pallet top surface nearest the robot on the −x side, world-aligned), `overhead_camera_link`
(x forward = down), `overhead_camera_optical_frame` (REP-103).
`base` (DH frame, = base_link rotated π about z) is what `ur_ik.py` uses.

## Interfaces
| Name | Type | Notes |
|---|---|---|
| `/overhead_camera/image`, `/depth_image`, `/camera_info` | Image / Image (32FC1, m) / CameraInfo | frame `overhead_camera_optical_frame` |
| `/overhead_camera/points` | PointCloud2 (organized 640×480, xyz+rgb) | frame `overhead_camera_link` (re-stamped by `cloud_relay` from `points_raw`) |
| `/sim/box_poses` | tf2_msgs/TFMessage | ground-truth world poses of all `box_*` models, 30 Hz. **Not** on `/tf` |
| `/suction/cmd` | std_msgs/Bool | true = on (attaches if a box qualifies), false = release |
| `/suction/grasped`, `/suction/attached_model` | Bool / String | 10 Hz + on every change |
| `/conveyor/<zone>/cmd_vel`, `/conveyor/<zone>/eye` | Float64 / LaserScan | per-zone belt speed and photo-eye (zones: infeed, accum, pick) |
| `/conveyor_controller/resume`, `/stop` | std_srvs/Trigger | pick zone holds after a box reaches the pick eye until `resume`; `stop` = all zones |
| `/conveyor_controller/box_ready`, `/running`, `/eyes` | Bool / Bool / String | box_ready = held box settled for `settle_time` |
| `/box_spawner/spawn_box` | std_srvs/Trigger | next box of the seeded sequence (refused until the infeed gap allows) |
| `/sim/box_specs` | palletizer_interfaces/BoxSpec (transient local) | ground-truth dims/mass, EVALUATION ONLY |
| `/suction/wrench` | geometry_msgs/WrenchStamped | F/T on the cup joint |
| `/world/palletizing_cell/create`, `/remove`, `/set_pose` | ros_gz_interfaces Spawn/Delete/SetEntityPose | bridged gz services |
| `/joint_trajectory_controller/follow_joint_trajectory` | FollowJointTrajectory | position-controlled arm |

## Perception (palletizer_perception)
`detector.py` (no ROS): depth → world points (camera pose from config, `pixel_center_offset`
0.5 for gz), belt-plane subtraction in the pick ROI, top faces = connected regions on height
plateaus; per face: ray/plane intersection → minAreaRect (yaw) + second moments (L, W) +
centroid (grasp point). Merged equal-height faces (fill ratio < 0.93 or side > 0.66 m) are
split by RGB k-means; if impossible the face gets confidence 0.2 and is not returned.
The chosen box is the most downstream confident face. `box_perception` averages 5 fresh
depth frames; `/perception/detect_box` (DetectBox) on demand, and it auto-publishes
`/perception/detected_box` on the rising edge of `/conveyor_controller/box_ready`.
Markers on `/perception/markers`. Evaluation: `ros2 run palletizer_perception eval_perception`.

## SuctionGripper (palletizer_gazebo/src/SuctionGripper.cc)
On `/suction/cmd` true it scans models named `box_*`, takes each box collision's face most
anti-parallel to the cup axis, and requires: gap (cup face → face center, along the cup axis)
∈ [−10 mm, max_gap = 8 mm], lateral offset ≤ max_offset = 20 mm, tilt ≤ 0.15 rad. The best
box is attached by creating an entity with `components::DetachableJoint{cup, box, "fixed"}`
(the Physics system builds the joint; removing the entity detaches). Rejections are logged
with the reason (`[SuctionGripper] reject [box_x]: off-center by …`). Tolerances come from
`gripper:` in cell_config.yaml.

## Build / run / test
```bash
cd ~/palletizer_ws && source /opt/ros/jazzy/setup.bash
colcon build && source install/setup.bash
ros2 launch palletizer_bringup palletizer.launch.py              # GUI + RViz, 1 box / 10 s
ros2 launch palletizer_bringup palletizer.launch.py gui:=false spawn_rate:=0.0
src/palletizer_tools/test/run_all.sh                             # all tests (starts its own headless sim)
src/palletizer_tools/test/sim_ctl.sh start|stop                  # headless sim for running single tests
python3 src/palletizer_tools/test/test_suction.py                # single test (sim must be running)
```
Test outputs (rgb.png, depth.png, depth_raw.npy) go to `~/palletizer_ws/test_output/`.
`test/take_screenshot.py` regenerates `docs/world.png`.

## Conventions
- All geometry lives in cell_config.yaml; never hard-code cell dimensions elsewhere.
  Exception: world name `palletizing_cell` is also in bridge.yaml.
- Collision shapes are simple boxes/cylinders; visuals may be fancier.
- Graspable models must be named `box_*` and have one link with a box collision.
- Commit after each working milestone.

## Known issues / gotchas
- **Two gz-sim 8 installs** on this machine: the ROS vendor build (8.11, `/opt/ros/jazzy/opt/gz_sim_vendor`,
  used by ros_gz_sim, gz_ros2_control and `gz` on PATH) and OSRF gz-harmonic 8.15 in `/usr`.
  Plugins must link the vendor build (`find_package(gz_sim_vendor)` then `find_package(gz-sim)`);
  `test_build.sh` verifies with ldd. Don't use the `/usr` `gz.transport13` Python bindings in ROS nodes.
- **rgbd point cloud frame**: gz-sensors publishes the cloud in camera-link axes (x forward)
  but stamps it with `<gz_frame_id>` (optical, needed for the images). Bridged to
  `points_raw`, re-stamped to `overhead_camera_link` by `cloud_relay`.
- **Bridge drops names from SceneBroadcaster pose/info** (Pose_V → TFMessage needs header
  data frame_id/child_frame_id), hence the `ModelPosePublisher` system.
- **Large images/clouds need reliable QoS**: best-effort subscribers drop fragments of
  ~1–10 MB messages (rates fall to ~7 Hz).
- **Reach**: with the 0.7 m pedestal the far top corners of a full 1.2 m stack are at the edge
  of UR10e reach (a 0.5 m pedestal can't reach them at all). Placement approaches from above
  at the top layer may be infeasible at those corners; the packing planner should check IK.
- The belt is an accumulation conveyor: boxes queue behind the one stopped at the pick zone.
  A `resume` while a box is still at the pick zone lets it ride to the end stop.
- The arm is position-controlled via gz_ros2_control; a 10 kg box lifts fine in tests, but
  fast motions with heavy boxes haven't been tuned.
- `--headless-rendering` (EGL) is used when gui:=false; "libEGL warning: failed to create dri2
  screen" in the log is harmless on this hybrid-GPU laptop.
- `gz sdf -k` warns that `gz_frame_id` is not in the SDF spec; gz-sensors reads it anyway.
