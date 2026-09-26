# Palletizer — project context

ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8) portfolio project: an autonomous palletizing cell.
A UR10e with a suction gripper stands between a zone conveyor and a EUR pallet. Random boxes
arrive on the conveyor; an overhead RGB-D camera measures each box; a packing planner picks a
stable spot; MoveIt 2 picks at the top-face center and places; a BehaviorTree runs the loop.
Results and acceptance numbers: `docs/RESULTS.md`.

## Phases
- **v0 (done)**: workspace, robot description, world, runtime suction plugin, spawner, conveyor, tests.
- **v1 (done)**: A spawner/world (photo-eye zones, depth noise, F/T), B perception,
  C MoveIt pick & place + reachability map, D packing (core + ROS), E orchestrator + metrics + benchmark.
- Next: RL planner on `palletizer_packing.core.env.PalletizingEnv` (built + tested, not trained).

## Ground rules
- Ground truth (`/sim/box_poses`, `/sim/box_specs`) is ONLY for evaluation/tests, the collapse
  detector (metrics_logger), and simulated plant equipment (box_spawner = upstream process,
  sim_ops = reject pusher / forklift). Never an input to perception, planning or control.
- All geometry and tunables live in `palletizer_bringup/config/cell_config.yaml`. Acceptance
  thresholds (`perception_eval`, `pick_place_test`) are never loosened.
- Python deps not in apt (pybullet, gymnasium) live in `.venv` (requirements.txt, numpy pinned
  to the apt version); only `palletizer_packing` imports them. ament_python packages set
  `[build_scripts] executable = /usr/bin/env python3` so `ros2 run` uses the venv python.
- Source order everywhere: `/opt/ros/jazzy/setup.bash` → `.venv/bin/activate` →
  `install/setup.bash`. Build with `python -m colcon build` (venv active).
- Commit after each working milestone. Keep this file and README.md current.

## Packages (`src/`)
| Package | Type | Role |
|---|---|---|
| `palletizer_bringup` | cmake | `config/cell_config.yaml`, `launch/palletizer.launch.py` (`mode:=manual\|auto`) |
| `palletizer_description` | cmake | UR10e (official macro) on 0.7 m pedestal + suction gripper, F/T sensor on `suction_cup_joint`, gz_ros2_control |
| `palletizer_gazebo` | cmake, C++ | world xacro, `SuctionGripper` + `ModelPosePublisher` gz systems, bridge, `sim.launch.py` (`gui`, `rviz`, `rtf`) |
| `palletizer_interfaces` | cmake | msgs DetectedBox, Placement, BoxSpec, CycleReport, PalletReport; srvs DetectBox, PlanPlacement, CommitPlacement, Weigh; actions PickBox, PlaceBox |
| `palletizer_tools` | python | box_spawner, conveyor_controller (photo-eyes), cloud_relay, sim_ops, `sim_client.py`, `ur_ik.py`, v0 tests |
| `palletizer_perception` | python | `detector.py` (no ROS), `box_perception` node, `eval_perception`, touching test |
| `palletizer_moveit_config` | cmake | SRDF (group `manipulator` base_link→suction_tcp), Pilz PTP/LIN + OMPL, `move_group.launch.py` |
| `palletizer_motion` | python | `motion_server` (PickBox/PlaceBox actions, scene, weigh), `moveit_client.py` (no moveit_py), `reachability.py` + `config/reachability_ur10e.npz` |
| `palletizer_packing` | python | `core/` (NO ROS: pallet, rules, planners, physics [pybullet], env [gymnasium], reach) + `packing_node`; pytest in `test/` |
| `palletizer_orchestrator` | cmake, C++ + python | BT.CPP v4 `orchestrator` + `trees/palletize.xml`, `metrics_logger` (+ collapse detector), `pick_place_test`, `benchmark` |

## Cell layout (world = floor under the robot base axis; +x = conveyor travel)
- UR10e `base_link` at (0, 0, 0.70) on a 0.25×0.25×0.70 m pedestal (0.5 m can't reach the stack corners).
- Conveyor 3.0×0.6 m belt, centerline y = 0.80, top z = 0.80, x ∈ [−2.55, 0.45], three driven zones
  infeed [−2.55, −1.55] / accum [−1.55, −0.65] / pick [−0.65, 0.45] with photo-eyes at x = −1.60 / −0.70 / 0.25.
  Boxes stop with their leading edge at the pick eye → centers in x ∈ [−0.15, 0.15].
  Controller (`conveyor_controller.py`): one box per downstream zone (occupancy counted from
  debounced eye edges; a zone releases only into an EMPTY zone), eye changes ignored while the
  zone's belt is stopped (the pick eye clearing must hold 0.5 s), readings below the lidar's min
  range count as blocked (overhanging boxes), eye posts 0.12 m outside the belt, transit
  watchdog as a safety net. The pick zone never runs while its eye is blocked (`resume` only
  clears the hold).
- Pallet EUR 1.2×0.8×0.144 centered (0, −0.60). Pallet frame for packing: u = x + 0.6,
  v = −0.2 − y (0 at the robot-side edge, 0.8 at the far edge), z above the pallet top.
- Camera (0, 0.80, 2.30) looking down, 640×480 @ 15 Hz, hfov 1.2, depth noise σ = 2 mm.
- Home/park joints [−π/2, −π/2, π/2, −π/2, −π/2, 0]: arm over the pallet side, out of the camera view.

## Data flow (auto mode)
```
conveyor_controller --box_ready--> orchestrator (BT) --DetectBox--> box_perception
                                        |--PlanPlacement(check_only)--> packing_node
                                        |--PickBox--> motion_server --MoveGroup/scene--> move_group --> JTC
                                        |--ResumeConveyor
                                        |--PlanPlacement(mass) / PlaceBox / Weigh(verify)
                                        |--CommitPlacement--> packing_node + motion_server scene
                                        `--CycleReport/PalletReport--> metrics_logger (+ GT) --> results/*.csv
```

## Interfaces
| Name | Type | Notes |
|---|---|---|
| `/perception/detect_box` | DetectBox | averages 5 fresh depth frames; fails if a robot link is over the ROI (TF) |
| `/perception/detected_box`, `/perception/markers` | DetectedBox / MarkerArray | auto-published on the box_ready rising edge |
| `/packing/plan_placement`, `/packing/commit`, `/packing/reset` | PlanPlacement / CommitPlacement / Trigger | model updated from the *commanded* placement |
| `/packing/markers`, `/packing/state` | MarkerArray (height map + ghost) / String JSON | |
| `/motion/pick`, `/motion/place` | PickBox / PlaceBox actions | feedback: phase, progress |
| `/motion/park`, `/motion/drop_held`, `/motion/scene/add_placed`, `/motion/scene/clear_placed` | Trigger / CommitPlacement | |
| `/suction/weigh` | Weigh | tare / mass from `/suction/wrench` |
| `/conveyor_controller/{resume,stop}`, `.../box_ready`, `.../running`, `.../eyes` | Trigger / Bool / String | |
| `/conveyor/<zone>/cmd_vel`, `/conveyor/<zone>/eye` | Float64 / LaserScan | zones infeed, accum, pick |
| `/sim/reject_pick_zone_box`, `/sim/reject_held_box`, `/sim/clear_pallet` | Trigger | simulated equipment (sim_ops) |
| `/orchestrator/cycle_report`, `/orchestrator/pallet_report` | CycleReport / PalletReport | |
| `/sim/box_poses`, `/sim/box_specs` | TFMessage / BoxSpec | GROUND TRUTH, evaluation only |
| `/suction/cmd`, `/suction/grasped`, `/suction/attached_model`, `/suction/wrench` | Bool / Bool / String / WrenchStamped | |
| `/overhead_camera/{image,depth_image,camera_info,points}` | | points in `overhead_camera_link` (re-stamped) |

## Key algorithms
- **Perception** (`detector.py`): world back-projection (`camera.pixel_center_offset` 0.5 for gz),
  belt-plane subtraction in the ROI, top faces = connected regions on height plateaus; per face
  ray/plane intersection → minAreaRect (yaw) + second moments (L, W; unbiased w.r.t. pixels) +
  centroid (grasp point). Merged equal-height faces split by RGB k-means, else declined (conf 0.2).
- **Motion** (`motion_server.py`): pick = PTP hub at the pick bearing → PTP pre-grasp (0.15 m above)
  → LIN down (slow last 3 cm) to a 3 mm gap → suction + confirm → attach collision box (shrunk
  3 mm xy, 1.5 cm trimmed at the bottom) → LIN up → PTP up to lift height → weigh (F/T, tared before
  the grasp) → PTP to the pallet-side hub (the conveyor can resume). Place = PTP to directly above
  the target at hub height → LIN down to pre-place → LIN (slow last 3 cm) to 5 mm above the rest
  surface → release + confirm → detach → LIN retreat. Swings go through the +x (downstream) side
  via the pan 2π branch (`motion.swing_bearing`), so the carried box never passes over queued
  boxes. Hub height clears the tallest committed stack. Every failure path parks the arm out of
  the camera view. Grasp yaw = box yaw + k·π/2 and place yaw + k·π, chosen for minimal wrist
  rotation; the grasp offset measured at pick is compensated at place. IK for joint goals:
  `palletizer_tools/ur_ik.py` (numeric, joint limits); KDL inside Pilz LIN.
- **Reachability map**: top-down IK + MoveIt collision validity for the pre-place and place poses on a
  5 cm grid × box-top heights 0.15–1.20 m × yaw 0/90° (identical: yaw only moves wrist_3).
  Near-field keep-out in front of the pedestal at low heights; **usable max stack height 1.05 m**
  (box top above the pallet top), limited by the far top corners.
- **Packing** (`core/`): 1 cm height map; candidates = 5 cm grid + contact points × {0°, 90°};
  constraints: bounds (no overhang), max height (min(1.2, 1.05 from the map)), reachability map,
  reach-over (stacks between the robot and the target ≤ rest + pre_place − 0.10). Stability: support
  ≥ 0.75 within 5 mm, COM in the support hull, mass ≤ 1.5 × each supporting box. Naive = first
  feasible in a far-to-near, left-to-right scan. Heuristic = DBL score (rest height, max-height
  increase, far rows, left, side contact, roughness) + deck-area term + re-ranking that keeps a flat
  region for a 0.5 m box; the top 3 are verified by PyBullet (1.5 s, ≤ 10 mm / 2°).
  Both planners take an optional motion `validator` (the ROS wrapper plugs in
  `/motion/check_placement`: IK + collision validity of release, pre-place, above-target and a
  mid-LIN sample in the *current* scene), applied in ranked order, ≤ `packing.motion_check_max`
  calls per plan. `PlanPlacement.exclude` skips targets that already failed in motion.
- **Orchestrator** (`trees/palletize.xml`): Park → loop{MorePallets → Cycle}. Cycle: WaitBoxReady →
  DetectBox (retry 3, else Park + reject) → pre-check fit (pallet full → PalletSwap) → PickBox (retry
  once, else Park + reject) → ResumeConveyor → PlanPlacement with the measured mass (full → swap) →
  PlaceBox + VerifyRelease (on failure re-plan excluding that target and place once more, else
  reject the held box) → CommitPlacement → ReportCycle. DetectBox failure → Park → 3 retries
  with 0.5 s delay → reject.
  PalletSwap = FinishPallet (report) → ClearPallet → ResetPackingModel → ClearPlanningScene.
  Groot2 on port 1667.

## Build / run / test
See README.md for the command list. Key entry points:
`ros2 launch palletizer_bringup palletizer.launch.py mode:=auto planner:=heuristic`,
`src/palletizer_tools/test/run_all.sh`, `python -m pytest src/palletizer_packing/test`,
`ros2 run palletizer_perception eval_perception`, `ros2 run palletizer_orchestrator pick_place_test`,
`ros2 run palletizer_orchestrator benchmark`. `sim_ctl.sh start|stop` (env `SIM_ARGS`) runs a
headless manual-mode stack for the scripted tests.

## Known issues / gotchas
- **Two gz-sim 8 installs** (ROS vendor 8.11 used by everything; OSRF 8.15 in /usr). Plugins link
  the vendor build; `test_build.sh` checks with ldd. Don't import the /usr `gz.transport13` bindings.
- The gz rgbd point cloud uses camera-link axes but is stamped with `gz_frame_id` → `cloud_relay`.
- gz principal point: pixel i is centred at i + 0.5 (`camera.pixel_center_offset`).
- ros_gz_bridge drops names from SceneBroadcaster Pose_V → `ModelPosePublisher`.
- Large images/clouds need reliable QoS (best effort loses fragments).
- rclpy: don't call `Executor.shutdown()` while `spin()` runs in another thread (use a spin_once
  loop + join; see `sim_client.py`).
- MoveIt `execution_duration_monitoring` is wall-clock based → disabled (spurious aborts when RTF ≠ 1).
- The copied UR `ompl_planning.yaml` had no group entry → added `manipulator` (else OMPL aborts, −27).
- Never `pkill -f <pattern>` from a shell whose own command line contains the pattern (it kills
  itself); use `pkill -f "[p]attern"` or process-group kills (`sim_ctl.sh`).
- Fill % is low (~25–30 %) by construction: boxes up to 0.6 m on a 1.2×0.8 m deck, a strict support
  rule (75 % within 5 mm → random heights almost never form a common plane), and the pallet ends at
  the first box with no feasible placement. See docs/RESULTS.md for the analysis.
- Simulated sensing and actuation are ideal (pinhole camera, no blur, joint position control) →
  perception and placement errors are far below the thresholds; expect several mm on hardware.
- The offline reachability map is computed on an EMPTY pallet; tall stacks near the pedestal can
  block the upper arm for other targets → the motion validator is what catches that (costs
  ~1–3 s of planning per box).
- box_spawner must not spawn before its previous box appears in `/sim/box_poses` (with sim time
  the timer fires twice back-to-back at startup; two boxes were spawned into each other).
- Startup races: the motion server waits for `/joint_states` before serving (controllers can
  come up after the orchestrator).
- Swing routing uses the canonical pan (TCP bearing in (−180°, 180°]); never pick 2π branches
  relative to the current pan (it drifts until the +x route hits the joint limit and the arm
  swings through the queue — this knocked boxes off the belt).
- Always run `src/palletizer_tools/test/kill_sim.sh` between launches: gz/move_group can outlive
  `ros2 launch`, and a leftover controller_manager breaks the next run's controller spawners
  ("Controller already loaded" / "Failed to configure controller"). `sim_ctl.sh` and the
  benchmark do this automatically.
- The metrics logger keeps a box ledger: `lost_boxes` counts boxes that end below belt level
  outside the pallet (knocked off). Collapses = placed boxes drifting > 20 mm.
