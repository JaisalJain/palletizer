# Palletizer — project context

ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8) portfolio project: an autonomous palletizing cell.
A UR10e with a suction gripper stands between a zone conveyor and a pallet station fed by a
pallet conveyor. Boxes arrive on the conveyor; an overhead RGB-D camera measures each box; a
second overhead RGB-D camera measures each delivered pallet, the stack and every placement;
a packing planner (single-SKU layer patterns or v1 mixed planners) picks the target; MoveIt 2
picks at the top-face center and places; a BehaviorTree runs the loop and the pallet-exchange
handshake. Results and acceptance numbers: `docs/RESULTS.md`.

## Phases
- **v0 (done)**: workspace, robot description, world, runtime suction plugin, spawner, conveyor, tests.
- **v1 (done)**: spawner/world, box perception, MoveIt pick & place + reachability map, packing
  (core + ROS), orchestrator + metrics + benchmark (mixed random boxes).
- **v2 (done)**: A SKU catalog + sku/order spawner, B pallet station + pallet perception
  (measured pallet, perceived height map, placement check, stack monitoring), C single-SKU
  pattern planner, D pallet-full decision + PalletExchange handshake + PalletStatus, E RViz
  debug images, F speed (planning in parallel with the pick, ≥ 4 picks/min).
- Next: RL planner on `palletizer_packing.core.env.PalletizingEnv` (built + tested, not trained).

## Ground rules
- **Nothing about boxes or pallets is configured into the robot.** Decision-making (perception,
  packing, motion, orchestrator) loads the config with `cell_config.load(role='robot')`, which
  strips the plant-only sections `spawner` (SKU catalog, mixed distribution) and `exchanger`
  (pallet types). Box dims come from box perception, pallet dims / pose / deck height from
  pallet perception. The config holds only mounting poses, the pallet station AREA, tolerances
  and limits. Checked by `src/palletizer_tools/test/test_no_hardcoding.py`.
- Ground truth (`/sim/box_poses`, `/sim/box_specs`, `/sim/pallet_poses`) is ONLY for evaluation /
  tests (metrics_logger: collapse detector + placement check evaluation, eval scripts) and for
  simulated plant equipment (box_spawner = upstream process, pallet_exchanger = pallet conveyor,
  sim_ops = reject pusher). Never an input to perception, planning or control. Collapse
  detection and placement verification for the robot are done by pallet_perception.
- All geometry and tunables live in `palletizer_bringup/config/cell_config.yaml`. Acceptance
  thresholds (`perception_eval`, `pallet_perception_eval`, `pick_place_test`) are never loosened.
- Gazebo + RViz open by default (`gui:=true`, user decision 2026-10-16: most runs want the GUI).
  Every scripted run (sim_ctl.sh, benchmark, sku_runs, rviz_full.py, tests) passes `gui:=false`
  explicitly (8 GB VRAM laptop; the GUI also lowers the RTF: 0.79 vs 0.86 headless).
- Python deps not in apt (pybullet, gymnasium) live in `.venv` (requirements.txt, numpy pinned
  to the apt version); only `palletizer_packing` imports them. ament_python packages set
  `[build_scripts] executable = /usr/bin/env python3` so `ros2 run` uses the venv python.
- Source order everywhere: `/opt/ros/jazzy/setup.bash` → `.venv/bin/activate` →
  `install/setup.bash`. Build with `python -m colcon build` (venv active).
- Commit after each working milestone (branch `main`; author Jaisal K Jain, no AI co-author
  lines). Keep this file, README.md and docs/RESULTS.md current.
- No machine-specific paths: workspace-relative defaults come from `palletizer_tools.workspace`
  (`$PALLETIZER_WS` or the workspace the overlay was built from); test output → `test_output/`.

## Packages (`src/`)
| Package | Type | Role |
|---|---|---|
| `palletizer_bringup` | cmake | `config/cell_config.yaml`, `launch/palletizer.launch.py` (`mode:=manual\|mixed\|single_sku`, `auto` = mixed; `world:=plain\|warehouse`, default plain) |
| `palletizer_description` | cmake | UR10e (official macro) on 0.7 m pedestal + suction gripper, F/T sensor on `suction_cup_joint`, gz_ros2_control; starts at the park pose |
| `palletizer_gazebo` | cmake, C++ | world xacro (no pallet: station markings, pallet lane, pallet camera; `environment:=plain\|warehouse`), `models/palletizer_warehouse` (Blender-built set dressing, from `tools/warehouse_blender`), gz systems `SuctionGripper`, `ModelPosePublisher` (box_ + pallet_ GT), `PalletMover`; bridge; `sim.launch.py` (`gui`, `rviz`, `rtf`, `world`); `rviz/cell.rviz` |
| `palletizer_interfaces` | cmake | msgs DetectedBox, Placement, BoxSpec, CycleReport, PalletReport, DetectedPallet, HeightMap, PlacementCheck, StackEvent, PackingState, PalletStatus; srvs DetectBox, PlanPlacement, CommitPlacement, Weigh, MeasurePallet, SetPallet, VerifyPlacement; actions PickBox, PlaceBox, PalletExchange |
| `palletizer_tools` | python | box_spawner (sku / order / mixed), pallet_exchanger, conveyor_controller, cloud_relay, sim_ops, `pallet_frame.py`, `sim_client.py`, `ur_ik.py`, `cell_config.py` (role), `workspace.py`, sim tests |
| `palletizer_perception` | python | `detector.py` + `box_perception` (conveyor), `pallet_detector.py` + `pallet_perception` (pallet station), `camera_model.py`, `debug_view.py`; `eval_perception`, `eval_pallet_perception` |
| `palletizer_moveit_config` | cmake | SRDF (group `manipulator` base_link→suction_tcp), Pilz PTP/LIN + OMPL, `move_group.launch.py` |
| `palletizer_motion` | python | `motion_server` (PickBox/PlaceBox, scene incl. measured pallet, weigh, check_placement), `moveit_client.py`, `reachability.py` + `config/reachability_ur10e.npz` (world frame, station area) |
| `palletizer_packing` | python | `core/` (NO ROS: pallet, rules, planners, pattern, perceived, physics [pybullet], env [gymnasium], reach) + `packing_node`, `pattern_report`; pytest in `test/` |
| `palletizer_orchestrator` | cmake, C++ + python | BT.CPP v4 `orchestrator` + `trees/palletize.xml`, `status_overlay`, `metrics_logger` (GT evaluation), `pick_place_test`, `benchmark`, `sku_runs` (`--snapshot`), `sim_snapshot` (media cameras) |

## Cell layout (world = floor under the robot base axis; +x = conveyor travel)
- UR10e `base_link` at (0, 0, 0.70) on a 0.25×0.25×0.70 m pedestal.
- Conveyor 3.0×0.6 m belt, centerline y = 0.80, top z = 0.80, x ∈ [−2.55, 0.45], three driven zones
  infeed [−2.55, −1.55] / accum [−1.55, −0.65] / pick [−0.65, 0.45] with photo-eyes at x = −1.60 / −0.70 / 0.25.
  Boxes stop with their leading edge at the pick eye → centers in x ∈ [−0.15, 0.15].
  Controller (`conveyor_controller.py`): one box per downstream zone (occupancy counted from
  debounced eye edges; a zone releases only into an EMPTY zone), eye changes ignored while the
  zone's belt is stopped (the pick eye clearing must hold 0.5 s), readings below the lidar's min
  range count as blocked (overhanging boxes), eye posts 0.12 m outside the belt, transit
  watchdog as a safety net. The pick zone never runs while its eye is blocked (`resume` only
  clears the hold).
- Pallet station: AREA 1.32×0.92 m centred (0, −0.60) (all the robot knows). Pallet lane along +x at
  y = −0.60: a new pallet enters from x = −2.9, a full one leaves to x = +2.9 (PalletMover,
  0.35 m/s). Delivered pose = station centre ± 2 cm, ± 1.5° (inside the reachability map).
  Pallet types (exchanger only): EUR 1200×800, 1000×800, HALF 800×600, deck 0.144.
- Pallet frame (measured): origin = deck corner nearest the robot on the −u side, v = pallet axis
  pointing away from the robot, u = v rotated +90° (EUR at the station: u ≈ +x, v ≈ −y), z above
  the measured deck. TF `pallet_frame` (x = u, y = −v).
- Conveyor camera (0, 0.80, 2.30) looking down, 640×480 @ 15 Hz, hfov 1.2, depth noise σ = 2 mm.
- Pallet camera (0, −0.60, 2.80) looking down, image x along +x, 800×600 @ 5 Hz, hfov 0.9
  (~3.2 mm/pixel at the deck), σ = 2 mm; mast at (0, −1.55), outside the lane.
- `robot.home` [−π/2, −π/2, π/2, −π/2, −π/2, 0] is only the IK seed (pan rotated to each target);
  `robot.park` (TCP at (0.35, 0.08, 1.65)) is the start / park pose, outside both camera views.
  With a box held (pallet closed after the pick) "park" goes to `robot.hold_xyz` (0.8, 0.3, 1.5)
  instead: at the folded park pose a big box hits the forearm.

## Data flow (mode single_sku / mixed)
```
conveyor_controller --box_ready--> orchestrator (BT) --DetectBox--> box_perception
                                        |--PlanPlacement(check_only)--> packing_node (SKU? slot?)
                                        |--[ PickBox --> motion_server --> move_group --> JTC
                                        |    || VerifyPlacement(prev) --> pallet_perception
                                        |    || PlanPlacement --> packing_node (perceived map) ]
                                        |--ResumeConveyor, PlanPlacement(recheck, mass), PlaceBox, Weigh
                                        |--CommitPlacement--> packing_node + motion scene
                                        |--PalletFull? --> Park, FinishPallet (FULL) --> PalletExchange
                                        |      --> pallet_exchanger (PalletMover) --> MeasurePallet
                                        |      --> SetPallet (packing, motion scene) --> EMPTY
                                        `--CycleReport/PalletReport/PalletStatus--> metrics_logger (+ GT)
pallet_perception: /pallet_perception/height_map --> packing_node; stack_events --> orchestrator
```

## Interfaces
| Name | Type | Notes |
|---|---|---|
| `/perception/detect_box` | DetectBox | averages 5 fresh depth frames; refuses if the tool (TF) is over the ROI or any pixel over the ROI is higher than a box can be |
| `/perception/detected_box`, `/perception/markers`, `/perception/debug_image` | DetectedBox / MarkerArray / Image | debug image rendered only while subscribed |
| `/pallet_perception/measure_pallet` | MeasurePallet | empty pallet → DetectedPallet (size_u, size_v, deck_z, centre, origin, yaw); resets the stack model |
| `/pallet_perception/verify_placement` | VerifyPlacement | placement by change detection → PlacementCheck (+ topic `placement_check`) |
| `/pallet_perception/reset` | Trigger | pallet gone |
| `/pallet_perception/pallet`, `/pallet_perception/height_map` | DetectedPallet / HeightMap | transient local; map rows = v, cols = u, NaN = unobserved |
| `/pallet_perception/stack_events` | StackEvent | SHIFT (> 20 mm) / COLLAPSE, from continuous monitoring |
| `/pallet_perception/debug_image`, `/pallet_perception/markers` | Image / MarkerArray | outline, measured tops, next-target ghost, height-map inset |
| `/packing/plan_placement`, `/packing/commit`, `/packing/reset`, `/packing/set_pallet` | PlanPlacement / CommitPlacement / Trigger / SetPallet | responses carry PackingState; plan failure `reason` |
| `/packing/status`, `/packing/next_target`, `/packing/markers`, `/packing/state` | PackingState / Placement / MarkerArray / String | |
| `/motion/pick`, `/motion/place` | PickBox / PlaceBox actions | feedback: phase, progress |
| `/motion/park`, `/motion/drop_held`, `/motion/check_placement`, `/motion/scene/{add_placed,clear_placed,set_pallet}` | Trigger / CommitPlacement / SetPallet | |
| `/motion/check_placements` | CheckPlacements | a sequence (a whole pattern), each placement checked with the previous ones as temporary scene boxes |
| `/suction/weigh` | Weigh | tare / mass from `/suction/wrench` (averaged by sim-time stamps) |
| `/pallet_exchange` | PalletExchange action | pallet_exchanger; `exchange_mode` auto / operator |
| `/pallet/confirm_exchange` (exchanger), `/pallet/close` (orchestrator) | Trigger | operator |
| `/pallet/status` | PalletStatus | transient local; EMPTY, IN_PROGRESS, FULL, EXCHANGING, READY + reason |
| `/pallet/overlay` | rviz_2d_overlay_msgs/OverlayText | status_overlay |
| `/conveyor_controller/{resume,stop}`, `.../box_ready`, `.../running`, `.../eyes` | Trigger / Bool / String | |
| `/sim/reject_pick_zone_box`, `/sim/reject_held_box` | Trigger | simulated equipment (sim_ops) |
| `/sim/pallet_mover/{cmd,state}` | String | PalletMover gz system (pallet_exchanger only) |
| `/orchestrator/cycle_report`, `/orchestrator/pallet_report` | CycleReport / PalletReport | |
| `/sim/box_poses`, `/sim/box_specs`, `/sim/pallet_poses` | TFMessage / BoxSpec / TFMessage | GROUND TRUTH, evaluation / plant only |

## Key algorithms
- **Box perception** (`detector.py`): world back-projection (`pixel_center_offset` 0.5 for gz),
  belt-plane subtraction in the ROI, top faces = connected regions on height plateaus; per face
  ray/plane intersection → minAreaRect (yaw) + second moments (L, W; unbiased w.r.t. pixels) +
  centroid (grasp point). Merged equal-height faces split by RGB k-means, else declined.
- **Pallet perception** (`pallet_detector.py`): deck = highest large *flat* plateau in the station
  area (flatness test rejects obliquely seen vertical faces, e.g. the pedestal), deck-board gaps
  closed (10 cm), edge ring re-grown, rectangle by second moments → dims, centre, yaw, deck z;
  frame axes assigned by the robot position. Height map: max of observed points per 1 cm cell
  (NaN = occluded). Placement = pixels now at the commanded top height that were clearly lower
  in the reference frame (works when same-height boxes touch), flat top pixels only, rectangle
  fit → error vs commanded. Stack monitoring: per placed box, its visible top pixels (not dropped
  / covered) + pixels that rose to its height next to it → centroid shift; > 20 mm SHIFT, most of
  the top dropped COLLAPSE. Frames only with no robot link over the station (TF) and nothing
  higher than a stack can be; placements announced by the cycle report are excluded until
  verified.
- **Motion** (`motion_server.py`): pick = PTP straight to pre-grasp (0.15 m above; via the hub at
  the pick bearing only if that fails) → LIN down (slow last 3 cm) to a 3 mm gap → suction +
  confirm → attach collision box → LIN up → PTP up to max(lift, hub) height (clears conveyor and
  every stack) → weigh. Place = place poses (IK + collision check) → PTP to directly above the
  target at hub height → one LIN down (slow last 3 cm) to 5 mm above the rest surface → release
  + confirm → detach → LIN retreat. Fast LIN (0.6/0.45) retried at 0.25 when Pilz reports a
  joint-limit violation. Swings via the +x side (canonical pan). Every failure parks the arm.
- **Reachability map**: world-frame grid over the station area (5 cm, nodes symmetric about the
  station centre), box-top z 0.25–1.45 m, top-down IK + MoveIt validity for pre-place and place
  (station floor modelled as a slab up to `max_deck_z`). `PalletReach` intersects it with the
  measured pallet: per-position lookup + usable stack height (EUR at the centre: 1.05 m, as v1).
- **Packing core**: 1 cm height map, a footprint covers a cell if it overlaps it by > 3 mm (a
  5 mm pattern gap never shares a cell) + exact record overlaps in the rest height. Rules as v1
  (bounds, height, reachability, reach-over, support ≥ 0.75 within 5 mm, COM, mass ratio 1.5).
  Planning state = record merged with the perceived map (perception wins on disagreements where
  both are locally flat; logged). Mixed planners as v1; `reserve_footprint` is a planner
  hyperparameter, not a box assumption.
- **Pattern planner** (`core/pattern.py`): inflated boxes (L+gap, W+gap) on the inflated deck;
  guillotine DP over normal sets + first-order non-guillotine (5-block) patterns → all layouts
  with the best count (or best − `interlock_drop`); variants = mirrors × slack (centred / far);
  slots filtered by the reachability map at every layer height; layer pair A/B chosen by
  (count, interlock) or (interlock ≥ min, count, interlock) with `pattern_priority`, fast
  support pre-filter, full static stability check of the whole pallet and PyBullet per layer.
  Order within a layer: far side first. Then the whole pattern is checked in motion against
  its own future stack (`/motion/check_placements`): slots blocked by lower stacks (the upper
  arm's shoulder housing sweeps ~0.8–0.97 m high within ~0.25 m of the base axis: a layer-4
  box next to the pedestal blocks far top-layer targets) are dropped from their layer
  (`restrict()`: plus unsupported boxes above) and the stack re-checked. Runtime: SKU = first box (±10 mm), next slot checked on
  the perceived state + motion validator; reasons PATTERN_COMPLETE / SKU_CHANGE / HEIGHT_LIMIT /
  NO_FEASIBLE_PLACEMENT. Empty pallet + other box → new SKU.
- **Orchestrator** (`trees/palletize.xml`): Park → NewPallet (Park, ResetPalletModels, PalletExchange,
  MeasurePallet → SetPallet) → loop{MorePallets → Cycle}. Cycle: WaitBoxReady → DetectBox (Park +
  3 retries, else reject) → pre-check (fails on a started pallet → ClosePallet(reason) + retry on
  the new pallet; else reject) → Parallel[Pick (retry once / reject) ‖ VerifyPlacement(prev) +
  PlanPlacement] → ResumeConveyor → recheck with measured mass (else ClosePallet while holding,
  arm at the hold pose) →
  PlaceBox + VerifyRelease (re-plan excluding the target; on a started pallet close it and place
  on the next; else reject) → Commit → ReportCycle → PalletFull? (PATTERN_COMPLETE / OPERATOR) →
  ClosePallet. ClosePallet = Park, VerifyPlacement, FinishPallet (FULL + report), NewPallet.
  Each cycle first checks `Holding` (/suction/grasped): a leftover held box is rejected.
  Groot2 on port 1667.

## Build / run / test
See README.md for the command list. Key entry points:
`ros2 launch palletizer_bringup palletizer.launch.py mode:=single_sku sku:=400x300x300`,
`src/palletizer_tools/test/run_all.sh`, `python -m pytest src/palletizer_packing/test`,
`ros2 run palletizer_perception eval_pallet_perception`, `ros2 run palletizer_orchestrator
sku_runs`, `pick_place_test`, `benchmark --prefix v2bench`. `sim_ctl.sh start|stop` (env
`SIM_ARGS`) runs a headless manual-mode stack (a first pallet is delivered) for scripted tests.

CI (`.github/workflows/`, `ros:jazzy` container): `tests` = packing + perception pytest +
no-hardcoding (PYTHONPATH = the package sources, no build); `build` = README apt deps +
`test_build.sh` (fails on any build warning). Validated locally with actionlint and `act`
(act needs node in the job image: `ros:jazzy` + nodejs for local runs only).

Warehouse world (`world:=warehouse`): `tools/warehouse_blender/` builds it in Blender 5 from
`cell_config.yaml` and exports `palletizer_gazebo/models/palletizer_warehouse` (OBJ visuals + 18
static collision boxes; roof/walls cast no shadows; floor visual replaces the plain ground visual,
collision stays the world's ground_plane). Set dressing only: `offline/wh_verify.py` checks that
nothing is inside the robot reach, pallet lane, conveyor, photo-eye beams or camera ROIs (the
conveyor camera's image corners see the north fence base, outside its ROI). `model://` lookup via
the package env hook + `sim.launch.py`. `.blend` files and renders stay out of git.

README media (`docs/media/`, keep < 15 MB): `tools/make_media.sh [--sim] [--rviz] [--gif]`, one
script per image in `tools/media/` (scratch in `tools/media/_work/`, gitignored). `--sim` re-runs
four pallets with `sku_runs --snapshot` (CSVs stay out of `results/`); `sim_snapshot` spawns fixed
cameras (`VIEWS`, `--view` to try poses) and saves the mid-run, FULL and exchange frames.
`--rviz` (`rviz_full.py` -> `rviz_full.png`) needs a display (RViz as an X11 client, captured
with xdotool + xwd). `--gif` (`record_gif.py` -> `warehouse_demo.gif`, the README top): headless
warehouse run, a gz camera sensor for the recording only, frames saved with sim stamps (one cycle
+ the exchange), resampled at 12.5 fps on the sim clock (1x whatever the RTF), cuts only;
`--encode-only` re-encodes the last recording.

## Known issues / gotchas
- **Two gz-sim 8 installs** (ROS vendor 8.11 used by everything; OSRF 8.15 in /usr). Plugins link
  the vendor build; `test_build.sh` checks with ldd. Don't import the /usr `gz.transport13` bindings.
- The gz rgbd point cloud uses camera-link axes but is stamped with `gz_frame_id` → `cloud_relay`.
- gz principal point: pixel i is centred at i + 0.5 (`pixel_center_offset`), both cameras.
- ros_gz_bridge drops names from SceneBroadcaster Pose_V → `ModelPosePublisher` (one instance per
  prefix; world models must not start with `box_` / `pallet_`).
- Large images/clouds need reliable QoS (best effort loses fragments).
- rclpy: don't call `Executor.shutdown()` while `spin()` runs in another thread (use a spin_once
  loop + join; see `sim_client.py`).
- MoveIt `execution_duration_monitoring` is wall-clock based → disabled (spurious aborts when RTF ≠ 1).
- The copied UR `ompl_planning.yaml` had no group entry → added `manipulator` (else OMPL aborts, −27).
- Never `pkill -f <pattern>` from a shell whose own command line contains the pattern (it kills
  itself); use `pkill -f "[p]attern"` or process-group kills (`sim_ctl.sh`).
- Real-time factor falls with the number of stacked boxes (DART solves each contact island as one
  LCP: column stacks ≈ 0.9 empty, 0.5 at 48, 0.25 at 80 boxes; an interlocked stack is ONE island:
  0.1 at 28, 0.04 at 29 bridging boxes). Bullet collision detector, PGS solver and max_contacts
  did not help. Sim-time metrics (picks/min) are unaffected; wall time is.
- Timestamps: F/T weighing averages by message stamps (sim time); arrival times made a window
  empty when the motion server's executor threads were busy.
- Stack monitoring must not see a new box as a neighbour's shift: the orchestrator's cycle report
  marks placements pending until verified; tests that place boxes must publish it too.
- The pedestal keep-out (folded arm vs pedestal at low heights close to the robot) costs a slot for
  small SKUs and depends on the delivered pallet pose (reported per pallet). Near-row stacks
  above ~0.8 m (world) next to the pedestal block the upper arm for some far targets; the
  offline map cannot know (no boxes) → the pattern's motion check drops those top-layer slots.
- Timeouts that wait for sim-time events (fresh frames, suction confirm, settles, F/T windows,
  pallet-camera views, the packing map wait, the BT's service / wait timeouts (`SimTimer` in
  `orchestrator.cpp`), pallet travel in the exchanger, the metrics logger's settle) use node time
  with a wall cap (`palletizer_tools/sim_time.py`): with big interlocked stacks the RTF falls to
  ~0.03–0.1 (a full pallet's 8 s exit then takes minutes of wall time).
- Pilz LIN at 0.6 m/s sometimes exceeds a joint limit near wrist reconfigurations → the same line
  is re-planned at 0.25, then 0.1 (arm nearly stretched at far top corners;
  `motion.lin_fallback_scaling`).
- Simulated sensing and actuation are ideal (pinhole camera, no blur, joint position control) →
  perception and placement errors are far below the thresholds; expect several mm on hardware.
- Mixed-mode fill % is low (~25 %) by construction (random boxes, strict support rule, the pallet
  ends at the first box that fits nowhere); see docs/RESULTS.md.
- The offline reachability map is computed without placed boxes; tall stacks can block the upper
  arm → the motion validator (current scene) catches that.
- box_spawner must not spawn before its previous box appears in `/sim/box_poses`.
- Startup races: the motion server waits for `/joint_states` before serving.
- Swing routing uses the canonical pan (TCP bearing in (−180°, 180°]); never pick 2π branches
  relative to the current pan.
- Always run `src/palletizer_tools/test/kill_sim.sh` between launches (it also kills
  pallet_exchanger / pallet_perception): gz/move_group can outlive `ros2 launch`, and a leftover
  controller_manager breaks the next run's controller spawners. `sim_ctl.sh`, the benchmark and
  sku_runs do this automatically. Don't start a second sim while a batch is running.
- GUI from the VS Code snap terminal: gz sim gui / RViz die with a `/snap/core20 ... libpthread`
  symbol error (the snap exports GTK_PATH, GTK_EXE_PREFIX, LOCPATH, GIO_MODULE_DIR, GDK_PIXBUF_*,
  XDG_DATA_DIRS). Use a normal terminal or `env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH ...`.
  Headless runs are unaffected.
- After a ROS apt upgrade an old `build/` cache can point at a removed library version (e.g.
  `No rule to make target .../libfastcdr.so.2.2.7`): rebuild that package with
  `--cmake-clean-cache`.
- The metrics logger keeps a box ledger: `lost_boxes` = boxes below belt level outside the station
  area and lane (knocked off). GT collapses = placed boxes drifting > 20 mm (evaluation of
  pallet_perception's stack events).
