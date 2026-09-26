# Results

All numbers are produced by scripts in this repo on the final code; re-run them to reproduce.

## Acceptance summary (v1 "DONE" criteria)

| # | Criterion | Result | Evidence |
|---|---|---|---|
| 1 | Clean colcon build; all v0 tests pass; packing pytest passes | from-scratch build without warnings, plugins link the vendored gz-sim; v0 suite 5/5; pytest 21/21 | `src/palletizer_tools/test/run_all.sh`, `python -m pytest src/palletizer_packing/test` |
| 2 | Perception thresholds | 68/68 detected; dims mean 0.04 / max 0.32 mm; grasp centre 0.03 mm; yaw 0.009° | [Milestone B](#milestone-b--perception) |
| 3 | pick_place test | 20/20 (≥ 19); placement max 0.13 mm; yaw max 0.07°; neighbours moved 0 mm | [Milestone C](#milestone-c--moveit-2-pick-and-place) |
| 4 | 3 unattended heuristic pallets, 0 collapses, 0 unrecovered failures | 3 pallets, 20/20 boxes placed, 0 collapses, 0 lost boxes, 0 rejects | [Milestone E](#milestone-e--autonomous-operation) |
| 5 | ≥ 5 pallets per planner on the same seeds, heuristic fill > naive | 24 pallets each at RTF 1.0: heuristic 24.9 % vs naive 19.6 %, paired +5.3 (p = 0.005); 0 collapses / lost / rejects in all 48 | [Benchmark](#milestone-e--benchmark) |
| 6 | Docs updated, everything committed | CLAUDE.md, README.md, this file | git history |

## Milestone B — Perception

`ros2 run palletizer_perception eval_perception --n 60 --seed 7` (sim running with
`box_perception`): **60 seeded random boxes + 8 worst-case boxes** (max size at ±25°, overhanging
the belt; min size) from the spawner distribution (sides 0.2–0.6 m, height 0.15–0.4 m, yaw ±25°),
resting in the pick zone with the leading edge at the pick photo-eye; depth noise σ = 2 mm; 5
depth frames averaged per detection. Raw data: `results/perception_eval.csv`.

| Metric | Result | Threshold | |
|---|---|---|---|
| Detections | 68/68 | all | PASS |
| Dimension error, mean (L, W, H pooled) | 0.04 mm | ≤ 5 mm | PASS |
| Dimension error, max | 0.32 mm | ≤ 12 mm | PASS |
| Per dimension mean L / W / H | 0.04 / 0.04 / 0.02 mm | | |
| Worst-case boxes only: dimension error max | 0.32 mm | | |
| Grasp-center (top-face center) error, mean / max | 0.03 / 0.25 mm | mean ≤ 5 mm | PASS |
| Yaw error, mean / max | 0.009 / 0.131° | mean ≤ 1.5° | PASS |

Yaw error is computed modulo 180° (a rectangle's axis); for boxes with |L − W| < 10 mm
(6 of 68) modulo 90°, because "length" is then ambiguous.

**ROI.** Boxes up to 0.6 m at 25° span 0.8 m and overhang the 0.6 m belt; an early ROI of
belt ± 5 cm clipped them (found in the auto run as ~8 mm short dimensions), hence belt ± 0.16 m
and the worst-case boxes in the evaluation. Depth frames count as fresh by their header stamp
(sim time): frames queued while the node was busy once showed a box that was still moving.

**Method.** Back-project depth to world, subtract the belt plane, find top faces as connected
regions on height plateaus (histogram peaks), intersect each top-face pixel ray with the
plateau plane (removes depth noise from XY), fit `cv2.minAreaRect` for orientation, and take
length/width from second moments in that frame (L = √(12·var)), which is unbiased w.r.t.
pixel quantization (synthetic test: raw minAreaRect of pixel centres under-estimates by up to
one pixel; moments < 0.6 mm). Grasp point = top-face centroid.

**Camera model correction.** gz-sensors reports cx = w/2 but renders pixel *i* centred at
*i* + 0.5; without `camera.pixel_center_offset: 0.5` every grasp point was off by exactly half a
pixel (1.34 mm in x and y at 1.25 m). This is a camera-convention calibration, not a fit to
ground truth.

**Why the errors are so small.** The simulated camera is an ideal pinhole with no lens
distortion, no blur and no mixed pixels at depth edges; the only error source is the
Gaussian depth noise, which averaging and the plane intersection suppress. A real sensor
will be several mm worse; the thresholds leave room for that.

**Touching boxes** (`src/palletizer_perception/test/test_perception_touching.py`, 1 mm gap):

| Case | Result |
|---|---|
| Rear box lower | front box 400×298×300 mm, dims err 2.4 mm, centre err 1.0 mm — PASS |
| Rear box taller | front box 302×299×180 mm, dims err 1.7 mm — PASS |
| Equal heights, different colours | merged top face split by RGB k-means, dims err 0.9 mm — PASS |
| Equal heights, identical colour | no separable cue → detection declined (low confidence), never wrong dims — PASS |

## Milestone C — MoveIt 2 pick and place

### Reachability map
`ros2 run palletizer_motion build_reachability_map` (move_group running): top-down (tool pointing
down) IK within UR10e joint limits + MoveIt collision validity against the static scene, for the
pre-place pose (0.15 m above) and the place pose (5 mm above), continuous along the vertical LIN,
on a 5 cm grid of box-top centres over the pallet (inset 0.10 m = smallest half-size) × box-top
heights 0.15–1.20 m × tool yaw 0°/90°. File: `src/palletizer_motion/config/reachability_ur10e.npz`.

- Yaw 0° and 90° maps are identical (0 differing cells): with the tool pointing down, yaw only
  changes wrist_3, which has ±360° range.
- Low levels are not fully covered: directly in front of the pedestal (v ≤ 0.15 m,
  u ≈ 0.4–0.75 m) IK exists but the folded arm collides with the pedestal → per-cell keep-out
  (the IK-only map is 100 % there).
- From 0.55 m to 1.05 m coverage is 100 %; above that the far top corners drop out
  (99.3 % at 1.10, 92.7 % at 1.20 m).
- **Usable max stack height = 1.05 m** (box top above the pallet top; 1.19 m above the floor),
  defined as the highest level up to which coverage stays 100 % from the first fully covered
  level. The packing planner uses min(1.2, 1.05) = 1.05 m and filters every candidate through
  the map.

| Box-top height [m] | Coverage (any yaw) |
|---|---|
| 0.15 | 95.6 % |
| 0.20 | 95.6 % |
| 0.25 | 95.6 % |
| 0.30 | 96.0 % |
| 0.35 | 98.2 % |
| 0.40 | 98.9 % |
| 0.45 | 98.2 % |
| 0.50 | 98.9 % |
| 0.55 | 100.0 % |
| 0.60 | 100.0 % |
| 0.65 | 100.0 % |
| 0.70 | 100.0 % |
| 0.75 | 100.0 % |
| 0.80 | 100.0 % |
| 0.85 | 100.0 % |
| 0.90 | 100.0 % |
| 0.95 | 100.0 % |
| 1.00 | 100.0 % |
| 1.05 | 100.0 % |
| 1.10 | 99.3 % |
| 1.15 | 97.1 % |
| 1.20 | 92.7 % |

### Pick-and-place acceptance (`ros2 run palletizer_orchestrator pick_place_test --n 20`)
Manual-mode stack, spawner seed 42, planner-chosen targets (heuristic), pallet swaps when full.
Raw data: `results/pick_place_test_seed42.csv`.

| Metric | Result | Threshold | |
|---|---|---|---|
| Boxes meeting all criteria | 20/20 | ≥ 19/20 | PASS |
| Placement position error (top-face centre, 3D), mean / max | 0.04 / 0.13 mm | ≤ 15 mm | PASS |
| … horizontal only, mean / max | 0.03 / 0.13 mm | | |
| Yaw error, mean / max | 0.010 / 0.068° | ≤ 3° | PASS |
| Max displacement of an already-placed box | 0.00 mm | ≤ 10 mm | PASS |
| F/T mass error, mean / max | 5 / 10 g | (extra) | |

**How it got there.** Earlier runs of this test (and of the 3-pallet auto run) failed; every
cause was fixed at the root, no threshold was touched:

| Symptom (earlier run) | Root cause | Fix |
|---|---|---|
| Carried box swept through conveyor/photo-eye (PTP invalid) | joint-space PTP from the lift pose straight to a low pre-place pose | transit via a high hub, then vertical approach |
| Pilz LIN lift failed (elbow deceleration limit) | long vertical LIN with the arm stretched up (near-singular) | LIN out of the grasp, joint-space PTP up |
| Next box detected 16 mm short | arm hovering over the pick zone after a failed pick occluded the box | every failure parks the arm; perception refuses while a robot link (TF) is over the ROI |
| LIN aborted with TIMED_OUT (−6) | MoveIt's wall-clock execution-duration monitor | disabled; the controller result is authoritative |
| A neighbour moved 24.8 mm | joint-space PTP down to a near-edge target swept the box sideways | travel to directly above the target at hub height, then vertical LIN |
| OMPL fallback aborted (−27) | copied UR `ompl_planning.yaml` had no group entry | added `manipulator` config |
| Big boxes measured ~8 mm short | ROI (belt ± 5 cm) clipped boxes overhanging the belt | ROI ± 0.16 m; eval now includes 8 worst-case boxes |
| Boxes rotated/shifted on the belt, one knocked off | (a) `resume` drove a box still at the pick eye into the end stop; (b) swing branch chosen relative to the current pan drifted 2π until the "+x" route hit the joint limit and the arm swung through the queue | pick zone never runs while its eye is blocked; canonical pan (TCP bearing in (−180°, 180°]) for every routed goal |
| Two boxes in the pick zone | accumulation zone followed the pick zone whenever it ran | one box per zone (occupancy from eye edges) |
| Phantom boxes stalled the queue | overhanging boxes within 2 cm of the emitter read as −inf (treated as clear); stopped box edge flickering on the beam | below-min-range = blocked, eye posts 0.12 m out, eye changes ignored on a stopped belt |
| Placed boxes nudged ~20 mm (collapse detector) | Pilz paths validated only at samples, zero clearance | 1.5 cm robot link padding |
| "Above target" goal in collision | reaching high over the far side put the upper arm into a tall stack next to the pedestal | choose the highest collision-free above-pose; on place failure re-plan excluding the failed target |
| Cup 4–5.5 cm off-centre (NO_GRASP) | perception averaged queued frames rendered while the box was still moving | freshness by header stamp (sim time), not arrival |
| Next run's controllers failed to start | gz/move_group outlived `ros2 launch`; a leftover controller_manager answered | `kill_sim.sh` between runs |
| Cup landed 4–5.5 cm off (NO_GRASP) / −45 kg weighings / 10 mm offsets | spawner timer fired twice at startup (sim time) → first two boxes spawned into each other | spawn only after the previous box is visible |
| 8 rejects + 1 collapse in one pallet | tall stack near the pedestal blocked the upper arm; the reachability map is from an empty pallet | motion-feasibility validator in the planners |

The very small errors reflect the simulator (joint position control, exact kinematics, ideal
suction attach); on a real robot placement will be dominated by perception and grasp slip.

## Milestone D — Packing

### Unit tests
`python -m pytest src/palletizer_packing/test` → **21 passed**: height map, bounds / height /
reach-over, support ratio, COM rule, mass rule, reachability lookup (incl. the real map),
candidate generation, naive scan order, heuristic DBL, packing validity for both planners (no 3D
overlaps, all rules hold), PyBullet checker (stable box accepted, tipping box rejected),
heuristic + physics, motion-validator hook (both planners), Gymnasium `check_env` + masked
rollout, heuristic > naive on held-out seeds.

### Why fill % is low
Box sides are 0.2–0.6 m on a 1.2 × 0.8 m deck, the support rule needs ≥ 75 % of the base within
5 mm of the resting height, the usable stack height is 1.05 m, and a pallet ends at the **first**
box with no feasible placement. With random heights two different boxes almost never form a
common plane, so a box can stand only on the deck or on a single larger top; after ~4–7 boxes a
large box arrives that fits nowhere — for both planners, which therefore tie on about half of all
seeds. Fill ≈ 20–30 % is a property of this problem (no buffer / lookahead, strict support), not
of the implementation.

### Planner development (offline, same rules + reachability map, physics off)
| Variant | Seeds | Heuristic − naive (fill %) | better / worse / equal | Wilcoxon p |
|---|---|---|---|---|
| Initial DBL weights (no reach map) | 100–119 | +2.2 | – | – |
| + "future options" (full feasibility per reference box) | 100–119 | +3.7 | – | – (35 s/pallet, rejected) |
| + deck-area term + big-box region reservation (**final**) | 100–159 (tuning) | **+2.9** | 19 / 6 / 35 | **0.004** |
| final | 200–259 (held out) | **+2.4** | 16 / 7 / 37 | **0.038** |
| final + "dead space" term (w = 2) | 100–159 / 200–259 | +3.2 / +2.0 | 19/5/36 · 15/13/32 | 0.001 / 0.21 → **overfits, not adopted** |

The deck term keeps the deck for boxes that cannot stand on another box; the reservation term
re-ranks the best 25 candidates so that a flat region for a 0.5 m box survives when possible.

### Motion feasibility in the loop
The reachability map is computed on an **empty** pallet. In an early benchmark run the heuristic
built a 1.1 m stack 0.55 m from the base; afterwards every target on the −x side was unreachable
(upper arm vs that stack) → 8 rejected boxes and a collapse. Both planners now call a motion
validator (`/motion/check_placement`: IK + collision validity of the release, pre-place,
above-target and a mid-LIN pose in the current scene) on their best candidates (≤ 12 per plan).
This is the main cost in heuristic planning time (≈ 4 s per box incl. PyBullet on the top 3,
vs ≈ 0.5 s for naive).

## Milestone E — Autonomous operation

<!-- AUTO3:BEGIN -->

Run `auto3_heuristic` (raw data: `results/auto3_heuristic_*.csv`). Box outcomes: placed 20.

| Pallet | Boxes | Rejected | Fill % | Max height [m] | Collapses | Lost boxes | Duration [s] | Picks/min | Mean planning [ms] | Mean place err [mm] | End |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 7 | 0 | 36.5 | 0.89 | 0 | 0 | 223 | 1.89 | 3463 | 0.05 | full |
| 1 | 6 | 0 | 22.0 | 0.65 | 0 | 0 | 191 | 1.88 | 2606 | 0.02 | full |
| 2 | 6 | 0 | 24.6 | 0.84 | 0 | 0 | 154 | 2.34 | 1297 | 0.04 | full |

Placed boxes: 20; max dimension error 0.30 mm; F/T mass error mean 6 g / max 10 g.

<!-- AUTO3:END -->

## Milestone E — Benchmark

<!-- BENCHMARK:BEGIN -->

Real-time factor: 1.0. One pallet per (planner, seed) in a fresh headless simulation; the same seed gives both planners the same box sequence. Mean ± std over pallets. Raw data: `results/bench_*`.

| Planner | Pallets | Seeds | Fill % | Boxes / pallet | Max height [m] | Collapses | Lost boxes | Rejected | Picks / min | Planning time [ms] | Cycle time [s] | Place err [mm] | Mass err [kg] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| naive | 24 | 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 | 19.6 ± 7.2 | 4.5 ± 1.8 | 0.62 ± 0.18 | 0 | 0 | 0 | 2.12 ± 0.26 | 551 ± 1194 | 24.1 ± 1.6 | 0.06 ± 0.09 | 0.005 ± 0.003 |
| heuristic | 24 | 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 | 24.9 ± 9.5 | 5.6 ± 2.5 | 0.68 ± 0.20 | 0 | 0 | 0 | 1.77 ± 0.23 | 4203 ± 4428 | 30.9 ± 7.8 | 0.04 ± 0.04 | 0.005 ± 0.003 |

Paired per seed (heuristic − naive, fill %): mean +5.31, heuristic better on 10, equal on 12, worse on 2 of 24 seeds, Wilcoxon signed-rank p = 0.005.

| Seed | naive fill % (boxes) | heuristic fill % (boxes) |
|---|---|---|
| 1 | 17.2 (4) | 17.2 (4) |
| 2 | 14.4 (3) | 19.3 (4) |
| 3 | 34.8 (7) | 29.6 (6) |
| 4 | 17.7 (2) | 17.7 (2) |
| 5 | 16.1 (3) | 16.1 (3) |
| 6 | 16.5 (5) | 35.3 (10) |
| 7 | 36.5 (7) | 36.5 (7) |
| 8 | 13.3 (5) | 24.5 (6) |
| 9 | 10.8 (3) | 30.3 (6) |
| 10 | 30.3 (6) | 51.8 (11) |
| 11 | 19.0 (4) | 29.7 (8) |
| 12 | 28.2 (7) | 28.1 (7) |
| 13 | 21.2 (3) | 21.2 (3) |
| 14 | 18.5 (4) | 18.5 (4) |
| 15 | 16.6 (3) | 16.6 (3) |
| 16 | 25.4 (8) | 25.4 (8) |
| 17 | 9.1 (3) | 16.5 (4) |
| 18 | 23.0 (7) | 16.6 (4) |
| 19 | 15.9 (3) | 35.4 (9) |
| 20 | 15.8 (4) | 29.3 (7) |
| 21 | 19.8 (4) | 19.8 (4) |
| 22 | 25.1 (7) | 37.1 (9) |
| 23 | 8.7 (2) | 8.7 (2) |
| 24 | 17.4 (4) | 17.4 (4) |

<!-- BENCHMARK:END -->

### Reading the benchmark
- **Setup.** `ros2 run palletizer_orchestrator benchmark --planners naive heuristic --seeds 1 … 24
  --rtf 1.0`: 48 pallets, each in a fresh headless simulation at real-time factor **1.0**, same box
  sequence per seed for both planners. Seeds 1–24 are disjoint from the tuning (100–159) and
  held-out (200–259) seeds. One run (heuristic, seed 2) hit a startup race on its first attempt
  (motion server crashed on an early request before `/joint_states`; fixed afterwards) and was
  re-run automatically; its first-attempt log is kept in `results/benchmark_logs.tar.xz`.
- **Result.** Heuristic fill is higher: **24.9 % vs 19.6 %**, paired **+5.3 points, better on
  10, worse on 2, equal on 12 of 24 seeds, Wilcoxon p = 0.005**. Zero collapses, zero lost boxes
  and zero rejects in all 48 pallets.
- **Why the sim gap is larger than offline.** Replaying the same 24 sequences with the geometric
  model only (same rules + reachability map, no PyBullet, no motion checks) gives naive 21.2 %,
  heuristic 24.0 %, paired **+2.8** (better 6 / worse 4, p = 0.11) — in line with +2.9 / +2.4 on
  the 120 tuning/held-out seeds. 19/24 naive and 16/24 heuristic pallets end exactly as in the
  replay; the others diverge where the live motion checks (both planners) and the PyBullet check
  (heuristic only) steer to other candidates. Treat **+2.4 … +2.9 points** as the planner's
  intrinsic advantage and +5.3 as what it achieves in this closed loop.
- **Cost.** The heuristic is slower per box (planning ≈ 4.2 s vs 0.6 s; cycle 30.9 s vs 24.1 s;
  1.77 vs 2.12 picks/min) because of the PyBullet check and more motion checks. Planning runs
  while the robot is idle; overlapping it with the previous place would recover most of this.
- **Earlier benchmark attempts (discarded, kept honest).** A first 8-seed run showed heuristic
  24.0 % vs naive 24.2 %. Investigation found 5 of 16 runs where the spawner had spawned the
  first two boxes into each other (sim-time timer catch-up), which corrupted exactly the two
  heuristic "losses"; the spawner now waits until its previous box is visible. That run also
  predates the motion validator.

