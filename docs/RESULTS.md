# Results

All numbers are produced by scripts in this repo; re-run them to reproduce.

## Milestone B — Perception

`ros2 run palletizer_perception eval_perception --n 60 --seed 7` (sim running with
`box_perception`). 60 seeded random boxes from the spawner distribution (sides 0.2–0.6 m,
height 0.15–0.4 m, yaw ±25°) resting in the pick zone with the leading edge at the pick
photo-eye; depth noise σ = 2 mm; 5 depth frames averaged per detection.
Raw data: `results/perception_eval.csv`.

| Metric | Result | Threshold | |
|---|---|---|---|
| Detections | 60/60 | all | PASS |
| Dimension error, mean (L, W, H pooled) | 0.05 mm | ≤ 5 mm | PASS |
| Dimension error, max | 0.92 mm | ≤ 12 mm | PASS |
| Per dimension mean L / W / H | 0.07 / 0.05 / 0.02 mm | | |
| Grasp-center (top-face center) error, mean / max | 0.05 / 0.58 mm | mean ≤ 5 mm | PASS |
| Yaw error, mean / max | 0.009 / 0.131° | mean ≤ 1.5° | PASS |

Yaw error is computed modulo 180° (a rectangle's axis); for boxes with |L − W| < 10 mm
(2 of 60) modulo 90°, because "length" is then ambiguous.

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
| Placement position error (top-face centre, 3D), mean / max | 0.28 / 4.96 mm | ≤ 15 mm | PASS |
| … horizontal only, mean / max | 0.28 / 4.96 mm | | |
| Yaw error, mean / max | 0.013 / 0.068° | ≤ 3° | PASS |
| Max displacement of an already-placed box | 0.00 mm | ≤ 10 mm | PASS |
| F/T mass error, mean / max | 5 / 10 g | (extra) | |

**How it got there (earlier runs of the same test, kept honest):** the first 20-box run
scored 13/20. Causes, each fixed at the root rather than by relaxing anything:
1. PTP from the lift pose to a low pre-place pose swung the carried box through the
   conveyor/photo-eye → transit via a high hub at the target bearing.
2. Long vertical LIN lift with the arm stretched up violated elbow deceleration limits
   (near-singular) → LIN out of the grasp, then joint-space PTP up.
3. After a failed pick the arm hovered over the pick zone and occluded the next box
   (detected 16 mm short) → every failure path parks the arm; perception refuses to detect
   while a robot link (TF) is over the ROI.
4. MoveIt's wall-clock execution-duration monitor aborted a slow LIN (−6) → disabled; the
   controller result is authoritative.
5. Joint-space PTP down to near-edge targets swept the box into a neighbour (one neighbour
   moved 24.8 mm) → travel at hub height to directly above the target, then vertical LIN.
6. OMPL fallback aborted (−27): the copied UR `ompl_planning.yaml` had no group entry.
7. The test driver lacked the BT's held-box recovery, so one failure cascaded.

The very small errors reflect the simulator (joint position control, exact kinematics, ideal
suction attach); placing on the real robot will be dominated by perception and grasp slip.

## Milestone D — Packing

### Unit tests
`python -m pytest src/palletizer_packing/test` → 19 passed (height map, bounds/height/reach-over,
support ratio, COM rule, mass rule, reachability lookup incl. the real map, candidate
generation, naive scan order, heuristic DBL, packing validity for both planners (no 3D overlaps,
all rules hold), PyBullet checker (stable box accepted, tipping box rejected), heuristic with
physics, Gymnasium `check_env` + masked rollout, heuristic > naive on held-out seeds).

### Why fill % is low, and what the heuristic changes (offline, same rules, physics off)
Box sides are 0.2–0.6 m on a 1.2 × 0.8 m deck, the support rule needs ≥ 75 % of the base within
5 mm of the resting height, and a pallet ends at the **first** box with no feasible placement.
With random heights, two different boxes almost never form a common plane, so a box can stand
only on the deck or on a single larger top. After ~6 boxes the deck is covered and the next
large box ends the pallet — for both planners (they tie on most seeds). Measured over 60 seeds
(100–159, the tuning set):

| Variant | Fill % (mean) | Notes |
|---|---|---|
| Naive (first feasible, far→near, left→right) | 23.4 | already DBL-like |
| Heuristic, initial DBL weights | 24.8 (20 seeds) | rest height weighted high → small boxes use deck area |
| + "future options" count (full feasibility per reference box) | 26.3 (20 seeds) | ~35 s planning / pallet → rejected |
| + deck-area term, rest-height weight 0.3 | 24.9 | paired +1.5, p = 0.13 |
| **+ big-box region reservation (final)** | **25.7** | paired **+2.3**, better 18 / worse 8 / equal 34, Wilcoxon **p = 0.038** |

Held-out check in the unit test (seeds 200–239): naive 27.1 %, heuristic 29.3 %, better on 11,
worse on 7, equal on 22. The absolute level is set by the problem (box sizes vs deck, strict
support rule, stop at the first misfit), not by planner quality; mixed-SKU layer building or a
buffer/lookahead of several boxes would be the next lever.

Planning time (heuristic incl. PyBullet on the top 3 candidates) is reported per box in the
benchmark below.

## Milestone E — Autonomous operation

<!-- AUTO3:BEGIN -->
<!-- AUTO3:END -->

## Milestone E — Benchmark

<!-- BENCHMARK:BEGIN -->
<!-- BENCHMARK:END -->
