# Results

All numbers are produced by scripts in this repo on the final code; re-run them to reproduce.
v2 (measured pallets, single-SKU patterns, pallet exchange) comes first; the v1 results
(mixed random boxes) follow unchanged below, with a v2 regression of the mixed mode.

<!-- TOC:BEGIN -->
**Contents**

- [v2 results](#v2-results)
  - [Acceptance summary (v2 "DONE" criteria)](#acceptance-summary-v2-done-criteria)
  - [Milestone B — Pallet perception](#milestone-b--pallet-perception)
  - [Milestone C — Single-SKU pattern planner](#milestone-c--single-sku-pattern-planner)
  - [Single-SKU pallets (acceptance runs)](#single-sku-pallets-acceptance-runs)
    - [SKU change, unknown SKU / pallet, interlock priority](#sku-change-unknown-sku--pallet-interlock-priority)
  - [Milestone D — Pallet-full decision and exchange handshake](#milestone-d--pallet-full-decision-and-exchange-handshake)
  - [Milestone F — Speed](#milestone-f--speed)
  - [Mixed mode (v1 planners) under v2](#mixed-mode-v1-planners-under-v2)
  - [Warehouse world (`world:=warehouse`)](#warehouse-world-worldwarehouse)
  - [v2 test changes](#v2-test-changes)
- [v1 results](#v1-results)
  - [Acceptance summary (v1 "DONE" criteria)](#acceptance-summary-v1-done-criteria)
  - [Milestone B — Perception](#milestone-b--perception)
  - [Milestone C — MoveIt 2 pick and place](#milestone-c--moveit-2-pick-and-place)
    - [Reachability map](#reachability-map)
    - [Pick-and-place acceptance (ros2 run palletizer_orchestrator pick_place_test --n 20)](#pick-and-place-acceptance-ros2-run-palletizer_orchestrator-pick_place_test---n-20)
  - [Milestone D — Packing](#milestone-d--packing)
    - [Unit tests](#unit-tests)
    - [Why fill % is low](#why-fill--is-low)
    - [Planner development (offline, same rules + reachability map, physics off)](#planner-development-offline-same-rules--reachability-map-physics-off)
    - [Motion feasibility in the loop](#motion-feasibility-in-the-loop)
  - [Milestone E — Autonomous operation](#milestone-e--autonomous-operation)
  - [Milestone E — Benchmark](#milestone-e--benchmark)
    - [Reading the benchmark](#reading-the-benchmark)
<!-- TOC:END -->

# v2 results

## Acceptance summary (v2 "DONE" criteria)

<!-- V2SUMMARY:BEGIN -->

| # | Criterion | Result | Evidence |
|---|---|---|---|
| 1 | Clean build; all v0/v1 tests and pytest pass | from-scratch build without warnings (3 gz plugins link the vendored gz-sim); sim suite 7/7 (build, no-hardcoding, controllers, camera, conveyor, suction, pallet mover); pytest packing 54/54, perception unit 9/9; v1 box perception eval 68/68 (dims max 0.31 mm), touching boxes 4/4, pick & place 20/20 (with pallet exchanges) | `run_all.sh`, pytest, [v2 test changes](#v2-test-changes) |
| 2 | No-hardcoding check | code check `test_no_hardcoding.py` (decision code loads the robot view of the config, which has no box / pallet sizes; no plant-only section is read; the C++ orchestrator reads no geometry) 23/23; runtime: a 347×253×219 carton on a 1140×760×150 pallet (neither configured) → measured 346.9×253.0×219.0 and 1137.1×760.2×150.0 mm, 36/36 pattern boxes placed | [unknown run](#sku-change-unknown-sku--pallet-interlock-priority) |
| 3 | Milestone B thresholds | pallet dims ≤ 3.1 mm, pose ≤ 0.5 mm / 0.17°, height map ≤ 4.2 mm on occupied cells, camera vs GT placement ≤ 0.67 mm, shoved box detected (39.9 / 29.9 mm for 40 / 30 mm), none for 10 mm, no false events | [Milestone B](#milestone-b--pallet-perception) |
| 4 | Every catalog SKU: a full pallet unattended | 9/9 SKUs: boxes placed = pattern count, PATTERN_COMPLETE, 0 collapses (GT), 0 stack events, exchange completed and the next pallet's first box placed; SKU change run closed with SKU_CHANGE after 10 boxes and continued on a new pallet | [Single-SKU pallets](#single-sku-pallets-acceptance-runs) |
| 5 | Picks/min ≥ 4 in single_sku mode | 4.26–4.89 per pallet (incl. the first box's arrival), 4.55–5.04 steady; v1 1.77–2.12 | [Milestone F](#milestone-f--speed) |
| 6 | RViz config with both debug image panels + screenshot | `palletizer_gazebo/rviz/cell.rviz` (conveyor and pallet debug images docked next to the 3D view, 2D status overlay, markers); screenshot in the README | README |
| 7 | RESULTS single-SKU table next to the v1 mixed results; docs, commits | this file; v2 mixed-mode regression next to the v1 benchmark; CLAUDE.md, README.md | git history |

The catalog runs were made with the code at commit `0cec7b2` (300×200×200, and 450×350 /
520×320 / 380×280 with the second LIN slow-down level), the SKU-change run with the same code,
the unknown-SKU run with the sim-time waits (`69c8ccc`), the mixed-mode regression and all tests
with the final code. The later commits changed only failure paths (second LIN slow-down level,
sim-time waits, hold pose when a pallet closes with a box on the cup); of these, the completed
catalog runs needed only the LIN slow-down.

<!-- V2SUMMARY:END -->

## Milestone B — Pallet perception

`ros2 run palletizer_perception eval_pallet_perception --reps 3` (manual-mode sim, arm parked;
raw data `results/pallet_perception_eval.json`). Pallet camera 800×600 at 2.8 m, depth noise
σ = 2 mm, 4 frames averaged. Ground truth only scores.

1. **Pallet detection**: 3 exchanges per pallet type (EUR 1200×800, 1000×800, HALF 800×600),
   each delivered at a random pose (±2 cm, ±1.5°).
2. **Placements**: 9 boxes spawned onto an EUR pallet at deliberately perturbed poses around
   nominal "commanded" slots (a layer of six equal-height 394×294×300 boxes with 5 mm gaps + a
   mixed second layer), each measured by `verify_placement` like after a robot place. The
   camera's placement error (measured − commanded) is compared with the true one (actual −
   commanded).
3. **Height map** vs a ground-truth rasterization of the stack (same 1 cm grid).
4. **Stack monitoring**: nothing moves for 2.5 monitoring periods, then three boxes are shoved
   with `set_pose` (10, 40, 30 mm).

| Metric | Result | Threshold | |
|---|---|---|---|
| Pallets measured | 9/9 | all | PASS |
| Pallet dims error, max (u / v) | 3.10 / 0.36 mm | ≤ 10 mm | PASS |
| Pallet position error, max | 0.47 mm | ≤ 10 mm | PASS |
| Pallet yaw error, max | 0.17° | ≤ 1° | PASS |
| Deck height error, max | 0.01 mm | | |
| Placements measured | 9/9 | all | |
| Camera vs ground-truth placement, max / mean (xy) | 0.67 / 0.31 mm | ≤ 5 mm | PASS |
| … z / yaw, max | 0.03 mm / 0.44° | | |
| Height map vs ground truth, occupied interior cells (6303), max / mean | 4.19 / 1.70 mm | ≤ 10 mm | PASS |
| … all occupied cells incl. box-edge cells within 10 mm | 99.2 % | (edge cells straddle two heights) | |
| Occupied cells not observed (occluded) | 8.3 % | (the record fills them) | |
| Stack events while nothing moves | 0 | 0 | PASS |
| Shove 10 mm (below the 20 mm threshold) | no event | no event | PASS |
| Shove 40 mm / 30 mm | SHIFT, estimated 39.9 / 29.9 mm, attributed to the shoved box, none elsewhere | event | PASS |

Per pallet type (dims error u / v, centre, yaw, max of 3): HALF 1.4 / 0.2 mm, 0.1 mm, 0.00°;
1000×800 3.1 / 0.4 mm, 0.4 mm, 0.17°; EUR 3.1 / 0.3 mm, 0.5 mm, 0.10°. The length along u is
measured 1–3 mm short (deck-board end faces seen obliquely are removed by the flatness test);
well inside the threshold. In the robot's own runs, every placement was also checked by the
camera and compared with ground truth by the metrics logger: max 0.08–0.70 mm per pallet
(tables below), and the 20-box pick & place test 18/18 within 0.66 mm.

**How it works** (`pallet_detector.py`, all measured, no pallet in the config):
- *Deck*: the highest large flat height plateau in the station area. A per-pixel flatness
  test rejects vertical faces seen obliquely — the robot pedestal's side face at deck height
  first got merged into the deck (dims +22 mm). The gaps between deck boards are closed, the
  outermost pixel ring re-grown (the flatness test drops it: −6 mm), and the rectangle comes
  from second moments (unbiased w.r.t. pixel quantization). The pallet frame's v axis is the deck
  axis pointing away from the robot.
- *Placement check by change detection*: pixels that now lie at the commanded top height and were
  clearly lower before the place. Adjacent equal-height boxes (5 mm gap) merge in any single
  frame, the change does not. An early version asked for a rise of half the box height and
  missed a 6 mm strip that had seen the neighbour's side wall (dims −6 mm, 3 mm centre error);
  only flat top pixels are fitted (side-wall pixels biased the centre by 3–6 mm).
- *Height map*: max of the observed points per 1 cm cell, NaN where occluded.
- *Stack monitoring*: every placed box is compared with its own state at placement (its top
  pixels then, the heights around it then); pixels of other same-height boxes are excluded, so
  a later neighbour is not a "shift", and slow creep accumulates (unit test: two 12 mm moves with
  a placement in between → SHIFT 24 mm). Frames are used only with no robot link over the
  station (TF) and nothing higher than a stack over it; placements announced by the cycle
  report are excluded until verified. An earlier version compared consecutive frames: it
  reported a pending box as its neighbour's collapse and would not accumulate creep.
- Unit tests on synthetic ray-cast depth images (`src/palletizer_perception/test/
  test_pallet_detector.py`, 9 tests): deck dims/pose, height map, new box next to an
  equal-height box, no events at rest, 30 / 35 mm shifts, 12 mm not reported, creep, collapse.

**Packing on the perceived stack**: before every plan the placement record is merged with the
perceived height map; where both are locally unambiguous (flat 3×3 neighbourhood in the record,
consistent observed neighbours) and differ by > 15 mm, perception wins and the disagreement is
logged (`packing` log, counted in PackingState). The planner waits up to 3 s for a map that
includes the last commit (the verification runs during the pick, so it is normally there).

## Milestone C — Single-SKU pattern planner

`python -m pytest src/palletizer_packing/test` → test_pattern.py (30 tests): known optimal
counts for exact modules without gap (600×400 on EUR = 4, 400×300 = 8, 300×200 = 16,
600×400 on 1200×1000 = 5, 400×300 on 1000×800 = 6) and for the catalog cartons with the 5 mm gap
(same counts; exact modules with a gap do not fit: 2 × 600 + 5 > 1200, hence cartons under
module size), a problem that needs the non-guillotine 5-block structure (8 boxes of 0.3×0.1 on
0.5×0.5, guillotine 7), every catalog SKU: no overlaps (gap kept), no overhang, under the
height limit, every box statically stable in placement order, far side first; interlock for
520×320 (B ≠ A, 100 % bridging, whole pallet stable in PyBullet); column stack for 600×400
(unique optimum) with priority count, and priority interlock trading a box per B layer for
interlocking (400×300×200: 40 → 38 boxes, 50 % bridging); unreachable slots dropped; SKU
recognition (±10 mm same SKU, beyond → SKU_CHANGE, a new pallet re-adopts); the planner refuses
a slot whose supporting layer perception does not see (NO_FEASIBLE_PLACEMENT); motion validator.

Patterns of the catalog on an EUR pallet at the station centre with the real reachability map
(`ros2 run palletizer_packing pattern_report --physics`):

<!-- PATTERNS:BEGIN -->

Pallet EUR 1200 x 800 mm at the station centre, gap 5 mm.

| SKU | Carton [mm] | Optimum / layer | count: A / B x layers = boxes (interlock) | interlock: A / B x layers = boxes (interlock) | Fill % (count) | Note (count) |
|---|---|---|---|---|---|---|
| 600x400x400 | 594 x 394 x 400 | 4 | 4 / 4 x 2 = 8 (0 %) | 4 / 3 x 2 = 7 (100 %) | 67.8 | column stack (no interlocking layer with the same count) |
| 600x400x300 | 594 x 394 x 300 | 4 | 4 / 4 x 3 = 12 (0 %) | 4 / 4 x 3 = 12 (0 %) | 76.3 | column stack (no interlocking layer with the same count) |
| 400x300x300 | 394 x 294 x 300 | 8 | 8 / 8 x 3 = 24 (0 %) | 8 / 7 x 3 = 23 (50 %) | 79.0 | column stack (no interlocking layer with the same count) |
| 400x300x200 | 394 x 294 x 200 | 8 | 8 / 8 x 5 = 40 (0 %) | 8 / 7 x 5 = 38 (50 %) | 87.8 | column stack (no interlocking layer with the same count) |
| 300x200x200 | 294 x 194 x 200 | 16 | 15 / 15 x 5 = 75 (0 %) | 15 / 15 x 5 = 75 (0 %) | 84.9 | column stack (no interlocking layer with the same count); 1/1 slots of the optimum dropped (unreachable) |
| 500x300x250 | 494 x 294 x 250 | 6 | 6 / 6 x 4 = 24 (100 %) | 6 / 6 x 4 = 24 (100 %) | 82.5 | interlocked A/B (100 % of boxes bridge) |
| 450x350x300 | 450 x 350 x 300 | 4 | 4 / 4 x 3 = 12 (50 %) | 4 / 4 x 3 = 12 (50 %) | 51.4 | interlocked A/B (50 % of boxes bridge) |
| 520x320x250 | 520 x 320 x 250 | 4 | 4 / 4 x 4 = 16 (100 %) | 4 / 4 x 4 = 16 (100 %) | 60.3 | interlocked A/B (100 % of boxes bridge) |
| 380x280x220 | 380 x 280 x 220 | 8 | 8 / 8 x 5 = 40 (0 %) | 8 / 7 x 5 = 38 (50 %) | 88.7 | column stack (no interlocking layer with the same count) |

<!-- PATTERNS:END -->

- **Optimum vs used**: 300×200×200 fits 16 per layer on the deck, but one slot of the near
  row lies in the pedestal keep-out (the folded arm hits the pedestal at low heights close to
  the robot) → 15 per layer. The keep-out cost depends on the delivered pallet pose: a pallet
  delivered 18 mm closer to the robot also loses a slot of 400×300 (7 instead of 8 per layer).
  The single-SKU table below reports the count actually used on every pallet.
- **Stack-aware motion check** (not in the offline table above, it needs MoveIt): the first
  SKU batch closed 400×300×200 at 28/35 and 300×200×200 at 60/75 with NO_FEASIBLE_PLACEMENT —
  the first slot of layer 5 (far-left corner) was in collision: MoveIt reported
  `upper_arm_link` vs a layer-4 box next to the pedestal. At layer 4 that box's top (0.94 m)
  is at the height of the upper arm's shoulder housing (≈ 0.80–0.97 m within ≈ 0.25 m of the
  base axis), which sweeps over it at the pan angle for far-left targets. The offline
  reachability map has no boxes. Fix: when a pattern is created, `/motion/check_placements`
  checks every slot with all earlier slots in the planning scene as temporary boxes; infeasible
  slots are dropped from their layer only (and boxes above that lose support), and the stack is
  re-checked. 400×300×200 on that pallet: 3 top-layer slots dropped → 32 boxes, then 32/32
  PATTERN_COMPLETE. (A fixed "no tall boxes near the pedestal" rule would have cost many slots
  that are fine: the collision depends on the pan angle.)
- **Column stack vs interlock**: for the modular sizes on EUR (600×400, 400×300, 380×280) the
  densest layer is unique up to mirroring (a 2×2 or 4×2 grid) → with priority `count` the
  layers are column-stacked (stated in the log / PackingState note). Priority `interlock`
  drops one box per B layer to interlock (−1 box per two layers). 500×300, 450×350 and 520×320
  have a different layer with the same count → interlocked at no cost.
- Pattern search time: 0.1–1.4 s (300×200: 5 s with the PyBullet check of every layer); done
  once per pallet at its first box.

## Single-SKU pallets (acceptance runs)

`ros2 run palletizer_orchestrator sku_runs` (seed 1: every run gets the same first pallet, an
EUR delivered 18 mm closer to the robot than the station centre at −1.06°). "Used / layer" is
layer A / B of the chosen pattern; "Pattern" is the count after the reachability map and the
stack-aware motion check; "Optimum / layer" is the best count on the measured deck alone.
Volume fill % = placed box volume / (deck area × usable height for this pallet and SKU).

<!-- SKU_RUNS:BEGIN -->

One pallet per SKU on EUR (fresh headless simulation each), then the exchange and the first box of the next pallet. Everything in the table is what the robot measured / decided, except the last columns (ground truth, evaluation). Raw data: `results/sku_*`.

| SKU | Carton (measured) [mm] | Pallet (measured) [mm] | Optimum / layer | Used / layer (A / B) | Layers | Pattern | Boxes placed | Volume fill % | Picks / min (pallet · steady) | End | Next pallet | Collapses (GT) | Stack events (detected) | Cam vs GT placement max [mm] | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 600x400x400 | 594 x 394 x 400 (600x400x400) | 1196 x 800 | 4 | 4/4 | 2 | 8 | 8 | 68.0 | 4.26 · 4.55 | PATTERN_COMPLETE | started | 0 | 0 | 0.08 | 0.84 |
| 600x400x300 | 594 x 394 x 300 (600x400x300) | 1196 x 800 | 4 | 4/4 | 3 | 12 | 12 | 76.5 | 4.56 · 4.73 | PATTERN_COMPLETE | started | 0 | 0 | 0.15 | 0.71 |
| 400x300x300 | 394 x 294 x 300 (400x300x300) | 1196 x 800 | 8 | 7/7 | 3 | 21 | 21 | 69.3 | 4.72 · 4.84 | PATTERN_COMPLETE | started | 0 | 0 | 0.58 | 0.59 |
| 400x300x200 | 394 x 294 x 200 (400x300x200) | 1196 x 800 | 8 | 7/7 | 5 | 32 | 32 | 70.4 | 4.77 · 4.98 | PATTERN_COMPLETE | started | 0 | 0 | 0.57 | 0.45 |
| 300x200x200 | 294 x 194 x 200 (300x200x200) | 1196 x 800 | 16 | 15/15 | 5 | 68 | 68 | 77.2 | 4.89 · 5.04 | PATTERN_COMPLETE | started | 0 | 0 | 0.67 | 0.37 |
| 500x300x250 | 494 x 294 x 250 (500x300x250) | 1196 x 800 | 6 | 6/6 | 4 | 21 | 21 | 72.4 | 4.77 · 4.99 | PATTERN_COMPLETE | started | 0 | 0 | 0.56 | 0.35 |
| 450x350x300 | 450 x 350 x 300 (450x350x300) | 1196 x 800 | 4 | 4/4 | 3 | 12 | 12 | 51.5 | 4.57 · 4.74 | PATTERN_COMPLETE | started | 0 | 0 | 0.10 | 0.66 |
| 520x320x250 | 520 x 320 x 250 (520x320x250) | 1196 x 800 | 4 | 4/4 | 4 | 15 | 15 | 56.7 | 4.62 · 4.85 | PATTERN_COMPLETE | started | 0 | 0 | 0.45 | 0.37 |
| 380x280x220 | 380 x 280 x 220 (380x280x220) | 1196 x 800 | 8 | 8/8 | 4 | 31 | 31 | 68.9 | 4.81 · 4.96 | PATTERN_COMPLETE | started | 0 | 0 | 0.50 | 0.51 |
| skuchange | 394 x 294 x 200 | 1196 x 800 | 8 | 7/7 | 5 | 32 | 10 | 22.0 | 4.06 · 4.53 | SKU_CHANGE | started | 0 | 0 | 0.49 | 0.84 |
| unknown | 347 x 253 x 219 | 1137 x 760 | 8 | 8/8 | 5 | 36 | 36 | 69.6 | 4.78 · 4.98 | PATTERN_COMPLETE | started | 0 | 0 | 0.54 | 0.18 |

<!-- SKU_RUNS:END -->

### SKU change, unknown SKU / pallet, interlock priority

<!-- EXTRA_RUNS:BEGIN -->

**SKU change** (`sku_runs --name skuchange --order "400x300x200:10, 600x400x300:4"`; the robot
is not told): 10 cartons of 400×300×200 → pattern 7/7 × 5 = 32 (as in the catalog run); the
11th box measures 594×394×300 → pre-check `SKU_CHANGE` (box 594 x 394 x 300 mm is not SKU
394 x 294 x 200 mm) → **pallet 0 FULL (SKU_CHANGE) with 10/32 boxes** → exchange → new pallet
measured (1197.7 × 800.4 mm) → the same box starts the new pallet with its own pattern (4/4 × 3
= 12) and is placed. 11/11 boxes placed, 0 collapses, 0 stack events, camera vs ground truth
≤ 0.49 mm, 4.06 picks/min (steady 4.53).

**No-hardcoding run** (`sku_runs --name unknown --sku 347x253x219 --pallets 1140x760x150`):
a carton and a pallet type that appear nowhere in the config (not in the SKU catalog, not in
the pallet types; deck 150 mm instead of 144). The robot measured the pallet as
1137.1 × 760.2 mm, deck 150.0 mm, centre (0.0005, −0.5821), yaw −1.06° (true 1140 × 760 × 150,
(0.000, −0.582), −1.06°: 2.9 / 0.2 mm, 0.0 mm, 0.00°) and the carton as 346.9 × 253.0 × 219.0 mm
(true 347 × 253 × 219). Usable height for this pallet and carton 1.15 m → 5 layers; best layer
8 boxes, **interlocked A/B (62 % of the boxes bridge)**; 4 slots dropped by the stack-aware
motion check → pattern 36. **36/36 placed, PATTERN_COMPLETE**, exchange to another 1140×760
pallet, first box placed on it. 0 collapses, 0 stack events, camera vs ground truth ≤ 0.54 mm,
4.78 picks/min (steady 4.98). Interlocked stacks are one contact island for the physics
engine: RTF fell to ~0.1 near the top (0.18 over the pallet).

**Pattern priority `interlock`** (`--sku 400x300x200 --extra pattern_priority:=interlock`):
on the same pallet the planner picks
layers A = 7 (the unreachable slot) and B = 8 interlocking with it (50 % bridging), 3 slots
dropped by the motion check → **34 boxes, 2 more than priority count (7/7 column stack, 32)**:
here the unreachable slot costs only layer A. The run was **stopped at 29/34** (0 collapses,
0 stack events, placement error ≤ 0.13 mm, camera vs ground truth ≤ 0.54 mm): with 29 boxes
in one interlocked contact island the simulation ran at RTF 0.04 (≈ 10 min wall per box).
Raw data in `results/partial/` (log in `results/v2_logs.tar.xz`). The completed interlocked pallets are the unknown-SKU run
above (62 % bridging, 36/36) and the catalog SKUs 500×300, 450×350, 520×320 (interlocked with
priority count, because a same-count interlocking layer exists).

<!-- EXTRA_RUNS:END -->

## Milestone D — Pallet-full decision and exchange handshake

- The orchestrator decides FULL itself. After every commit the BT condition `PalletFull`
  checks for PATTERN_COMPLETE (placed = pattern count) or an operator request (`/pallet/close` →
  OPERATOR). Before a pick, a pre-check that fails on a started pallet closes it with the
  planner's reason: SKU_CHANGE, HEIGHT_LIMIT or NO_FEASIBLE_PLACEMENT (a place that fails twice
  on the same slot also closes it). ClosePallet = Park → verify the last placement →
  `FinishPallet` (PalletReport, `/pallet/status` FULL + reason) → NewPallet.
- NewPallet = Park → forget the old pallet (perception, packing, planning scene) → EXCHANGING →
  `/pallet_exchange` (PalletExchange: feedback phases "full pallet out (n boxes)", "remove",
  "spawn new pallet", "new pallet in") → READY → `measure_pallet` (retry ×3) → `set_pallet` to
  packing and the motion scene → EMPTY. The first box sets IN_PROGRESS.
- `pallet_exchanger` (plant): the full pallet and every box on it slide out along +x on the
  pallet lane (PalletMover gz system: all models teleported rigidly together every physics
  step, trapezoidal profile 0.35 m/s), are removed, a new pallet of the next type in the order
  slides in from −x and stops at the station centre ± 2 cm / ± 1.5°. `exchange_mode:=operator`
  waits for `/pallet/confirm_exchange`.
- `test_pallet_mover` (sim): a dropped box rests on the deck (the pallet's collision moved with
  it); between exchanges the boxes are **free physics bodies** (a box overhanging the edge tips
  over; a shoved box stays shoved); during an exchange a box rides rigidly with its pallet (max
  deviation 0.0 mm, poses compared at equal sim stamps), then both are removed; the new pallet
  carries boxes; operator mode waits for the confirmation, then completes.
- Every SKU run above ends with FULL + reason, the exchange, the new pallet measured, and the
  first box of the next pallet placed ("Next pallet: started").
- RViz: `/pallet/overlay` (rviz_2d_overlay_plugins TextOverlay, `status_overlay` node) shows
  pallet id, state, measured pallet and SKU, boxes placed / pattern count, layer, fill %,
  stack events and reason.

## Milestone F — Speed

Picks per minute in sim time over each pallet (from the pallet's measurement to its last
placement, including the first box's trip along the conveyor), and steady state (first to last
placement): **4.26–4.89 picks/min per pallet, 4.55–5.04 steady**, all single-SKU pallets above
(v1 mixed: 1.77 heuristic / 2.12 naive). Collapses: 0 (ground truth) and 0 stack events
(perception) on every pallet.

| Change | Effect (per cycle, sim time) |
|---|---|
| Plan while the arm picks: BT `Parallel[Pick ‖ VerifyPlacement(previous) → PlanPlacement]`; after the pick only a re-check with the measured mass (µs) | planning (pattern slot + motion validation, 0.1–0.5 s; mixed heuristic 1–4 s) and the pallet-camera verification (~2 s) are off the critical path |
| Pre-check (before the pick) without motion validation in pattern mode | −0.1…0.3 s |
| Straight PTP to the pre-grasp pose (hub only as a fallback); lift straight up to hub height and swing directly to above the target (no pallet-side hub) | −2 PTPs, ≈ −4 s |
| PTP scaling 0.6 / 0.4 → 1.0 / 1.0 (joint accelerations stay within the 3 rad/s² of `joint_limits.yaml`) | swings ≈ −40 % |
| LIN 0.25 → 0.6 m/s (acc 0.45), re-planned at 0.25 and 0.1 when Pilz reports a joint-limit violation | vertical moves ≈ −50 % |
| Place descent: one LIN through pre-place (was LIN to pre-place + LIN) | −1 plan + accel/decel |
| F/T settle + window 0.3 + 0.4 → 0.15 + 0.25 s (weigh by message stamps); release settle 0.4 → 0.15 s | −0.8 s |
| Conveyor detection refuses only if the tool is over the ROI or something higher than a box is in view (was: any arm link origin): no park + re-detect after near-row places | −3 s on those cycles |

Cycle before / after (600×400×400, one cycle, sim time): 27 s → 13 s. The sim's real-time
factor falls as the stack grows (see below); picks/min are in sim time, so they are not
affected, but wall-clock durations are.

**Simulation speed.** RTF with the two cameras and an empty cell ≈ 0.9 (CPU-bound, headless);
with stacked boxes (measured by spawning a 5-layer 300×200 stack): 16 boxes 0.89, 32 0.78, 48
0.51, 64 0.36, 80 0.25. A stack is one DART contact island (one LCP); the bullet collision
detector, the PGS solver and max_contacts 4 gave no gain (PGS was 3× slower), so the runs use the
defaults at a requested RTF of 1.0 (> 1 is not reachable on this machine). The measured RTF per
pallet is in the table.

## Mixed mode (v1 planners) under v2

`ros2 run palletizer_orchestrator benchmark --planners naive heuristic --seeds 1 2 3 4 --prefix
v2bench`: the v1 mode still works on top of everything v2 changed (measured pallet delivered
with pose noise, perceived height map merged into the model, exchange handshake, faster
motion). One pallet per (planner, seed), fresh headless sim.

<!-- V2BENCH:BEGIN -->

Real-time factor: 1.0. One pallet per (planner, seed) in a fresh headless simulation; the same seed gives both planners the same box sequence. Mean ± std over pallets. Raw data: `results/v2bench_*`.

| Planner | Pallets | Seeds | Fill % | Boxes / pallet | Max height [m] | Collapses | Lost boxes | Rejected | Picks / min | Planning time [ms] | Cycle time [s] | Place err [mm] | Mass err [kg] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| naive | 4 | 1 2 3 4 | 21.8 ± 5.5 | 4.0 ± 1.4 | 0.57 ± 0.19 | 0 | 0 | 0 | 3.86 ± 0.17 | 231 ± 133 | 13.1 ± 0.5 | 0.03 ± 0.02 | 0.005 ± 0.002 |
| heuristic | 4 | 1 2 3 4 | 21.8 ± 5.5 | 4.0 ± 1.4 | 0.52 ± 0.17 | 0 | 0 | 0 | 3.23 ± 0.11 | 1594 ± 951 | 15.4 ± 2.2 | 0.04 ± 0.03 | 0.005 ± 0.002 |

Paired per seed (heuristic − naive, fill %): mean -0.00, heuristic better on 0, equal on 4, worse on 0 of 4 seeds.

| Seed | naive fill % (boxes) | heuristic fill % (boxes) |
|---|---|---|
| 1 | 17.3 (4) | 17.3 (4) |
| 2 | 20.4 (4) | 20.4 (4) |
| 3 | 31.1 (6) | 31.1 (6) |
| 4 | 18.6 (2) | 18.6 (2) |

<!-- V2BENCH:END -->

Same seeds in v1 (24-seed benchmark below): naive 17.2 (4), 14.4 (3), 34.8 (7), 17.7 (2);
heuristic 17.2 (4), 19.3 (4), 29.6 (6), 17.7 (2). The pallets now differ slightly because the
pallet is delivered at a random pose (the reachability and the usable height depend on it:
1.00–1.05 m here) and the planners see the perceived stack; on these 4 seeds both planners
tie (they tie on half of all seeds in v1 too). The v1 24-seed comparison remains the planner
evidence. **0 collapses, 0 lost boxes, 0 rejects in all 8 pallets; 3.86 / 3.23 picks/min
(v1: 2.12 / 1.77)**. Heuristic seed 2 exercised the close-while-holding path (the measured mass
broke the mass rule for every position after weighing: the pallet was closed with the box on
the cup, the arm waited at the hold pose during the exchange, and the box went onto the new
pallet).

## Warehouse world (`world:=warehouse`)

The warehouse (built in Blender, `tools/warehouse_blender`) is set dressing around the same cell.
Each evaluation ran in a fresh headless sim per world (`SIM_ARGS="seed:=42 world:=<world>"
sim_ctl.sh start`), with the same commands and thresholds as above. RTF is measured from `/clock`
against wall time over each run (laptop, RTX 5060 8 GB; requested RTF 1.0).

| | `world:=plain` | `world:=warehouse` | Threshold |
|---|---|---|---|
| `eval_perception --n 60 --seed 7` | 68/68, dims mean 0.03 / max 0.32 mm, grasp centre 0.03 mm, yaw 0.009° | identical | 5 / 12 / 5 mm, 1.5° |
| `eval_pallet_perception --reps 3` | 9/9, dims ≤ 3.09 mm, pose ≤ 0.47 mm / 0.169°, placements ≤ 0.67 mm, height map ≤ 4.36 mm, shoves 10→0, 40→1, 30→1 | same; height map ≤ 4.25 mm | 10 mm, 1°, 5 mm, 10 mm |
| `pick_place_test --n 20` | 20/20, place err 0.04 / 0.18 mm, yaw ≤ 0.068°, neighbours 0.00 mm | 20/20, 0.04 / 0.20 mm, yaw ≤ 0.068°, neighbours 0.00 mm | ≥ 19/20, 15 mm, 3°, 10 mm |
| RTF headless, idle / during pick_place | 0.86 / 0.84 | 0.85 / 0.82 | |
| RTF `gui:=true` (Gazebo + RViz), idle | 0.79 | 0.65 | |

Depth-based perception does not see the warehouse: nothing of it is inside either camera's ROI
(`offline/wh_verify.py`: 0 of 8.75 M surface samples inside the robot reach, pallet lane,
conveyor, photo-eye beams or camera ROIs; the conveyor camera's image corners see the base of the
north fence, outside its ROI). The RGB images now show textured concrete; its rendered brightness
is within ~5 % of the plain ground. The extra geometry costs 1–3 % RTF headless and ~18 % with the
Gazebo GUI open.

## v2 test changes

v0 / v1 tests still pass; the ones that relied on the old behaviour were updated, none deleted:
- `test_core.py::test_real_reachability_map`: the map is now world-frame over the station area;
  the test looks it up through `PalletReach` for an EUR pallet at the station and checks the v1
  facts (usable height 1.05 m, far top corners, pedestal keep-out). `PalletSpec.from_config` is
  gone (the pallet is measured): `PalletSpec.measured`.
- `test_suction.py`: `sim.home()` now goes to the park pose (the old home hangs over the pallet
  station, in the pallet camera's view).
- `pick_place_test`: the "pallet swap" is a pallet exchange + measurement (the old
  `/sim/clear_pallet` forklift is gone); placements are also verified by the pallet camera
  during the next pick, like the BT, and the camera error is compared with ground truth.
- `eval_perception`: the mixed-box distribution moved to `spawner.mixed`. Re-run on v2: 68/68,
  dims mean 0.03 / max 0.31 mm, grasp centre 0.03 mm, yaw 0.009° (same as v1); touching boxes
  4/4. `results/perception_eval.csv` and `results/pick_place_test.csv` now hold these v2 re-runs
  (pick & place 20/20, placement max 0.09 mm, yaw max 0.07°, neighbours 0.00 mm, 3 pallet
  exchanges, camera vs ground truth ≤ 0.66 mm).
- `run_all.sh`: + `test_no_hardcoding` (no sim) and `test_pallet_mover`.
- `palletizer_perception/test/conftest.py`: only the sim script is excluded from collection, so
  the new synthetic-depth unit tests run with plain pytest.
- Packing core: a footprint covers a 1 cm cell only if it overlaps it by > 3 mm (so a 5 mm
  pattern gap never shares a cell), and the rest height also checks exact record overlaps (the
  first version with only the tolerance let a 4 mm overlap through in the mixed-packing test).


# v1 results

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

