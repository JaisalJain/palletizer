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
