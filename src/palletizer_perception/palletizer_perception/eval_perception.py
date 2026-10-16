"""Perception evaluation against ground truth (needs a running sim + box_perception).

Spawns N seeded random boxes (spawner distribution, yaw +/-25 deg) at rest in the pick zone
with the leading edge at the pick photo-eye, calls /perception/detect_box and compares with
ground truth. Writes results/perception_eval.csv and prints a summary with PASS/FAIL against
the thresholds in cell_config (perception_eval).

Usage: ros2 run palletizer_perception eval_perception [--n 60] [--seed 7] [--out DIR]
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from palletizer_interfaces.srv import DetectBox
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger

from palletizer_tools import boxes, workspace
from palletizer_tools.sim_client import SimClient


def ground_truth(sim, name, size):
    pos, quat = sim.box_poses[name]
    R = Rotation.from_quat(quat)
    top = pos + R.apply([0, 0, size[2] / 2.0])
    yaw = R.as_euler('xyz')[2]
    if size[1] > size[0]:
        yaw += math.pi / 2
    L, W = max(size[0], size[1]), min(size[0], size[1])
    return L, W, size[2], top, yaw


def yaw_error(yaw, yaw_gt, square):
    period = math.pi / 2 if square else math.pi
    e = math.fmod(yaw - yaw_gt, period)
    e = (e + period / 2) % period - period / 2
    return abs(e)


def spawn_at_pick(sim, name, size, mass, yaw, rgba, rng):
    cv = sim.cell['conveyor']
    eye = [z for z in cv['zones'] if z['name'] == 'pick'][0]['eye_x']
    x = eye - boxes.x_extent(size, yaw) / 2.0 - 0.01
    y = cv['y'] + rng.uniform(-0.03, 0.03)
    sim.spawn_box(name, size, mass, (x, y, cv['belt_top_z'] + size[2] / 2 + 0.003), yaw, rgba)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=60)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--out', default=workspace.path('results'))
    ap.add_argument('--no-edge-cases', action='store_true',
                    help='skip the worst-case boxes (max size at +/-25 deg, overhanging the belt)')
    args = ap.parse_args(argv)

    rclpy.init()
    sim = SimClient('eval_perception')
    detect = sim.create_client(DetectBox, '/perception/detect_box')
    sim.wait_ready()
    th = sim.cell['perception_eval']
    rng = np.random.default_rng(args.seed)
    sim.call(sim.stop_cli, Trigger.Request())
    sim.delete_all_boxes()

    sp = sim.cell['spawner']
    hi, lo = sp['mixed']['size_max'], sp['mixed']['size_min']
    yr = sp['yaw_range']
    edge = [] if args.no_edge_cases else [
        ((hi[0], hi[1], hi[2]), yr), ((hi[0], hi[1], lo[2]), -yr),
        ((hi[0], lo[1], hi[2]), yr), ((lo[0], hi[1], hi[2]), -yr),
        ((hi[0], 0.45, 0.3), yr), ((0.45, hi[1], 0.3), -yr),
        ((lo[0], lo[1], lo[2]), yr), ((hi[0], hi[1], hi[2]), 0.0)]
    rows = []
    for i in range(args.n + len(edge)):
        size, mass, yaw, rgba = boxes.random_box(rng, sim.cell['spawner'])
        if i >= args.n:
            size, yaw = edge[i - args.n]
        name = f'box_eval_{i:03d}'
        spawn_at_pick(sim, name, size, mass, yaw, rgba, rng)
        time.sleep(1.0)
        res = sim.call(detect, DetectBox.Request(), timeout=10.0)
        L, W, H, top, yaw_gt = ground_truth(sim, name, size)
        row = {'i': i, 'name': name, 'edge_case': int(i >= args.n), 'L_gt': L, 'W_gt': W, 'H_gt': H,
               'yaw_gt_deg': math.degrees(yaw_gt),
               'success': res.success, 'message': res.message}
        if res.success:
            b = res.box
            c = np.array([b.top_center.position.x, b.top_center.position.y, b.top_center.position.z])
            square = abs(L - W) < 0.01
            row.update({
                'L': b.length, 'W': b.width, 'H': b.height, 'yaw_deg': math.degrees(b.yaw),
                'conf': b.confidence,
                'err_L_mm': abs(b.length - L) * 1000, 'err_W_mm': abs(b.width - W) * 1000,
                'err_H_mm': abs(b.height - H) * 1000,
                'err_center_mm': float(np.linalg.norm(c - top)) * 1000,
                'err_center_xy_mm': float(np.linalg.norm(c[:2] - top[:2])) * 1000,
                'err_yaw_deg': math.degrees(yaw_error(b.yaw, yaw_gt, square)),
                'square': square})
            print(f'{i:3d} {name}: dims err {row["err_L_mm"]:.1f}/{row["err_W_mm"]:.1f}/'
                  f'{row["err_H_mm"]:.1f} mm, center {row["err_center_mm"]:.1f} mm, '
                  f'yaw {row["err_yaw_deg"]:.2f} deg', flush=True)
        else:
            print(f'{i:3d} {name}: DETECTION FAILED ({res.message})', flush=True)
        rows.append(row)
        sim.delete_box(name)

    sim.call(sim.resume_cli, Trigger.Request())
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, 'perception_eval.csv')
    keys = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k)
                  if k in rows[0] else 100 + len(k))
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    ok_rows = [r for r in rows if r['success']]
    if not ok_rows:
        print(f'\nperception eval: 0/{len(rows)} detections, csv {path}\n  [FAIL] detections')
        sim.shutdown()
        rclpy.try_shutdown()
        return 1
    dim_err = np.array([[r['err_L_mm'], r['err_W_mm'], r['err_H_mm']] for r in ok_rows])
    center = np.array([r['err_center_mm'] for r in ok_rows])
    yawe = np.array([r['err_yaw_deg'] for r in ok_rows])
    checks = [
        ('detections', len(ok_rows) == len(rows), f'{len(ok_rows)}/{len(rows)}'),
        ('dim error mean', dim_err.mean() <= th['dim_mean_mm'],
         f'{dim_err.mean():.2f} mm (<= {th["dim_mean_mm"]})'),
        ('dim error max', dim_err.max() <= th['dim_max_mm'],
         f'{dim_err.max():.2f} mm (<= {th["dim_max_mm"]})'),
        ('grasp-center error mean', center.mean() <= th['center_mean_mm'],
         f'{center.mean():.2f} mm (<= {th["center_mean_mm"]}), max {center.max():.2f}'),
        ('yaw error mean', yawe.mean() <= th['yaw_mean_deg'],
         f'{yawe.mean():.3f} deg (<= {th["yaw_mean_deg"]}), max {yawe.max():.3f}'),
    ]
    print(f'\nperception eval: {len(rows)} boxes ({args.n} random + {len(edge)} edge cases), '
          f'seed {args.seed}, csv {path}')
    print(f'  per-dim mean L/W/H: {dim_err.mean(axis=0).round(2)} mm, '
          f'max {dim_err.max(axis=0).round(2)} mm')
    failed = 0
    for label, ok, text in checks:
        print(f'  [{"PASS" if ok else "FAIL"}] {label}: {text}')
        failed += not ok
    sim.shutdown()
    rclpy.try_shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
