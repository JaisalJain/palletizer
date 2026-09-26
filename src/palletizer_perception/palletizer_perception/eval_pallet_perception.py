"""Pallet-perception evaluation against ground truth (Milestone B acceptance).

Needs a manual-mode sim (pallet_exchanger, pallet_perception running; the arm parked):
  1. pallet detection: `--reps` exchanges per pallet type (random delivered pose), measured
     deck dims / centre / yaw / height vs ground truth
  2. placements: boxes are spawned onto an EUR pallet at deliberately perturbed poses around
     nominal "commanded" slots (a 5 mm-gap layer of equal-height boxes + a mixed second
     layer); after each one /pallet_perception/verify_placement measures it. The camera's
     placement error (measured - commanded) must agree with the ground-truth error
     (actual - commanded), i.e. |measured - actual| <= placement_agree_mm
  3. height map vs ground truth on occupied cells (cells fully inside a top face; cells
     straddling a box edge are reported separately)
  4. stack monitoring: no events while nothing moves; a box shoved (set_pose) by less than
     the shift threshold -> no event; shoved by more -> SHIFT event for that box
Writes results/pallet_perception_eval.json and prints PASS/FAIL per threshold.

Usage: ros2 run palletizer_perception eval_pallet_perception [--reps 3] [--seed 3]
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from palletizer_interfaces.msg import CycleReport, DetectedBox, HeightMap, Placement, StackEvent
from palletizer_interfaces.srv import MeasurePallet, VerifyPlacement
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation

from palletizer_perception.pallet_detector import normalize_axis
from palletizer_tools.pallet_frame import PalletFrame
from palletizer_tools.sim_client import SimClient


def gt_frame(sim, key, robot_xy):
    name, xyz, yaw = sim.station_pallet()
    L, W, H = sim.cell['exchanger']['pallet_types'][key]['size']
    axis = np.array([math.cos(yaw), math.sin(yaw)])
    return PalletFrame.from_axes(xyz[:2], axis, L, W, H, robot_xy), yaw, (L, W, H)


def box_top(sim, name, size):
    pos, quat = sim.box_poses[name]
    R = Rotation.from_quat(quat)
    top = pos + R.apply([0, 0, size[2] / 2])
    yaw = R.as_euler('xyz')[2]
    return top, yaw


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--seed', type=int, default=3)
    ap.add_argument('--types', nargs='+', default=['HALF', '1000x800', 'EUR'])
    ap.add_argument('--out', default=os.path.expanduser('~/palletizer_ws/results'))
    ap.add_argument('--debug', default='', help='save height maps (npz) here')
    args = ap.parse_args(argv)
    rclpy.init()
    sim = SimClient('eval_pallet_perception')
    th = sim.cell['pallet_perception_eval']
    robot_xy = sim.cell['robot']['pedestal']['xy']
    rng = np.random.default_rng(args.seed)
    measure = sim.create_client(MeasurePallet, '/pallet_perception/measure_pallet')
    verify = sim.create_client(VerifyPlacement, '/pallet_perception/verify_placement')
    maps, events = [], []
    sim.create_subscription(HeightMap, '/pallet_perception/height_map', maps.append,
                            QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                       reliability=ReliabilityPolicy.RELIABLE))
    sim.create_subscription(StackEvent, '/pallet_perception/stack_events', events.append, 50)
    report_pub = sim.create_publisher(CycleReport, '/orchestrator/cycle_report', 50)
    sim.wait_ready()
    sim.home()
    sim.delete_all_boxes()
    out = {'pallets': [], 'placements': [], 'height_map': {}, 'shove': []}

    # ---------------------------------------------------------------- 1. pallet detection
    for key in args.types:
        for rep in range(args.reps):
            res = sim.exchange_pallet(pallet_type=key)
            if not res.success:
                print(f'exchange failed: {res.message}')
                continue
            time.sleep(1.0)
            m = sim.call(measure, MeasurePallet.Request(), timeout=60.0)
            gt, gyaw, (L, W, H) = gt_frame(sim, key, robot_xy)
            row = {'type': key, 'rep': rep, 'success': m.success, 'message': m.message}
            if m.success:
                f = PalletFrame.from_msg(m.pallet)
                row.update({
                    'size_u': f.size_u, 'size_v': f.size_v, 'gt_u': gt.size_u, 'gt_v': gt.size_v,
                    'err_u_mm': abs(f.size_u - gt.size_u) * 1000,
                    'err_v_mm': abs(f.size_v - gt.size_v) * 1000,
                    'err_center_mm': float(np.linalg.norm(f.center - gt.center)) * 1000,
                    'err_yaw_deg': math.degrees(abs(normalize_axis(f.yaw - gt.yaw))),
                    'err_deck_mm': abs(f.deck_z - H) * 1000,
                    'gt_yaw_deg': math.degrees(gyaw)})
                print(f'{key} #{rep}: dims err {row["err_u_mm"]:.1f} / {row["err_v_mm"]:.1f} mm, '
                      f'centre {row["err_center_mm"]:.1f} mm, yaw {row["err_yaw_deg"]:.3f} deg '
                      f'(true yaw {row["gt_yaw_deg"]:+.2f}), deck {row["err_deck_mm"]:.1f} mm',
                      flush=True)
            else:
                print(f'{key} #{rep}: MEASUREMENT FAILED ({m.message})', flush=True)
            out['pallets'].append(row)

    # ---------------------------------------------------------------- 2. placements
    if args.types[-1] != 'EUR':
        sim.exchange_pallet(pallet_type='EUR')
        sim.call(measure, MeasurePallet.Request(), timeout=60.0)
    gt, gyaw, _ = gt_frame(sim, 'EUR', robot_xy)
    sim.call(measure, MeasurePallet.Request(), timeout=60.0)       # fresh model for this stack
    n_events0 = len(events)
    a, b, h1 = 0.394, 0.294, 0.300
    slots = []            # (u, v, du, dv, rest, height, perturb(du, dv, dyaw))
    for row, v0 in enumerate((0.45, 0.10)):          # far row first
        for i in range(3):
            u0 = i * (a + 0.005) + 0.004
            slots.append((u0 + a / 2, v0 + b / 2, a, b, 0.0, h1,
                          (rng.uniform(-0.002, 0.002), rng.uniform(-0.015, 0.015),
                           math.radians(rng.uniform(-0.8, 0.8)))))
    for (u, v, du, dv, hh) in ((0.30, 0.60, 0.50, 0.35, 0.25), (0.85, 0.55, 0.40, 0.30, 0.40),
                               (0.55, 0.25, 0.30, 0.20, 0.15)):
        slots.append((u, v, du, dv, h1, hh, (rng.uniform(-0.015, 0.015),
                                             rng.uniform(-0.015, 0.015),
                                             math.radians(rng.uniform(-2, 2)))))
    names = []
    for k, (u, v, du, dv, rest, hh, (eu, ev, ey)) in enumerate(slots):
        cmd = gt.to_world(u, v, rest + hh)                      # commanded top centre
        act = gt.to_world(u + eu, v + ev, rest + hh)
        name = f'box_ppe_{k:02d}'
        yaw = gt.yaw + ey
        pl = Placement()
        pl.top_center.position.x, pl.top_center.position.y, pl.top_center.position.z = \
            map(float, cmd)
        pl.yaw = gt.yaw
        pl.size_x, pl.size_y, pl.height = du, dv, hh
        pl.rest_z = gt.deck_z + rest
        # like the orchestrator: the placement is reported (pending) before the box is there
        report_pub.publish(CycleReport(success=True, outcome='placed', placement=pl,
                                       sim_model_name=name))
        time.sleep(0.2)
        sim.spawn_box(name, (du, dv, hh), 4.0,
                      (act[0], act[1], gt.deck_z + rest + hh / 2 + 0.003), yaw=yaw)
        time.sleep(1.5)
        names.append((name, (du, dv, hh)))
        req = VerifyPlacement.Request()
        req.placement = pl
        req.box = DetectedBox(length=max(du, dv), width=min(du, dv), height=hh)
        req.index = k
        req.sim_model_name = name
        r = sim.call(verify, req, timeout=60.0).check
        top, tyaw = box_top(sim, name, (du, dv, hh))
        gt_err = np.linalg.norm(top[:2] - cmd[:2])
        row = {'index': k, 'detected': r.detected, 'cam_err_xy_mm': r.err_xy * 1000,
               'gt_err_xy_mm': gt_err * 1000}
        if r.detected:
            meas = np.array([r.measured_top.position.x, r.measured_top.position.y,
                             r.measured_top.position.z])
            row.update({
                'agree_xy_mm': float(np.linalg.norm(meas[:2] - top[:2])) * 1000,
                'agree_z_mm': abs(meas[2] - top[2]) * 1000,
                'agree_yaw_deg': math.degrees(abs(normalize_axis(
                    r.measured_yaw - tyaw, math.pi / 2 if abs(du - dv) < 0.01 else math.pi))),
                'dims_err_mm': max(abs(r.measured_length - max(du, dv)),
                                   abs(r.measured_width - min(du, dv))) * 1000})
            print(f'placement {k}: camera error {row["cam_err_xy_mm"]:.1f} mm, true error '
                  f'{row["gt_err_xy_mm"]:.1f} mm -> agreement {row["agree_xy_mm"]:.2f} mm xy, '
                  f'{row["agree_z_mm"]:.2f} mm z, {row["agree_yaw_deg"]:.2f} deg', flush=True)
        else:
            print(f'placement {k}: NOT DETECTED ({r.message})', flush=True)
        out['placements'].append(row)

    # ---------------------------------------------------------------- 3. height map
    time.sleep(1.0)
    hm = maps[-1]
    Hp = np.array(hm.data, dtype=float).reshape(hm.rows, hm.cols)
    res_ = hm.resolution
    f = PalletFrame.from_msg(hm.pallet)
    Hg = np.zeros_like(Hp)
    inner = np.zeros(Hp.shape, bool)       # cell fully inside the top face of the top box
    cu = (np.arange(hm.cols) + 0.5) * res_
    cv = (np.arange(hm.rows) + 0.5) * res_
    CU, CV = np.meshgrid(cu, cv)
    for name, size in names:
        top, tyaw = box_top(sim, name, size)
        u, v = f.to_pallet(top[0], top[1])
        # (u, v) is mirrored w.r.t. world (v = -rot90(u)): a world rotation +a is -a in u, v
        rel = -normalize_axis(tyaw - f.yaw, 2 * math.pi)
        c, s = math.cos(rel), math.sin(rel)
        du, dv = CU - u, CV - v
        a_, b_ = du * c + dv * s, -du * s + dv * c
        h = top[2] - f.deck_z
        half_cell = res_ / 2 * (abs(c) + abs(s))
        cover = (np.abs(a_) <= size[0] / 2 + half_cell) & (np.abs(b_) <= size[1] / 2 + half_cell)
        full = (np.abs(a_) <= size[0] / 2 - half_cell) & (np.abs(b_) <= size[1] / 2 - half_cell)
        higher = h > Hg
        Hg = np.where(cover & higher, h, Hg)
        inner = np.where(cover & higher, full, inner)
    obs = np.isfinite(Hp)
    occ = Hg > 0
    e_in = np.abs(Hp - Hg)[inner & obs]
    e_all = np.abs(Hp - Hg)[occ & obs]
    if args.debug:
        np.savez(args.debug, Hp=Hp, Hg=Hg, inner=inner, frame=[f.origin[0], f.origin[1], f.yaw],
                 boxes=[np.r_[box_top(sim, n, sz)[0], box_top(sim, n, sz)[1], sz]
                        for n, sz in names])
    out['height_map'] = {
        'occupied_cells': int(occ.sum()), 'interior_cells': int(inner.sum()),
        'interior_observed': int((inner & obs).sum()),
        'interior_err_mean_mm': float(e_in.mean() * 1000), 'interior_err_max_mm': float(
            e_in.max() * 1000),
        'interior_within_10mm_pct': float((e_in <= 0.010).mean() * 100),
        'all_occupied_within_10mm_pct': float((e_all <= 0.010).mean() * 100),
        'unobserved_pct': float((~obs & occ).sum() / max(occ.sum(), 1) * 100)}
    print(f'height map: {out["height_map"]}', flush=True)

    # ---------------------------------------------------------------- 4. monitoring
    time.sleep(2.5 * sim.cell['pallet_perception']['monitor_period'])
    false_events = len(events) - n_events0
    thr = sim.cell['pallet_perception']['shift_threshold']
    for k, dist in ((6, 0.010), (7, 0.040), (5, -0.030)):
        name, size = names[k]
        n0 = len(events)
        pos, quat = sim.box_poses[name]
        yaw = Rotation.from_quat(quat).as_euler('xyz')[2]
        d = f.u_axis * dist if k != 5 else f.v_axis * dist
        before = sim.box_poses[name][0].copy()
        sim.set_pose(name, (pos[0] + d[0], pos[1] + d[1], pos[2] + 0.002), yaw)
        # a few monitoring passes (period + acquisition of fresh frames)
        sim.wait_for(lambda: len(events) > n0,
                     4 * sim.cell['pallet_perception']['monitor_period'] + 8.0)
        time.sleep(0.5)
        true_shift = float(np.linalg.norm(sim.box_poses[name][0][:2] - before[:2]))
        mine = [e for e in events[n0:] if e.index == k]
        others = [e for e in events[n0:] if e.index != k]
        row = {'index': k, 'shove_mm': abs(dist) * 1000, 'true_shift_mm': true_shift * 1000,
               'expected_event': true_shift > thr, 'events': len(mine),
               'est_shift_mm': mine[0].shift * 1000 if mine else None,
               'other_events': len(others)}
        print(f'shove box {k} by {abs(dist) * 1000:.0f} mm (true {true_shift * 1000:.1f} mm): '
              f'{len(mine)} event(s) {[round(e.shift * 1000, 1) for e in mine]} mm, '
              f'{len(others)} on other boxes', flush=True)
        out['shove'].append(row)

    # ---------------------------------------------------------------- summary
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, 'pallet_perception_eval.json')
    with open(path, 'w') as fh:
        json.dump(out, fh, indent=1, default=float)
    P = [r for r in out['pallets'] if r['success']]
    Pl = [r for r in out['placements'] if r['detected']]
    dims = max(max(r['err_u_mm'], r['err_v_mm']) for r in P)
    cen = max(r['err_center_mm'] for r in P)
    yawm = max(r['err_yaw_deg'] for r in P)
    agree = max(r['agree_xy_mm'] for r in Pl)
    hmx = out['height_map']['interior_err_max_mm']
    shove_ok = all((r['events'] > 0) == r['expected_event'] and r['other_events'] == 0
                   for r in out['shove'])
    checks = [
        ('pallets measured', len(P) == len(out['pallets']), f'{len(P)}/{len(out["pallets"])}'),
        ('pallet dims error max', dims <= th['dims_mm'], f'{dims:.2f} mm (<= {th["dims_mm"]})'),
        ('pallet position error max', cen <= th['pose_mm'], f'{cen:.2f} mm (<= {th["pose_mm"]})'),
        ('pallet yaw error max', yawm <= th['yaw_deg'], f'{yawm:.3f} deg (<= {th["yaw_deg"]})'),
        ('placements measured', len(Pl) == len(out['placements']),
         f'{len(Pl)}/{len(out["placements"])}'),
        ('camera vs ground-truth placement error', agree <= th['placement_agree_mm'],
         f'max {agree:.2f} mm (<= {th["placement_agree_mm"]})'),
        ('height map vs ground truth (occupied interior cells)', hmx <= th['height_map_mm'],
         f'max {hmx:.2f} mm (<= {th["height_map_mm"]}), '
         f'{out["height_map"]["interior_observed"]} cells'),
        ('no stack events while nothing moves', false_events == 0, f'{false_events} events'),
        ('shoves: event iff moved > threshold, attributed to the shoved box', shove_ok,
         '; '.join(f'{r["true_shift_mm"]:.0f} mm -> {r["events"]}' for r in out['shove'])),
    ]
    print(f'\npallet perception eval (seed {args.seed}), json {path}')
    failed = 0
    for label, ok, text in checks:
        print(f'  [{"PASS" if ok else "FAIL"}] {label}: {text}')
        failed += not ok
    for name, _ in names:
        sim.delete_box(name)
    sim.shutdown()
    rclpy.try_shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
