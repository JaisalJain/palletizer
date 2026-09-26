"""Milestone C acceptance: N consecutive boxes picked and placed at planner-chosen targets.

Needs the manual-mode stack (sim + perception + packing + motion), e.g.
    SIM_ARGS="seed:=42" src/palletizer_tools/test/sim_ctl.sh start

Per box: spawn (seeded sequence) -> wait for the pick eye -> detect -> plan (geometry pre-check,
pallet exchange + measurement if full) -> pick (+weigh) -> resume conveyor -> plan with
measured mass -> place -> commit. (v2: the pallet is measured by pallet_perception, the
placement also checked by it; ground truth only scores.) Ground truth is used ONLY to score:
  placement position error (top-face center vs commanded) <= 15 mm, yaw error <= 3 deg,
  no already-placed box on the pallet displaced by more than 10 mm.
PASS if >= 19/20 boxes meet all three (thresholds in cell_config pick_place_test).
Writes results/pick_place_test.csv.
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
from palletizer_interfaces.action import PickBox, PlaceBox
from palletizer_interfaces.msg import BoxSpec, CycleReport
from palletizer_interfaces.srv import CommitPlacement, DetectBox, PlanPlacement, VerifyPlacement
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger

from palletizer_tools.sim_client import SimClient

# Planning includes motion-feasibility checks (~1-3 s each, up to motion_check_max): same
# budget as the behavior tree's PlanPlacement node.
PLAN_TIMEOUT = 90.0


def yaw_err(a, b, square):
    period = math.pi / 2 if square else math.pi
    e = (a - b) % period
    return abs(min(e, period - e))


class PickPlaceTest:

    def __init__(self, sim):
        self.sim = sim
        self.specs = {}
        sim.create_subscription(
            BoxSpec, '/sim/box_specs', lambda m: self.specs.__setitem__(m.name, m),
            QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.detect = sim.create_client(DetectBox, '/perception/detect_box')
        self.plan = sim.create_client(PlanPlacement, '/packing/plan_placement')
        self.commit = sim.create_client(CommitPlacement, '/packing/commit')
        self.scene_add = sim.create_client(CommitPlacement, '/motion/scene/add_placed')
        self.triggers = {k: sim.create_client(Trigger, s) for k, s in {
            'reset_packing': '/packing/reset',
            'clear_scene': '/motion/scene/clear_placed', 'reject_pick': '/sim/reject_pick_zone_box',
            'reject_held': '/sim/reject_held_box', 'drop_held': '/motion/drop_held',
            'park': '/motion/park'}.items()}
        self.pick = ActionClient(sim, PickBox, '/motion/pick')
        self.place = ActionClient(sim, PlaceBox, '/motion/place')
        self.box_ready = False
        from std_msgs.msg import Bool
        sim.create_subscription(Bool, '/conveyor_controller/box_ready',
                                lambda m: setattr(self, 'box_ready', m.data), 10)
        self.on_pallet = []          # sim names of boxes on the current pallet
        self.verify = sim.create_client(VerifyPlacement, '/pallet_perception/verify_placement')
        self.report = sim.create_publisher(CycleReport, '/orchestrator/cycle_report', 50)
        self.unverified = None       # (placement, box, sim name, index) of the last place
        self.index = 0
        self.checks = []             # camera placement check vs ground truth

    def verify_last(self):
        """Pallet-camera measurement of the last placement (like the behavior tree: while
        the arm picks the next box); compared with ground truth."""
        if self.unverified is None:
            return
        pl, box, name, idx = self.unverified
        self.unverified = None
        r = self.sim.call(self.verify, VerifyPlacement.Request(
            placement=pl, box=box, index=idx, sim_model_name=name), timeout=90.0)
        if not r.check.detected or name not in self.sim.box_poses:
            self.checks.append({'name': name, 'detected': 0})
            return
        top, _, _ = self.gt_top(name)
        m = r.check.measured_top.position
        self.checks.append({'name': name, 'detected': 1, 'cam_err_mm': r.check.err_xy * 1000,
                            'cam_vs_gt_mm': 1000 * float(np.linalg.norm(top[:2] - [m.x, m.y]))})

    def action(self, client, goal, timeout=180.0):
        client.wait_for_server(timeout_sec=30.0)
        f = client.send_goal_async(goal)
        self.sim.wait_for(f.done, 30.0)
        rf = f.result().get_result_async()
        self.sim.wait_for(rf.done, timeout)
        return rf.result().result

    def trig(self, key):
        return self.sim.call(self.triggers[key], Trigger.Request(), timeout=30.0)

    def swap(self):
        """Pallet exchange: the full pallet leaves with its boxes, a new one is measured."""
        self.sim.start_pallet(exchange=True)
        self.on_pallet = []
        self.unverified = None
        self.index = 0

    def gt_top(self, name):
        spec = self.specs[name]
        pos, quat = self.sim.box_poses[name]
        R = Rotation.from_quat(quat)
        top = pos + R.apply([0, 0, spec.size_z / 2])
        yaw = R.as_euler('xyz')[2] + (math.pi / 2 if spec.size_y > spec.size_x else 0.0)
        return top, yaw, abs(spec.size_x - spec.size_y) < 0.01

    def run_one(self, i):
        sim = self.sim
        row = {'i': i}
        res = None
        for _ in range(120):
            res = sim.call(sim.spawn_box_cli, Trigger.Request())
            if res.success:
                break
            time.sleep(0.5)
        name = res.message.split(':')[0]
        row['name'] = name
        if not sim.wait_for(lambda: self.box_ready, 60.0):
            row['outcome'] = 'no_box_ready'
            return row
        det = sim.call(self.detect, DetectBox.Request(), timeout=15.0)
        if not det.success:
            row['outcome'] = f'detect_failed: {det.message}'
            self.trig('park')
            self.trig('reject_pick')
            sim.call(sim.resume_cli, Trigger.Request())
            return row
        box = det.box
        pre = sim.call(self.plan, PlanPlacement.Request(box=box, mass=0.0, check_only=True),
                       timeout=PLAN_TIMEOUT)
        if not pre.success:
            self.swap()
            row['pallet_swap_before'] = 1
        t0 = time.time()
        pr = None
        for attempt in range(2):
            self.pick.wait_for_server(timeout_sec=30.0)
            f = self.pick.send_goal_async(PickBox.Goal(box=box))
            if attempt == 0:
                sim.wait_for(f.done, 30.0)
                self.verify_last()                 # the arm is on its way to the conveyor
            sim.wait_for(f.done, 30.0)
            rf = f.result().get_result_async()
            sim.wait_for(rf.done, 180.0)
            pr = rf.result().result
            if pr.success:
                break
        if not pr.success:
            row['outcome'] = f'pick_failed:{pr.error_code} {pr.message}'
            self.trig('park')
            self.trig('reject_pick')
            sim.call(sim.resume_cli, Trigger.Request())
            return row
        sim.call(sim.resume_cli, Trigger.Request())
        pl = sim.call(self.plan, PlanPlacement.Request(box=box, mass=pr.measured_mass),
                      timeout=PLAN_TIMEOUT)
        if not pl.success:
            self.swap()
            row['pallet_swap_after'] = 1
            pl = sim.call(self.plan, PlanPlacement.Request(box=box, mass=pr.measured_mass),
                      timeout=PLAN_TIMEOUT)
        before = {n: sim.box_poses[n][0].copy() for n in self.on_pallet if n in sim.box_poses}
        rr = self.action(self.place, PlaceBox.Goal(placement=pl.placement, box=box))
        if not rr.success:          # same recovery as the behavior tree: re-plan elsewhere
            row['replanned'] = 1
            pl = sim.call(self.plan, PlanPlacement.Request(box=box, mass=pr.measured_mass,
                                                           exclude=[pl.placement]),
                          timeout=PLAN_TIMEOUT)
            if pl.success:
                rr = self.action(self.place, PlaceBox.Goal(placement=pl.placement, box=box))
        row['motion_time_s'] = time.time() - t0
        if not rr.success:
            row['outcome'] = f'place_failed:{rr.error_code} {rr.message}'
            self.trig('reject_held')
            self.trig('drop_held')
            self.trig('park')
            return row
        req = CommitPlacement.Request(placement=pl.placement, box=box, mass=pr.measured_mass,
                                      sim_model_name=pr.sim_model_name)
        sim.call(self.commit, req)
        sim.call(self.scene_add, req)
        self.report.publish(CycleReport(success=True, outcome='placed', placement=pl.placement,
                                        sim_model_name=pr.sim_model_name))
        self.unverified = (pl.placement, box, pr.sim_model_name, self.index)
        self.index += 1
        time.sleep(1.5)
        top, yaw, square = self.gt_top(name)
        p = pl.placement
        tgt = np.array([p.top_center.position.x, p.top_center.position.y, p.top_center.position.z])
        moved = [(float(np.linalg.norm(sim.box_poses[n][0] - p0)), n)
                 for n, p0 in before.items() if n in sim.box_poses] or [(0.0, '')]
        disp, moved_name = max(moved)
        row['moved_neighbor'] = moved_name
        self.on_pallet.append(name)
        row.update({
            'outcome': 'placed', 'planner': p.planner, 'rotated': int(p.rotated),
            'place_err_mm': 1000 * float(np.linalg.norm(top - tgt)),
            'place_err_xy_mm': 1000 * float(np.linalg.norm(top[:2] - tgt[:2])),
            'yaw_err_deg': math.degrees(yaw_err(yaw, p.yaw, square)),
            'max_neighbor_disp_mm': 1000 * disp,
            'measured_mass': pr.measured_mass, 'true_mass': self.specs[name].mass})
        return row


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=20)
    ap.add_argument('--out', default=os.path.expanduser('~/palletizer_ws/results'))
    args = ap.parse_args(argv)
    rclpy.init()
    sim = SimClient('pick_place_test')
    sim.wait_ready()
    th = sim.cell['pick_place_test']
    t = PickPlaceTest(sim)
    sim.delete_all_boxes()
    t.swap()
    t.trig('park')
    sim.call(sim.resume_cli, Trigger.Request())
    rows = []
    print(f'== pick_place_test: {args.n} boxes', flush=True)
    for i in range(args.n):
        row = t.run_one(i)
        ok = (row.get('outcome') == 'placed' and row['place_err_mm'] <= th['place_err_mm'] and
              row['yaw_err_deg'] <= th['yaw_err_deg'] and
              row['max_neighbor_disp_mm'] <= th['neighbor_disp_mm'])
        row['pass'] = int(ok)
        rows.append(row)
        if row.get('outcome') == 'placed':
            print(f'  [{"PASS" if ok else "FAIL"}] {i:2d} {row["name"]}: place err '
                  f'{row["place_err_mm"]:.2f} mm, yaw err {row["yaw_err_deg"]:.3f} deg, '
                  f'neighbours moved <= {row["max_neighbor_disp_mm"]:.2f} mm '
                  f'({row.get("moved_neighbor") or "-"}), mass '
                  f'{row["measured_mass"]:.2f}/{row["true_mass"]:.2f} kg'
                  f'{" (pallet swapped)" if row.get("pallet_swap_before") or row.get("pallet_swap_after") else ""}',
                  flush=True)
        else:
            print(f'  [FAIL] {i:2d} {row.get("name")}: {row.get("outcome")}', flush=True)
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, 'pick_place_test.csv')
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    t.trig('park')
    t.verify_last()
    n_ok = sum(r['pass'] for r in rows)
    placed = [r for r in rows if r.get('outcome') == 'placed']
    pe = [r['place_err_mm'] for r in placed]
    ye = [r['yaw_err_deg'] for r in placed]
    nd = [r['max_neighbor_disp_mm'] for r in placed]
    passed = n_ok >= th['min_success'] * args.n / 20.0
    print(f'\npick_place_test: {n_ok}/{args.n} boxes met all criteria '
          f'(need >= {th["min_success"]}/20)')
    if placed:
        print(f'  placement error mean {np.mean(pe):.2f} / max {np.max(pe):.2f} mm '
              f'(<= {th["place_err_mm"]}), yaw error mean {np.mean(ye):.3f} / max '
              f'{np.max(ye):.3f} deg (<= {th["yaw_err_deg"]}), neighbour displacement max '
              f'{np.max(nd):.2f} mm (<= {th["neighbor_disp_mm"]})')
    cv = [c['cam_vs_gt_mm'] for c in t.checks if c['detected']]
    if cv:
        print(f'  pallet camera: {len(cv)}/{len(t.checks)} placements measured, camera vs '
              f'ground truth max {max(cv):.2f} mm (camera placement error max '
              f'{max(c["cam_err_mm"] for c in t.checks if c["detected"]):.2f} mm)')
    print(f'pick_place_test: {"PASS" if passed else "FAIL"}  (csv {path})')
    sim.shutdown()
    rclpy.try_shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
