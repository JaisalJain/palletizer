"""Metrics logger + collapse detector (EVALUATION ONLY - uses ground truth).

Inputs: /orchestrator/cycle_report, /orchestrator/pallet_report (from the orchestrator),
        /sim/box_specs, /sim/box_poses (ground truth).
Outputs (CSV in results_dir):
  <run>_boxes.csv   per box: dims (detected vs true), perception error, measured vs true mass,
                    planner, planning time, placement error (position/yaw vs commanded),
                    cycle time, outcome
  <run>_pallets.csv per pallet: boxes, volume fill %, max height, collapses, rejects,
                    duration, picks/min, mean planning time
Collapse detector: every placed box's ground-truth pose is recorded `settle_time` after it
is reported; any later drift > collapse_threshold (20 mm) counts as one collapse.
Publishes /metrics/summary (String JSON) after every pallet.
"""

import csv
import json
import math
import os
import time

import numpy as np
from palletizer_interfaces.msg import BoxSpec, CycleReport, PalletReport
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from palletizer_tools import boxes

BOX_FIELDS = [
    'run', 'planner', 'seed', 'pallet', 'box_index', 'sim_name', 'outcome', 'success',
    'det_L', 'det_W', 'det_H', 'true_L', 'true_W', 'true_H', 'dim_err_max_mm',
    'measured_mass', 'true_mass', 'mass_err_kg', 'planning_time_s', 'candidates',
    'physics_checks', 'target_x', 'target_y', 'target_z', 'target_yaw_deg', 'rotated',
    'place_err_mm', 'place_err_xy_mm', 'place_yaw_err_deg', 'cycle_time_s', 'detect_attempts',
    'grasp_attempts', 'wall_time']
PALLET_FIELDS = [
    'run', 'planner', 'seed', 'pallet', 'boxes', 'rejected', 'box_volume_m3', 'fill_percent',
    'max_height_m', 'collapses', 'lost_boxes', 'duration_s', 'picks_per_min', 'mean_planning_time_s',
    'max_planning_time_s', 'mean_cycle_time_s', 'mean_place_err_mm', 'max_place_err_mm',
    'mean_mass_err_kg', 'end_reason', 'wall_time']


def yaw_err(a, b, square):
    period = math.pi / 2 if square else math.pi
    e = (a - b) % period
    return abs(min(e, period - e))


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')
        self.results_dir = os.path.expanduser(self.declare_parameter(
            'results_dir', '~/palletizer_ws/results').value)
        self.run = self.declare_parameter('run_name', time.strftime('run_%Y%m%d_%H%M%S')).value
        self.planner = self.declare_parameter('planner', '').value
        self.seed = int(self.declare_parameter('seed', -1).value)
        self.settle = float(self.declare_parameter('settle_time', 1.5).value)
        self.collapse_thr = float(self.declare_parameter('collapse_threshold', 0.020).value)
        os.makedirs(self.results_dir, exist_ok=True)
        self.box_csv = os.path.join(self.results_dir, f'{self.run}_boxes.csv')
        self.pallet_csv = os.path.join(self.results_dir, f'{self.run}_pallets.csv')
        for path, fields in ((self.box_csv, BOX_FIELDS), (self.pallet_csv, PALLET_FIELDS)):
            if not os.path.exists(path):
                with open(path, 'w', newline='') as f:
                    csv.DictWriter(f, fieldnames=fields).writeheader()

        self.specs = {}
        self.poses = {}
        self.pending = []          # (due_time, report) waiting for the box to settle
        self.tracked = {}          # name -> reference pose (pos, quat) on the current pallet
        self.collapsed = set()
        self.lost = set()          # boxes that left the belt/pallet (floor), whole run
        self.lost_this_pallet = 0
        self.rows = []             # current pallet box rows
        from palletizer_tools import cell_config
        self.cell = cell_config.load()
        self.pallets_done = 0
        self.create_subscription(
            BoxSpec, '/sim/box_specs', lambda m: self.specs.__setitem__(m.name, m),
            QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, '/sim/box_poses', self.on_poses, 10)
        self.create_subscription(CycleReport, '/orchestrator/cycle_report', self.on_cycle, 50)
        self.create_subscription(
            PalletReport, '/orchestrator/pallet_report', self.on_pallet,
            QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.summary_pub = self.create_publisher(String, '/metrics/summary', 10)
        self.create_timer(0.25, self.tick)
        self.get_logger().info(f'metrics -> {self.box_csv}, {self.pallet_csv}')

    def on_poses(self, msg):
        self.poses = boxes.poses_from_tf(msg)

    def on_cycle(self, r):
        self.pending.append((time.monotonic() + self.settle, r))

    # ------------------------------------------------------------------ per box
    def finalize_box(self, r):
        spec = self.specs.get(r.sim_model_name)
        d = r.detected
        row = {k: '' for k in BOX_FIELDS}
        row.update({
            'run': self.run, 'planner': self.planner or r.placement.planner, 'seed': self.seed,
            'pallet': r.pallet_index, 'box_index': r.box_index, 'sim_name': r.sim_model_name,
            'outcome': r.outcome, 'success': int(r.success),
            'det_L': d.length, 'det_W': d.width, 'det_H': d.height,
            'measured_mass': r.measured_mass, 'cycle_time_s': r.cycle_time,
            'detect_attempts': r.detect_attempts, 'grasp_attempts': r.grasp_attempts,
            'wall_time': time.time()})
        if r.success:
            p = r.placement
            row.update({'planning_time_s': p.planning_time, 'candidates': p.candidates_checked,
                        'physics_checks': p.physics_checks, 'target_x': p.top_center.position.x,
                        'target_y': p.top_center.position.y, 'target_z': p.top_center.position.z,
                        'target_yaw_deg': math.degrees(p.yaw), 'rotated': int(p.rotated)})
        if spec is not None:
            L, W = max(spec.size_x, spec.size_y), min(spec.size_x, spec.size_y)
            row.update({'true_L': L, 'true_W': W, 'true_H': spec.size_z, 'true_mass': spec.mass,
                        'mass_err_kg': abs(r.measured_mass - spec.mass) if r.measured_mass else '',
                        'dim_err_max_mm': 1000 * max(abs(d.length - L), abs(d.width - W),
                                                     abs(d.height - spec.size_z))})
            if r.success and r.sim_model_name in self.poses:
                pos, quat = self.poses[r.sim_model_name]
                R = Rotation.from_quat(quat)
                top = pos + R.apply([0, 0, spec.size_z / 2])
                yaw = R.as_euler('xyz')[2] + (math.pi / 2 if spec.size_y > spec.size_x else 0.0)
                tgt = np.array([p.top_center.position.x, p.top_center.position.y,
                                p.top_center.position.z])
                row.update({
                    'place_err_mm': 1000 * float(np.linalg.norm(top - tgt)),
                    'place_err_xy_mm': 1000 * float(np.linalg.norm(top[:2] - tgt[:2])),
                    'place_yaw_err_deg': math.degrees(yaw_err(yaw, p.yaw, abs(L - W) < 0.01))})
                self.tracked[r.sim_model_name] = (pos.copy(), quat.copy())
        with open(self.box_csv, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=BOX_FIELDS).writerow(row)
        self.rows.append(row)

    # ------------------------------------------------------------------ collapse detector
    def check_collapses(self):
        for name, (p0, _) in self.tracked.items():
            if name in self.collapsed or name not in self.poses:
                continue
            drift = float(np.linalg.norm(self.poses[name][0] - p0))
            if drift > self.collapse_thr:
                self.collapsed.add(name)
                self.get_logger().warn(f'COLLAPSE: {name} drifted {drift * 1000:.0f} mm')

    def check_lost(self):
        """Box ledger: any box below belt level that is not over the pallet fell off."""
        belt = self.cell['conveyor']['belt_top_z']
        pal = self.cell['pallet']
        cx, cy = pal['center_xy']
        hx, hy = pal['size'][0] / 2 + 0.05, pal['size'][1] / 2 + 0.05
        for name, (pos, _) in self.poses.items():
            if name in self.lost:
                continue
            over_pallet = abs(pos[0] - cx) <= hx and abs(pos[1] - cy) <= hy
            if pos[2] < belt - 0.15 and not over_pallet:
                self.lost.add(name)
                self.lost_this_pallet += 1
                self.get_logger().warn(f'LOST BOX: {name} fell off at {pos.round(2).tolist()}')

    def tick(self):
        self.check_lost()
        now = time.monotonic()
        due = [x for x in self.pending if x[0] <= now]
        self.pending = [x for x in self.pending if x[0] > now]
        for _, r in due:
            self.finalize_box(r)
        self.check_collapses()

    # ------------------------------------------------------------------ per pallet
    def on_pallet(self, r):
        # finish boxes still settling first; check collapses on the final pallet state
        for _, c in self.pending:
            if c.pallet_index == r.pallet_index:
                self.finalize_box(c)
        self.pending = [x for x in self.pending if x[1].pallet_index != r.pallet_index]
        self.check_collapses()
        rows = [x for x in self.rows if x['pallet'] == r.pallet_index]
        placed = [x for x in rows if x['success'] == 1]

        def col(k, rs=placed):
            return [float(x[k]) for x in rs if x[k] != '']
        pt, ct, pe, me = col('planning_time_s'), col('cycle_time_s'), col('place_err_mm'), \
            col('mass_err_kg')
        row = {
            'run': self.run, 'planner': self.planner or r.planner, 'seed': self.seed,
            'pallet': r.pallet_index, 'boxes': r.boxes,
            'rejected': sum(1 for x in rows if x['success'] == 0),
            'box_volume_m3': r.box_volume, 'fill_percent': r.fill_percent,
            'max_height_m': r.max_height, 'collapses': len(self.collapsed),
            'lost_boxes': self.lost_this_pallet,
            'duration_s': r.duration,
            'picks_per_min': 60.0 * r.boxes / r.duration if r.duration > 0 else 0.0,
            'mean_planning_time_s': float(np.mean(pt)) if pt else '',
            'max_planning_time_s': float(np.max(pt)) if pt else '',
            'mean_cycle_time_s': float(np.mean(ct)) if ct else '',
            'mean_place_err_mm': float(np.mean(pe)) if pe else '',
            'max_place_err_mm': float(np.max(pe)) if pe else '',
            'mean_mass_err_kg': float(np.mean(me)) if me else '',
            'end_reason': r.end_reason, 'wall_time': time.time()}
        with open(self.pallet_csv, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=PALLET_FIELDS).writerow(row)
        self.pallets_done += 1
        self.summary_pub.publish(String(data=json.dumps(row)))
        self.get_logger().info(
            f'pallet {r.pallet_index}: {r.boxes} boxes, fill {r.fill_percent:.1f} %, '
            f'collapses {len(self.collapsed)}, {row["picks_per_min"]:.2f} picks/min')
        self.tracked.clear()
        self.collapsed.clear()
        self.lost_this_pallet = 0


def main():
    rclpy.init()
    rclpy.spin(MetricsLogger())
    return 0


if __name__ == '__main__':
    main()
