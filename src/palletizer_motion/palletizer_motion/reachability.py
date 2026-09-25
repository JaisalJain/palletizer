"""Offline top-down reachability map over the pallet footprint.

For every grid position (u, v) of a box's top-face center (pallet frame: u along the pallet
length from its -x edge, v from the robot-side edge into the pallet), every box-top height h
above the pallet top and tool yaw 0 / 90 deg, a cell is reachable iff:
  * IK exists (UR10e joint limits) for the pre-place pose (h + pre_place_height) and the
    place pose (h + release_gap), on one continuous branch along the vertical LIN line, and
  * both states are collision-free against the static planning scene (MoveIt
    /check_state_validity; needs move_group + motion_server running).

Writes palletizer_motion/config/<reachability.file> (source tree, so it is versioned) and
prints the coverage per height plus the derived usable max stack height.

Usage: ros2 run palletizer_motion build_reachability_map [--out PATH] [--no-collision]
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import math
import os
import sys
import time

import numpy as np

from palletizer_tools import cell_config
from palletizer_tools.ur_ik import UrKinematics

_KIN = None
_CELL = None


def pallet_axes(cell):
    pal = cell['pallet']
    x0 = pal['center_xy'][0] - pal['size'][0] / 2
    y_edge = pal['center_xy'][1] + pal['size'][1] / 2      # robot-side edge
    return x0, y_edge, pal['size'][2]


def grid(cell):
    r = cell['reachability']
    pal = cell['pallet']
    step, inset = r['grid_step'], r['edge_inset']
    us = np.round(np.arange(inset, pal['size'][0] - inset + 1e-9, step), 4)
    vs = np.round(np.arange(inset, pal['size'][1] - inset + 1e-9, step), 4)
    h0, h1, hs = r['top_heights']
    hs_ = np.round(np.arange(h0, h1 + 1e-9, hs), 4)
    return us, vs, hs_


def _init(cell):
    global _KIN, _CELL
    _CELL = cell
    _KIN = UrKinematics(cell)


def _seed(xy):
    q = np.array(_CELL['robot']['home'], dtype=float)
    tcp = _KIN.fk(q)[:3, 3]
    offset = math.atan2(tcp[1], tcp[0]) - q[0]
    q[0] = math.atan2(xy[1], xy[0]) - offset
    return q


def _column(args):
    """IK along one (u, v, yaw) column for all heights. Returns list of (q_pre, q_place)."""
    u, v, yaw, hs = args
    x0, y_edge, ptop = pallet_axes(_CELL)
    m = _CELL['motion']
    x, y = x0 + u, y_edge - v
    out = []
    seed = _seed((x, y))
    for h in hs:
        z_place = ptop + h + m['release_gap']
        z_pre = ptop + h + m['pre_place_height']
        q_pre = _KIN.ik(_KIN.down_pose((x, y, z_pre), yaw), seed)
        if q_pre is None:
            out.append(None)
            continue
        q, ok = q_pre, True
        for z in np.linspace(z_pre, z_place, 4)[1:]:
            q_next = _KIN.ik(_KIN.down_pose((x, y, z), yaw), q)
            if q_next is None or np.max(np.abs(q_next - q)) > 0.35:
                ok = False
                break
            q = q_next
        out.append((q_pre, q) if ok else None)
        seed = q_pre
    return out


def build(cell, check_collision=True, workers=None):
    us, vs, hs = grid(cell)
    yaws = [0.0, math.pi / 2]
    jobs = [(u, v, yaw, hs) for yaw in yaws for v in vs for u in us]
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=workers, initializer=_init, initargs=(cell,)) as ex:
        cols = list(ex.map(_column, jobs, chunksize=4))
    print(f'IK done in {time.time() - t0:.0f} s for {len(jobs) * len(hs)} cells', flush=True)

    reach = np.zeros((len(yaws), len(hs), len(vs), len(us)), dtype=bool)
    validity = None
    if check_collision:
        validity = _validity_checker(cell)
    for (u, v, yaw, _), col in zip(jobs, cols):
        iy, iv, iu = yaws.index(yaw), int(np.where(vs == v)[0][0]), int(np.where(us == u)[0][0])
        for ih, sol in enumerate(col):
            if sol is None:
                continue
            ok = True
            if validity is not None:
                ok = validity(sol[0]) and validity(sol[1])
            reach[iy, ih, iv, iu] = ok
    if validity is not None:
        validity.close()
    return us, vs, hs, np.array(yaws), reach


class _Validity:
    def __init__(self, cell):
        import rclpy
        from palletizer_motion.moveit_client import MoveItClient
        rclpy.init()
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        import threading
        self.rclpy = rclpy
        self.node = Node('reachability_map')
        self.mv = MoveItClient(self.node, cell['motion']['planning_group'], cell['motion']['tcp_link'])
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.running = True
        self.t = threading.Thread(target=self._spin, daemon=True)
        self.t.start()
        if not self.mv.validity.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('/check_state_validity not available (start move_group)')

    def _spin(self):
        while self.running:
            self.ex.spin_once(timeout_sec=0.05)

    def __call__(self, q):
        return self.mv.state_valid(q)

    def close(self):
        self.running = False
        self.t.join(1.0)
        self.rclpy.try_shutdown()


def _validity_checker(cell):
    return _Validity(cell)


def usable_height(hs, reach, min_coverage):
    """Usable max stack (box-top) height: starting at the lowest level with full coverage,
    the highest h such that every level up to h keeps coverage >= min_coverage (a cell counts
    if any yaw is reachable; yaw only changes wrist_3).

    Low levels directly in front of the pedestal are not fully covered (the folded arm
    collides with the pedestal): that is a near-field keep-out handled per cell by the
    map filter, not a height limit, so the scan starts at the first fully covered level."""
    any_yaw = reach.any(axis=0)
    cov = any_yaw.reshape(len(hs), -1).mean(axis=1)
    full = [i for i, c in enumerate(cov) if c + 1e-9 >= min_coverage]
    if not full:
        return 0.0, cov
    usable = float(hs[full[0]])
    for i in range(full[0], len(hs)):
        if cov[i] + 1e-9 < min_coverage:
            break
        usable = float(hs[i])
    return usable, cov


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=None)
    ap.add_argument('--no-collision', action='store_true')
    ap.add_argument('--workers', type=int, default=None)
    args = ap.parse_args(argv)
    cell = cell_config.load()
    out = args.out or os.path.join(os.path.expanduser('~/palletizer_ws/src/palletizer_motion/config'),
                                   cell['reachability']['file'])
    us, vs, hs, yaws, reach = build(cell, not args.no_collision, args.workers)
    usable, cov = usable_height(hs, reach, cell['reachability']['min_coverage'])
    np.savez_compressed(out, u=us, v=vs, h=hs, yaw=yaws, reach=reach, usable_top_height=usable,
                        coverage=cov, pallet_top=pallet_axes(cell)[2],
                        collision_checked=not args.no_collision)
    print(f'saved {out}')
    same = np.array_equal(reach[0], reach[1])
    print(f'yaw 0 and 90 deg maps identical: {same} '
          f'(differences: {int((reach[0] != reach[1]).sum())} cells)')
    print('box-top height [m] : coverage (any yaw) | yaw0 | yaw90')
    for i, h in enumerate(hs):
        print(f'  {h:5.2f} : {cov[i] * 100:6.1f}% | {reach[0, i].mean() * 100:6.1f}% | '
              f'{reach[1, i].mean() * 100:6.1f}%')
    print(f'usable max stack (box-top) height: {usable:.2f} m above the pallet top')
    return 0


if __name__ == '__main__':
    sys.exit(main())
