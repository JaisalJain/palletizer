"""Offline top-down reachability map over the pallet STATION AREA, in world coordinates.

The robot does not know the pallet in advance, so the map covers the whole area where the
pallet conveyor may deliver one (pallet_station.area): for every grid position (x, y) of a
box's top-face centre, every box-top height z (world) and tool yaw 0 deg, a cell is
reachable iff:
  * IK exists (UR10e joint limits) for the pre-place pose (z + pre_place_height) and the
    place pose (z + release_gap), on one continuous branch along the vertical LIN line, and
  * both states are collision-free against the static planning scene (MoveIt
    /check_state_validity; needs move_group + motion_server running), with the station
    floor modelled as a slab up to pallet_station.max_deck_z (the highest deck accepted).
Tool yaw 90 deg is not computed: v1 showed the maps are identical (with the tool pointing
down, yaw only moves wrist_3, which has +/-360 deg range).

At run time the packing node intersects this map with the MEASURED pallet (its footprint and
deck height) to get the per-pallet usable stack height (core/reach.py PalletReach).

Writes palletizer_motion/config/<reachability.file> (source tree, so it is versioned) and
prints the coverage per height over the station area.

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


def grid(cell):
    r = cell['reachability']
    st = cell['pallet_station']
    step, inset = r['grid_step'], r['edge_inset']
    (cx, cy), (ax, ay) = st['center_xy'], st['area']
    # nodes symmetric about the station centre (a pallet delivered there has its box
    # positions on the nodes, as in v1; the lookup is conservative between nodes)
    nx = int(np.floor((ax / 2 - inset) / step + 1e-9))
    ny = int(np.floor((ay / 2 - inset) / step + 1e-9))
    xs = np.round(cx + step * np.arange(-nx, nx + 1), 4)
    ys = np.round(cy + step * np.arange(-ny, ny + 1), 4)
    z0, z1, zs = r['top_z']
    return xs, ys, np.round(np.arange(z0, z1 + 1e-9, zs), 4)


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
    x, y, yaw, zs = args
    m = _CELL['motion']
    out = []
    seed = _seed((x, y))
    for z in zs:
        z_place = z + m['release_gap']
        z_pre = z + m['pre_place_height']
        q_pre = _KIN.ik(_KIN.down_pose((x, y, z_pre), yaw), seed)
        if q_pre is None:
            out.append(None)
            continue
        q, ok = q_pre, True
        for zz in np.linspace(z_pre, z_place, 4)[1:]:
            q_next = _KIN.ik(_KIN.down_pose((x, y, zz), yaw), q)
            if q_next is None or np.max(np.abs(q_next - q)) > 0.35:
                ok = False
                break
            q = q_next
        out.append((q_pre, q) if ok else None)
        seed = q_pre
    return out


def build(cell, check_collision=True, workers=None):
    us, vs, hs = grid(cell)
    yaws = [0.0]
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
        from palletizer_motion.moveit_client import box_object, make_pose
        st = cell['pallet_station']
        dz = st['max_deck_z']
        slab = box_object('station_slab', 'world', (st['area'][0], st['area'][1], dz),
                          make_pose((st['center_xy'][0], st['center_xy'][1], dz / 2)))
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.running = True
        self.t = threading.Thread(target=self._spin, daemon=True)
        self.t.start()
        if not self.mv.validity.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('/check_state_validity not available (start move_group)')
        self.mv.add_objects([slab])

    def _spin(self):
        while self.running:
            self.ex.spin_once(timeout_sec=0.05)

    def __call__(self, q):
        return self.mv.state_valid(q)

    def close(self):
        self.mv.remove_objects(['station_slab'])
        self.running = False
        self.t.join(1.0)
        self.rclpy.try_shutdown()


def _validity_checker(cell):
    return _Validity(cell)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=None)
    ap.add_argument('--no-collision', action='store_true')
    ap.add_argument('--workers', type=int, default=None)
    args = ap.parse_args(argv)
    cell = cell_config.load(role='robot')
    out = args.out or os.path.join(os.path.expanduser('~/palletizer_ws/src/palletizer_motion/config'),
                                   cell['reachability']['file'])
    xs, ys, zs, yaws, reach = build(cell, not args.no_collision, args.workers)
    np.savez_compressed(out, x=xs, y=ys, z=zs, yaw=yaws, reach=reach,
                        collision_checked=not args.no_collision)
    print(f'saved {out}')
    print('box-top z [m] (world) : coverage over the station area')
    for i, z in enumerate(zs):
        print(f'  {z:5.2f} : {reach[:, i].any(axis=0).mean() * 100:6.1f}%')
    return 0


if __name__ == '__main__':
    sys.exit(main())
