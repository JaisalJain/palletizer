"""Pattern table for docs/RESULTS.md (reporting only; reads the SKU catalog like the spawner).

For every catalog SKU on a pallet type delivered to the station centre (nominal pose), with the
real reachability map: best count per layer on the deck (theoretical optimum, no
reachability), the layers A / B the planner uses with pattern_priority count and interlock,
boxes per pallet, interlock fraction, volume fill.

Usage: ros2 run palletizer_packing pattern_report [--pallet EUR] [--physics]
"""

import argparse
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np

from palletizer_packing.core.pallet import Box, PalletSpec
from palletizer_packing.core.pattern import PatternConfig, make_pattern
from palletizer_packing.core.reach import PalletReach, ReachabilityMap
from palletizer_packing.core.rules import RuleConfig
from palletizer_tools import cell_config
from palletizer_tools.pallet_frame import PalletFrame


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--pallet', default='EUR')
    ap.add_argument('--physics', action='store_true')
    args = ap.parse_args(argv)
    cell = cell_config.load()
    L, W, H = cell['exchanger']['pallet_types'][args.pallet]['size']
    cx, cy = cell['pallet_station']['center_xy']
    frame = PalletFrame(np.array([cx - L / 2, cy + W / 2]), 0.0, L, W, H)
    m = ReachabilityMap.load(os.path.join(get_package_share_directory('palletizer_motion'),
                                          'config', cell['reachability']['file']))
    reach = PalletReach(m, frame.to_world, L, W)
    rules = RuleConfig.from_config(cell)
    physics = None
    if args.physics:
        from palletizer_packing.core.physics import PhysicsChecker, physics_config
        physics = PhysicsChecker(physics_config(cell))
    print(f'Pallet {args.pallet} {L * 1000:.0f} x {W * 1000:.0f} mm at the station centre, '
          f'gap {cell["packing"]["pattern_gap"] * 1000:.0f} mm.\n')
    print('| SKU | Carton [mm] | Optimum / layer | count: A / B x layers = boxes (interlock) | '
          'interlock: A / B x layers = boxes (interlock) | Fill % (count) | Note (count) |')
    print('|---|---|---|---|---|---|---|')
    for key, e in cell['spawner']['catalog'].items():
        bl, bw, bh = e['size']
        box = Box(max(bl, bw), min(bl, bw), bh)
        usable = reach.usable_top_height(box.width / 2, cell['pallet_station']['max_stack_height'])
        spec = PalletSpec.measured(L, W, cell, usable)
        out = {}
        for prio in ('count', 'interlock'):
            cfg = PatternConfig.from_config(cell)
            cfg.priority = prio
            t0 = time.perf_counter()
            p = make_pattern(box, spec, rules, cfg, reach, physics)
            out[prio] = (p, time.perf_counter() - t0)
        pc, pi = out['count'][0], out['interlock'][0]
        fill = 100 * pc.count * box.volume / (L * W * usable)

        def fmt(p):
            return (f'{p.layer_a.count} / {p.layer_b.count} x {p.layers} = {p.count} '
                    f'({p.interlock * 100:.0f} %)')
        print(f'| {key} | {bl * 1000:.0f} x {bw * 1000:.0f} x {bh * 1000:.0f} | {pc.optimum} | '
              f'{fmt(pc)} | {fmt(pi)} | {fill:.1f} | {pc.note} |')
    return 0


if __name__ == '__main__':
    sys.exit(main())
