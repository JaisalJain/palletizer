#!/usr/bin/env python3
"""docs/media/patterns.png: layer A / B of the single-SKU pattern planner for six SKUs.

Planner output (palletizer_packing.core.pattern.make_pattern) with the real reachability map,
EUR pallet at the station centre (the unseen 347x253x219 carton on its 1140x760 pallet, as in
the unknown-SKU run). Needs the workspace overlay (reachability map, catalog).
"""

import os
import sys

import numpy as np
from matplotlib.patches import Rectangle

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402

from ament_index_python.packages import get_package_share_directory  # noqa: E402
from palletizer_packing.core.pallet import Box, PalletSpec  # noqa: E402
from palletizer_packing.core.pattern import PatternConfig, far_first, make_pattern  # noqa: E402
from palletizer_packing.core.reach import PalletReach, ReachabilityMap  # noqa: E402
from palletizer_packing.core.rules import RuleConfig  # noqa: E402
from palletizer_tools import cell_config  # noqa: E402
from palletizer_tools.pallet_frame import PalletFrame  # noqa: E402

# (SKU key or LxWxH mm, pallet L x W x H m or None = EUR, caption)
SKUS = [
    ('520x320x250', None, 'interlocked'),
    ('500x300x250', None, 'interlocked'),
    ('347x253x219', (1.14, 0.76, 0.15), 'unseen carton + pallet'),
    ('450x350x300', None, 'interlocked'),
    ('380x280x220', None, 'column stack'),
    ('300x200x200', None, 'column stack, 1 slot unreachable'),
]


def plan(cell, rmap, key, pallet):
    L, W, H = pallet or cell['exchanger']['pallet_types']['EUR']['size']
    cx, cy = cell['pallet_station']['center_xy']
    frame = PalletFrame(np.array([cx - L / 2, cy + W / 2]), 0.0, L, W, H)
    reach = PalletReach(rmap, frame.to_world, L, W)
    rules = RuleConfig.from_config(cell)
    if key in cell['spawner']['catalog']:
        bl, bw, bh = cell['spawner']['catalog'][key]['size']
    else:
        bl, bw, bh = (float(v) / 1000 for v in key.split('x'))
    box = Box(max(bl, bw), min(bl, bw), bh)
    usable = reach.usable_top_height(box.width / 2, cell['pallet_station']['max_stack_height'])
    spec = PalletSpec.measured(L, W, cell, usable)
    p = make_pattern(box, spec, rules, PatternConfig.from_config(cell), reach, None)
    return p, L, W


def draw_layer(ax, rects, L, W, color, below=None):
    ax.add_patch(Rectangle((0, 0), L, W, facecolor=common.PANEL, edgecolor=common.MUTED,
                           linewidth=1.2))
    for r in below or []:
        ax.add_patch(Rectangle((r.u0, r.v0), r.du, r.dv, fill=False, edgecolor=common.TEXT2,
                               linewidth=1.0, linestyle=(0, (3, 3)), alpha=0.55, zorder=3))
    for i, r in enumerate(far_first(rects), start=1):
        ax.add_patch(Rectangle((r.u0, r.v0), r.du, r.dv, facecolor=color, edgecolor=common.BG,
                               linewidth=2.0, alpha=0.92, zorder=2))
        cu, cv = r.center
        ax.text(cu, cv, str(i), ha='center', va='center', fontsize=9, color=common.TEXT,
                alpha=0.85, zorder=4)
    ax.set_xlim(-0.03, 1.23)
    ax.set_ylim(-0.12, 0.83)
    ax.set_aspect('equal')
    ax.axis('off')


def main():
    plt = common.mpl_dark()
    cell = cell_config.load()          # reporting only: reads the catalog like the spawner
    rmap = ReachabilityMap.load(os.path.join(get_package_share_directory('palletizer_motion'),
                                             'config', cell['reachability']['file']))
    FW, FH = 16.0, 7.6                  # figure [in]; manual layout: 3 x 2 SKU cells
    fig = plt.figure(figsize=(FW, FH))
    fig.text(0.4 / FW, 1 - 0.55 / FH, 'Single-SKU layer patterns', ha='left', fontsize=20,
             fontweight='bold')
    fig.text(0.4 / FW, 1 - 0.92 / FH, 'Layer A / layer B, top-down, robot at the bottom. '
             'Numbers = placement order (far side first). Dashed = the layer below: boxes '
             'across its seams interlock the stack.', ha='left', fontsize=12, color=common.TEXT2)
    cell_w, ax_w, gap = 5.2, 2.45, 0.1
    ax_h = ax_w * 0.95 / 1.26
    for k, (key, pallet, caption) in enumerate(SKUS):
        p, L, W = plan(cell, rmap, key, pallet)
        row, col = divmod(k, 3)
        x0 = 0.4 + col * cell_w
        top = FH - 1.35 - row * 3.2          # top of this cell's title block [in]
        y0 = top - 0.95 - ax_h
        axa = fig.add_axes([x0 / FW, y0 / FH, ax_w / FW, ax_h / FH])
        axb = fig.add_axes([(x0 + ax_w + gap) / FW, y0 / FH, ax_w / FW, ax_h / FH])
        a, b = p.layer_a.rects, p.layer_b.rects
        draw_layer(axa, a, L, W, common.BLUE)
        draw_layer(axb, b, L, W, common.ORANGE, below=a)
        axa.text(0.0, -0.075, 'A', fontsize=11, color=common.TEXT2, fontweight='bold')
        axb.text(0.0, -0.075, 'B', fontsize=11, color=common.TEXT2, fontweight='bold')
        pal = 'EUR 1200×800' if pallet is None else f'{L * 1000:.0f}×{W * 1000:.0f} pallet'
        fig.text(x0 / FW, (top - 0.3) / FH, f'{key.replace("x", "×")} mm', fontsize=15,
                 fontweight='bold', ha='left')
        fig.text(x0 / FW, (top - 0.58) / FH,
                 f'{p.layer_a.count} + {p.layer_b.count} per layer · {p.layers} layers = '
                 f'{p.count} boxes · interlock {p.interlock * 100:.0f} %',
                 fontsize=11, color=common.TEXT, ha='left')
        fig.text(x0 / FW, (top - 0.82) / FH, f'{pal} · {caption}', fontsize=10.5,
                 color=common.TEXT2, ha='left')
        print(f'{key}: {p.count} boxes, interlock {p.interlock:.2f} ({p.note})')
    common.save_fig(fig, common.out('patterns.png'), common.FULL_W)


if __name__ == '__main__':
    main()
