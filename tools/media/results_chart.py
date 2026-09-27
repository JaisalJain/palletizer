#!/usr/bin/env python3
"""docs/media/results.png: picks/min v1 -> v2 (every pallet a dot, mean labelled) and the
volume fill of every single-SKU acceptance pallet. Read from the results/ CSVs only."""

import csv
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402


def rows(pattern):
    out = []
    for f in sorted(glob.glob(os.path.join(common.RESULTS, pattern))):
        out += list(csv.DictReader(open(f)))
    return out


def full_sku_pallets():
    """First pallet of every single-SKU acceptance run that ended with the pattern complete
    (the 9 catalog SKUs + the unseen carton; the SKU-change run is a partial pallet)."""
    return [r for r in rows('sku_*_pallets.csv')
            if r['pallet'] == '0' and r['end_reason'] == 'PATTERN_COMPLETE']


def main():
    plt = common.mpl_dark()
    groups = [
        ('v1 · mixed · naive', rows('bench_naive_seed*_pallets.csv'), common.MUTED),
        ('v1 · mixed · heuristic', rows('bench_heuristic_seed*_pallets.csv'), common.MUTED),
        ('v2 · mixed · naive', rows('v2bench_naive_seed*_pallets.csv'), common.BLUE),
        ('v2 · mixed · heuristic', rows('v2bench_heuristic_seed*_pallets.csv'), common.BLUE),
        ('v2 · single-SKU', full_sku_pallets(), common.BLUE),
    ]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6.4),
                                   gridspec_kw={'width_ratios': [1, 1.05], 'wspace': 0.42})
    fig.subplots_adjust(left=0.13, right=0.97, top=0.80, bottom=0.12)
    rng = np.random.default_rng(0)
    for i, (label, rs, color) in enumerate(groups[::-1]):
        v = np.array([float(r['picks_per_min']) for r in rs])
        y = i + rng.uniform(-0.16, 0.16, len(v))
        ax1.scatter(v, y, s=46, color=color, alpha=0.75, edgecolor=common.BG, linewidth=1.2,
                    zorder=3)
        ax1.plot([v.mean()] * 2, [i - 0.3, i + 0.3], color=common.TEXT, linewidth=2.2,
                 solid_capstyle='round', zorder=4)
        x = max(v.max(), v.mean())
        ax1.annotate(f'{v.mean():.2f}', (x, i), xytext=(9, 0), textcoords='offset points',
                     va='center', fontsize=12.5, fontweight='bold', color=common.TEXT,
                     bbox=dict(fc=common.BG, ec='none', pad=1), zorder=5)
        ax1.annotate(f'n = {len(v)}', (x, i), xytext=(52, 0), textcoords='offset points',
                     va='center', fontsize=10.5, color=common.TEXT2,
                     bbox=dict(fc=common.BG, ec='none', pad=1), zorder=5)
    ax1.set_yticks(range(len(groups)))
    ax1.set_yticklabels([g[0] for g in groups[::-1]], fontsize=12)
    ax1.set_xlim(0, 6.3)
    ax1.axvline(4.0, color=common.ORANGE, linewidth=1.4, linestyle=(0, (4, 3)), zorder=2)
    ax1.text(4.0, len(groups) - 0.45, ' v2 target ≥ 4', color=common.ORANGE, fontsize=11,
             va='bottom')
    ax1.set_xlabel('picks per minute (sim time), every dot = one pallet')
    ax1.grid(axis='x')
    ax1.tick_params(axis='y', length=0)
    ax1.spines['left'].set_visible(False)
    ax1.set_ylim(-0.6, len(groups) - 0.2)
    ax1.set_title('Throughput: v1 → v2', loc='left', pad=14)

    sku = sorted(full_sku_pallets(), key=lambda r: float(r['fill_percent']))
    names = []
    for r in sku:
        key = r['run'].replace('sku_', '')
        if key == 'unknown':
            key = (f'{float(r["sku_L"]) * 1000:.0f}x{float(r["sku_W"]) * 1000:.0f}x'
                   f'{float(r["sku_H"]) * 1000:.0f} (unseen)')
        names.append(key.replace('x', '×'))
    fill = [float(r['fill_percent']) for r in sku]
    y = np.arange(len(sku))
    ax2.barh(y, fill, height=0.62, color=common.BLUE, zorder=3)
    for yi, f, r in zip(y, fill, sku):
        ax2.annotate(f'{f:.0f} %', (f, yi), xytext=(7, 0), textcoords='offset points',
                     va='center', fontsize=11.5, fontweight='bold')
        ax2.annotate(f'{r["boxes"]} boxes', (f, yi), xytext=(46, 0), textcoords='offset points',
                     va='center', fontsize=10.5, color=common.TEXT2,
                     bbox=dict(fc=common.BG, ec='none', pad=1), zorder=5)
    ax2.set_yticks(y)
    ax2.set_yticklabels(names, fontsize=11.5)
    ax2.set_xlim(0, 100)
    ax2.set_xlabel('volume fill % (box volume / deck area × usable height)')
    ax2.grid(axis='x')
    ax2.tick_params(axis='y', length=0)
    ax2.spines['left'].set_visible(False)
    ax2.set_title('Single-SKU pallets: fill, 0 collapses', loc='left', pad=14)

    fig.suptitle('Results', x=0.02, y=0.965, ha='left', fontsize=20, fontweight='bold')
    fig.text(0.02, 0.885, 'From results/*.csv (Gazebo, headless). v2 plans while the arm picks; '
             'single-SKU pallets are complete layer patterns, verified by the pallet camera.',
             fontsize=12, color=common.TEXT2)
    common.save_fig(fig, common.out('results.png'), common.FULL_W)


if __name__ == '__main__':
    main()
