"""Single-SKU acceptance runs: one full pallet per SKU, headless, unattended.

Each run is a fresh simulation:
    ros2 launch palletizer_bringup palletizer.launch.py mode:=single_sku sku:=<key> gui:=false
         max_pallets:=1 finish_with_next_box:=true seed:=<s> rtf:=<rtf> run_name:=sku_<key>
It ends when the first pallet is FULL, the exchange has brought the next pallet and its first
box is placed. Extra runs: `--order` (a SKU change mid-stream, closed with SKU_CHANGE) and any
`--sku` / `--pallets` the robot was never configured for (no-hardcoding check).
The summary table goes into docs/RESULTS.md between SKU_RUNS markers.

Usage: ros2 run palletizer_orchestrator sku_runs [--skus 400x300x300 ...] [--rtf 1.0]
       ros2 run palletizer_orchestrator sku_runs --name skuchange --order "a:12, b:6"
       ros2 run palletizer_orchestrator sku_runs --name unknown --sku 347x253x219 \
            --pallets 1140x760x150
       ros2 run palletizer_orchestrator sku_runs --summary-only
       ros2 run palletizer_orchestrator sku_runs --skus 600x400x300 --snapshot docs/media/_work
            (also saves images: mid-run, the finished pallet before the exchange, the
             exchange; see sim_snapshot.py)
"""

import argparse
import csv
import glob
import os
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

from palletizer_orchestrator.benchmark import DOC, RESULTS, WS, cleanup

BEGIN = '<!-- SKU_RUNS:BEGIN -->'
END = '<!-- SKU_RUNS:END -->'
CONFIG = os.path.join(WS, 'src', 'palletizer_bringup', 'config', 'cell_config.yaml')


def run_one(name, launch_args, timeout, log_dir, attempt=1, snapshot=None):
    run = f'sku_{name}'
    # media runs (--snapshot) keep their CSVs next to the images, not in results/
    results = os.path.join(snapshot[0], 'results') if snapshot else RESULTS
    os.makedirs(results, exist_ok=True)
    for f in glob.glob(os.path.join(results, f'{run}_*.csv')):
        os.remove(f)
    suffix = '' if attempt == 1 else f'_attempt{attempt}'
    log = open(os.path.join(log_dir, f'{run}{suffix}.log'), 'w')
    cmd = ['ros2', 'launch', 'palletizer_bringup', 'palletizer.launch.py', 'mode:=single_sku',
           'gui:=false', 'rviz:=false', 'max_pallets:=1', 'finish_with_next_box:=true',
           f'run_name:={run}', f'results_dir:={results}'] + launch_args
    cleanup()
    t0 = time.time()
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    snap = None
    if snapshot:
        out, hero_after = snapshot
        snap = subprocess.Popen(
            ['ros2', 'run', 'palletizer_orchestrator', 'sim_snapshot',
             '--out', os.path.join(out, run), '--hero-after', str(hero_after)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        proc.wait(timeout=timeout)
        status = 'ok'
    except subprocess.TimeoutExpired:
        status = 'timeout'
    finally:
        if snap is not None and snap.poll() is None:
            os.killpg(snap.pid, signal.SIGINT)
            try:
                snap.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(snap.pid, signal.SIGKILL)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        log.close()
        cleanup()
    pallets = os.path.join(results, f'{run}_pallets.csv')
    rows = list(csv.DictReader(open(pallets))) if os.path.exists(pallets) else []
    print(f'{run}: {status} in {time.time() - t0:.0f} s, '
          f'{"pallet recorded" if rows else "NO pallet recorded"}', flush=True)
    if not rows and attempt < 2:
        print(f'{run}: retrying (attempt {attempt + 1})', flush=True)
        return run_one(name, launch_args, timeout, log_dir, attempt + 1, snapshot)
    return status


def load(run):
    p = os.path.join(RESULTS, f'{run}_pallets.csv')
    b = os.path.join(RESULTS, f'{run}_boxes.csv')
    c = os.path.join(RESULTS, f'{run}_placement_checks.csv')
    rd = lambda f: list(csv.DictReader(open(f))) if os.path.exists(f) else []  # noqa: E731
    return rd(p), rd(b), rd(c)


def f(x, fmt='{:.1f}'):
    return fmt.format(float(x)) if x not in ('', None) else 'n/a'


def summary(names):
    catalog = yaml.safe_load(open(CONFIG))['spawner']['catalog']
    lines = [BEGIN, '',
             'One pallet per SKU on EUR (fresh headless simulation each), then the exchange and '
             'the first box of the next pallet. Everything in the table is what the robot '
             'measured / decided, except the last columns (ground truth, evaluation). '
             'Raw data: `results/sku_*`.', '',
             '| SKU | Carton (measured) [mm] | Pallet (measured) [mm] | Optimum / layer | '
             'Used / layer (A / B) | Layers | Pattern | Boxes placed | Volume fill % | '
             'Picks / min (pallet · steady) | End | Next pallet | Collapses (GT) | '
             'Stack events (detected) | Cam vs GT placement max [mm] | RTF |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for name in names:
        pallets, boxes, checks = load(f'sku_{name}')
        if not pallets:
            lines.append(f'| {name} | no result | | | | | | | | | | | | | | |')
            continue
        r = pallets[0]
        b0 = [b for b in boxes if b['pallet'] == '0' and b['success'] == '1']
        dims = np.array([[float(b['det_L']), float(b['det_W']), float(b['det_H'])] for b in b0])
        started = any(b['pallet'] == '1' and b['success'] == '1' for b in boxes)
        pat = f'{r["boxes_in_pattern"]}'
        opt = ''
        # optimum per layer (deck, no reachability) and per-layer counts from the log
        # the log of the attempt that produced the result (the last one, after a retry)
        logs = [os.path.join(RESULTS, 'logs', f'sku_{name}{s}.log')
                for s in ('', '_attempt2', '_attempt3')]
        logs = [p for p in logs if os.path.exists(p)]
        log = logs[-1] if logs else ''
        per_layer = ''
        if log:
            for line in open(log):
                if 'pattern for' in line and '(optimum' in line:
                    seg = line.split('mm (usable')[1]
                    per_layer = seg.split(': ')[1].split(' per layer')[0]
                    opt = seg.split('(optimum ')[1].split(')')[0]
                    break
        key = name if name in catalog else ''
        carton = f'{dims.mean(0)[0] * 1000:.0f} x {dims.mean(0)[1] * 1000:.0f} x ' \
                 f'{dims.mean(0)[2] * 1000:.0f}' if len(dims) else 'n/a'
        if key:
            carton += f' ({key})'
        lines.append(
            f'| {name} | {carton} | {float(r["pallet_u"]) * 1000:.0f} x '
            f'{float(r["pallet_v"]) * 1000:.0f} | {opt} | {per_layer} | {r["layers"]} | {pat} | '
            f'{r["boxes"]} | {f(r["fill_percent"])} | '
            f'{f(r["picks_per_min"], "{:.2f}")} · {f(r["picks_per_min_steady"], "{:.2f}")} | '
            f'{r["end_reason"]} | {"started" if started else "NO"} | {r["collapses"]} | '
            f'{r["detected_events"]} | {f(r["max_cam_vs_gt_mm"], "{:.2f}")} | '
            f'{f(r["rtf"], "{:.2f}")} |')
    lines += ['', END]
    block = '\n'.join(lines)
    doc = open(DOC).read()
    if BEGIN in doc and END in doc:
        doc = doc[:doc.index(BEGIN)] + block + doc[doc.index(END) + len(END):]
    else:
        doc += '\n## v2 — Single-SKU pallets\n\n' + block + '\n'
    with open(DOC, 'w') as fh:
        fh.write(doc)
    print(block)


def all_names(catalog):
    """Catalog SKUs first, then any other sku_* run in results/ (order / custom runs)."""
    found = sorted(os.path.basename(f)[len('sku_'):-len('_pallets.csv')]
                   for f in glob.glob(os.path.join(RESULTS, 'sku_*_pallets.csv')))
    return list(catalog) + [n for n in found if n not in catalog]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--skus', nargs='+', default=None, help='default: the whole catalog')
    ap.add_argument('--name', default='', help='run name for --order / a single custom run')
    ap.add_argument('--sku', default='')
    ap.add_argument('--order', default='')
    ap.add_argument('--pallets', default='')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--rtf', type=float, default=1.0)
    ap.add_argument('--timeout', type=float, default=7200.0)
    ap.add_argument('--extra', nargs='*', default=[], help='more launch args (key:=value)')
    ap.add_argument('--summary-only', action='store_true')
    ap.add_argument('--snapshot', default='', metavar='DIR',
                    help='save media snapshots of every run into DIR/sku_<name>/')
    ap.add_argument('--hero-after', type=int, default=6,
                    help='--snapshot: placed boxes before the mid-run shot')
    args = ap.parse_args(argv)
    catalog = yaml.safe_load(open(CONFIG))['spawner']['catalog']
    log_dir = os.path.join(args.snapshot or RESULTS, 'logs')
    os.makedirs(log_dir, exist_ok=True)
    common = [f'seed:={args.seed}', f'rtf:={args.rtf}'] + args.extra
    snapshot = (os.path.abspath(args.snapshot), args.hero_after) if args.snapshot else None
    if args.pallets:
        common.append(f'pallets:={args.pallets}')
    if args.name:
        la = common + ([f'sku:={args.sku}'] if args.sku else []) + \
            ([f'order:={args.order}'] if args.order else [])
        if not args.summary_only:
            run_one(args.name, la, args.timeout, log_dir, snapshot=snapshot)
        if not snapshot:
            summary(all_names(catalog))
        return 0
    skus = args.skus or list(catalog)
    if not args.summary_only:
        for k in skus:
            run_one(k, common + [f'sku:={k}'], args.timeout, log_dir, snapshot=snapshot)
    if not snapshot:
        summary(all_names(catalog))
    return 0


if __name__ == '__main__':
    sys.exit(main())
