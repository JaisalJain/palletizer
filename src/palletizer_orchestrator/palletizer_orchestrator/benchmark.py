"""Benchmark: N pallets per planner, headless, on a fixed seed list.

Each (planner, seed) runs one pallet in a fresh simulation:
    ros2 launch palletizer_bringup palletizer.launch.py mode:=auto gui:=false
         planner:=<p> seed:=<s> max_pallets:=1 rtf:=<rtf> run_name:=bench_<p>_seed<s>
The same seed gives both planners the identical box sequence. Per-box / per-pallet CSVs go
to results/, and a markdown summary table is written into docs/RESULTS.md between the
BENCHMARK markers.

Usage: ros2 run palletizer_orchestrator benchmark [--planners naive heuristic]
       [--seeds 1 2 3 4 5] [--rtf 1.0] [--timeout 1800] [--summary-only]
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

WS = os.path.expanduser('~/palletizer_ws')
RESULTS = os.path.join(WS, 'results')
DOC = os.path.join(WS, 'docs', 'RESULTS.md')
BEGIN = '<!-- BENCHMARK:BEGIN -->'
END = '<!-- BENCHMARK:END -->'


def run_one(planner, seed, rtf, timeout, log_dir):
    run = f'bench_{planner}_seed{seed}'
    for f in glob.glob(os.path.join(RESULTS, f'{run}_*.csv')):
        os.remove(f)
    log = open(os.path.join(log_dir, f'{run}.log'), 'w')
    cmd = ['ros2', 'launch', 'palletizer_bringup', 'palletizer.launch.py', 'mode:=auto',
           'gui:=false', 'rviz:=false', f'planner:={planner}', f'seed:={seed}',
           'max_pallets:=1', f'rtf:={rtf}', f'run_name:={run}', f'results_dir:={RESULTS}']
    t0 = time.time()
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        proc.wait(timeout=timeout)
        status = 'ok'
    except subprocess.TimeoutExpired:
        status = 'timeout'
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        log.close()
    pallets = os.path.join(RESULTS, f'{run}_pallets.csv')
    rows = list(csv.DictReader(open(pallets))) if os.path.exists(pallets) else []
    print(f'{run}: {status} in {time.time() - t0:.0f} s, '
          f'{"pallet recorded" if rows else "NO pallet recorded"}', flush=True)
    return status


def load(planners):
    out = {}
    for p in planners:
        pallets, boxes = [], []
        for f in sorted(glob.glob(os.path.join(RESULTS, f'bench_{p}_seed*_pallets.csv'))):
            rows = [r for r in csv.DictReader(open(f)) if r['pallet'] == '0']
            pallets += rows
        for f in sorted(glob.glob(os.path.join(RESULTS, f'bench_{p}_seed*_boxes.csv'))):
            boxes += [r for r in csv.DictReader(open(f)) if r['pallet'] == '0']
        out[p] = (pallets, boxes)
    return out


def ms(values, fmt='{:.1f}'):
    v = np.array([float(x) for x in values if x not in ('', None)])
    if v.size == 0:
        return 'n/a'
    return f'{fmt.format(v.mean())} ± {fmt.format(v.std())}'


def summary(planners, rtf):
    data = load(planners)
    lines = [
        BEGIN,
        '',
        f'Real-time factor: {rtf}. One pallet per (planner, seed) in a fresh headless '
        'simulation; the same seed gives both planners the same box sequence. '
        'Mean ± std over pallets. Raw data: `results/bench_*`.',
        '',
        '| Planner | Pallets | Seeds | Fill % | Boxes / pallet | Max height [m] | Collapses | '
        'Rejected | Picks / min | Planning time [ms] | Cycle time [s] | Place err [mm] | '
        'Mass err [kg] |',
        '|---|---|---|---|---|---|---|---|---|---|---|---|---|',
    ]
    for p in planners:
        pallets, boxes = data[p]
        placed = [b for b in boxes if b['success'] == '1']
        seeds = ' '.join(sorted({r['seed'] for r in pallets}, key=int))
        lines.append(
            f'| {p} | {len(pallets)} | {seeds} | {ms([r["fill_percent"] for r in pallets])} | '
            f'{ms([r["boxes"] for r in pallets])} | '
            f'{ms([r["max_height_m"] for r in pallets], "{:.2f}")} | '
            f'{sum(int(r["collapses"]) for r in pallets)} | '
            f'{sum(int(r["rejected"]) for r in pallets)} | '
            f'{ms([r["picks_per_min"] for r in pallets], "{:.2f}")} | '
            f'{ms([1000 * float(b["planning_time_s"]) for b in placed], "{:.0f}")} | '
            f'{ms([b["cycle_time_s"] for b in placed])} | '
            f'{ms([b["place_err_mm"] for b in placed], "{:.2f}")} | '
            f'{ms([b["mass_err_kg"] for b in placed], "{:.3f}")} |')
    # paired comparison
    if len(planners) == 2:
        a, b = planners
        fa = {r['seed']: float(r['fill_percent']) for r in data[a][0]}
        fb = {r['seed']: float(r['fill_percent']) for r in data[b][0]}
        common = sorted(set(fa) & set(fb), key=int)
        if common:
            d = np.array([fb[s] - fa[s] for s in common])
            lines += ['', f'Paired per seed ({b} − {a}, fill %): mean {d.mean():+.2f}, '
                      f'{b} better on {(d > 0).sum()}, equal on {(d == 0).sum()}, worse on '
                      f'{(d < 0).sum()} of {len(common)} seeds.']
            lines += ['', '| Seed | ' + ' | '.join(f'{p} fill % (boxes)' for p in planners) + ' |',
                      '|---|' + '---|' * len(planners)]
            boxes_n = {p: {r['seed']: r['boxes'] for r in data[p][0]} for p in planners}
            for s in common:
                lines.append(f'| {s} | {fa[s]:.1f} ({boxes_n[a][s]}) | {fb[s]:.1f} ({boxes_n[b][s]}) |')
    lines += ['', END]
    block = '\n'.join(lines)
    doc = open(DOC).read() if os.path.exists(DOC) else '# Results\n'
    if BEGIN in doc and END in doc:
        doc = doc[:doc.index(BEGIN)] + block + doc[doc.index(END) + len(END):]
    else:
        doc += '\n## Milestone E — Benchmark\n\n' + block + '\n'
    with open(DOC, 'w') as f:
        f.write(doc)
    print(block)


def summarize_run(run, marker):
    """Markdown summary of one auto run (all pallets) into docs/RESULTS.md between
    <!-- MARKER:BEGIN --> / <!-- MARKER:END -->."""
    pallets = list(csv.DictReader(open(os.path.join(RESULTS, f'{run}_pallets.csv'))))
    boxes = list(csv.DictReader(open(os.path.join(RESULTS, f'{run}_boxes.csv'))))
    begin, end = f'<!-- {marker}:BEGIN -->', f'<!-- {marker}:END -->'
    outcomes = {}
    for b in boxes:
        outcomes[b['outcome']] = outcomes.get(b['outcome'], 0) + 1
    placed = [b for b in boxes if b['success'] == '1']
    lines = [begin, '',
             f'Run `{run}` (raw data: `results/{run}_*.csv`). Box outcomes: ' +
             ', '.join(f'{k} {v}' for k, v in sorted(outcomes.items())) + '.', '',
             '| Pallet | Boxes | Rejected | Fill % | Max height [m] | Collapses | Duration [s] | '
             'Picks/min | Mean planning [ms] | Mean place err [mm] | End |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    for r in pallets:
        lines.append(
            f'| {r["pallet"]} | {r["boxes"]} | {r["rejected"]} | {float(r["fill_percent"]):.1f} | '
            f'{float(r["max_height_m"]):.2f} | {r["collapses"]} | {float(r["duration_s"]):.0f} | '
            f'{float(r["picks_per_min"]):.2f} | '
            f'{1000 * float(r["mean_planning_time_s"] or 0):.0f} | '
            f'{float(r["mean_place_err_mm"] or 0):.2f} | {r["end_reason"]} |')
    me = [abs(float(b['measured_mass']) - float(b['true_mass'])) for b in placed if b['true_mass']]
    de = [float(b['dim_err_max_mm']) for b in placed if b['dim_err_max_mm']]
    lines += ['', f'Placed boxes: {len(placed)}; max dimension error {max(de):.2f} mm; '
              f'F/T mass error mean {1000 * np.mean(me):.0f} g / max {1000 * max(me):.0f} g.', '',
              end]
    block = '\n'.join(lines)
    doc = open(DOC).read()
    doc = doc[:doc.index(begin)] + block + doc[doc.index(end) + len(end):]
    with open(DOC, 'w') as f:
        f.write(doc)
    print(block)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--planners', nargs='+', default=['naive', 'heuristic'])
    ap.add_argument('--seeds', nargs='+', type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument('--rtf', type=float, default=1.0)
    ap.add_argument('--timeout', type=float, default=1800.0)
    ap.add_argument('--summary-only', action='store_true')
    ap.add_argument('--run-summary', nargs=2, metavar=('RUN', 'MARKER'),
                    help='write the summary of one auto run into docs/RESULTS.md and exit')
    args = ap.parse_args(argv)
    if args.run_summary:
        summarize_run(*args.run_summary)
        return 0
    os.makedirs(RESULTS, exist_ok=True)
    log_dir = os.path.join(RESULTS, 'logs')
    os.makedirs(log_dir, exist_ok=True)
    if not args.summary_only:
        for seed in args.seeds:
            for p in args.planners:
                run_one(p, seed, args.rtf, args.timeout, log_dir)
    summary(args.planners, args.rtf)
    return 0


if __name__ == '__main__':
    sys.exit(main())
