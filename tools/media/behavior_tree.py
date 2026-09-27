#!/usr/bin/env python3
"""docs/media/behavior_tree.svg: the orchestrator's BehaviorTree (trees/palletize.xml) as a
graphviz diagram, collapsed to the main structure: decorators are folded into their child,
recovery branches of a Fallback (every child after the first) become one dashed node that
lists their steps, SubTrees are drawn once as their own cluster. Needs graphviz (`dot`).
"""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402

XML = os.path.join(common.ROOT, 'src', 'palletizer_orchestrator', 'trees', 'palletize.xml')

DECORATORS = {'Inverter': 'not', 'ForceSuccess': 'force success', 'Delay': 'delay',
              'KeepRunningUntilFailure': 'loop until failure',
              'RetryUntilSuccessful': 'retry'}
CONDITIONS = {'Holding', 'MorePallets', 'IsPalletStarted', 'PalletFull'}
SKIP = {'AlwaysSuccess', 'AlwaysFailure'}
CONTROL = {'Sequence': '→  Sequence', 'Fallback': '?  Fallback', 'Parallel': '⇉  Parallel'}
TREES = ['NewPallet', 'ClosePallet', 'Cycle', 'Palletize']  # created bottom-up (LR)

C_ACTION, C_COND, C_CTRL, C_SUB, C_REC = common.BLUE, common.AQUA, '#4a4a46', '#6b5bd6', '#2d2d2b'


def deco_label(el):
    d = DECORATORS[el.tag]
    if el.tag == 'RetryUntilSuccessful':
        d += f' ×{el.get("num_attempts")}'
    return d


def steps(el):
    """Flat leaf names of a recovery branch."""
    if el.tag in SKIP:
        return []
    if el.tag == 'SubTree':
        return [f'⤷ {el.get("ID")}']
    kids = list(el)
    if not kids:
        return [el.tag]
    if el.tag in DECORATORS:
        return [f'{s} ({deco_label(el)})' if i == 0 else s
                for i, s in enumerate(steps(kids[0]))]
    return [s for k in kids for s in steps(k)]


class Graph:
    def __init__(self):
        self.lines, self.n = [], 0

    def node(self, label, **attrs):
        self.n += 1
        name = f'n{self.n}'
        a = ', '.join(f'{k}="{v}"' for k, v in attrs.items())
        label = label.replace('"', '\\"')
        self.lines.append(f'  {name} [label="{label}", {a}];')
        return name

    def edge(self, a, b, **attrs):
        a_ = ', '.join(f'{k}="{v}"' for k, v in attrs.items())
        self.lines.append(f'  {a} -> {b} [{a_}];')

    def build(self, el, prefix=''):
        """Node for element `el` (decorator labels accumulated in `prefix`)."""
        if el.tag in DECORATORS:
            p = f'{prefix}{deco_label(el)} · '
            return self.build(list(el)[0], p)
        if el.tag == 'SubTree':
            return self.node(f'{prefix}⤷ {el.get("ID")}', shape='box', style='rounded,filled',
                             fillcolor=C_SUB, color=C_SUB)
        if el.tag in CONTROL:
            name = el.get('name')
            label = CONTROL[el.tag] + (f'\\n{name}' if name else '')
            me = self.node(prefix + label, shape='box', style='filled', fillcolor=C_CTRL,
                           color=C_CTRL, fontname='Helvetica-Bold')
            # rankdir=LR draws the first-created child at the bottom: create them reversed
            for i, k in reversed(list(enumerate(el))):
                if k.tag in SKIP:
                    continue
                if el.tag == 'Fallback' and i > 0 and len(list(k)) > 0 and k.tag != 'SubTree':
                    label = (k.get('name') or 'recovery') + ':\\l' + '\\l'.join(
                        '  ' + s for s in steps(k)) + '\\l'
                    c = self.node(label, shape='box', style='dashed,rounded,filled',
                                  fillcolor=C_REC, color=common.MUTED, fontcolor=common.TEXT2,
                                  fontsize='11')
                    self.edge(me, c, style='dashed', color=common.MUTED)
                else:
                    self.edge(me, self.build(k))
            return me
        # leaf
        if el.tag in CONDITIONS:
            return self.node(prefix + el.tag + '?', shape='box', style='rounded,filled',
                             fillcolor=C_COND, color=C_COND)
        return self.node(prefix + el.tag, shape='box', style='rounded,filled',
                         fillcolor=C_ACTION, color=C_ACTION)


def main():
    root = ET.parse(XML).getroot()
    trees = {t.get('ID'): t for t in root.findall('BehaviorTree')}
    g = Graph()
    head = [
        'digraph BT {',
        f'  graph [bgcolor="{common.BG}", rankdir=LR, nodesep=0.12, ranksep=0.35, pad=0.3,'
        f' fontname="Helvetica", fontcolor="{common.TEXT}", fontsize=16, compound=true, ordering=out,'
        ' newrank=true];',
        f'  node [fontname="Helvetica", fontsize=12, fontcolor="{common.TEXT}", margin="0.12,0.06",'
        ' height=0.3, penwidth=1.2];',
        f'  edge [color="{common.MUTED}", arrowhead=none, penwidth=1.2];',
    ]
    for tid in TREES:
        g.lines.append(f'  subgraph cluster_{tid} {{')
        g.lines.append(f'    label=<<b>{tid}</b>>; labeljust=l; style="rounded";'
                       f' color="{common.GRID}"; penwidth=1.5; margin=14;')
        top = list(trees[tid])[0]
        g.build(top)
        g.lines.append('  }')
    legend = [
        '  subgraph cluster_legend {',
        '    label=""; style="invis";',
        f'    l1 [label="action", shape=box, style="rounded,filled", fillcolor="{C_ACTION}", color="{C_ACTION}"];',
        f'    l2 [label="condition?", shape=box, style="rounded,filled", fillcolor="{C_COND}", color="{C_COND}"];',
        f'    l3 [label="→ / ? / ⇉  control", shape=box, style=filled, fillcolor="{C_CTRL}", color="{C_CTRL}"];',
        f'    l4 [label="⤷ SubTree", shape=box, style="rounded,filled", fillcolor="{C_SUB}", color="{C_SUB}"];',
        f'    l5 [label="recovery branch", shape=box, style="dashed,rounded,filled", fillcolor="{C_REC}",'
        f' color="{common.MUTED}", fontcolor="{common.TEXT2}"];',
        '    l5 -> l4 -> l3 -> l2 -> l1 [style=invis];',
        '    {rank=same; l1; l2; l3; l4; l5;}',
        '  }',
    ]
    dot = '\n'.join(head + g.lines + legend + ['}'])
    os.makedirs(common.work(), exist_ok=True)
    with open(common.work('behavior_tree.dot'), 'w') as f:
        f.write(dot)
    path = common.out('behavior_tree.svg')
    subprocess.run(['dot', '-Tsvg', '-o', path, common.work('behavior_tree.dot')], check=True)
    subprocess.run(['dot', '-Tpng', '-Gdpi=80', '-o', common.work('behavior_tree_preview.png'),
                    common.work('behavior_tree.dot')], check=True)
    print(f'saved {os.path.relpath(path, common.ROOT)} ({os.path.getsize(path) / 1024:.0f} kB)')


if __name__ == '__main__':
    main()
