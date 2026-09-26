"""Single-SKU pattern planner (no ROS). Run: python -m pytest src/palletizer_packing/test"""

import itertools
import os

import numpy as np
import pytest
import yaml

from palletizer_packing.core.pallet import Box, PalletSpec, PalletState
from palletizer_packing.core.pattern import (PatternConfig, PatternPlanner, far_first,
                                             interlock, make_pattern, search_layouts)
from palletizer_packing.core.reach import ReachabilityMap
from palletizer_packing.core.rules import RuleConfig, stability

HERE = os.path.dirname(os.path.abspath(__file__))
CELL_CONFIG = os.path.join(HERE, '..', '..', 'palletizer_bringup', 'config', 'cell_config.yaml')
EUR = PalletSpec(1.2, 0.8, 0.01, 1.05)


@pytest.fixture(scope='module')
def catalog():
    with open(CELL_CONFIG) as f:
        return yaml.safe_load(f)['spawner']['catalog']


def sku(catalog, key):
    L, W, H = catalog[key]['size']
    return Box(L, W, H, 0.0)


def no_overlaps(rects, gap=0.0):
    for a, b in itertools.combinations(rects, 2):
        du = min(a.u0 + a.du, b.u0 + b.du) - max(a.u0, b.u0)
        dv = min(a.v0 + a.dv, b.v0 + b.dv) - max(a.v0, b.v0)
        if du > -gap + 1e-6 and dv > -gap + 1e-6:
            return False
    return True


def in_deck(rects, U, V):
    return all(r.u0 >= -1e-9 and r.v0 >= -1e-9 and r.u0 + r.du <= U + 1e-9 and
               r.v0 + r.dv <= V + 1e-9 for r in rects)


# ------------------------------------------------------------------------- known optima
@pytest.mark.parametrize('U, V, L, W, n', [
    (1.2, 0.8, 0.6, 0.4, 4),       # ISO modules on EUR, exact fit
    (1.2, 0.8, 0.4, 0.3, 8),
    (1.2, 0.8, 0.3, 0.2, 16),
    (1.2, 1.0, 0.6, 0.4, 5),       # 600 x 400 on 1200 x 1000 (pinwheel-free: 2 + 3)
    (1.0, 0.8, 0.4, 0.3, 6),
])
def test_known_counts_exact_modules(U, V, L, W, n):
    """Exact module sizes, no gap: the classic counts."""
    best, layouts = search_layouts(U, V, L, W, 0.0)
    assert best == n
    for lay in layouts:
        assert in_deck(lay.rects, U, V) and no_overlaps(lay.rects)


@pytest.mark.parametrize('key, n', [('600x400x400', 4), ('600x400x300', 4), ('400x300x300', 8),
                                    ('400x300x200', 8), ('300x200x200', 16)])
def test_known_counts_catalog_cartons(catalog, key, n):
    """Catalog cartons (slightly under module) with the 5 mm placement gap: same counts, and
    every layout keeps the gap between boxes and stays on the deck (no overhang)."""
    b = sku(catalog, key)
    best, layouts = search_layouts(1.2, 0.8, b.length, b.width, 0.005)
    assert best == n
    for lay in layouts:
        assert in_deck(lay.rects, 1.2, 0.8)
        assert no_overlaps(lay.rects, gap=0.005 - 1e-4)


def test_exact_modules_do_not_fit_with_a_gap():
    """Why the catalog cartons are under module size: 2 x 600 + 5 mm > 1200."""
    best, _ = search_layouts(1.2, 0.8, 0.6, 0.4, 0.005)
    assert best < 4


def test_non_guillotine_pattern_found():
    """A problem whose optimum needs the 5-block (pinwheel) structure: 8 boxes of 0.3 x 0.1
    on 0.5 x 0.5 (the best guillotine pattern has 7)."""
    best, layouts = search_layouts(0.5, 0.5, 0.3, 0.1, 0.0)
    assert best == 8
    assert any(lay.kind.startswith('5-block') and lay.count == 8 for lay in layouts)
    assert all(no_overlaps(lay.rects) and in_deck(lay.rects, 0.5, 0.5) for lay in layouts)


# ------------------------------------------------------------------------- patterns
@pytest.mark.parametrize('key', ['600x400x400', '600x400x300', '400x300x300', '400x300x200',
                                 '300x200x200', '500x300x250', '450x350x300', '520x320x250',
                                 '380x280x220'])
def test_pattern_valid_for_every_sku(catalog, key):
    """Whole pallet: no 3D overlaps (gap kept), inside the deck, under the height limit,
    every box statically stable in placement order, far side first within a layer."""
    b = sku(catalog, key)
    p = make_pattern(b, EUR, RuleConfig(), PatternConfig())
    assert p.count == sum(len(p.layer_a.rects if k % 2 else p.layer_b.rects)
                          for k in range(1, p.layers + 1))
    assert p.count > 0 and p.layers == int(1.05 // b.height + 1e-9)
    assert p.layer_a.count == p.optimum          # count priority keeps the optimum
    state = PalletState(EUR)
    for slot in p.slots:
        r = slot.rect
        pl_ = state.rest_height(r.u0, r.v0, r.du, r.dv)
        assert pl_ == pytest.approx(slot.z0, abs=1e-9)
        from palletizer_packing.core.pallet import Placement
        placement = Placement(r.u0, r.v0, r.du, r.dv, slot.z0, b.height, r.rotated)
        assert stability(state, placement, b, RuleConfig()).ok
        assert placement.top <= EUR.max_height + 1e-9
        state.place(placement, b)
    for k in range(1, p.layers + 1):
        rs = [s.rect for s in p.slots if s.layer == k]
        assert in_deck(rs, 1.2, 0.8) and no_overlaps(rs, gap=0.005 - 1e-4)
        far = [r.v0 + r.dv for r in rs]
        assert all(far[i] >= far[i + 1] - 1e-6 for i in range(len(far) - 1)), 'far side first'


def test_interlock_when_possible(catalog):
    """Non-modular 520 x 320: layer B differs from A and interlocks (boxes bridge two
    boxes below) with the same count; the whole pallet passes PyBullet."""
    pytest.importorskip('pybullet')
    from palletizer_packing.core.physics import PhysicsChecker, PhysicsConfig
    ph = PhysicsChecker(PhysicsConfig())
    b = sku(catalog, '520x320x250')
    p = make_pattern(b, EUR, RuleConfig(), PatternConfig(), physics=ph)
    ph.close()
    assert p.layer_a.key() != p.layer_b.key()
    assert p.layer_a.count == p.layer_b.count == p.optimum
    assert p.interlock >= 0.3 and interlock(p.layer_b, p.layer_a) >= 0.3
    assert 'interlocked' in p.note


def test_column_stack_when_the_optimum_is_unique(catalog):
    """600 x 400 on EUR: the 2 x 2 grid is the only 4-box layer -> column stack (count
    priority) ..."""
    b = sku(catalog, '600x400x400')
    p = make_pattern(b, EUR, RuleConfig(), PatternConfig(priority='count'))
    assert p.count == 8 and p.interlock == 0.0 and 'column stack' in p.note


def test_interlock_priority_trades_boxes_for_interlocking(catalog):
    """... while priority 'interlock' drops a box per B layer to interlock."""
    b = sku(catalog, '400x300x200')
    pc = make_pattern(b, EUR, RuleConfig(), PatternConfig(priority='count'))
    pi = make_pattern(b, EUR, RuleConfig(), PatternConfig(priority='interlock'))
    assert pc.interlock == 0.0 and pc.count == 40
    assert pi.interlock >= 0.3 and pi.count < pc.count
    assert pi.layer_a.count == 8 and pi.layer_b.count == 7


def test_unreachable_slots_are_dropped(catalog):
    """Slots whose centre is unreachable at their layer height are dropped and the count is
    reported (the pedestal keep-out costs small SKUs boxes)."""
    b = sku(catalog, '300x200x200')
    u = np.round(np.arange(0.0, 1.2001, 0.05), 3)
    v = np.round(np.arange(0.0, 0.8001, 0.05), 3)
    h = np.round(np.arange(0.0, 1.2001, 0.05), 3)
    reach = np.ones((len(h), len(v), len(u)), bool)
    reach[:, :4, 10:16] = False                  # a keep-out: v < 0.2, u in [0.5, 0.75]
    m = ReachabilityMap(u, v, h, reach)
    p = make_pattern(b, EUR, RuleConfig(), PatternConfig(), reach=m)
    free = make_pattern(b, EUR, RuleConfig(), PatternConfig())
    assert 0 < p.count < free.count
    for s in p.slots:
        assert m.reachable(*s.rect.center, s.layer * b.height)


def test_far_first_order():
    from palletizer_packing.core.pattern import Rect
    rs = [Rect(0.0, 0.0, 0.3, 0.2, False), Rect(0.3, 0.6, 0.3, 0.2, False),
          Rect(0.0, 0.6, 0.3, 0.2, False), Rect(0.3, 0.0, 0.3, 0.2, False)]
    o = far_first(rs)
    assert [(r.u0, r.v0) for r in o] == [(0.0, 0.6), (0.3, 0.6), (0.0, 0.0), (0.3, 0.0)]


# ------------------------------------------------------------------------- runtime planner
def test_pattern_planner_fills_and_completes(catalog):
    b = sku(catalog, '600x400x300')
    pl = PatternPlanner(RuleConfig(), PatternConfig(), 0.010)
    s = PalletState(EUR)
    n = 0
    while True:
        p = pl.plan(s, Box(b.length + 0.001, b.width - 0.002, b.height + 0.001, 9.0))
        if p is None:
            break
        s.place(p, b)
        n += 1
    assert n == pl.pattern.count == 12
    assert pl.last_reason == 'PATTERN_COMPLETE'


def test_sku_recognition(catalog):
    """Within +/-10 mm = same SKU; any dimension further off -> SKU_CHANGE."""
    b = sku(catalog, '400x300x300')
    pl = PatternPlanner(RuleConfig(), PatternConfig(), 0.010)
    s = PalletState(EUR)
    s.place(pl.plan(s, b), b)
    assert pl.plan(s, Box(b.length + 0.009, b.width - 0.009, b.height + 0.009)) is not None
    for other in (Box(b.length + 0.012, b.width, b.height), Box(b.length, b.width, b.height + 0.1),
                  sku(catalog, '400x300x200'), sku(catalog, '380x280x220')):
        assert pl.plan(s, other) is None and pl.last_reason == 'SKU_CHANGE'
    pl.reset()                                   # next pallet: the new box defines the SKU
    s2 = PalletState(EUR)
    assert pl.plan(s2, sku(catalog, '400x300x200')) is not None


def test_pattern_planner_checks_the_perceived_layer(catalog):
    """If the layer below is not where the pattern expects it (perception disagrees with
    the record), the next slot is refused (NO_FEASIBLE_PLACEMENT), not placed in the air."""
    b = sku(catalog, '600x400x300')
    pl = PatternPlanner(RuleConfig(), PatternConfig(), 0.010)
    s = PalletState(EUR)
    for _ in range(4):
        s.place(pl.plan(s, b), b)
    perceived = s.height.copy()
    first = pl.pattern.slots[4].rect
    sv, su = s.cells(first.u0, first.v0, first.du, first.dv)
    perceived[sv, su] = 0.0                      # the box under slot 4 is gone
    assert pl.plan(s.with_height(perceived), b) is None
    assert pl.last_reason == 'NO_FEASIBLE_PLACEMENT'


def test_motion_validator_applied(catalog):
    b = sku(catalog, '600x400x300')
    pl = PatternPlanner(RuleConfig(), PatternConfig(), 0.010)
    seen = []
    pl.validator = lambda p: seen.append(p) or False
    assert pl.plan(PalletState(EUR), b) is None and len(seen) == 1
    assert pl.last_reason == 'NO_FEASIBLE_PLACEMENT'
