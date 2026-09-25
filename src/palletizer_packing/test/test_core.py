"""Unit tests for the packing core (no ROS). Run: python -m pytest src/palletizer_packing/test"""

import itertools
import os
import sys

import numpy as np
import pytest
import yaml

from palletizer_packing.core.pallet import Box, PalletSpec, PalletState, Placement
from palletizer_packing.core.planners import (HeuristicPlanner, NaivePlanner, PlannerConfig,
                                              candidates, make_placement)
from palletizer_packing.core.reach import ReachabilityMap
from palletizer_packing.core.rules import RuleConfig, feasible, no_reach_over, stability
from palletizer_packing.core.sequence import box_sequence

HERE = os.path.dirname(os.path.abspath(__file__))
CELL_CONFIG = os.path.join(HERE, '..', '..', 'palletizer_bringup', 'config', 'cell_config.yaml')


@pytest.fixture(scope='module')
def cell():
    with open(CELL_CONFIG) as f:
        return yaml.safe_load(f)


@pytest.fixture
def spec():
    return PalletSpec(1.2, 0.8, 0.01, 1.05)


@pytest.fixture
def rules():
    return RuleConfig()


def put(state, u0, v0, du, dv, h, mass=5.0):
    p = make_placement(state, Box(max(du, dv), min(du, dv), h, mass), u0, v0, du, dv, dv > du)
    state.place(p, Box(max(du, dv), min(du, dv), h, mass))
    return p


def overlaps_3d(a, b):
    pa, pb_ = a.placement, b.placement
    eps = 1e-6
    return (pa.u0 < pb_.u0 + pb_.du - eps and pb_.u0 < pa.u0 + pa.du - eps and
            pa.v0 < pb_.v0 + pb_.dv - eps and pb_.v0 < pa.v0 + pa.dv - eps and
            pa.z0 < pb_.z0 + pb_.height - eps and pb_.z0 < pa.z0 + pa.height - eps)


# ------------------------------------------------------------------------- core has no ROS
def test_core_imports_no_ros():
    for mod in list(sys.modules):
        if mod.startswith('rclpy'):
            pytest.skip('rclpy already imported by another test module')
    import palletizer_packing.core.env  # noqa: F401
    import palletizer_packing.core.physics  # noqa: F401
    assert not any(m.startswith('rclpy') for m in sys.modules)


# ------------------------------------------------------------------------- pallet model
def test_heightmap_update_and_rest_height(spec):
    s = PalletState(spec)
    p = put(s, 0.0, 0.0, 0.40, 0.30, 0.25)
    assert s.height[:30, :40].min() == pytest.approx(0.25)
    assert s.height[30:, :].max() == 0.0 and s.height[:, 40:].max() == 0.0
    assert s.owner[0, 0] == 0
    assert s.rest_height(0.2, 0.1, 0.4, 0.3) == pytest.approx(0.25)
    assert s.rest_height(0.41, 0.0, 0.3, 0.3) == 0.0          # 1 cm gap: no overlap
    assert p.top == pytest.approx(0.25)
    vol = 0.4 * 0.3 * 0.25
    assert s.fill_percent() == pytest.approx(100 * vol / (1.2 * 0.8 * 1.05))


def test_roughness_zero_when_flat(spec):
    s = PalletState(spec)
    assert s.roughness() == 0.0
    put(s, 0.0, 0.0, 1.2, 0.8, 0.2)
    assert s.roughness() == 0.0


# ------------------------------------------------------------------------- constraints
def test_bounds_and_height(spec, rules):
    s = PalletState(spec)
    box = Box(0.4, 0.3, 0.3)
    assert feasible(s, make_placement(s, box, 0.8, 0.5, 0.4, 0.3, False), box, rules, None)[0]
    ok, why = feasible(s, make_placement(s, box, 0.85, 0.0, 0.4, 0.3, False), box, rules, None)
    assert not ok and why == 'out of bounds'
    tall = PalletState(PalletSpec(1.2, 0.8, 0.01, 0.25))
    ok, why = feasible(tall, make_placement(tall, box, 0.0, 0.0, 0.4, 0.3, False), box, rules, None)
    assert not ok and why == 'too high'


def test_support_ratio(spec, rules):
    s = PalletState(spec)
    put(s, 0.0, 0.0, 0.40, 0.40, 0.20)
    box = Box(0.4, 0.4, 0.2)
    full = make_placement(s, box, 0.0, 0.0, 0.4, 0.4, False)
    assert stability(s, full, box, rules).ok
    # half on the box, half over the pallet deck -> 50 % support
    half = make_placement(s, box, 0.2, 0.0, 0.4, 0.4, False)
    st = stability(s, half, box, rules)
    assert not st.ok and st.support_ratio == pytest.approx(0.5, abs=0.03)
    # 80 % supported passes the ratio rule
    most = make_placement(s, box, 0.08, 0.0, 0.4, 0.4, False)
    assert stability(s, most, box, rules).ok


def test_com_outside_support_polygon(spec):
    rules = RuleConfig(support_ratio=0.1)            # isolate the COM rule
    s = PalletState(spec)
    put(s, 0.0, 0.0, 0.20, 0.40, 0.20)               # support only under one end
    box = Box(0.6, 0.4, 0.2)
    p = make_placement(s, box, 0.0, 0.0, 0.6, 0.4, False)
    st = stability(s, p, box, rules)
    assert not st.ok and not st.com_inside


def test_mass_rule(spec, rules):
    s = PalletState(spec)
    put(s, 0.0, 0.0, 0.4, 0.4, 0.2, mass=4.0)
    ok_box = Box(0.4, 0.4, 0.2, mass=6.0)             # 1.5 x 4 = 6 -> allowed
    heavy = Box(0.4, 0.4, 0.2, mass=6.5)
    unknown = Box(0.4, 0.4, 0.2, mass=0.0)
    for box, expect in ((ok_box, True), (heavy, False), (unknown, True)):
        p = make_placement(s, box, 0.0, 0.0, 0.4, 0.4, False)
        assert stability(s, p, box, rules).ok is expect


def test_reach_over(spec, rules):
    s = PalletState(spec)
    put(s, 0.3, 0.0, 0.4, 0.3, 0.40)                  # tall stack next to the robot edge
    far = make_placement(s, Box(0.4, 0.3, 0.2), 0.3, 0.45, 0.4, 0.3, False)
    assert far.z0 == 0.0
    assert not no_reach_over(s, far, rules)           # would carry the box over a 0.40 m stack
    beside = make_placement(s, Box(0.4, 0.3, 0.2), 0.78, 0.45, 0.4, 0.3, False)
    assert no_reach_over(s, beside, rules)


def test_reachability_map_lookup():
    u = np.array([0.1, 0.2, 0.3])
    v = np.array([0.1, 0.2])
    h = np.array([0.2, 0.4])
    reach = np.ones((2, 2, 2, 3), dtype=bool)
    reach[:, 1, 1, 2] = False                          # (u=0.3, v=0.2, h=0.4) unreachable
    m = ReachabilityMap(u, v, h, reach, 0.4)
    assert m.reachable(0.1, 0.1, 0.3)
    assert not m.reachable(0.25, 0.15, 0.3)            # conservative: next level + 4 neighbours
    assert m.reachable(0.15, 0.15, 0.4)
    assert not m.reachable(0.1, 0.1, 0.45)             # above the map


def test_real_reachability_map(cell):
    path = os.path.join(HERE, '..', '..', 'palletizer_motion', 'config',
                        cell['reachability']['file'])
    if not os.path.exists(path):
        pytest.skip('reachability map not built')
    m = ReachabilityMap.load(path)
    assert m.usable_top_height == pytest.approx(1.05)
    assert m.reachable(0.6, 0.4, 0.6)                  # middle of the pallet
    assert not m.reachable(0.1, 0.7, 1.2)              # far top corner
    assert not m.reachable(0.6, 0.1, 0.15)             # in front of the pedestal, low


# ------------------------------------------------------------------------- planners
def test_candidates_include_contact_points(spec):
    s = PalletState(spec)
    put(s, 0.0, 0.5, 0.33, 0.3, 0.2)
    c = candidates(s, Box(0.3, 0.2, 0.2), 0.05, 0.01)
    assert any(abs(u - 0.34) < 1e-6 and abs(v - 0.5) < 1e-6 for u, v, *_ in c)


def test_naive_first_feasible_in_scan_order(spec, rules):
    s = PalletState(spec)
    pl = NaivePlanner(rules, PlannerConfig(0.05, 0.01, None, 0))
    p = pl.plan(s, Box(0.4, 0.3, 0.2))
    assert p.u0 == 0.0 and p.v0 + p.dv == pytest.approx(0.8) and not p.rotated


def test_heuristic_prefers_far_bottom_left(spec, rules, cell):
    s = PalletState(spec)
    pl = HeuristicPlanner(rules, PlannerConfig.from_config(cell) if cell else None, None, None)
    pl.cfg.physics_top_k = 0
    p = pl.plan(s, Box(0.4, 0.3, 0.2))
    assert p.z0 == 0.0 and p.v0 + p.dv == pytest.approx(0.8) and p.u0 == 0.0


@pytest.mark.parametrize('planner_name', ['naive', 'heuristic'])
def test_sequence_valid_packing(spec, rules, cell, planner_name):
    """Pack a sequence: every placement obeys all rules and no two boxes intersect."""
    cls = NaivePlanner if planner_name == 'naive' else HeuristicPlanner
    cfg = PlannerConfig.from_config(cell)
    cfg.physics_top_k = 0
    pl = cls(rules, cfg, None, None)
    s = PalletState(spec)
    for box in box_sequence(cell['spawner'], 3, 60):
        p = pl.plan(s, box)
        if p is None:
            break
        ok, why = feasible(s, p, box, rules, None)
        assert ok, why
        s.place(p, box)
    assert len(s.boxes) >= 5
    for a, b in itertools.combinations(s.boxes, 2):
        assert not overlaps_3d(a, b), (a.placement, b.placement)
    assert s.max_height <= spec.max_height + 1e-9


def offline_fill(cell, planner_cls, seed, spec, rules):
    cfg = PlannerConfig.from_config(cell)
    cfg.physics_top_k = 0
    pl = planner_cls(rules, cfg, None, None)
    s = PalletState(spec)
    for box in box_sequence(cell['spawner'], seed, 200):
        p = pl.plan(s, box)
        if p is None:
            break
        s.place(p, box)
    return s.fill_percent()


def test_heuristic_fills_better_than_naive_offline(cell, spec, rules):
    """Paired over 40 seeds (200-239, disjoint from the tuning seeds 100-159). Per-pallet
    variance is large (~8 % std), so a handful of seeds is not enough to compare."""
    seeds = range(200, 240)
    naive = np.array([offline_fill(cell, NaivePlanner, sd, spec, rules) for sd in seeds])
    heur = np.array([offline_fill(cell, HeuristicPlanner, sd, spec, rules) for sd in seeds])
    d = heur - naive
    print(f'naive {naive.mean():.1f} %, heuristic {heur.mean():.1f} %, '
          f'better {(d > 0).sum()} / worse {(d < 0).sum()} / equal {(d == 0).sum()}')
    assert heur.mean() > naive.mean()
    assert (d > 0).sum() > (d < 0).sum()


# ------------------------------------------------------------------------- physics
def test_physics_checker(spec, rules, cell):
    from palletizer_packing.core.physics import PhysicsChecker, physics_config
    ph = PhysicsChecker(physics_config(cell))
    try:
        s = PalletState(spec)
        box = Box(0.4, 0.3, 0.3, 5.0)
        assert ph.stable(s, make_placement(s, box, 0.0, 0.0, 0.4, 0.3, False), box)
        put(s, 0.0, 0.0, 0.2, 0.3, 0.3)
        # long box resting on a narrow support with its COM far outside: tips over
        wide = Box(0.6, 0.3, 0.2, 8.0)
        p = make_placement(s, wide, 0.0, 0.0, 0.6, 0.3, False)
        assert not ph.stable(s, p, wide)
        assert ph.last['max_rotation_deg'] > 2.0
    finally:
        ph.close()


def test_heuristic_with_physics(spec, rules, cell):
    from palletizer_packing.core.physics import PhysicsChecker, physics_config
    ph = PhysicsChecker(physics_config(cell))
    try:
        pl = HeuristicPlanner(rules, PlannerConfig.from_config(cell), None, ph)
        s = PalletState(spec)
        placed = 0
        for box in box_sequence(cell['spawner'], 5, 30):
            p = pl.plan(s, box)
            if p is None:                      # pallet full
                break
            assert 1 <= p.info['physics_checks'] <= pl.cfg.physics_top_k
            s.place(p, box)
            placed += 1
        assert placed >= 3
    finally:
        ph.close()


# ------------------------------------------------------------------------- gymnasium env
def test_gym_env(cell, spec, rules):
    from gymnasium.utils.env_checker import check_env
    from palletizer_packing.core.env import PalletizingEnv
    env = PalletizingEnv(spec, rules, cell['spawner'], n_boxes=30, lookahead=2)
    check_env(env, skip_render_check=True)
    obs, info = env.reset(seed=4)
    assert obs['heightmap'].shape == spec.shape and obs['box'].shape == (4,)
    total, steps = 0.0, 0
    terminated = truncated = False
    while not (terminated or truncated) and steps < 40:
        mask = env.action_masks()
        a = int(np.flatnonzero(mask)[0]) if mask.any() else 0
        obs, r, terminated, truncated, info = env.step(a)
        total += r
        steps += 1
    assert total > 0 and info['fill_percent'] > 0
    # invalid action is penalized and the box is offered again
    env.reset(seed=4)
    bad = env.action_space.n - 1                     # far corner, rotated: out of bounds
    obs2, r, *_ = env.step(bad)
    assert r == env.invalid_penalty
