"""Placement planners: plan(state, box, lookahead=[]) -> Placement | None. No ROS imports."""

from dataclasses import dataclass
import time

import numpy as np
from scipy import ndimage

from palletizer_packing.core.pallet import Box, PalletState, Placement
from palletizer_packing.core.rules import RuleConfig, feasible


def orientations(box: Box):
    out = [(box.length, box.width, False)]
    if abs(box.length - box.width) > 1e-3:
        out.append((box.width, box.length, True))
    return out


def candidates(state: PalletState, box: Box, step: float, gap: float):
    """Footprint min corners: a coarse grid plus contact points next to placed boxes and
    flush with the pallet edges, for both orientations."""
    s = state.spec
    out = []
    for du, dv, rot in orientations(box):
        us = set(np.round(np.arange(0.0, s.length - du + 1e-9, step), 4))
        vs = set(np.round(np.arange(0.0, s.width - dv + 1e-9, step), 4))
        us.add(round(s.length - du, 4))
        vs.add(round(s.width - dv, 4))
        for b in state.boxes:
            p = b.placement
            us.update({round(p.u0 + p.du + gap, 4), round(p.u0 - gap - du, 4), round(p.u0, 4),
                       round(p.u0 + p.du - du, 4)})
            vs.update({round(p.v0 + p.dv + gap, 4), round(p.v0 - gap - dv, 4), round(p.v0, 4),
                       round(p.v0 + p.dv - dv, 4)})
        us = [u for u in us if -1e-9 <= u <= s.length - du + 1e-9]
        vs = [v for v in vs if -1e-9 <= v <= s.width - dv + 1e-9]
        for v in vs:
            for u in us:
                out.append((u, v, du, dv, rot))
    return out


def make_placement(state, box, u0, v0, du, dv, rot):
    """Placement resting on the highest cell under the footprint."""
    z0 = state.rest_height(u0, v0, du, dv)
    return Placement(u0=u0, v0=v0, du=du, dv=dv, z0=z0, height=box.height, rotated=rot)


@dataclass
class PlannerConfig:
    step: float = 0.05
    gap: float = 0.01
    weights: dict = None
    physics_top_k: int = 3

    @classmethod
    def from_config(cls, cell):
        pk = cell['packing']
        return cls(pk['candidate_step'], pk['gap'], dict(pk['heuristic']),
                   pk['physics']['top_k'] if pk['physics']['enabled'] else 0)


class Planner:
    name = 'base'

    def __init__(self, rules: RuleConfig, cfg: PlannerConfig, reach_map=None, physics=None):
        self.rules = rules
        self.cfg = cfg
        self.reach = reach_map
        self.physics = physics
        self.last_stats = {}
        self.exclude = []          # placements that failed in motion (same box): skip them

    def excluded(self, p):
        return any(p.rotated == e.rotated and abs(p.u0 - e.u0) < 0.02 and abs(p.v0 - e.v0) < 0.02
                   for e in self.exclude)

    def plan(self, state: PalletState, box: Box, lookahead=(), check_only=False):
        raise NotImplementedError

    def feasible_candidates(self, state, box):
        out = []
        n = 0
        for u, v, du, dv, rot in candidates(state, box, self.cfg.step, self.cfg.gap):
            n += 1
            p = make_placement(state, box, u, v, du, dv, rot)
            if self.excluded(p):
                continue
            ok, _ = feasible(state, p, box, self.rules, self.reach)
            if ok:
                out.append(p)
        return out, n


class NaivePlanner(Planner):
    """First feasible candidate in scan order: far rows first (v descending), then u
    ascending, orientation 0 before 90 deg. No scoring, no physics check."""
    name = 'naive'

    def plan(self, state, box, lookahead=(), check_only=False):
        t0 = time.perf_counter()
        cands = candidates(state, box, self.cfg.step, self.cfg.gap)
        cands.sort(key=lambda c: (-round(c[1] + c[3], 4), round(c[0], 4), c[4]))
        n = 0
        for u, v, du, dv, rot in cands:
            n += 1
            p = make_placement(state, box, u, v, du, dv, rot)
            if self.excluded(p):
                continue
            ok, _ = feasible(state, p, box, self.rules, self.reach)
            if ok:
                p.info = {'candidates_checked': n, 'physics_checks': 0,
                          'planning_time': time.perf_counter() - t0}
                self.last_stats = p.info
                return p
        self.last_stats = {'candidates_checked': n, 'physics_checks': 0,
                           'planning_time': time.perf_counter() - t0}
        return None


class HeuristicPlanner(Planner):
    """Deepest-bottom-left fill with a score (lower resulting height, more side contact,
    less height-map roughness), filling from the far side toward the robot. The top-k
    candidates are verified with the physics checker."""
    name = 'heuristic'

    def score(self, state: PalletState, p: Placement):
        w = self.cfg.weights
        s = state.spec
        r = s.resolution
        # contact: fraction of the footprint perimeter bordered by the pallet edge or by a
        # neighbour that reaches at least half-way up the box side
        sv, su = state.cells(p.u0, p.v0, p.du, p.dv)
        ring = int(round(self.cfg.gap / r)) + 1
        H = state.height
        contact, total = 0, 0
        thresh = p.z0 + 0.5 * p.height
        for side in ('u-', 'u+', 'v-', 'v+'):
            if side in ('u-', 'u+'):
                n = sv.stop - sv.start
                total += n
                col = su.start - ring if side == 'u-' else su.stop - 1 + ring
                if col < 0 or col >= H.shape[1]:
                    contact += n
                else:
                    contact += int((H[sv, col] >= thresh).sum())
            else:
                n = su.stop - su.start
                total += n
                row = sv.start - ring if side == 'v-' else sv.stop - 1 + ring
                if row < 0 or row >= H.shape[0]:
                    contact += n if side == 'v+' else 0     # robot-side edge is open
                else:
                    contact += int((H[row, su] >= thresh).sum())
        contact_frac = contact / max(total, 1)
        # roughness change
        after = H.copy()
        after[sv, su] = p.top
        rough = state.roughness(after) - state.roughness()
        far = 1.0 - (p.v0 + p.dv) / s.width
        deck = float((H[sv, su] <= 1e-9).sum()) * r * r / (s.length * s.width)
        return (w['w_rest_height'] * p.z0 +
                w.get('w_deck', 0.0) * deck +
                w['w_max_height'] * max(0.0, p.top - state.max_height) +
                w['w_far'] * far +
                w['w_left'] * p.u0 / s.length -
                w['w_contact'] * contact_frac +
                w['w_roughness'] * rough)

    def reserve_regions(self, state, height):
        """Number of positions where a `reserve_footprint` box would stand on a flat region
        (window max - min <= support_tol) and still fit under the max height."""
        w = self.cfg.weights
        res = state.spec.resolution
        ku, kv = (int(round(x / res)) for x in w.get('reserve_footprint', (0.5, 0.5)))
        H = height
        count = 0
        for a, b in {(ku, kv), (kv, ku)}:
            if a > H.shape[1] or b > H.shape[0]:
                continue
            mx = ndimage.maximum_filter(H, size=(b, a), mode='nearest')
            mn = ndimage.minimum_filter(H, size=(b, a), mode='nearest')
            ok = (mx - mn <= self.rules.support_tol) & (mx + 0.15 <= state.spec.max_height)
            ok = ok[b // 2:H.shape[0] - (b - 1) // 2, a // 2:H.shape[1] - (a - 1) // 2]
            count += int(ok.sum())
        return count

    def rerank(self, state, feas):
        """Re-rank the best candidates: don't destroy the last region that can take a big box."""
        w = self.cfg.weights
        n = int(w.get('rerank_top_n', 0))
        if n <= 0 or not feas:
            return feas
        before = self.reserve_regions(state, state.height)
        head = feas[:n]
        for p in head:
            H = state.height.copy()
            sv, su = state.cells(p.u0, p.v0, p.du, p.dv)
            H[sv, su] = p.top
            after = self.reserve_regions(state, H)
            p.score += (w.get('w_reserve', 0.0) * float(before > 0 and after == 0) -
                        w.get('w_regions', 0.0) * min(after, 200))
        head.sort(key=lambda p: p.score)
        return head + feas[n:]

    def plan(self, state, box, lookahead=(), check_only=False):
        t0 = time.perf_counter()
        feas, n = self.feasible_candidates(state, box)
        for p in feas:
            p.score = self.score(state, p)
        feas.sort(key=lambda p: p.score)
        if not check_only:
            feas = self.rerank(state, feas)
        chosen, checks = None, 0
        if check_only or self.physics is None or self.cfg.physics_top_k <= 0:
            chosen = feas[0] if feas else None
        else:
            for p in feas[:self.cfg.physics_top_k]:
                checks += 1
                if self.physics.stable(state, p, box):
                    chosen = p
                    break
        stats = {'candidates_checked': n, 'feasible': len(feas), 'physics_checks': checks,
                 'planning_time': time.perf_counter() - t0}
        self.last_stats = stats
        if chosen is not None:
            chosen.info = stats
        return chosen


PLANNERS = {'naive': NaivePlanner, 'heuristic': HeuristicPlanner}


def make_planner(name, cell, reach_map=None, physics=None):
    return PLANNERS[name](RuleConfig.from_config(cell), PlannerConfig.from_config(cell),
                          reach_map, physics)
