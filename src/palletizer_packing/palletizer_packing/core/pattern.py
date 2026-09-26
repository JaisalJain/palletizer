"""Single-SKU layer patterns (manufacturer's pallet loading problem). No ROS imports.

Layer search (identical boxes a x b, both orientations, gap g between boxes, none at the
pallet edges): work with inflated boxes (a+g, b+g) in the inflated pallet (U+g, V+g).
  * guillotine DP over the normal sets (all i*a' + j*b' <= side) gives the best count f(x, y)
    of every sub-rectangle, with the arrangement,
  * first-order non-guillotine (5-block / pinwheel) patterns: a centre block and four blocks
    around it, each filled by the DP. Every pattern with count >= best - drop is collected.
  Exhaustive over the normal sets, fine for pallet-sized problems (ms to a few s).
Layers: layer A and a different layer B that interlocks with it (B boxes bridging A boxes),
chosen among the collected patterns, their mirror images and slack positions:
  * pattern_priority 'count': max boxes per layer first; column stacking (B = A) when no
    other layer with the same count interlocks,
  * 'interlock': B may drop up to `interlock_drop` boxes if that buys interlocking.
Every slot must be reachable (reachability map, at its layer's height); slots that are not
are dropped (the count is reported). Every layer is checked with the static stability rules
and (optionally) the PyBullet checker. Placement order within a layer: far side first, so
the arm never reaches over boxes of the current layer.
"""

from dataclasses import dataclass, field
from functools import lru_cache
import math
import time

from palletizer_packing.core.pallet import Box, PalletSpec, PalletState, Placement
from palletizer_packing.core.rules import RuleConfig, stability


@dataclass(frozen=True)
class Rect:
    """Box footprint in the pallet frame [m]; rotated = box length along v."""
    u0: float
    v0: float
    du: float
    dv: float
    rotated: bool

    @property
    def center(self):
        return self.u0 + self.du / 2, self.v0 + self.dv / 2


@dataclass
class Layout:
    rects: list                     # [Rect] (placement order not yet applied)
    kind: str = ''                  # grid / guillotine / 5-block / mirror ...

    @property
    def count(self):
        return len(self.rects)

    def key(self):
        return tuple(sorted((round(r.u0, 4), round(r.v0, 4), r.rotated) for r in self.rects))


@dataclass
class PatternConfig:
    gap: float = 0.005
    priority: str = 'count'         # count | interlock
    interlock_drop: int = 1         # boxes per layer 'interlock' may give up
    min_interlock: float = 0.3      # fraction of layer-B boxes that must bridge >= 2 boxes
    max_layouts: int = 400

    @classmethod
    def from_config(cls, cell):
        p = cell['packing']['pattern']
        return cls(cell['packing']['pattern_gap'], p['priority'], p['interlock_drop'],
                   p['min_interlock'], p['max_layouts'])


# --------------------------------------------------------------------------- layer search
def _normal_set(side, a, b):
    out = set()
    for i in range(side // a + 1):
        for j in range((side - i * a) // b + 1):
            out.add(i * a + j * b)
    return sorted(out)


class LayerSearch:
    """Integer (mm) search on inflated dims."""

    def __init__(self, U, V, a, b):
        self.U, self.V, self.a, self.b = U, V, a, b
        self.nx = _normal_set(U, a, b)
        self.ny = _normal_set(V, a, b)
        self.f = lru_cache(maxsize=None)(self._f)

    def _snap(self, x, n):
        # largest normal value <= x: the usable part of a side
        lo = 0
        for v in n:
            if v <= x:
                lo = v
        return lo

    def _homog(self, x, y):
        a, b = self.a, self.b
        c1 = (x // a) * (y // b)
        c2 = (x // b) * (y // a)
        return (c1, 'h0') if c1 >= c2 else (c2, 'h1')

    def _f(self, x, y):
        """(count, plan) best guillotine arrangement of an x-by-y rectangle."""
        x, y = self._snap(x, self.nx), self._snap(y, self.ny)
        if x < min(self.a, self.b) or y < min(self.a, self.b):
            return 0, None
        best = self._homog(x, y)
        best = (best[0], (best[1],))
        for c in self.nx:
            if c <= 0 or c > x // 2:
                continue
            n = self.f(c, y)[0] + self.f(x - c, y)[0]
            if n > best[0]:
                best = (n, ('v', c))
        for c in self.ny:
            if c <= 0 or c > y // 2:
                continue
            n = self.f(x, c)[0] + self.f(x, y - c)[0]
            if n > best[0]:
                best = (n, ('h', c))
        return best

    def rects(self, x0, y0, x, y):
        """Inflated rectangles (x, y, w, h, rotated) of the DP arrangement of a block."""
        n, plan = self.f(x, y)
        if not n:
            return []
        x, y = self._snap(x, self.nx), self._snap(y, self.ny)
        if plan[0] in ('h0', 'h1'):
            w, h = (self.a, self.b) if plan[0] == 'h0' else (self.b, self.a)
            return [(x0 + i * w, y0 + j * h, w, h, plan[0] == 'h1')
                    for j in range(y // h) for i in range(x // w)]
        if plan[0] == 'v':
            c = plan[1]
            return self.rects(x0, y0, c, y) + self.rects(x0 + c, y0, x - c, y)
        c = plan[1]
        return self.rects(x0, y0, x, c) + self.rects(x0, y0 + c, x, y - c)

    def best(self):
        return self.f(self.U, self.V)[0]

    def layouts(self, drop=0, limit=400):
        """Guillotine best + all 5-block patterns with count >= best5 - drop."""
        U, V = self.U, self.V
        found = {}
        g = self.rects(0, 0, U, V)
        found[_key(g)] = (len(g), g, 'guillotine')
        xs = [x for x in self.nx if x <= U] + [U]
        ys = [y for y in self.ny if y <= V] + [V]
        cand = []
        best = len(g)
        for x1 in xs:
            for x2 in xs:
                if x2 < x1:
                    continue
                for y1 in ys:
                    for y2 in ys:
                        if y2 < y1:
                            continue
                        blocks = ((0, 0, x2, y1), (x2, 0, U - x2, y2), (x1, y2, U - x1, V - y2),
                                  (0, y1, x1, V - y1), (x1, y1, x2 - x1, y2 - y1))
                        n = sum(self.f(w, h)[0] for _, _, w, h in blocks if w > 0 and h > 0)
                        if n >= best - drop:
                            cand.append((n, blocks))
                            best = max(best, n)
        cand.sort(key=lambda c: -c[0])
        for n, blocks in cand:
            if n < best - drop:
                continue
            rs = []
            for bx, by, w, h in blocks:
                if w > 0 and h > 0:
                    rs += self.rects(bx, by, w, h)
            k = _key(rs)
            if k not in found:
                found[k] = (len(rs), rs, '5-block')
            if len(found) >= limit:
                break
        return best, sorted(found.values(), key=lambda t: -t[0])


def _key(rs):
    return tuple(sorted((r[0], r[1], r[4]) for r in rs))


def search_layouts(U, V, L, W, gap, drop=0, limit=400):
    """Best count and layouts of L x W boxes on a U x V deck (m), gap between boxes.
    Returns (best, [Layout]) with rects in metres, in the deck frame."""
    mm = lambda x: int(math.floor(x * 1000 + 1e-6))      # noqa: E731
    a, b, g = mm(L + gap), mm(W + gap), mm(gap)
    s = LayerSearch(mm(U) + g, mm(V) + g, a, b)
    best, found = s.layouts(drop, limit)
    out = []
    for n, rs, kind in found:
        rects = [Rect(x / 1000, y / 1000, (w - g) / 1000, (h - g) / 1000, rot)
                 for x, y, w, h, rot in rs]
        out.append(Layout(rects, kind))
    return best, out


# --------------------------------------------------------------------------- variants
def _bbox(rects):
    return (min(r.u0 for r in rects), min(r.v0 for r in rects),
            max(r.u0 + r.du for r in rects), max(r.v0 + r.dv for r in rects))


def variants(layout, U, V):
    """Mirror images (u, v, both) x slack placement (centred / pushed to the far side)."""
    out = []
    if not layout.rects:
        return out
    for mu in (False, True):
        for mv in (False, True):
            rs = [Rect(U - r.u0 - r.du if mu else r.u0, V - r.v0 - r.dv if mv else r.v0,
                       r.du, r.dv, r.rotated) for r in layout.rects]
            u0, v0, u1, v1 = _bbox(rs)
            for slack_v in ('centre', 'far'):
                du = (U - (u1 - u0)) / 2 - u0
                dv = ((V - (v1 - v0)) / 2 - v0) if slack_v == 'centre' else (V - v1)
                moved = [Rect(r.u0 + du, r.v0 + dv, r.du, r.dv, r.rotated) for r in rs]
                out.append(Layout(moved, f'{layout.kind}{" mu" if mu else ""}'
                                         f'{" mv" if mv else ""} {slack_v}'))
    uniq = {}
    for lay in out:
        uniq.setdefault(lay.key(), lay)
    return list(uniq.values())


def far_first(rects):
    """Placement order: far side first (v of the far edge descending), then along u."""
    return sorted(rects, key=lambda r: (-round(r.v0 + r.dv, 2), round(r.u0, 3)))


# --------------------------------------------------------------------------- interlock
def _overlap(r, s):
    du = min(r.u0 + r.du, s.u0 + s.du) - max(r.u0, s.u0)
    dv = min(r.v0 + r.dv, s.v0 + s.dv) - max(r.v0, s.v0)
    return max(du, 0.0) * max(dv, 0.0)


def interlock(upper, lower, min_frac=0.1):
    """Fraction of upper-layer boxes resting on >= 2 lower-layer boxes (each carrying
    >= min_frac of its area): 0 = column stack."""
    if not upper.rects:
        return 0.0
    n = 0
    for r in upper.rects:
        area = r.du * r.dv
        k = sum(1 for s in lower.rects if _overlap(r, s) >= min_frac * area)
        n += k >= 2
    return n / len(upper.rects)


def quick_support(upper, lower, ratio):
    """Every upper box rests with >= ratio of its area on lower boxes (rectangle overlaps;
    equal heights). A fast filter before the full static check."""
    for r in upper.rects:
        if sum(_overlap(r, s) for s in lower.rects) < ratio * r.du * r.dv - 1e-9:
            return False
    return True


# --------------------------------------------------------------------------- plan
@dataclass
class Slot:
    index: int
    layer: int                      # 1-based
    rect: Rect
    z0: float                       # nominal resting height above the deck


@dataclass
class Pattern:
    box: Box
    spec: PalletSpec
    layer_a: Layout
    layer_b: Layout
    layers: int
    slots: list                     # [Slot] in placement order
    optimum: int                    # best count per layer ignoring reachability
    interlock: float                # of B on A
    note: str = ''
    stats: dict = field(default_factory=dict)

    @property
    def count(self):
        return len(self.slots)


def layer_ok(state: PalletState, layout: Layout, box: Box, z0, rules: RuleConfig):
    """Static stability of every box of a layer, placed in order on `state` (modified)."""
    for r in far_first(layout.rects):
        p = Placement(r.u0, r.v0, r.du, r.dv, z0, box.height, r.rotated)
        rest = state.rest_height(p.u0, p.v0, p.du, p.dv)
        if abs(rest - z0) > rules.support_tol:
            return False, f'rest height {rest:.3f} != {z0:.3f}'
        st = stability(state, p, box, rules)
        if not st.ok:
            return False, st.reason
        state.place(p, box)
    return True, ''


def make_pattern(box: Box, spec: PalletSpec, rules: RuleConfig, cfg: PatternConfig,
                 reach=None, physics=None, log=None):
    """Pattern for identical boxes on the measured deck. spec.max_height = usable height.
    reach: object with reachable(u, v, top) (PalletReach) or None."""
    U, V = spec.length, spec.width
    L, W, H = box.length, box.width, box.height
    layers = int(math.floor(spec.max_height / H + 1e-9))
    drop = cfg.interlock_drop if cfg.priority == 'interlock' else 0
    best, layouts = search_layouts(U, V, L, W, cfg.gap, drop, cfg.max_layouts)
    stats = {'layouts': len(layouts), 'optimum': best, 'layers': layers}
    if layers < 1 or best == 0:
        return Pattern(box, spec, Layout([]), Layout([]), 0, [], best, 0.0,
                       'box does not fit', stats)

    cands = []
    for lay in layouts:
        cands += variants(lay, U, V)
    # per candidate: reachable slots at the odd-layer and even-layer heights
    odd = [k for k in range(1, layers + 1, 2)]
    even = [k for k in range(2, layers + 1, 2)]

    def usable(lay, ks):
        """Slots of lay that are reachable on every layer in ks."""
        if not ks:
            return lay
        keep = [r for r in lay.rects
                if all(reach is None or reach.reachable(*r.center, k * H) for k in ks)]
        return Layout(keep, lay.kind)

    def dedupe(opts):
        seen, out = set(), []
        for lay, c in opts:
            k = lay.key()
            if k not in seen:
                seen.add(k)
                out.append((lay, c))
        return out

    A_opts = dedupe(sorted(((usable(c, odd), c) for c in cands), key=lambda t: -t[0].count))
    B_opts = dedupe(sorted(((usable(c, even), c) for c in cands), key=lambda t: -t[0].count))
    top_a = A_opts[0][0].count
    top_b = B_opts[0][0].count if even else 0
    A_opts = [t for t in A_opts if t[0].count >= top_a - drop][:40]
    B_opts = [t for t in B_opts if t[0].count >= top_b - drop][:40] if even else \
        [(Layout([]), None)]
    pairs = {}

    def add_pair(a_lay, b_lay):
        k = (a_lay.key(), b_lay.key())
        if k in pairs or not a_lay.rects:
            return
        if even and not (quick_support(b_lay, a_lay, rules.support_ratio) and
                         (layers < 3 or quick_support(a_lay, b_lay, rules.support_ratio))):
            return
        total = a_lay.count * len(odd) + b_lay.count * len(even)
        il = min(interlock(b_lay, a_lay), interlock(a_lay, b_lay) if layers >= 3 else 1.0) \
            if even else 0.0
        key = (il >= cfg.min_interlock, total, il) if cfg.priority == 'interlock' else \
            (total, il)
        pairs[k] = (key, a_lay, b_lay, il)

    for a_lay, c in A_opts:
        add_pair(a_lay, usable(c, even) if even else Layout([]))     # column stack
        col = usable(c, odd + even)            # column stack of slots reachable on all layers
        add_pair(col, col if even else Layout([]))
        for b_lay, _ in B_opts:
            add_pair(a_lay, b_lay)
    ranked = sorted(pairs.values(), key=lambda t: t[0], reverse=True)

    chosen = None
    tried = 0
    for key, a_lay, b_lay, il in ranked:
        if tried >= 20:
            break
        tried += 1
        ok, why = _check_stack(a_lay, b_lay, box, layers, spec, rules, physics)
        if ok:
            chosen = (a_lay, b_lay, il)
            break
        if log:
            log(f'pattern pair rejected: {why}')
    if chosen is None:
        return Pattern(box, spec, Layout([]), Layout([]), 0, [], best, 0.0,
                       'no stable pattern', stats)
    a_lay, b_lay, il = chosen
    slots = []
    for k in range(1, layers + 1):
        lay = a_lay if k % 2 else b_lay
        for r in far_first(lay.rects):
            slots.append(Slot(len(slots), k, r, (k - 1) * H))
    notes = []
    if even and il < 1e-9:
        notes.append('column stack (no interlocking layer with the same count)')
    elif even:
        notes.append(f'interlocked A/B ({il * 100:.0f} % of boxes bridge)')
    if a_lay.count < best or (even and b_lay.count < best):
        notes.append(f'{best - a_lay.count}/{best - b_lay.count if even else 0} slots of the '
                     f'optimum dropped (unreachable{" / interlock" if drop else ""})')
    stats.update({'per_layer_a': a_lay.count, 'per_layer_b': b_lay.count if even else 0,
                  'pairs_checked': tried})
    return Pattern(box, spec, a_lay, b_lay if even else a_lay, layers, slots, best, il,
                   '; '.join(notes), stats)


def _check_stack(a_lay, b_lay, box, layers, spec, rules, physics):
    """Build the whole pallet in the model: every box statically stable, every layer
    stable in PyBullet (if given)."""
    state = PalletState(spec)
    H = box.height
    for k in range(1, layers + 1):
        lay = a_lay if k % 2 else b_lay
        if not lay.rects:
            return False, f'layer {k} empty'
        before = state.copy()
        ok, why = layer_ok(state, lay, box, (k - 1) * H, rules)
        if not ok:
            return False, f'layer {k}: {why}'
        if physics is not None:
            new = [Placement(r.u0, r.v0, r.du, r.dv, (k - 1) * H, H, r.rotated)
                   for r in far_first(lay.rects)]
            if not physics.stable_many(before, new, box):
                return False, f'layer {k}: PyBullet {physics.last}'
    return True, ''


# --------------------------------------------------------------------------- runtime planner
class PatternPlanner:
    """Single-SKU planner: the first box fixes the SKU (measured dims) and the pattern;
    every later box must match it within `sku_tol`. plan() returns the next slot of the
    pattern, checked against the CURRENT (perceived) state: bounds, height, the layer below
    where the pattern expects it, static stability, and the motion validator."""
    name = 'pattern'

    def __init__(self, rules: RuleConfig, cfg: PatternConfig, sku_tol=0.010, physics=None):
        self.rules, self.cfg, self.sku_tol, self.physics = rules, cfg, sku_tol, physics
        self.reach = None
        self.pattern = None
        self.sku = None
        self.validator = None
        self.validator_max = 4
        self.exclude = []
        self.last_stats = {}
        self.last_reason = ''
        self.log = None

    def reset(self):
        self.pattern, self.sku = None, None

    def matches(self, box: Box):
        s = self.sku
        return (abs(box.length - s.length) <= self.sku_tol and
                abs(box.width - s.width) <= self.sku_tol and
                abs(box.height - s.height) <= self.sku_tol)

    def start(self, box: Box, spec: PalletSpec):
        self.sku = Box(box.length, box.width, box.height, 0.0)
        self.pattern = make_pattern(self.sku, spec, self.rules, self.cfg, self.reach,
                                    self.physics, self.log)
        return self.pattern

    def _fail(self, reason, msg, t0, **kw):
        self.last_reason = reason
        self.last_stats = {'reason': reason, 'message': msg,
                           'planning_time': time.perf_counter() - t0, **kw}
        return None

    def plan(self, state: PalletState, box: Box, lookahead=(), check_only=False):
        t0 = time.perf_counter()
        self.last_reason = ''
        if self.pattern is None or (not state.boxes and not self.matches(box)):
            if state.boxes:
                return self._fail('NO_FEASIBLE_PLACEMENT', 'boxes on the pallet but no pattern',
                                  t0)
            self.start(box, state.spec)          # the first box of a pallet defines the SKU
        elif not self.matches(box):
            s = self.sku
            return self._fail('SKU_CHANGE', f'box {box.length * 1000:.0f} x {box.width * 1000:.0f}'
                              f' x {box.height * 1000:.0f} mm is not SKU {s.length * 1000:.0f} x '
                              f'{s.width * 1000:.0f} x {s.height * 1000:.0f} mm', t0)
        pat = self.pattern
        if pat.count == 0:
            return self._fail('HEIGHT_LIMIT' if pat.layers == 0 else 'NO_FEASIBLE_PLACEMENT',
                              f'no pattern: {pat.note}', t0)
        i = len(state.boxes)
        if i >= pat.count:
            return self._fail('PATTERN_COMPLETE', f'{pat.count} boxes placed', t0)
        slot = pat.slots[i]
        r = slot.rect
        z0 = state.rest_height(r.u0, r.v0, r.du, r.dv)
        p = Placement(r.u0, r.v0, r.du, r.dv, z0, box.height, r.rotated)
        p.info = {'slot': i, 'layer': slot.layer}
        if p.top > state.spec.max_height + self.rules.support_tol:
            return self._fail('HEIGHT_LIMIT', f'slot {i} top {p.top:.3f} m > '
                              f'{state.spec.max_height:.3f} m', t0)
        if abs(z0 - slot.z0) > 2 * self.rules.support_tol + 0.01:
            return self._fail('NO_FEASIBLE_PLACEMENT', f'slot {i}: surface at {z0:.3f} m, '
                              f'pattern expects {slot.z0:.3f} m', t0)
        st = stability(state, p, box, self.rules)
        if not st.ok:
            return self._fail('NO_FEASIBLE_PLACEMENT', f'slot {i}: {st.reason}', t0)
        if any(p.rotated == e.rotated and abs(p.u0 - e.u0) < 0.02 and abs(p.v0 - e.v0) < 0.02
               for e in self.exclude):
            return self._fail('NO_FEASIBLE_PLACEMENT', f'slot {i} failed in motion', t0)
        checks = 0
        # the pre-check (before the pick) only needs SKU, slot and rules; the motion check
        # runs in the full plan, in parallel with the pick
        if self.validator is not None and not check_only:
            checks = 1
            if not self.validator(p):
                return self._fail('NO_FEASIBLE_PLACEMENT', f'slot {i}: motion check failed', t0,
                                  motion_checks=1)
        p.info.update({'candidates_checked': 1, 'physics_checks': 0, 'motion_checks': checks,
                       'planning_time': time.perf_counter() - t0})
        self.last_stats = p.info
        return p
