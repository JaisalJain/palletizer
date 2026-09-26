"""Reachability map lookup (built offline by palletizer_motion build_reachability_map).

The map is a boolean grid over the pallet STATION area in world coordinates (x, y, box-top
z). The robot does not know the pallet in advance: PalletReach intersects the map with the
MEASURED pallet (pallet frame -> world) and derives the usable stack height for it.
"""

import numpy as np


class ReachabilityMap:
    """Conservative lookup on a regular grid (a0, a1, h): a query is reachable only if all
    surrounding grid nodes (2 x 2 in the plane x the next height level up) are reachable."""

    def __init__(self, a0, a1, h, reach, usable_top_height=None):
        self.u = np.asarray(a0, dtype=float)
        self.v = np.asarray(a1, dtype=float)
        self.h = np.asarray(h, dtype=float)
        r = np.asarray(reach)
        self.any = r.any(axis=0) if r.ndim == 4 else r
        self.usable_top_height = usable_top_height

    @classmethod
    def load(cls, path):
        d = np.load(path)
        if 'x' in d:                                         # world-frame station map (v2)
            return cls(d['x'], d['y'], d['z'], d['reach'])
        return cls(d['u'], d['v'], d['h'], d['reach'], float(d['usable_top_height']))

    @classmethod
    def everywhere(cls, spec):
        """Map that allows everything (tests / RL without a robot), in pallet coordinates."""
        u = np.array([0.0, spec.length])
        v = np.array([0.0, spec.width])
        h = np.array([0.0, spec.max_height])
        return cls(u, v, h, np.ones((2, 2, 2), dtype=bool), spec.max_height)

    @staticmethod
    def _bracket(axis, x):
        if x <= axis[0]:
            return [0]
        if x >= axis[-1]:
            return [len(axis) - 1]
        i = int(np.searchsorted(axis, x))
        if abs(axis[i] - x) < 1e-6:
            return [i]
        return [i - 1, i]

    def inside(self, a, b, tol=1e-6):
        return (self.u[0] - tol <= a <= self.u[-1] + tol and
                self.v[0] - tol <= b <= self.v[-1] + tol)

    def reachable(self, u, v, top):
        if top > self.h[-1] + 1e-6:
            return False
        ih = int(np.searchsorted(self.h, top - 1e-6))
        ih = min(max(ih, 0), len(self.h) - 1)
        for iv in self._bracket(self.v, v):
            for iu in self._bracket(self.u, u):
                if not self.any[ih, iv, iu]:
                    return False
        return True


class PalletReach:
    """The world map seen from a measured pallet: reachable(u, v, top above the deck).

    to_world(u, v, h) -> world (x, y, z). Positions outside the mapped station area are
    unreachable (the map says nothing about them)."""

    def __init__(self, world_map: ReachabilityMap, to_world, size_u, size_v):
        self.map = world_map
        self.to_world = to_world
        self.size_u, self.size_v = size_u, size_v
        self.edge = 0.03       # box centres this close to the mapped area border still count

    def reachable(self, u, v, top):
        x, y, z = self.to_world(u, v, top)
        if not self.map.inside(x, y, self.edge):
            return False
        return self.map.reachable(x, y, z)

    def coverage(self, top, inset, step=0.05):
        """Fraction of box-centre positions (grid over the footprint inset by `inset`) that
        are reachable at box-top height `top` above the deck."""
        us = np.arange(inset, self.size_u - inset + 1e-9, step)
        vs = np.arange(inset, self.size_v - inset + 1e-9, step)
        if len(us) == 0 or len(vs) == 0:
            return 0.0
        ok = [self.reachable(u, v, top) for v in vs for u in us]
        return float(np.mean(ok))

    def usable_top_height(self, inset, max_height, step=0.05, min_coverage=1.0):
        """Highest box-top height (above the deck) up to which coverage stays >= min_coverage,
        starting from the first fully covered level (low levels in front of the pedestal are
        a near-field keep-out handled per position, not a height limit; as in v1)."""
        levels = np.arange(step, max_height + 1e-9, step)
        cov = [self.coverage(h, inset) for h in levels]
        full = [i for i, c in enumerate(cov) if c + 1e-9 >= min_coverage]
        if not full:
            return 0.0
        usable = float(levels[full[0]])
        for i in range(full[0], len(levels)):
            if cov[i] + 1e-9 < min_coverage:
                break
            usable = float(levels[i])
        return usable
