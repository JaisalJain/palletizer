"""Reachability map lookup (built offline by palletizer_motion build_reachability_map)."""

import numpy as np


class ReachabilityMap:
    """Conservative lookup: a (u, v, top) query is reachable only if all surrounding grid
    nodes (2 in u x 2 in v x the next height level up) are reachable for some yaw."""

    def __init__(self, u, v, h, reach, usable_top_height=None):
        self.u = np.asarray(u, dtype=float)
        self.v = np.asarray(v, dtype=float)
        self.h = np.asarray(h, dtype=float)
        self.any = np.asarray(reach).any(axis=0) if np.asarray(reach).ndim == 4 else np.asarray(reach)
        self.usable_top_height = usable_top_height

    @classmethod
    def load(cls, path):
        d = np.load(path)
        return cls(d['u'], d['v'], d['h'], d['reach'], float(d['usable_top_height']))

    @classmethod
    def everywhere(cls, spec):
        """Map that allows everything (tests / RL without a robot)."""
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
