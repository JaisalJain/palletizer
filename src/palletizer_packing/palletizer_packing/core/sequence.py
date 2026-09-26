"""Seeded random box sequences with the spawner's distribution (offline tests / RL)."""

import numpy as np

from palletizer_packing.core.pallet import Box


def box_sequence(spawner_cfg, seed, n):
    rng = np.random.default_rng(seed)
    spawner_cfg = spawner_cfg.get('mixed', spawner_cfg)
    out = []
    for _ in range(n):
        size = [float(rng.uniform(lo, hi)) for lo, hi in
                zip(spawner_cfg['size_min'], spawner_cfg['size_max'])]
        mass = float(rng.uniform(*spawner_cfg['mass_range']))
        L, W = max(size[0], size[1]), min(size[0], size[1])
        out.append(Box(L, W, size[2], mass))
    return out
