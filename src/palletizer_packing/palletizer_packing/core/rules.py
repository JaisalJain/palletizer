"""Feasibility constraints and static stability rules. No ROS imports."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import ConvexHull

from palletizer_packing.core.pallet import Box, PalletState, Placement


@dataclass
class RuleConfig:
    support_ratio: float = 0.75
    support_tol: float = 0.005
    mass_ratio: float = 1.5
    reach_over_clearance: float = 0.10
    pre_place_height: float = 0.15
    reach_over_margin: float = 0.05     # corridor half-width margin beyond the box footprint [m]

    @classmethod
    def from_config(cls, cell):
        pk = cell['packing']
        return cls(pk['support_ratio'], pk['support_tol'], pk['mass_ratio'],
                   pk['reach_over_clearance'], cell['motion']['pre_place_height'])


def in_bounds(state: PalletState, p: Placement):
    s = state.spec
    e = 1e-6
    return p.u0 >= -e and p.v0 >= -e and p.u0 + p.du <= s.length + e and p.v0 + p.dv <= s.width + e


def under_max_height(state: PalletState, p: Placement):
    return p.top <= state.spec.max_height + 1e-6


def reachable(reach_map, p: Placement):
    if reach_map is None:
        return True
    u, v = p.center
    return reach_map.reachable(u, v, p.top)


def no_reach_over(state: PalletState, p: Placement, cfg: RuleConfig):
    """Stacks between the robot edge (v = 0) and the box must stay below
    rest height + pre_place_height - clearance, so the carried box never passes over them."""
    if p.v0 <= 1e-6:
        return True
    sv, su = state.cells(p.u0 - cfg.reach_over_margin, 0.0,
                         p.du + 2 * cfg.reach_over_margin, p.v0)
    corridor = state.height[sv, su]
    if corridor.size == 0:
        return True
    return float(corridor.max()) <= p.z0 + cfg.pre_place_height - cfg.reach_over_clearance + 1e-6


@dataclass
class Stability:
    ok: bool
    support_ratio: float
    com_inside: bool
    mass_ok: bool
    reason: str = ''


def stability(state: PalletState, p: Placement, box: Box, cfg: RuleConfig):
    sv, su = state.cells(p.u0, p.v0, p.du, p.dv)
    region = state.height[sv, su]
    if p.z0 <= 1e-6:
        return Stability(True, 1.0, True, True)     # on the pallet deck
    supported = region >= p.z0 - cfg.support_tol
    ratio = float(supported.mean())
    if ratio < cfg.support_ratio:
        return Stability(False, ratio, False, True, f'support {ratio:.2f} < {cfg.support_ratio}')

    # centre of mass (uniform box -> footprint centre) inside the support polygon
    r = state.spec.resolution
    iv, iu = np.nonzero(supported)
    iv = iv + sv.start
    iu = iu + su.start
    corners = np.concatenate([
        np.c_[iu * r, iv * r], np.c_[(iu + 1) * r, iv * r],
        np.c_[iu * r, (iv + 1) * r], np.c_[(iu + 1) * r, (iv + 1) * r]])
    # clip support polygon to the footprint
    corners[:, 0] = np.clip(corners[:, 0], p.u0, p.u0 + p.du)
    corners[:, 1] = np.clip(corners[:, 1], p.v0, p.v0 + p.dv)
    c = np.array(p.center)
    try:
        hull = ConvexHull(np.unique(corners, axis=0))
        inside = bool(np.all(hull.equations[:, :2] @ c + hull.equations[:, 2] <= 1e-9))
    except Exception:  # noqa: BLE001  (degenerate hull)
        inside = False
    if not inside:
        return Stability(False, ratio, False, True, 'centre of mass outside support polygon')

    # mass rule: not heavier than mass_ratio x any box directly under it
    if box.mass > 0:
        owners = np.unique(state.owner[sv, su][supported])
        for oid in owners:
            if oid < 0:
                continue
            m_below = state.boxes[oid].box.mass
            if m_below > 0 and box.mass > cfg.mass_ratio * m_below + 1e-9:
                return Stability(False, ratio, True, False,
                                 f'{box.mass:.1f} kg > {cfg.mass_ratio} x {m_below:.1f} kg below')
    return Stability(True, ratio, True, True)


def feasible(state, p, box, cfg, reach_map):
    """(ok, reason). Cheap checks first."""
    if not in_bounds(state, p):
        return False, 'out of bounds'
    if not under_max_height(state, p):
        return False, 'too high'
    if not reachable(reach_map, p):
        return False, 'unreachable'
    if not no_reach_over(state, p, cfg):
        return False, 'reach-over'
    st = stability(state, p, box, cfg)
    if not st.ok:
        return False, st.reason
    return True, ''
