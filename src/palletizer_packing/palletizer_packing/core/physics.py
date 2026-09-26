"""PyBullet stability check (headless). The only module that imports pybullet.

Rebuilds the pallet and all placed boxes at their commanded poses, adds the candidate box
slightly above its resting height, simulates `sim_time` seconds and accepts the candidate
only if no box moved more than `max_displacement` or rotated more than `max_rotation_deg`.
"""

from dataclasses import dataclass
import math

import numpy as np
import pybullet as pb

from palletizer_packing.core.pallet import Box, PalletState, Placement


@dataclass
class PhysicsConfig:
    sim_time: float = 1.5
    timestep: float = 1.0 / 240.0
    max_displacement: float = 0.010
    max_rotation_deg: float = 2.0
    friction: float = 0.5
    drop_height: float = 0.002
    default_mass: float = 5.0


def physics_config(cell):
    ph = cell['packing']['physics']
    return PhysicsConfig(ph['sim_time'], ph['timestep'], ph['max_displacement'],
                         ph['max_rotation_deg'], ph['friction'], ph['drop_height'])


class PhysicsChecker:

    def __init__(self, cfg: PhysicsConfig):
        self.cfg = cfg
        self.client = pb.connect(pb.DIRECT)
        self.last = {}

    def close(self):
        if self.client is not None:
            pb.disconnect(self.client)
            self.client = None

    def _box(self, half, pos, mass):
        c = self.client
        col = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=half, physicsClientId=c)
        body = pb.createMultiBody(mass, col, basePosition=pos, physicsClientId=c)
        pb.changeDynamics(body, -1, lateralFriction=self.cfg.friction, physicsClientId=c)
        return body

    def stable(self, state: PalletState, candidate: Placement, box: Box):
        return self.stable_many(state, [candidate], box)

    def stable_many(self, state: PalletState, candidates, box: Box):
        """Add several boxes (e.g. a whole layer) at once; stable if nothing moves."""
        c = self.client
        pb.resetSimulation(physicsClientId=c)
        pb.setGravity(0, 0, -9.81, physicsClientId=c)
        pb.setTimeStep(self.cfg.timestep, physicsClientId=c)
        s = state.spec
        # pallet deck: static box whose top is z = 0
        self._box([s.length / 2 + 0.2, s.width / 2 + 0.2, 0.05],
                  [s.length / 2, s.width / 2, -0.05], 0.0)
        bodies = []
        for b in state.boxes:
            p = b.placement
            u, v = p.center
            bodies.append(self._box([p.du / 2, p.dv / 2, p.height / 2],
                                    [u, v, p.z0 + p.height / 2],
                                    b.box.mass if b.box.mass > 0 else self.cfg.default_mass))
        n_old = len(bodies)
        for cand in candidates:
            u, v = cand.center
            bodies.append(self._box([cand.du / 2, cand.dv / 2, cand.height / 2],
                                    [u, v, cand.z0 + cand.height / 2 + self.cfg.drop_height],
                                    box.mass if box.mass > 0 else self.cfg.default_mass))
        start = [pb.getBasePositionAndOrientation(b, physicsClientId=c) for b in bodies]
        for _ in range(int(round(self.cfg.sim_time / self.cfg.timestep))):
            pb.stepSimulation(physicsClientId=c)
        worst_d, worst_r = 0.0, 0.0
        for b, (p0, q0) in zip(bodies, start):
            p1, q1 = pb.getBasePositionAndOrientation(b, physicsClientId=c)
            d = float(np.linalg.norm(np.array(p1) - np.array(p0)))
            if bodies.index(b) >= n_old:
                d = max(0.0, d - self.cfg.drop_height)     # settling from the drop height
            dq = pb.getDifferenceQuaternion(q0, q1, physicsClientId=c)
            ang = 2.0 * math.degrees(math.acos(min(1.0, abs(dq[3]))))
            worst_d, worst_r = max(worst_d, d), max(worst_r, ang)
        self.last = {'max_displacement': worst_d, 'max_rotation_deg': worst_r}
        return worst_d <= self.cfg.max_displacement and worst_r <= self.cfg.max_rotation_deg
