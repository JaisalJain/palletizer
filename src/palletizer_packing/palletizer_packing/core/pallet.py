"""Pallet model: 1 cm height map plus a record of placed boxes. No ROS imports.

Pallet frame: u along the pallet length (0..L, world +x), v from the robot-side edge into the
pallet (0..W, world -y), z above the pallet top surface. Grid arrays are indexed [iv, iu].
"""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass(frozen=True)
class Box:
    """A box to place: measured top-face dims (length >= width), height and mass (<=0: unknown)."""
    length: float
    width: float
    height: float
    mass: float = 0.0

    @property
    def volume(self):
        return self.length * self.width * self.height


@dataclass
class Placement:
    """Box footprint min corner (u0, v0), footprint size along u/v, resting height."""
    u0: float
    v0: float
    du: float
    dv: float
    z0: float
    height: float
    rotated: bool                    # True: length along v
    score: float = 0.0
    info: dict = field(default_factory=dict)

    @property
    def center(self):
        return self.u0 + self.du / 2, self.v0 + self.dv / 2

    @property
    def top(self):
        return self.z0 + self.height


@dataclass
class PlacedBox:
    id: int
    placement: Placement
    box: Box
    name: str = ''


@dataclass
class PalletSpec:
    length: float = 1.2
    width: float = 0.8
    resolution: float = 0.01
    max_height: float = 1.05         # max box-top height above the pallet top

    @property
    def shape(self):
        return (int(round(self.width / self.resolution)), int(round(self.length / self.resolution)))

    @classmethod
    def from_config(cls, cell, usable_height=None):
        pal, pk = cell['pallet'], cell['packing']
        max_h = pal['max_stack_height']
        if usable_height is not None:
            max_h = min(max_h, usable_height)
        return cls(pal['size'][0], pal['size'][1], pk['resolution'], max_h)


class PalletState:

    def __init__(self, spec: PalletSpec):
        self.spec = spec
        self.height = np.zeros(spec.shape)                 # top height per cell [m]
        self.owner = -np.ones(spec.shape, dtype=int)        # id of the top box per cell (-1: pallet)
        self.boxes: list[PlacedBox] = []

    def copy(self):
        s = PalletState(self.spec)
        s.height = self.height.copy()
        s.owner = self.owner.copy()
        s.boxes = list(self.boxes)
        return s

    # ------------------------------------------------------------------ grid helpers
    def cells(self, u0, v0, du, dv):
        """Index ranges of the cells covered by a footprint (any overlap counts)."""
        r = self.spec.resolution
        iu0 = max(0, int(math.floor(u0 / r + 1e-6)))
        iu1 = min(self.height.shape[1], int(math.ceil((u0 + du) / r - 1e-6)))
        iv0 = max(0, int(math.floor(v0 / r + 1e-6)))
        iv1 = min(self.height.shape[0], int(math.ceil((v0 + dv) / r - 1e-6)))
        return slice(iv0, iv1), slice(iu0, iu1)

    def rest_height(self, u0, v0, du, dv):
        sv, su = self.cells(u0, v0, du, dv)
        region = self.height[sv, su]
        return float(region.max()) if region.size else 0.0

    # ------------------------------------------------------------------ updates
    def place(self, placement: Placement, box: Box, name=''):
        pb = PlacedBox(len(self.boxes), placement, box, name)
        sv, su = self.cells(placement.u0, placement.v0, placement.du, placement.dv)
        self.height[sv, su] = placement.top
        self.owner[sv, su] = pb.id
        self.boxes.append(pb)
        return pb

    # ------------------------------------------------------------------ metrics
    @property
    def max_height(self):
        return float(self.height.max())

    @property
    def box_volume(self):
        return sum(b.box.volume for b in self.boxes)

    def fill_percent(self):
        s = self.spec
        return 100.0 * self.box_volume / (s.length * s.width * s.max_height)

    def roughness(self, height=None):
        h = self.height if height is None else height
        return float(np.abs(np.diff(h, axis=0)).sum() + np.abs(np.diff(h, axis=1)).sum()) * \
            self.spec.resolution
