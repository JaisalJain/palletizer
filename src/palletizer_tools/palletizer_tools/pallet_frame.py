"""Pallet frame <-> world (no ROS). The frame comes from the MEASURED pallet.

Pallet frame: origin at the deck corner nearest the robot on the -u side, v = pallet axis
pointing away from the robot, u = v rotated by +90 deg about z (for a pallet in front of the
robot at -y this is u = +x, v = -y, as in v1), z = height above the deck top.
"""

from dataclasses import dataclass
import math

import numpy as np


def rot90(v):
    return np.array([-v[1], v[0]])


@dataclass
class PalletFrame:
    origin: np.ndarray          # world xy of the frame origin
    yaw: float                  # world yaw of the u axis
    size_u: float
    size_v: float
    deck_z: float

    @property
    def u_axis(self):
        return np.array([math.cos(self.yaw), math.sin(self.yaw)])

    @property
    def v_axis(self):
        # u = rot90(v)  <=>  v = -rot90(u)
        return -rot90(self.u_axis)

    @property
    def center(self):
        return self.origin + self.u_axis * self.size_u / 2 + self.v_axis * self.size_v / 2

    def to_world(self, u, v, h=0.0):
        xy = self.origin + self.u_axis * u + self.v_axis * v
        return np.array([xy[0], xy[1], self.deck_z + h])

    def to_pallet(self, x, y, z=None):
        d = np.array([x, y]) - self.origin
        u, v = float(d @ self.u_axis), float(d @ self.v_axis)
        return (u, v) if z is None else (u, v, float(z - self.deck_z))

    def to_pallet_array(self, xy):
        """(N, 2) world xy -> (N, 2) pallet uv."""
        d = np.asarray(xy) - self.origin
        return np.c_[d @ self.u_axis, d @ self.v_axis]

    def box_yaw(self, rotated):
        """World yaw of a box length axis: along u, or along v when rotated."""
        return self.yaw + (math.pi / 2 if rotated else 0.0)

    def corners(self):
        return [self.to_world(u, v) for u, v in
                ((0, 0), (self.size_u, 0), (self.size_u, self.size_v), (0, self.size_v))]

    @classmethod
    def from_axes(cls, center_xy, axis_a, len_a, len_b, deck_z, robot_xy):
        """From a measured rectangle: centre, direction of side `a` (length len_a), other
        side len_b. v = the axis pointing away from the robot."""
        a = np.asarray(axis_a, float) / np.linalg.norm(axis_a)
        b = rot90(a)
        away = np.asarray(center_xy, float) - np.asarray(robot_xy, float)
        if abs(a @ away) > abs(b @ away):
            v, size_v, size_u = a, len_a, len_b
        else:
            v, size_v, size_u = b, len_b, len_a
        if v @ away < 0:
            v = -v
        u = rot90(v)
        origin = np.asarray(center_xy, float) - u * size_u / 2 - v * size_v / 2
        return cls(origin, math.atan2(u[1], u[0]), size_u, size_v, deck_z)

    @classmethod
    def from_msg(cls, m):
        """From a palletizer_interfaces/DetectedPallet."""
        return cls(np.array([m.origin.position.x, m.origin.position.y]), m.yaw, m.size_u,
                   m.size_v, m.deck_z)

    def fill_msg(self, m):
        m.size_u, m.size_v, m.deck_z, m.yaw = self.size_u, self.size_v, self.deck_z, self.yaw
        c = self.center
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        for pose, xy in ((m.center, c), (m.origin, self.origin)):
            pose.position.x, pose.position.y, pose.position.z = float(xy[0]), float(xy[1]), \
                float(self.deck_z)
            pose.orientation.z, pose.orientation.w = qz, qw
        return m
