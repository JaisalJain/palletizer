"""Pinhole depth camera mounted at a known pose (config: mounting pose only). No ROS."""

import math

import numpy as np
from scipy.spatial.transform import Rotation

# REP-103 optical frame relative to a gz camera link (x forward): rpy(-pi/2, 0, -pi/2)
R_LINK_OPT = Rotation.from_euler('xyz', [-math.pi / 2, 0.0, -math.pi / 2]).as_matrix()


class DepthCamera:

    def __init__(self, cam_cfg):
        self.cfg = cam_cfg
        self.pos = np.array(cam_cfg['xyz'], dtype=float)
        self.R_world_opt = Rotation.from_euler('xyz', cam_cfg['rpy']).as_matrix() @ R_LINK_OPT
        self.offset = float(cam_cfg.get('pixel_center_offset', 0.0))
        self.K = None
        self.rays = None           # world ray directions per pixel, scaled so z_opt = 1

    def set_intrinsics(self, K, width, height):
        K = np.asarray(K, dtype=float).reshape(3, 3)
        if self.K is not None and np.allclose(K, self.K) and self.rays.shape[:2] == (height, width):
            return
        self.K = K
        u, v = np.meshgrid(np.arange(width) + self.offset, np.arange(height) + self.offset)
        rays = np.dstack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones(u.shape)])
        self.rays = rays @ self.R_world_opt.T

    @property
    def shape(self):
        return self.rays.shape[:2]

    def world_points(self, depth):
        return self.pos + self.rays * depth[..., None]

    def plane_points(self, mask, z):
        """World XY of the pixel rays in `mask` intersected with the plane at height z."""
        r = self.rays[mask]
        t = (z - self.pos[2]) / r[:, 2]
        return self.pos[None, :2] + r[:, :2] * t[:, None]

    def pixel_pitch(self, z):
        return abs(self.pos[2] - z) / self.K[0, 0]

    def project(self, xyz):
        """World points (N, 3) -> pixel coordinates (N, 2) (OpenCV convention, pixel centres
        at integers; the gz half-pixel offset is removed)."""
        p = (np.asarray(xyz, float).reshape(-1, 3) - self.pos) @ self.R_world_opt
        uv = (p[:, :2] / p[:, 2:3]) @ self.K[:2, :2].T + self.K[:2, 2]
        return uv - self.offset
