"""Box detection from an overhead depth image (no ROS imports; unit-testable).

Pipeline:
  1. back-project every pixel to world XYZ (camera pose from cell_config, depth = optical z)
  2. ROI on the belt, subtract the belt plane -> height above belt
  3. top faces = connected regions on height plateaus (histogram peaks), so touching boxes of
     different heights separate naturally; merged equal-height boxes are split by RGB colour
  4. each top face: pixel rays intersected with the plateau plane -> metric XY points,
     minAreaRect -> orientation, second moments in that frame -> length/width (unbiased
     w.r.t. pixel quantization), centroid -> grasp point
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

# REP-103 optical frame relative to a gz camera link (x forward): rpy(-pi/2, 0, -pi/2)
R_LINK_OPT = Rotation.from_euler('xyz', [-math.pi / 2, 0.0, -math.pi / 2]).as_matrix()


@dataclass
class TopFace:
    length: float
    width: float
    height: float
    center: np.ndarray        # world xyz of the top-face center
    yaw: float                # length axis, in (-pi/2, pi/2]
    area: float               # measured top area [m^2]
    fill_ratio: float
    pixels: int
    corners: np.ndarray       # 4x3 world corners of the fitted rectangle
    confidence: float
    note: str = ''
    mask: np.ndarray = None   # top-face pixels (debug view)


def normalize_yaw(yaw):
    """Wrap an axis direction (pi-periodic) to (-pi/2, pi/2]."""
    y = math.fmod(yaw, math.pi)
    if y <= -math.pi / 2:
        y += math.pi
    elif y > math.pi / 2:
        y -= math.pi
    return y


class BoxDetector:

    def __init__(self, cell):
        self.cell = cell
        cam = cell['camera']
        self.cam_pos = np.array(cam['xyz'], dtype=float)
        self.R_world_opt = Rotation.from_euler('xyz', cam['rpy']).as_matrix() @ R_LINK_OPT
        self.p = cell['perception']
        cv = cell['conveyor']
        self.belt_z = cv['belt_top_z']
        self.roi_x = cell['pick_zone']['roi_x']
        self.roi_y = (cv['y'] - cv['belt_width'] / 2 - self.p['roi_margin_y'],
                      cv['y'] + cv['belt_width'] / 2 + self.p['roi_margin_y'])
        self.pick_eye_x = [z for z in cv['zones'] if z['name'] == 'pick'][0]['eye_x']
        self._rays = None
        self._K = None

    # ------------------------------------------------------------------ geometry
    def set_intrinsics(self, K, width, height):
        K = np.asarray(K, dtype=float).reshape(3, 3)
        if self._K is not None and np.allclose(K, self._K) and self._rays.shape[:2] == (height, width):
            return
        self._K = K
        off = float(self.cell['camera'].get('pixel_center_offset', 0.0))
        u, v = np.meshgrid(np.arange(width) + off, np.arange(height) + off)
        xn = (u - K[0, 2]) / K[0, 0]
        yn = (v - K[1, 2]) / K[1, 1]
        rays_opt = np.dstack([xn, yn, np.ones_like(xn)])       # z_opt = 1
        self._rays = rays_opt @ self.R_world_opt.T              # world direction, scaled so z_opt = 1

    def world_points(self, depth):
        return self.cam_pos + self._rays * depth[..., None]

    def plane_points(self, mask, z):
        """World XY of pixel rays intersected with the horizontal plane at z."""
        r = self._rays[mask]
        t = (z - self.cam_pos[2]) / r[:, 2]
        return self.cam_pos[None, :2] + r[:, :2] * t[:, None]

    def pixel_pitch(self, z):
        return (self.cam_pos[2] - z) / self._K[0, 0]

    def project(self, xyz):
        """World points (N, 3) -> pixel coordinates (N, 2), OpenCV convention."""
        p = (np.asarray(xyz, float).reshape(-1, 3) - self.cam_pos) @ self.R_world_opt
        uv = (p[:, :2] / p[:, 2:3]) @ self._K[:2, :2].T + self._K[:2, 2]
        return uv - float(self.cell['camera'].get('pixel_center_offset', 0.0))

    def occluded(self, depth):
        """Something higher than any box over the pick ROI (e.g. the forearm): the view of
        the box may be blocked."""
        with np.errstate(invalid='ignore'):
            P = self.world_points(np.where(np.isfinite(depth), depth, np.nan))
            roi = ((P[..., 0] >= self.roi_x[0]) & (P[..., 0] <= self.roi_x[1]) &
                   (P[..., 1] >= self.roi_y[0]) & (P[..., 1] <= self.roi_y[1]))
            high = roi & (P[..., 2] - self.belt_z > self.p['max_object_height'] +
                          self.p['occluder_clearance'])
        return int(high.sum()) > 20

    # ------------------------------------------------------------------ detection
    def detect(self, depth, rgb=None):
        """Returns (faces sorted downstream-first, debug dict). depth in meters (HxW)."""
        depth = np.where(np.isfinite(depth), depth, np.nan)
        P = self.world_points(depth)
        h = P[..., 2] - self.belt_z
        roi = ((P[..., 0] >= self.roi_x[0]) & (P[..., 0] <= self.roi_x[1]) &
               (P[..., 1] >= self.roi_y[0]) & (P[..., 1] <= self.roi_y[1]))
        obj = roi & np.isfinite(h) & (h > self.p['min_object_height']) & \
            (h < self.p['max_object_height'])

        faces = []
        used = np.zeros_like(obj)
        for level in self._plateaus(h[obj]):
            top = obj & ~used & (np.abs(h - level) < self.p['plateau_tol'])
            n, labels, stats, _ = cv2.connectedComponentsWithStats(top.astype(np.uint8), 8)
            for i in range(1, n):
                comp = labels == i
                z = self.belt_z + float(np.median(h[comp]))
                area = comp.sum() * self.pixel_pitch(z) ** 2
                if area < self.p['min_top_area']:
                    continue
                used |= comp
                faces.extend(self._faces_from_component(comp, z, rgb))
        faces.sort(key=lambda f: -self._front_x(f))
        return faces, {'height': h, 'roi': roi, 'object': obj, 'used': used}

    def _plateaus(self, hv):
        """Plateau heights (descending): histogram peaks with enough pixels."""
        if hv.size == 0:
            return []
        b = self.p['plateau_bin']
        bins = np.arange(self.p['min_object_height'], self.p['max_object_height'] + b, b)
        hist, edges = np.histogram(hv, bins=bins)
        # count pixels within +/- tol of each bin center
        k = max(1, int(round(self.p['plateau_tol'] / b)))
        win = np.convolve(hist, np.ones(2 * k + 1, dtype=int), mode='same')
        centers = (edges[:-1] + edges[1:]) / 2
        min_px = 1500
        peaks = []
        for i in np.argsort(-win):
            if win[i] < min_px:
                break
            if all(abs(centers[i] - c) > 2 * self.p['plateau_tol'] for c in peaks):
                peaks.append(centers[i])
        # refine each peak to the mean height of its pixels
        refined = []
        for c in peaks:
            sel = hv[np.abs(hv - c) < self.p['plateau_tol']]
            refined.append(float(np.median(sel)))
        return sorted(refined, reverse=True)

    def _faces_from_component(self, comp, z, rgb):
        face = self._fit(comp, z)
        if face.fill_ratio >= self.p['min_fill_ratio'] and face.length <= self.p['max_side']:
            return [face]
        # Probably several equal-height boxes touching: split by colour.
        if rgb is not None:
            parts = self._color_split(comp, rgb)
            if parts:
                out = []
                for part in parts:
                    f = self._fit(part, z)
                    f.note = 'color_split'
                    out.append(f)
                if all(f.fill_ratio >= self.p['min_fill_ratio'] and f.length <= self.p['max_side']
                       for f in out):
                    return out
        face.confidence = min(face.confidence, 0.2)
        face.note = 'merged_or_irregular'
        return [face]

    def _color_split(self, comp, rgb):
        pix = rgb[comp].reshape(-1, 3).astype(np.float32)
        if len(pix) < 200:
            return None
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.5)
        _, lab, centers = cv2.kmeans(pix, 2, None, crit, 3, cv2.KMEANS_PP_CENTERS)
        if np.linalg.norm(centers[0] - centers[1]) < self.p['color_split_min_dist']:
            return None
        parts = []
        idx = np.flatnonzero(comp.ravel())
        for k in range(2):
            m = np.zeros(comp.size, dtype=np.uint8)
            m[idx[lab.ravel() == k]] = 1
            m = cv2.morphologyEx(m.reshape(comp.shape), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            n, labels, stats, _ = cv2.connectedComponentsWithStats(m, 8)
            if n < 2:
                return None
            big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            parts.append(labels == big)
        return parts

    def _fit(self, comp, z):
        xy = self.plane_points(comp, z)
        (cx, cy), (a, b), ang = cv2.minAreaRect(xy.astype(np.float32))
        yaw = math.radians(ang)
        if b > a:
            yaw += math.pi / 2
        c, s = math.cos(yaw), math.sin(yaw)
        mean = xy.mean(axis=0)
        q = (xy - mean) @ np.array([[c, -s], [s, c]])         # columns: length axis, width axis
        pitch = self.pixel_pitch(z)
        dims = np.sqrt(12.0 * q.var(axis=0))
        # centre from the extents (robust to a ragged edge) blended with the centroid
        length, width = float(dims[0]), float(dims[1])
        if width > length:
            length, width = width, length
            yaw += math.pi / 2
        yaw = normalize_yaw(yaw)
        area = len(xy) * pitch * pitch
        fill = area / max(length * width, 1e-9)
        c, s = math.cos(yaw), math.sin(yaw)
        ex, ey = np.array([c, s]), np.array([-s, c])
        corners = np.array([mean + sx * length / 2 * ex + sy * width / 2 * ey
                            for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))])
        conf = float(np.clip(1.0 - abs(1.0 - fill) * 5.0, 0.0, 1.0))
        return TopFace(
            length=length, width=width, height=z - self.belt_z,
            center=np.array([mean[0], mean[1], z]), yaw=yaw, area=area, fill_ratio=fill,
            pixels=int(comp.sum()), corners=np.c_[corners, np.full(4, z)], confidence=conf,
            mask=comp)

    @staticmethod
    def _front_x(face):
        return float(face.corners[:, 0].max())

    def pick_candidate(self, faces):
        """The box held at the pick eye: the most downstream confident face."""
        good = [f for f in faces if f.confidence >= 0.5]
        return good[0] if good else None
