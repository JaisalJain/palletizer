"""Pallet-station perception from the overhead pallet camera (no ROS imports; unit-testable).

  detect_pallet  empty pallet -> deck plateau (highest large height plateau in the station
                 area), gaps between deck boards closed, rectangle from second moments of the
                 closed deck (unbiased w.r.t. pixel quantization): dims, centre, yaw, deck z
  height_map     stack height above the deck on the packing grid (pallet frame), NaN where
                 no pixel saw the cell (occluded); per cell the max of the observed points,
                 i.e. the same "any overlap" convention as the placement record
  new_box        placement measurement by CHANGE detection: pixels that rose to the
                 commanded top height since the reference frame. Works when boxes of the same
                 height touch (they merge in any single frame)
  check_shifts   stack monitoring: for every placed box, compare its top-face pixels with the
                 reference; the displaced centroid of its top face = shift estimate
All geometry is measured; the config gives only the camera mounting pose, the station AREA,
tolerances and limits.
"""

from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from palletizer_perception.camera_model import DepthCamera
from palletizer_tools.pallet_frame import PalletFrame


def normalize_axis(yaw, period=math.pi):
    y = math.fmod(yaw, period)
    if y <= -period / 2:
        y += period
    elif y > period / 2:
        y -= period
    return y


@dataclass
class Rect:
    center: np.ndarray        # world xy
    yaw: float                # length axis, in (-pi/2, pi/2]
    length: float
    width: float
    fill: float               # pixel area / rectangle area
    pixels: int

    def corners(self, z):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ex, ey = np.array([c, s]), np.array([-s, c])
        return np.array([np.r_[self.center + a * self.length / 2 * ex + b * self.width / 2 * ey, z]
                         for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))])


@dataclass
class PalletMeasurement:
    frame: PalletFrame
    rect: Rect
    confidence: float
    mask: np.ndarray = None   # closed deck pixels


@dataclass
class BoxRecord:
    """A placed box as perception measured it (the expected stack model)."""
    index: int
    center: np.ndarray        # world xyz of the measured top-face centre
    yaw: float
    length: float
    width: float
    mask: np.ndarray          # top-face pixels
    commanded: np.ndarray     # commanded top-face centre (world xyz)
    sim_name: str = ''
    events: list = field(default_factory=list)


@dataclass
class ShiftEvent:
    index: int
    kind: str                 # SHIFT | COLLAPSE
    shift: float
    dz: float
    new_center: np.ndarray
    message: str = ''


class PalletDetector:

    def __init__(self, cell):
        self.cell = cell
        self.cam = DepthCamera(cell['pallet_camera'])
        self.p = cell['pallet_perception']
        st = cell['pallet_station']
        self.station_c = np.array(st['center_xy'], float)
        self.station_a = np.array(st['area'], float)
        self.max_stack = float(st['max_stack_height'])
        self.res = float(cell['packing']['resolution'])
        self.robot_xy = np.array(cell['robot']['pedestal']['xy'], float)

    # ------------------------------------------------------------------ geometry
    def points(self, depth):
        depth = np.where(np.isfinite(depth), depth, np.nan)
        return self.cam.world_points(depth)

    def in_station(self, P, margin=0.0):
        d = np.abs(P[..., :2] - self.station_c)
        half = self.station_a / 2 + margin
        with np.errstate(invalid='ignore'):
            return (d[..., 0] <= half[0]) & (d[..., 1] <= half[1])

    def occluded(self, P, deck_z=None):
        """True if something hangs over the station higher than any stack can be (robot,
        carried box). Complements the TF check in the node."""
        top = (deck_z if deck_z is not None else self.p['deck_z_range'][1]) + self.max_stack + \
            self.p['occluder_clearance']
        with np.errstate(invalid='ignore'):
            high = self.in_station(P) & (P[..., 2] > top)
        return int(high.sum()) > 20

    @staticmethod
    def flat(Z, max_step):
        """Pixels whose 4-neighbours are within max_step in height: horizontal surfaces.
        Vertical faces seen obliquely (pedestal, box sides) change by cm per pixel."""
        Zp = np.pad(Z, 1, mode='edge')
        with np.errstate(invalid='ignore'):
            steps = np.stack([np.abs(Zp[1:-1, 2:] - Z), np.abs(Zp[1:-1, :-2] - Z),
                              np.abs(Zp[2:, 1:-1] - Z), np.abs(Zp[:-2, 1:-1] - Z)])
            return np.nanmax(steps, axis=0) < max_step

    def _k(self, meters, z):
        return max(1, int(round(meters / self.cam.pixel_pitch(z))))

    def fit_rect(self, mask, z):
        """Rectangle of the pixels in `mask` (rays intersected with the plane at z):
        minAreaRect for the orientation, second moments for the side lengths (unbiased
        w.r.t. pixel quantization), centroid = centre."""
        xy = self.cam.plane_points(mask, z)
        if len(xy) < 20:
            return None
        (_, _), (a, b), ang = cv2.minAreaRect(xy.astype(np.float32))
        yaw = math.radians(ang) + (math.pi / 2 if b > a else 0.0)
        c, s = math.cos(yaw), math.sin(yaw)
        mean = xy.mean(axis=0)
        q = (xy - mean) @ np.array([[c, -s], [s, c]])
        L, W = np.sqrt(12.0 * q.var(axis=0))
        if W > L:
            L, W, yaw = W, L, yaw + math.pi / 2
        pitch = self.cam.pixel_pitch(z)
        fill = len(xy) * pitch * pitch / max(L * W, 1e-9)
        return Rect(mean, normalize_axis(yaw), float(L), float(W), float(fill), len(xy))

    # ------------------------------------------------------------------ pallet
    def _plateaus(self, zv, min_px):
        b, tol = self.p['plateau_bin'], self.p['plateau_tol']
        lo, hi = self.p['deck_z_range']
        hist, edges = np.histogram(zv, bins=np.arange(lo, hi + b, b))
        k = max(1, int(round(tol / b)))
        win = np.convolve(hist, np.ones(2 * k + 1, dtype=int), mode='same')
        centers = (edges[:-1] + edges[1:]) / 2
        peaks = []
        for i in np.argsort(-win):
            if win[i] < min_px:
                break
            if all(abs(centers[i] - c) > 2 * tol for c in peaks):
                peaks.append(centers[i])
        return sorted((float(np.median(zv[np.abs(zv - c) < tol])) for c in peaks), reverse=True)

    def detect_pallet(self, depth):
        """(PalletMeasurement | None, message) from an (averaged) depth image of the EMPTY
        pallet."""
        P = self.points(depth)
        Z = P[..., 2]
        lo, hi = self.p['deck_z_range']
        with np.errstate(invalid='ignore'):
            band = self.in_station(P) & (Z >= lo) & (Z <= hi)
            cand = band & self.flat(Z, 0.005)
        pitch = self.cam.pixel_pitch(hi)
        min_px = int(0.3 * self.p['min_deck_area'] / pitch ** 2)
        levels = self._plateaus(Z[cand], min_px)
        if not levels:
            return None, 'no deck plateau in the station area'
        z = levels[0]
        with np.errstate(invalid='ignore'):
            deck = cand & (np.abs(Z - z) < self.p['plateau_tol'])
        k = self._k(self.p['deck_close'], z) | 1
        closed = cv2.morphologyEx(deck.astype(np.uint8), cv2.MORPH_CLOSE,
                                  cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(closed, 8)
        if n < 2:
            return None, 'no deck component'
        big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        mask = (lab == big).astype(np.uint8)
        # the flatness test drops the outermost deck pixels (their neighbour is the floor):
        # grow back by one pixel onto pixels at deck height
        with np.errstate(invalid='ignore'):
            edge = band & (np.abs(Z - z) < self.p['plateau_tol'])
        mask |= cv2.dilate(mask, np.ones((3, 3), np.uint8)) & edge.astype(np.uint8)
        # fill holes (deck gaps closed from the inside, e.g. at a missing board)
        ff = np.zeros((mask.shape[0] + 2, mask.shape[1] + 2), np.uint8)
        outside = mask.copy()
        cv2.floodFill(outside, ff, (0, 0), 1)
        mask = (mask | (1 - outside)).astype(bool)
        r = self.fit_rect(mask, z)
        if r is None:
            return None, 'deck too small'
        if r.length * r.width < self.p['min_deck_area'] or r.width < self.p['min_side']:
            return None, f'deck {r.length:.2f} x {r.width:.2f} m is not a pallet'
        axis = np.array([math.cos(r.yaw), math.sin(r.yaw)])
        frame = PalletFrame.from_axes(r.center, axis, r.length, r.width, z, self.robot_xy)
        conf = float(np.clip(1.0 - abs(1.0 - r.fill) * 5.0, 0.0, 1.0))
        return PalletMeasurement(frame, r, conf, mask), 'ok'

    # ------------------------------------------------------------------ height map
    def footprint_mask(self, P, frame, margin=0.0):
        uv = frame.to_pallet_array(P[..., :2].reshape(-1, 2)).reshape(P.shape[:2] + (2,))
        with np.errstate(invalid='ignore'):
            return ((uv[..., 0] >= -margin) & (uv[..., 0] < frame.size_u + margin) &
                    (uv[..., 1] >= -margin) & (uv[..., 1] < frame.size_v + margin)), uv

    def height_map(self, P, frame):
        rows = int(round(frame.size_v / self.res))
        cols = int(round(frame.size_u / self.res))
        inside, uv = self.footprint_mask(P, frame)
        m = inside & np.isfinite(P[..., 2])
        iu = np.clip((uv[m][:, 0] / self.res).astype(int), 0, cols - 1)
        iv = np.clip((uv[m][:, 1] / self.res).astype(int), 0, rows - 1)
        h = np.maximum(P[..., 2][m] - frame.deck_z, 0.0)     # deck gaps read below the deck
        H = np.full(rows * cols, -np.inf)
        np.maximum.at(H, iv * cols + iu, h)
        H[~np.isfinite(H)] = np.nan
        return H.reshape(rows, cols).astype(np.float32)

    # ------------------------------------------------------------------ placements
    def new_box(self, Z_ref, Z_now, P_now, target_xyz, target_yaw, size_xy, height):
        """Top face of a newly placed box: pixels near the target that now lie at the
        commanded top height and were clearly lower in the reference frame. (Not "rose by
        half the box height": pixels that saw the upper part of a neighbour's side wall
        before rose by less.)
        Returns (Rect, z_top, mask) or (None, message)."""
        tol = self.p['new_box_tol']
        zt = float(target_xyz[2])
        reach = max(size_xy) / 2 + 0.10
        with np.errstate(invalid='ignore'):
            near = np.hypot(P_now[..., 0] - target_xyz[0], P_now[..., 1] - target_xyz[1]) < reach
            was_lower = Z_ref < zt - 2 * self.p['plateau_tol']
            cand = near & (np.abs(Z_now - zt) < tol) & was_lower
        k = self._k(0.006, zt)
        cand = cv2.morphologyEx(cand.astype(np.uint8), cv2.MORPH_OPEN,
                                np.ones((k, k), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(cand, 8)
        if n < 2:
            return None, 'no new top face near the target'
        big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        mask = lab == big
        z = float(np.median(Z_now[mask]))
        # top face only: side-wall pixels seen obliquely near the top edge would bias the
        # centroid outward (they change height by cm per pixel)
        with np.errstate(invalid='ignore'):
            mask &= (np.abs(Z_now - z) < self.p['plateau_tol']) & self.flat(Z_now, 0.005)
        # the flatness test drops the outermost ring: grow back onto top-height pixels
        with np.errstate(invalid='ignore'):
            ring = near & (np.abs(Z_now - z) < self.p['plateau_tol']) & was_lower
        mask |= cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool) & ring
        z = float(np.median(Z_now[mask]))
        r = self.fit_rect(mask, z)
        if r is None:
            return None, 'new top face too small'
        if r.fill < self.p['min_new_box_fill']:
            return None, f'new top face not rectangular (fill {r.fill:.2f}: occluded?)'
        return (r, z, mask), 'ok'

    # ------------------------------------------------------------------ monitoring
    def changed(self, Z_ref, Z_now, exclude=None):
        thr = self.p['change_threshold']
        with np.errstate(invalid='ignore'):
            D = Z_now - Z_ref
            ch = np.isfinite(D) & (np.abs(D) > thr)
        ch = cv2.morphologyEx(ch.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        ch = ch.astype(bool)
        if exclude is not None:
            ch &= ~exclude
        return D, ch

    def refresh_masks(self, Z_ref, records):
        """Keep in each record only the pixels that still show its top face (a box placed on
        top, or seen in front of it, hides part of it)."""
        tol = self.p['plateau_tol']
        for rec in records:
            with np.errstate(invalid='ignore'):
                rec.mask &= np.abs(Z_ref - rec.center[2]) < tol

    def check_shifts(self, Z_ref, Z_now, records, exclude=None):
        """Stack monitoring. For every placed box whose surroundings changed, compare its
        visible top-face pixels: now = (old ones that did not drop or get covered) + (pixels
        next to it that rose to its top height); shift = displacement of their centroid.
        Most of the visible top dropped -> COLLAPSE."""
        D, ch = self.changed(Z_ref, Z_now, exclude)
        thr, tol = self.p['change_threshold'], self.p['new_box_tol']
        events = []
        if not ch.any():
            return events
        keep = ~exclude if exclude is not None else np.ones(D.shape, bool)
        for rec in records:
            z = float(rec.center[2])
            pitch = self.cam.pixel_pitch(z)
            k = self._k(0.06, z) * 2 + 1
            near = cv2.dilate(rec.mask.astype(np.uint8), np.ones((k, k), np.uint8)).astype(bool)
            if (ch & near).sum() * pitch * pitch < self.p['min_change_area']:
                continue
            with np.errstate(invalid='ignore'):
                valid = rec.mask & keep & np.isfinite(D)
                vanished = valid & (D < -thr)
                covered = valid & (D > thr)
                appeared = near & keep & ~rec.mask & (np.abs(Z_now - z) < tol) & (D > thr)
            seen = valid & ~covered
            if seen.sum() < 0.2 * max(rec.mask.sum(), 1):
                continue                        # hidden (covered / excluded): cannot tell
            if vanished.sum() > 0.5 * seen.sum():
                dz = float(np.median(D[vanished]))
                events.append(ShiftEvent(rec.index, 'COLLAPSE', float('nan'), dz, rec.center,
                                         f'top face of box {rec.index} dropped '
                                         f'{-dz * 1000:.0f} mm'))
                continue
            new = (seen & ~vanished) | appeared
            c_old = self.cam.plane_points(seen, z).mean(axis=0)
            c_new = self.cam.plane_points(new, z).mean(axis=0)
            shift = float(np.linalg.norm(c_new - c_old))
            if shift > self.p['shift_threshold']:
                events.append(ShiftEvent(rec.index, 'SHIFT', shift, 0.0,
                                         np.r_[rec.center[:2] + c_new - c_old, z],
                                         f'box {rec.index} shifted {shift * 1000:.0f} mm'))
                rec.mask = (rec.mask & ~vanished) | appeared
        return events
