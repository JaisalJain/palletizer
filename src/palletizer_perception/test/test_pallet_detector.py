"""Pallet detector on synthetic depth images (no ROS, no sim).
Run: python -m pytest src/palletizer_perception/test

A small ray caster renders the pallet camera's depth image of a pallet (deck boards with
gaps) and boxes (rotated about z) on the floor, with the camera model of cell_config.
"""

import math
import os
import sys

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
for pkg in ('palletizer_perception', 'palletizer_tools'):
    sys.path.insert(0, os.path.join(SRC, pkg))

from palletizer_perception.pallet_detector import PalletDetector  # noqa: E402
from palletizer_tools.pallet_frame import PalletFrame  # noqa: E402

CONFIG = os.path.join(SRC, 'palletizer_bringup', 'config', 'cell_config.yaml')
W_IMG, H_IMG = 800, 600


@pytest.fixture(scope='module')
def det():
    cell = yaml.safe_load(open(CONFIG))
    for k in ('spawner', 'exchanger'):
        cell.pop(k)
    d = PalletDetector(cell)
    f = W_IMG / 2 / math.tan(cell['pallet_camera']['hfov'] / 2)
    d.cam.set_intrinsics([f, 0, W_IMG / 2, 0, f, H_IMG / 2, 0, 0, 1], W_IMG, H_IMG)
    return d


def boxes_of_pallet(cx, cy, yaw, L, W, H):
    """Deck boards (5 across the width, 0.1 m, gaps) + a solid lower block."""
    out = [((cx, cy), yaw, (L, W), (0.0, H - 0.022))]           # (the gaps see this)
    c, s = math.cos(yaw), math.sin(yaw)
    for i in range(5):
        off = -W / 2 + 0.05 + i * (W - 0.1) / 4
        out.append(((cx - s * off, cy + c * off), yaw, (L, 0.1), (H - 0.022, H)))
    return out


def render(det, solids, noise=0.0, seed=0):
    """Depth (optical z) of the nearest hit: floor z = 0 and solids
    ((x, y), yaw, (sx, sy), (z0, z1))."""
    cam = det.cam
    rays = cam.rays                              # world direction per pixel, z_opt = 1
    o = cam.pos
    t_best = np.where(rays[..., 2] < 0, -o[2] / rays[..., 2], np.inf)
    for (x, y), yaw, (sx, sy), (z0, z1) in solids:
        c, s = math.cos(yaw), math.sin(yaw)
        # ray in the box frame
        ox, oy = o[0] - x, o[1] - y
        lo = np.array([c * ox + s * oy, -s * ox + c * oy, o[2]])
        d = np.stack([c * rays[..., 0] + s * rays[..., 1], -s * rays[..., 0] + c * rays[..., 1],
                      rays[..., 2]], -1)
        lo_b = np.array([-sx / 2, -sy / 2, z0])
        hi_b = np.array([sx / 2, sy / 2, z1])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lo_b - lo) / d
            t2 = (hi_b - lo) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= tmin) & (tmax > 0)
        t_best = np.where(hit & (tmin < t_best), tmin, t_best)
    depth = t_best.astype(np.float32)             # optical z (rays are scaled to z_opt = 1)
    if noise:
        depth += np.random.default_rng(seed).normal(0, noise, depth.shape).astype(np.float32)
    return depth


def place(frame, u, v, du, dv, h, z0, yaw_off=0.0):
    x, y, _ = frame.to_world(u + du / 2, v + dv / 2)
    return ((x, y), frame.yaw + yaw_off, (du, dv), (frame.deck_z + z0, frame.deck_z + z0 + h))


@pytest.fixture(scope='module')
def pallet(det):
    solids = boxes_of_pallet(0.012, -0.588, math.radians(0.8), 1.2, 0.8, 0.144)
    depth = render(det, solids, noise=0.001)
    m, msg = det.detect_pallet(depth)
    assert m is not None, msg
    return solids, m.frame


def test_pallet_dims_and_pose(det, pallet):
    _, f = pallet
    assert f.size_u == pytest.approx(1.2, abs=0.005)
    assert f.size_v == pytest.approx(0.8, abs=0.005)
    assert np.allclose(f.center, [0.012, -0.588], atol=0.002)
    assert math.degrees(abs(f.yaw - math.radians(0.8))) < 0.2
    assert f.deck_z == pytest.approx(0.144, abs=0.002)
    # v points away from the robot, origin at the robot-side corner
    assert f.v_axis[1] < -0.99 and f.origin[1] > f.center[1]


def test_height_map(det, pallet):
    solids, f = pallet
    box = place(f, 0.1, 0.2, 0.4, 0.3, 0.3, 0.0)
    depth = render(det, solids + [box], noise=0.001)
    H = det.height_map(det.points(depth), f)
    assert H.shape == (80, 120)
    inner = H[22:48, 12:48]
    assert np.nanmax(np.abs(inner - 0.3)) < 0.01
    assert np.nanmax(np.abs(H[60:75, 80:110])) < 0.01          # empty deck


def test_new_box_next_to_equal_height_box(det, pallet):
    """Change detection separates a new box from an equal-height neighbour 5 mm away."""
    solids, f = pallet
    a = place(f, 0.004, 0.40, 0.394, 0.294, 0.3, 0.0)
    before = render(det, solids + [a], noise=0.001, seed=1)
    b = place(f, 0.403, 0.40, 0.394, 0.294, 0.3, 0.0)
    after = render(det, solids + [a, b], noise=0.001, seed=2)
    Zr, P = det.points(before)[..., 2], det.points(after)
    tgt = f.to_world(0.403 + 0.197, 0.40 + 0.147, 0.3)
    found, msg = det.new_box(Zr, P[..., 2], P, tgt, f.yaw, (0.394, 0.294), 0.3)
    assert found is not None, msg
    r, z, _ = found
    assert np.linalg.norm(r.center - tgt[:2]) < 0.0015
    assert abs(r.length - 0.394) < 0.004 and abs(r.width - 0.294) < 0.004
    assert abs(z - tgt[2]) < 0.002


def _records(det, f, solids, placed, seed=0):
    """Place boxes one by one like the robot; returns records + the scene."""
    recs, scene = [], list(solids)
    for i, (u, v, du, dv, h, z0) in enumerate(placed):
        before = det.points(render(det, scene, 0.001, seed + 2 * i))[..., 2]
        box = place(f, u, v, du, dv, h, z0)
        scene.append(box)
        P = det.points(render(det, scene, 0.001, seed + 2 * i + 1))
        tgt = f.to_world(u + du / 2, v + dv / 2, z0 + h)
        (r, z, mask), _ = det.new_box(before, P[..., 2], P, tgt, f.yaw, (du, dv), h)
        recs.append(det.record(i, np.r_[r.center, z], r.yaw, r.length, r.width, mask, tgt,
                               P[..., 2]))
    return recs, scene


LAYER = [(0.004, 0.45, 0.394, 0.294, 0.3, 0.0), (0.403, 0.45, 0.394, 0.294, 0.3, 0.0),
         (0.004, 0.10, 0.394, 0.294, 0.3, 0.0), (0.403, 0.10, 0.394, 0.294, 0.3, 0.0),
         (0.02, 0.30, 0.35, 0.30, 0.25, 0.3)]       # on boxes 0 and 2 (across the row gap)


def test_no_events_when_nothing_moves(det, pallet):
    solids, f = pallet
    recs, scene = _records(det, f, solids, LAYER)
    Z = det.points(render(det, scene, 0.001, 99))[..., 2]
    assert det.check_shifts(Z, recs) == []


@pytest.mark.parametrize('k, du, dv, expect', [(4, 0.03, 0.0, True), (4, 0.012, 0.0, False),
                                               (3, 0.0, -0.035, True)])
def test_shift_detected(det, pallet, k, du, dv, expect):
    solids, f = pallet
    recs, scene = _records(det, f, solids, LAYER)
    (x, y), yaw, size, zz = scene[len(solids) + k]
    d = f.u_axis * du + f.v_axis * dv
    scene[len(solids) + k] = ((x + d[0], y + d[1]), yaw, size, zz)
    Z = det.points(render(det, scene, 0.001, 7))[..., 2]
    ev = det.check_shifts(Z, recs)
    mine = [e for e in ev if e.index == k]
    assert bool(mine) == expect and all(e.index == k for e in ev)
    if expect:
        assert mine[0].shift == pytest.approx(math.hypot(du, dv), abs=0.004)


def test_creep_accumulates_from_placement(det, pallet):
    """Two 12 mm moves (each below the 20 mm threshold) with a placement in between:
    the box is 24 mm from where it was placed -> SHIFT."""
    solids, f = pallet
    recs, scene = _records(det, f, solids, LAYER[:4])
    i = len(solids) + 3
    (x, y), yaw, size, zz = scene[i]
    scene[i] = ((x + 0.012 * f.u_axis[0], y + 0.012 * f.u_axis[1]), yaw, size, zz)
    Z = det.points(render(det, scene, 0.001, 11))[..., 2]
    assert det.check_shifts(Z, recs) == []
    more, scene = _records(det, f, scene, [LAYER[4]], seed=50)     # a later placement
    more[0].index = 4
    recs += more
    (x, y), yaw, size, zz = scene[i]
    scene[i] = ((x + 0.012 * f.u_axis[0], y + 0.012 * f.u_axis[1]), yaw, size, zz)
    Z = det.points(render(det, scene, 0.001, 12))[..., 2]
    ev = det.check_shifts(Z, recs[:4] + recs[4:])
    assert [e.index for e in ev] == [3] and ev[0].shift == pytest.approx(0.024, abs=0.004)


def test_collapse_detected(det, pallet):
    solids, f = pallet
    recs, scene = _records(det, f, solids, LAYER)
    del scene[len(solids) + 4]                     # the box on top is gone
    Z = det.points(render(det, scene, 0.001, 5))[..., 2]
    ev = det.check_shifts(Z, recs)
    assert [(e.index, e.kind) for e in ev] == [(4, 'COLLAPSE')]
