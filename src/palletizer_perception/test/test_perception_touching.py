#!/usr/bin/env python3
"""Perception with two boxes touching in the pick ROI (needs a running sim + box_perception).

Cases (front box at the pick eye, second box touching it from behind):
  1. different heights, rear box lower
  2. different heights, rear box taller
  3. equal heights, different colours (merged top face -> colour split)
  4. equal heights, identical colour: no separable cue -> must NOT return wrong dims
     (either detection fails or it reports the true front box)
Pass criterion per case: front box dims within 12 mm (Milestone B max), grasp point within
10 mm, or an explicit failure in case 4.
"""

import math
import sys
import time

import numpy as np
import rclpy
from palletizer_interfaces.srv import DetectBox
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger

from palletizer_tools.boxes import x_extent
from palletizer_tools.sim_client import SimClient

CASES = [
    ('rear lower', (0.40, 0.30, 0.30), (0.35, 0.28, 0.20), (0.75, 0.58, 0.36), (0.62, 0.46, 0.30), False),
    ('rear taller', (0.30, 0.30, 0.18), (0.45, 0.35, 0.36), (0.75, 0.58, 0.36), (0.62, 0.46, 0.30), False),
    ('equal height, colours differ', (0.40, 0.30, 0.25), (0.30, 0.30, 0.25),
     (0.80, 0.62, 0.40), (0.60, 0.44, 0.28), False),
    ('equal height, same colour', (0.40, 0.30, 0.25), (0.30, 0.30, 0.25),
     (0.72, 0.55, 0.36), (0.72, 0.55, 0.36), True),
]


def main():
    rclpy.init()
    sim = SimClient('test_touching')
    detect = sim.create_client(DetectBox, '/perception/detect_box')
    sim.wait_ready()
    cv = sim.cell['conveyor']
    eye = [z for z in cv['zones'] if z['name'] == 'pick'][0]['eye_x']
    sim.call(sim.stop_cli, Trigger.Request())
    sim.delete_all_boxes()
    failed = 0
    print('== test_perception_touching')
    for label, sa, sb, ca, cb, may_fail in CASES:
        xa = eye - x_extent(sa, 0) / 2 - 0.01
        xb = xa - sa[0] / 2 - sb[0] / 2 - 0.001          # touching (1 mm)
        sim.spawn_box('box_front', sa, 4.0, (xa, cv['y'], cv['belt_top_z'] + sa[2] / 2 + 0.003), 0.0,
                      ca + (1.0,))
        sim.spawn_box('box_rear', sb, 4.0, (xb, cv['y'], cv['belt_top_z'] + sb[2] / 2 + 0.003), 0.0,
                      cb + (1.0,))
        time.sleep(1.0)
        pos, quat = sim.box_poses['box_front']
        top = pos + Rotation.from_quat(quat).apply([0, 0, sa[2] / 2])
        gap = (sim.box_poses['box_front'][0][0] - sa[0] / 2) - (sim.box_poses['box_rear'][0][0] + sb[0] / 2)
        res = sim.call(detect, DetectBox.Request(), timeout=10.0)
        if res.success:
            b = res.box
            dims_err = max(abs(b.length - max(sa[:2])), abs(b.width - min(sa[:2])),
                           abs(b.height - sa[2])) * 1000
            c = np.array([b.top_center.position.x, b.top_center.position.y, b.top_center.position.z])
            cerr = np.linalg.norm(c - top) * 1000
            ok = dims_err <= 12.0 and cerr <= 10.0
            text = (f'detected {b.length * 1000:.0f}x{b.width * 1000:.0f}x{b.height * 1000:.0f} mm, '
                    f'dims err {dims_err:.1f} mm, center err {cerr:.1f} mm, '
                    f'yaw {math.degrees(b.yaw):+.2f} deg, conf {b.confidence:.2f}')
        else:
            ok = may_fail
            text = f'no detection ({res.message})' + (' - acceptable: no separable cue' if ok else '')
        print(f'  [{"PASS" if ok else "FAIL"}] {label} (gap {gap * 1000:.1f} mm): {text}', flush=True)
        failed += not ok
        sim.delete_all_boxes()
    sim.call(sim.resume_cli, Trigger.Request())
    print(f'test_perception_touching: {"PASS" if not failed else f"FAIL ({failed})"}')
    sim.shutdown()
    rclpy.try_shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
