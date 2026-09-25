#!/usr/bin/env python3
"""Test 5: suction grasp, lift, release; off-center grasp is rejected."""

import time

import numpy as np
from sim_helpers import run
from std_srvs.srv import Trigger

BOX = (0.40, 0.30, 0.25)
MASS = 10.0     # worst case of the spawner range (1-10 kg)
GAP = 0.003      # cup face this far above the box top when suction turns on
LIFT = 0.30


def test(sim, c):
    sim.call(sim.stop_cli, Trigger.Request())   # hold all zones during the test
    sim.delete_all_boxes()
    px, py, pz = sim.cell['pick_zone']['xyz']
    name = 'box_suctiontest'
    sim.spawn_box(name, BOX, MASS, (px, py, pz + BOX[2] / 2 + 0.005), yaw=0.15)
    time.sleep(2.0)  # settle

    sim.suction(False)
    sim.move_joints(sim.pick_approach_joints())
    top = sim.box_top_center(name, BOX[2])
    yaw = sim.box_yaw(name)
    rest_z = sim.box_poses[name][0][2]

    # --- off-center attempt must be rejected ---
    off = np.array([0.05 * np.cos(yaw + np.pi / 2), 0.05 * np.sin(yaw + np.pi / 2), 0.0])
    sim.move_tcp(top + off + [0, 0, 0.15], yaw)
    tcp = sim.move_tcp(top + off + [0, 0, GAP], yaw)
    c.check(np.linalg.norm(tcp[:3, 3] - (top + off + [0, 0, GAP])) < 0.003,
            f'TCP 50 mm off-center, {GAP * 1000:.0f} mm above box top')
    sim.grasped = None
    sim.suction(True)
    sim.wait_for(lambda: sim.grasped is not None, 2.0)
    time.sleep(1.0)
    c.check(sim.grasped is False and sim.attached == '',
            f'off-center grasp rejected (grasped={sim.grasped}, attached="{sim.attached}")')
    sim.suction(False)
    sim.move_tcp(top + off + [0, 0, 0.15], yaw)

    # --- centered grasp ---
    sim.move_tcp(top + [0, 0, 0.15], yaw)
    tcp = sim.move_tcp(top + [0, 0, GAP], yaw)
    c.check(np.linalg.norm(tcp[:3, 3] - (top + [0, 0, GAP])) < 0.003,
            f'TCP centered {GAP * 1000:.0f} mm above box top')
    sim.suction(True)
    ok = sim.wait_for(lambda: sim.grasped is True and sim.attached == name, 3.0)
    c.check(ok, f'attached: grasped={sim.grasped}, attached_model="{sim.attached}"')

    box_before = sim.box_poses[name][0].copy()
    sim.move_tcp(top + [0, 0, GAP + LIFT], yaw, max_vel=0.3)
    time.sleep(0.5)
    box_after = sim.box_poses[name][0].copy()
    dz = box_after[2] - box_before[2]
    dxy = np.linalg.norm(box_after[:2] - box_before[:2])
    c.check(abs(dz - LIFT) < 0.02, f'box followed the lift: dz = {dz:.3f} m (expected {LIFT})')
    c.check(dxy < 0.02, f'box stayed under the cup: dxy = {dxy * 1000:.1f} mm')
    c.check(sim.grasped is True, 'still grasped after lift')

    # --- release: box falls back onto the belt ---
    sim.suction(False)
    ok = sim.wait_for(lambda: sim.grasped is False and sim.attached == '', 3.0)
    c.check(ok, f'released: grasped={sim.grasped}, attached_model="{sim.attached}"')
    time.sleep(2.0)
    z = sim.box_poses[name][0][2]
    c.check(abs(z - rest_z) < 0.02, f'box fell back to the belt: z = {z:.3f} m (rest {rest_z:.3f})')

    sim.move_tcp(top + [0, 0, 0.4], yaw)
    sim.home()
    sim.delete_all_boxes()
    sim.call(sim.resume_cli, Trigger.Request())


if __name__ == '__main__':
    run(test, 'test_suction')
