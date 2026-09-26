#!/usr/bin/env python3
"""Test 6: pallet conveyor (PalletMover + pallet_exchanger).

- a pallet is at the station and its collision moves with it (a dropped box rests on the deck)
- boxes on the pallet are FREE physics bodies while no exchange runs: a box dropped
  overhanging the edge tips off, a resting box is not held in place (shove + settle)
- during an exchange the pallet and its boxes move rigidly together, then are removed,
  and a new pallet arrives whose deck again carries boxes
"""

import math
import threading
import time

import numpy as np
from palletizer_tools import boxes
from sim_helpers import run
from tf2_msgs.msg import TFMessage

BOX = (0.30, 0.20, 0.20)


def on_pallet_frame(sim, name):
    """Box position in the station pallet's frame (ground truth)."""
    _, pxyz, pyaw = sim.station_pallet()
    d = sim.box_poses[name][0] - pxyz
    c, s = math.cos(pyaw), math.sin(pyaw)
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])


def deck_z(sim):
    """Deck height from the exchanger's pallet types (GT, test only)."""
    return sim.cell['exchanger']['pallet_types']['EUR']['size'][2]


def test(sim, c):
    sim.delete_all_boxes()
    if not sim.wait_for(lambda: sim.station_pallet() is not None and
                        sim.mover_state.startswith('idle'), 90.0):
        c.check(False, 'a pallet is at the station')
        return
    time.sleep(1.0)
    name, pxyz, pyaw = sim.station_pallet()
    c.check(True, f'pallet {name} at the station ({pxyz[0]:+.3f}, {pxyz[1]:+.3f}), '
                  f'yaw {math.degrees(pyaw):+.2f} deg')
    c.check(sim.mover_state.startswith('idle'), f'mover idle ({sim.mover_state})')
    z0 = deck_z(sim)

    # --- a box dropped on the deck rests on it (pallet collision moved with the pallet)
    cxy = pxyz[:2] + np.array([0.2, -0.1])
    sim.spawn_box('box_mt_a', BOX, 3.0, (cxy[0], cxy[1], z0 + BOX[2] / 2 + 0.05), yaw=pyaw)
    time.sleep(2.0)
    za = sim.box_poses['box_mt_a'][0][2]
    c.check(abs(za - (z0 + BOX[2] / 2)) < 0.005,
            f'dropped box rests on the deck (z {za:.4f}, expected {z0 + BOX[2] / 2:.4f})')

    # --- free physics while idle: a box whose centre overhangs the edge falls off
    edge = pxyz[:2] + np.array([0.6 + 0.08, 0.0])       # +x edge of an EUR pallet (+8 cm)
    sim.spawn_box('box_mt_b', BOX, 3.0, (edge[0], edge[1], z0 + BOX[2] / 2 + 0.01), yaw=0.0)
    time.sleep(3.0)
    zb = sim.box_poses['box_mt_b'][0][2]
    c.check(zb < z0 + BOX[2] / 2 - 0.03,
            f'overhanging box tipped over the edge (centre dropped '
            f'{(z0 + BOX[2] / 2 - zb) * 1000:.0f} mm)')
    sim.delete_box('box_mt_b')

    # --- a shoved box stays where it was shoved (nothing drags it back) and settles
    before = on_pallet_frame(sim, 'box_mt_a')
    p = sim.box_poses['box_mt_a'][0]
    sim.set_pose('box_mt_a', (p[0] + 0.05, p[1], p[2] + 0.002), yaw=pyaw)
    time.sleep(1.5)
    after = on_pallet_frame(sim, 'box_mt_a')
    shift = float(np.linalg.norm(after[:2] - before[:2]))
    c.check(abs(shift - 0.05) < 0.005, f'shoved box stays shoved ({shift * 1000:.1f} mm)')

    # --- exchange: box rides rigidly with the pallet, both removed; new pallet arrives
    rel0 = on_pallet_frame(sim, 'box_mt_a')
    worst = [0.0]
    moved = [0.0]
    # pair box and pallet ground truth by stamp (two publishers, same sim steps)
    hist = {'box': {}, 'pal': {}}

    def rec(key, prefix):
        def cb(msg):
            if msg.transforms:
                st = msg.transforms[0].header.stamp
                hist[key][(st.sec, st.nanosec)] = boxes.poses_from_tf(msg, prefix=prefix)
        return cb
    subs = [sim.create_subscription(TFMessage, '/sim/box_poses', rec('box', 'box_'), 50),
            sim.create_subscription(TFMessage, '/sim/pallet_poses', rec('pal', 'pallet_'), 50)]
    stop = threading.Event()

    def watch():
        while not stop.is_set():
            for k in sorted(set(hist['box']) & set(hist['pal'])):
                bp, pp = hist['box'].pop(k), hist['pal'].pop(k)
                if name not in pp or 'box_mt_a' not in bp:
                    continue
                pxyz_now, q = pp[name]
                yaw = 2 * math.atan2(q[2], q[3])
                d = bp['box_mt_a'][0] - pxyz_now
                cc, ss = math.cos(yaw), math.sin(yaw)
                rel = np.array([cc * d[0] + ss * d[1], -ss * d[0] + cc * d[1], d[2]])
                worst[0] = max(worst[0], float(np.linalg.norm(rel - rel0)))
                moved[0] = max(moved[0], float(pxyz_now[0] - pxyz[0]))
            time.sleep(0.05)
    t = threading.Thread(target=watch, daemon=True)
    t.start()
    phases = []
    res = sim.exchange_pallet(feedback=lambda fb: phases.append(fb.feedback.phase))
    stop.set()
    t.join()
    for sub in subs:
        sim.destroy_subscription(sub)
    c.check(res.success, f'exchange succeeded: {res.message}')
    c.check(moved[0] > 1.0, f'full pallet slid out along +x ({moved[0]:.2f} m observed)')
    c.check(worst[0] < 0.005, f'box rode rigidly with the pallet (max {worst[0] * 1000:.1f} mm)')
    c.check('box_mt_a' not in sim.box_poses and name not in sim.pallet_poses,
            'old pallet and its box removed')
    new = sim.station_pallet()
    c.check(new is not None and new[0] != name, f'new pallet at the station ({phases[-3:]})')
    if new is None:
        return
    c.check(sim.mover_state.startswith('idle'), f'mover idle after the exchange '
                                                f'({sim.mover_state})')
    sim.spawn_box('box_mt_c', BOX, 3.0, (new[1][0], new[1][1], z0 + BOX[2] / 2 + 0.05))
    time.sleep(2.0)
    zc = sim.box_poses['box_mt_c'][0][2]
    c.check(abs(zc - (z0 + BOX[2] / 2)) < 0.005, f'box rests on the new deck (z {zc:.4f})')
    sim.delete_box('box_mt_c')


if __name__ == '__main__':
    run(test, 'test_pallet_mover')
