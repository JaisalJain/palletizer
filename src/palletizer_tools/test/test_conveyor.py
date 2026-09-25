#!/usr/bin/env python3
"""Test 4: spawned boxes ride the belt; the belt stops when a box reaches the pick zone."""

import time

from sim_helpers import run
from std_srvs.srv import Trigger


def test(sim, c):
    sim.delete_all_boxes()
    sim.call(sim.resume_cli, Trigger.Request())
    c.check(sim.wait_for(lambda: sim.belt_running is True, 3.0), 'belt running after resume')

    res = sim.call(sim.spawn_box_cli, Trigger.Request())
    c.check(res.success, f'box_spawner/spawn_box: {res.message}')
    name = res.message.split(':')[0]
    c.check(sim.wait_for(lambda: name in sim.box_poses, 5.0), f'{name} in /sim/box_poses')

    time.sleep(1.5)  # let it drop onto the belt
    x0 = sim.box_poses[name][0][0]
    t0 = time.time()
    time.sleep(2.0)
    x1 = sim.box_poses[name][0][0]
    speed = (x1 - x0) / (time.time() - t0)
    belt_speed = sim.cell['conveyor']['speed']
    c.check(abs(speed - belt_speed) < 0.25 * belt_speed,
            f'box rides the belt at {speed:.3f} m/s (belt {belt_speed} m/s)')

    stopped = sim.wait_for(lambda: sim.belt_running is False, 20.0)
    c.check(stopped, 'belt stopped')
    time.sleep(1.0)
    pick_x = sim.cell['pick_zone']['xyz'][0]
    xs = sim.box_poses[name][0][0]
    c.check(abs(xs - pick_x) < 0.08, f'box stopped at x={xs:.3f} (pick zone x={pick_x})')
    time.sleep(2.0)
    drift = abs(sim.box_poses[name][0][0] - xs)
    c.check(drift < 0.005, f'box stays put while belt is stopped (drift {drift * 1000:.1f} mm)')

    # Resume on command: the belt runs again and the box moves on.
    sim.call(sim.resume_cli, Trigger.Request())
    time.sleep(1.5)
    moved = sim.box_poses[name][0][0] - xs
    c.check(sim.belt_running is True and moved > 0.1,
            f'belt resumed on command, box moved {moved:.3f} m')
    sim.delete_all_boxes()


if __name__ == '__main__':
    run(test, 'test_conveyor')
