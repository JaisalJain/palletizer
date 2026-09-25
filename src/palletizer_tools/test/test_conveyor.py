#!/usr/bin/env python3
"""Test 4: boxes ride the belt; the pick zone stops at the photo-eye; queued boxes keep a
>= 0.10 m gap (zone accumulation); resume releases the zone.

The controller only uses photo-eyes; ground truth (/sim/box_poses, /sim/box_specs) is used
here purely to check the result.
"""

import math
import time

from palletizer_interfaces.msg import BoxSpec
from rclpy.qos import DurabilityPolicy, QoSProfile
from sim_helpers import run
from std_srvs.srv import Trigger

from palletizer_tools.boxes import x_extent


def test(sim, c):
    specs = {}
    sim.create_subscription(
        BoxSpec, '/sim/box_specs', lambda m: specs.__setitem__(m.name, m),
        QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    zones = {z['name']: z for z in sim.cell['conveyor']['zones']}
    pick_eye = zones['pick']['eye_x']
    accum_eye = zones['accum']['eye_x']

    sim.delete_all_boxes()
    sim.call(sim.resume_cli, Trigger.Request())
    c.check(sim.wait_for(lambda: sim.belt_running is True, 3.0), 'belt running after resume')

    def spawn():
        res = None
        for _ in range(60):       # the spawner waits for a gap at the infeed
            res = sim.call(sim.spawn_box_cli, Trigger.Request())
            if res.success:
                break
            time.sleep(0.5)
        c.check(res.success, f'spawn_box: {res.message}')
        name = res.message.split(':')[0]
        sim.wait_for(lambda: name in sim.box_poses and name in specs, 5.0)
        return name

    def front(name):
        xyz, q = sim.box_poses[name]
        s = specs[name]
        yaw = 2.0 * math.atan2(q[2], q[3])
        return xyz[0] + x_extent((s.size_x, s.size_y), yaw) / 2.0, \
            xyz[0] - x_extent((s.size_x, s.size_y), yaw) / 2.0

    first = spawn()
    time.sleep(1.5)
    x0 = sim.box_poses[first][0][0]
    t0 = time.time()
    time.sleep(2.0)
    speed = (sim.box_poses[first][0][0] - x0) / (time.time() - t0)
    belt_speed = sim.cell['conveyor']['speed']
    c.check(abs(speed - belt_speed) < 0.25 * belt_speed,
            f'{first} rides the belt at {speed:.3f} m/s (belt {belt_speed} m/s)')
    second = spawn()

    stopped = sim.wait_for(lambda: sim.belt_running is False, 25.0)
    c.check(stopped, 'pick zone stopped (photo-eye)')
    time.sleep(1.0)
    f1, r1 = front(first)
    c.check(abs(f1 - pick_eye) < 0.04,
            f'{first} leading edge at x={f1:.3f} (pick eye at {pick_eye})')
    x_stop = sim.box_poses[first][0][0]
    time.sleep(2.0)
    drift = abs(sim.box_poses[first][0][0] - x_stop)
    c.check(drift < 0.005, f'{first} stays put while held (drift {drift * 1000:.1f} mm)')

    # The second box accumulates behind with a gap.
    sim.wait_for(lambda: front(second)[0] > accum_eye - 0.05, 20.0)
    time.sleep(1.5)
    f2, _ = front(second)
    gap = r1 - f2
    c.check(gap >= 0.10, f'queued {second}: gap to {first} = {gap:.3f} m (>= 0.10)')
    x2 = sim.box_poses[second][0][0]
    time.sleep(1.5)
    c.check(abs(sim.box_poses[second][0][0] - x2) < 0.005, f'{second} waits at the accum eye')

    # "Pick" the first box, resume: the second box advances to the pick eye.
    sim.delete_box(first)
    sim.call(sim.resume_cli, Trigger.Request())
    ok = sim.wait_for(lambda: sim.belt_running is False and abs(front(second)[0] - pick_eye) < 0.05,
                      20.0)
    c.check(ok, f'after resume {second} advanced to the pick eye (front {front(second)[0]:.3f})')
    sim.delete_all_boxes()
    sim.call(sim.resume_cli, Trigger.Request())


if __name__ == '__main__':
    run(test, 'test_conveyor')
