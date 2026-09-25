#!/usr/bin/env python3
"""Test 2: controllers are active; a joint trajectory moves the arm to a target and back."""

import numpy as np
from sim_helpers import run


def test(sim, c):
    states = sim.controller_states()
    for name in ['joint_state_broadcaster', 'joint_trajectory_controller']:
        c.check(states.get(name) == 'active', f'{name} is {states.get(name)}')

    start = sim.joints.copy()
    target = start + np.array([0.5, 0.3, -0.4, 0.3, 0.4, 0.6])
    err = sim.move_joints(target)
    c.check(err < 0.01, f'reached target, max joint error {err:.4f} rad')
    moved = float(np.max(np.abs(sim.joints - start)))
    c.check(moved > 0.3, f'arm actually moved ({moved:.2f} rad)')

    err = sim.move_joints(start)
    c.check(err < 0.01, f'returned to start, max joint error {err:.4f} rad')


if __name__ == '__main__':
    run(test, 'test_controllers')
