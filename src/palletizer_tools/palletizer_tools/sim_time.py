"""Waiting in node (sim) time with a wall-clock cap (no ROS imports beyond the node passed).

The simulation's real-time factor drops to ~0.1 with large interlocked stacks; a wall-clock
timeout for something that takes sim time (fresh camera frames, a settle, a suction confirm)
then expires too early. These helpers measure the timeout on the node's clock (sim time with
use_sim_time) and give up after `wall_cap` s of wall time (default 10 x the timeout + 30 s) in
case the clock stops.
"""

import time


def now(node):
    return node.get_clock().now().nanoseconds * 1e-9


def wait(node, cond, timeout, wall_cap=None, period=0.01):
    """True as soon as cond() holds; False after `timeout` s of node time (or the cap)."""
    cap = wall_cap if wall_cap is not None else 10.0 * timeout + 30.0
    t0, w0 = now(node), time.monotonic()
    while True:
        if cond():
            return True
        if now(node) - t0 >= timeout or time.monotonic() - w0 >= cap:
            return bool(cond())
        time.sleep(period)


def sleep(node, dt, wall_cap=None):
    """Sleep dt s of node time."""
    wait(node, lambda: False, dt, wall_cap)
