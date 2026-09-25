"""Shared helpers for the integration tests. Requires a running sim:

    ros2 launch palletizer_bringup palletizer.launch.py gui:=false spawn_rate:=0.0
"""

import os
import sys

import rclpy

from palletizer_tools.sim_client import JOINTS, SimClient  # noqa: F401

OUTPUT_DIR = os.path.join(
    os.environ.get('PALLETIZER_TEST_OUTPUT', os.path.expanduser('~/palletizer_ws/test_output')))


class Checks:
    """Collects PASS/FAIL lines and turns them into an exit code."""

    def __init__(self, name):
        self.name = name
        self.failed = 0

    def check(self, ok, msg):
        print(f'  [{"PASS" if ok else "FAIL"}] {msg}', flush=True)
        if not ok:
            self.failed += 1
        return ok

    def finish(self):
        print(f'{self.name}: {"PASS" if self.failed == 0 else f"FAIL ({self.failed})"}', flush=True)
        return 0 if self.failed == 0 else 1


def run(test_fn, name):
    rclpy.init()
    sim = SimClient()
    checks = Checks(name)
    print(f'== {name}', flush=True)
    try:
        if checks.check(sim.wait_ready(), 'sim is up (/joint_states received)'):
            test_fn(sim, checks)
    except Exception as e:  # noqa: BLE001
        checks.check(False, f'exception: {e!r}')
    finally:
        code = checks.finish()
        sim.shutdown()
        rclpy.try_shutdown()
    sys.exit(code)
