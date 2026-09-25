"""Zone conveyor controller driven only by photo-eyes (no ground truth).

Zones (upstream -> downstream) come from cell_config conveyor.zones. Each zone has a photo-eye
near its downstream end. Zero-pressure accumulation, one box per downstream zone:
  - pick zone: runs while its eye is clear; stops when the eye becomes blocked and HOLDS
    until ~/resume. It never runs while its eye is blocked: a resume with the box still
    there keeps it in place (driving it into the end stop would rotate it and push the
    queue into it).
  - a zone releases a box into the next zone only when that zone is EMPTY. Occupancy is
    counted from the eyes: a box enters zone i+1 when the eye of zone i goes blocked->clear,
    and leaves the pick zone when the pick eye goes blocked->clear (picked or removed).
    So a queued box always waits at its eye with the designed gap (>= 0.10 m) instead of
    following the previous box into the pick zone.
  - the infeed (first zone) may hold several boxes (the spawner keeps their gap).
  - ~/stop: maintenance stop of all zones until ~/resume.

Topics out: /conveyor/<zone>/cmd_vel (Float64), ~/running (Bool, pick zone running),
            ~/box_ready (Bool, pick eye blocked and pick zone stopped for settle_time),
            ~/eyes (String, e.g. "infeed=0 accum=1 pick=1")
Topics in:  /conveyor/<zone>/eye (LaserScan)
Services:   ~/resume, ~/stop (std_srvs/Trigger)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from palletizer_tools import cell_config


class ConveyorController(Node):

    def __init__(self):
        super().__init__('conveyor_controller')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None)
        cv = self.cell['conveyor']
        self.speed = float(self.declare_parameter('speed', float(cv['speed'])).value)
        self.zones = [z['name'] for z in cv['zones']]
        self.blocked_range = cv['photo_eye']['blocked_range']
        self.debounce = int(cv['photo_eye']['debounce'])
        self.removal_confirm = float(cv['photo_eye']['removal_confirm'])
        self.raw = {z: False for z in self.zones}
        self.transit_limit = {z['name']: cv['transit_timeout_factor'] * (z['x_max'] - z['x_min']) /
                              max(self.speed, 1e-3) for z in cv['zones']}
        self.raw_count = {z: 0 for z in self.zones}        # consecutive samples disagreeing
        self.clear_running = {z: 0.0 for z in self.zones}  # s the eye was clear while running
        self.settle_time = cv['settle_time']

        self.blocked = {z: False for z in self.zones}
        self.prev_blocked = {z: False for z in self.zones}
        self.occupancy = {z: 0 for z in self.zones}   # boxes inside zones[1:] (from eyes)
        self.running = {z: True for z in self.zones}
        self.holding = False          # pick zone stopped for a box, waiting for resume
        self.maintenance_stop = False
        self.ready_since = None

        self.cmd_pubs = {z: self.create_publisher(Float64, f'/conveyor/{z}/cmd_vel', 10)
                         for z in self.zones}
        for z in self.zones:
            self.create_subscription(
                LaserScan, f'/conveyor/{z}/eye', lambda m, z=z: self.on_eye(z, m), 10)
        self.running_pub = self.create_publisher(Bool, '~/running', 10)
        self.ready_pub = self.create_publisher(Bool, '~/box_ready', 10)
        self.eyes_pub = self.create_publisher(String, '~/eyes', 10)
        self.create_service(Trigger, '~/resume', self.on_resume)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(0.02, self.step)
        self.get_logger().info(
            f'conveyor_controller ready: zones {self.zones}, speed {self.speed} m/s')

    def on_eye(self, zone, msg):
        # -inf / below range_min: something right in front of the emitter -> blocked
        near = any(r == -math.inf or (math.isfinite(r) and r < msg.range_min) for r in msg.ranges)
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r]
        self.raw[zone] = near or (bool(valid) and min(valid) < self.blocked_range)

    def update_eyes(self, dt):
        """Debounced eye states. A box cannot cross an eye while its zone's belt is stopped,
        so changes on a stopped zone are ignored - except the pick eye clearing (the robot
        lifted the box), which must stay clear for removal_confirm seconds."""
        pick = self.zones[-1]
        for z in self.zones:
            if self.raw[z] == self.blocked[z]:
                self.raw_count[z] = 0
                continue
            self.raw_count[z] += 1
            if self.running.get(z, True):
                need = self.debounce
            elif z == pick and self.blocked[z] and not self.raw[z]:
                need = max(1, int(round(self.removal_confirm / dt)))
            else:
                continue                      # stopped belt: the reading is flicker
            if self.raw_count[z] >= need:
                self.blocked[z] = self.raw[z]
                self.raw_count[z] = 0

    def update_occupancy(self):
        for i, z in enumerate(self.zones):
            if self.prev_blocked[z] and not self.blocked[z]:
                # falling edge of the eye at the end of zone z: one box left zone z
                # (into the next zone, or out of the cell at the pick eye)
                if i > 0:
                    self.occupancy[z] = max(0, self.occupancy[z] - 1)
                if i + 1 < len(self.zones):
                    self.occupancy[self.zones[i + 1]] += 1
            self.prev_blocked[z] = self.blocked[z]
        # a box standing at an eye is certainly inside that zone
        for z in self.zones[1:]:
            if self.blocked[z]:
                self.occupancy[z] = max(self.occupancy[z], 1)
        # transit watchdog: a claimed box must reach the eye within the zone transit time
        dt = 0.02
        for z in self.zones[1:]:
            if self.occupancy[z] > 0 and not self.blocked[z] and self.running.get(z, False):
                self.clear_running[z] += dt
                if self.clear_running[z] > self.transit_limit[z]:
                    self.get_logger().warn(
                        f'zone {z}: counted {self.occupancy[z]} box(es) but none reached the '
                        f'eye within {self.transit_limit[z]:.1f} s; resetting occupancy')
                    self.occupancy[z] = 0
                    self.clear_running[z] = 0.0
            else:
                self.clear_running[z] = 0.0

    def step(self):
        self.update_eyes(0.02)
        self.update_occupancy()
        pick = self.zones[-1]
        # --- pick zone ---
        if not self.holding and self.blocked[pick]:
            self.holding = True
            self.get_logger().info('box at pick eye, holding pick zone')
        run = {pick: not self.holding and not self.maintenance_stop}
        # --- upstream zones: run while their eye is clear; release a box past the eye only
        # into an empty, running downstream zone ---
        for i in range(len(self.zones) - 2, -1, -1):
            z, nxt = self.zones[i], self.zones[i + 1]
            release = run[nxt] and self.occupancy[nxt] == 0
            run[z] = not self.maintenance_stop and (not self.blocked[z] or release)
        self.running = run
        for z in self.zones:
            self.cmd_pubs[z].publish(Float64(data=self.speed if run[z] else 0.0))

        now = self.get_clock().now().nanoseconds * 1e-9
        stopped_with_box = self.holding and self.blocked[pick]
        if stopped_with_box and self.ready_since is None:
            self.ready_since = now
        elif not stopped_with_box:
            self.ready_since = None
        ready = self.ready_since is not None and now - self.ready_since >= self.settle_time
        self.running_pub.publish(Bool(data=run[pick]))
        self.ready_pub.publish(Bool(data=ready))
        self.eyes_pub.publish(String(data=' '.join(
            f'{z}={int(self.blocked[z])}' for z in self.zones) + ' occupancy ' + ' '.join(
            f'{z}={self.occupancy[z]}' for z in self.zones[1:])))

    def on_resume(self, request, response):
        self.maintenance_stop = False
        self.holding = False          # re-latches immediately if the box is still at the eye
        response.success = True
        response.message = ('box still at the pick eye: holding' if self.blocked[self.zones[-1]]
                            else 'running')
        return response

    def on_stop(self, request, response):
        self.maintenance_stop = True
        response.success = True
        response.message = 'all zones stopped'
        return response


def main():
    rclpy.init()
    rclpy.spin(ConveyorController())


if __name__ == '__main__':
    main()
