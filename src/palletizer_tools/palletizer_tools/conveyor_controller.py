"""Zone conveyor controller driven only by photo-eyes (no ground truth).

Zones (upstream -> downstream) come from cell_config conveyor.zones. Each zone has a photo-eye
near its downstream end. Zero-pressure accumulation:
  - pick zone: runs while its eye is clear; stops when the eye becomes blocked and HOLDS
    until ~/resume. After a resume the zone runs; if the eye is still blocked (box not
    picked) the box is released downstream and the zone stops again at the next arrival.
  - other zones: run if their eye is clear or the next zone downstream is running.
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
        self.settle_time = cv['settle_time']

        self.blocked = {z: False for z in self.zones}
        self.running = {z: True for z in self.zones}
        self.holding = False          # pick zone stopped for a box, waiting for resume
        self.releasing = False        # resume issued while the pick eye is still blocked
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
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r]
        self.blocked[zone] = bool(valid) and min(valid) < self.blocked_range

    def step(self):
        pick = self.zones[-1]
        # --- pick zone ---
        if self.releasing and not self.blocked[pick]:
            self.releasing = False
        if not self.holding and not self.releasing and self.blocked[pick]:
            self.holding = True
            self.get_logger().info('box at pick eye, holding pick zone')
        run = {pick: not self.holding and not self.maintenance_stop}
        # --- upstream zones: run if eye clear or downstream zone running ---
        for i in range(len(self.zones) - 2, -1, -1):
            z = self.zones[i]
            run[z] = not self.maintenance_stop and (not self.blocked[z] or run[self.zones[i + 1]])
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
            f'{z}={int(self.blocked[z])}' for z in self.zones)))

    def on_resume(self, request, response):
        self.maintenance_stop = False
        if self.holding:
            self.holding = False
            self.releasing = self.blocked[self.zones[-1]]
        response.success = True
        response.message = 'released box downstream' if self.releasing else 'running'
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
