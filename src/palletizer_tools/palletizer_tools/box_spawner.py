"""Spawns random cardboard boxes at the upstream end of the conveyor.

The spawner plays the upstream process (it is not part of the robot cell's control), so it
may use ground-truth poses to keep a minimum gap between boxes on the infeed.

Params:  rate (spawn attempts/s, 0 = on demand only; default from cell_config), seed
         (-1 = random), max_boxes (0 = unlimited), cell_config,
         sku   single-SKU stream: catalog key ("400x300x300") or raw dims in mm ("347x253x219")
         order order list "400x300x300:30, 600x400x300:20" (the SKU changes mid-stream; the
               robot is NOT told); spawning stops when the list is done
         (neither sku nor order: mixed mode, v1 random boxes)
Service: ~/spawn_box (std_srvs/Trigger) spawn the next box now (if the gap allows)
Topics:  ~/spawned (std_msgs/String) name of every spawned box
         /sim/box_specs (palletizer_interfaces/BoxSpec, transient local) ground-truth specs,
         EVALUATION ONLY
"""

import math
import time

import numpy as np
import rclpy
from palletizer_interfaces.msg import BoxSpec
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.srv import SpawnEntity
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from palletizer_tools import boxes, cell_config


class BoxSpawner(Node):

    def __init__(self):
        super().__init__('box_spawner')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None)
        sp = self.cell['spawner']
        self.rate = float(self.declare_parameter('rate', float(sp['rate'])).value)
        seed = int(self.declare_parameter('seed', -1).value)
        self.max_boxes = int(self.declare_parameter('max_boxes', 0).value)
        sku = str(self.declare_parameter('sku', '').value)
        order = str(self.declare_parameter('order', '').value)
        # [(key, size, mass_range, count)]; count 0 = unlimited
        self.order = boxes.parse_order(order, sp) if order else \
            ([(sku,) + boxes.parse_sku(sku, sp) + (0,)] if sku else [])
        self.order_pos = [0, 0]      # item index, boxes spawned of that item
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.min_gap = sp['min_gap']

        self.poses = {}
        self.specs = {}             # name -> (size, yaw) of boxes this spawner created
        self.counter = 0
        self.next_box = None        # drawn but not yet spawned (keeps seeded order)
        # Last spawned box until it shows up in /sim/box_poses: the gap check can't see it
        # before that. (With sim time the spawn timer can fire twice back-to-back at startup;
        # without this the second box was spawned into the first.)
        self.pending = None
        self.pending_since = 0.0
        cb = MutuallyExclusiveCallbackGroup()
        self.create_subscription(TFMessage, '/sim/box_poses', self.on_poses, 10)
        # Own group: the synchronous call() from the timer/service needs the response
        # callback to run on another executor thread.
        self.spawn_cli = self.create_client(
            SpawnEntity, f"/world/{self.cell['world_name']}/create",
            callback_group=MutuallyExclusiveCallbackGroup())
        self.spawned_pub = self.create_publisher(String, '~/spawned', 10)
        self.spec_pub = self.create_publisher(
            BoxSpec, '/sim/box_specs',
            QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '~/spawn_box', self.on_spawn_srv, callback_group=cb)
        if self.rate > 0:
            self.create_timer(1.0 / self.rate, self.on_timer, callback_group=cb)
        self.get_logger().info(
            f'box_spawner ready (rate {self.rate}/s, seed {seed}, max_boxes {self.max_boxes}, '
            f'{"order " + order if order else "sku " + sku if sku else "mixed"})')

    def on_poses(self, msg):
        self.poses = boxes.poses_from_tf(msg)

    def gap_ok(self, size, yaw):
        """True if the new box would be >= min_gap behind every box on the belt."""
        if self.pending is not None:
            if self.pending not in self.poses and time.monotonic() - self.pending_since < 10.0:
                return False
            self.pending = None
        spawn_x = boxes.spawn_xyz(self.cell, size, yaw)[0]
        front = spawn_x + boxes.x_extent(size, yaw) / 2.0
        cv = self.cell['conveyor']
        for name, (xyz, quat) in self.poses.items():
            if abs(xyz[1] - cv['y']) > cv['belt_width'] or xyz[2] < cv['belt_top_z'] - 0.05:
                continue   # not on the belt
            if name in self.specs:
                s, _ = self.specs[name]
                yaw_now = 2.0 * math.atan2(quat[2], quat[3])
                half = boxes.x_extent(s, yaw_now) / 2.0
            else:
                half = 0.45   # unknown box (e.g. spawned by a test): worst case
            if xyz[0] - half < front + self.min_gap:
                return False
        return True

    def next_name(self):
        while True:
            self.counter += 1
            name = f'box_{self.counter:04d}'
            if name not in self.poses:
                return name

    def spawn_one(self):
        if self.max_boxes and len(self.specs) >= self.max_boxes:
            return False, 'max_boxes reached'
        if self.next_box is None:
            if self.order:
                i, n = self.order_pos
                if i >= len(self.order):
                    return False, 'order list complete'
                key, sku_size, mass_range, _ = self.order[i]
                self.next_box = boxes.sku_box(self.rng, sku_size, mass_range,
                                              self.cell['spawner']) + (key,)
            else:
                self.next_box = boxes.random_box(self.rng, self.cell['spawner']) + ('',)
        size, mass, yaw, rgba, sku = self.next_box
        if not self.gap_ok(size, yaw):
            return False, 'not enough gap at the infeed'
        if not self.spawn_cli.wait_for_service(timeout_sec=2.0):
            return False, 'spawn service not available'
        name = self.next_name()
        req = boxes.spawn_request(
            name, size, mass, rgba, boxes.spawn_xyz(self.cell, size, yaw), yaw)
        res = self.spawn_cli.call(req)
        if res is None or not res.success:
            return False, f'gz refused to spawn {name}'
        self.next_box = None
        if self.order:
            self.order_pos[1] += 1
            count = self.order[self.order_pos[0]][3]
            if count and self.order_pos[1] >= count:
                self.order_pos = [self.order_pos[0] + 1, 0]
        self.pending, self.pending_since = name, time.monotonic()
        self.specs[name] = (size, yaw)
        spec = BoxSpec()
        spec.stamp = self.get_clock().now().to_msg()
        spec.name = name
        spec.size_x, spec.size_y, spec.size_z = size
        spec.mass = mass
        spec.spawn_yaw = yaw
        spec.sku = sku
        self.spec_pub.publish(spec)
        self.spawned_pub.publish(String(data=name))
        desc = (f'{name}{" [" + sku + "]" if sku else ""}: {size[0]:.3f}x{size[1]:.3f}x{size[2]:.3f} m, '
                f'{mass:.2f} kg, yaw {math.degrees(yaw):+.1f} deg')
        self.get_logger().info(f'spawned {desc}')
        return True, desc

    def on_timer(self):
        self.spawn_one()

    def on_spawn_srv(self, request, response):
        response.success, response.message = self.spawn_one()
        return response


def main():
    rclpy.init()
    node = BoxSpawner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    main()
