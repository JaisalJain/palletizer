"""Spawns random cardboard boxes at the upstream end of the conveyor.

Params:  rate (boxes/s, 0 = on demand only; default from cell_config), seed, cell_config
Service: ~/spawn_box (std_srvs/Trigger) spawns one box now (if the spawn area is clear)
Topic:   ~/spawned (std_msgs/String) name of every spawned box
"""

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
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
        self.rng = np.random.default_rng(None if seed < 0 else seed)

        self.poses = {}
        self.counter = 0
        cb = MutuallyExclusiveCallbackGroup()
        self.create_subscription(TFMessage, '/sim/box_poses', self.on_poses, 10)
        world = self.cell['world_name']
        # Own group: the synchronous call() from the timer/service needs the response
        # callback to run on another executor thread.
        self.spawn_cli = self.create_client(
            SpawnEntity, f'/world/{world}/create',
            callback_group=MutuallyExclusiveCallbackGroup())
        self.spawned_pub = self.create_publisher(String, '~/spawned', 10)
        self.create_service(Trigger, '~/spawn_box', self.on_spawn_srv, callback_group=cb)
        if self.rate > 0:
            self.create_timer(1.0 / self.rate, self.on_timer, callback_group=cb)
        self.get_logger().info(f'box_spawner ready (rate {self.rate} boxes/s)')

    def on_poses(self, msg):
        self.poses = boxes.poses_from_tf(msg)

    def spawn_area_clear(self):
        sx = self.cell['spawner']['spawn_x']
        clearance = self.cell['spawner']['clearance']
        return all(abs(p[0][0] - sx) > clearance for p in self.poses.values())

    def next_name(self):
        while True:
            self.counter += 1
            name = f'box_{self.counter:04d}'
            if name not in self.poses:
                return name

    def spawn_one(self):
        if not self.spawn_area_clear():
            return False, 'spawn area occupied'
        if not self.spawn_cli.wait_for_service(timeout_sec=2.0):
            return False, 'spawn service not available'
        size, mass, yaw, rgba = boxes.random_box(self.rng, self.cell['spawner'])
        name = self.next_name()
        req = boxes.spawn_request(name, size, mass, rgba, boxes.spawn_xyz(self.cell, size), yaw)
        res = self.spawn_cli.call(req)
        if res is None or not res.success:
            return False, f'gz refused to spawn {name}'
        self.spawned_pub.publish(String(data=name))
        desc = (f'{name}: {size[0]:.2f}x{size[1]:.2f}x{size[2]:.2f} m, '
                f'{mass:.1f} kg, yaw {yaw:+.2f}')
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
