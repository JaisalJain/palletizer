"""Simulated plant operations (environment side, NOT robot control).

These model physical equipment/operators that exist around a real cell:
  /sim/reject_pick_zone_box (Trigger)  reject pusher: removes the box resting in the pick zone
  /sim/reject_held_box (Trigger)       operator removes the box hanging on the gripper
                                       (after suction off) - used only as a last recovery
  /sim/clear_pallet (Trigger)          forklift pallet swap: removes every box on the pallet

Because they act on the simulated world, they may use ground truth (/sim/box_poses) to find
which models to delete. Nothing here feeds perception, planning or control.
"""

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from palletizer_tools import boxes, cell_config


class SimOps(Node):

    def __init__(self):
        super().__init__('sim_ops')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None)
        self.poses = {}
        self.attached = ''
        self.create_subscription(TFMessage, '/sim/box_poses', self.on_poses, 10)
        self.create_subscription(String, '/suction/attached_model', self.on_attached, 10)
        self.delete_cli = self.create_client(
            DeleteEntity, f"/world/{self.cell['world_name']}/remove",
            callback_group=MutuallyExclusiveCallbackGroup())
        cb = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, '/sim/reject_pick_zone_box', self.on_reject_pick, callback_group=cb)
        self.create_service(Trigger, '/sim/reject_held_box', self.on_reject_held, callback_group=cb)
        self.create_service(Trigger, '/sim/clear_pallet', self.on_clear_pallet, callback_group=cb)
        self.get_logger().info('sim_ops ready')

    def on_poses(self, msg):
        self.poses = boxes.poses_from_tf(msg)

    def on_attached(self, msg):
        self.attached = msg.data

    def delete(self, name):
        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        if not self.delete_cli.wait_for_service(timeout_sec=3.0):
            return False
        res = self.delete_cli.call(req)
        return res is not None and res.success

    def on_reject_pick(self, request, response):
        cv = self.cell['conveyor']
        pick = [z for z in cv['zones'] if z['name'] == 'pick'][0]
        names = [n for n, (xyz, _) in self.poses.items()
                 if pick['x_min'] <= xyz[0] <= pick['x_max'] + 0.1
                 and abs(xyz[1] - cv['y']) < cv['belt_width'] / 2 + 0.1
                 and xyz[2] < cv['belt_top_z'] + 0.5]
        if not names:
            response.success, response.message = False, 'no box in the pick zone'
            return response
        name = max(names, key=lambda n: self.poses[n][0][0])      # most downstream
        response.success = self.delete(name)
        response.message = name
        return response

    def on_reject_held(self, request, response):
        name = self.attached
        if not name:
            response.success, response.message = False, 'nothing attached'
            return response
        response.success = self.delete(name)
        response.message = name
        return response

    def on_clear_pallet(self, request, response):
        pal = self.cell['pallet']
        cx, cy = pal['center_xy']
        hx, hy = pal['size'][0] / 2 + 0.3, pal['size'][1] / 2 + 0.3
        names = [n for n, (xyz, _) in self.poses.items()
                 if abs(xyz[0] - cx) <= hx and abs(xyz[1] - cy) <= hy and n != self.attached]
        ok = all(self.delete(n) for n in names)
        response.success = ok
        response.message = f'removed {len(names)} boxes'
        return response


def main():
    rclpy.init()
    node = SimOps()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.spin()


if __name__ == '__main__':
    main()
