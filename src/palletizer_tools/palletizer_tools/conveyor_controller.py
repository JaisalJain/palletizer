"""Runs the conveyor belt and stops it when a box reaches the pick zone.

Uses ground-truth box poses from Gazebo (/sim/box_poses). The belt stops as soon as a box
center passes pick_x - stop_window (inside the belt footprint) and stays stopped until
~/resume is called and that box has been removed from the pick zone (picked). A resume
while the box is still there is honored once: the belt runs and the box moves on to the
end stop.

Topics:   /conveyor/belt_cmd_vel (std_msgs/Float64, out), ~/box_in_pick_zone (std_msgs/String,
          name or ''), ~/running (std_msgs/Bool)
Services: ~/resume, ~/stop (std_srvs/Trigger)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from palletizer_tools import boxes, cell_config


class ConveyorController(Node):

    def __init__(self):
        super().__init__('conveyor_controller')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None)
        cv = self.cell['conveyor']
        self.speed = float(self.declare_parameter('speed', float(cv['speed'])).value)
        self.pick_x = self.cell['pick_zone']['xyz'][0]
        self.stop_x = self.pick_x - self.cell['pick_zone']['stop_window']
        self.belt_y = cv['y']
        self.half_w = cv['belt_width'] / 2.0
        self.belt_top = cv['belt_top_z']

        self.running = True
        self.ignored = set()      # boxes allowed past the pick zone by a manual resume
        self.box_at_pick = ''

        self.cmd_pub = self.create_publisher(Float64, '/conveyor/belt_cmd_vel', 10)
        self.zone_pub = self.create_publisher(String, '~/box_in_pick_zone', 10)
        self.running_pub = self.create_publisher(Bool, '~/running', 10)
        self.create_subscription(TFMessage, '/sim/box_poses', self.on_poses, 10)
        self.create_service(Trigger, '~/resume', self.on_resume)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(0.1, self.publish_cmd)
        self.get_logger().info(f'conveyor_controller ready (speed {self.speed} m/s)')

    def on_board(self, xyz):
        return (abs(xyz[1] - self.belt_y) < self.half_w + 0.1 and
                self.belt_top - 0.05 < xyz[2] < self.belt_top + 0.6)

    def on_poses(self, msg):
        poses = boxes.poses_from_tf(msg)
        self.ignored &= set(poses)
        at_pick = ''
        for name, (xyz, _) in poses.items():
            if name in self.ignored or not self.on_board(xyz):
                continue
            if xyz[0] >= self.stop_x:
                at_pick = name
                break
        if at_pick and self.running:
            self.get_logger().info(f'{at_pick} reached the pick zone, stopping belt')
            self.running = False
        self.box_at_pick = at_pick
        self.zone_pub.publish(String(data=at_pick))

    def publish_cmd(self):
        self.cmd_pub.publish(Float64(data=self.speed if self.running else 0.0))
        self.running_pub.publish(Bool(data=self.running))

    def on_resume(self, request, response):
        if self.box_at_pick:
            self.ignored.add(self.box_at_pick)
        self.running = True
        response.success = True
        response.message = 'belt running'
        return response

    def on_stop(self, request, response):
        self.running = False
        response.success = True
        response.message = 'belt stopped'
        return response


def main():
    rclpy.init()
    rclpy.spin(ConveyorController())


if __name__ == '__main__':
    main()
