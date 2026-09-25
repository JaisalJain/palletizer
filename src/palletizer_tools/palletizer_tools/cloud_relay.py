"""Re-stamps the overhead point cloud with the correct frame.

gz-sensors publishes rgbd_camera points in the camera *link* convention (x forward),
but stamps them with <gz_frame_id>, which must be the optical frame for the images and
camera_info. This relay fixes the cloud's frame_id without copying point data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class CloudRelay(Node):

    def __init__(self):
        super().__init__('cloud_relay')
        self.frame_id = self.declare_parameter('frame_id', 'overhead_camera_link').value
        self.pub = self.create_publisher(PointCloud2, '/overhead_camera/points', 5)
        # Reliable QoS: best-effort drops fragments of these ~10 MB messages.
        self.create_subscription(PointCloud2, '/overhead_camera/points_raw', self.on_cloud, 5)

    def on_cloud(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(CloudRelay())


if __name__ == '__main__':
    main()
