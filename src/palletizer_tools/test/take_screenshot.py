#!/usr/bin/env python3
"""Renders an overview image of the cell for the README (docs/world.png).

Stages a scene (boxes on the belt and pallet, arm above the pick zone), spawns a temporary
static camera, bridges its image, and saves it. Needs a running sim (gui not required).
Usage: take_screenshot.py [output.png]
"""

import os
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from ros_gz_interfaces.srv import SpawnEntity
from sensor_msgs.msg import Image
from sim_helpers import SimClient
from std_srvs.srv import Trigger

CAM_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="doc_camera">
    <static>true</static>
    <link name="link">
      <sensor name="doc_cam" type="camera">
        <always_on>true</always_on>
        <update_rate>5</update_rate>
        <topic>/doc_camera/image</topic>
        <camera>
          <horizontal_fov>1.0</horizontal_fov>
          <image><width>1280</width><height>800</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>30</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""


def look_at(eye, target):
    """(x, y, z, w) quaternion for a gz camera (looks along +x) at eye looking at target."""
    from scipy.spatial.transform import Rotation
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    left = np.cross([0, 0, 1], f)
    left /= np.linalg.norm(left)
    up = np.cross(f, left)
    return Rotation.from_matrix(np.column_stack([f, left, up])).as_quat()


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        '~/palletizer_ws/docs/world.png')
    rclpy.init()
    sim = SimClient('doc_screenshot')
    sim.wait_ready()
    sim.delete_all_boxes()

    # Stage: a few boxes on the pallet (ground-truth pallet pose: this is a picture, not
    # robot control), boxes on the belt, arm above the pick zone.
    sim.wait_for(lambda: sim.station_pallet() is not None, 60.0)
    _, pxyz, _ = sim.station_pallet()
    ptype = sim.cell['exchanger']['pallet_types'][sim.cell['exchanger']['order'][0]]['size']
    ox, oy, top = pxyz[0] - ptype[0] / 2, pxyz[1] + ptype[1] / 2, ptype[2]
    stacked = [((0.40, 0.30, 0.25), (ox + 0.22, oy - 0.17)), ((0.40, 0.30, 0.25), (ox + 0.64, oy - 0.17)),
               ((0.35, 0.45, 0.30), (ox + 0.20, oy - 0.57)), ((0.50, 0.40, 0.30), (ox + 0.70, oy - 0.57)),
               ((0.38, 0.30, 0.20), (ox + 0.22, oy - 0.17))]
    heights = {}
    for i, (size, (x, y)) in enumerate(stacked):
        z0 = heights.get((round(x, 2), round(y, 2)), top)
        rgba = (0.74 - 0.04 * i, 0.57 - 0.02 * i, 0.36, 1.0)
        sim.spawn_box(f'box_doc_p{i}', size, 3.0, (x, y, z0 + size[2] / 2 + 0.002), 0.0, rgba)
        heights[(round(x, 2), round(y, 2))] = z0 + size[2]

    sim.call(sim.resume_cli, Trigger.Request())
    for i in range(3):
        sim.call(sim.spawn_box_cli, Trigger.Request())
        time.sleep(5.0)  # ~1.25 m spacing on the belt
    sim.move_joints(sim.pick_approach_joints())

    # Temporary overview camera.
    eye, target = (-2.8, -3.2, 3.4), (-0.4, 0.3, 1.25)
    req = SpawnEntity.Request()
    req.entity_factory.name = 'doc_camera'
    req.entity_factory.sdf = CAM_SDF
    req.entity_factory.pose.position.x, req.entity_factory.pose.position.y, \
        req.entity_factory.pose.position.z = eye
    q = look_at(eye, target)
    o = req.entity_factory.pose.orientation
    o.x, o.y, o.z, o.w = (float(v) for v in q)
    sim.call(sim.spawn_cli, req)

    bridge_proc = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '/doc_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    frames = []
    sub = sim.create_subscription(Image, '/doc_camera/image', frames.append, 5)
    ok = sim.wait_for(lambda: len(frames) >= 3, 30.0)
    sim.destroy_subscription(sub)
    os.killpg(bridge_proc.pid, signal.SIGINT)     # ros2 run spawns a child: kill the group
    bridge_proc.wait(timeout=10)
    if ok:
        img = CvBridge().imgmsg_to_cv2(frames[-1], 'bgr8')
        os.makedirs(os.path.dirname(out), exist_ok=True)
        cv2.imwrite(out, img)
        print(f'saved {out}')
    else:
        print('no image received from the doc camera')

    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import DeleteEntity
    dreq = DeleteEntity.Request()
    dreq.entity.name = 'doc_camera'
    dreq.entity.type = Entity.MODEL
    sim.call(sim.delete_cli, dreq)
    sim.shutdown()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
