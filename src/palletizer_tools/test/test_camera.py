#!/usr/bin/env python3
"""Test 3: camera topics publish at the configured rate; a box shows up in RGB and depth.

Saves test_output/rgb.png, depth.png (colorized) and depth_raw.npy.
"""

import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from sim_helpers import OUTPUT_DIR, run
from std_srvs.srv import Trigger

TOPICS = {
    '/overhead_camera/image': Image,
    '/overhead_camera/depth_image': Image,
    '/overhead_camera/points': PointCloud2,
    '/overhead_camera/camera_info': CameraInfo,
}
BOX = (0.40, 0.30, 0.30)


def collect(topic, typ, duration, max_msgs=None, headers_only=False):
    """Receive messages on a short-lived node spun in this thread."""
    node = rclpy.create_node('camera_test_collector')
    msgs = []
    node.create_subscription(
        typ, topic, lambda m: msgs.append(m.header if headers_only else m), 10)
    end = time.time() + duration
    while time.time() < end and (max_msgs is None or len(msgs) < max_msgs):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    return msgs


def grab(topic):
    msgs = collect(topic, Image, 5.0, max_msgs=1)
    if not msgs:
        raise RuntimeError(f'no message on {topic}')
    return msgs[0]


def test(sim, c):
    cam = sim.cell['camera']
    expected = cam['rate']

    # --- rates (from sim-time header stamps) and frames ---
    # One topic at a time with reliable QoS (best effort drops fragments of large images).
    frames = {}
    for t, typ in TOPICS.items():
        headers = collect(t, typ, 4.0, headers_only=True)
        s = sorted({h.stamp.sec + h.stamp.nanosec * 1e-9 for h in headers})
        rate = (len(s) - 1) / (s[-1] - s[0]) if len(s) > 2 else 0.0
        frames[t] = headers[-1].frame_id if headers else None
        c.check(abs(rate - expected) < 0.2 * expected,
                f'{t}: {rate:.1f} Hz (expected {expected:.0f}), frame "{frames.get(t)}"')
    c.check(frames.get('/overhead_camera/image') == 'overhead_camera_optical_frame',
            'image frame is overhead_camera_optical_frame')
    c.check(frames.get('/overhead_camera/points') == 'overhead_camera_link',
            'point cloud frame is overhead_camera_link (x-forward cloud)')

    # --- a box under the camera ---
    sim.call(sim.stop_cli, Trigger.Request())   # hold all zones: box stays under the camera
    sim.delete_all_boxes()
    time.sleep(1.0)
    empty_depth = CvBridge().imgmsg_to_cv2(grab('/overhead_camera/depth_image'))
    px, py, pz = sim.cell['pick_zone']['xyz']
    sim.spawn_box('box_camtest', BOX, 3.0, (px, py, pz + BOX[2] / 2 + 0.005), yaw=0.2)
    time.sleep(3.0)
    rgb_msg = grab('/overhead_camera/image')
    depth_msg = grab('/overhead_camera/depth_image')

    # Point cloud in its stamped frame (overhead_camera_link) -> world: the center point
    # must land on the box top at the pick zone.
    cloud = collect('/overhead_camera/points', PointCloud2, 5.0, max_msgs=1)[0]
    pts = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z')).reshape(
        cloud.height, cloud.width, 3)
    p_cam = pts[cloud.height // 2, cloud.width // 2]
    R = Rotation.from_euler('xyz', cam['rpy']).as_matrix()
    p_world = R @ p_cam + np.array(cam['xyz'])
    box_top = np.array([px, py, pz + BOX[2]])
    c.check(np.linalg.norm(p_world - box_top) < 0.02,
            f'cloud center point in world {p_world.round(3)} = box top {box_top.round(3)}')

    bridge = CvBridge()
    rgb = bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
    depth = np.array(bridge.imgmsg_to_cv2(depth_msg), dtype=np.float32)
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    cv2.imwrite(os.path.join(OUTPUT_DIR, 'rgb.png'), rgb)
    np.save(os.path.join(OUTPUT_DIR, 'depth_raw.npy'), depth)
    finite = np.where(np.isfinite(depth), depth, cam['far'])
    vis = cv2.applyColorMap(
        cv2.convertScaleAbs(np.clip(finite, 0.8, 1.8) - 0.8, alpha=255.0), cv2.COLORMAP_JET)
    cv2.imwrite(os.path.join(OUTPUT_DIR, 'depth.png'), vis)
    c.check(os.path.getsize(os.path.join(OUTPUT_DIR, 'rgb.png')) > 0,
            f'saved rgb.png / depth.png to {OUTPUT_DIR}')

    belt_depth = cam['xyz'][2] - sim.cell['conveyor']['belt_top_z']
    top_depth = belt_depth - BOX[2]
    center = finite[finite.shape[0] // 2, finite.shape[1] // 2]
    empty_center = float(empty_depth[empty_depth.shape[0] // 2, empty_depth.shape[1] // 2])
    c.check(abs(empty_center - belt_depth) < 0.02,
            f'before spawn: depth at image center {empty_center:.3f} m = belt ({belt_depth:.2f})')
    c.check(abs(center - top_depth) < 0.02,
            f'after spawn: depth at image center {center:.3f} m = box top ({top_depth:.2f})')

    # Box-top blob: pixels at the box-top depth; compare area with the projection.
    mask = (np.abs(finite - top_depth) < 0.02).astype(np.uint8)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    if n > 1:
        i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = stats[i, cv2.CC_STAT_AREA]
        cx, cy = cents[i]
    else:
        area, cx, cy = 0, 0, 0
    fx = (cam['width'] / 2) / np.tan(cam['hfov'] / 2)
    expected_area = (BOX[0] * fx / top_depth) * (BOX[1] * fx / top_depth)
    c.check(abs(area - expected_area) < 0.1 * expected_area,
            f'depth blob area {area} px vs expected {expected_area:.0f} px')
    c.check(abs(cx - cam['width'] / 2) < 10 and abs(cy - cam['height'] / 2) < 10,
            f'depth blob centered at ({cx:.0f}, {cy:.0f}) px')

    # RGB: the cardboard box is clearly brighter/browner than the black belt.
    blob = labels == i if n > 1 else np.zeros(mask.shape, bool)
    box_mean = rgb[blob].mean(axis=0) if blob.any() else np.zeros(3)
    ring = cv2.dilate(blob.astype(np.uint8), np.ones((25, 25), np.uint8)).astype(bool) & ~blob
    bg_mean = rgb[ring].mean(axis=0)
    c.check(box_mean.sum() > bg_mean.sum() + 60,
            f'box visible in RGB (box BGR {box_mean.round()}, surroundings {bg_mean.round()})')
    sim.delete_all_boxes()
    sim.call(sim.resume_cli, Trigger.Request())


if __name__ == '__main__':
    run(test, 'test_camera')
