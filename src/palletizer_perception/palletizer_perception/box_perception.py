"""Overhead RGB-D box perception node.

Inputs (reliable QoS): /overhead_camera/depth_image, /overhead_camera/camera_info,
/overhead_camera/image (only used to split equal-height touching boxes),
/conveyor_controller/box_ready (Bool, trigger).
Outputs: /perception/detected_box (DetectedBox), /perception/markers (MarkerArray),
         /perception/debug_image (Image, RGB): segmentation tint, top-face contour, fitted
         rectangle, grasp point, yaw arrow, measured L x W x H / yaw / confidence and whether
         the box matches the current SKU (/pallet/status)
Service: /perception/detect_box (DetectBox) - averages `perception.frames` fresh depth frames.
"""

import math
import threading

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from palletizer_interfaces.msg import DetectedBox, PalletStatus
from palletizer_interfaces.srv import DetectBox
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener

from palletizer_perception import debug_view as dv
from palletizer_perception.detector import BoxDetector
from palletizer_tools import cell_config, sim_time


class BoxPerception(Node):

    def __init__(self):
        super().__init__('box_perception')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None, role='robot')
        self.det = BoxDetector(self.cell)
        self.n_frames = int(self.cell['perception']['frames'])
        self.timeout = float(self.cell['perception']['detect_timeout'])
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frames = []          # (recv_time, depth ndarray)
        self.rgb = None
        self.info = None
        self.next_id = 1
        self.last_ready = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        sensors = ReentrantCallbackGroup()
        self.create_subscription(Image, '/overhead_camera/depth_image', self.on_depth, 10,
                                 callback_group=sensors)
        self.create_subscription(Image, '/overhead_camera/image', self.on_rgb, 10,
                                 callback_group=sensors)
        self.create_subscription(CameraInfo, '/overhead_camera/camera_info', self.on_info, 10,
                                 callback_group=sensors)
        trig = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Bool, '/conveyor_controller/box_ready', self.on_ready, 10,
                                 callback_group=trig)
        self.box_pub = self.create_publisher(DetectedBox, '/perception/detected_box', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/markers', 10)
        self.debug_pub = self.create_publisher(
            Image, '/perception/debug_image',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.sku_tol = float(self.cell['packing']['sku_tolerance'])
        self.status = None
        self.create_subscription(
            PalletStatus, '/pallet/status', lambda m: setattr(self, 'status', m),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(DetectBox, '/perception/detect_box', self.on_detect_srv,
                            callback_group=trig)
        self.get_logger().info('box_perception ready')

    # ---------------------------------------------------------------- inputs
    def on_depth(self, msg):
        d = np.array(self.bridge.imgmsg_to_cv2(msg), dtype=np.float32)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9    # sim time
        with self.lock:
            self.frames.append((stamp, d))
            self.frames = self.frames[-self.n_frames * 3:]

    def on_rgb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def on_info(self, msg):
        self.info = msg

    def on_ready(self, msg):
        if msg.data and not self.last_ready:
            ok, text, box = self.detect()
            if ok:
                self.box_pub.publish(box)
            else:
                self.get_logger().warn(f'auto-detect failed: {text}')
        self.last_ready = msg.data

    def on_detect_srv(self, request, response):
        response.success, response.message, box = self.detect()
        if response.success:
            response.box = box
            self.box_pub.publish(box)
        return response

    # ---------------------------------------------------------------- detection
    def robot_in_view(self):
        """Name of a tool link hanging over the pick ROI (from TF), or ''. (Higher arm links
        are caught by occluded(): they matter only if they actually cover the ROI.)"""
        p = self.cell['perception']
        m = p['occlusion_margin']
        x0, x1 = self.det.roi_x[0] - m, self.det.roi_x[1] + m
        y0, y1 = self.det.roi_y[0] - m, self.det.roi_y[1] + m
        for link in p['occlusion_links']:
            try:
                t = self.tf_buffer.lookup_transform('world', link, rclpy.time.Time())
            except Exception:  # noqa: BLE001
                continue
            tr = t.transform.translation
            if x0 <= tr.x <= x1 and y0 <= tr.y <= y1 and tr.z > self.det.belt_z:
                return link
        return ''

    def detect(self):
        link = self.robot_in_view()
        if link:
            return False, f'robot in the camera view ({link} over the pick zone)', DetectedBox()
        # Fresh = rendered after this request (header stamp, sim time). Arrival time is not
        # enough: frames queued while the node was busy would show the box still moving.
        t_req = sim_time.now(self)

        def have():
            with self.lock:
                return sum(1 for t, _ in self.frames if t >= t_req) >= self.n_frames and \
                    self.info is not None
        # timeout in sim time (the RTF can drop far below 1 with big stacks)
        if not sim_time.wait(self, have, self.timeout, period=0.02):
            return False, 'no fresh depth frames', DetectedBox()
        with self.lock:
            fresh = [d for t, d in self.frames if t >= t_req]

        info = self.info
        self.det.set_intrinsics(info.k, info.width, info.height)
        stack = np.stack(fresh[-self.n_frames:])
        with np.errstate(invalid='ignore'):
            depth = np.nanmean(np.where(np.isfinite(stack), stack, np.nan), axis=0)
        if self.det.occluded(depth):
            self.publish_debug([], None, None, 'robot in the camera view')
            return False, 'robot in the camera view (something above the pick ROI)', DetectedBox()
        faces, dbg = self.det.detect(depth, self.rgb)
        face = self.det.pick_candidate(faces)
        if face is None:
            self.publish_markers(faces, None)
            self.publish_debug(faces, None, dbg, f'no confident box ({len(faces)} faces)')
            return False, f'no confident box in the pick ROI ({len(faces)} faces)', DetectedBox()

        box = DetectedBox()
        box.header.stamp = self.get_clock().now().to_msg()
        box.header.frame_id = 'world'
        box.id = self.next_id
        self.next_id += 1
        box.length, box.width, box.height = face.length, face.width, face.height
        box.top_center.position.x, box.top_center.position.y, box.top_center.position.z = \
            (float(v) for v in face.center)
        box.top_center.orientation.z = math.sin(face.yaw / 2)
        box.top_center.orientation.w = math.cos(face.yaw / 2)
        box.yaw = face.yaw
        box.confidence = face.confidence
        self.publish_markers(faces, face)
        self.publish_debug(faces, face, dbg, '')
        self.get_logger().info(
            f'box {box.id}: {face.length * 1000:.0f} x {face.width * 1000:.0f} x '
            f'{face.height * 1000:.0f} mm, center ({face.center[0]:.3f}, {face.center[1]:.3f}), '
            f'yaw {math.degrees(face.yaw):+.1f} deg, conf {face.confidence:.2f} {face.note}')
        return True, 'ok', box

    def sku_text(self, face):
        s = self.status
        if s is None or s.mode != 'single_sku':
            return 'SKU: - (mixed mode)', (200, 200, 200)
        if s.sku_length <= 0 or s.boxes_placed == 0:
            return 'SKU: first box of the pallet (defines the SKU)', (120, 200, 255)
        d = (abs(face.length - s.sku_length), abs(face.width - s.sku_width),
             abs(face.height - s.sku_height))
        ref = f'{s.sku_length * 1000:.0f}x{s.sku_width * 1000:.0f}x{s.sku_height * 1000:.0f}'
        if max(d) <= self.sku_tol:
            return f'SKU match ({ref}, max diff {max(d) * 1000:.1f} mm)', (80, 255, 80)
        return f'SKU CHANGE (current {ref}, diff {max(d) * 1000:.0f} mm)', (255, 90, 60)

    def publish_debug(self, faces, chosen, dbg, message):
        """Conveyor camera view with the detection drawn in (only when someone listens)."""
        if self.debug_pub.get_subscription_count() == 0 or self.rgb is None or \
                self.det._K is None:
            return
        img = np.ascontiguousarray(self.rgb.copy())
        z = self.det.belt_z
        roi = [(self.det.roi_x[0], self.det.roi_y[0], z), (self.det.roi_x[1], self.det.roi_y[0], z),
               (self.det.roi_x[1], self.det.roi_y[1], z), (self.det.roi_x[0], self.det.roi_y[1], z)]
        dv.poly(img, self.det.project(roi), (255, 255, 0), 1)
        if dbg is not None:
            dv.tint(img, dbg['object'], (60, 140, 255), 0.35)
        for f in faces:
            if f is not chosen:
                dv.poly(img, self.det.project(f.corners), (255, 150, 0), 2)
        lines, colors = [], []
        if chosen is not None:
            dv.tint(img, chosen.mask, (0, 255, 90), 0.35)
            dv.contour(img, chosen.mask, (255, 255, 0), 1)
            dv.poly(img, self.det.project(chosen.corners), (0, 255, 0), 2)
            c = self.det.project(chosen.center)[0]
            dv.crosshair(img, c, (255, 0, 0), 12, 2)
            tip = chosen.center + np.array([math.cos(chosen.yaw), math.sin(chosen.yaw), 0.0]) * \
                (chosen.length / 2)
            dv.arrow(img, c, self.det.project(tip)[0], (255, 0, 255), 2)
            sku, sku_c = self.sku_text(chosen)
            lines = [f'{chosen.length * 1000:.0f} x {chosen.width * 1000:.0f} x '
                     f'{chosen.height * 1000:.0f} mm', f'yaw {math.degrees(chosen.yaw):+.1f} deg'
                     f'   conf {chosen.confidence:.2f}', sku]
            colors = [(255, 255, 255), (255, 255, 255), sku_c]
        else:
            lines, colors = [f'no detection: {message}'], [(255, 120, 120)]
        dv.text_block(img, lines, (8, 8), colors)
        msg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'overhead_camera_optical_frame'
        self.debug_pub.publish(msg)

    def publish_markers(self, faces, chosen):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, f in enumerate(faces):
            m = Marker()
            m.header.frame_id = 'world'
            m.ns = 'box_outline'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.008
            m.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0) if f is chosen else \
                ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.8)
            m.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2]) + 0.002)
                        for c in list(f.corners) + [f.corners[0]]]
            arr.markers.append(m)
        if chosen is not None:
            g = Marker()
            g.header.frame_id = 'world'
            g.ns = 'grasp_point'
            g.type = Marker.SPHERE
            g.scale.x = g.scale.y = g.scale.z = 0.03
            g.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            g.pose.position = Point(x=float(chosen.center[0]), y=float(chosen.center[1]),
                                    z=float(chosen.center[2]))
            g.pose.orientation.w = 1.0
            arr.markers.append(g)
            t = Marker()
            t.header.frame_id = 'world'
            t.ns = 'dims'
            t.type = Marker.TEXT_VIEW_FACING
            t.scale.z = 0.05
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.pose.position = Point(x=float(chosen.center[0]), y=float(chosen.center[1]),
                                    z=float(chosen.center[2]) + 0.1)
            t.pose.orientation.w = 1.0
            t.text = (f'{chosen.length * 1000:.0f}x{chosen.width * 1000:.0f}x'
                      f'{chosen.height * 1000:.0f} mm  {math.degrees(chosen.yaw):+.1f}deg')
            arr.markers.append(t)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = BoxPerception()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.spin()


if __name__ == '__main__':
    main()
