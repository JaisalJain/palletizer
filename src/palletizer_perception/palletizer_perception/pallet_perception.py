"""Pallet-station perception node (overhead RGB-D above the pallet station).

Services:
  /pallet_perception/measure_pallet  (MeasurePallet)  measure the empty pallet: deck dims,
                                      pose, height; resets the stack model
  /pallet_perception/verify_placement (VerifyPlacement) measure the last placement by change
                                      detection vs the reference frame; also checks the
                                      rest of the stack for shifts
  /pallet_perception/reset (Trigger)  pallet gone (exchange started)
Topics (out):
  /pallet_perception/pallet (DetectedPallet, transient local), TF world -> pallet_frame
      (x = u, y = -v, z up; right-handed)
  /pallet_perception/height_map (HeightMap, transient local) after every measurement
  /pallet_perception/placement_check (PlacementCheck), /pallet_perception/stack_events
      (StackEvent), /pallet_perception/markers (MarkerArray)
  /pallet_perception/debug_image (Image, RGB, 1 Hz while someone listens): pallet camera view
      with the measured pallet outline, measured box tops (red: moved), the NEXT target as a
      ghost rectangle (/packing/next_target) and a coloured height-map inset
Topics (in): /pallet_camera/{depth_image,image,camera_info} (reliable),
  /orchestrator/cycle_report (placed boxes not yet verified are excluded from monitoring)

Frames are used only when no robot link hangs over the station (TF, same check as the
conveyor perception) and no pixel over the station is higher than any stack can be.
Continuous stack monitoring: every monitor_period s with a clear view, every placed box is
compared with its state at placement; shifted > shift_threshold from where it was placed ->
StackEvent SHIFT, top dropped -> COLLAPSE.
"""

import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from palletizer_interfaces.msg import (CycleReport, DetectedPallet, HeightMap, Placement,
                                       PlacementCheck, StackEvent)
from palletizer_interfaces.srv import MeasurePallet, VerifyPlacement
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from palletizer_perception import debug_view as dv
from palletizer_perception.pallet_detector import PalletDetector, normalize_axis
from palletizer_tools import cell_config
from palletizer_tools.pallet_frame import PalletFrame

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def rect_pixels(P, center, yaw, sx, sy, margin=0.0):
    """Pixels whose world xy lies in a rotated rectangle (world size sx along yaw, sy)."""
    c, s = math.cos(yaw), math.sin(yaw)
    d = P[..., :2] - np.asarray(center)[:2]
    a = d[..., 0] * c + d[..., 1] * s
    b = -d[..., 0] * s + d[..., 1] * c
    with np.errstate(invalid='ignore'):
        return (np.abs(a) <= sx / 2 + margin) & (np.abs(b) <= sy / 2 + margin)


class PalletPerception(Node):

    def __init__(self):
        super().__init__('pallet_perception')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None, role='robot')
        self.det = PalletDetector(self.cell)
        self.p = self.cell['pallet_perception']
        self.n_frames = int(self.p['frames'])
        self.bridge = CvBridge()
        self.lock = threading.Lock()      # frames
        self.model = threading.Lock()     # pallet / stack model
        self.frames = []                  # (stamp, depth)
        self.rgb = None
        self.info = None

        self.frame = None                 # PalletFrame of the measured pallet
        self.pallet_msg = None
        self.pallet_id = 0
        self.Z_ref = None                 # reference height image (world z per pixel)
        self.P_ref = None
        self.records = []                 # BoxRecord per verified placement
        self.pending = []                 # placed, not yet verified: (xyz, yaw, sx, sy)
        self.verified = 0
        self.events = 0
        self.last_view = None             # latest clear averaged depth (debug view)
        self.last_map = None              # latest perceived height map (debug view)
        self.next_target = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static = StaticTransformBroadcaster(self)
        sensors = ReentrantCallbackGroup()
        rq = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/pallet_camera/depth_image', self.on_depth, rq,
                                 callback_group=sensors)
        self.create_subscription(Image, '/pallet_camera/image', self.on_rgb, rq,
                                 callback_group=sensors)
        self.create_subscription(CameraInfo, '/pallet_camera/camera_info', self.on_info, 10,
                                 callback_group=sensors)
        self.create_subscription(CycleReport, '/orchestrator/cycle_report', self.on_cycle, 50,
                                 callback_group=sensors)
        self.pallet_pub = self.create_publisher(DetectedPallet, '/pallet_perception/pallet', LATCHED)
        self.map_pub = self.create_publisher(HeightMap, '/pallet_perception/height_map', LATCHED)
        self.check_pub = self.create_publisher(PlacementCheck, '/pallet_perception/placement_check',
                                               50)
        self.event_pub = self.create_publisher(StackEvent, '/pallet_perception/stack_events', 50)
        self.marker_pub = self.create_publisher(MarkerArray, '/pallet_perception/markers', 10)
        self.debug_pub = self.create_publisher(Image, '/pallet_perception/debug_image', LATCHED)
        self.create_subscription(Placement, '/packing/next_target',
                                 lambda m: setattr(self, 'next_target', m), LATCHED,
                                 callback_group=sensors)
        self.create_timer(1.0, self.publish_debug, callback_group=sensors)
        work = MutuallyExclusiveCallbackGroup()
        self.create_service(MeasurePallet, '/pallet_perception/measure_pallet', self.on_measure,
                            callback_group=work)
        self.create_service(VerifyPlacement, '/pallet_perception/verify_placement',
                            self.on_verify, callback_group=work)
        self.create_service(Trigger, '/pallet_perception/reset', self.on_reset, callback_group=work)
        self.create_timer(float(self.p['monitor_period']), self.monitor, callback_group=work)
        self.get_logger().info('pallet_perception ready')

    # ------------------------------------------------------------------ inputs
    def on_depth(self, msg):
        d = np.array(self.bridge.imgmsg_to_cv2(msg), dtype=np.float32)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.frames.append((stamp, d))
            self.frames = self.frames[-self.n_frames * 3:]

    def on_rgb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def on_info(self, msg):
        self.info = msg

    def on_cycle(self, r):
        if r.success:
            pl = r.placement
            self.pending.append((np.array([pl.top_center.position.x, pl.top_center.position.y,
                                           pl.top_center.position.z]), pl.yaw,
                                 max(pl.size_x, pl.size_y), min(pl.size_x, pl.size_y)))

    # ------------------------------------------------------------------ frames
    def robot_over_station(self):
        m = self.p['occlusion_margin']
        c, a = self.det.station_c, self.det.station_a / 2 + m
        for link in self.p['occlusion_links']:
            try:
                t = self.tf_buffer.lookup_transform('world', link, rclpy.time.Time())
            except Exception:  # noqa: BLE001
                continue
            tr = t.transform.translation
            if abs(tr.x - c[0]) <= a[0] and abs(tr.y - c[1]) <= a[1] and tr.z > 0.0:
                return link
        return ''

    def acquire(self, timeout):
        """Averaged depth of n fresh frames taken while the station is clear, or None."""
        t_wall = time.monotonic()
        while time.monotonic() - t_wall < timeout:
            if self.robot_over_station() or self.info is None:
                time.sleep(0.1)
                continue
            t_req = self.get_clock().now().nanoseconds * 1e-9
            while time.monotonic() - t_wall < timeout:
                with self.lock:
                    fresh = [d for t, d in self.frames if t >= t_req]
                if len(fresh) >= self.n_frames:
                    break
                time.sleep(0.05)
            else:
                return None
            if self.robot_over_station():
                continue
            info = self.info
            self.det.cam.set_intrinsics(info.k, info.width, info.height)
            stack = np.stack(fresh[-self.n_frames:])
            with np.errstate(invalid='ignore'):
                depth = np.nanmean(np.where(np.isfinite(stack), stack, np.nan), axis=0)
            P = self.det.points(depth)
            if self.det.occluded(P, self.frame.deck_z if self.frame else None):
                time.sleep(0.1)
                continue
            self.last_view = (depth, P)
            return depth, P
        return None

    # ------------------------------------------------------------------ services
    def on_measure(self, req, res):
        view = self.acquire(req.timeout if req.timeout > 0 else self.p['measure_timeout'])
        if view is None:
            res.success, res.message = False, 'no unoccluded view of the station'
            return res
        depth, P = view
        m, msg = self.det.detect_pallet(depth)
        if m is None:
            res.success, res.message = False, msg
            return res
        with self.model:
            self.frame = m.frame
            self.pallet_id += 1
            self.Z_ref, self.P_ref = P[..., 2].copy(), P
            self.records, self.pending, self.verified, self.events = [], [], 0, 0
            pm = DetectedPallet()
            pm.header.stamp = self.get_clock().now().to_msg()
            pm.header.frame_id = 'world'
            pm.valid = True
            pm.confidence = m.confidence
            m.frame.fill_msg(pm)
            self.pallet_msg = pm
        self.pallet_pub.publish(pm)
        self.publish_tf(m.frame)
        self.publish_map(P)
        self.publish_markers()
        res.success, res.pallet = True, pm
        f = m.frame
        res.message = (f'pallet {f.size_u * 1000:.1f} x {f.size_v * 1000:.1f} mm, deck '
                       f'z {f.deck_z * 1000:.1f} mm, centre ({f.center[0]:.4f}, {f.center[1]:.4f}),'
                       f' yaw {math.degrees(f.yaw):+.2f} deg, conf {m.confidence:.2f}')
        self.get_logger().info(res.message)
        return res

    def on_reset(self, req, res):
        with self.model:
            self.frame = None
            self.Z_ref = None
            self.records, self.pending = [], []
            pm = DetectedPallet()
            pm.header.stamp = self.get_clock().now().to_msg()
            pm.header.frame_id = 'world'
            pm.valid = False
            self.pallet_msg = pm
        self.pallet_pub.publish(pm)
        self.publish_markers()
        res.success, res.message = True, 'pallet model cleared'
        return res

    def take_pending(self, xyz):
        if not self.pending:
            return
        i = int(np.argmin([np.linalg.norm(p[0][:2] - xyz[:2]) for p in self.pending]))
        self.pending.pop(i)

    def pending_mask(self, P):
        mask = np.zeros(P.shape[:2], bool)
        for xyz, yaw, sx, sy in self.pending:
            mask |= rect_pixels(P, xyz, yaw, sx, sy, 0.05)
        return mask

    def on_verify(self, req, res):
        pl = req.placement
        tgt = np.array([pl.top_center.position.x, pl.top_center.position.y,
                        pl.top_center.position.z])
        check = PlacementCheck()
        check.header.frame_id = 'world'
        check.pallet_id = self.pallet_id
        check.index = req.index
        check.commanded = pl
        check.sim_model_name = req.sim_model_name
        if self.frame is None:
            res.success, res.message = False, 'no pallet measured'
            return res
        view = self.acquire(req.timeout if req.timeout > 0 else self.p['measure_timeout'])
        if view is None:
            res.success, res.message = False, 'no unoccluded view of the station'
            return res
        depth, P = view
        Z = P[..., 2]
        L, W = max(req.box.length, req.box.width), min(req.box.length, req.box.width)
        with self.model:
            self.take_pending(tgt)
            found, msg = self.det.new_box(self.Z_ref, Z, P, tgt, pl.yaw, (pl.size_x, pl.size_y),
                                          pl.height)
            if found is not None:
                r, z, mask = found
                check.detected = True
                check.measured_top.position.x, check.measured_top.position.y = \
                    float(r.center[0]), float(r.center[1])
                check.measured_top.position.z = z
                check.measured_top.orientation.z = math.sin(r.yaw / 2)
                check.measured_top.orientation.w = math.cos(r.yaw / 2)
                check.measured_yaw, check.measured_length, check.measured_width = \
                    r.yaw, r.length, r.width
                check.err_xy = float(np.linalg.norm(r.center - tgt[:2]))
                check.err_z = float(z - tgt[2])
                period = math.pi / 2 if abs(L - W) < 0.01 else math.pi
                check.err_yaw = abs(normalize_axis(r.yaw - pl.yaw, period))
                rec = self.det.record(req.index, np.r_[r.center, z], r.yaw, r.length, r.width,
                                      mask, tgt, Z, req.sim_model_name)
                check.message = (f'placement {req.index}: error {check.err_xy * 1000:.1f} mm xy, '
                                 f'{check.err_z * 1000:+.1f} mm z, '
                                 f'{math.degrees(check.err_yaw):.2f} deg')
            else:
                check.detected = False
                check.message = f'placement {req.index}: not measured ({msg})'
                # keep monitoring it at its commanded footprint
                zc = self.det.cam.plane_points(np.ones(Z.shape, bool), tgt[2])
                mask = rect_pixels(zc.reshape(Z.shape + (2,)), tgt, pl.yaw, L, W)
                rec = self.det.record(req.index, tgt.copy(), pl.yaw, L, W, mask, tgt, Z,
                                      req.sim_model_name)
            k = self.det._k(0.01, tgt[2]) * 2 + 1
            excl = cv2.dilate(rec.mask.astype(np.uint8), np.ones((k, k), np.uint8)).astype(bool)
            excl |= self.pending_mask(P)
            events = self.det.check_shifts(Z, self.records, exclude=excl)
            self.records.append(rec)
            self.Z_ref, self.P_ref = Z.copy(), P       # reference for the next placement
            self.verified = len(self.records)
        check.header.stamp = self.get_clock().now().to_msg()
        self.check_pub.publish(check)
        self.publish_events(events)
        self.publish_map(P)
        self.publish_markers()
        self.get_logger().info(check.message)
        res.success, res.message, res.check = True, check.message, check
        return res

    # ------------------------------------------------------------------ monitoring
    def monitor(self):
        if self.frame is None or not self.records:
            return
        # a short window (n frames need n / rate s of sim time; RTF < 1 stretches that):
        # gives up quickly when the arm is in the view
        view = self.acquire(3.0 * (self.n_frames + 2) / self.cell['pallet_camera']['rate'])
        if view is None:
            return
        _, P = view
        Z = P[..., 2]
        with self.model:
            if self.frame is None:
                return
            excl = self.pending_mask(P)
            events = self.det.check_shifts(Z, self.records, exclude=excl)
            self.get_logger().debug(f'monitor: {len(self.records)} boxes, {len(self.pending)} '
                                    f'pending ({int(excl.sum())} px excluded), '
                                    f'{len(events)} events')
        if events:
            self.publish_events(events)
            self.publish_map(P)
            self.publish_markers()

    def publish_events(self, events):
        for e in events:
            self.events += 1
            rec = next((r for r in self.records if r.index == e.index), None)
            m = StackEvent()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'world'
            m.pallet_id = self.pallet_id
            m.index = e.index
            m.kind = e.kind
            m.shift = e.shift if math.isfinite(e.shift) else -1.0
            m.dz = e.dz
            if rec is not None:
                m.position = Point(x=float(rec.commanded[0]), y=float(rec.commanded[1]),
                                   z=float(rec.commanded[2]))
                m.sim_model_name = rec.sim_name
                rec.events.append(e.kind)
                if e.kind == 'SHIFT':
                    rec.center = e.new_center
            m.message = e.message
            self.event_pub.publish(m)
            self.get_logger().warn(f'STACK EVENT {e.kind}: {e.message}')

    # ------------------------------------------------------------------ outputs
    def publish_tf(self, f: PalletFrame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'pallet_frame'
        t.transform.translation.x, t.transform.translation.y = map(float, f.origin)
        t.transform.translation.z = float(f.deck_z)
        t.transform.rotation.z = math.sin(f.yaw / 2)
        t.transform.rotation.w = math.cos(f.yaw / 2)
        self.tf_static.sendTransform(t)

    def publish_map(self, P):
        if self.frame is None:
            return
        H = self.det.height_map(P, self.frame)
        self.last_map = H
        m = HeightMap()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'pallet_frame'
        m.pallet = self.pallet_msg
        m.resolution = self.det.res
        m.rows, m.cols = H.shape
        m.data = H.ravel().tolist()
        m.placements = self.verified
        self.map_pub.publish(m)

    def publish_debug(self):
        if self.debug_pub.get_subscription_count() == 0 or self.rgb is None or \
                self.info is None:
            return
        self.det.cam.set_intrinsics(self.info.k, self.info.width, self.info.height)
        cam = self.det.cam
        img = np.ascontiguousarray(self.rgb.copy())
        f = self.frame
        lines = []
        if f is None:
            lines.append('no pallet measured')
        else:
            dv.poly(img, cam.project(np.array(f.corners())), (60, 170, 255), 2)
            o = cam.project(f.to_world(0, 0))[0]
            dv.arrow(img, o, cam.project(f.to_world(0.25, 0))[0], (255, 60, 60), 2)
            dv.arrow(img, o, cam.project(f.to_world(0, 0.25))[0], (60, 255, 60), 2)
            for rec in list(self.records):
                c, s_ = math.cos(rec.yaw), math.sin(rec.yaw)
                ex, ey = np.array([c, s_]), np.array([-s_, c])
                pts = [np.r_[rec.center[:2] + a * rec.length / 2 * ex + b * rec.width / 2 * ey,
                             rec.center[2]] for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
                dv.poly(img, cam.project(np.array(pts)),
                        (255, 50, 50) if rec.events else (40, 255, 90), 2)
            t = self.next_target
            if t is not None and t.height > 0:
                c, s_ = math.cos(t.yaw), math.sin(t.yaw)
                L, W = (t.size_y, t.size_x) if t.rotated else (t.size_x, t.size_y)
                ex, ey = np.array([c, s_]), np.array([-s_, c])
                ctr = np.array([t.top_center.position.x, t.top_center.position.y])
                pts = [np.r_[ctr + a * L / 2 * ex + b * W / 2 * ey, t.top_center.position.z]
                       for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
                uv = cam.project(np.array(pts))
                dv.fill_poly(img, uv, (255, 255, 255), 0.35)
                dv.poly(img, uv, (255, 255, 255), 2)
                dv.crosshair(img, cam.project(np.array([ctr[0], ctr[1],
                                                        t.top_center.position.z]))[0],
                             (255, 255, 255), 8, 1)
            lines += [f'pallet {f.size_u * 1000:.0f} x {f.size_v * 1000:.0f} mm, deck '
                      f'{f.deck_z * 1000:.0f} mm, yaw {math.degrees(f.yaw):+.2f} deg',
                      f'placements verified {self.verified}   stack events {self.events}',
                      'white: next target']
            if self.last_map is not None:
                st = self.cell['pallet_station']['max_stack_height']
                inset = dv.heightmap_inset(self.last_map, st, 2)
                x = img.shape[1] - inset.shape[1] - 10
                y = img.shape[0] - inset.shape[0] - 24
                dv.paste(img, inset, (x, y), f'perceived height map 0-{st:.1f} m')
        dv.text_block(img, lines, (8, 8))
        msg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'pallet_camera_optical_frame'
        self.debug_pub.publish(msg)

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        if self.frame is not None:
            m = Marker()
            m.header.frame_id = 'world'
            m.ns = 'pallet_outline'
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.012
            m.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=1.0)
            cs = self.frame.corners()
            m.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2]) + 0.003)
                        for c in cs + [cs[0]]]
            arr.markers.append(m)
            for i, rec in enumerate(self.records):
                b = Marker()
                b.header.frame_id = 'world'
                b.ns = 'measured_tops'
                b.id = i
                b.type = Marker.LINE_STRIP
                b.scale.x = 0.006
                b.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0) if rec.events else \
                    ColorRGBA(r=0.2, g=1.0, b=0.4, a=1.0)
                c, s = math.cos(rec.yaw), math.sin(rec.yaw)
                ex, ey = np.array([c, s]), np.array([-s, c])
                pts = [rec.center[:2] + a * rec.length / 2 * ex + bb * rec.width / 2 * ey
                       for a, bb in ((1, 1), (-1, 1), (-1, -1), (1, -1), (1, 1))]
                b.points = [Point(x=float(q[0]), y=float(q[1]), z=float(rec.center[2]) + 0.004)
                            for q in pts]
                arr.markers.append(b)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = PalletPerception()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.spin()


if __name__ == '__main__':
    main()
