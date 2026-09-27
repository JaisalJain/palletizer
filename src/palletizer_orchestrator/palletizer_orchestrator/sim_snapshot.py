"""Media snapshots of a running cell (README images). Simulation side only: it spawns fixed
cameras into Gazebo and reads ground truth to pick the moment; it never talks to the robot.

Saved into --out (PNG, full camera resolution):
  hero.png            3/4 view mid-run: a stack in progress, the arm carrying a box (first frame
                      after --hero-after placed boxes with a box on the cup, off the pick zone)
  conveyor_debug.png  /perception/debug_image      } at the same moment (the debug images are
  pallet_debug.png    /pallet_perception/debug_image } rendered only while subscribed)
  full_pallet.png     the finished pallet (close view) when /pallet/status says FULL
  full_hero.png       the same moment, 3/4 view
  exchange/NNN.png    lane view from EXCHANGING until the new pallet is READY, with
  exchange/frames.json  sim time and ground-truth pallet x per frame
  views/<view>.png    --views-only: one frame per camera and exit (camera pose tuning)

Usage: ros2 run palletizer_orchestrator sim_snapshot --out DIR [--hero-after 6]
       (sku_runs --snapshot DIR starts it next to every run)
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
from palletizer_interfaces.msg import CycleReport, PalletStatus
from palletizer_tools.sim_client import SimClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image

# name: (eye, target, hfov [rad]); world frame, see the cell layout in CLAUDE.md
VIEWS = {
    'hero': ((-3.0, -3.4, 3.6), (-0.3, -0.2, 0.9), 0.95),
    'pallet': ((-2.1, -2.8, 2.8), (0.05, -0.6, 0.55), 0.95),
    'lane': ((4.0, -3.3, 2.6), (-0.4, -0.6, 0.35), 1.2),
}
WIDTH, HEIGHT = 1920, 1080

CAM_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <sensor name="{name}_sensor" type="camera">
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>/media/{view}/image</topic>
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>40</far></clip>
          <anti_aliasing>4</anti_aliasing>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""


def look_at(eye, target):
    """(x, y, z, w) quaternion of a gz camera (looks along +x) at `eye` looking at `target`."""
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    left = np.cross([0, 0, 1], f)
    left /= np.linalg.norm(left)
    up = np.cross(f, left)
    return Rotation.from_matrix(np.column_stack([f, left, up])).as_quat()


class Snapshot:
    def __init__(self, sim, out, views, rate):
        self.sim, self.out, self.views = sim, out, views
        self.bridge = CvBridge()
        self.frames = {v: None for v in views}       # latest (sim time, image msg)
        self.count = {v: 0 for v in views}           # frames received
        self.debug = {}
        self.lock = threading.Lock()
        self.placed = 0
        self.status = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        for v in views:
            sim.create_subscription(Image, f'/media/{v}/image',
                                    lambda m, v=v: self._on_frame(v, m), qos)
        sim.create_subscription(CycleReport, '/orchestrator/cycle_report', self._on_cycle, 10)
        sim.create_subscription(
            PalletStatus, '/pallet/status', self._on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.rate = rate
        self.bridge_proc = None

    # --- callbacks
    def _on_frame(self, view, msg):
        with self.lock:
            self.frames[view] = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg)
            self.count[view] += 1

    def _on_cycle(self, msg):
        if msg.success and msg.outcome == 'placed':
            self.placed += 1

    def _on_status(self, msg):
        self.status = msg

    def _on_debug(self, name, msg):
        self.debug[name] = msg

    # --- cameras
    def spawn(self):
        for v in self.views:
            eye, target, hfov = VIEWS[v]
            req = SpawnEntity.Request()
            req.entity_factory.name = f'media_cam_{v}'
            req.entity_factory.sdf = CAM_SDF.format(name=f'media_cam_{v}', view=v, hfov=hfov,
                                                    w=WIDTH, h=HEIGHT, rate=self.rate)
            p = req.entity_factory.pose
            p.position.x, p.position.y, p.position.z = (float(c) for c in eye)
            o = p.orientation
            o.x, o.y, o.z, o.w = (float(c) for c in look_at(eye, target))
            self.sim.call(self.sim.spawn_cli, req)
        args = [f'/media/{v}/image@sensor_msgs/msg/Image[gz.msgs.Image' for v in self.views]
        self.bridge_proc = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + args,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def remove(self):
        if self.bridge_proc and self.bridge_proc.poll() is None:
            os.killpg(self.bridge_proc.pid, signal.SIGINT)   # ros2 run spawns a child
            try:
                self.bridge_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.bridge_proc.pid, signal.SIGKILL)
        for v in self.views:
            req = DeleteEntity.Request()
            req.entity.name = f'media_cam_{v}'
            req.entity.type = Entity.MODEL
            try:
                self.sim.call(self.sim.delete_cli, req, timeout=3.0)
            except Exception:
                pass

    # --- helpers
    def fresh(self, view, timeout=120.0):
        """The next frame of `view` rendered after this call (skips one: it may have been
        rendered before)."""
        n = self.count[view]
        ok = self.sim.wait_for(lambda: self.count[view] >= n + 2, timeout)
        return self.frames[view] if ok else None

    def save(self, msg, name):
        path = os.path.join(self.out, name)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
        print(f'sim_snapshot: saved {path}', flush=True)

    def carrying(self):
        """A box on the cup, TCP off the pick zone (swinging to / above the pallet)."""
        if not self.sim.grasped or self.sim.joints is None:
            return False
        xyz = self.sim.tcp_pose()[:3, 3]
        return xyz[1] < 0.35 and xyz[2] > 1.15


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--hero-after', type=int, default=6, help='placed boxes before the hero shot')
    ap.add_argument('--rate', type=float, default=2.0, help='camera rate [Hz, sim time]')
    ap.add_argument('--views-only', action='store_true')
    ap.add_argument('--view', action='append', default=[], metavar='NAME=ex,ey,ez,tx,ty,tz,hfov',
                    help='override / add a camera (eye, target, hfov); repeatable')
    ap.add_argument('--timeout', type=float, default=7200.0, help='wall [s]')
    args = ap.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    for v in args.view:
        name, vals = v.split('=')
        c = [float(x) for x in vals.split(',')]
        VIEWS[name] = (tuple(c[0:3]), tuple(c[3:6]), c[6])

    import rclpy
    rclpy.init()
    sim = SimClient('sim_snapshot')
    snap = Snapshot(sim, args.out, list(VIEWS), args.rate)
    try:
        sim.wait_ready(timeout=300.0)
        snap.spawn()
        print('sim_snapshot: cameras spawned', flush=True)
        if args.views_only:
            for v in VIEWS:
                f = snap.fresh(v)
                if f:
                    snap.save(f[1], os.path.join('views', f'{v}.png'))
            return 0
        run(snap, sim, args)
    finally:
        snap.remove()
        sim.shutdown()
        rclpy.try_shutdown()
    return 0


def run(snap, sim, args):
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    end = time.time() + args.timeout
    hero_done = full_done = False
    exch = []
    debug_subs = []
    start = time.time()
    while time.time() < end and not (hero_done and full_done and exch and
                                     snap.status is not None and
                                     snap.status.state in (PalletStatus.READY,
                                                           PalletStatus.EMPTY,
                                                           PalletStatus.IN_PROGRESS)):
        st = snap.status.state if snap.status is not None else None
        # debug images: subscribe shortly before the hero moment (rendered on demand)
        if not snap.count['hero'] and time.time() > start + 120:
            print('sim_snapshot: no camera frames yet', flush=True)
            start = time.time()
        if not debug_subs and snap.placed >= max(args.hero_after - 1, 0):
            for name, topic in (('conveyor_debug', '/perception/debug_image'),
                                ('pallet_debug', '/pallet_perception/debug_image')):
                debug_subs.append(sim.create_subscription(
                    Image, topic, lambda m, n=name: snap._on_debug(n, m), qos))
        if not hero_done and snap.placed >= args.hero_after and snap.carrying():
            f = snap.fresh('hero')
            if f and snap.carrying():
                snap.save(f[1], 'hero.png')
                for name, msg in snap.debug.items():
                    snap.save(msg, f'{name}.png')
                hero_done = True
        if not full_done and st == PalletStatus.FULL:
            for v, name in (('pallet', 'full_pallet.png'), ('hero', 'full_hero.png')):
                f = snap.fresh(v)
                if f:
                    snap.save(f[1], name)
            full_done = True
        if full_done and st == PalletStatus.EXCHANGING or (exch and st == PalletStatus.EXCHANGING):
            f = snap.frames['lane']
            if f and (not exch or f[0] > exch[-1]['t']):
                name = os.path.join('exchange', f'{len(exch):03d}.png')
                snap.save(f[1], name)
                xs = {n: float(p[0][0]) for n, p in sim.pallet_poses.items()}
                exch.append({'t': f[0], 'file': name, 'pallet_x': xs})
                with open(os.path.join(snap.out, 'exchange', 'frames.json'), 'w') as fh:
                    json.dump(exch, fh, indent=1)
        if exch and st in (PalletStatus.READY, PalletStatus.EMPTY, PalletStatus.IN_PROGRESS):
            # one last frame: the new pallet in place
            f = snap.fresh('lane')
            if f:
                name = os.path.join('exchange', f'{len(exch):03d}.png')
                snap.save(f[1], name)
                xs = {n: float(p[0][0]) for n, p in sim.pallet_poses.items()}
                exch.append({'t': f[0], 'file': name, 'pallet_x': xs})
                with open(os.path.join(snap.out, 'exchange', 'frames.json'), 'w') as fh:
                    json.dump(exch, fh, indent=1)
            break
        time.sleep(0.05)


if __name__ == '__main__':
    sys.exit(main())
