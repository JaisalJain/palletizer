"""Simulated pallet conveyor / forklift (plant equipment, NOT robot control).

Action /pallet_exchange (PalletExchange): the full pallet at the station slides out along
+x together with every box on it (PalletMover gz system, kinematic), is removed at the end
of the lane, and a new empty pallet (next type of the order) slides in from the -x end and
stops at the station centre + a small random pose offset. The robot is never told the
pallet type or pose: it measures them (pallet_perception).

Being simulated equipment, this node may use ground truth (/sim/pallet_poses,
/sim/box_poses) to know which models to move. Nothing here feeds perception, planning or
control. The mover drives the models only during a move: between exchanges the boxes on
the pallet are free physics bodies.

Params: exchange_mode auto | operator (operator: each exchange waits for
        /pallet/confirm_exchange), pallets "EUR,HALF,..." (types in turn, last repeats;
        default exchanger.order; a type may also be raw dims in mm "1100x850x144"),
        initial (bring the first pallet in at startup), seed, cell_config
Services: /pallet/confirm_exchange (Trigger, operator mode)
Topics: ~/state (String): idle | <phase>
"""

import math
import threading
import time

import numpy as np
import rclpy
from palletizer_interfaces.action import PalletExchange
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from palletizer_tools import boxes, cell_config, sim_time

WOOD = (0.72, 0.56, 0.36, 1.0)


def pallet_sdf(name, size, mass):
    """Block pallet: one collision box (whole volume), visuals: 5 deck boards across the
    width on 3 stringers + 9 blocks (EUR-like). The deck has gaps like a real pallet."""
    L, W, H = size
    vis = []

    def vbox(n, x, y, z, sx, sy, sz):
        vis.append(f"""
      <visual name="{n}">
        <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material><ambient>{' '.join(map(str, WOOD))}</ambient><diffuse>{' '.join(map(str, WOOD))}</diffuse>
        <specular>0.1 0.1 0.1 1</specular></material>
      </visual>""")
    board, t = 0.1, 0.022
    for i in range(5):
        vbox(f'deck_{i}', 0, -W / 2 + board / 2 + i * (W - board) / 4, H - t / 2, L, board, t)
    for j in range(3):
        y = -W / 2 + board / 2 + j * (W - board) / 2
        vbox(f'stringer_{j}', 0, y, H - 1.5 * t, L, board, t)
        vbox(f'bottom_{j}', 0, y, t / 2, L, board, t)
        for k, x in enumerate((-L / 2 + 0.05, 0.0, L / 2 - 0.05)):
            vbox(f'block_{j}_{k}', x, y, t + (H - 3 * t) / 2, 0.1 if k != 1 else 0.14, board,
                 H - 3 * t)
    ixx = mass * (W * W + H * H) / 12
    iyy = mass * (L * L + H * H) / 12
    izz = mass * (L * L + W * W) / 12
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <inertial><mass>{mass}</mass><inertia><ixx>{ixx}</ixx><ixy>0</ixy><ixz>0</ixz>
        <iyy>{iyy}</iyy><iyz>0</iyz><izz>{izz}</izz></inertia></inertial>
      <collision name="collision">
        <pose>0 0 {H / 2:.4f} 0 0 0</pose>
        <geometry><box><size>{L:.4f} {W:.4f} {H:.4f}</size></box></geometry>
        <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>
      </collision>{''.join(vis)}
    </link>
  </model>
</sdf>
"""


def parse_pallet_type(key, ex_cfg):
    """Type name from exchanger.pallet_types, or raw 'LxWxH' in mm -> (size, mass)."""
    key = key.strip()
    types = ex_cfg['pallet_types']
    if key in types:
        return tuple(float(v) for v in types[key]['size']), float(types[key]['mass'])
    dims = [float(v) / 1000.0 for v in key.lower().split('x')]
    if len(dims) != 3:
        raise ValueError(f'unknown pallet type "{key}"')
    return (max(dims[:2]), min(dims[:2]), dims[2]), 20.0


class PalletExchanger(Node):

    def __init__(self):
        super().__init__('pallet_exchanger')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None)
        self.ex = self.cell['exchanger']
        self.st = self.cell['pallet_station']
        self.mode = self.declare_parameter('exchange_mode', 'auto').value
        order = self.declare_parameter('pallets', ','.join(self.ex['order'])).value
        self.order = [o.strip() for o in str(order).split(',') if o.strip()]
        for o in self.order:
            parse_pallet_type(o, self.ex)             # fail early on a bad type
        initial = bool(self.declare_parameter('initial', False).value)
        seed = int(self.declare_parameter('seed', -1).value)
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.delivered = 0

        self.pallet_poses = {}
        self.box_poses = {}
        self.mover_state = ''
        self.pallets = {}           # name -> size of pallets this node spawned
        self.confirm = threading.Event()
        cb = ReentrantCallbackGroup()
        self.create_subscription(TFMessage, '/sim/pallet_poses', self.on_pallets, 10,
                                 callback_group=cb)
        self.create_subscription(TFMessage, '/sim/box_poses', self.on_boxes, 10, callback_group=cb)
        self.create_subscription(String, '/sim/pallet_mover/state', self.on_mover, 10,
                                 callback_group=cb)
        self.mover_pub = self.create_publisher(String, '/sim/pallet_mover/cmd', 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        world = self.cell['world_name']
        self.spawn_cli = self.create_client(SpawnEntity, f'/world/{world}/create',
                                            callback_group=MutuallyExclusiveCallbackGroup())
        self.delete_cli = self.create_client(DeleteEntity, f'/world/{world}/remove',
                                             callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(Trigger, '/pallet/confirm_exchange', self.on_confirm,
                            callback_group=cb)
        self.busy = threading.Lock()
        self.server = ActionServer(
            self, PalletExchange, '/pallet_exchange', self.execute, callback_group=cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,      # queued behind a running one
            cancel_callback=lambda _: CancelResponse.REJECT)
        self.move_id = int(time.time()) % 100000 * 10
        if initial:
            threading.Thread(target=self.initial_pallet, daemon=True).start()
        self.get_logger().info(f'pallet_exchanger ready ({self.mode}, pallets {self.order})')

    # ------------------------------------------------------------------ inputs
    def on_pallets(self, msg):
        self.pallet_poses = boxes.poses_from_tf(msg, prefix='pallet_')

    def on_boxes(self, msg):
        self.box_poses = boxes.poses_from_tf(msg)

    def on_mover(self, msg):
        self.mover_state = msg.data

    def on_confirm(self, request, response):
        self.confirm.set()
        response.success, response.message = True, 'exchange confirmed'
        return response

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def wait_for(cond, timeout, period=0.05):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(period)
        return cond()

    def wait_sim(self, cond, timeout):
        """Wait for something the simulation does (pallet travel, a model appearing) in sim
        time: a full interlocked pallet drops the RTF to ~0.04, so 8 s of travel take minutes."""
        return sim_time.wait(self, cond, timeout, period=0.05)

    def call(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        return fut.result() if self.wait_for(fut.done, timeout) else None

    def at_station(self):
        """Name of the pallet model at the station (ground truth), or ''."""
        c = np.array(self.st['center_xy'])
        best = ''
        for name, (xyz, _) in self.pallet_poses.items():
            if np.linalg.norm(xyz[:2] - c) < 0.5:
                best = name
        return best

    def boxes_on(self, pallet):
        """Boxes resting on / above the pallet footprint (ground truth)."""
        xyz, quat = self.pallet_poses[pallet]
        L, W, _ = self.pallets.get(pallet, (1.2, 0.8, 0.144))
        yaw = 2.0 * math.atan2(quat[2], quat[3])
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for name, (p, _) in self.box_poses.items():
            d = p[:2] - xyz[:2]
            u, v = c * d[0] + s * d[1], -s * d[0] + c * d[1]
            if abs(u) <= L / 2 + 0.15 and abs(v) <= W / 2 + 0.15 and p[2] > 0.05:
                out.append(name)
        return out

    def move(self, names, dx, dy, timeout=120.0):
        self.move_id += 1
        mid = self.move_id
        self.mover_pub.publish(String(data=f'move {mid} {dx:.4f} {dy:.4f} {self.ex["speed"]} '
                                           f'{self.ex["accel"]} ' + ' '.join(names)))
        return self.wait_sim(lambda: self.mover_state == f'idle {mid}', timeout)

    def delete(self, name):
        req = DeleteEntity.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        res = self.call(self.delete_cli, req)
        return res is not None and res.success

    def next_type(self, override=''):
        key = override or self.order[min(self.delivered, len(self.order) - 1)]
        self.delivered += 1
        return key

    def publish_state(self, s):
        self.state_pub.publish(String(data=s))

    # ------------------------------------------------------------------ phases
    def bring_in(self, key, feedback):
        size, mass = parse_pallet_type(key, self.ex)
        name = f'pallet_{int(time.time() * 1000) % 10**9:09d}'
        cx, cy = self.st['center_xy']
        n_xy, n_yaw = self.ex['pose_noise_xy'], self.ex['pose_noise_yaw']
        tx = cx + float(self.rng.uniform(-n_xy, n_xy))
        ty = cy + float(self.rng.uniform(-n_xy, n_xy))
        yaw = float(self.rng.uniform(-n_yaw, n_yaw))
        req = SpawnEntity.Request()
        req.entity_factory.name = name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf = pallet_sdf(name, size, mass)
        p = req.entity_factory.pose
        p.position.x, p.position.y, p.position.z = float(self.ex['entry_x']), ty, 0.0
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        feedback('spawn new pallet', 0.55)
        res = self.call(self.spawn_cli, req)
        if res is None or not res.success:
            return False, f'spawn of {name} failed', name
        self.pallets[name] = size
        if not self.wait_sim(lambda: name in self.pallet_poses, 10.0):
            return False, f'{name} not visible in ground truth', name
        feedback('new pallet in', 0.6)
        if not self.move([name], tx - self.ex['entry_x'], 0.0):
            return False, 'pallet in: mover timeout', name
        time.sleep(0.5)
        self.get_logger().info(
            f'{name} ({key}, {size[0]:.3f} x {size[1]:.3f} m) at the station: '
            f'({tx:.3f}, {ty:.3f}), yaw {math.degrees(yaw):+.2f} deg')
        return True, f'{key} pallet delivered', name

    def take_out(self, feedback):
        pallet = self.at_station()
        if not pallet:
            return True, 'station empty'
        names = [pallet] + self.boxes_on(pallet)
        feedback(f'full pallet out ({len(names) - 1} boxes)', 0.1)
        xyz = self.pallet_poses[pallet][0]
        if not self.move(names, self.ex['exit_x'] - xyz[0], 0.0):
            return False, 'pallet out: mover timeout'
        feedback('remove full pallet', 0.45)
        ok = all(self.delete(n) for n in names)
        self.pallets.pop(pallet, None)
        self.wait_sim(lambda: pallet not in self.pallet_poses, 5.0)
        return ok, f'{pallet} removed with {len(names) - 1} boxes'

    def exchange(self, feedback, pallet_type=''):
        self.mode = self.get_parameter('exchange_mode').value      # may change at run time
        if self.mode == 'operator':
            feedback('waiting for /pallet/confirm_exchange', 0.02)
            self.confirm.clear()
            while not self.confirm.wait(0.5):
                if not rclpy.ok():
                    return False, 'shutdown', ''
        ok, msg = self.take_out(feedback)
        if not ok:
            return False, msg, ''
        ok, msg2, name = self.bring_in(self.next_type(pallet_type), feedback)
        return ok, f'{msg}; {msg2}', name

    def initial_pallet(self):
        with self.busy:
            if not self.wait_for(lambda: self.mover_state.startswith('idle'), 120.0):
                self.get_logger().error('PalletMover not running')
                return
            time.sleep(1.0)
            if self.at_station():
                return
            ok, msg, _ = self.bring_in(self.next_type(), lambda p, _: self.publish_state(p))
            self.publish_state('idle')
            self.get_logger().info(f'initial pallet: {msg}')

    def execute(self, gh):
        with self.busy:
            def feedback(phase, progress):
                fb = PalletExchange.Feedback()
                fb.phase, fb.progress = phase, float(progress)
                gh.publish_feedback(fb)
                self.publish_state(phase)
                self.get_logger().info(f'[exchange] {phase}')
            if not self.wait_for(lambda: self.mover_state.startswith('idle'), 60.0):
                gh.abort()
                return PalletExchange.Result(success=False, message='PalletMover not running')
            ok, msg, name = self.exchange(feedback, gh.request.pallet_type)
            feedback('done' if ok else 'failed', 1.0)
            self.publish_state('idle')
        res = PalletExchange.Result(success=ok, message=msg, sim_pallet_name=name)
        (gh.succeed if ok else gh.abort)()
        return res


def main():
    rclpy.init()
    node = PalletExchanger()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.spin()


if __name__ == '__main__':
    main()
