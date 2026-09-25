"""Shared helpers for the v0 integration tests. Requires a running sim:

    ros2 launch palletizer_bringup palletizer.launch.py gui:=false spawn_rate:=0.0
"""

import os
import sys
import threading
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

from palletizer_tools import boxes, cell_config
from palletizer_tools.ur_ik import UrKinematics

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

OUTPUT_DIR = os.path.join(
    os.environ.get('PALLETIZER_TEST_OUTPUT', os.path.expanduser('~/palletizer_ws/test_output')))


class Checks:
    """Collects PASS/FAIL lines and turns them into an exit code."""

    def __init__(self, name):
        self.name = name
        self.failed = 0

    def check(self, ok, msg):
        print(f'  [{"PASS" if ok else "FAIL"}] {msg}', flush=True)
        if not ok:
            self.failed += 1
        return ok

    def finish(self):
        print(f'{self.name}: {"PASS" if self.failed == 0 else f"FAIL ({self.failed})"}', flush=True)
        return 0 if self.failed == 0 else 1


class SimClient(Node):

    def __init__(self, name='palletizer_test'):
        super().__init__(name)
        self.cell = cell_config.load()
        self.kin = UrKinematics(self.cell)
        self.world = self.cell['world_name']

        self.joints = None
        self.box_poses = {}
        self.grasped = None
        self.attached = None
        self.belt_running = None
        self.lock = threading.Lock()

        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.create_subscription(TFMessage, '/sim/box_poses', self._on_boxes, 10)
        self.create_subscription(Bool, '/suction/grasped', self._on_grasped, 10)
        self.create_subscription(String, '/suction/attached_model', self._on_attached, 10)
        self.create_subscription(Bool, '/conveyor_controller/running', self._on_running, 10)
        self.suction_pub = self.create_publisher(Bool, '/suction/cmd', 10)

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.list_ctrl = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.spawn_cli = self.create_client(SpawnEntity, f'/world/{self.world}/create')
        self.delete_cli = self.create_client(DeleteEntity, f'/world/{self.world}/remove')
        self.spawn_box_cli = self.create_client(Trigger, '/box_spawner/spawn_box')
        self.resume_cli = self.create_client(Trigger, '/conveyor_controller/resume')
        self.stop_cli = self.create_client(Trigger, '/conveyor_controller/stop')

        self.executor_ = MultiThreadedExecutor()
        self.executor_.add_node(self)
        self.thread = threading.Thread(target=self.executor_.spin, daemon=True)
        self.thread.start()

    # ---------------- subscriptions ----------------
    def _on_js(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.joints = np.array([pos[j] for j in JOINTS])

    def _on_boxes(self, msg):
        self.box_poses = boxes.poses_from_tf(msg)

    def _on_grasped(self, msg):
        self.grasped = msg.data

    def _on_attached(self, msg):
        self.attached = msg.data

    def _on_running(self, msg):
        self.belt_running = msg.data

    # ---------------- utils ----------------
    @staticmethod
    def wait_for(cond, timeout, period=0.05):
        end = time.time() + timeout
        while time.time() < end:
            if cond():
                return True
            time.sleep(period)
        return cond()

    def call(self, client, req, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available')
        fut = client.call_async(req)
        if not self.wait_for(fut.done, timeout):
            raise RuntimeError(f'service {client.srv_name} timed out')
        return fut.result()

    def wait_ready(self, timeout=60.0):
        return self.wait_for(lambda: self.joints is not None, timeout)

    # ---------------- controllers / arm ----------------
    def controller_states(self):
        res = self.call(self.list_ctrl, ListControllers.Request())
        return {c.name: c.state for c in res.controller}

    def move_joints(self, q, max_vel=0.6, min_time=2.0, settle_tol=0.003, timeout=30.0):
        """Execute a single-segment trajectory to q and wait until the arm settles."""
        q = np.asarray(q, dtype=float)
        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('trajectory action server not available')
        dt = max(min_time, float(np.max(np.abs(q - self.joints))) / max_vel)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        pt.velocities = [0.0] * 6
        pt.time_from_start.sec = int(dt)
        pt.time_from_start.nanosec = int((dt % 1) * 1e9)
        goal.trajectory.points = [pt]
        send = self.traj_client.send_goal_async(goal)
        if not self.wait_for(send.done, 10.0) or not send.result().accepted:
            raise RuntimeError('trajectory goal rejected')
        result = send.result().get_result_async()
        self.wait_for(result.done, dt + timeout)
        self.wait_for(lambda: np.max(np.abs(self.joints - q)) < settle_tol, 5.0)
        return float(np.max(np.abs(self.joints - q)))

    def tcp_pose(self):
        return self.kin.fk(self.joints)

    def move_tcp(self, xyz, yaw, **kw):
        q = self.kin.ik(self.kin.down_pose(xyz, yaw), self.joints)
        if q is None:
            raise RuntimeError(f'IK failed for {xyz}')
        self.move_joints(q, **kw)
        return self.tcp_pose()

    def home(self):
        return self.move_joints(self.cell['robot']['home'])

    def pick_approach_joints(self):
        """Home pose mirrored toward the conveyor (arm above the pick zone)."""
        q = list(self.cell['robot']['home'])
        q[0] = np.pi / 2
        return q

    # ---------------- boxes / conveyor ----------------
    def delete_all_boxes(self):
        for name in list(self.box_poses):
            req = DeleteEntity.Request()
            req.entity.name = name
            req.entity.type = Entity.MODEL
            self.call(self.delete_cli, req)
        return self.wait_for(lambda: not self.box_poses, 5.0) or not self.box_poses

    def spawn_box(self, name, size, mass, xyz, yaw=0.0, rgba=(0.72, 0.55, 0.35, 1.0)):
        req = boxes.spawn_request(name, size, mass, rgba, xyz, yaw)
        res = self.call(self.spawn_cli, req)
        if not res.success:
            raise RuntimeError(f'spawn of {name} failed')
        if not self.wait_for(lambda: name in self.box_poses, 5.0):
            raise RuntimeError(f'{name} never appeared in /sim/box_poses')

    def box_top_center(self, name, height):
        pos, quat = self.box_poses[name]
        return pos + Rotation.from_quat(quat).apply([0, 0, height / 2.0])

    def box_yaw(self, name):
        return Rotation.from_quat(self.box_poses[name][1]).as_euler('xyz')[2]

    def suction(self, on):
        self.suction_pub.publish(Bool(data=bool(on)))

    def shutdown(self):
        self.executor_.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)


def run(test_fn, name):
    rclpy.init()
    sim = SimClient()
    checks = Checks(name)
    print(f'== {name}', flush=True)
    try:
        if checks.check(sim.wait_ready(), 'sim is up (/joint_states received)'):
            test_fn(sim, checks)
    except Exception as e:  # noqa: BLE001
        checks.check(False, f'exception: {e!r}')
    finally:
        code = checks.finish()
        sim.shutdown()
        rclpy.try_shutdown()
    sys.exit(code)
