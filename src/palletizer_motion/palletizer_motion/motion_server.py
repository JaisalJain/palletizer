"""Pick and place with MoveIt 2 (Pilz PTP/LIN), exposed as ROS 2 actions.

Actions:  /motion/pick  (PickBox)   pre-grasp -> LIN descend -> suction on + confirm -> attach
                                     -> LIN lift to transit height -> weigh (F/T)
          /motion/place (PlaceBox)  pre-place -> LIN descend -> release ~5 mm above the
                                     resting surface + confirm -> detach -> LIN retreat
Services: /suction/weigh (Weigh), /motion/park (Trigger),
          /motion/scene/add_placed (CommitPlacement), /motion/scene/clear_placed (Trigger)

Inputs are perception results and planner placements only; the suction plugin's
attached_model name is passed through for evaluation and never used for control.
"""

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from palletizer_interfaces.action import PickBox, PlaceBox
from palletizer_interfaces.srv import CommitPlacement, Weigh
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from palletizer_motion.moveit_client import JOINTS, MoveItClient, box_object, make_pose
from palletizer_motion.scene import static_objects
from palletizer_tools import cell_config
from palletizer_tools.ur_ik import LOWER, UPPER, UrKinematics

G = 9.81


def tool_quat(yaw):
    """Quaternion (xyzw) of a TCP pointing straight down with tool x at `yaw`."""
    return (Rotation.from_euler('z', yaw) * Rotation.from_euler('x', math.pi)).as_quat()


class MotionServer(Node):

    def __init__(self):
        super().__init__('motion_server')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None)
        self.m = self.cell['motion']
        self.kin = UrKinematics(self.cell)
        self.cb = ReentrantCallbackGroup()
        self.mv = MoveItClient(self, self.m['planning_group'], self.m['tcp_link'],
                               callback_group=self.cb)

        self.joints = None
        self.grasped = False
        self.attached_name = ''
        self.wrench = []            # (monotonic time, fz)
        self.tare_fz = None
        self.lock = threading.Lock()
        self.busy = threading.Lock()
        self.held = None            # dict with grasp info while carrying a box
        self.placed_ids = []

        self.create_subscription(JointState, '/joint_states', self.on_js, 10, callback_group=self.cb)
        self.create_subscription(Bool, '/suction/grasped', self.on_grasped, 10, callback_group=self.cb)
        self.create_subscription(String, '/suction/attached_model', self.on_attached, 10,
                                 callback_group=self.cb)
        self.create_subscription(WrenchStamped, '/suction/wrench', self.on_wrench, 50,
                                 callback_group=self.cb)
        self.suction_pub = self.create_publisher(Bool, '/suction/cmd', 10)

        self.create_service(Weigh, '/suction/weigh', self.on_weigh, callback_group=self.cb)
        self.create_service(Trigger, '/motion/park', self.on_park, callback_group=self.cb)
        self.create_service(CommitPlacement, '/motion/scene/add_placed', self.on_add_placed,
                            callback_group=self.cb)
        self.create_service(Trigger, '/motion/scene/clear_placed', self.on_clear_placed,
                            callback_group=self.cb)
        self.pick_server = ActionServer(
            self, PickBox, '/motion/pick', self.execute_pick, callback_group=self.cb,
            goal_callback=self.accept_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.place_server = ActionServer(
            self, PlaceBox, '/motion/place', self.execute_place, callback_group=self.cb,
            goal_callback=self.accept_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        threading.Thread(target=self.init_scene, daemon=True).start()

    # ------------------------------------------------------------------ inputs
    def on_js(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.joints = np.array([pos[j] for j in JOINTS])

    def on_grasped(self, msg):
        self.grasped = msg.data

    def on_attached(self, msg):
        self.attached_name = msg.data

    def on_wrench(self, msg):
        with self.lock:
            self.wrench.append((time.monotonic(), msg.wrench.force.z))
            self.wrench = self.wrench[-2000:]

    def init_scene(self):
        if not self.mv.wait_ready(120.0):
            self.get_logger().error('move_group not available')
            return
        ok = self.mv.add_objects(static_objects(self.cell))
        self.get_logger().info(f'static planning scene applied: {ok}; motion_server ready')

    def accept_goal(self, goal):
        return GoalResponse.ACCEPT if not self.busy.locked() else GoalResponse.REJECT

    # ------------------------------------------------------------------ helpers
    def mean_fz(self, window):
        t_end = time.monotonic() + window
        time.sleep(window)
        with self.lock:
            vals = [f for t, f in self.wrench if t_end - window <= t <= t_end + 0.05]
        return float(np.mean(vals)) if vals else None

    def on_weigh(self, request, response):
        fz = self.mean_fz(request.duration if request.duration > 0 else self.m['weigh_window'])
        if fz is None:
            response.success, response.message = False, 'no F/T data'
            return response
        if request.tare:
            self.tare_fz = fz
        response.force_z = fz
        response.mass = (fz - self.tare_fz) / G if self.tare_fz is not None else 0.0
        response.success = True
        return response

    def wait_for(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            time.sleep(0.01)
        return cond()

    def ptp(self, q):
        return self.mv.move_joints(q, self.m['ptp_velocity_scaling'],
                                   self.m['ptp_acceleration_scaling'],
                                   self.m['planning_time'], self.m['ompl_fallback'])

    def lin(self, xyz, yaw, slow=False):
        v = self.m['lin_slow_velocity_scaling'] if slow else self.m['lin_velocity_scaling']
        a = self.m['lin_acceleration_scaling']
        return self.mv.move_lin(xyz, tool_quat(yaw), v, a, self.m['planning_time'])

    def descend(self, xyz, yaw):
        """LIN to slow_zone above xyz, then slow LIN to xyz."""
        above = np.array(xyz) + [0, 0, self.m['slow_zone']]
        ok, msg = self.lin(above, yaw)
        if not ok:
            return ok, msg
        return self.lin(xyz, yaw, slow=True)

    def seed_for(self, xy):
        """Elbow-up, wrist-down seed with the arm pointing at xy."""
        q = np.array(self.cell['robot']['home'], dtype=float)
        # home (pan = -pi/2) puts the TCP at atan2 bearing -pi/2 + offset; offset from FK
        tcp = self.kin.fk(q)[:3, 3]
        offset = math.atan2(tcp[1], tcp[0]) - q[0]
        q[0] = math.atan2(xy[1], xy[0]) - offset
        return q

    def solve(self, xyz, yaws, ref=None, seed=None):
        """IK for a down-pointing TCP at xyz; picks the yaw candidate that minimizes wrist
        rotation relative to `ref` (soft-penalizing |wrist_3| > pi). Returns (q, yaw)."""
        seed = self.seed_for(xyz) if seed is None else seed
        ref = self.joints if ref is None else ref
        best = None
        for yaw in yaws:
            q = self.kin.ik(self.kin.down_pose(xyz, yaw), seed)
            if q is None:
                continue
            q = self.unwrap_near(q, ref)
            cost = abs(q[5] - ref[5]) + 0.5 * max(0.0, abs(q[5]) - math.pi) + \
                0.1 * float(np.sum(np.abs(q[:5] - seed[:5])))
            if best is None or cost < best[0]:
                best = (cost, q, yaw)
        return (best[1], best[2]) if best else (None, None)

    @staticmethod
    def unwrap_near(q, ref):
        """Shift each joint by 2*pi multiples toward ref while staying within joint limits."""
        q = q.copy()
        for i in range(6):
            while q[i] - ref[i] > math.pi and q[i] - 2 * math.pi >= LOWER[i]:
                q[i] -= 2 * math.pi
            while q[i] - ref[i] < -math.pi and q[i] + 2 * math.pi <= UPPER[i]:
                q[i] += 2 * math.pi
        return q

    def feedback(self, goal_handle, fb_type, phase, progress):
        fb = fb_type.Feedback()
        fb.phase = phase
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'[{fb_type.__name__}] {phase}')

    # ------------------------------------------------------------------ pick
    def execute_pick(self, goal_handle):
        with self.busy:
            res = self._pick(goal_handle)
        (goal_handle.succeed if res.success else goal_handle.abort)()
        return res

    def _fail(self, res, code, msg):
        res.success = False
        res.error_code = code
        res.message = msg
        self.get_logger().warn(f'{code}: {msg}')
        return res

    def _pick(self, gh):
        box = gh.request.box
        res = PickBox.Result()
        m = self.m
        top = np.array([box.top_center.position.x, box.top_center.position.y,
                        box.top_center.position.z])
        grasp_yaws = [box.yaw + k * math.pi / 2 for k in range(4)]
        pre = top + [0, 0, m['pre_grasp_height']]
        q_pre, yaw = self.solve(pre, grasp_yaws, ref=self.joints)
        if q_pre is None:
            return self._fail(res, 'NO_IK', f'no IK for pre-grasp {pre.round(3)}')

        self.feedback(gh, PickBox, 'tare F/T', 0.05)
        self.suction_pub.publish(Bool(data=False))
        self.feedback(gh, PickBox, 'move to pre-grasp', 0.1)
        ok, msg = self.ptp(q_pre)
        if not ok:
            return self._fail(res, 'PLAN_FAILED', f'pre-grasp: {msg}')
        time.sleep(m['weigh_settle'])
        tare = self.mean_fz(m['weigh_window'])
        self.tare_fz = tare

        self.feedback(gh, PickBox, 'descend', 0.35)
        grasp = top + [0, 0, m['pick_gap']]
        ok, msg = self.descend(grasp, yaw)
        if not ok:
            self.lin(pre, yaw)
            return self._fail(res, 'EXEC_FAILED', f'descend: {msg}')

        self.feedback(gh, PickBox, 'suction on', 0.5)
        self.suction_pub.publish(Bool(data=True))
        if not self.wait_for(lambda: self.grasped, m['grasp_confirm_timeout']):
            self.suction_pub.publish(Bool(data=False))
            self.lin(pre, yaw)
            return self._fail(res, 'NO_GRASP', 'suction did not confirm a grasp')

        # Attach the measured box to the TCP in the planning scene.
        T_tcp = self.kin.fk(self.joints)
        R_tcp = T_tcp[:3, :3]
        clr = m['held_box_bottom_clearance']
        shrink = 2 * m['held_box_shrink_xy']
        box_center = top - [0, 0, (box.height - clr) / 2]
        R_box = Rotation.from_euler('z', box.yaw).as_matrix()
        p_in_tcp = R_tcp.T @ (box_center - T_tcp[:3, 3])
        q_in_tcp = Rotation.from_matrix(R_tcp.T @ R_box).as_quat()
        self.mv.attach_box('held_box', (box.length - shrink, box.width - shrink, box.height - clr),
                           p_in_tcp, q_in_tcp, ['suction_cup', 'suction_body', 'wrist_3_link'])
        # Where the detected top-face center sits in the TCP frame (grasp offset compensation)
        self.held = {
            'top_in_tcp': R_tcp.T @ (top - T_tcp[:3, 3]),
            'yaw_offset': yaw - box.yaw,       # tool yaw relative to the box length axis
            'box': box,
        }

        self.feedback(gh, PickBox, 'lift', 0.7)
        lift_z = max(pre[2], self.cell['conveyor']['belt_top_z'] + m['transit_clearance'] +
                     box.height + m['pick_gap'])
        ok, msg = self.lin(np.array([grasp[0], grasp[1], lift_z]), yaw)
        if not ok:
            return self._fail(res, 'EXEC_FAILED', f'lift: {msg}')
        if not self.grasped:
            self.mv.detach_and_remove('held_box')
            self.held = None
            return self._fail(res, 'LOST_GRASP', 'box dropped during lift')

        self.feedback(gh, PickBox, 'weigh', 0.9)
        time.sleep(m['weigh_settle'])
        fz = self.mean_fz(m['weigh_window'])
        res.measured_mass = (fz - tare) / G if fz is not None and tare is not None else 0.0
        res.sim_model_name = self.attached_name
        res.success = True
        res.message = f'picked, {res.measured_mass:.2f} kg'
        self.held['mass'] = res.measured_mass
        self.feedback(gh, PickBox, 'done', 1.0)
        return res

    # ------------------------------------------------------------------ place
    def execute_place(self, goal_handle):
        with self.busy:
            res = self._place(goal_handle)
        (goal_handle.succeed if res.success else goal_handle.abort)()
        return res

    def _place(self, gh):
        pl = gh.request.placement
        res = PlaceBox.Result()
        m = self.m
        if self.held is None or not self.grasped:
            return self._fail(res, 'EXEC_FAILED', 'not holding a box')
        target_top = np.array([pl.top_center.position.x, pl.top_center.position.y,
                               pl.top_center.position.z])
        yaws = [pl.yaw + self.held['yaw_offset'] + k * math.pi for k in range(2)]

        def tcp_for(top_xyz, yaw):
            R = Rotation.from_euler('z', yaw) * Rotation.from_euler('x', math.pi)
            return top_xyz - R.apply(self.held['top_in_tcp'])

        release_top = target_top + [0, 0, m['release_gap']]
        pre_top = target_top + [0, 0, m['pre_place_height']]
        # choose the yaw with the smaller wrist rotation, using the pre-place pose
        best = None
        for yaw in yaws:
            q, _ = self.solve(tcp_for(pre_top, yaw), [yaw], ref=self.joints)
            if q is None:
                continue
            cost = abs(q[5] - self.joints[5])
            if best is None or cost < best[0]:
                best = (cost, q, yaw)
        if best is None:
            return self._fail(res, 'NO_IK', f'no IK for pre-place {pre_top.round(3)}')
        _, q_pre, yaw = best

        self.feedback(gh, PlaceBox, 'move to pre-place', 0.1)
        ok, msg = self.ptp(q_pre)
        if not ok:
            return self._fail(res, 'PLAN_FAILED', f'pre-place: {msg}')
        self.feedback(gh, PlaceBox, 'descend', 0.4)
        ok, msg = self.descend(tcp_for(release_top, yaw), yaw)
        if not ok:
            self.lin(tcp_for(pre_top, yaw), yaw)
            return self._fail(res, 'EXEC_FAILED', f'descend: {msg}')

        self.feedback(gh, PlaceBox, 'release', 0.6)
        self.suction_pub.publish(Bool(data=False))
        if not self.wait_for(lambda: not self.grasped, m['release_confirm_timeout']):
            return self._fail(res, 'RELEASE_FAILED', 'suction did not release')
        self.mv.detach_and_remove('held_box')
        self.held = None
        time.sleep(m['release_settle'])

        self.feedback(gh, PlaceBox, 'retreat', 0.8)
        tcp_now = self.kin.fk(self.joints)[:3, 3]
        ok, msg = self.lin(tcp_now + [0, 0, m['retreat_height']], yaw)
        if not ok:
            return self._fail(res, 'EXEC_FAILED', f'retreat: {msg}')
        res.success = True
        res.message = 'placed'
        self.feedback(gh, PlaceBox, 'done', 1.0)
        return res

    # ------------------------------------------------------------------ misc services
    def on_park(self, request, response):
        with self.busy:
            ok, msg = self.ptp(self.cell['robot']['home'])
        response.success, response.message = ok, msg
        return response

    def on_add_placed(self, request, response):
        pl, box = request.placement, request.box
        obj_id = f'placed_{len(self.placed_ids):03d}'
        center = (pl.top_center.position.x, pl.top_center.position.y,
                  pl.top_center.position.z - box.height / 2)
        q = Rotation.from_euler('z', pl.yaw).as_quat()
        ok = self.mv.add_objects([box_object(
            obj_id, 'world', (box.length, box.width, box.height), make_pose(center, q))])
        if ok:
            self.placed_ids.append(obj_id)
        response.success, response.message = ok, obj_id
        return response

    def on_clear_placed(self, request, response):
        ok = self.mv.remove_objects(self.placed_ids) if self.placed_ids else True
        n = len(self.placed_ids)
        self.placed_ids = []
        response.success, response.message = ok, f'removed {n} boxes from the planning scene'
        return response


def main():
    rclpy.init()
    node = MotionServer()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    ex.spin()


if __name__ == '__main__':
    main()
