"""Minimal MoveIt 2 client over move_group's ROS interfaces (no moveit_py).

- MoveGroup action: Pilz PTP (joint goals) / LIN (pose goals), OMPL fallback
- /apply_planning_scene: collision objects, attach/detach
- /check_state_validity: collision check of a joint vector
"""

import time

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, MoveItErrorCodes, OrientationConstraint,
                             PlanningScene, PositionConstraint, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def box_object(obj_id, frame, size, pose, operation=CollisionObject.ADD):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = obj_id
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(v) for v in size]
    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = operation
    return co


def make_pose(xyz, quat_xyzw=(0.0, 0.0, 0.0, 1.0)):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
        (float(v) for v in quat_xyzw)
    return p


class MoveItClient:
    """Wraps move_group interfaces for one planning group. Calls block (use a
    multi-threaded executor so responses are processed)."""

    def __init__(self, node, group, tcp_link, frame='world', callback_group=None):
        self.node = node
        self.group = group
        self.tcp_link = tcp_link
        self.frame = frame
        self.move_group = ActionClient(node, MoveGroup, '/move_action',
                                       callback_group=callback_group)
        self.apply_scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                              callback_group=callback_group)
        self.validity = node.create_client(GetStateValidity, '/check_state_validity',
                                           callback_group=callback_group)
        self.log = node.get_logger()

    # ------------------------------------------------------------ helpers
    @staticmethod
    def _wait(fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.005)
        return fut.done()

    def wait_ready(self, timeout=60.0):
        return (self.move_group.wait_for_server(timeout_sec=timeout) and
                self.apply_scene.wait_for_service(timeout_sec=timeout) and
                self.validity.wait_for_service(timeout_sec=timeout))

    # ------------------------------------------------------------ motion
    def _request(self, pipeline, planner, constraints, vel, acc, planning_time):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = self.group
        r.pipeline_id = pipeline
        r.planner_id = planner
        r.num_planning_attempts = 1
        r.allowed_planning_time = float(planning_time)
        r.max_velocity_scaling_factor = float(vel)
        r.max_acceleration_scaling_factor = float(acc)
        r.start_state.is_diff = True
        r.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def _execute(self, goal, timeout=120.0):
        send = self.move_group.send_goal_async(goal)
        if not self._wait(send, 10.0) or not send.result().accepted:
            return MoveItErrorCodes.FAILURE, 'goal rejected'
        res_f = send.result().get_result_async()
        if not self._wait(res_f, timeout):
            send.result().cancel_goal_async()
            return MoveItErrorCodes.TIMED_OUT, 'execution timeout'
        code = res_f.result().result.error_code.val
        return code, '' if code == MoveItErrorCodes.SUCCESS else f'error code {code}'

    def joint_constraints(self, q, tol=1e-3):
        c = Constraints()
        for name, v in zip(JOINTS, q):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(v)
            jc.tolerance_above = jc.tolerance_below = tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def pose_constraints(self, xyz, quat_xyzw, pos_tol=1e-4, rot_tol=1e-3):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.frame
        pc.link_name = self.tcp_link
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])
        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [make_pose(xyz)]
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = self.frame
        oc.link_name = self.tcp_link
        oc.orientation = make_pose((0, 0, 0), quat_xyzw).orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = \
            oc.absolute_z_axis_tolerance = rot_tol
        oc.weight = 1.0
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        return c

    def move_joints(self, q, vel, acc, planning_time=5.0, ompl_fallback=True):
        """PTP to a joint goal; OMPL (RRTConnect) if the straight joint path collides."""
        cons = self.joint_constraints(q)
        code, msg = self._execute(self._request(
            'pilz_industrial_motion_planner', 'PTP', cons, vel, acc, planning_time))
        if code == MoveItErrorCodes.SUCCESS or not ompl_fallback:
            return code == MoveItErrorCodes.SUCCESS, f'PTP {msg}'
        if code not in (MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.INVALID_MOTION_PLAN,
                        MoveItErrorCodes.GOAL_IN_COLLISION, MoveItErrorCodes.FAILURE,
                        MoveItErrorCodes.NO_IK_SOLUTION, MoveItErrorCodes.START_STATE_IN_COLLISION):
            return False, f'PTP {msg}'
        self.log.info(f'PTP failed ({msg}), trying OMPL')
        code, msg2 = self._execute(self._request(
            'ompl', 'RRTConnectkConfigDefault', cons, vel, acc, planning_time))
        return code == MoveItErrorCodes.SUCCESS, f'PTP {msg}; OMPL {msg2}'

    def move_lin(self, xyz, quat_xyzw, vel, acc, planning_time=5.0):
        """Straight-line Cartesian motion of the TCP (Pilz LIN)."""
        code, msg = self._execute(self._request(
            'pilz_industrial_motion_planner', 'LIN',
            self.pose_constraints(xyz, quat_xyzw), vel, acc, planning_time))
        return code == MoveItErrorCodes.SUCCESS, f'LIN {msg}'

    # ------------------------------------------------------------ scene
    def apply(self, scene, timeout=10.0):
        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self.apply_scene.call_async(req)
        return self._wait(fut, timeout) and fut.result().success

    def add_objects(self, objects):
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = list(objects)
        return self.apply(ps)

    def remove_objects(self, ids):
        objs = []
        for i in ids:
            co = CollisionObject()
            co.header.frame_id = self.frame
            co.id = i
            co.operation = CollisionObject.REMOVE
            objs.append(co)
        return self.add_objects(objs)

    def attach_box(self, obj_id, size, xyz_in_tcp, quat_in_tcp, touch_links):
        aco = AttachedCollisionObject()
        aco.link_name = self.tcp_link
        aco.object = box_object(obj_id, self.tcp_link, size, make_pose(xyz_in_tcp, quat_in_tcp))
        aco.touch_links = list(touch_links)
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        return self.apply(ps)

    def detach_and_remove(self, obj_id):
        aco = AttachedCollisionObject()
        aco.link_name = self.tcp_link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.REMOVE
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        ok = self.apply(ps)
        return self.remove_objects([obj_id]) and ok

    def state_valid(self, q, timeout=5.0):
        req = GetStateValidity.Request()
        rs = RobotState()
        rs.joint_state = JointState(name=JOINTS, position=[float(v) for v in q])
        rs.is_diff = True
        req.robot_state = rs
        req.group_name = self.group
        fut = self.validity.call_async(req)
        return self._wait(fut, timeout) and fut.result().valid


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi
