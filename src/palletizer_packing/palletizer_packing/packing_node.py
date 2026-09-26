"""ROS 2 wrapper around the packing core.

Services: /packing/plan_placement (PlanPlacement), /packing/commit (CommitPlacement),
          /packing/reset (Trigger: empty the model of the current pallet),
          /packing/set_pallet (SetPallet: the MEASURED pallet; valid=false = none)
Topics:   /packing/markers (MarkerArray: height map + ghost box), /packing/state (String JSON),
          /packing/status (PackingState, transient local), /packing/next_target (Placement)
In:       /pallet_perception/height_map (HeightMap)
Params:   mode (mixed | single_sku), planner (mixed mode: naive | heuristic)

Nothing about pallets or boxes comes from the config: the deck size and pose come from
set_pallet (pallet_perception), box dims from each request (box perception). The usable
stack height is the reachability map intersected with the measured pallet. The model is the
record of *commanded* placements, merged before every plan with the perceived height map
(perception wins on unambiguous cells; every disagreement is logged).
"""

import json
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import numpy as np
from palletizer_interfaces.msg import HeightMap, PackingState
from palletizer_interfaces.msg import Placement as PlacementMsg
from palletizer_interfaces.srv import CommitPlacement, PlanPlacement, SetPallet
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from palletizer_packing.core.pallet import Box, PalletSpec, PalletState, Placement
from palletizer_packing.core.pattern import PatternConfig, PatternPlanner
from palletizer_packing.core.perceived import merge
from palletizer_packing.core.planners import make_planner
from palletizer_packing.core.reach import PalletReach, ReachabilityMap
from palletizer_packing.core.rules import RuleConfig, stability
from palletizer_tools import cell_config
from palletizer_tools.pallet_frame import PalletFrame

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class PackingNode(Node):

    def __init__(self):
        super().__init__('packing')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None, role='robot')
        pk = self.cell['packing']
        self.mode = self.declare_parameter('mode', pk['mode']).value
        self.planner_name = self.declare_parameter('planner', pk['planner']).value
        map_path = os.path.join(get_package_share_directory('palletizer_motion'), 'config',
                                self.cell['reachability']['file'])
        self.world_map = ReachabilityMap.load(map_path)
        self.physics = None
        if pk['physics']['enabled'] and (self.mode == 'single_sku' or
                                         self.planner_name == 'heuristic'):
            from palletizer_packing.core.physics import PhysicsChecker, physics_config
            self.physics = PhysicsChecker(physics_config(self.cell))
        self.rules = RuleConfig.from_config(self.cell)
        if self.mode == 'single_sku':
            pcfg = PatternConfig.from_config(self.cell)
            pcfg.priority = self.declare_parameter('pattern_priority', pcfg.priority).value
            self.planner = PatternPlanner(self.rules, pcfg,
                                          pk['sku_tolerance'], self.physics)
            self.planner.log = lambda m: self.get_logger().info(m)
            self.planner_name = 'pattern'
        else:
            self.planner = make_planner(self.planner_name, self.cell, None, self.physics)
        self.planner.validator_max = int(pk['motion_check_max'])

        self.frame = None          # PalletFrame of the measured pallet
        self.reach = None
        self.spec = None
        self.state = None
        self.hmap = None           # latest HeightMap for this pallet
        self.map_lock = threading.Lock()
        self.ghost = None
        self.last_plan = None      # (box id, Placement) of the last plan
        self.disagreements = 0
        self.merged = None
        self.plan_state = None

        srv_group = MutuallyExclusiveCallbackGroup()
        self.create_service(PlanPlacement, '/packing/plan_placement', self.on_plan,
                            callback_group=srv_group)
        self.create_service(CommitPlacement, '/packing/commit', self.on_commit,
                            callback_group=srv_group)
        self.create_service(Trigger, '/packing/reset', self.on_reset, callback_group=srv_group)
        self.create_service(SetPallet, '/packing/set_pallet', self.on_set_pallet,
                            callback_group=srv_group)
        # Motion feasibility in the current scene (motion server); own group so the call can
        # complete while a plan request is being served.
        self.check_cli = self.create_client(CommitPlacement, '/motion/check_placement',
                                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(HeightMap, '/pallet_perception/height_map', self.on_map,
                                 LATCHED, callback_group=ReentrantCallbackGroup())
        self.marker_pub = self.create_publisher(MarkerArray, '/packing/markers', 10)
        self.state_pub = self.create_publisher(String, '/packing/state', 10)
        self.status_pub = self.create_publisher(PackingState, '/packing/status', LATCHED)
        self.target_pub = self.create_publisher(PlacementMsg, '/packing/next_target', LATCHED)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            f'packing ready: mode {self.mode}, planner {self.planner_name}, physics '
            f'{"on" if self.physics else "off"}; waiting for a measured pallet')

    # ------------------------------------------------------------ pallet
    def on_set_pallet(self, req, res):
        p = req.pallet
        if not p.valid:
            self.frame = self.reach = self.spec = self.state = None
            self.target_pub.publish(PlacementMsg())
            self.planner.reset() if hasattr(self.planner, 'reset') else None
            self.ghost = None
            res.success, res.message = True, 'no pallet'
            res.state = self.packing_state()
            self.publish_status()
            return res
        self.frame = PalletFrame.from_msg(p)
        f = self.frame
        self.reach = PalletReach(self.world_map, f.to_world, f.size_u, f.size_v)
        st = self.cell['pallet_station']
        usable = self.reach.usable_top_height(self.cell['reachability']['edge_inset'] + 0.02,
                                              st['max_stack_height'])
        self.spec = PalletSpec.measured(f.size_u, f.size_v, self.cell, usable)
        self.new_model()
        res.success = self.spec.max_height > 0
        res.message = (f'pallet {f.size_u * 1000:.0f} x {f.size_v * 1000:.0f} mm, usable stack '
                       f'height {self.spec.max_height:.2f} m (reachability map)')
        self.get_logger().info(res.message)
        res.state = self.packing_state()
        self.publish_status()
        return res

    def new_model(self):
        self.state = PalletState(self.spec)
        self.planner.reach = self.reach
        if hasattr(self.planner, 'reset'):
            self.planner.reset()
        self.ghost = None
        self.last_plan = None
        self.disagreements = 0
        self.merged = None

    def on_map(self, m):
        if m.pallet.valid:
            with self.map_lock:
                self.hmap = m

    def current_map(self):
        """Latest perceived map, if it is of the current pallet."""
        with self.map_lock:
            m = self.hmap
        if m is None or self.frame is None:
            return None
        c = self.frame.center
        if math.hypot(m.pallet.center.position.x - c[0], m.pallet.center.position.y - c[1]) > 0.01:
            return None                             # a map of another pallet
        return m

    # ------------------------------------------------------------ conversions
    def to_msg(self, p: Placement, box: Box):
        f = self.frame
        m = PlacementMsg()
        u, v = p.center
        x, y, z = f.to_world(u, v, p.top)
        m.top_center.position.x, m.top_center.position.y, m.top_center.position.z = \
            float(x), float(y), float(z)
        m.yaw = math.atan2(math.sin(f.box_yaw(p.rotated)), math.cos(f.box_yaw(p.rotated)))
        m.top_center.orientation.z = math.sin(m.yaw / 2)
        m.top_center.orientation.w = math.cos(m.yaw / 2)
        m.size_x, m.size_y = p.du, p.dv                 # footprint along the pallet u / v
        m.height = p.height
        m.rest_z = f.deck_z + p.z0
        m.rotated = p.rotated
        m.score = float(p.score)
        m.planner = self.planner_name
        m.planning_time = float(p.info.get('planning_time', 0.0))
        m.candidates_checked = int(p.info.get('candidates_checked', 0))
        m.physics_checks = int(p.info.get('physics_checks', 0))
        return m

    def from_msg(self, m: PlacementMsg, box: Box):
        u, v = self.frame.to_pallet(m.top_center.position.x, m.top_center.position.y)
        du, dv = (box.width, box.length) if m.rotated else (box.length, box.width)
        return Placement(u0=u - du / 2, v0=v - dv / 2, du=du, dv=dv,
                         z0=m.rest_z - self.frame.deck_z, height=box.height, rotated=m.rotated)

    @staticmethod
    def box_from(det, mass):
        return Box(max(det.length, det.width), min(det.length, det.width), det.height,
                   mass if mass > 0 else 0.0)

    # ------------------------------------------------------------ perception merge
    def planning_state(self, wait=True):
        """Record merged with the perceived height map, if perception has seen the last
        commit (waits up to map_wait); else the record (logged). The pre-check (wait=False)
        does not wait."""
        if not self.cell['packing']['perception_merge']:
            return self.state
        n = len(self.state.boxes)
        end = time.monotonic() + (float(self.cell['packing']['map_wait']) if wait else 0.0)
        while True:
            m = self.current_map()
            if m is not None and m.placements >= n:
                break
            if not wait:
                return self.state
            if time.monotonic() > end:
                self.get_logger().warn(
                    f'no perceived height map with {n} placements '
                    f'({"none" if m is None else m.placements}): planning on the record')
                return self.state
            time.sleep(0.05)
        H = np.array(m.data, dtype=float).reshape(m.rows, m.cols)
        rec = self.state.height
        if H.shape != rec.shape:
            r = min(H.shape[0], rec.shape[0]), min(H.shape[1], rec.shape[1])
            Hp = np.full(rec.shape, np.nan)
            Hp[:r[0], :r[1]] = H[:r[0], :r[1]]
            H = Hp
        merged, regions = merge(rec, H, self.cell['pallet_perception']['disagreement_tol'])
        for g in regions:
            iv, iu = g['centre']
            self.get_logger().warn(
                f'perception disagrees with the placement record: {g["cells"]} cells around '
                f'u={iu * self.spec.resolution:.2f} v={iv * self.spec.resolution:.2f}: perceived '
                f'{g["perceived"]:.3f} m, record {g["record"]:.3f} m -> using perception')
        self.disagreements += len(regions)
        self.merged = merged
        return self.state.with_height(merged)

    # ------------------------------------------------------------ services
    def on_plan(self, req, res):
        box = self.box_from(req.box, req.mass)
        if self.state is None:
            res.success, res.pallet_full, res.reason = False, False, 'NO_PALLET'
            res.message = 'no measured pallet'
            res.state = self.packing_state()
            return res
        if req.recheck and self.last_plan is not None and self.last_plan[0] == req.box.id:
            p = self.last_plan[1]
            st = stability(self.plan_state, p, box, self.rules)
            if st.ok:
                res.success, res.placement = True, self.to_msg(p, box)
                res.message = 'recheck with measured mass: ok'
                self.get_logger().info(f'recheck with {box.mass:.1f} kg: ok')
                res.state = self.packing_state()
                return res
            self.get_logger().info(f'recheck with {box.mass:.1f} kg failed ({st.reason}): '
                                   'planning again')
        state = self.planning_state(wait=not req.check_only)
        self.plan_state = state
        self.planner.exclude = [self.from_msg(m, box) for m in req.exclude]
        self.planner.validator = (lambda p, b=box: self.motion_feasible(p, b)) \
            if self.check_cli.service_is_ready() else None
        if self.mode == 'single_sku' and not state.boxes and (
                self.planner.pattern is None or not self.planner.matches(box)):
            self.start_pattern(box)
        p = self.planner.plan(state, box, check_only=req.check_only)
        self.planner.exclude = []
        stats = self.planner.last_stats
        if p is None:
            res.success = False
            res.reason = self.planner.last_reason or 'NO_FEASIBLE_PLACEMENT'
            res.pallet_full = res.reason != 'SKU_CHANGE'
            res.message = stats.get('message') or (
                f'no feasible placement ({stats.get("candidates_checked", 0)} candidates, '
                f'{stats.get("planning_time", 0.0) * 1000:.0f} ms)')
            self.ghost = None
            res.state = self.packing_state()
            return res
        res.success = True
        res.placement = self.to_msg(p, box)
        res.message = (f'{"rotated " if p.rotated else ""}at u={p.u0:.3f} v={p.v0:.3f} '
                       f'z={p.z0:.3f}, {res.placement.planning_time * 1000:.0f} ms')
        self.last_plan = (req.box.id, p)
        if not req.check_only:
            self.ghost = (p, box)
            self.target_pub.publish(res.placement)
            self.get_logger().info(f'plan: {res.message}')
        res.state = self.packing_state()
        return res

    def start_pattern(self, box):
        """Pattern for this SKU on this pallet; usable height with the SKU's own inset."""
        inset = min(box.length, box.width) / 2
        usable = self.reach.usable_top_height(inset, self.cell['pallet_station']
                                              ['max_stack_height'])
        spec = PalletSpec.measured(self.spec.length, self.spec.width, self.cell, usable)
        t0 = time.perf_counter()
        self.planner.start(box, spec)
        pat = self.planner.pattern
        self.get_logger().info(
            f'pattern for {box.length * 1000:.0f} x {box.width * 1000:.0f} x '
            f'{box.height * 1000:.0f} mm on {spec.length * 1000:.0f} x {spec.width * 1000:.0f} mm '
            f'(usable {usable:.2f} m): {pat.layer_a.count}/{pat.layer_b.count} per layer '
            f'(optimum {pat.optimum}), {pat.layers} layers = {pat.count} boxes; {pat.note} '
            f'[{(time.perf_counter() - t0) * 1000:.0f} ms]')

    def motion_feasible(self, p, box):
        req = CommitPlacement.Request()
        req.placement = self.to_msg(p, box)
        req.box.length, req.box.width, req.box.height = box.length, box.width, box.height
        fut = self.check_cli.call_async(req)
        end = time.monotonic() + 10.0
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.002)
        if not fut.done():
            return False
        if not fut.result().success:
            self.get_logger().info(f'motion check rejected u={p.u0:.2f} v={p.v0:.2f} '
                                   f'z={p.z0:.2f}: {fut.result().message}')
        return fut.result().success

    def on_commit(self, req, res):
        if self.state is None:
            res.success, res.message = False, 'no measured pallet'
            return res
        box = self.box_from(req.box, req.mass)
        p = self.from_msg(req.placement, box)
        self.state.place(p, box, req.sim_model_name)
        self.ghost = None
        self.target_pub.publish(PlacementMsg())          # no pending target
        self.last_plan = None
        res.success = True
        res.message = (f'{len(self.state.boxes)} boxes, fill {self.state.fill_percent():.1f} %, '
                       f'max height {self.state.max_height:.2f} m')
        res.state = self.packing_state()
        self.publish_status()
        self.publish_markers()
        return res

    def on_reset(self, req, res):
        n = len(self.state.boxes) if self.state else 0
        if self.spec is not None:
            self.new_model()
        res.success = True
        res.message = f'pallet model reset ({n} boxes removed)'
        self.publish_status()
        self.publish_markers()
        return res

    # ------------------------------------------------------------ status
    def packing_state(self):
        s = PackingState()
        s.mode = self.mode
        s.has_pallet = self.state is not None
        if self.state is None:
            return s
        s.pallet_u, s.pallet_v = self.spec.length, self.spec.width
        s.usable_height = self.spec.max_height
        s.placed = len(self.state.boxes)
        s.fill_percent = self.state.fill_percent()
        s.max_height = self.state.max_height
        s.disagreements = self.disagreements
        pat = getattr(self.planner, 'pattern', None)
        if pat is not None:
            s.has_sku = True
            s.sku_length, s.sku_width, s.sku_height = pat.box.length, pat.box.width, pat.box.height
            s.per_layer_a, s.per_layer_b = pat.layer_a.count, pat.layer_b.count
            s.optimum_per_layer, s.layers, s.pattern_count = pat.optimum, pat.layers, pat.count
            s.pattern_note = pat.note
            s.usable_height = pat.spec.max_height
            s.fill_percent = 100.0 * self.state.box_volume / (
                pat.spec.length * pat.spec.width * pat.spec.max_height)
            i = len(self.state.boxes)
            s.layer = pat.slots[i].layer if i < pat.count else pat.layers
        else:
            s.layer = 0
        return s

    def publish_status(self):
        s = self.packing_state()
        self.status_pub.publish(s)
        self.state_pub.publish(String(data=json.dumps({
            'mode': self.mode, 'planner': self.planner_name, 'pallet': s.has_pallet,
            'boxes': s.placed, 'pattern_count': s.pattern_count, 'layer': s.layer,
            'fill_percent': s.fill_percent, 'max_height': s.max_height,
            'disagreements': s.disagreements})))

    # ------------------------------------------------------------ visualization
    def publish_markers(self):
        arr = MarkerArray()
        hm = Marker()
        hm.header.frame_id = 'world'
        hm.ns = 'heightmap'
        hm.type = Marker.CUBE_LIST
        if self.state is None:
            hm.action = Marker.DELETE
            arr.markers.append(hm)
        else:
            f = self.frame
            r = self.spec.resolution * 2
            hm.scale.x = hm.scale.y = r
            hm.scale.z = 0.005
            q = f.yaw
            hm.pose.orientation.z, hm.pose.orientation.w = math.sin(q / 2), math.cos(q / 2)
            o = f.to_world(0, 0)
            hm.pose.position = Point(x=float(o[0]), y=float(o[1]), z=float(o[2]))
            H = self.merged if self.merged is not None and \
                self.merged.shape == self.state.height.shape else self.state.height
            step = 2
            for iv in range(0, H.shape[0], step):
                for iu in range(0, H.shape[1], step):
                    h = float(H[iv:iv + step, iu:iu + step].max())
                    if h <= 0:
                        continue
                    # marker pose is the pallet frame with y = -v
                    hm.points.append(Point(x=(iu + 1) * self.spec.resolution,
                                           y=-(iv + 1) * self.spec.resolution, z=h + 0.003))
                    t = min(1.0, h / max(self.spec.max_height, 1e-6))
                    hm.colors.append(ColorRGBA(r=t, g=0.3, b=1.0 - t, a=0.8))
            if not hm.points:
                hm.action = Marker.DELETE
            arr.markers.append(hm)
        g = Marker()
        g.header.frame_id = 'world'
        g.ns = 'ghost'
        if self.ghost is None or self.frame is None:
            g.action = Marker.DELETE
        else:
            p, box = self.ghost
            u, v = p.center
            c = self.frame.to_world(u, v, p.z0 + p.height / 2)
            g.type = Marker.CUBE
            g.pose.position = Point(x=float(c[0]), y=float(c[1]), z=float(c[2]))
            yaw = self.frame.yaw
            g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            g.scale.x, g.scale.y, g.scale.z = p.du, p.dv, p.height
            g.color = ColorRGBA(r=0.2, g=1.0, b=0.3, a=0.35)
        arr.markers.append(g)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    ex = MultiThreadedExecutor()
    ex.add_node(PackingNode())
    ex.spin()


if __name__ == '__main__':
    main()
