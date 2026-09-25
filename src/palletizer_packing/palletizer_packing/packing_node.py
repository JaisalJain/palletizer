"""ROS 2 wrapper around the packing core.

Services: /packing/plan_placement (PlanPlacement), /packing/commit (CommitPlacement),
          /packing/reset (Trigger)
Topics:   /packing/markers (MarkerArray: height map + ghost box), /packing/state (String JSON)
Param:    planner (naive | heuristic; default from cell_config packing.planner)

The pallet model is updated from the *commanded* placement (commit), never from ground truth.
"""

import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from palletizer_interfaces.msg import Placement as PlacementMsg
from palletizer_interfaces.srv import CommitPlacement, PlanPlacement
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from palletizer_packing.core.pallet import Box, PalletSpec, PalletState, Placement
from palletizer_packing.core.planners import make_planner
from palletizer_packing.core.reach import ReachabilityMap
from palletizer_tools import cell_config


class PackingNode(Node):

    def __init__(self):
        super().__init__('packing')
        cfg_path = self.declare_parameter('cell_config', '').value
        self.cell = cell_config.load(cfg_path or None)
        self.planner_name = self.declare_parameter(
            'planner', self.cell['packing']['planner']).value
        map_path = os.path.join(get_package_share_directory('palletizer_motion'), 'config',
                                self.cell['reachability']['file'])
        self.reach = ReachabilityMap.load(map_path)
        self.spec = PalletSpec.from_config(self.cell, self.reach.usable_top_height)
        physics = None
        if self.cell['packing']['physics']['enabled'] and self.planner_name == 'heuristic':
            from palletizer_packing.core.physics import PhysicsChecker, physics_config
            physics = PhysicsChecker(physics_config(self.cell))
        self.planner = make_planner(self.planner_name, self.cell, self.reach, physics)
        pal = self.cell['pallet']
        self.x0 = pal['center_xy'][0] - pal['size'][0] / 2
        self.y_edge = pal['center_xy'][1] + pal['size'][1] / 2
        self.z_top = pal['size'][2]
        self.state = PalletState(self.spec)
        self.ghost = None

        self.create_service(PlanPlacement, '/packing/plan_placement', self.on_plan)
        self.create_service(CommitPlacement, '/packing/commit', self.on_commit)
        self.create_service(Trigger, '/packing/reset', self.on_reset)
        self.marker_pub = self.create_publisher(MarkerArray, '/packing/markers', 10)
        self.state_pub = self.create_publisher(String, '/packing/state', 10)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            f'packing ready: planner {self.planner_name}, max stack height '
            f'{self.spec.max_height:.2f} m (reachability map), physics '
            f'{"on" if physics else "off"}')

    # ------------------------------------------------------------ conversions
    def to_msg(self, p: Placement, box: Box):
        m = PlacementMsg()
        u, v = p.center
        m.top_center.position.x = self.x0 + u
        m.top_center.position.y = self.y_edge - v
        m.top_center.position.z = self.z_top + p.top
        m.yaw = math.pi / 2 if p.rotated else 0.0
        m.top_center.orientation.z = math.sin(m.yaw / 2)
        m.top_center.orientation.w = math.cos(m.yaw / 2)
        m.size_x, m.size_y = p.du, p.dv
        m.height = p.height
        m.rest_z = self.z_top + p.z0
        m.rotated = p.rotated
        m.score = float(p.score)
        m.planner = self.planner_name
        m.planning_time = float(p.info.get('planning_time', 0.0))
        m.candidates_checked = int(p.info.get('candidates_checked', 0))
        m.physics_checks = int(p.info.get('physics_checks', 0))
        return m

    def from_msg(self, m: PlacementMsg, box: Box):
        u = m.top_center.position.x - self.x0
        v = self.y_edge - m.top_center.position.y
        du, dv = (box.width, box.length) if m.rotated else (box.length, box.width)
        return Placement(u0=u - du / 2, v0=v - dv / 2, du=du, dv=dv,
                         z0=m.rest_z - self.z_top, height=box.height, rotated=m.rotated)

    @staticmethod
    def box_from(det, mass):
        return Box(max(det.length, det.width), min(det.length, det.width), det.height,
                   mass if mass > 0 else 0.0)

    # ------------------------------------------------------------ services
    def on_plan(self, req, res):
        box = self.box_from(req.box, req.mass)
        exclude = [self.from_msg(m, box) for m in req.exclude]
        self.planner.exclude = exclude
        p = self.planner.plan(self.state, box, check_only=req.check_only)
        self.planner.exclude = []
        stats = self.planner.last_stats
        if p is None:
            res.success = False
            res.pallet_full = True
            res.message = (f'no feasible placement ({stats.get("candidates_checked", 0)} candidates, '
                           f'{stats.get("planning_time", 0.0) * 1000:.0f} ms)')
            self.ghost = None
            return res
        res.success = True
        res.placement = self.to_msg(p, box)
        res.message = (f'{"rotated " if p.rotated else ""}at u={p.u0:.3f} v={p.v0:.3f} '
                       f'z={p.z0:.3f}, {res.placement.planning_time * 1000:.0f} ms')
        if not req.check_only:
            self.ghost = (p, box)
            self.get_logger().info(f'plan: {res.message}')
        return res

    def on_commit(self, req, res):
        box = self.box_from(req.box, req.mass)
        p = self.from_msg(req.placement, box)
        self.state.place(p, box, req.sim_model_name)
        self.ghost = None
        res.success = True
        res.message = (f'{len(self.state.boxes)} boxes, fill {self.state.fill_percent():.1f} %, '
                       f'max height {self.state.max_height:.2f} m')
        self.publish_markers()
        return res

    def on_reset(self, req, res):
        n = len(self.state.boxes)
        self.state = PalletState(self.spec)
        self.ghost = None
        res.success = True
        res.message = f'pallet model reset ({n} boxes removed)'
        self.publish_markers()
        return res

    # ------------------------------------------------------------ visualization
    def publish_markers(self):
        arr = MarkerArray()
        hm = Marker()
        hm.header.frame_id = 'world'
        hm.ns = 'heightmap'
        hm.type = Marker.CUBE_LIST
        r = self.spec.resolution * 2
        hm.scale.x = hm.scale.y = r
        hm.scale.z = 0.005
        hm.pose.orientation.w = 1.0
        H = self.state.height
        step = 2
        for iv in range(0, H.shape[0], step):
            for iu in range(0, H.shape[1], step):
                h = float(H[iv:iv + step, iu:iu + step].max())
                if h <= 0:
                    continue
                hm.points.append(Point(x=self.x0 + (iu + 1) * self.spec.resolution,
                                       y=self.y_edge - (iv + 1) * self.spec.resolution,
                                       z=self.z_top + h + 0.003))
                t = min(1.0, h / max(self.spec.max_height, 1e-6))
                hm.colors.append(ColorRGBA(r=t, g=0.3, b=1.0 - t, a=0.8))
        if not hm.points:
            hm.action = Marker.DELETE
        arr.markers.append(hm)
        g = Marker()
        g.header.frame_id = 'world'
        g.ns = 'ghost'
        if self.ghost is None:
            g.action = Marker.DELETE
        else:
            p, box = self.ghost
            u, v = p.center
            g.type = Marker.CUBE
            g.pose.position = Point(x=self.x0 + u, y=self.y_edge - v,
                                    z=self.z_top + p.z0 + p.height / 2)
            g.pose.orientation.w = 1.0
            g.scale.x, g.scale.y, g.scale.z = p.du, p.dv, p.height
            g.color = ColorRGBA(r=0.2, g=1.0, b=0.3, a=0.35)
        arr.markers.append(g)
        self.marker_pub.publish(arr)
        self.state_pub.publish(String(data=json.dumps({
            'planner': self.planner_name, 'boxes': len(self.state.boxes),
            'fill_percent': self.state.fill_percent(), 'max_height': self.state.max_height})))


def main():
    rclpy.init()
    rclpy.spin(PackingNode())


if __name__ == '__main__':
    main()
