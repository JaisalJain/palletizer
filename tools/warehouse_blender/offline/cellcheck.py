"""Collision / clearance checker for the palletizing cell (python-fcl + UR10e collision meshes).
World frame = cell frame. Boxes are given as (name, center xyz, size xyz, yaw)."""
import numpy as np, fcl, trimesh, json, os
from scipy.spatial.transform import Rotation as Rot
from urkin import link_frames, T, TOOL, pi
HERE = os.path.dirname(os.path.abspath(__file__))
# UR_COLLISION=<ur_description>/meshes/ur10e/collision
MESH = os.environ.get("UR_COLLISION", os.path.join(HERE, "ur10e_collision"))
VIS = [("base", (0, 0, 0), (0, 0, pi)), ("shoulder", (0, 0, 0), (0, 0, pi)),
       ("upperarm", (0, 0, 0.1762), (pi/2, 0, -pi/2)), ("forearm", (0, 0, 0.0393), (pi/2, 0, -pi/2)),
       ("wrist1", (0, 0, -0.135), (pi/2, 0, 0)), ("wrist2", (0, 0, -0.12), (0, 0, 0)),
       ("wrist3", (0, -0.0005, -0.1168), (pi/2, 0, 0))]
def _bvh(m):
    b = fcl.BVHModel(); b.beginModel(len(m.vertices), len(m.faces))
    b.addSubModel(np.asarray(m.vertices, float), np.asarray(m.faces, np.int32)); b.endModel(); return b
LINK_GEOM = [_bvh(trimesh.load(os.path.join(MESH, n + ".stl"))) for n, _, _ in VIS]
LINK_OFF = [T(xyz, rpy) for _, xyz, rpy in VIS]
LINK_NAMES = [n for n, _, _ in VIS]
def _tf(M): return fcl.Transform(M[:3, :3], M[:3, 3])
def box_obj(c, s, yaw=0.0, shrink=0.0):
    s = np.maximum(np.array(s, float) - 2 * shrink, 1e-4)
    M = np.eye(4); M[:3, :3] = Rot.from_euler("z", yaw).as_matrix(); M[:3, 3] = c
    return fcl.CollisionObject(fcl.Box(*s), _tf(M))
def static_env(cfg):
    cv = cfg["conveyor"]; x0, x1, y, bw, zt = cv["x_start"], cv["x_end"], cv["y"], cv["belt_width"], cv["belt_top_z"]
    env = {
        "pedestal": ((0, 0, 0.35), (0.25, 0.25, 0.70)),
        "conveyor": (((x0 + x1) / 2, y, zt / 2), (x1 - x0, bw, zt)),
        "rail_near": (((x0 + x1) / 2, y - bw / 2 - 0.03, zt - 0.03), (x1 - x0 + 0.04, 0.04, 0.14)),
        "rail_far": (((x0 + x1) / 2, y + bw / 2 + 0.03, zt - 0.03), (x1 - x0 + 0.04, 0.04, 0.14)),
        "end_stop": ((x1 + 0.02, y, zt + 0.09), (0.04, bw + 0.1, 0.18)),
        "drive": ((x1 - 0.2, y - bw / 2 - 0.2, zt - 0.1), (0.2, 0.3, 0.2)),
        "mast_conv": ((0, cfg["camera"]["mast_xy"][1], 1.2), (0.08, 0.08, 2.4)),
        "arm_conv": ((0, (cfg["camera"]["mast_xy"][1] + cfg["camera"]["xyz"][1]) / 2, 2.36), (0.06, 0.75, 0.06)),
        "mast_pal": ((0, cfg["pallet_camera"]["mast_xy"][1], 1.45), (0.08, 0.08, 2.9)),
        "arm_pal": ((0, (cfg["pallet_camera"]["mast_xy"][1] + cfg["pallet_camera"]["xyz"][1]) / 2, 2.86), (0.06, 1.05, 0.06)),
        "floor": ((0, 0, -0.05), (20, 20, 0.1)),
    }
    for z in cv["zones"]:
        for s in (-1, 1):
            env[f"eye_{z['name']}_{s}"] = ((z["eye_x"], y + s * (bw / 2 + cv["photo_eye"]["offset"]), 0.43), (0.03, 0.03, 0.86))
    return {k: box_obj(c, s) for k, (c, s) in env.items()}
def robot_objs(q, held=None):
    """Robot link objects (+ gripper, + held box (dims, yaw_rel, gap))."""
    F = link_frames(q)
    objs = {}
    for i, (g, off) in enumerate(zip(LINK_GEOM, LINK_OFF)):
        objs[LINK_NAMES[i]] = fcl.CollisionObject(g, _tf(F[i] @ off))
    tool0 = F[-1] @ TOOL
    objs["gripper"] = fcl.CollisionObject(fcl.Cylinder(0.05, 0.15), _tf(tool0 @ T((0, 0, 0.075))))
    tcp = tool0 @ T((0, 0, 0.15))
    if held is not None:
        dims, c, yaw = held
        objs["held_box"] = box_obj(c, dims, yaw, shrink=0.001)
    return objs, tcp
# pairs never checked (adjacent / mounted)
SKIP = {("base", "pedestal"), ("base", "shoulder"), ("shoulder", "upperarm"), ("upperarm", "forearm"),
        ("forearm", "wrist1"), ("wrist1", "wrist2"), ("wrist2", "wrist3"), ("wrist3", "gripper"),
        ("gripper", "held_box"), ("wrist2", "gripper"), ("shoulder", "pedestal"), ("wrist1", "wrist3")}
def _skip(a, b): return (a, b) in SKIP or (b, a) in SKIP
def collide(a, b):
    req = fcl.CollisionRequest(enable_contact=True, num_max_contacts=4); res = fcl.CollisionResult()
    fcl.collide(a, b, req, res)
    return max([c.penetration_depth for c in res.contacts], default=0.0) if res.is_collision else 0.0
def distance(a, b):
    r = fcl.DistanceResult(); return fcl.distance(a, b, fcl.DistanceRequest(), r)
def check_frame(q, held, boxes, env, tol=0.002):
    """boxes: list of (name, c, size, yaw). Returns list of (a, b, depth) with depth > tol."""
    robot, tcp = robot_objs(q, held)
    hits = []
    bx = {n: box_obj(c, s, y, shrink=0.001) for n, c, s, y in boxes}
    names = list(robot)
    for i, a in enumerate(names):          # self collision
        for b in names[i + 1:]:
            if not _skip(a, b):
                d = collide(robot[a], robot[b])
                if d > tol: hits.append((a, b, d))
    for a in names:                        # robot vs world
        for n, o in list(env.items()) + list(bx.items()):
            if _skip(a, n) or (a == "base" and n in ("pedestal",)):
                continue
            d = collide(robot[a], o)
            if d > tol: hits.append((a, n, d))
    bn = list(bx)                          # box vs box (belt queue, stack)
    for i, a in enumerate(bn):
        for b in bn[i + 1:]:
            d = collide(bx[a], bx[b])
            if d > tol: hits.append((a, b, d))
        for n, o in env.items():
            if n in ("conveyor", "floor"):  # boxes rest on these
                continue
            d = collide(bx[a], o)
            if d > tol: hits.append((a, n, d))
    return hits, tcp
