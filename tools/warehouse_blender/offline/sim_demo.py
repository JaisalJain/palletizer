"""Offline emulation of the palletizer stack for the demo video (no ROS / Gazebo).

Mirrors the real software, not an animator's guess:
  * palletize.xml cycle: WaitBoxReady -> DetectBox (no arm over the ROI) -> PickBox ->
    ResumeConveyor -> PlaceBox -> VerifyRelease -> PalletFull? -> ClosePallet (Park, Verify,
    PalletExchange out/in, MeasurePallet) -> first box on the new pallet
  * motion_server.py: solve() / unwrap_near() / route_pan() / hub_z() / lift_z(),
    pick = PTP pre-grasp, tare, descend (LIN + slow LIN), suction, LIN up, PTP lift, weigh;
    place = PTP above target at hub height, vertical descend, release, settle, LIN retreat;
    park = hub over the station, then the park joints
  * Pilz PTP: joint-synchronized trapezoid at the UR10e limits (120/180 deg/s, 3 rad/s^2) x scaling
    Pilz LIN: Cartesian trapezoid, 1.0 m/s / 2.0 m/s^2 x lin scalings, fallback scalings
  * conveyor_controller: zones with photo-eyes, pick zone latches on its eye until resume
  * pallet_exchanger: trapezoid 0.35 m/s, 0.4 m/s^2, full pallet out, new pallet in with pose noise
  * pattern: 400x300x300 on EUR = 7/7 x 3 layers (docs/RESULTS.md), slot in front of the pedestal dropped
Every robot motion is collision-checked (python-fcl, UR10e collision meshes) while planning.
Output: assets/anim2.json (24 fps) + a report.
"""
import json, math, os, sys, time as _time
import numpy as np
from scipy.spatial.transform import Rotation
try:                                         # the repo's own IK (palletizer_tools.ur_ik)
    from palletizer_tools.ur_ik import UrKinematics, LOWER, UPPER
except ImportError:
    import os as _os
    try:                                     # inside the repo: tools/warehouse_blender/offline
        sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "../../../src/palletizer_tools"))
        from palletizer_tools.ur_ik import UrKinematics, LOWER, UPPER
    except ImportError:                      # standalone copy next to this file
        from ur_ik import UrKinematics, LOWER, UPPER
import cellcheck as CC

CFG = json.load(open(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ.get("ROBOT_WORLD", "."), "assets", "cell_config.json")))
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.environ.get("ROBOT_WORLD", "."), "assets", "anim2.json")
K = UrKinematics(CFG)
M = CFG["motion"]; CV = CFG["conveyor"]; EX = CFG["exchanger"]; ST = CFG["pallet_station"]
HZ, FPS = 240, 24
DT = 1.0 / HZ
VMAX = np.radians([120, 120, 180, 180, 180, 180]); AMAX = np.full(6, 3.0)
LIN_V, LIN_A = 1.0, 2.0
BOX = (0.394, 0.294, 0.300)                  # carton L x W x H (length axis = yaw axis)
DECK = 0.144
rng = np.random.default_rng(7)

# ----------------------------------------------------------------------------- world state
class Obj:
    def __init__(s, name, size, pose, kind):
        s.name, s.size, s.kind = name, tuple(size), kind
        s.x, s.y, s.z, s.yaw = pose
        s.visible, s.v, s.zone = True, 0.0, None
    def pose(s): return [s.x, s.y, s.z, s.yaw]
objs = {}
def add(o): objs[o.name] = o; return o

# pallet A at the station with the stack in progress
pal_A = add(Obj("pallet_A", (1.2, 0.8, DECK), (0.0, -0.6, 0.0, 0.0), "pallet"))
pallet = {"obj": pal_A, "boxes": []}
# pattern (pallet frame: u = +x, v = away from the robot = -y, origin at the robot-side -u corner)
FAR = [(0.15 + 0.3 * i, 0.6) for i in range(4)]
NEAR = [(0.15 + 0.3 * i, 0.2) for i in range(4)]
def slot_world(pal, u, v, layer):
    c, s = math.cos(pal.yaw), math.sin(pal.yaw)
    lu, lv = u - 0.6, -(v - 0.4)              # from the deck centre, in pallet axes (x=u, y=-v)
    return (pal.x + c * lu - s * lv, pal.y + s * lu + c * lv, DECK + 0.3 * layer + 0.15, pal.yaw + math.pi / 2)

# ----------------------------------------------------------------------------- kinematics (motion_server)
HOME = np.array(CFG["robot"]["home"]); PARK = np.array(CFG["robot"]["park"])
_t = K.fk(HOME)[:3, 3]; OFF = math.atan2(_t[1], _t[0]) - HOME[0]
def seed_for(xy):
    q = HOME.copy(); q[0] = math.atan2(xy[1], xy[0]) - OFF; return q
def unwrap_near(q, ref):
    q = q.copy()
    for i in range(6):
        while q[i] - ref[i] > math.pi and q[i] - 2 * math.pi >= LOWER[i]: q[i] -= 2 * math.pi
        while q[i] - ref[i] < -math.pi and q[i] + 2 * math.pi <= UPPER[i]: q[i] += 2 * math.pi
    return q
def solve(xyz, yaws, ref, seed=None):
    seed = seed_for(xyz) if seed is None else seed
    best = None
    for yaw in yaws:
        q = K.ik(K.down_pose(xyz, yaw), seed)
        if q is None: continue
        q = unwrap_near(q, ref)
        cost = abs(q[5] - ref[5]) + 0.5 * max(0.0, abs(q[5]) - math.pi) + 0.1 * float(np.sum(np.abs(q[:5] - seed[:5])))
        if best is None or cost < best[0]: best = (cost, q, yaw)
    return (best[1], best[2]) if best else (None, None)
def route_pan(q):
    s = M["swing_bearing"]; b = q[0] + OFF
    b = s + ((b - s + math.pi) % (2 * math.pi)) - math.pi
    if b <= s - math.pi: b += 2 * math.pi
    q = q.copy(); q[0] = b - OFF; return q
stack_top = 0.0
def hub_z(h): return max(CV["belt_top_z"] + M["lift_clearance"], stack_top + M["stack_clearance"]) + h + M["pick_gap"]
def lift_z(h): return CV["belt_top_z"] + M["lift_clearance"] + h + M["pick_gap"]
def tcp_of(q): return K.fk(q)
def tool_yaw(q): T = K.fk(q); return math.atan2(T[1, 0], T[0, 0])

# ----------------------------------------------------------------------------- collision (planning scene)
ENV = CC.static_env(CFG)
held = None                                  # {"obj":Obj, "rel": 4x4 TCP->box}
def scene_boxes(exclude=()):
    out = []
    for o in objs.values():
        if not o.visible or o.name in exclude or (held and o is held["obj"]): continue
        if o.kind == "pallet":
            out.append((o.name, (o.x, o.y, DECK / 2), o.size, o.yaw))
        else:
            out.append((o.name, (o.x, o.y, o.z), o.size, o.yaw))
    return out
def held_geom(q):
    if not held: return None
    T = K.fk(q) @ held["rel"]
    return (held["obj"].size, T[:3, 3], math.atan2(T[1, 0], T[0, 0]))
def path_hits(qs, exclude=(), every=6):
    env = dict(ENV)
    boxes = scene_boxes(exclude)
    worst = []
    for q in qs[::every] + [qs[-1]]:
        robot, _ = CC.robot_objs(q, held_geom(q))
        for a, ra in robot.items():
            for n, o in list(env.items()) + [(b[0], CC.box_obj(b[1], b[2], b[3], 0.001)) for b in boxes]:
                if CC._skip(a, n) or (a == "base" and n == "pedestal"): continue
                if a == "held_box" and n in ("floor",): continue
                d = CC.collide(ra, o)
                if d > 0.002: worst.append((a, n, round(d, 4)))
        if worst: return worst
    return worst

# ----------------------------------------------------------------------------- motion profiles
def trapezoid(dist, v, a):
    """Samples s in [0,1] at HZ for a trapezoid (triangle if short) over dist."""
    if dist < 1e-9: return [1.0]
    v = min(v, math.sqrt(dist * a)); ta = v / a; da = 0.5 * a * ta * ta
    tc = (dist - 2 * da) / v; Tt = 2 * ta + tc
    n = max(2, int(math.ceil(Tt * HZ))); out = []
    for k in range(1, n + 1):
        t = Tt * k / n
        if t < ta: d = 0.5 * a * t * t
        elif t < ta + tc: d = da + v * (t - ta)
        else: tt = Tt - t; d = dist - 0.5 * a * tt * tt
        out.append(d / dist)
    return out
def ptp_samples(q0, q1, vs=1.0, as_=1.0):
    d = q1 - q0
    # synchronized: the parameter profile limited by the slowest joint
    lead = [(abs(di) / (VMAX[i] * vs), abs(di) / (AMAX[i] * as_)) for i, di in enumerate(d)]
    Tv = max(l[0] for l in lead); Ta = max(l[1] for l in lead)
    if Tv < 1e-9: return [q1.copy()]
    v, a = 1.0 / Tv, 1.0 / Ta                # in s-units
    return [q0 + d * s for s in trapezoid(1.0, v, a)]
def lin_samples(q0, xyz, yaw, vs, as_):
    p0 = K.fk(q0)[:3, 3]; p1 = np.array(xyz, float); L = np.linalg.norm(p1 - p0)
    ss = trapezoid(L, LIN_V * vs, LIN_A * as_)
    out, q = [], q0.copy()
    for s in ss:
        qn = K.ik(K.down_pose(p0 + (p1 - p0) * s, yaw), q)
        if qn is None: return None
        qn = unwrap_near(qn, q)
        if np.max(np.abs(qn - q)) > 0.2: return None      # IK branch jump: not a valid LIN
        out.append(qn); q = qn
    return out
def joint_speed_ok(qs, q_prev):
    a = np.vstack([q_prev] + qs)
    return np.all(np.abs(np.diff(a, axis=0)) * HZ <= VMAX * 1.02)

# ----------------------------------------------------------------------------- simulation runtime
q = np.array(PARK, float)
t = 0.0; tick = 0
rec = {"q": [], "obj": {}, "led": [], "andon": [], "phase": []}
events = []; report = {"ptp": 0, "lin": 0, "ptp_hub_fallback": 0, "lin_fallback": 0, "collisions": []}
phase = "start"; andon = False
def ev(name): events.append((round(t, 3), name)); print(f"{t:7.2f}s  {name}", flush=True)

# conveyor: zones incl. the visual upstream extension (-14 .. x_start)
ZONES = [{"name": "upstream", "x0": -14.0, "x1": CV["x_start"], "eye": CV["x_start"] - 0.05}] + \
        [{"name": z["name"], "x0": z["x_min"], "x1": z["x_max"], "eye": z["eye_x"]} for z in CV["zones"]]
for z in ZONES: z.update(run=True, v=0.0, latched=False)
def ext(o): return (o.size[0] * abs(math.cos(o.yaw)) + o.size[1] * abs(math.sin(o.yaw))) / 2
belt = []                                    # objects on the belt, any order
def spawn_belt(name, x, dy, yaw):
    o = add(Obj(name, BOX, (x, CV["y"] + dy, CV["belt_top_z"] + BOX[2] / 2, yaw), "box")); belt.append(o); return o
def eye_blocked(z):
    return any(o.x - ext(o) <= z["eye"] <= o.x + ext(o) for o in belt)
pick_eye_clear_since = 0.0; resume_req = False
def conveyor_step():
    global pick_eye_clear_since, resume_req
    pick = ZONES[-1]
    blocked = eye_blocked(pick)
    if blocked and not pick["latched"] and pick["run"]:
        pick["latched"] = True
    if not blocked: pick_eye_clear_since = pick_eye_clear_since or t
    else: pick_eye_clear_since = 0.0
    if resume_req and not blocked and t - (pick_eye_clear_since or t) >= CV["photo_eye"]["removal_confirm"]:
        pick["latched"] = False; resume_req = False
    pick["run"] = not pick["latched"]
    for i in range(len(ZONES) - 2, -1, -1):
        z, nxt = ZONES[i], ZONES[i + 1]
        z["run"] = (not eye_blocked(z)) or nxt["run"]
    for z in ZONES:                          # 1 m/s^2 belt ramps
        tgt = CV["speed"] if z["run"] else 0.0
        z["v"] += max(-DT * 1.0, min(DT * 1.0, tgt - z["v"]))
    order = sorted(belt, key=lambda o: -o.x)
    for i, o in enumerate(order):
        zone = next((z for z in ZONES if z["x0"] <= o.x < z["x1"]), ZONES[-1])
        nx = o.x + zone["v"] * DT
        if i > 0:                            # never into the box ahead (physical contact)
            f = order[i - 1]; nx = min(nx, f.x - ext(f) - ext(o) - 0.02)
        nx = min(nx, CV["x_end"] - ext(o))   # end stop
        o.x = max(o.x, nx)
def box_ready():
    pick = ZONES[-1]
    return pick["latched"] and pick["v"] == 0.0 and eye_blocked(pick) and _ready_since() >= CV["settle_time"]
_stopped_at = [None]
def _ready_since():
    pick = ZONES[-1]
    if pick["latched"] and pick["v"] == 0.0 and eye_blocked(pick):
        if _stopped_at[0] is None: _stopped_at[0] = t
        return t - _stopped_at[0]
    _stopped_at[0] = None; return 0.0
def pick_box_obj():
    pick = ZONES[-1]
    c = [o for o in belt if o.x - ext(o) <= pick["eye"] <= o.x + ext(o)]
    return c[0] if c else None

# pallet mover
mover = None                                 # {"objs":[...], "dx":..., "samples":[...], "i":0, "start":{name:x}}
def mover_step():
    global mover
    if not mover: return
    s = mover["s"][mover["i"]]
    for o in mover["objs"]: o.x = mover["x0"][o.name] + mover["dx"] * s
    mover["i"] += 1
    if mover["i"] >= len(mover["s"]): mover = None
def start_move(os_, dx):
    global mover
    mover = {"objs": os_, "dx": dx, "x0": {o.name: o.x for o in os_}, "s": trapezoid(abs(dx), EX["speed"], EX["accel"]), "i": 0}

def record():
    rec["q"].append([round(float(v), 5) for v in q])
    for o in objs.values():
        tr = rec["obj"].setdefault(o.name, {"size": o.size, "kind": o.kind, "first": len(rec["q"]) - 1, "track": []})
        tr["track"].append([round(o.x, 5), round(o.y, 5), round(o.z, 5), round(o.yaw, 5)] if o.visible else None)
    rec["led"].append(bool(held)); rec["andon"].append(andon); rec["phase"].append(phase)

def step():
    """Advance the world by one tick (robot pose q is set by the caller)."""
    global t, tick
    conveyor_step(); mover_step()
    if held:
        T = K.fk(q) @ held["rel"]; o = held["obj"]
        o.x, o.y, o.z = T[:3, 3]; o.yaw = math.atan2(T[1, 0], T[0, 0])
    if tick % (HZ // FPS) == 0: record()
    t += DT; tick += 1
def sleep(sec):
    for _ in range(int(round(sec * HZ))): step()
def wait_until(pred, timeout=600):
    t0 = t
    while not pred():
        step()
        if t - t0 > timeout: raise RuntimeError("wait timeout")
def play(qs):
    global q
    for qq in qs:
        q = np.array(qq); step()

# ----------------------------------------------------------------------------- motion API (motion_server)
def ptp(q_goal, exclude=()):
    """Pilz PTP (collision-checked). Falls back to a via point above (the OMPL fallback in spirit)."""
    report["ptp"] += 1
    qs = ptp_samples(q, q_goal)
    hits = path_hits(qs, exclude)
    if hits:
        report["ptp_hub_fallback"] += 1
        T = K.fk(q)[:3, 3]; via = np.array([T[0], T[1], min(T[2] + 0.25, 1.55)])
        qv, _ = solve(via, [tool_yaw(q)], ref=q, seed=q)
        if qv is not None:
            qs1 = ptp_samples(q, route_pan(qv)); qs2 = ptp_samples(qs1[-1], q_goal)
            if not path_hits(qs1 + qs2, exclude): qs, hits = qs1 + qs2, []
        if hits: report["collisions"].append(("ptp", round(t, 2), hits[:3]))
    sleep(0.08)                              # planning / execution latency
    play(qs)
def lin(xyz, yaw, slow=False, exclude=()):
    report["lin"] += 1
    vs = M["lin_slow_velocity_scaling"] if slow else M["lin_velocity_scaling"]; as_ = M["lin_acceleration_scaling"]
    qs = lin_samples(q, xyz, yaw, vs, as_)
    for fv, fa in [(vs, as_)] + [tuple(x) for x in M["lin_fallback_scaling"]]:
        if qs is not None and joint_speed_ok(qs, q): break
        report["lin_fallback"] += 1
        qs = lin_samples(q, xyz, yaw, min(vs, fv), min(as_, fa))
    if qs is None: raise RuntimeError(f"LIN failed to {xyz}")
    hits = path_hits(qs, exclude)
    if hits: report["collisions"].append(("lin", round(t, 2), hits[:3]))
    sleep(0.05)
    play(qs)
def descend(xyz, yaw, exclude=()):
    above = np.array(xyz) + [0, 0, M["slow_zone"]]
    lin(above, yaw, exclude=exclude); lin(xyz, yaw, slow=True, exclude=exclude)

def pick(box):
    global held, phase
    phase = "pick"
    top = np.array([box.x, box.y, box.z + box.size[2] / 2])
    yaws = [box.yaw + k * math.pi / 2 for k in range(4)]
    pre = top + [0, 0, M["pre_grasp_height"]]
    q_pre, yaw = solve(pre, yaws, ref=q)
    ptp(route_pan(q_pre), exclude=(box.name,))             # target box is an obstacle only near the end
    sleep(M["weigh_settle"] + M["weigh_window"])            # tare
    descend(top + [0, 0, M["pick_gap"]], yaw, exclude=(box.name,))
    sleep(0.12)                                              # suction on -> grasp confirmed
    T = K.fk(q); Tb = np.eye(4); Tb[:3, :3] = Rotation.from_euler("z", box.yaw).as_matrix(); Tb[:3, 3] = [box.x, box.y, box.z]
    held = {"obj": box, "rel": np.linalg.inv(T) @ Tb, "yaw_offset": yaw - box.yaw}
    belt.remove(box); ev(f"grasp {box.name}")
    lin(pre, yaw)
    lift = np.array([pre[0], pre[1], max(pre[2], lift_z(box.size[2]), hub_z(box.size[2]))])
    q_lift, _ = solve(lift, [yaw], ref=q, seed=q)
    ptp(route_pan(q_lift))
    sleep(M["weigh_settle"] + M["weigh_window"]); ev(f"weighed {box.name}")
def place(target):
    """target: (x, y, z_center, yaw_of_length_axis)."""
    global held, phase, stack_top
    phase = "place"
    box = held["obj"]; h = box.size[2]
    top = np.array([target[0], target[1], target[2] + h / 2])
    top_in_tcp_z = -(h / 2) + held["rel"][2, 3] * 0 - M["pick_gap"]      # box top sits pick_gap below the cup
    # Tool yaw (2 candidates, box symmetric under pi) x wrist-3 branch (+-2pi): the one that
    # turns the CARRIED BOX least in the world over the joint-space transit. motion_server
    # minimizes |dq6| only; with a ~150 deg base swing that can spin the box by up to ~270 deg.
    best = None
    q_start = route_pan(q)
    for k in range(2):
        yaw = target[3] + held["yaw_offset"] + k * math.pi
        pre_tcp = top + [0, 0, M["pre_place_height"] + M["pick_gap"]]
        qp, _ = solve(pre_tcp, [yaw], ref=q)
        if qp is None: continue
        for w in (-2 * math.pi, 0.0, 2 * math.pi):
            qc = route_pan(qp.copy()); qc[5] += w
            if not (LOWER[5] <= qc[5] <= UPPER[5]): continue
            rot = world_rotation(q_start, qc)
            cost = abs(rot) + 0.1 * abs(qc[5] - q[5])
            if best is None or cost < best[0]: best = (cost, qc, yaw)
    _, q_pre, yaw = best
    pre_tcp = top + [0, 0, M["pre_place_height"] + M["pick_gap"]]
    rel_tcp = top + [0, 0, M["release_gap"] + M["pick_gap"]]
    z = max(hub_z(h), pre_tcp[2]); q_above = None
    while z >= pre_tcp[2] - 1e-6:
        qa, _ = solve(np.array([pre_tcp[0], pre_tcp[1], z]), [yaw], ref=q_pre, seed=q_pre)
        if qa is not None and not path_hits([route_pan(qa)]):
            q_above, above = qa, np.array([pre_tcp[0], pre_tcp[1], z]); break
        z -= 0.05
    q_above = route_pan(q_above); q_above[5] = unwrap_near(q_above, q_pre)[5]
    ptp(q_above)
    descend(rel_tcp, yaw)
    sleep(0.1)                                               # suction off -> release confirmed
    o = held["obj"]; held = None; ev(f"release {o.name}")
    z0 = o.z
    for k in range(1, 5):                                    # 5 mm drop onto the stack
        o.z = z0 - M["release_gap"] * min(1.0, (k / 4) ** 2); step()
    o.z = target[2]; o.x, o.y = target[0], target[1]
    o.yaw = target[3] + math.pi * round((o.yaw - target[3]) / math.pi)   # same physical box: keep its label side
    pallet["boxes"].append(o); stack_top = max(stack_top, target[2] + h / 2)
    sleep(M["release_settle"])
    lin(K.fk(q)[:3, 3] + [0, 0, M["retreat_height"]], yaw)
    sleep(0.12)                                              # VerifyRelease (F/T) + commit
def world_rotation(qa, qb, n=24):
    """Tool (= carried box) yaw change in the world along the straight joint-space path."""
    tot, prev = 0.0, tool_yaw(qa)
    for k in range(1, n + 1):
        y = tool_yaw(qa + (qb - qa) * k / n); tot += math.remainder(y - prev, 2 * math.pi); prev = y
    return tot
def go_park():
    global phase
    phase = "park"
    T = K.fk(q); yaw = math.atan2(T[1, 0], T[0, 0])
    sx, sy = pallet["obj"].x, pallet["obj"].y
    b = math.atan2(sy, sx); r = M["hub_radius"]
    hub = np.array([r * math.cos(b), r * math.sin(b), max(hub_z(0.0), T[2, 3])])
    qh, _ = solve(hub, [yaw], ref=q, seed=seed_for(hub))
    ptp(route_pan(qh)); ptp(route_pan(PARK.copy()))
def arm_occludes_roi():
    x0, x1 = CFG["pick_zone"]["roi_x"]; hw = CV["belt_width"] / 2 + CFG["perception"]["roi_margin_y"] + CFG["perception"]["occlusion_margin"]
    F = CC.link_frames(q)
    pts = [F[i][:3, 3] for i in (5, 6)] + [K.fk(q)[:3, 3]]
    return any(x0 - 0.1 <= p[0] <= x1 + 0.1 and abs(p[1] - CV["y"]) <= hw for p in pts)

# ----------------------------------------------------------------------------- scenario
def build_pattern():
    """7/7 x 3: drop the near-row slot the arm cannot reach without hitting the pedestal
    (found with the same collision model at layer 1)."""
    return None
def main():
    global phase, andon, pallet, stack_top, held, q
    # pattern: far row 4 + near row 3; which near slot is dropped is decided by a reachability test
    dropped = choose_dropped_slot()
    near = [s for i, s in enumerate(NEAR) if i != dropped]
    order = FAR + near
    # start: 2 full layers + far row of layer 3 placed (18 boxes), robot parked, boxes queued
    names = 0
    for layer in range(3):
        for k, (u, v) in enumerate(order):
            if layer == 2 and k >= 4: break
            x, y, z, yaw = slot_world(pal_A, u, v, layer)
            o = add(Obj(f"pal_A_box_{names:02d}", BOX, (x + rng.uniform(-0.0015, 0.0015), y + rng.uniform(-0.0015, 0.0015), z, yaw + rng.uniform(-0.003, 0.003)), "box"))
            pallet["boxes"].append(o); names += 1
    stack_top = DECK + 0.9
    # belt queue (conveyor + upstream line), yaw within the spawner's +-25 deg
    spawn_belt("belt_00", -0.05, 0.010, 0.21)
    spawn_belt("belt_01", -0.72, -0.020, -0.14)
    spawn_belt("belt_02", -1.62, 0.015, 0.33)
    spawn_belt("belt_03", -2.40, -0.010, -0.05)
    for i, x in enumerate((-4.6, -6.3, -9.3, -11.8, -12.9, -13.7)):
        spawn_belt(f"belt_{4 + i:02d}", x, float(rng.uniform(-0.04, 0.04)), float(rng.uniform(-0.3, 0.3)))
    # the robot starts over the stack, just retreated from its last place (as in a running cycle)
    x, y, z, yaw = slot_world(pal_A, *order[3], 2)
    q0, _ = solve(np.array([x, y, z + 0.15 + 0.15 + 0.003]), [math.pi / 2 + 0.0], ref=seed_for((x, y)))
    q = route_pan(q0)
    ev("start")
    layer, idx = 2, 4
    placed_on_new = 0
    while True:
        phase = "wait_box"
        wait_until(lambda: box_ready() and not arm_occludes_roi())
        phase = "detect"; sleep(5 / 15 + 0.1)             # 5 depth frames @15 Hz + processing
        box = pick_box_obj(); ev(f"detected {box.name}")
        sleep(0.05)                                        # PreCheckFit
        pick(box)
        global resume_req
        resume_req = True; ev("resume conveyor")
        sleep(0.05)                                        # recheck with measured mass
        if placed_on_new == 0 and pallet["obj"] is pal_A:
            u, v = order[idx]; target = slot_world(pal_A, u, v, layer)
        else:
            u, v = order[placed_on_new]; target = slot_world(pallet["obj"], u, v, 0)
        place(target)
        if pallet["obj"] is not pal_A:
            placed_on_new += 1
            ev("first box on the new pallet"); break
        idx += 1
        if idx == len(order):                              # PATTERN_COMPLETE -> ClosePallet
            ev("PATTERN_COMPLETE (21 boxes)"); andon = True
            go_park(); ev("parked")
            phase = "verify"; sleep(0.8 + 0.3)            # VerifyPlacement (4 frames @5 Hz)
            phase = "exchange_out"; ev("exchange: full pallet out")
            start_move([pal_A] + pallet["boxes"], 4.6 - pal_A.x); wait_until(lambda: mover is None)
            # the full pallet waits at the stretch-wrapper infeed (outside the fence)
            phase = "exchange_in"
            tx, ty, tyaw = ST["center_xy"][0] + 0.012, ST["center_xy"][1] - 0.007, 0.011
            pal_B = add(Obj("pallet_B", (1.2, 0.8, DECK), (-4.2, ty, 0.0, tyaw), "pallet"))
            ev("exchange: new pallet in"); start_move([pal_B], tx - (-4.2)); wait_until(lambda: mover is None)
            sleep(0.5)
            pallet = {"obj": pal_B, "boxes": []}; stack_top = DECK
            phase = "measure"; sleep(4 / 5 + 0.4); ev("pallet measured"); andon = False
    phase = "done"
    go_park(); sleep(1.5)
    ev("end")

def choose_dropped_slot():
    """Near-row slot whose layer-1 release pose collides the folded arm with the pedestal
    (or has the smallest clearance): the one the reachability map drops."""
    best = None
    for i, (u, v) in enumerate(NEAR):
        x, y, z, yaw = slot_world(pal_A, u, v, 0)
        qq, _ = solve(np.array([x, y, z + 0.15 + 0.008]), [yaw, yaw + math.pi], ref=seed_for((x, y)))
        if qq is None: d = -1.0
        else:
            robot, _ = CC.robot_objs(route_pan(qq))
            d = min(CC.distance(robot[n], ENV["pedestal"]) for n in ("upperarm", "forearm", "wrist1", "wrist2"))
        print(f"  near slot u={u:.2f}: min arm-pedestal clearance {d*1000:.0f} mm")
        if best is None or d < best[0]: best = (d, i)
    print(f"  dropped slot: near row u={NEAR[best[1]][0]:.2f}")
    return best[1]

if __name__ == "__main__":
    t_wall = _time.time()
    main()
    N = len(rec["q"])
    for o in rec["obj"].values():                   # continuous yaw (no +-pi wraps for Blender)
        tr = o["track"]; prev = None
        for v in tr:
            if v is None: continue
            if prev is not None: v[3] = prev + math.remainder(v[3] - prev, 2 * math.pi)
            prev = v[3]
    out = {"fps": FPS, "frames": N, "q": rec["q"], "objects": rec["obj"], "led": rec["led"], "andon": rec["andon"],
           "phase": rec["phase"], "events": events, "report": report}
    json.dump(out, open(OUT, "w"))
    print(f"frames {N} ({N / FPS:.1f} s), wall {(_time.time() - t_wall):.0f} s, report {json.dumps({k: v for k, v in report.items() if k != 'collisions'})}")
    for c in report["collisions"]: print("  PLANNING COLLISION", c)
