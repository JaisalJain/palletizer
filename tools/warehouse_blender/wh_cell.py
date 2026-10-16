"""Demo-only replica of the palletizing cell (already modelled in the SDF world, so NOT
exported): pedestal, UR10e with the suction gripper, conveyor, camera masts, the pallet with
a stack in progress and cartons on the belts. Dimensions come from cell_config.
The UR10e kinematic chain mirrors ur_description (validated: park pose -> TCP (0.35, 0.08, 1.65)).
"""
import bpy, math, os, random
from mathutils import Matrix, Vector
import wh_lib as L
from wh_lib import MB, mat, T

PI = math.pi
# ur10e default_kinematics.yaml: joint origins parent -> child (xyz, rpy); all joints about z
UR_J = [((0, 0, 0.1807), (0, 0, 0)), ((0, 0, 0), (1.570796327, 0, 0)), ((-0.6127, 0, 0), (0, 0, 0)),
        ((-0.57155, 0, 0.17415), (0, 0, 0)), ((0, -0.11985, 0), (1.570796327, 0, 0)),
        ((0, 0.11655, 0), (1.570796326589793, PI, PI))]
# visual_parameters.yaml mesh_offset (x, y, z, roll, pitch, yaw) per link
UR_VIS = [("shoulder", (0, 0, 0), (0, 0, PI)), ("upperarm", (0, 0, 0.1762), (PI / 2, 0, -PI / 2)),
          ("forearm", (0, 0, 0.0393), (PI / 2, 0, -PI / 2)), ("wrist1", (0, 0, -0.135), (PI / 2, 0, 0)),
          ("wrist2", (0, 0, -0.12), (0, 0, 0)), ("wrist3", (0, -0.0005, -0.1168), (PI / 2, 0, 0))]
TOOL0 = T(rpy=(0, -PI / 2, -PI / 2)) @ T(rpy=(PI / 2, 0, PI / 2))   # wrist_3 -> flange -> tool0

Q_HERO = [-2.3495, -1.9257, 1.3621, -1.0072, -1.5708, 2.3629]      # TCP (-0.15, -0.40, 1.494)
BOX = (0.294, 0.394, 0.300)                                          # 400x300x300 SKU, x/y as stacked


def cell_materials(M):
    M["cv_frame"] = mat("cell_conveyor_frame", (0.35, 0.37, 0.40), 0.4, 0.6)
    M["belt"] = mat("cell_belt", (0.05, 0.05, 0.05), 0.7)
    L.add_noise_variation(M["belt"], 30.0, 0.3)
    M["alu"] = mat("cell_aluminium", (0.78, 0.8, 0.82), 0.3, 1.0)
    M["pedestal"] = mat("cell_pedestal", (0.25, 0.27, 0.30), 0.45, 0.4)
    M["suction_grey"] = mat("cell_suction_grey", (0.55, 0.57, 0.6), 0.3, 0.8)
    M["suction_blue"] = mat("cell_suction_cup", (0.1, 0.3, 0.8), 0.5)
    M["cam"] = mat("cell_camera", (0.06, 0.06, 0.07), 0.35, 0.2)
    M["lens"] = mat("cell_lens", (0.02, 0.02, 0.03), 0.05, 0.0, coat=1.0)
    M["label"] = L.textured("cell_label", "label.png", rough=0.4, scale=1.0, gz_rgba=(0.95, 0.95, 0.9, 1))
    M["tape"] = mat("cell_tape", (0.72, 0.56, 0.36), 0.25, coat=0.6)
    M["ir"] = mat("cell_ir_projector", (0.6, 0.05, 0.05), 0.2, emit=1.5)
    return M


def carton(name, dims, center, yaw, M, C, rng, kraft=None, labels=True):
    lx, ly, lz = dims
    mb = MB()
    k = kraft or M[f"kraft{rng.randrange(4)}"]
    mb.add_box((0, 0, 0), dims, k)
    long_x = lx >= ly
    tl, tw = (lx, 0.05) if long_x else (0.05, ly)
    mb.add_box((0, 0, lz / 2 + 0.0006), (tl + 0.002, tw, 0.0012), M["tape"])
    for s in (-1, 1):   # tape flaps down the ends
        if long_x:
            mb.add_box((s * (lx / 2 + 0.0006), 0, lz / 2 - 0.035), (0.0012, 0.05, 0.07), M["tape"])
        else:
            mb.add_box((0, s * (ly / 2 + 0.0006), lz / 2 - 0.035), (0.05, 0.0012, 0.07), M["tape"])
    if labels:
        w, h = 0.15, 0.10
        y = ly / 2 + 0.0008
        mb.add_quad([(-w / 2 + lx * 0.12, y, -h / 2), (w / 2 + lx * 0.12, y, -h / 2),
                     (w / 2 + lx * 0.12, y, h / 2), (-w / 2 + lx * 0.12, y, h / 2)], M["label"],
                    uvs=[(1, 0), (0, 0), (0, 1), (1, 1)])
        x = lx / 2 + 0.0008
        mb.add_quad([(x, -w / 2, -h / 2 - 0.03), (x, w / 2, -h / 2 - 0.03), (x, w / 2, h / 2 - 0.03),
                     (x, -w / 2, h / 2 - 0.03)], M["label"], uvs=[(0, 0), (1, 0), (1, 1), (0, 1)])
    ob = mb.build(name, C)
    ob.location = center
    ob.rotation_euler = (0, 0, yaw)
    bv = ob.modifiers.new("bevel", "BEVEL")
    bv.width, bv.segments, bv.limit_method = 0.004, 2, "ANGLE"
    return ob


def import_glb(path, C, name):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=path)
    new = [o for o in bpy.data.objects if o not in before]
    meshes = [o for o in new if o.type == "MESH"]
    for o in meshes:
        mw = o.matrix_world.copy()
        o.parent = None
        o.matrix_world = mw
    for o in meshes:
        o.data = o.data.copy()
        o.data.transform(o.matrix_world)
        o.matrix_world = Matrix.Identity(4)
    base = meshes[0]
    names = [o.name for o in new]
    if len(meshes) > 1:
        with bpy.context.temp_override(active_object=base, object=base, selected_objects=meshes,
                                       selected_editable_objects=meshes):
            bpy.ops.object.join()
    for n in names:
        o = bpy.data.objects.get(n)
        if o is not None and o != base:
            bpy.data.objects.remove(o, do_unlink=True)
    for c in list(base.users_collection):
        c.objects.unlink(base)
    C.objects.link(base)
    base.name = name
    base.data.name = name
    for m in base.data.materials:
        if m is None:
            continue
        b = m.node_tree.nodes.get("Principled BSDF") if m.use_nodes else None
        if b:
            c = b.inputs["Base Color"].default_value
            lum = 0.3 * c[0] + 0.6 * c[1] + 0.1 * c[2]
            b.inputs["Roughness"].default_value = 0.28 if lum > 0.3 else 0.45
            if "Coat Weight" in b.inputs and lum > 0.3:
                b.inputs["Coat Weight"].default_value = 0.3
    return base


def robot(M, C, cfg, q=Q_HERO):
    ped = cfg["robot"]["pedestal"]
    pz = ped["size"][2]
    root = L.empty("ur_base_link", C, T((ped["xy"][0], ped["xy"][1], pz)))
    bli = L.empty("ur_base_link_inertia", C, T(rpy=(0, 0, PI)), parent=root)
    adir = os.path.join(L.ROOT, "assets", "ur10e")
    vis = import_glb(os.path.join(adir, "base.glb"), C, "ur_vis_base")
    vis.parent = bli
    vis.matrix_basis = T(rpy=(0, 0, PI))
    parent, links = bli, []
    for i, ((xyz, rpy), (mesh, vxyz, vrpy)) in enumerate(zip(UR_J, UR_VIS)):
        jo = L.empty(f"ur_j{i + 1}_origin", C, T(xyz, rpy), parent=parent, size=0.05)
        ln = L.empty(f"ur_link{i + 1}", C, Matrix.Identity(4), parent=jo, size=0.08)
        ln.rotation_mode = "XYZ"
        ln.rotation_euler = (0, 0, q[i])
        v = import_glb(os.path.join(adir, mesh + ".glb"), C, f"ur_vis_{mesh}")
        v.parent = ln
        v.matrix_basis = T(vxyz, vrpy)
        links.append(ln)
        parent = ln
    tool0 = L.empty("ur_tool0", C, TOOL0, parent=links[-1], size=0.05)
    g = cfg["gripper"]
    gm = MB()
    gm.add_cyl((0, 0, g["body_length"] / 2), g["body_radius"], g["body_length"], M["suction_grey"], seg=32)
    gm.add_cyl((0, 0, 0.012), 0.063, 0.024, M["suction_grey"], seg=32)                    # flange adapter
    gm.add_cyl((0, 0, g["body_length"] + g["cup_length"] / 2), g["cup_radius"] * 0.8, g["cup_length"],
               M["suction_blue"], seg=32, r2=g["cup_radius"])
    gm.add_cyl((0.05, 0, 0.07), 0.008, 0.06, M["dark"] if "dark" in M else M["cam"], axis="X", seg=10)  # vacuum hose stub
    gob = gm.build("ur_suction_gripper", C, smooth=False)
    gob.parent = tool0
    gob.matrix_basis = Matrix.Identity(4)
    tcp = L.empty("ur_suction_tcp", C, T((0, 0, g["body_length"] + g["cup_length"])), parent=tool0, size=0.04)
    return root, links, tcp


def set_pose(links, q):
    for ln, qi in zip(links, q):
        ln.rotation_euler = (0, 0, qi)
    bpy.context.view_layer.update()


def equipment(M, C, cfg):
    cv = cfg["conveyor"]
    x0, x1, y, bw, zt = cv["x_start"], cv["x_end"], cv["y"], cv["belt_width"], cv["belt_top_z"]
    e = MB()
    # conveyor: belt, side rails, return frame, legs, end stop, drive
    e.add_box(((x0 + x1) / 2, y, zt - 0.02), (x1 - x0, bw, 0.04), M["belt"])
    for s in (-1, 1):
        e.add_box(((x0 + x1) / 2, y + s * (bw / 2 + 0.03), zt - 0.03), (x1 - x0 + 0.04, 0.04, 0.14), M["cv_frame"])
        e.add_box(((x0 + x1) / 2, y + s * (bw / 2 + 0.03), zt - 0.28), (x1 - x0, 0.03, 0.03), M["cv_frame"])
    for zc in cv["zones"][1:]:
        e.add_box((zc["x_min"], y, zt + 0.0005), (0.006, bw, 0.001), M["cv_frame"])   # zone seams
    for x in (x0 + 0.15, x0 + 1.1, x0 + 2.05, x1 - 0.15):
        for s in (-1, 1):
            e.add_box((x, y + s * (bw / 2 + 0.03), (zt - 0.1) / 2), (0.06, 0.06, zt - 0.1), M["cv_frame"])
            e.add_box((x, y + s * (bw / 2 + 0.03), 0.01), (0.14, 0.14, 0.02), M["cv_frame"])
        e.add_box((x, y, 0.2), (0.04, bw + 0.06, 0.04), M["cv_frame"])
    e.add_box((x1 + 0.02, y, zt + 0.09), (0.04, bw + 0.1, 0.18), M["yellow"])
    e.add_cyl((x1 - 0.2, y - bw / 2 - 0.16, zt - 0.1), 0.08, 0.25, M["crate_blue"], axis="Y", seg=20)
    e.add_box((x1 - 0.2, y - bw / 2 - 0.3, zt - 0.1), (0.2, 0.08, 0.2), M["cv_frame"])
    # photo-eyes (emitter robot side, reflector far side), as in the SDF
    pe = cv["photo_eye"]
    for z in cv["zones"]:
        e.add_box((z["eye_x"], y - bw / 2 - pe["offset"], zt + pe["height"]), (0.03, 0.03, 0.05), M["green_emit"])
        e.add_box((z["eye_x"], y + bw / 2 + pe["offset"], zt + pe["height"]), (0.04, 0.01, 0.05), M["red_emit"])
        for s in (-1, 1):
            e.add_box((z["eye_x"], y + s * (bw / 2 + pe["offset"]), (zt + pe["height"]) / 2), (0.02, 0.02, zt + pe["height"] - 0.03), M["alu"])
    # pedestal with base plate + anchors
    ped = cfg["robot"]["pedestal"]
    sx, sy, sz = ped["size"]
    e.add_box((0, 0, sz / 2), (sx, sy, sz - 0.04), M["pedestal"])
    e.add_box((0, 0, 0.012), (0.5, 0.5, 0.024), M["pedestal"])
    e.add_box((0, 0, sz - 0.015), (0.3, 0.3, 0.03), M["pedestal"])
    for dx in (-0.2, 0.2):
        for dy in (-0.2, 0.2):
            e.add_cyl((dx, dy, 0.035), 0.018, 0.024, M["galv"], seg=6)
    for s in (-1, 1):   # gussets
        e.add_box((s * (sx / 2 + 0.04), 0, 0.12), (0.08, 0.01, 0.2), M["pedestal"])
        e.add_box((0, s * (sy / 2 + 0.04), 0.12), (0.01, 0.08, 0.2), M["pedestal"])
    # camera masts, arms, housings (overhead RGB-D cameras)
    for key in ("camera", "pallet_camera"):
        c = cfg[key]
        mx, my = c["mast_xy"]
        mh = c["mast_size"][2]
        cx, cy, cz = c["xyz"]
        e.add_box((mx, my, mh / 2), (0.08, 0.08, mh), M["dark"])
        e.add_box((mx, my, 0.01), (0.4, 0.4, 0.02), M["dark"])
        e.add_box((mx, (my + cy) / 2, cz + 0.06), (0.06, abs(my - cy) + 0.1, 0.06), M["dark"])
        e.add_box((cx, cy, cz + 0.015), (0.03, 0.14, 0.03), M["cam"])
        for dyy in (-0.045, 0.0, 0.045):
            e.add_cyl((cx, cy + dyy, cz - 0.002), 0.008, 0.004, M["lens"] if dyy else M["ir"], seg=12)
        e.add_beam((mx, my, mh - 0.35), (mx, (my + cy) / 2, cz + 0.03), 0.03, 0.03, M["dark"])
        e.add_cyl((mx + 0.05, my, mh - 0.5), 0.006, 1.0, M["black"], seg=6)          # cable
    # station markings, as in the SDF (lane lines + station in/out lines)
    st, ex = cfg["pallet_station"], cfg["exchanger"]
    lane_len = ex["exit_x"] - ex["entry_x"] + 1.4
    for s in (-1, 1):
        e.add_box(((ex["entry_x"] + ex["exit_x"]) / 2, ex["lane_y"] + s * (st["area"][1] / 2 + 0.03), 0.001),
                  (lane_len, 0.05, 0.002), M["paint_y"])
        e.add_box((st["center_xy"][0] + s * st["area"][0] / 2, st["center_xy"][1], 0.0015),
                  (0.03, st["area"][1], 0.003), M["paint_w"])
    ob = e.build("CELL_equipment", C)
    return ob


def stack(M, C, cfg, rng, layers_full=3, top_count=5):
    cx, cy = cfg["pallet_station"]["center_xy"]
    pal = L_pallet(M, C, "CELL_pallet", (cx, cy, 0), 0.0)
    kraft = M["kraft1"]
    obs = []
    xs = [-0.45, -0.15, 0.15, 0.45]
    order = [(i, 0) for i in range(4)] + [(i, 1) for i in range(4)]   # far row first
    for l in range(layers_full + 1):
        n = 8 if l < layers_full else top_count
        for k, (i, j) in enumerate(order[:n]):
            yy = cy - 0.2 if j == 0 else cy + 0.2
            c = (cx + xs[i] + rng.uniform(-0.002, 0.002), yy + rng.uniform(-0.002, 0.002),
                 0.144 + BOX[2] / 2 + BOX[2] * l)
            obs.append(carton(f"CELL_box_L{l}_{k}", BOX, c, rng.uniform(-0.004, 0.004), M, C, rng, kraft))
    return pal, obs


def L_pallet(M, C, name, loc, yaw):
    import wh_env
    p = wh_env.pallet_mb(M, True)
    ob = p.build(name, C)
    ob.location, ob.rotation_euler = loc, (0, 0, yaw)
    return ob


def belt_cartons(M, C, cfg, rng):
    cv = cfg["conveyor"]
    zt, y = cv["belt_top_z"], cv["y"]
    sk = [(0.394, 0.294, 0.3)]
    spots = [(0.02, 0.20), (-1.05, -0.15), (-2.05, 0.30)]           # pick zone, accum, infeed
    obs = [carton(f"CELL_belt_box_{i}", sk[0], (x, y + rng.uniform(-0.03, 0.03), zt + 0.15), yaw, M, C, rng)
           for i, (x, yaw) in enumerate(spots)]
    for i, x in enumerate((-4.6, -6.3, -9.3, -11.8, -12.9)):        # upstream line (demo only)
        obs.append(carton(f"CELL_up_box_{i}", sk[0], (x, y + rng.uniform(-0.05, 0.05), zt + 0.15),
                          rng.uniform(-0.2, 0.2), M, C, rng))
    return obs


def build(cfg, M, seed=5):
    rng = random.Random(seed)
    root = L.coll("CELL_DEMO")
    C = L.coll("CELL_demo_objects", root)
    cell_materials(M)
    equipment(M, C, cfg)
    pal, boxes = stack(M, C, cfg, rng)
    belt = belt_cartons(M, C, cfg, rng)
    base, links, tcp = robot(M, C, cfg, Q_HERO)
    set_pose(links, Q_HERO)
    held = carton("CELL_held_box", BOX, (0, 0, 0), 0.0, M, C, rng, M["kraft1"])
    tw = tcp.matrix_world
    held.matrix_world = Matrix.Translation((tw.translation.x, tw.translation.y, tw.translation.z - BOX[2] / 2))
    bpy.context.view_layer.update()
    mw = held.matrix_world.copy()
    held.parent = tcp
    held.matrix_world = mw
    return dict(links=links, tcp=tcp, held=held, boxes=boxes, belt=belt, pallet=pal)
