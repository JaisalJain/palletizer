"""Drive the demo scene from the offline stack emulation (assets/anim2.json, sim_demo.py):
robot joints, every carton (stack, belt, carried), both pallets, gripper vacuum LED, andon.
Replaces an earlier hand-keyed animation that had collisions."""
import bpy, json, os, math, random
import wh_lib as L
import wh_cell, wh_env

OLD_PREFIX = ("CELL_box_", "CELL_belt_box_", "CELL_up_box_", "CELL_held_box", "CELL_pallet")


def _fcurve(ob, path, index):
    ad = ob.animation_data_create()
    if ad.action is None:
        ad.action = bpy.data.actions.new(ob.name + "_act")
    act = ad.action
    try:
        return act.fcurve_ensure_for_datablock(ob, path, index=index)       # Blender 4.4+
    except Exception:
        ob.keyframe_insert(path, index=index, frame=0)
        for fc in _all_fcurves(act):
            if fc.data_path == path and fc.array_index == index:
                fc.keyframe_points.clear()
                return fc


def _all_fcurves(act):
    try:
        return list(act.fcurves)
    except Exception:
        out = []
        for layer in act.layers:
            for strip in layer.strips:
                for cb in strip.channelbags:
                    out += list(cb.fcurves)
        return out


def _keys(ob, path, index, frames, values, interp="LINEAR", owner=None):
    """Write keyframes in one go (foreach_set). Drops keys inside constant/linear runs."""
    keep = [0]
    for k in range(1, len(values) - 1):
        a, b, c = values[k - 1], values[k], values[k + 1]
        if interp == "CONSTANT":
            if b != a:
                keep.append(k)
        elif abs((a + c) / 2 - b) > 1e-6:
            keep.append(k)
    if len(values) > 1:
        keep.append(len(values) - 1)
    fc = _fcurve(owner or ob, path, index)
    fc.keyframe_points.clear()
    fc.keyframe_points.add(len(keep))
    co = []
    for k in keep:
        co += [float(frames[k]), float(values[k])]
    fc.keyframe_points.foreach_set("co", co)
    ienum = {"CONSTANT": 0, "LINEAR": 1, "BEZIER": 2}[interp]
    fc.keyframe_points.foreach_set("interpolation", [ienum] * len(keep))
    fc.update()


def clear_old():
    for o in list(bpy.data.objects):
        if o.name.startswith(OLD_PREFIX) or o.name.startswith("SIM_"):
            bpy.data.objects.remove(o, do_unlink=True)
    for n in ("ur_link1", "ur_link2", "ur_link3", "ur_link4", "ur_link5", "ur_link6"):
        o = bpy.data.objects.get(n)
        if o:
            o.animation_data_clear()


def _emissive(name, rgb):
    """Indicator lamp: dark lens when off (base colour), coloured only through emission."""
    m = L.mat(name, (0.04, 0.045, 0.045), 0.25, emit=0.0)
    m.node_tree.nodes["Principled BSDF"].inputs["Emission Color"].default_value = tuple(rgb) + (1.0,)
    return m


def apply(path=None, seed=5):
    data = json.load(open(path or os.path.join(L.ROOT, "assets", "anim2.json")))
    N, fps = data["frames"], data["fps"]
    sc = bpy.context.scene
    sc.render.fps = fps
    sc.frame_start, sc.frame_end = 1, N
    clear_old()
    C = L.coll("CELL_sim_objects", L.coll("CELL_DEMO"))
    M = wh_env.materials()
    wh_cell.cell_materials(M)
    rng = random.Random(seed)
    frames = list(range(1, N + 1))
    # robot
    for i in range(6):
        ln = bpy.data.objects[f"ur_link{i + 1}"]
        _keys(ln, "rotation_euler", 2, frames, [row[i] for row in data["q"]])
    # objects
    for name, o in data["objects"].items():
        f0 = o["first"]
        if o["kind"] == "pallet":
            ob = wh_env.pallet_mb(M, True).build("SIM_" + name, C)
        else:
            ob = wh_cell.carton("SIM_" + name, tuple(o["size"]), (0, 0, 0), 0.0, M, C, rng,
                                M[f"kraft{rng.randrange(4)}"])
        tr = o["track"]
        last = next(v for v in tr if v is not None)
        rows, vis = [], []
        for k in range(N):
            j = k - f0
            v = tr[j] if 0 <= j < len(tr) else None
            vis.append(v is not None)
            last = v if v is not None else last
            rows.append(last)
        ob.location = rows[0][:3]
        ob.rotation_euler = (0, 0, rows[0][3])
        if o["kind"] == "pallet":
            rows = [[r[0], r[1], 0.0, r[3]] for r in rows]
        for c in range(3):
            _keys(ob, "location", c, frames, [r[c] for r in rows])
        _keys(ob, "rotation_euler", 2, frames, [r[3] for r in rows])
        if not all(vis):
            for p in ("hide_render", "hide_viewport"):
                _vis_keys(ob, p, frames, vis)
    # gripper vacuum LED (green while a box is held)
    tool0 = bpy.data.objects["ur_tool0"]
    mb = L.MB()
    led = _emissive("SIM_vacuum_led", (0.1, 1.0, 0.3))
    mb.add_cyl((0, 0, 0.095), 0.0415, 0.012, led, seg=32, caps=False)
    ring = mb.build("SIM_vacuum_led", C)
    ring.parent = tool0
    ring.matrix_parent_inverse.identity()
    _node_keys(led, frames, [6.0 if v else 0.0 for v in data["led"]])
    # andon amber (pallet FULL -> exchange -> measured)
    am = _emissive("SIM_andon_amber", (1.0, 0.55, 0.02))
    mb = L.MB()
    mb.add_cyl((3.8, 2.1, 2.48), 0.043, 0.075, am, seg=24)
    mb.build("SIM_andon_amber", C)
    _node_keys(am, frames, [10.0 if v else 0.0 for v in data["andon"]])
    sc.frame_set(1)
    return data


def _vis_keys(ob, prop, frames, vis):
    vals = [0.0 if v else 1.0 for v in vis]
    keep = [0] + [k for k in range(1, len(vals)) if vals[k] != vals[k - 1]]
    for k in keep:
        setattr(ob, prop, bool(vals[k]))
        ob.keyframe_insert(prop, frame=frames[k])
    setattr(ob, prop, bool(vals[0]))


def _node_keys(mat, frames, vals):
    b = mat.node_tree.nodes["Principled BSDF"]
    sock = b.inputs["Emission Strength"]
    keep = [0] + [k for k in range(1, len(vals)) if vals[k] != vals[k - 1]]
    for k in keep:
        sock.default_value = vals[k]
        sock.keyframe_insert("default_value", frame=frames[k])
    try:
        for fc in _all_fcurves(mat.node_tree.animation_data.action):
            for kp in fc.keyframe_points:
                kp.interpolation = "CONSTANT"
    except Exception:
        pass
