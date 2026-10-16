"""Export the warehouse (collection WH_export) as a Gazebo model: palletizer_warehouse.

  models/palletizer_warehouse/
    model.config, model.sdf
    meshes/warehouse_shell.obj|.mtl     building, roof, services, docks  (cast_shadows=false:
                                        the roof must not shadow the cell under the world's sun)
    meshes/warehouse_interior.obj|.mtl  racks, loads, fence, props       (casts shadows)
    meshes/*.png                        floor / hazard / HMI textures
Materials are exported flat (gz_rgba) or with their UV texture (gz_tex); procedural render-only
detail is dropped. Collision: axis-aligned boxes decoded from the COL_* proxies.
OBJ is written Z-up / Y-forward, i.e. in the SDF model frame (= world frame, cell origin).
"""
import bpy, os, shutil, math
from mathutils import Vector, Matrix
import wh_lib as L

MODEL = "palletizer_warehouse"


def _gz_material(src):
    name = "GZ__" + src.name
    m = bpy.data.materials.get(name)
    if m:
        return m
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    rgba = list(src.get("gz_rgba", list(src.diffuse_color)))
    b.inputs["Base Color"].default_value = rgba[:3] + [1.0]
    sb = src.node_tree.nodes.get("Principled BSDF") if src.use_nodes else None
    rough = sb.inputs["Roughness"].default_value if sb and not sb.inputs["Roughness"].is_linked else 0.6
    metal = sb.inputs["Metallic"].default_value if sb else 0.0
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if len(rgba) > 3 and rgba[3] < 1.0:
        b.inputs["Alpha"].default_value = rgba[3]
    if src.get("gz_emissive"):
        b.inputs["Emission Color"].default_value = rgba[:3] + [1.0]
        b.inputs["Emission Strength"].default_value = 1.0
    tex = src.get("gz_tex")
    if tex:
        ti = nt.nodes.new("ShaderNodeTexImage")
        ti.image = L.img(os.path.join(L.TEX, tex))
        nt.links.new(ti.outputs["Color"], b.inputs["Base Color"])
    return m


def _merged(objs, name):
    dg = bpy.context.evaluated_depsgraph_get()
    tmp = []
    for o in objs:
        me = bpy.data.meshes.new_from_object(o.evaluated_get(dg))
        me.transform(o.matrix_world)
        for i, m in enumerate(me.materials):
            if m is not None:
                me.materials[i] = _gz_material(m)
        t = bpy.data.objects.new("tmp_" + o.name, me)
        bpy.context.scene.collection.objects.link(t)
        tmp.append(t)
    with bpy.context.temp_override(active_object=tmp[0], object=tmp[0], selected_objects=tmp,
                                   selected_editable_objects=tmp):
        bpy.ops.object.join()
    ob = tmp[0]
    ob.name = name
    return ob


def _write_obj(ob, path):
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True, forward_axis="Y", up_axis="Z",
                          export_materials=True, path_mode="STRIP", export_triangulated_mesh=True,
                          export_normals=True, export_uv=True, apply_modifiers=True,
                          export_object_groups=False, export_material_groups=False)


def _fix_mtl(path):
    """Blender writes no Kd for a textured material; Gazebo's OBJ loader then defaults the diffuse
    colour to black and multiplies the texture by it (black floor). Textured -> Kd 1 1 1."""
    blocks, cur = [], []
    for line in open(path):
        if line.startswith("newmtl") and cur:
            blocks.append(cur)
            cur = []
        cur.append(line)
    blocks.append(cur)
    with open(path, "w") as f:
        for b in blocks:
            if any(l.startswith("map_Kd") for l in b) and not any(l.startswith("Kd ") for l in b):
                b.insert(next(i for i, l in enumerate(b) if l.startswith("Ka ")) + 1, "Kd 1.000000 1.000000 1.000000\n")
            f.writelines(b)


def collision_boxes():
    """COL_* meshes are made only of MB.add_box (8 verts each, axis-aligned)."""
    boxes = []
    for o in bpy.data.collections["WH_collision"].objects:
        vs = [o.matrix_world @ v.co for v in o.data.vertices]
        for k in range(0, len(vs), 8):
            chunk = vs[k:k + 8]
            lo = Vector([min(v[i] for v in chunk) for i in range(3)])
            hi = Vector([max(v[i] for v in chunk) for i in range(3)])
            boxes.append((o.name, (lo + hi) / 2, hi - lo))
    return boxes


def export(repo_models_dir):
    out = os.path.join(repo_models_dir, MODEL)
    mdir = os.path.join(out, "meshes")
    os.makedirs(mdir, exist_ok=True)
    src = [o for o in bpy.data.collections["WH_export"].objects if o.type == "MESH"]
    groups = {"shell": [o for o in src if o.get("gz_group") == "shell"],
              "interior": [o for o in src if o.get("gz_group") == "interior"]}
    stats = {}
    for g, objs in groups.items():
        ob = _merged(objs, f"EXPORT_{g}")
        p = os.path.join(mdir, f"warehouse_{g}.obj")
        _write_obj(ob, p)
        _fix_mtl(p[:-4] + ".mtl")
        stats[g] = dict(objects=len(objs), tris=sum(len(pl.vertices) - 2 for pl in ob.data.polygons),
                        mb=round(os.path.getsize(p) / 1e6, 1))
        me = ob.data
        bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.meshes.remove(me)
    used = set()
    for g in groups:
        with open(os.path.join(mdir, f"warehouse_{g}.mtl")) as f:
            for line in f:
                if line.strip().startswith("map_"):
                    used.add(line.split()[-1])
    for t in used:
        shutil.copy(os.path.join(L.TEX, os.path.basename(t)), os.path.join(mdir, os.path.basename(t)))
    boxes = collision_boxes()
    write_sdf(out, boxes)
    for m in [m for m in bpy.data.materials if m.name.startswith("GZ__")]:
        bpy.data.materials.remove(m)
    return dict(stats=stats, textures=sorted(used), collision_boxes=len(boxes), out=out)


def write_sdf(out, boxes):
    cols = "\n".join(
        f"""      <collision name="{n}_{i}">
        <pose>{c.x:.4f} {c.y:.4f} {c.z:.4f} 0 0 0</pose>
        <geometry><box><size>{s.x:.4f} {s.y:.4f} {s.z:.4f}</size></box></geometry>
      </collision>""" for i, (n, c, s) in enumerate(boxes))
    uri = f"model://{MODEL}/meshes"
    sdf = f"""<?xml version="1.0"?>
<!--
  Warehouse environment around the palletizing cell (visual set dressing + coarse collision).
  Generated by tools/warehouse_blender/wh_export.py from the Blender scene; do not edit by hand.
  Model frame = world frame (cell origin under the robot base axis). Nothing in this model is
  inside the robot's reach, the pallet lane, the photo-eye beams or either camera's perception
  ROI (checked by tools/warehouse_blender/offline/wh_verify.py). The conveyor camera's image
  corners see the base of the north fence, outside its ROI.
  The ground collision stays the world's ground_plane; this model only adds the floor visual.
-->
<sdf version="1.9">
  <model name="{MODEL}">
    <static>true</static>
    <link name="building">
      <!-- no shadows: roof and walls would otherwise put the whole cell in shadow -->
      <visual name="shell">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>{uri}/warehouse_shell.obj</uri></mesh></geometry>
      </visual>
      <visual name="interior">
        <cast_shadows>true</cast_shadows>
        <geometry><mesh><uri>{uri}/warehouse_interior.obj</uri></mesh></geometry>
      </visual>
{cols}
    </link>
  </model>
</sdf>
"""
    with open(os.path.join(out, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(out, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{MODEL}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>palletizer</name></author>
  <description>Warehouse around the palletizing cell: racking, docks, infeed line, stretch
    wrapper, fencing. Built procedurally in Blender from cell_config.yaml.</description>
</model>
""")
