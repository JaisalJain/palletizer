"""Shared helpers for the warehouse world build (runs inside Blender 5.x).

MeshBuilder batches thousands of boxes / beams / cylinders into one mesh per group, so
the scene stays light in Blender and exports as a few draw calls for Gazebo.
Materials carry a `gz_rgba` custom property (and optionally `gz_tex`) used by the exporter.
"""
import bpy, bmesh, math, json, os, random
from mathutils import Vector, Matrix, Euler

# working folder (assets/, renders/, *.blend): $ROBOT_WORLD, else the parent of this script's folder
ROOT = os.environ.get("ROBOT_WORLD", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TEX = os.path.join(ROOT, "assets", "textures")


def cfg():
    with open(os.path.join(ROOT, "assets", "cell_config.json")) as f:
        return json.load(f)


# ----------------------------------------------------------------------------- scene
def coll(name, parent=None):
    c = bpy.data.collections.get(name)
    if c is None:
        c = bpy.data.collections.new(name)
        (parent or bpy.context.scene.collection).children.link(c)
    return c


def wipe_collection(name):
    c = bpy.data.collections.get(name)
    if not c:
        return
    for ch in list(c.children):
        wipe_collection(ch.name)
    for o in list(c.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(c)


def purge():
    for _ in range(3):
        bpy.data.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)


# ----------------------------------------------------------------------------- materials
_M = {}


def _bsdf(m):
    m.use_nodes = True
    nt = m.node_tree
    return nt, nt.nodes.get("Principled BSDF")


def _set(b, key, val):
    if key in b.inputs:
        b.inputs[key].default_value = val


def mat(name, rgba, rough=0.5, metal=0.0, emit=0.0, alpha=1.0, transmission=0.0, coat=0.0,
        gz=None):
    """Flat PBR material. gz: override Gazebo colour (defaults to rgba)."""
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    nt, b = _bsdf(m)
    col = tuple(rgba[:3]) + (1.0,)
    _set(b, "Base Color", col)
    _set(b, "Roughness", rough)
    _set(b, "Metallic", metal)
    _set(b, "Transmission Weight", transmission)
    _set(b, "Coat Weight", coat)
    if emit > 0:
        _set(b, "Emission Color", col)
        _set(b, "Emission Strength", emit)
    if alpha < 1.0:
        _set(b, "Alpha", alpha)
        try:
            m.surface_render_method = "BLENDED"
        except Exception:
            pass
    m.diffuse_color = col
    m["gz_rgba"] = list(gz or (list(rgba[:3]) + [alpha]))
    m["gz_emissive"] = emit > 0
    _M[name] = m
    return m


def img(path, colorspace="sRGB"):
    n = os.path.basename(path)
    im = bpy.data.images.get(n)
    if im is None:
        im = bpy.data.images.load(path, check_existing=True)
    im.colorspace_settings.name = colorspace
    return im


def textured(name, albedo, rough_img=None, rough=0.5, metal=0.0, box_proj=None, scale=1.0,
             gz_rgba=(0.6, 0.6, 0.6, 1), bump=0.0):
    """Image-textured material. UV-mapped unless box_proj (object-space box projection,
    for render-only use; the exporter then falls back to gz_rgba)."""
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    nt, b = _bsdf(m)
    for n in list(nt.nodes):
        if n.type not in ("BSDF_PRINCIPLED", "OUTPUT_MATERIAL"):
            nt.nodes.remove(n)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (scale, scale, scale)
    nt.links.new(tc.outputs["Object" if box_proj else "UV"], mp.inputs["Vector"])
    ti = nt.nodes.new("ShaderNodeTexImage")
    ti.image = img(os.path.join(TEX, albedo))
    if box_proj:
        ti.projection = "BOX"
        ti.projection_blend = 0.2
    nt.links.new(mp.outputs["Vector"], ti.inputs["Vector"])
    nt.links.new(ti.outputs["Color"], b.inputs["Base Color"])
    if rough_img:
        tr = nt.nodes.new("ShaderNodeTexImage")
        tr.image = img(os.path.join(TEX, rough_img), "Non-Color")
        nt.links.new(mp.outputs["Vector"], tr.inputs["Vector"])
        nt.links.new(tr.outputs["Color"], b.inputs["Roughness"])
    else:
        _set(b, "Roughness", rough)
    _set(b, "Metallic", metal)
    if bump > 0:
        bn = nt.nodes.new("ShaderNodeBump")
        bn.inputs["Strength"].default_value = bump
        bn.inputs["Distance"].default_value = 0.002
        nt.links.new(ti.outputs["Color"], bn.inputs["Height"])
        nt.links.new(bn.outputs["Normal"], b.inputs["Normal"])
    m.diffuse_color = gz_rgba
    m["gz_rgba"] = list(gz_rgba)
    if not box_proj:
        m["gz_tex"] = albedo
    _M[name] = m
    return m


def add_wave_bump(m, scale=6.0, strength=0.35, axis="X"):
    """Corrugated sheet look for wall panels (render only)."""
    nt = m.node_tree
    b = nt.nodes.get("Principled BSDF")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    w = nt.nodes.new("ShaderNodeTexWave")
    w.wave_type = "BANDS"
    w.bands_direction = axis
    w.inputs["Scale"].default_value = scale
    w.inputs["Distortion"].default_value = 0.0
    bn = nt.nodes.new("ShaderNodeBump")
    bn.inputs["Strength"].default_value = strength
    nt.links.new(tc.outputs["Object"], w.inputs["Vector"])
    nt.links.new(w.outputs["Fac"], bn.inputs["Height"])
    nt.links.new(bn.outputs["Normal"], b.inputs["Normal"])


def add_noise_variation(m, scale=3.0, amount=0.15, rough_amt=0.15):
    """Multiply base colour by low-frequency noise: grime / paint variation (render only)."""
    nt = m.node_tree
    b = nt.nodes.get("Principled BSDF")
    base = tuple(b.inputs["Base Color"].default_value)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = scale
    nz.inputs["Detail"].default_value = 6
    ramp = nt.nodes.new("ShaderNodeMapRange")
    ramp.inputs["To Min"].default_value = 1 - amount
    ramp.inputs["To Max"].default_value = 1 + amount * 0.3
    mix = nt.nodes.new("ShaderNodeMix")
    mix.data_type = "RGBA"
    mix.blend_type = "MULTIPLY"
    mix.inputs["Factor"].default_value = 1.0
    ca = [i for i in mix.inputs if i.type == "RGBA"]
    ca[0].default_value = base
    nt.links.new(tc.outputs["Object"], nz.inputs["Vector"])
    nt.links.new(nz.outputs["Fac"], ramp.inputs["Value"])
    comb = nt.nodes.new("ShaderNodeCombineColor")
    for i in range(3):
        nt.links.new(ramp.outputs["Result"], comb.inputs[i])
    nt.links.new(comb.outputs["Color"], ca[1])
    nt.links.new([o for o in mix.outputs if o.type == "RGBA"][0], b.inputs["Base Color"])
    r0 = b.inputs["Roughness"].default_value
    rr = nt.nodes.new("ShaderNodeMapRange")
    rr.inputs["To Min"].default_value = max(0.02, r0 - rough_amt)
    rr.inputs["To Max"].default_value = min(1.0, r0 + rough_amt)
    nt.links.new(nz.outputs["Fac"], rr.inputs["Value"])
    nt.links.new(rr.outputs["Result"], b.inputs["Roughness"])


# ----------------------------------------------------------------------------- mesh builder
class MB:
    """Batches primitives into one mesh. uv='world' projects each face on its dominant
    plane (metres / uv_scale), uv='unit' maps every face to 0..1."""

    def __init__(self):
        self.v, self.f, self.fm, self.fuv = [], [], [], []
        self.mats = []

    def mi(self, m):
        if m not in self.mats:
            self.mats.append(m)
        return self.mats.index(m)

    def _face(self, idx, m, uv, uvs):
        self.f.append(idx)
        self.fm.append(self.mi(m))
        if uv == "unit":
            self.fuv.append([(0, 0), (1, 0), (1, 1), (0, 1)][: len(idx)] if uvs is None else uvs)
        else:
            pts = [self.v[i] for i in idx]
            a, b_, c = Vector(pts[0]), Vector(pts[1]), Vector(pts[2])
            n = (b_ - a).cross(c - a)
            ax = max(range(3), key=lambda k: abs(n[k]))
            u, w = [(1, 2), (0, 2), (0, 1)][ax]
            s = uv if isinstance(uv, (int, float)) else 1.0
            self.fuv.append([(p[u] / s, p[w] / s) for p in pts])

    def add_box(self, c, s, m, rz=0.0, M=None, uv=1.0):
        hx, hy, hz = s[0] / 2, s[1] / 2, s[2] / 2
        corners = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
                   (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
        T = M if M is not None else (Matrix.Translation(c) @ Matrix.Rotation(rz, 4, "Z"))
        o = len(self.v)
        self.v += [tuple(T @ Vector(p)) for p in corners]
        for q in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
            self._face([o + i for i in q], m, uv, None)

    def add_beam(self, p1, p2, w, h, m, up=(0, 0, 1)):
        p1, p2 = Vector(p1), Vector(p2)
        d = p2 - p1
        L = d.length
        if L < 1e-6:
            return
        x = d.normalized()
        upv = Vector(up)
        if abs(x.dot(upv)) > 0.99:
            upv = Vector((1, 0, 0)) if abs(x.x) < 0.9 else Vector((0, 1, 0))
        y = upv.cross(x).normalized()
        z = x.cross(y)
        R = Matrix((x, y, z)).transposed().to_4x4()
        self.add_box(None, (L, w, h), m, M=Matrix.Translation((p1 + p2) / 2) @ R)

    def add_cyl(self, c, r, h, m, axis="Z", seg=16, M=None, caps=True, r2=None):
        r2 = r if r2 is None else r2
        rot = {"Z": Matrix.Identity(4), "X": Matrix.Rotation(math.pi / 2, 4, "Y"),
               "Y": Matrix.Rotation(-math.pi / 2, 4, "X")}[axis]
        T = M if M is not None else Matrix.Translation(c) @ rot
        o = len(self.v)
        for k in range(seg):
            a = 2 * math.pi * k / seg
            self.v.append(tuple(T @ Vector((r * math.cos(a), r * math.sin(a), -h / 2))))
            self.v.append(tuple(T @ Vector((r2 * math.cos(a), r2 * math.sin(a), h / 2))))
        for k in range(seg):
            i0, i1 = o + 2 * k, o + 2 * ((k + 1) % seg)
            self.f.append([i0, i1, i1 + 1, i0 + 1]); self.fm.append(self.mi(m))
            self.fuv.append([(k / seg, 0), ((k + 1) / seg, 0), ((k + 1) / seg, 1), (k / seg, 1)])
        if caps:
            for top in (0, 1):
                idx = [o + 2 * k + top for k in range(seg)]
                if not top:
                    idx = idx[::-1]
                self.f.append(idx); self.fm.append(self.mi(m))
                self.fuv.append([(0.5 + 0.5 * math.cos(2 * math.pi * k / seg),
                                  0.5 + 0.5 * math.sin(2 * math.pi * k / seg)) for k in range(seg)])

    def add_quad(self, pts, m, uvs=None, uv="unit"):
        o = len(self.v)
        self.v += [tuple(p) for p in pts]
        self._face(list(range(o, o + len(pts))), m, uv if uvs is None else "unit", uvs)

    def extend(self, other, M):
        """Append another builder's geometry transformed by M (instancing into a batch)."""
        o = len(self.v)
        self.v += [tuple(M @ Vector(p)) for p in other.v]
        for face, fm, fuv in zip(other.f, other.fm, other.fuv):
            self.f.append([o + i for i in face])
            self.fm.append(self.mi(other.mats[fm]))
            self.fuv.append(fuv)

    def build(self, name, collection, smooth=False):
        me = bpy.data.meshes.new(name)
        me.from_pydata(self.v, [], self.f)
        for m in self.mats:
            me.materials.append(m)
        me.polygons.foreach_set("material_index", self.fm)
        uvl = me.uv_layers.new(name="UVMap")
        flat = [c for face in self.fuv for uv in face for c in uv]
        uvl.data.foreach_set("uv", flat)
        if smooth:
            me.shade_smooth()
        me.validate()
        me.update()
        ob = bpy.data.objects.new(name, me)
        collection.objects.link(ob)
        return ob


# ----------------------------------------------------------------------------- objects
def empty(name, collection, M=None, parent=None, size=0.1):
    e = bpy.data.objects.new(name, None)
    e.empty_display_size = size
    collection.objects.link(e)
    if parent:
        e.parent = parent
    if M is not None:
        e.matrix_basis = M
    return e


def text_mesh(body, size, loc, rot, m, collection, extrude=0.002, align="CENTER", name=None):
    cu = bpy.data.curves.new(name or ("txt_" + body[:12]), "FONT")
    cu.body = body
    cu.size = size
    cu.extrude = extrude
    cu.align_x = align
    cu.align_y = "CENTER"
    tmp = bpy.data.objects.new("tmp_txt", cu)
    collection.objects.link(tmp)
    tmp.location, tmp.rotation_euler = loc, rot
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(tmp.evaluated_get(dg))
    ob = bpy.data.objects.new(name or ("text_" + body[:16]), me)
    collection.objects.link(ob)
    ob.matrix_world = tmp.matrix_world.copy()
    bpy.data.objects.remove(tmp, do_unlink=True)
    bpy.data.curves.remove(cu)
    me.materials.clear()
    me.materials.append(m)
    return ob


def tag(ob, group, collide=False):
    ob["gz_group"] = group
    ob["gz_collide"] = collide
    return ob


def T(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return Matrix.Translation(xyz) @ Euler(rpy, "XYZ").to_matrix().to_4x4()
