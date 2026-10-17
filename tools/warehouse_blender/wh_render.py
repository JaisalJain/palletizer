"""Lighting, cameras and render setup for the demo (Blender only, nothing here goes to Gazebo).

Renders run in a separate background Blender process (render_async) so the UI stays live.
"""
import bpy, math, os, subprocess, sys, json
from mathutils import Vector
import wh_lib as L

SUN_EL, SUN_AZ = math.radians(52), math.radians(-12)      # from +x (east), slightly south


def sun_dir():
    return Vector((math.cos(SUN_EL) * math.cos(SUN_AZ), math.cos(SUN_EL) * math.sin(SUN_AZ), math.sin(SUN_EL)))


def world():
    sc = bpy.context.scene
    w = sc.world or bpy.data.worlds.new("World")
    sc.world = w
    w.use_nodes = True
    nt = w.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    sky = nt.nodes.new("ShaderNodeTexSky")
    for t in ("MULTIPLE_SCATTERING", "NISHITA", "SINGLE_SCATTERING", "HOSEK_WILKIE"):
        try:
            sky.sky_type = t
            break
        except Exception:
            continue
    for attr, val in (("sun_disc", False), ("sun_elevation", SUN_EL), ("sun_rotation", SUN_AZ + math.pi / 2),
                      ("altitude", 200.0), ("air_density", 1.0), ("dust_density", 1.5)):
        try:
            setattr(sky, attr, val)
        except Exception:
            pass
    bg.inputs["Strength"].default_value = 0.22
    nt.links.new(sky.outputs["Color"], bg.inputs["Color"])
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])


def lights(lamps, C):
    for o in list(C.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    sd = bpy.data.lights.get("LT_sun") or bpy.data.lights.new("LT_sun", "SUN")
    sd.energy, sd.angle, sd.color = 4.5, math.radians(0.6), (1.0, 0.96, 0.9)
    sun = bpy.data.objects.new("LT_sun", sd)
    C.objects.link(sun)
    sun.rotation_euler = sun_dir().to_track_quat("Z", "Y").to_euler()
    ad = bpy.data.lights.get("LT_highbay") or bpy.data.lights.new("LT_highbay", "AREA")
    ad.shape, ad.size, ad.energy, ad.color = "DISK", 0.4, 170.0, (1.0, 0.92, 0.82)
    try:
        ad.spread = math.radians(130)
    except Exception:
        pass
    for i, (x, y, z) in enumerate(lamps):
        o = bpy.data.objects.new(f"LT_highbay_{i:02d}", ad)
        C.objects.link(o)
        o.location = (x, y, z - 0.01)
    # trailer interior (dock D3) so the open door does not read as a black hole
    d = bpy.data.lights.new("LT_trailer", "AREA")
    d.shape, d.size, d.size_y, d.energy, d.color = "RECTANGLE", 2.0, 10.0, 220.0, (1.0, 0.95, 0.88)
    o = bpy.data.objects.new("LT_trailer", d)
    C.objects.link(o)
    o.location = (0.0, 21.0, 2.7)
    # office interior glow + open east door fill
    for (loc, rot, e, s) in (((-19.8, 8.2, 1.7), (math.pi / 2, 0, 0), 180.0, (7.0, 1.2)),
                             ((-19.8, 8.2, 4.9), (math.pi / 2, 0, 0), 180.0, (7.0, 1.2))):
        d = bpy.data.lights.new("LT_office", "AREA")
        d.shape, d.size, d.size_y, d.energy, d.color = "RECTANGLE", s[0], s[1], e, (1.0, 0.82, 0.6)
        o = bpy.data.objects.new("LT_office", d)
        C.objects.link(o)
        o.location, o.rotation_euler = loc, (-math.pi / 2, 0, 0)


def haze(C, density=0.0025):
    """Interior volume for light shafts through the skylights / east door."""
    me = bpy.data.meshes.get("FX_haze") or bpy.data.meshes.new("FX_haze")
    if not me.vertices:
        import bmesh
        bm = bmesh.new()
        bmesh.ops.create_cube(bm, size=1.0)
        bm.to_mesh(me)
        bm.free()
    o = bpy.data.objects.get("FX_haze") or bpy.data.objects.new("FX_haze", me)
    if o.name not in C.objects:
        C.objects.link(o)
    o.location, o.scale = (0, -1, 5.0), (47.8, 29.8, 9.9)
    m = bpy.data.materials.get("FX_haze") or bpy.data.materials.new("FX_haze")
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    pv = nt.nodes.new("ShaderNodeVolumePrincipled")
    pv.inputs["Density"].default_value = density
    pv.inputs["Anisotropy"].default_value = 0.55
    pv.inputs["Color"].default_value = (0.9, 0.92, 0.95, 1)
    nt.links.new(pv.outputs["Volume"], out.inputs["Volume"])
    me.materials.clear()
    me.materials.append(m)
    o.display_type = "BOUNDS"
    o.hide_select = True
    return o


CAMS = {
    #  name          location               target               lens  dof_target / fstop
    "hero":     ((3.3, 2.95, 3.05),      (-0.25, -0.6, 1.1),   24, ("CELL_held_box", 8.0)),
    "robot":    ((2.55, -1.95, 1.05),    (-0.12, -0.38, 1.3),  50, ("CELL_held_box", 2.8)),
    "infeed":   ((-8.2, 2.35, 1.75),     (0.2, 0.2, 1.15),     28, ("CELL_up_box_0", 5.6)),
    "aerial":   ((20.0, -24.0, 42.0),    (-1.0, -0.5, 0.0),    28, None),
    "docks":    ((-1.5, 3.2, 2.6),       (2.0, 14.0, 1.9),     22, None),
    "wide":     ((21.5, -12.5, 6.5),     (0.0, 0.0, 1.2),      24, None),
}
# objects hidden per camera (cutaway for the aerial view)
HIDE = {"aerial": ["GZ_roof", "GZ_services", "FX_haze"]}


def cameras(C):
    for n, (loc, tgt, lens, dof) in CAMS.items():
        name = f"CAM_{n}"
        cd = bpy.data.cameras.get(name) or bpy.data.cameras.new(name)
        cd.lens, cd.sensor_width, cd.clip_start, cd.clip_end = lens, 36, 0.05, 400
        o = bpy.data.objects.get(name) or bpy.data.objects.new(name, cd)
        if o.name not in C.objects:
            C.objects.link(o)
        o.location = loc
        o.rotation_euler = (Vector(tgt) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        cd.dof.use_dof = dof is not None
        if dof:
            tgt_ob = bpy.data.objects.get(dof[0])
            if tgt_ob:
                cd.dof.focus_object = tgt_ob
            cd.dof.aperture_fstop = dof[1]
    bpy.context.scene.camera = bpy.data.objects["CAM_hero"]


def settings(samples=192, res=(1920, 1080)):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "OPTIX"
    prefs.refresh_devices()
    for d in prefs.devices:
        d.use = d.type == "OPTIX"
    sc.cycles.device = "GPU"
    sc.cycles.samples = samples
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.adaptive_threshold = 0.02
    sc.cycles.use_denoising = True
    try:
        sc.cycles.denoiser = "OPTIX"
    except Exception:
        pass
    sc.cycles.max_bounces, sc.cycles.diffuse_bounces, sc.cycles.glossy_bounces = 8, 3, 3
    sc.cycles.transmission_bounces, sc.cycles.volume_bounces = 6, 1
    sc.cycles.sample_clamp_indirect = 6.0
    sc.cycles.caustics_reflective = sc.cycles.caustics_refractive = False
    sc.cycles.volume_step_rate = 4.0
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    try:
        sc.render.image_settings.media_type = "IMAGE"     # Blender 5: set before file_format
    except Exception:
        pass
    sc.render.image_settings.file_format = "PNG"
    try:
        sc.view_settings.view_transform = "AgX"
        sc.view_settings.look = "AgX - Medium High Contrast"
    except Exception:
        pass
    sc.view_settings.exposure = -0.55
    sc.unit_settings.system = "METRIC"


def render_async(blend, cams, outdir, samples=192, res=(1920, 1080), haze=True, tag=""):
    """Render cameras in a background Blender process; returns the Popen handle + log path."""
    os.makedirs(outdir, exist_ok=True)
    job = os.path.join(os.path.dirname(blend), "_render_job.py")
    with open(job, "w") as f:
        f.write(f"""
import bpy, sys, os
sys.path.insert(0, {os.path.dirname(__file__)!r})
import wh_render as R
R.settings({samples}, {tuple(res)})
bpy.data.objects['FX_haze'].hide_render = {not haze}
for c in {list(cams)!r}:
    sc = bpy.context.scene
    sc.camera = bpy.data.objects['CAM_' + c]
    hid = [bpy.data.objects[n] for n in R.HIDE.get(c, []) if n in bpy.data.objects]
    old = [o.hide_render for o in hid]
    for o in hid: o.hide_render = True
    sc.render.filepath = os.path.join({outdir!r}, c + {tag!r} + '.png')
    bpy.ops.render.render(write_still=True)
    for o, h in zip(hid, old): o.hide_render = h
    print('RENDERED', c, flush=True)
""")
    log = os.path.join(outdir, "_render_log.txt")
    lf = open(log, "w")
    p = subprocess.Popen([bpy.app.binary_path, "-b", blend, "-P", job], stdout=lf, stderr=subprocess.STDOUT,
                         creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    return p.pid, log


def eevee_video(out_path, res=(1920, 1080)):
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_EEVEE"
    ee = sc.eevee
    for attr, val in (("taa_render_samples", 32), ("use_raytracing", True), ("use_shadows", True),
                      ("volumetric_tile_size", "4"), ("use_volumetric_shadows", True),
                      ("volumetric_samples", 32), ("fast_gi_method", "GLOBAL_ILLUMINATION"),
                      ("use_fast_gi", True), ("shadow_ray_count", 2), ("shadow_step_count", 8)):
        try:
            setattr(ee, attr, val)
        except Exception:
            pass
    try:
        ee.ray_tracing_options.resolution_scale = "1"
    except Exception:
        pass
    # EEVEE's shadow pool overflows with dozens of shadowed area lights ("Shadow buffer full":
    # flicker + very slow frames). In the video only the sun casts shadows; the high-bays,
    # office and trailer fills are shadowless (Cycles stills keep them all).
    try:
        ee.shadow_pool_size = "1024"
    except Exception:
        pass
    for o in bpy.data.objects:
        if o.type == "LIGHT" and o.data.type != "SUN":
            o.data.use_shadow = False
    sc.render.use_motion_blur = True
    sc.render.motion_blur_shutter = 0.5
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.resolution_percentage = 100
    im = sc.render.image_settings
    try:
        im.media_type = "VIDEO"
    except Exception:
        pass
    im.file_format = "FFMPEG"
    ff = sc.render.ffmpeg
    ff.format, ff.codec = "MPEG4", "H264"
    ff.constant_rate_factor, ff.ffmpeg_preset = "HIGH", "GOOD"
    ff.audio_codec = "NONE"
    sc.render.filepath = out_path
    if "CAM_anim" in bpy.data.objects:
        sc.camera = bpy.data.objects["CAM_anim"]
