"""README hero video v2: nine shots cut from the continuous stack emulation (anim2.json).
Each shot is a real time window of the same run; cuts only skip time, nothing is retimed.
    blender -b warehouse_anim2.blend -P wh_hero2.py                      -> renders/hero2/shot_XX.mp4
    WH_HERO_PREVIEW=1 blender -b warehouse_anim2.blend -P wh_hero2.py   -> quick framing stills
Captions / title / fades / final encode: offline/make_hero_video2.py"""
import bpy, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import wh_render as R

FPS = 24
# name, source window [s], camera (start, end), look-at (start, end), lens, f-stop, hidden objects
SHOTS = [
    ("aerial", (0.0, 3.0),   ((17, -23, 31), (9, -12.5, 17.5)),        ((0, -0.4, 1), (0, -0.3, 1)),           30, None,
     ["GZ_roof", "GZ_services", "FX_haze"]),
    ("infeed", (7.4, 10.9),  ((-8.2, 1.95, 1.6), (-7.6, 1.9, 1.6)),    ((-5.5, 0.8, 0.95), (-4.8, 0.8, 1.0)),   30, 4.0, []),
    ("pick",   (15.8, 20.8), ((1.30, 1.78, 1.32), (1.05, 1.88, 1.26)), ((0.03, 0.8, 1.02), (0.03, 0.8, 1.12)),  40, 2.8, []),
    ("place",  (20.8, 25.4), ((2.0, -1.95, 1.45), (1.75, -2.1, 1.65)), ((0.35, -0.1, 1.3), (0.15, -0.4, 1.2)),  35, 4.0, []),
    ("layer",  (33.0, 37.0), ((3.4, -2.7, 2.9), (3.6, -1.1, 2.8)),     ((0.2, -0.5, 1.0), (0.2, -0.5, 1.0)),    30, 5.6, []),
    ("out",    (45.0, 49.0), ((5.35, -1.15, 1.05), (5.3, -1.2, 1.1)),  ((1.0, -0.6, 0.75), (2.4, -0.6, 0.8)),   32, 5.6, []),
    ("in",     (60.0, 63.5), ((-0.6, -2.05, 1.35), (-0.8, -2.0, 1.3)), ((-2.9, -0.6, 0.4), (-1.7, -0.6, 0.4)),  28, 4.0, []),
    ("first",  (77.8, 82.0), ((1.5, -2.15, 1.15), (1.3, -2.2, 1.05)),  ((-0.2, -0.5, 0.7), (-0.44, -0.8, 0.5)), 35, 4.0, []),
    ("crane",  (82.0, 85.5), ((7.6, 6.9, 5.0), (11.5, 9.3, 7.4)),      ((0, -0.5, 0.9), (0, -0.8, 0.9)),        26, None, []),
]


def frames(win):
    return int(round(win[0] * FPS)) + 1, int(round(win[1] * FPS))


def setup_camera(i, shot):
    name, win, cam_p, look_p, lens, fstop, _ = shot
    f0, f1 = frames(win)
    C = bpy.data.collections["RENDER_cameras"]
    cn = f"CAM_h2_{i:02d}_{name}"
    tgt = bpy.data.objects.get(cn + "_t") or bpy.data.objects.new(cn + "_t", None)
    cd = bpy.data.cameras.get(cn) or bpy.data.cameras.new(cn)
    cam = bpy.data.objects.get(cn) or bpy.data.objects.new(cn, cd)
    for o in (tgt, cam):
        if o.name not in C.objects:
            C.objects.link(o)
        o.animation_data_clear()
    cd.lens, cd.sensor_width, cd.clip_start, cd.clip_end = lens, 36, 0.03, 400
    cd.dof.use_dof = fstop is not None
    if fstop:
        cd.dof.focus_object, cd.dof.aperture_fstop = tgt, fstop
    cam.constraints.clear()
    tc = cam.constraints.new("TRACK_TO")
    tc.target, tc.track_axis, tc.up_axis = tgt, "TRACK_NEGATIVE_Z", "UP_Y"
    prefs = bpy.context.preferences.edit
    old = prefs.keyframe_new_interpolation_type
    prefs.keyframe_new_interpolation_type = "LINEAR"
    for f, lp, tp in ((f0, cam_p[0], look_p[0]), (f1, cam_p[1], look_p[1])):
        cam.location, tgt.location = lp, tp
        cam.keyframe_insert("location", frame=f)
        tgt.keyframe_insert("location", frame=f)
    prefs.keyframe_new_interpolation_type = old
    return cam, f0, f1


def hide(names, state):
    for n in names:
        o = bpy.data.objects.get(n)
        if o:
            o.hide_render = state


if __name__ == "__main__" and bpy.app.background:
    out = os.path.join(os.path.dirname(HERE), "renders", "hero2")
    os.makedirs(out, exist_ok=True)
    preview = bool(os.environ.get("WH_HERO_PREVIEW"))
    only = os.environ.get("WH_SHOTS")
    R.eevee_video(os.path.join(out, "tmp.mp4"))
    sc = bpy.context.scene
    for i, shot in enumerate(SHOTS):
        if only and shot[0] not in only.split(","):
            continue
        cam, f0, f1 = setup_camera(i, shot)
        sc.camera = cam
        hide(shot[6], True)
        if preview:
            sc.render.use_motion_blur = False
            sc.eevee.taa_render_samples = 16
            sc.render.resolution_percentage = 50
            try:
                sc.render.image_settings.media_type = "IMAGE"
            except Exception:
                pass
            sc.render.image_settings.file_format = "JPEG"
            for f in (f0, (f0 + f1) // 2, f1):
                sc.frame_set(f)
                sc.render.filepath = os.path.join(out, "preview", f"{i:02d}_{shot[0]}_{f:04d}.jpg")
                bpy.ops.render.render(write_still=True)
        else:
            sc.frame_start, sc.frame_end = f0, f1
            sc.render.filepath = os.path.join(out, f"shot_{i:02d}_{shot[0]}.mp4")
            sc.render.ffmpeg.constant_rate_factor = "HIGH"
            bpy.ops.render.render(animation=True)
        hide(shot[6], False)
        print("RENDERED", shot[0], flush=True)
