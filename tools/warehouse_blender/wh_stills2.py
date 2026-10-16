"""Cycles stills from the collision-checked run (warehouse_anim2.blend).
    blender -b warehouse_anim2.blend -P wh_stills2.py      -> renders/final2/*.png"""
import bpy, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import wh_render as R
OUT = os.path.join(os.path.dirname(HERE), "renders", "final2")
os.makedirs(OUT, exist_ok=True)
# camera, frame, DOF focus object (None = keep)
STILLS = [("hero", 560, "SIM_belt_01"), ("robot", 575, "SIM_belt_01"), ("wide", 560, None),
          ("aerial", 560, None), ("infeed", 230, None), ("docks", 560, None)]
sc = bpy.context.scene
R.settings(256, (1920, 1080))
bpy.data.objects["FX_haze"].hide_render = False
for cam, f, focus in STILLS:
    sc.frame_set(f)
    c = bpy.data.objects["CAM_" + cam]
    if focus and focus in bpy.data.objects:
        c.data.dof.use_dof, c.data.dof.focus_object = True, bpy.data.objects[focus]
    sc.camera = c
    hid = [bpy.data.objects[n] for n in R.HIDE.get(cam, []) if n in bpy.data.objects]
    for o in hid: o.hide_render = True
    sc.render.filepath = os.path.join(OUT, cam + ".png")
    bpy.ops.render.render(write_still=True)
    for o in hid: o.hide_render = False
    print("RENDERED", cam, flush=True)
