"""Entry point. In Blender's Python console / Text editor:
    exec(open(r"<robot_world>/blender/wh_build_all.py").read()); build_all()
or headless:  blender -b -P wh_build_all.py -- --save
"""
import bpy, sys, os, importlib
HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.path.join(
    os.environ["ROBOT_WORLD"], "blender")
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import wh_lib, wh_env, wh_cell, wh_render
for m in (wh_lib, wh_env, wh_cell, wh_render):
    importlib.reload(m)


def reset():
    for n in ("WAREHOUSE", "CELL_DEMO", "RENDER"):
        wh_lib.wipe_collection(n)
    for o in list(bpy.data.objects):
        if o.name in ("Cube", "Camera", "Light"):
            bpy.data.objects.remove(o, do_unlink=True)
    wh_lib.purge()


def build_env():
    cfg = wh_lib.cfg()
    M, lamps = wh_env.build(cfg)
    return cfg, M, lamps


def build_all():
    reset()
    cfg, M, lamps = build_env()
    cell = wh_cell.build(cfg, M)
    R = wh_lib.coll("RENDER")
    wh_render.world()
    wh_render.lights(lamps, wh_lib.coll("RENDER_lights", R))
    wh_render.haze(R)
    wh_render.cameras(wh_lib.coll("RENDER_cameras", R))
    wh_render.settings()
    return cfg, M, cell


if __name__ == "__main__" and "--save" in sys.argv:
    build_all()
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(os.path.dirname(HERE), "warehouse_demo.blend"))
