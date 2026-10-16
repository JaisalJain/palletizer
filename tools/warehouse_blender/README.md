# Warehouse world (Blender → Gazebo)

Procedural Blender 5.x build of the warehouse around the palletizing cell. It produces:

- `src/palletizer_gazebo/models/palletizer_warehouse`, the Gazebo model used by `world:=warehouse`
- `warehouse_demo.blend` / `warehouse_anim2.blend`, a render scene with the cell replica, the real
  UR10e meshes, Cycles stills and an EEVEE pick-and-place video

Everything is placed from `cell_config.yaml`, the same single source of truth the robot uses.

## Layout

World frame = cell frame (robot base axis at the origin, +x = conveyor travel).

| area | content |
|---|---|
| building | 48 × 30 m, 8.6 m clear, lattice trusses, 3 skylight strips, sprinklers, duct, 52 high-bays |
| south | 4 rows of selective racking (13 bays × 4 levels), ~500 mixed loads, reach-aisle forklift |
| centre | the fenced cell, light curtains at the pallet openings, andon, control cabinet + HMI; upstream infeed line with case sealer; empty-pallet dispenser; stretch wrapper and outbound lane |
| north | 5 dock doors (2 open, trailer backed in), levelers, staging lanes, forklift, 2-storey office |

## Keep-outs (checked by `offline/wh_verify.py`)

Nothing in the model is inside:

- the robot reach cylinder (r = 1.85 m)
- the pallet-lane swept volume
- the conveyor envelope
- the photo-eye beams
- either camera's perception ROI

Floor-level paint is the only exception. The roof and walls have `cast_shadows=false`, and the
world's lights are unchanged, so the cameras see the same lighting. The floor texture's mean
colour equals the plain world's ground colour.

## Demo animation: emulated stack, collision-checked

The robot motion is not keyframed by hand. `offline/sim_demo.py` emulates the real software:

- the `palletize.xml` cycle
- the `motion_server` pick and place sequences, with `solve` / `route_pan` / `hub_z` and Pilz-style
  PTP and LIN profiles at the configured scalings and the UR10e joint limits
- the conveyor zone logic, with resume only after the pick
- detection timing, with no detection while the arm is over the ROI
- the pallet exchanger
- the real 400×300×300 pattern: 7/7 × 3 layers, with the slot in front of the pedestal dropped

It also checks every motion with FCL against the UR10e collision meshes. `offline/validate2.py`
re-checks every rendered frame independently. The current run has:

- 0 contacts deeper than 2 mm
- joints within 120/180 deg/s
- the tool within 0.02° of vertical, so the carried box stays level
- carried-box rotation between 78° and 109°

One deliberate deviation: for the place yaw, the demo minimises the carried box's rotation in the
world. `motion_server.place_poses` only minimises |Δq6|, and with a large base swing that can spin
the box by up to about 270°.

## Rebuild

```bash
# 1. assets (any Python with numpy/pillow/pyyaml/trimesh/pycollada)
python offline/config_to_json.py ../../src/palletizer_bringup/config/cell_config.yaml $ROBOT_WORLD/assets/cell_config.json
python offline/make_textures.py $ROBOT_WORLD/assets/textures
python offline/convert_ur_meshes.py $(ros2 pkg prefix ur_description)/share/ur_description/meshes/ur10e/visual $ROBOT_WORLD/assets/ur10e
UR_COLLISION=$(ros2 pkg prefix ur_description)/share/ur_description/meshes/ur10e/collision \
  python offline/sim_demo.py $ROBOT_WORLD/assets/cell_config.json $ROBOT_WORLD/assets/anim2.json
python offline/validate2.py $ROBOT_WORLD/assets/anim2.json $ROBOT_WORLD/assets/cell_config.json

# 2. Blender (GUI Python console, or headless)
ROBOT_WORLD=... blender -b -P wh_build_all.py -- --save
#    then in Blender:  import wh_export; wh_export.export("<repo>/src/palletizer_gazebo/models")

# 3. README hero video: 9 shots cut from the emulated run -> < 10 MB mp4 + master + poster
blender -b $ROBOT_WORLD/warehouse_demo.blend --python-expr "import sys; sys.path.insert(0,'.'); import wh_anim2, bpy; wh_anim2.apply(); bpy.ops.wm.save_as_mainfile(filepath='$ROBOT_WORLD/warehouse_anim2.blend')"
blender -b $ROBOT_WORLD/warehouse_anim2.blend -P wh_hero2.py
blender -b $ROBOT_WORLD/warehouse_anim2.blend -P wh_stills2.py
python offline/make_hero_video2.py $ROBOT_WORLD/renders/hero2 $ROBOT_WORLD/renders/final2

# 4. verify
python offline/wh_verify.py ../../src/palletizer_gazebo/models/palletizer_warehouse ../../src/palletizer_bringup/config/cell_config.yaml
```

`$ROBOT_WORLD` is the working folder (`assets/`, `renders/`, and the `.blend` files are written there).
