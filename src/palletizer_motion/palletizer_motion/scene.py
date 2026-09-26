"""Static collision objects of the cell, all from cell_config.yaml (simple boxes).

No pallet here: the pallet at the station is measured (pallet_perception) and added by the
motion server's /motion/scene/set_pallet."""

from palletizer_motion.moveit_client import box_object, make_pose


def static_objects(cell, frame='world'):
    cv = cell['conveyor']
    cam = cell['camera']
    objs = []

    def add(name, size, center):
        objs.append(box_object(name, frame, size, make_pose(center)))

    add('floor', (8.0, 8.0, 0.02), (0.0, 0.0, -0.011))
    length = cv['x_end'] - cv['x_start']
    # conveyor body incl. side rails, up to the belt top
    add('conveyor', (length, cv['belt_width'] + 0.1, cv['belt_top_z']),
        ((cv['x_start'] + cv['x_end']) / 2, cv['y'], cv['belt_top_z'] / 2))
    add('conveyor_end_stop', (0.04, cv['belt_width'] + 0.1, 0.18),
        (cv['x_end'] + 0.02, cv['y'], cv['belt_top_z'] + 0.04))
    # photo-eye emitters / reflectors on the rails
    eye = cv['photo_eye']
    for z in cv['zones']:
        off = eye['offset']
        for side, dy in (('e', -cv['belt_width'] / 2 - off), ('r', cv['belt_width'] / 2 + off)):
            add(f'eye_{z["name"]}_{side}', (0.04, 0.04, 0.06),
                (z['eye_x'], cv['y'] + dy, cv['belt_top_z'] + eye['height']))
    mx, my = cam['mast_xy']
    msx, msy, msz = cam['mast_size']
    add('camera_mast', (msx + 0.02, msy + 0.02, msz), (mx, my, msz / 2))
    arm_len = abs(my - cam['xyz'][1]) + 0.1
    add('camera_arm', (0.08, arm_len, 0.08), (mx, (my + cam['xyz'][1]) / 2, cam['xyz'][2] + 0.06))
    add('camera_head', (0.08, 0.14, 0.08), tuple(cam['xyz']))
    pc = cell['pallet_camera']
    mx, my = pc['mast_xy']
    msx, msy, msz = pc['mast_size']
    add('pallet_camera_mast', (msx + 0.02, msy + 0.02, msz), (mx, my, msz / 2))
    arm_len = abs(my - pc['xyz'][1]) + 0.1
    add('pallet_camera_arm', (0.08, arm_len, 0.08),
        (mx, (my + pc['xyz'][1]) / 2, pc['xyz'][2] + 0.06))
    add('pallet_camera_head', (0.14, 0.08, 0.08), tuple(pc['xyz']))
    return objs
