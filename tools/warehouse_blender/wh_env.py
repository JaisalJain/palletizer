"""Warehouse environment around the palletizing cell (everything here is exported to Gazebo).

Layout (world frame = cell frame, robot base axis at the origin, +x = conveyor travel):
  building interior x[-24, 24] y[-16, 14], clear height 8.6 m, flat roof at 10 m
  south  : 3 rows of selective pallet racking (7 m, 4 levels)
  centre : the cell, fenced; upstream infeed line (-x), empty-pallet dispenser (-x),
           stretch wrapper + outbound lane (+x)
  north  : 5 dock doors with levelers, staging lanes, forklift traffic, 2-storey office (NW)
Keep-outs honoured (checked in wh_verify.py): robot reach + fence, both camera frusta,
pallet lane entry/exit (x = +-2.9 +- 0.6), photo-eye beams.
"""
import math, random
from mathutils import Vector, Matrix
import wh_lib as L
from wh_lib import MB, mat, tag

BX0, BX1, BY0, BY1 = -24.0, 24.0, -16.0, 14.0
WT = 0.25                 # wall thickness (outside the interior box)
H_TRUSS, H_ROOF = 8.6, 10.0
FENCE = dict(x0=-3.8, x1=3.8, y0=-2.3, y1=2.1, h=2.2, gap=0.15)
DOORS_N = [-10.0, -5.0, 0.0, 5.0, 10.0]        # dock door centres on the north wall
OPEN_DOCKS = {0.0, 5.0}
DOCK_W, DOCK_H = 3.0, 3.2
EAST_DOOR = (-7.0, -2.0, 5.0)                   # y0, y1, height (open roller door)
SKY_Y = [-9.0, 0.0, 9.0]                        # skylight strips (along x)
SKY_W = 1.6
FRAMES_X = [-18.0, -12.0, -6.0, 0.0, 6.0, 12.0, 18.0]


def _tint(m, k):
    """Render-only albedo multiplier (keeps the texture exported to Gazebo unchanged)."""
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    lk = b.inputs["Base Color"].links[0]
    src = lk.from_socket
    mix = nt.nodes.new("ShaderNodeMix")
    mix.data_type, mix.blend_type = "RGBA", "MULTIPLY"
    mix.inputs["Factor"].default_value = 1.0
    ca = [i for i in mix.inputs if i.type == "RGBA"]
    nt.links.new(src, ca[0])
    ca[1].default_value = (k, k, k * 0.98, 1)
    nt.links.new([o for o in mix.outputs if o.type == "RGBA"][0], b.inputs["Base Color"])


def materials():
    M = {}
    M["concrete"] = L.textured("wh_concrete", "concrete_albedo.png", "concrete_rough.png",
                               scale=0.25, gz_rgba=(0.55, 0.55, 0.52, 1))
    _tint(M["concrete"], 0.72)
    M["joint"] = mat("wh_saw_joint", (0.18, 0.18, 0.17), 0.9)
    M["plinth"] = mat("wh_plinth", (0.46, 0.46, 0.44), 0.85)
    L.add_noise_variation(M["plinth"], 1.5, 0.2)
    M["wall"] = mat("wh_wall_panel", (0.70, 0.72, 0.74), 0.45, 0.3)
    L.add_wave_bump(M["wall"], scale=9.0, strength=0.4, axis="Z")
    M["wall_ext"] = mat("wh_wall_ext", (0.62, 0.66, 0.70), 0.5, 0.4)
    L.add_wave_bump(M["wall_ext"], scale=9.0, strength=0.4, axis="Z")
    M["accent"] = mat("wh_accent_navy", (0.03, 0.10, 0.25), 0.4, 0.2)
    M["roof"] = mat("wh_roof_deck", (0.70, 0.72, 0.74), 0.5, 0.6)
    L.add_wave_bump(M["roof"], scale=4.0, strength=0.5, axis="Y")
    M["steel"] = mat("wh_steel_frame", (0.30, 0.33, 0.36), 0.45, 0.7)
    M["galv"] = mat("wh_galvanised", (0.62, 0.64, 0.65), 0.35, 1.0)
    M["sprinkler"] = mat("wh_sprinkler_red", (0.55, 0.03, 0.02), 0.4, 0.1)
    M["sky_glass"] = mat("wh_skylight", (0.85, 0.9, 0.95), 0.25, 0, transmission=1.0,
                         gz=(0.85, 0.9, 0.95, 0.6))
    M["lamp"] = mat("wh_lamp_emit", (1.0, 0.96, 0.9), 0.3, emit=18.0)
    M["lamp_body"] = mat("wh_lamp_body", (0.12, 0.13, 0.14), 0.4, 0.8)
    M["upright"] = mat("wh_rack_upright", (0.05, 0.20, 0.48), 0.4, 0.3)
    M["beam"] = mat("wh_rack_beam", (0.92, 0.33, 0.02), 0.35, 0.2)
    M["yellow"] = mat("wh_safety_yellow", (0.95, 0.68, 0.02), 0.4, 0.1)
    L.add_noise_variation(M["yellow"], 2.0, 0.12)
    M["black"] = mat("wh_black", (0.02, 0.02, 0.02), 0.6)
    M["dark"] = mat("wh_dark_steel", (0.07, 0.08, 0.09), 0.45, 0.6)
    M["mesh"] = mat("wh_fence_mesh", (0.10, 0.10, 0.11), 0.5, 0.6)
    M["paint_y"] = mat("wh_floor_yellow", (0.95, 0.70, 0.03), 0.55)
    M["paint_w"] = mat("wh_floor_white", (0.9, 0.9, 0.88), 0.55)
    M["paint_g"] = mat("wh_floor_green", (0.05, 0.45, 0.15), 0.55)
    M["paint_b"] = mat("wh_floor_blue", (0.05, 0.2, 0.6), 0.55)
    M["hazard"] = L.textured("wh_hazard", "hazard.png", rough=0.5, scale=1.0,
                             gz_rgba=(0.5, 0.4, 0.05, 1))
    M["wood"] = mat("wh_pallet_wood", (0.66, 0.52, 0.34), 0.8)
    L.add_noise_variation(M["wood"], 8.0, 0.25)
    M["wood_dark"] = mat("wh_pallet_wood_old", (0.45, 0.36, 0.25), 0.85)
    for i, c in enumerate([(0.60, 0.43, 0.26), (0.66, 0.49, 0.31), (0.55, 0.40, 0.25),
                           (0.70, 0.56, 0.38)]):
        M[f"kraft{i}"] = mat(f"wh_kraft_{i}", c, 0.75)
        L.add_noise_variation(M[f"kraft{i}"], 5.0, 0.1)
    M["white_box"] = mat("wh_white_box", (0.85, 0.85, 0.82), 0.6)
    M["wrap"] = mat("wh_stretch_wrap", (0.95, 0.96, 0.98), 0.18, 0, alpha=0.38, coat=1.0,
                    gz=(0.86, 0.88, 0.9, 1))       # milky film over the cartons (render)
    M["crate_blue"] = mat("wh_crate_blue", (0.03, 0.22, 0.55), 0.45)
    M["crate_grey"] = mat("wh_crate_grey", (0.35, 0.37, 0.38), 0.5)
    M["door"] = mat("wh_sectional_door", (0.28, 0.36, 0.45), 0.35, 0.5)
    L.add_wave_bump(M["door"], scale=1.6, strength=0.8, axis="Z")
    M["rubber"] = mat("wh_rubber", (0.03, 0.03, 0.03), 0.8)
    M["asphalt"] = mat("wh_asphalt", (0.10, 0.10, 0.10), 0.85)
    L.add_noise_variation(M["asphalt"], 2.0, 0.3)
    M["trailer"] = mat("wh_trailer", (0.92, 0.93, 0.94), 0.35, 0.2)
    M["trailer_in"] = mat("wh_trailer_inside", (0.5, 0.52, 0.55), 0.6, 0.3)
    M["office"] = mat("wh_office_wall", (0.88, 0.88, 0.86), 0.7)
    M["window"] = mat("wh_office_window", (1.0, 0.85, 0.6), 0.1, emit=3.0,
                      gz=(0.9, 0.8, 0.6, 1))
    M["glass"] = mat("wh_glass", (0.6, 0.7, 0.75), 0.05, 0, transmission=1.0,
                     gz=(0.6, 0.7, 0.75, 0.5))
    M["forklift"] = mat("wh_forklift_orange", (0.95, 0.42, 0.03), 0.35, 0.1, coat=0.6)
    M["green_emit"] = mat("wh_led_green", (0.05, 1.0, 0.2), 0.3, emit=12.0)
    M["red_emit"] = mat("wh_led_red", (1.0, 0.05, 0.02), 0.3, emit=10.0)
    M["amber"] = mat("wh_led_amber_off", (0.5, 0.3, 0.02), 0.3)
    M["exit_emit"] = mat("wh_exit_sign", (0.1, 0.9, 0.3), 0.3, emit=6.0)
    M["cabinet"] = mat("wh_cabinet_ral7035", (0.76, 0.77, 0.75), 0.45, 0.1)
    M["sign_w"] = mat("wh_sign_white", (0.95, 0.95, 0.95), 0.5)
    M["sign_red"] = mat("wh_sign_red", (0.75, 0.03, 0.03), 0.5)
    M["hmi"] = L.textured("wh_hmi_screen", "hmi.png", rough=0.2, scale=1.0,
                          gz_rgba=(0.1, 0.2, 0.3, 1))
    try:
        b = M["hmi"].node_tree.nodes["Principled BSDF"]
        ti = [n for n in M["hmi"].node_tree.nodes if n.type == "TEX_IMAGE"][0]
        M["hmi"].node_tree.links.new(ti.outputs["Color"], b.inputs["Emission Color"])
        b.inputs["Emission Strength"].default_value = 2.5
    except Exception:
        pass
    return M


# ============================================================================ pallets & loads
def pallet_mb(M, full=True, wood="wood"):
    """EUR pallet 1.2 (x) x 0.8 (y) x 0.144, origin at deck-bottom centre."""
    p = MB()
    w = M[wood]
    ys = (-0.3275, 0.0, 0.3275)
    xs = (-0.5275, 0.0, 0.5275)
    for y, wy in zip(ys, (0.145, 0.145, 0.145)):
        p.add_box((0, y, 0.011), (1.2, wy, 0.022), w)                      # bottom boards
    for x in xs:
        for y in ys:
            p.add_box((x, y, 0.061), (0.145 if x else 0.145, 0.1 if y else 0.145, 0.078), w)
    if full:
        for x in xs:
            p.add_box((x, 0, 0.111), (0.145, 0.8, 0.022), w)               # stringer boards
        for y, wy in zip((-0.3275, -0.1625, 0.0, 0.1625, 0.3275), (0.145, 0.1, 0.145, 0.1, 0.145)):
            p.add_box((0, y, 0.133), (1.2, wy, 0.022), w)                  # top deck
    else:
        p.add_box((0, 0, 0.122), (1.2, 0.8, 0.044), w)                     # simplified deck
    return p


def load_mb(M, kind, rng, full_pallet=False):
    """A pallet load, origin at floor centre, pallet long side along x."""
    b = MB()
    b.extend(pallet_mb(M, full_pallet), Matrix.Identity(4))
    z0 = 0.144
    if kind == "cartons":
        k = M[f"kraft{rng.randrange(4)}"]
        layers = rng.choice((2, 3))
        for l in range(layers):
            for i in range(4):
                for j in range(2):
                    b.add_box((-0.45 + 0.3 * i, -0.2 + 0.4 * j, z0 + 0.15 + 0.3 * l),
                              (0.294, 0.394, 0.298), k)
        if rng.random() < 0.5:                                             # partial top layer
            for i in range(rng.randrange(1, 5)):
                b.add_box((-0.45 + 0.3 * i, -0.2, z0 + 0.15 + 0.3 * layers), (0.294, 0.394, 0.298), k)
    elif kind == "big":
        k = M[f"kraft{rng.randrange(4)}"]
        for l in range(rng.choice((2, 3))):
            for i in range(2):
                for j in range(2):
                    b.add_box((-0.3 + 0.6 * i, -0.2 + 0.4 * j, z0 + 0.2 + 0.4 * l), (0.594, 0.394, 0.398), k)
    elif kind == "wrapped":
        h = rng.uniform(0.9, 1.35)
        b.add_box((0, 0, z0 + h / 2), (1.19, 0.79, h), M[f"kraft{rng.randrange(4)}"])
        b.add_box((0, 0, z0 + h / 2 + 0.005), (1.215, 0.815, h + 0.02), M["wrap"])
    elif kind == "white":
        h = rng.choice((0.8, 1.0, 1.2))
        b.add_box((0, 0, z0 + h / 2), (1.19, 0.79, h), M["white_box"])
        b.add_box((0, 0, z0 + h / 2 + 0.005), (1.215, 0.815, h + 0.02), M["wrap"])
    elif kind == "crates":
        c = M[rng.choice(("crate_blue", "crate_blue", "crate_grey"))]
        n = rng.choice((3, 4, 5))
        for l in range(n):
            for i in range(3):
                for j in range(2):
                    b.add_box((-0.4 + 0.4 * i, -0.2 + 0.4 * j, z0 + 0.12 + 0.24 * l), (0.39, 0.39, 0.235), c)
    return b


LOAD_KINDS = ["cartons", "cartons", "wrapped", "wrapped", "white", "big", "crates"]


# ============================================================================ building
def building(M, C):
    mb = MB()      # shell: no shadows in Gazebo
    # floor
    mb.add_quad([(BX0, BY0, 0), (BX1, BY0, 0), (BX1, BY1, 0), (BX0, BY1, 0)], M["concrete"], uv=4.0)
    # saw-cut joints, offset so none crosses a camera footprint (|x|,|y| < 3 around the cell)
    for x in [x0 for x0 in range(-21, 24, 6)]:
        mb.add_box((x, (BY0 + BY1) / 2, 0.0003), (0.008, BY1 - BY0, 0.0006), M["joint"])
    for y in [y0 for y0 in range(-15, 14, 6)]:
        mb.add_box(((BX0 + BX1) / 2, y, 0.0003), (BX1 - BX0, 0.008, 0.0006), M["joint"])

    def wall(axis, fixed, a0, a1, openings, outward):
        """Wall along `axis` ('x' or 'y') at coordinate `fixed`, from a0 to a1, with
        rectangular openings [(c0, c1, top)]. outward = +1/-1 normal direction."""
        cuts = sorted(set([a0, a1] + [o[0] for o in openings] + [o[1] for o in openings]))
        for s0, s1 in zip(cuts[:-1], cuts[1:]):
            mid = (s0 + s1) / 2
            hole = [o for o in openings if o[0] <= mid <= o[1]]
            zlo = hole[0][2] if hole else 0.0
            for z0, z1, m in ((0.0, 1.2, M["plinth"]), (1.2, H_ROOF + 0.2, M["wall"])):
                lo = max(z0, zlo)
                if z1 <= lo:
                    continue
                c = fixed + outward * WT / 2
                if axis == "x":
                    mb.add_box((mid, c, (lo + z1) / 2), (s1 - s0, WT, z1 - lo), m)
                else:
                    mb.add_box((c, mid, (lo + z1) / 2), (WT, s1 - s0, z1 - lo), m)
        # accent band (inside face, above every opening)
        if axis == "x":
            mb.add_box(((a0 + a1) / 2, fixed - outward * 0.006, 5.9), (a1 - a0, 0.012, 0.5), M["accent"])
        else:
            mb.add_box((fixed - outward * 0.006, (a0 + a1) / 2, 5.9), (0.012, a1 - a0, 0.5), M["accent"])

    # horizontal sandwich-panel seams on the inside faces
    for k in range(9):
        z = 1.2 + k * 1.0
        for (c, sz) in (((0, BY1 - 0.004, z), (BX1 - BX0, 0.008, 0.012)), ((0, BY0 + 0.004, z), (BX1 - BX0, 0.008, 0.012)),
                        ((BX1 - 0.004, -1, z), (0.008, BY1 - BY0, 0.012)), ((BX0 + 0.004, -1, z), (0.008, BY1 - BY0, 0.012))):
            if abs(z - 5.9) > 0.4:
                mb.add_box(c, sz, M["joint"])
    wall("x", BY1, BX0 - WT, BX1 + WT, [(d - DOCK_W / 2, d + DOCK_W / 2, DOCK_H) for d in DOORS_N], +1)
    wall("x", BY0, BX0 - WT, BX1 + WT, [(6.0, 7.1, 2.2)], -1)                  # personnel door
    wall("y", BX1, BY0, BY1, [(EAST_DOOR[0], EAST_DOOR[1], EAST_DOOR[2])], +1)
    wall("y", BX0, BY0, BY1, [], -1)

    rf = MB()
    # roof deck with skylight strips
    edges = [BY0 - WT]
    for y in SKY_Y:
        edges += [y - SKY_W / 2, y + SKY_W / 2]
    edges.append(BY1 + WT)
    for k in range(0, len(edges), 2):
        y0, y1 = edges[k], edges[k + 1]
        rf.add_box(((BX0 + BX1) / 2, (y0 + y1) / 2, H_ROOF + 0.1), (BX1 - BX0 + 2 * WT, y1 - y0, 0.2), M["roof"])
    for y in SKY_Y:
        rf.add_box(((BX0 + BX1) / 2, y, H_ROOF + 0.25), (BX1 - BX0 - 6, SKY_W, 0.02), M["sky_glass"])
        for side in (-1, 1):  # upstands
            rf.add_box(((BX0 + BX1) / 2, y + side * SKY_W / 2, H_ROOF + 0.12), (BX1 - BX0, 0.05, 0.25), M["galv"])
        for x in range(int(BX0) + 3, int(BX1) - 2, 3):                        # blanking ends
            pass
    # blank the skylight strip ends (skylights span x[-21, 21])
    for y in SKY_Y:
        for xs in ((BX0 - WT, BX0 + 3), (BX1 - 3, BX1 + WT)):
            rf.add_box(((xs[0] + xs[1]) / 2, y, H_ROOF + 0.1), (xs[1] - xs[0], SKY_W, 0.2), M["roof"])

    tag(rf.build("GZ_roof", C), "shell")
    # structural frames: I-section columns on both long walls + lattice trusses across
    def ibeam_col(x, y, h, dirn):
        f, d, t = 0.30, 0.30, 0.018
        mb.add_box((x, y + dirn * (d / 2 - t / 2), h / 2), (f, t, h), M["steel"])
        mb.add_box((x, y - dirn * (d / 2 - t / 2), h / 2), (f, t, h), M["steel"])
        mb.add_box((x, y, h / 2), (0.012, d, h), M["steel"])
        mb.add_box((x, y, 0.01), (0.5, 0.5, 0.02), M["steel"])               # base plate

    for x in FRAMES_X + [BX0 + 0.2, BX1 - 0.2]:
        ibeam_col(x, BY0 + 0.16, H_ROOF, 1)
        ibeam_col(x, BY1 - 0.16, H_ROOF, -1)
    for x in FRAMES_X:
        top, bot = H_ROOF - 0.08, H_TRUSS
        mb.add_box((x, (BY0 + BY1) / 2, top), (0.16, BY1 - BY0, 0.16), M["steel"])
        mb.add_box((x, (BY0 + BY1) / 2, bot), (0.16, BY1 - BY0 - 0.6, 0.16), M["steel"])
        n = 20
        ys = [BY0 + 0.3 + k * (BY1 - BY0 - 0.6) / n for k in range(n + 1)]
        for k, y in enumerate(ys):
            mb.add_beam((x, y, bot), (x, y, top), 0.08, 0.08, M["steel"])
            if k < n:
                y2 = ys[k + 1]
                a, b = ((y, bot), (y2, top)) if k % 2 == 0 else ((y, top), (y2, bot))
                mb.add_beam((x, a[0], a[1]), (x, b[0], b[1]), 0.07, 0.07, M["steel"])
    # purlins
    for k in range(int((BY1 - BY0) / 1.5) + 1):
        y = BY0 + k * 1.5
        if any(abs(y - s) < SKY_W / 2 + 0.05 for s in SKY_Y):
            continue
        mb.add_box(((BX0 + BX1) / 2, y, H_ROOF - 0.06), (BX1 - BX0, 0.08, 0.12), M["galv"])
    ob = tag(mb.build("GZ_shell", C), "shell", collide=False)

    # --- services (render + Gazebo): sprinklers, ventilation duct, lights, cable drops
    sv = MB()
    for y in range(-14, 14, 4):
        sv.add_cyl(((BX0 + BX1) / 2, y + 0.5, 9.3), 0.035, BX1 - BX0 - 0.6, M["sprinkler"], axis="X", seg=8, caps=False)
        for x in range(int(BX0) + 2, int(BX1) - 1, 3):
            sv.add_cyl((x, y + 0.5, 9.2), 0.012, 0.2, M["galv"], seg=6)
    sv.add_cyl(((BX0 + BX1) / 2, 11.2, 8.1), 0.32, BX1 - BX0 - 1.0, M["galv"], axis="X", seg=20)
    for x in range(-20, 21, 5):
        sv.add_cyl((x, 11.2, 8.9), 0.02, 1.2, M["dark"], seg=6)              # duct hangers
        sv.add_cyl((x, 11.2, 7.72), 0.16, 0.12, M["galv"], axis="Z", seg=16)   # diffusers
    lamps = []
    for x in (-21, -15, -9, -3, 3, 9, 15, 21):
        for y in (-12.8, -7.5, -3.5, 4.0, 8.5, 12.0):
            sv.add_cyl((x, y, 8.25), 0.24, 0.10, M["lamp_body"], seg=20)
            sv.add_cyl((x, y, 8.40), 0.10, 0.20, M["lamp_body"], seg=12)
            sv.add_cyl((x, y, 8.19), 0.20, 0.012, M["lamp"], seg=20)
            sv.add_cyl((x, y, 9.3), 0.006, 1.6, M["dark"], seg=4, caps=False)
            lamps.append((x, y, 8.17))
    for x in (-3.0, 3.0):
        for y in (-0.8, 1.2):
            sv.add_cyl((x, y, 8.25), 0.24, 0.10, M["lamp_body"], seg=20)
            sv.add_cyl((x, y, 8.19), 0.20, 0.012, M["lamp"], seg=20)
            sv.add_cyl((x, y, 9.3), 0.006, 1.6, M["dark"], seg=4, caps=False)
            lamps.append((x, y, 8.17))
    tag(sv.build("GZ_services", C), "shell")
    return lamps


# ============================================================================ racking
def racking(M, C, rng):
    rk = MB()
    ld = MB()
    bay, depth, flue = 2.75, 1.1, 0.2
    x0, nb = -21.0, 13
    levels = [0.0, 1.75, 3.45, 5.15]
    H = 6.9
    rows = [(-15.8, "N"), (-11.5, "S"), (-10.2, "N"), (-5.9, "S")]  # (back y, face)
    # rows: single against wall (faces N), double (back-to-back), single (faces S)
    rows = [(-15.75, -15.75 + depth), (-11.5, -11.5 + depth), (-11.5 + depth + flue, -11.5 + 2 * depth + flue),
            (-5.9, -5.9 + depth)]
    kinds = {}
    for (ya, yb) in rows:
        for i in range(nb + 1):
            x = x0 + i * bay
            for y in (ya + 0.04, yb - 0.04):
                rk.add_box((x, y, H / 2), (0.09, 0.075, H), M["upright"])
                rk.add_box((x, y, 0.005), (0.16, 0.14, 0.01), M["upright"])
            n = 7
            for k in range(n):                                            # frame bracing
                z1, z2 = 0.15 + k * (H - 0.3) / n, 0.15 + (k + 1) * (H - 0.3) / n
                a, b = (ya + 0.04, yb - 0.04) if k % 2 == 0 else (yb - 0.04, ya + 0.04)
                rk.add_beam((x, a, z1), (x, b, z2), 0.03, 0.03, M["upright"])
            rk.add_beam((x, ya + 0.04, 0.15), (x, yb - 0.04, 0.15), 0.03, 0.03, M["upright"])
            if (ya + yb) / 2 > -6 or (ya + yb) / 2 < -15:   # aisle-facing column guards
                pass
        for i in range(nb):
            xa = x0 + i * bay
            for lv in levels[1:]:
                for y in (ya + 0.06, yb - 0.06):
                    rk.add_box((xa + bay / 2, y, lv - 0.06), (bay - 0.1, 0.05, 0.12), M["beam"])
            for lv in levels:
                for s in range(3):
                    if rng.random() < 0.18:
                        continue
                    kind = rng.choice(LOAD_KINDS)
                    key = (kind, rng.randrange(3))
                    if key not in kinds:
                        kinds[key] = load_mb(M, kind, random.Random(hash(key) & 0xffff))
                    px = xa + 0.1 + 0.4 + s * 0.9 + rng.uniform(-0.03, 0.03)
                    Mx = Matrix.Translation((px, (ya + yb) / 2 + rng.uniform(-0.03, 0.03), lv)) @ \
                        Matrix.Rotation(math.pi / 2, 4, "Z")
                    ld.extend(kinds[key], Mx)
        # end-of-row protection + row sign (east end, faces +x)
        xe = x0 + nb * bay + 0.35
        rk.add_box((xe, (ya + yb) / 2, 0.2), (0.12, yb - ya + 0.2, 0.4), M["yellow"])
    # column guards for the aisle-facing uprights
    for (ya, yb) in (rows[0], rows[3]):
        yy = yb - 0.04 if ya < -15 else ya + 0.04
        for i in range(nb + 1):
            rk.add_box((x0 + i * bay, yy + (0.08 if ya < -15 else -0.08), 0.2), (0.18, 0.06, 0.4), M["yellow"])
    signs = MB()
    for k, (ya, yb) in enumerate(rows):
        xe = x0 + nb * bay + 0.5
        signs.add_box((xe, (ya + yb) / 2, 7.3), (0.04, 0.9, 0.5), M["accent"])
    tag(rk.build("GZ_racks", C), "interior", collide=False)
    tag(ld.build("GZ_rack_loads", C), "interior")
    tag(signs.build("GZ_rack_signs", C), "interior")
    for k, (ya, yb) in enumerate(rows):
        xe = x0 + nb * bay + 0.53
        t = L.text_mesh(f"ROW {k + 1:02d}", 0.28, (xe, (ya + yb) / 2, 7.3), (math.pi / 2, 0, math.pi / 2),
                        M["sign_w"], C, name=f"GZ_row_sign_{k}")
        tag(t, "interior")
    # collision proxies (one box per rack row)
    for k, (ya, yb) in enumerate(rows):
        c = MB()
        c.add_box((x0 + nb * bay / 2, (ya + yb) / 2, H / 2), (nb * bay, yb - ya, H), M["upright"])
        tag(c.build(f"COL_rack_{k}", C), "collision", collide=True)
    return rows


# ============================================================================ cell surroundings
def fence(M, C, cfg):
    F = FENCE
    fb = MB()
    ex = cfg["exchanger"]
    st = cfg["pallet_station"]
    lane_lo = ex["lane_y"] - st["area"][1] / 2 - 0.30
    lane_hi = ex["lane_y"] + st["area"][1] / 2 + 0.30
    cv = cfg["conveyor"]
    conv_lo, conv_hi = cv["y"] - cv["belt_width"] / 2 - 0.1, cv["y"] + cv["belt_width"] / 2 + 0.1
    gate = (1.2, 2.2)

    def panel(p0, p1, z0=F["gap"], z1=F["h"]):
        p0, p1 = Vector(p0), Vector(p1)
        d = p1 - p0
        L_ = d.length
        u = d.normalized()
        nv = int(L_ / 0.05)
        nh = int((z1 - z0) / 0.10)
        for k in range(1, nv):
            p = p0 + u * (k * L_ / nv)
            fb.add_box((p.x, p.y, (z0 + z1) / 2), (0.004, 0.004, z1 - z0 - 0.04), M["mesh"])
        for k in range(1, nh):
            z = z0 + k * (z1 - z0) / nh
            fb.add_beam((p0.x, p0.y, z), (p1.x, p1.y, z), 0.004, 0.004, M["mesh"])
        for z in (z0, z1):                                                   # frame tubes
            fb.add_beam((p0.x, p0.y, z), (p1.x, p1.y, z), 0.02, 0.02, M["mesh"])

    def run(p0, p1, openings=()):
        """Fence line from p0 to p1 with openings given as (a0, a1) along the line."""
        p0, p1 = Vector(p0), Vector(p1)
        L_ = (p1 - p0).length
        u = (p1 - p0).normalized()
        segs, a = [], 0.0
        for o0, o1 in sorted(openings):
            segs.append((a, o0)); a = o1
        segs.append((a, L_))
        for s0, s1 in segs:
            if s1 - s0 < 0.05:
                continue
            n = max(1, math.ceil((s1 - s0) / 1.3))
            for k in range(n + 1):
                p = p0 + u * (s0 + k * (s1 - s0) / n)
                fb.add_box((p.x, p.y, F["h"] / 2 + 0.05), (0.06, 0.06, F["h"] + 0.1), M["yellow"])
                fb.add_box((p.x, p.y, 0.005), (0.14, 0.14, 0.01), M["yellow"])
                if k < n:
                    q = p0 + u * (s0 + (k + 1) * (s1 - s0) / n)
                    panel(p + u * 0.03, q - u * 0.03)
        return u, p0

    run((F["x0"], F["y1"]), (F["x1"], F["y1"]))
    run((F["x0"], F["y0"]), (F["x1"], F["y0"]), [(gate[0] - F["x0"], gate[1] - F["x0"])])
    run((F["x1"], F["y0"]), (F["x1"], F["y1"]), [(lane_lo - F["y0"], lane_hi - F["y0"])])
    run((F["x0"], F["y0"]), (F["x0"], F["y1"]),
        [(lane_lo - F["y0"], lane_hi - F["y0"]), (conv_lo - F["y0"], conv_hi - F["y0"])])
    # gate leaf (closed) with interlock switch
    panel((gate[0] + 0.05, F["y0"] - 0.03), (gate[1] - 0.05, F["y0"] - 0.03), 0.1, 2.05)
    fb.add_box((gate[1] - 0.1, F["y0"] - 0.08, 1.1), (0.06, 0.05, 0.12), M["sign_red"])
    fb.add_box((gate[0] + 0.1, F["y0"] - 0.1, 1.05), (0.03, 0.12, 0.03), M["dark"])      # handle
    # conveyor pass-through: tunnel guard outside the fence + panel above the opening
    x = F["x0"]
    for y in (conv_lo, conv_hi):
        fb.add_box((x - 0.5, y, 0.95), (1.0, 0.01, 1.3), M["mesh"])
    fb.add_box((x - 0.5, (conv_lo + conv_hi) / 2, 1.6), (1.0, conv_hi - conv_lo, 0.01), M["mesh"])
    panel((x, conv_lo), (x, conv_hi), 1.62, F["h"])
    # light curtains at both pallet openings (emitter/receiver posts)
    lc = []
    for xx in (F["x0"], F["x1"]):
        for y in (lane_lo, lane_hi):
            fb.add_box((xx, y, 0.95), (0.05, 0.05, 1.8), M["dark"])
            fb.add_box((xx, y, 1.88), (0.06, 0.06, 0.06), M["yellow"])
            fb.add_box((xx, y + (0.03 if y == lane_lo else -0.03), 0.95), (0.01, 0.006, 1.6), M["red_emit"])
    # andon tower on the NE corner post, warning signs
    ax, ay = F["x1"], F["y1"]
    fb.add_cyl((ax, ay, 2.33), 0.035, 0.06, M["dark"], seg=12)
    fb.add_cyl((ax, ay, 2.40), 0.04, 0.08, M["red_emit"] if False else M["amber"], seg=16)
    fb.add_cyl((ax, ay, 2.48), 0.04, 0.08, M["amber"], seg=16)
    fb.add_cyl((ax, ay, 2.56), 0.04, 0.08, M["green_emit"], seg=16)
    fb.add_cyl((ax, ay, 2.61), 0.04, 0.02, M["dark"], seg=16)
    tag(fb.build("GZ_fence", C), "interior")
    sg = MB()
    # (position, outward facing direction)
    signs = [((-1.2, F["y1"] + 0.03, 1.55), (0, 1)), ((-1.2, F["y0"] - 0.03, 1.55), (0, -1)),
             ((F["x1"] + 0.03, 1.2, 1.55), (1, 0))]
    for (x, y, z), (fx, fy) in signs:
        sg.add_box((x, y, z), (0.6, 0.006, 0.42), M["yellow"], rz=(math.pi / 2 if fx else 0.0))
    tag(sg.build("GZ_fence_signs", C), "interior")
    for i, ((x, y, z), (fx, fy)) in enumerate(signs):
        rz = math.atan2(fy, fx) + math.pi / 2
        for txt, dz, s in (("CAUTION", 0.1, 0.1), ("ROBOT CELL", -0.03, 0.075), ("AUTHORISED ACCESS ONLY", -0.13, 0.04)):
            t = L.text_mesh(txt, s, (x + fx * 0.004, y + fy * 0.004, z + dz), (math.pi / 2, 0, rz),
                            M["black"], C, name=f"GZ_sign_{i}_{txt[:4]}")
            tag(t, "interior")
    # collision: thin boxes per fence side (gaps at openings kept open)
    cb = MB()
    for (x0, y0, x1, y1) in ((F["x0"], F["y1"], F["x1"], F["y1"]), (F["x0"], F["y0"], F["x1"], F["y0"])):
        cb.add_box(((x0 + x1) / 2, y0, F["h"] / 2), (x1 - x0, 0.05, F["h"]), M["mesh"])
    for xx in (F["x0"], F["x1"]):
        cb.add_box((xx, (F["y0"] + lane_lo) / 2, F["h"] / 2), (0.05, lane_lo - F["y0"], F["h"]), M["mesh"])
    cb.add_box((F["x1"], (lane_hi + F["y1"]) / 2, F["h"] / 2), (0.05, F["y1"] - lane_hi, F["h"]), M["mesh"])
    cb.add_box((F["x0"], (conv_hi + F["y1"]) / 2, F["h"] / 2), (0.05, F["y1"] - conv_hi, F["h"]), M["mesh"])
    tag(cb.build("COL_fence", C), "collision", collide=True)
    return dict(lane_lo=lane_lo, lane_hi=lane_hi)


def floor_marks(M, C, cfg, lanes):
    fm = MB()
    F = FENCE
    z, t = 0.0012, 0.0024
    # hazard band hugging the fence outside, and a yellow keep-out line 0.6 m out
    for (x0, y0, x1, y1) in ((F["x0"] - 0.6, F["y1"] + 0.6, F["x1"] + 0.6, F["y1"] + 0.6),
                             (F["x0"] - 0.6, F["y0"] - 0.6, F["x1"] + 0.6, F["y0"] - 0.6)):
        fm.add_box(((x0 + x1) / 2, y0, z), (x1 - x0, 0.08, t), M["paint_y"])
    cv = cfg["conveyor"]
    for xx in (F["x0"] - 0.6, F["x1"] + 0.6):
        top0 = (cv["y"] + cv["belt_width"] / 2 + 0.2) if xx < 0 else lanes["lane_hi"]
        for (ya, yb) in ((F["y0"] - 0.6, lanes["lane_lo"]), (top0, F["y1"] + 0.6)):
            fm.add_box((xx, (ya + yb) / 2, z), (0.08, yb - ya, t), M["paint_y"])
    for (x0, y0, x1, y1, w, h) in ((F["x0"], F["y1"] + 0.2, F["x1"], F["y1"] + 0.2, F["x1"] - F["x0"], 0.25),
                                   (F["x0"], F["y0"] - 0.2, F["x1"], F["y0"] - 0.2, F["x1"] - F["x0"], 0.25)):
        fm.add_quad([(x0, y0 - h / 2, z + 0.0004), (x1, y0 - h / 2, z + 0.0004),
                     (x1, y0 + h / 2, z + 0.0004), (x0, y0 + h / 2, z + 0.0004)], M["hazard"],
                    uvs=[(0, 0), (w / h, 0), (w / h, 1), (0, 1)])
    # main aisles (double yellow) and a green pedestrian walkway on the north side
    for y in (3.0, 5.4, -3.2, -4.6):
        fm.add_box((0, y, z), (BX1 - BX0 - 1.0, 0.1, t), M["paint_y"])
    fm.add_box((0, 5.9, z), (BX1 - BX0 - 1.0, 0.9, t), M["paint_g"])
    for x in range(-22, 23, 2):
        fm.add_box((x, 5.9, z + 0.0003), (0.6, 0.12, t), M["paint_w"])
    # rack aisle edge lines
    for y in (-14.55, -11.6, -9.0, -6.05):
        fm.add_box((-21 + 13 * 2.75 / 2, y, z), (13 * 2.75, 0.08, t), M["paint_y"])
    # staging lanes in front of the docks
    for d in DOORS_N:
        for x in (d - 1.3, d + 1.3):
            fm.add_box((x, 9.6, z), (0.08, 6.4, t), M["paint_w"])
        fm.add_box((d, 6.4, z), (2.68, 0.08, t), M["paint_w"])
    # outbound lane after the wrapper
    for y in (-1.25, 0.05):
        fm.add_box((10.5, y, z), (8.0, 0.08, t), M["paint_b"])
    tag(fm.build("GZ_floor_marks", C), "interior")
    texts = [("CELL 01  ·  AUTOMATED PALLETIZING", 0.42, (0.0, F["y1"] + 1.5, z + 0.001), 0),
             ("KEEP CLEAR", 0.35, (0.0, F["y0"] - 1.25, z + 0.001), math.pi),
             ("WALKWAY", 0.3, (-12.0, 5.9, z + 0.0015), 0),
             ("WALKWAY", 0.3, (12.0, 5.9, z + 0.0015), 0),
             ("OUTBOUND", 0.35, (11.5, -0.6, z + 0.001), -math.pi / 2)]
    for i, (s, sz, loc, rz) in enumerate(texts):
        m = M["paint_w"] if "WALK" not in s else M["paint_w"]
        tag(L.text_mesh(s, sz, loc, (0, 0, rz), m, C, extrude=0.0, name=f"GZ_floor_text_{i}"), "interior")
    for i, d in enumerate(DOORS_N):
        tag(L.text_mesh(f"D{i + 1}", 0.6, (d, 7.2, z + 0.001), (0, 0, 0), M["paint_w"], C, extrude=0.0,
                        name=f"GZ_dock_num_{i}"), "interior")


def infeed_line(M, C, cfg, rng):
    """Visual upstream conveyor feeding the cell's belt, a case sealer / labeller tunnel
    and a manual induction table. Belt surface ends 3 cm before the sim conveyor."""
    cv = cfg["conveyor"]
    y, bw, zt = cv["y"], cv["belt_width"], cv["belt_top_z"]
    x0, x1 = -14.0, cv["x_start"] - 0.03
    ib = MB()
    ib.add_box(((x0 + x1) / 2, y, zt - 0.02), (x1 - x0, bw, 0.04), M["rubber"])
    for s in (-1, 1):
        ib.add_box(((x0 + x1) / 2, y + s * (bw / 2 + 0.03), zt - 0.03), (x1 - x0, 0.04, 0.14), M["galv"])
    x = x0 + 0.3
    while x < x1:
        for s in (-1, 1):
            ib.add_box((x, y + s * (bw / 2 + 0.02), (zt - 0.1) / 2), (0.05, 0.05, zt - 0.1), M["galv"])
            ib.add_box((x, y + s * (bw / 2 + 0.02), 0.005), (0.12, 0.12, 0.01), M["galv"])
        ib.add_box((x, y, 0.25), (0.04, bw + 0.04, 0.04), M["galv"])
        x += 1.6
    ib.add_box((x0 + 0.4, y - bw / 2 - 0.2, 0.45), (0.3, 0.25, 0.25), M["crate_blue"])         # drive
    # case sealer / labeller tunnel
    sx = -9.5
    for s in (-1, 1):
        ib.add_box((sx, y + s * (bw / 2 + 0.3), 0.9), (1.6, 0.4, 1.8), M["cabinet"])
    ib.add_box((sx, y, 1.75), (1.6, bw + 1.0, 0.1), M["cabinet"])
    ib.add_box((sx, y - bw / 2 - 0.51, 1.3), (1.0, 0.01, 0.5), M["glass"])
    ib.add_box((sx + 0.9, y - bw / 2 - 0.3, 1.9), (0.08, 0.08, 0.25), M["green_emit"])
    # induction table + flat cartons
    ib.add_box((x0 - 0.7, y, 0.78), (1.2, 1.2, 0.04), M["galv"])
    for dx in (-0.5, 0.5):
        for dy in (-0.5, 0.5):
            ib.add_box((x0 - 0.7 + dx, y + dy, 0.38), (0.05, 0.05, 0.76), M["galv"])
    p = load_mb(M, "cartons", rng, True)
    ob = MB()
    ob.extend(pallet_mb(M, True), Matrix.Translation((x0 - 0.8, y - 1.6, 0)))
    for k in range(18):
        ob.add_box((x0 - 0.8, y - 1.6, 0.144 + 0.006 + k * 0.012), (1.15, 0.78, 0.011), M["kraft1"])
    del p
    ib.extend(ob, Matrix.Identity(4))
    tag(ib.build("GZ_infeed_line", C), "interior")


def pallet_dispenser_and_wrapper(M, C, cfg, rng):
    ex = cfg["exchanger"]
    ly = ex["lane_y"]
    pb = MB()
    # empty pallet dispenser upstream of the pallet entry
    dx = -5.4
    pal = pallet_mb(M, True, "wood")
    for k in range(11):
        pb.extend(pal, Matrix.Translation((dx + rng.uniform(-0.01, 0.01), ly + rng.uniform(-0.01, 0.01), k * 0.144)))
    for sx in (-0.7, 0.7):
        for sy in (-0.5, 0.5):
            pb.add_box((dx + sx, ly + sy, 1.1), (0.08, 0.08, 2.2), M["yellow"])
    for sy in (-0.5, 0.5):
        pb.add_box((dx, ly + sy, 2.2), (1.48, 0.08, 0.08), M["yellow"])
    pb.add_box((dx + 0.9, ly - 0.5, 1.0), (0.12, 0.3, 0.3), M["cabinet"])
    # stretch wrapper (turntable + mast + film carriage) with a wrapped load on it
    wx = 6.6
    pb.add_cyl((wx, ly, 0.04), 0.95, 0.08, M["dark"], seg=48)
    pb.add_cyl((wx, ly, 0.085), 0.9, 0.012, M["galv"], seg=48)
    my = ly - 1.45
    pb.add_box((wx, my, 1.6), (0.28, 0.28, 3.2), M["cabinet"])
    pb.add_box((wx, my + 0.55, 0.06), (0.35, 1.0, 0.12), M["cabinet"])
    pb.add_box((wx, my + 0.2, 1.3), (0.5, 0.2, 0.7), M["cabinet"])
    pb.add_cyl((wx + 0.12, my + 0.35, 1.3), 0.07, 0.5, M["wrap"], seg=16)
    pb.add_box((wx, my - 0.18, 2.3), (0.4, 0.06, 0.5), M["dark"])
    pb.add_box((wx + 0.3, my - 0.2, 2.6), (0.05, 0.05, 0.1), M["green_emit"])
    load = load_mb(M, "wrapped", random.Random(4), True)
    pb.extend(load, Matrix.Translation((wx, ly, 0.092)) @ Matrix.Rotation(0.3, 4, "Z"))
    # outbound: wrapped full pallets waiting for a forklift
    for i, xx in enumerate((9.6, 11.1, 12.6)):
        pb.extend(load_mb(M, rng.choice(["wrapped", "white"]), random.Random(10 + i), True),
                  Matrix.Translation((xx, ly, 0)) @ Matrix.Rotation(math.pi / 2 + rng.uniform(-0.03, 0.03), 4, "Z"))
    tag(pb.build("GZ_pallet_flow", C), "interior")
    cb = MB()
    cb.add_box((wx, my, 1.6), (0.3, 0.3, 3.2), M["cabinet"])
    cb.add_box((wx, ly, 0.05), (1.9, 1.9, 0.1), M["dark"])
    cb.add_box((dx, ly, 0.8), (1.5, 1.1, 1.6), M["yellow"])
    tag(cb.build("COL_pallet_flow", C), "collision", collide=True)


def control_station(M, C):
    cs = MB()
    cx, cy = 4.9, 1.6
    cs.add_box((cx, cy, 1.0), (1.0, 0.5, 2.0), M["cabinet"])
    cs.add_box((cx, cy, 2.04), (1.04, 0.54, 0.08), M["cabinet"])
    cs.add_box((cx, cy + 0.251, 1.0), (0.004, 0.004, 1.9), M["dark"])      # door split
    cs.add_box((cx - 0.2, cy + 0.26, 1.0), (0.03, 0.03, 0.15), M["dark"])
    cs.add_box((cx + 0.2, cy + 0.26, 1.0), (0.03, 0.03, 0.15), M["dark"])
    cs.add_box((cx, cy, 0.05), (1.0, 0.5, 0.1), M["dark"])
    # HMI pedestal facing the aisle
    hx, hy = 4.35, 2.6
    cs.add_box((hx, hy, 0.6), (0.08, 0.08, 1.2), M["galv"])
    cs.add_box((hx, hy, 0.01), (0.4, 0.4, 0.02), M["galv"])
    M_ = Matrix.Translation((hx, hy + 0.02, 1.35)) @ Matrix.Rotation(-0.35, 4, "X")
    cs.add_box(None, (0.52, 0.05, 0.36), M["dark"], M=M_)
    cs.add_box((cx + 0.35, cy + 0.26, 1.5), (0.06, 0.02, 0.06), M["sign_red"])  # e-stop
    tag(cs.build("GZ_control", C), "interior")
    scr = MB()
    Ms = Matrix.Translation((hx, hy + 0.047, 1.35)) @ Matrix.Rotation(-0.35, 4, "X")
    pts = [Ms @ Vector(p) for p in ((-0.24, 0, -0.16), (0.24, 0, -0.16), (0.24, 0, 0.16), (-0.24, 0, 0.16))]
    scr.add_quad(pts, M["hmi"], uvs=[(1, 0), (0, 0), (0, 1), (1, 1)])
    tag(scr.build("GZ_hmi_screen", C), "interior")


def docks(M, C, rng):
    dk = MB()
    for i, d in enumerate(DOORS_N):
        # leveler plate + yellow lip edges
        dk.add_box((d, BY1 - 1.25, 0.004), (2.0, 2.5, 0.008), M["dark"])
        for s in (-1, 1):
            dk.add_box((d + s * 1.02, BY1 - 1.25, 0.005), (0.06, 2.5, 0.01), M["yellow"])
        # hazard-striped jambs + door guide rails, dock light + bollards
        for s in (-1, 1):
            dk.add_box((d + s * (DOCK_W / 2 + 0.06), BY1 - 0.06, 1.3), (0.12, 0.12, 2.6), M["hazard"])
            dk.add_box((d + s * (DOCK_W / 2 - 0.03), BY1 - 0.05, DOCK_H + 0.35), (0.06, 0.08, 0.7), M["dark"])
        dk.add_box((d, BY1 - 0.3, DOCK_H + 0.75), (DOCK_W, 0.6, 0.04), M["dark"])
        dk.add_box((d + 1.9, BY1 - 0.08, 2.4), (0.18, 0.12, 0.32), M["dark"])
        dk.add_cyl((d + 1.9, BY1 - 0.15, 2.48), 0.05, 0.02, M["green_emit"] if d in OPEN_DOCKS else M["red_emit"], axis="Y", seg=12)
        for s in (-1, 1):
            dk.add_cyl((d + s * 1.75, BY1 - 0.35, 0.6), 0.08, 1.2, M["yellow"], seg=14)
        if d in OPEN_DOCKS:
            dk.add_box((d, BY1 + 0.05, DOCK_H - 0.25), (DOCK_W, 0.06, 0.5), M["door"])        # raised door
        else:
            dk.add_box((d, BY1 + 0.05, DOCK_H / 2), (DOCK_W, 0.06, DOCK_H), M["door"])
            dk.add_box((d, BY1 + 0.02, 1.1), (1.0, 0.01, 0.25), M["glass"])
        # exterior: shelter, bumpers
        for s in (-1, 1):
            dk.add_box((d + s * (DOCK_W / 2 + 0.25), BY1 + WT + 0.35, 1.3), (0.5, 0.7, 5.0), M["black"])
        dk.add_box((d, BY1 + WT + 0.35, 3.6), (DOCK_W + 1.0, 0.7, 0.8), M["black"])
        for s in (-1, 1):
            dk.add_box((d + s * 1.0, BY1 + WT + 0.06, -0.35), (0.25, 0.12, 0.45), M["rubber"])
    # truck court (1.2 m below the building floor) + surrounding yard
    dk.add_quad([(-40, BY1 + WT, -1.2), (40, BY1 + WT, -1.2), (40, 50, -1.2), (-40, 50, -1.2)], M["asphalt"], uv=4.0)
    dk.add_box((0, BY1 + WT / 2, -0.6), (80, WT, 1.2), M["plinth"])                   # dock wall
    for (xa, xb, ya, yb) in ((BX1 + WT, 60, BY0 - 40, BY1 + WT), (-60, BX0 - WT, BY0 - 40, BY1 + WT),
                             (BX0 - WT, BX1 + WT, BY0 - 40, BY0 - WT)):
        dk.add_quad([(xa, ya, -0.01), (xb, ya, -0.01), (xb, yb, -0.01), (xa, yb, -0.01)], M["asphalt"], uv=4.0)
    for k in range(-6, 7):                                                            # parking lines
        dk.add_box((k * 5.0 + 2.5, 30, -1.198), (0.12, 12, 0.004), M["paint_w"])
    # trailer backed into door 0 (open), bed level with the building floor
    tx = 0.0
    tl, tw, th = 13.6, 2.55, 2.8
    ty0 = BY1 + WT + 0.1
    tr = MB()
    for s in (-1, 1):
        tr.add_box((tx + s * (tw / 2 - 0.02), ty0 + tl / 2, th / 2), (0.04, tl, th), M["trailer"])
    tr.add_box((tx, ty0 + tl / 2, th + 0.02), (tw, tl, 0.04), M["trailer"])
    tr.add_box((tx, ty0 + tl / 2, -0.03), (tw, tl, 0.06), M["trailer_in"])
    tr.add_box((tx, ty0 + tl, th / 2), (tw, 0.04, th), M["trailer"])
    tr.add_box((tx, ty0 + tl / 2, -0.25), (0.3, tl, 0.35), M["dark"])                # chassis
    for yy in (ty0 + 1.4, ty0 + 2.7, ty0 + 4.0):                                   # rear axles
        for s in (-1, 1):
            tr.add_cyl((tx + s * 1.0, yy, -0.7), 0.5, 0.35, M["rubber"], axis="X", seg=20)
    for s in (-1, 1):                                                                 # landing gear
        tr.add_box((tx + s * 0.9, ty0 + tl - 2.5, -0.7), (0.1, 0.1, 1.0), M["dark"])
        tr.add_box((tx + s * (tw / 2 + 0.03), ty0 + 0.64, th / 2), (0.03, 1.27, th), M["trailer"])  # swung doors
    for k in range(4):                                                                # loaded pallets
        for s in (-1, 1):
            tr.extend(load_mb(M, "wrapped", random.Random(40 + k * 2 + s), True),
                      Matrix.Translation((tx + s * 0.62, ty0 + tl - 1.0 - k * 1.25, 0.0)) @ Matrix.Rotation(math.pi / 2, 4, "Z"))
    tag(dk.build("GZ_docks", C), "shell")
    tag(tr.build("GZ_trailer", C), "interior")
    # staging: wrapped pallets in the dock lanes
    sb = MB()
    for d in DOORS_N:
        n = rng.randrange(1, 5)
        for k in range(n):
            sb.extend(load_mb(M, rng.choice(["wrapped", "white", "cartons"]), random.Random(int(d * 10) + k), True),
                      Matrix.Translation((d + rng.uniform(-0.05, 0.05), 10.5 - k * 1.25, 0)) @
                      Matrix.Rotation(rng.uniform(-0.04, 0.04), 4, "Z"))
    tag(sb.build("GZ_staging", C), "interior")


def forklift_mb(M, fork_h=0.1, load=None):
    """Counterbalance forklift, origin at floor under the front axle, forks along +x."""
    f = MB()
    o, blk = M["forklift"], M["dark"]
    f.add_box((-1.0, 0, 0.55), (1.5, 1.05, 0.7), o)                 # chassis
    f.add_box((-1.55, 0, 0.75), (0.45, 1.05, 0.9), o)               # counterweight
    f.add_box((-1.0, 0, 1.0), (0.55, 0.5, 0.3), blk)                # seat
    f.add_box((-1.0, 0, 1.3), (0.1, 0.5, 0.5), blk)
    for (x, y) in ((-0.35, 0.5), (-0.35, -0.5), (-1.55, 0.5), (-1.55, -0.5)):
        f.add_box((x, y, 1.45), (0.06, 0.06, 1.3), blk)             # overhead guard posts
    f.add_box((-0.95, 0, 2.1), (1.3, 1.05, 0.05), blk)
    for k in range(6):
        f.add_box((-1.5 + k * 0.2, 0, 2.13), (0.04, 1.0, 0.02), blk)
    for (x, s) in ((-0.2, 1), (-0.2, -1), (-1.45, 1), (-1.45, -1)):
        f.add_cyl((x, s * 0.45, 0.28), 0.28, 0.22, M["rubber"], axis="Y", seg=18)
    f.add_cyl((-0.65, 0.2, 1.2), 0.17, 0.03, blk, axis="Z", seg=16)     # steering wheel
    for s in (-1, 1):
        f.add_box((0.05, s * 0.32, 1.25), (0.08, 0.1, 2.5), blk)        # mast
    f.add_box((0.05, 0, 2.45), (0.08, 0.74, 0.08), blk)
    f.add_box((0.12, 0, fork_h + 0.45), (0.04, 0.9, 0.8), blk)          # carriage
    for s in (-1, 1):
        f.add_box((0.72, s * 0.3, fork_h + 0.02), (1.15, 0.12, 0.045), blk)
    f.add_box((-0.3, 0.55, 2.2), (0.1, 0.1, 0.08), M["amber"])
    if load is not None:
        f.extend(load, Matrix.Translation((0.75, 0, fork_h + 0.045)) @ Matrix.Rotation(math.pi / 2, 4, "Z"))
    return f


def vehicles(M, C, rng):
    vb = MB()
    fl = forklift_mb(M, 0.25, load_mb(M, "wrapped", random.Random(77), True))
    vb.extend(fl, Matrix.Translation((-6.8, 8.0, 0)) @ Matrix.Rotation(math.radians(115), 4, "Z"))
    fl2 = forklift_mb(M, 0.3, load_mb(M, "cartons", random.Random(78), False))
    vb.extend(fl2, Matrix.Translation((2.0, -13.05, 0)) @ Matrix.Rotation(math.pi, 4, "Z"))
    # pallet jack near the dispenser
    pj = MB()
    for s in (-1, 1):
        pj.add_box((0.6, s * 0.28, 0.05), (1.15, 0.16, 0.07), M["forklift"])
    pj.add_box((0, 0, 0.35), (0.25, 0.6, 0.5), M["forklift"])
    pj.add_beam((0, 0, 0.5), (-0.45, 0, 1.2), 0.04, 0.04, M["dark"])
    pj.add_box((-0.48, 0, 1.22), (0.08, 0.35, 0.04), M["dark"])
    vb.extend(pj, Matrix.Translation((-7.4, -1.9, 0)) @ Matrix.Rotation(0.4, 4, "Z"))
    tag(vb.build("GZ_vehicles", C), "interior")


def office(M, C):
    ob = MB()
    x0, x1, y0, y1, h = BX0, -15.5, 8.4, BY1, 6.4
    ob.add_box(((x0 + x1) / 2, y0 + 0.1, h / 2), (x1 - x0, 0.2, h), M["office"])
    ob.add_box((x1 - 0.1, (y0 + y1) / 2, h / 2), (0.2, y1 - y0, h), M["office"])
    ob.add_box(((x0 + x1) / 2, (y0 + y1) / 2, h + 0.1), (x1 - x0, y1 - y0, 0.2), M["office"])
    ob.add_box(((x0 + x1) / 2, (y0 + y1) / 2, 3.2), (x1 - x0, y1 - y0, 0.2), M["office"])
    for z in (1.0, 4.2):                                         # window strips (warm, lit)
        ob.add_box(((x0 + x1) / 2 + 0.6, y0 - 0.005, z + 0.65), (x1 - x0 - 2.5, 0.02, 1.3), M["window"])
        ob.add_box((x1 + 0.005, (y0 + y1) / 2 + 0.5, z + 0.65), (0.02, y1 - y0 - 2.0, 1.3), M["window"])
        for k in range(8):
            xm = x0 + 0.65 + k * (x1 - x0 - 1.3) / 7 + 0.6
            if xm < x1 - 0.5:
                ob.add_box((xm, y0 - 0.02, z + 0.65), (0.06, 0.03, 1.36), M["dark"])
    ob.add_box((x1 - 1.2, y0 - 0.01, 1.05), (1.0, 0.03, 2.1), M["dark"])      # door
    ob.add_box((x1 - 1.2, y0 - 0.03, 2.35), (0.35, 0.02, 0.14), M["exit_emit"])
    for k in range(3):                                                        # roof railing
        pass
    for x in [x0 + 0.1 + k * 1.2 for k in range(8)]:
        ob.add_box((x, y0 + 0.1, h + 0.7), (0.04, 0.04, 1.0), M["yellow"])
    ob.add_box(((x0 + x1) / 2, y0 + 0.1, h + 1.2), (x1 - x0, 0.05, 0.05), M["yellow"])
    ob.add_box(((x0 + x1) / 2, y0 + 0.1, h + 0.7), (x1 - x0, 0.04, 0.04), M["yellow"])
    # stair along the east face to the roof deck
    n, run_ = 22, 0.24
    ys = y0 + 0.3
    for k in range(n):
        ob.add_box((x1 + 0.6, ys + k * run_, (k + 1) * h / n - 0.02), (1.0, 0.26, 0.04), M["galv"])
    for xs in (x1 + 0.08, x1 + 1.12):
        ob.add_beam((xs, ys - 0.2, 0), (xs, ys + n * run_, h), 0.05, 0.2, M["yellow"])
    ob.add_beam((x1 + 1.12, ys - 0.2, 1.0), (x1 + 1.12, ys + n * run_, h + 1.0), 0.04, 0.04, M["yellow"])
    tag(ob.build("GZ_office", C), "interior")
    cb = MB()
    cb.add_box(((x0 + x1) / 2, (y0 + y1) / 2, h / 2), (x1 - x0, y1 - y0, h), M["office"])
    tag(cb.build("COL_office", C), "collision", collide=True)


def wall_graphics(M, C):
    t = L.text_mesh("PALLETIZER", 1.3, (-3.0, BY0 + 0.02, 8.0), (math.pi / 2, 0, math.pi), M["accent"], C,
                    extrude=0.005, name="GZ_wall_logo")
    tag(t, "shell")
    t = L.text_mesh("AUTONOMOUS PALLETIZING  ·  CELL 01", 0.42, (-3.0, BY0 + 0.02, 7.05), (math.pi / 2, 0, math.pi),
                    M["accent"], C, extrude=0.005, name="GZ_wall_tag")
    tag(t, "shell")
    g = MB()
    g.add_box((BX1 - 0.02, (EAST_DOOR[0] + EAST_DOOR[1]) / 2, EAST_DOOR[2] + 0.4), (0.03, EAST_DOOR[1] - EAST_DOOR[0] + 0.4, 0.8), M["door"])
    g.add_box((6.55, BY0 + 0.05, 2.45), (0.35, 0.03, 0.14), M["exit_emit"])
    for s in (-1, 1):
        g.add_box((BX1 - 0.1, EAST_DOOR[0 if s < 0 else 1] - s * 0.1, 0.6), (0.2, 0.2, 1.2), M["yellow"])
    tag(g.build("GZ_wall_details", C), "shell")
    cb = MB()
    for (c, s) in (((0, BY1 + WT / 2, 5), (48.5, WT, 10)), ((0, BY0 - WT / 2, 5), (48.5, WT, 10)),
                   ((BX1 + WT / 2, 0, 5), (WT, 30, 10)), ((BX0 - WT / 2, 0, 5), (WT, 30, 10))):
        cb.add_box(c, s, M["wall"])
    tag(cb.build("COL_walls", C), "collision", collide=True)


def build(cfg, seed=3):
    rng = random.Random(seed)
    root = L.coll("WAREHOUSE")
    C = L.coll("WH_export", root)
    Ccol = L.coll("WH_collision", root)
    M = materials()
    lamps = building(M, C)
    racking(M, C, rng)
    lanes = fence(M, C, cfg)
    floor_marks(M, C, cfg, lanes)
    infeed_line(M, C, cfg, rng)
    pallet_dispenser_and_wrapper(M, C, cfg, rng)
    control_station(M, C)
    docks(M, C, rng)
    vehicles(M, C, rng)
    office(M, C)
    wall_graphics(M, C)
    for o in list(C.objects):
        if o.get("gz_group") == "collision":
            C.objects.unlink(o)
            Ccol.objects.link(o)
            o.hide_render = True
            o.display_type = "WIRE"
    return M, lamps
