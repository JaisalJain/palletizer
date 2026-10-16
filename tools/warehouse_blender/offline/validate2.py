"""Independent per-frame check of anim2.json: collisions (FCL, UR10e collision meshes), joint
speeds, tool tilt, carried-box rotation rate.   python validate2.py anim2.json cell_config.json"""
import json, sys, numpy as np, collections, math
import cellcheck as CC
from urkin import tcp
A = json.load(open(sys.argv[1])); cfg = json.load(open(sys.argv[2] if len(sys.argv) > 2 else 'cell_config.json'))
env = CC.static_env(cfg); N = A["frames"]; fps = A["fps"]; DECK = 0.144
Q = np.array(A["q"])
hits = collections.Counter(); first = {}; worst = collections.defaultdict(float)
min_clear = collections.defaultdict(lambda: 9.0)
tilt = []; held_rot = []
for f in range(N):
    boxes = []
    for n, o in A["objects"].items():
        k = f - o["first"]
        if k < 0 or k >= len(o["track"]) or o["track"][k] is None: continue
        x, y, z, yaw = o["track"][k]
        boxes.append((n, (x, y, DECK / 2) if o["kind"] == "pallet" else (x, y, z), o["size"], yaw))
    h, T = CC.check_frame(Q[f], None, boxes, env)
    for a, b, d in h:
        # held box vs cup: the cup rests 3 mm above the top of the box it carries (not a collision)
        key = tuple(sorted((a, b)))
        hits[key] += 1; first.setdefault(key, f); worst[key] = max(worst[key], d)
    tilt.append(math.degrees(math.acos(max(-1, min(1, -T[2, 2])))))
print(f"frames {N}: collision pairs (penetration > 2 mm):", "NONE" if not hits else "")
for k, n in hits.most_common(20):
    print(f"   {k[0]} x {k[1]}: {n} frames, first {first[k]/fps:.2f}s, max {worst[k]*1000:.1f} mm")
qd = np.abs(np.diff(Q, axis=0)) * fps
print("max joint speed [deg/s]:", np.degrees(qd.max(0)).round(0).tolist(), " limits: 120 120 180 180 180 180")
print(f"max tool tilt from vertical: {max(tilt):.2f} deg")
# box yaw rate while carried
rates = []
for n, o in A["objects"].items():
    tr = o["track"]
    for k in range(1, len(tr)):
        if tr[k] and tr[k-1]:
            dy = abs(math.remainder(tr[k][3] - tr[k-1][3], 2 * math.pi)) * fps
            if dy > 1e-3: rates.append(math.degrees(dy))
print(f"box rotation rate (any frame): max {max(rates):.0f} deg/s, 95th pct {np.percentile(rates,95):.0f} deg/s")
jumps=[(n,k) for n,o in A["objects"].items() for k in range(1,len(o["track"])) if o["track"][k] and o["track"][k-1] and abs(o["track"][k][3]-o["track"][k-1][3])>0.5]
print("yaw discontinuities:", jumps[:5] if jumps else "none")
