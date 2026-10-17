"""Keep-out check of the exported warehouse meshes against the cell (run with numpy + trimesh).
    python wh_verify.py <model_dir> <cell_config.yaml>
Samples every triangle's surface (plus all vertices) and counts points inside:
  robot     : cylinder r=1.85 m around the base axis (UR10e reach 1.30 + tool 0.15 + carried box)
  lane      : pallet lane swept volume, entry_x-0.65 .. exit_x+0.65, pallet +-0.46 m, <= 1.6 m
  conveyor  : sim conveyor + max box envelope
  eyes      : 3 cm around each photo-eye beam
  cam_roi   : pyramid from each camera to its perception ROI (the pixels perception reads)
  cam_view  : full camera frustum (info only; outside the ROIs perception ignores the pixels)
Floor-level geometry (z <= 5 mm: floor, paint, joints) is excluded: the plain world has a floor too.
"""
import sys, numpy as np, trimesh, yaml, os
mdir, cfgp = sys.argv[1], sys.argv[2]
c = yaml.safe_load(open(cfgp))
pts = []
for f in ("warehouse_shell.obj", "warehouse_interior.obj"):
    m = trimesh.load(os.path.join(mdir, "meshes", f), force="mesh", process=False, skip_materials=True)
    s, _ = trimesh.sample.sample_surface(m, 6_000_000, seed=1)
    pts += [m.vertices, s]
P = np.vstack(pts); P = P[P[:, 2] > 0.005]
x, y, z = P.T
cv, ex, st = c["conveyor"], c["exchanger"], c["pallet_station"]
res = {}
res["robot"] = np.sum((np.hypot(x, y) < 1.85) & (z < 2.8))
res["lane"] = np.sum((x > ex["entry_x"] - 0.65) & (x < ex["exit_x"] + 0.65) & (np.abs(y - ex["lane_y"]) < 0.46) & (z < 1.6))
res["conveyor"] = np.sum((x > cv["x_start"]) & (x < cv["x_end"] + 0.1) & (np.abs(y - cv["y"]) < 0.45) & (z < cv["belt_top_z"] + 0.45))
eye = cv["photo_eye"]
for zn in cv["zones"]:
    res["eye_" + zn["name"]] = np.sum((np.abs(x - zn["eye_x"]) < 0.03) & (np.abs(y - cv["y"]) < cv["belt_width"] / 2 + eye["offset"] + 0.03)
                                      & (np.abs(z - (cv["belt_top_z"] + eye["height"])) < 0.03))
def in_pyramid(cam, x0, x1, y0, y1, zlo):
    cx, cy, cz = cam
    t = (cz - z) / (cz - zlo)                 # 0 at camera, 1 at the ROI plane
    ok = (z < cz) & (z > 0.005) & (t > 0)
    xa, xb = cx + (x0 - cx) * t, cx + (x1 - cx) * t
    ya, yb = cy + (y0 - cy) * t, cy + (y1 - cy) * t
    return ok & (x >= np.minimum(xa, xb)) & (x <= np.maximum(xa, xb)) & (y >= np.minimum(ya, yb)) & (y <= np.maximum(ya, yb))
cam, pz = c["camera"], c["perception"]
roi = c["pick_zone"]["roi_x"]; half = cv["belt_width"] / 2 + pz["roi_margin_y"]
res["cam_roi_conveyor"] = np.sum(in_pyramid(cam["xyz"], roi[0], roi[1], cv["y"] - half, cv["y"] + half, 0.0))
pc = c["pallet_camera"]; sx, sy = st["center_xy"]; ax, ay = st["area"]
res["cam_roi_pallet"] = np.sum(in_pyramid(pc["xyz"], sx - ax / 2 - 0.1, sx + ax / 2 + 0.1, sy - ay / 2 - 0.1, sy + ay / 2 + 0.1, 0.0))
def frustum(camc, hw_axis, hfov, w, h):
    cx, cy, cz = camc["xyz"]; hh = np.tan(hfov / 2); hv = hh * h / w
    d = cz - z; ok = (d > 0) & (d < camc["far"])
    u, v = (y - cy, x - cx) if hw_axis == "y" else (x - cx, y - cy)
    return ok & (np.abs(u) < d * hh) & (np.abs(v) < d * hv)
fc = frustum(cam, "y", cam["hfov"], cam["width"], cam["height"])
fp = frustum(pc, "x", pc["hfov"], pc["width"], pc["height"])
res["cam_view_conveyor(info)"] = np.sum(fc); res["cam_view_pallet(info)"] = np.sum(fp)
print(f"{len(P):,} surface points above the floor")
for k, v in res.items():
    flag = "" if "info" in k else ("OK" if v == 0 else "FAIL")
    print(f"  {k:26s} {int(v):>9,d}  {flag}")
if np.sum(fc):
    q = P[fc]; print("  conveyor-camera frustum hits (info): x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]" % (*q[:,0].min(0).__array__().reshape(1), q[:,0].max(), q[:,1].min(), q[:,1].max(), q[:,2].min(), q[:,2].max()))
sys.exit(1 if any(v for k, v in res.items() if "info" not in k) else 0)
