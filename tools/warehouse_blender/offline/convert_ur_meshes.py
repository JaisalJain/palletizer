"""UR10e visual meshes (ur_description, Collada) -> glTF for Blender 5 (which has no Collada importer).
    python convert_ur_meshes.py <ur_description>/meshes/ur10e/visual <out_dir>     (trimesh + pycollada)
glTF is Y-up by spec and Blender's importer converts Y-up -> Z-up, so the ROS Z-up geometry is
pre-rotated by -90 deg about X to land unchanged in Blender."""
import sys, os, glob, numpy as np, trimesh
src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
Rx = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
for f in sorted(glob.glob(os.path.join(src, "*.dae"))):
    s = trimesh.load(f, force="scene"); s.apply_transform(Rx)
    s.export(os.path.join(out, os.path.basename(f)[:-4] + ".glb")); print(os.path.basename(f))
