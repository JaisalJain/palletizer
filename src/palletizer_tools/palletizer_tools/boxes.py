"""Box model generation shared by the spawner and the tests."""

import colorsys
import math

import numpy as np


def box_sdf(name, size, mass, rgba):
    sx, sy, sz = size
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    color = ' '.join(f'{c:.3f}' for c in rgba)
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{mass:.4f}</mass>
        <inertia>
          <ixx>{ixx:.6f}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{iyy:.6f}</iyy><iyz>0</iyz><izz>{izz:.6f}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def cardboard_rgba(rng):
    """Brown/kraft cardboard shades with some variation."""
    h = rng.uniform(0.07, 0.11)
    s = rng.uniform(0.35, 0.6)
    v = rng.uniform(0.55, 0.85)
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return (r, g, b, 1.0)


def random_box(rng, spawner_cfg):
    """Mixed mode (v1 distribution): uniformly random size and mass."""
    mx = spawner_cfg.get('mixed', spawner_cfg)
    size = tuple(float(rng.uniform(lo, hi)) for lo, hi in zip(mx['size_min'], mx['size_max']))
    mass = float(rng.uniform(*mx['mass_range']))
    yaw = float(rng.uniform(-spawner_cfg['yaw_range'], spawner_cfg['yaw_range']))
    return size, mass, yaw, cardboard_rgba(rng)


def parse_sku(key, spawner_cfg):
    """SKU key -> (external size (L, W, H) [m], mass range [kg]). Catalog keys give the
    catalog carton; any other "LxWxH" (mm) is taken literally with the default mass range."""
    key = key.strip()
    cat = spawner_cfg.get('catalog', {})
    if key in cat:
        e = cat[key]
        return tuple(float(v) for v in e['size']), tuple(e['mass'])
    try:
        dims = [float(v) / 1000.0 for v in key.lower().split('x')]
    except ValueError:
        dims = []
    if len(dims) != 3 or min(dims) <= 0:
        raise ValueError(f'unknown SKU "{key}" (not in the catalog, not "LxWxH" in mm)')
    return (max(dims[:2]), min(dims[:2]), dims[2]), tuple(spawner_cfg['default_mass'])


def parse_order(text, spawner_cfg):
    """ "400x300x300:30, 600x400x300:20" -> [(key, size, mass_range, count), ...]"""
    out = []
    for item in text.split(','):
        if not item.strip():
            continue
        key, _, n = item.partition(':')
        size, mass = parse_sku(key, spawner_cfg)
        out.append((key.strip(), size, mass, int(n) if n.strip() else 0))
    return out


def sku_box(rng, size, mass_range, spawner_cfg):
    """One carton of a SKU: random mass in the SKU's range, random orientation on the belt
    (length along or across the belt, +/- yaw_range)."""
    L, W, H = size
    s = (L, W, H) if rng.random() < 0.5 else (W, L, H)
    mass = float(rng.uniform(*mass_range))
    yaw = float(rng.uniform(-spawner_cfg['yaw_range'], spawner_cfg['yaw_range']))
    return s, mass, yaw, cardboard_rgba(rng)


def yaw_quaternion(yaw):
    """(x, y, z, w) quaternion for a rotation about z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def spawn_request(name, size, mass, rgba, xyz, yaw):
    from ros_gz_interfaces.srv import SpawnEntity
    req = SpawnEntity.Request()
    req.entity_factory.name = name
    req.entity_factory.allow_renaming = False
    req.entity_factory.sdf = box_sdf(name, size, mass, rgba)
    req.entity_factory.pose.position.x = float(xyz[0])
    req.entity_factory.pose.position.y = float(xyz[1])
    req.entity_factory.pose.position.z = float(xyz[2])
    q = yaw_quaternion(yaw)
    o = req.entity_factory.pose.orientation
    o.x, o.y, o.z, o.w = q
    return req


def x_extent(size, yaw):
    """Footprint extent along world x of a box with the given size and yaw."""
    return abs(size[0] * math.cos(yaw)) + abs(size[1] * math.sin(yaw))


def spawn_xyz(cell, size, yaw=0.0):
    """Spawn pose: rear edge just inside the upstream end of the belt."""
    cv = cell['conveyor']
    x = cv['x_start'] + cell['spawner']['rear_margin'] + x_extent(size, yaw) / 2.0
    return (x, cv['y'], cv['belt_top_z'] + size[2] / 2.0 + 0.005)


def poses_from_tf(msg, prefix='box_'):
    """{name: (xyz ndarray, quat xyzw ndarray)} from a /sim/box_poses TFMessage."""
    out = {}
    for t in msg.transforms:
        if t.child_frame_id.startswith(prefix):
            tr, r = t.transform.translation, t.transform.rotation
            out[t.child_frame_id] = (np.array([tr.x, tr.y, tr.z]),
                                     np.array([r.x, r.y, r.z, r.w]))
    return out
