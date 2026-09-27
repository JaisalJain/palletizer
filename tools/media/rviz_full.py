#!/usr/bin/env python3
"""docs/media/rviz_full.png: the full RViz layout (3D view, both perception debug panels, the
status overlay) during a single-SKU run.

Needs a display (RViz is a GUI). RViz must be an X11 client (QT_QPA_PLATFORM=xcb, also under
Xwayland): the window is found with xdotool, resized and captured with xwd.
  rviz_full.py               launch a headless-Gazebo run with RViz, wait for a stack in
                             progress, capture, stop everything
  rviz_full.py --attach      only capture an RViz window that is already open
Options: --sku 400x300x300  --after 10 (placed boxes before the capture)
"""

import argparse
import os
import signal
import struct
import subprocess
import sys
import time

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402


def find_window(name='RViz', timeout=120.0):
    end = time.time() + timeout
    while time.time() < end:
        r = subprocess.run(['xdotool', 'search', '--name', name], capture_output=True, text=True)
        ids = [w for w in r.stdout.split() if w]
        if ids:
            return ids[-1]
        time.sleep(1.0)
    raise RuntimeError(f'no window named {name} (is RViz running with QT_QPA_PLATFORM=xcb?)')


def read_xwd(data):
    """Minimal XWD (ZPixmap, 24/32 bpp TrueColor) reader -> RGB array."""
    hdr = struct.unpack('>25I', data[:100])
    if hdr[1] != 7:                                       # file version
        hdr = struct.unpack('<25I', data[:100])
    header_size, fmt, width, height = hdr[0], hdr[2], hdr[4], hdr[5]
    byte_order, bpp, bpl = hdr[7], hdr[11], hdr[12]
    red_mask, green_mask, blue_mask = hdr[14], hdr[15], hdr[16]
    ncolors = hdr[19]
    off = header_size + ncolors * 12
    if fmt != 2 or bpp not in (24, 32):
        raise RuntimeError(f'unsupported XWD format {fmt} / {bpp} bpp')
    raw = np.frombuffer(data[off:off + bpl * height], dtype=np.uint8).reshape(height, bpl)
    nb = bpp // 8
    b = raw[:, :width * nb].reshape(height, width, nb).astype(np.uint32)
    if byte_order != 0:                                   # MSB first
        b = b[..., ::-1]
    px = b[..., 0] | (b[..., 1] << 8) | (b[..., 2] << 16)
    if nb == 4:
        px |= b[..., 3] << 24

    def chan(mask):
        shift = (mask & -mask).bit_length() - 1
        return ((px & mask) >> shift).astype(np.uint8)
    return np.dstack([chan(red_mask), chan(green_mask), chan(blue_mask)])


def capture(path):
    win = find_window()
    subprocess.run(['xdotool', 'windowsize', win, '1800', '1000'], check=False)
    time.sleep(4.0)
    data = subprocess.run(['xwd', '-silent', '-id', win], capture_output=True,
                          check=True).stdout
    img = Image.fromarray(read_xwd(data))
    if np.asarray(img).mean() < 5:
        raise RuntimeError('captured an empty (black) window')
    common.save_png(img, path, common.FULL_W)


def wait_placed(n, timeout):
    """Wait until /pallet/status reports n boxes placed on the current pallet."""
    import rclpy
    from palletizer_interfaces.msg import PalletStatus
    from rclpy.qos import DurabilityPolicy, QoSProfile
    rclpy.init()
    node = rclpy.create_node('rviz_full_wait')
    st = {}
    node.create_subscription(PalletStatus, '/pallet/status',
                             lambda m: st.update(placed=m.boxes_placed),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end = time.time() + timeout
    while time.time() < end and st.get('placed', 0) < n:
        rclpy.spin_once(node, timeout_sec=0.5)
    ok = st.get('placed', 0) >= n
    node.destroy_node()
    rclpy.shutdown()
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--attach', action='store_true')
    ap.add_argument('--sku', default='400x300x300')
    ap.add_argument('--after', type=int, default=10)
    ap.add_argument('--timeout', type=float, default=3600.0)
    args = ap.parse_args()
    out = common.out('rviz_full.png')
    if args.attach:
        capture(out)
        return 0
    kill = os.path.join(common.ROOT, 'src', 'palletizer_tools', 'test', 'kill_sim.sh')
    subprocess.run([kill], check=False)
    env = dict(os.environ, QT_QPA_PLATFORM='xcb')
    log = open(common.work('rviz_run.log'), 'w')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'palletizer_bringup', 'palletizer.launch.py', 'mode:=single_sku',
         f'sku:={args.sku}', 'gui:=false', 'rviz:=true', 'max_pallets:=1',
         f'results_dir:={common.work("rviz_results")}'],
        stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        if not wait_placed(args.after, args.timeout):
            raise RuntimeError(f'fewer than {args.after} boxes placed in {args.timeout} s')
        time.sleep(3.0)
        capture(out)
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        subprocess.run([kill], check=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
