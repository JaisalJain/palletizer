#!/usr/bin/env python3
"""Screenshot of the RViz window (docs/rviz.png).

RViz must run as an X11 client (QT_QPA_PLATFORM=xcb, also under Xwayland). The window is
found with xdotool, resized, captured with xwd and converted to PNG here (no ImageMagick).
Usage: rviz_screenshot.py [out.png] [--wait-topic-count N]
"""

import os
import struct
import subprocess
import sys
import time

import cv2
import numpy as np


def find_window(name='RViz', timeout=60.0):
    end = time.time() + timeout
    while time.time() < end:
        r = subprocess.run(['xdotool', 'search', '--name', name], capture_output=True, text=True)
        ids = [w for w in r.stdout.split() if w]
        if ids:
            return ids[-1]
        time.sleep(1.0)
    raise RuntimeError(f'no window named {name}')


def read_xwd(data):
    """Minimal XWD (ZPixmap, 24/32 bpp TrueColor) reader -> RGB array."""
    hdr = struct.unpack('>25I', data[:100])
    if hdr[1] != 7:                                   # file version
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


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/palletizer_ws/docs/rviz.png')
    win = find_window()
    subprocess.run(['xdotool', 'windowsize', win, '1800', '1000'], check=False)
    time.sleep(3.0)
    data = subprocess.run(['xwd', '-silent', '-id', win], capture_output=True, check=True).stdout
    img = read_xwd(data)
    cv2.imwrite(out, img[:, :, ::-1])
    print(f'saved {out} ({img.shape[1]} x {img.shape[0]}), mean intensity {img.mean():.0f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
