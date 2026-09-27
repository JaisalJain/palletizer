#!/usr/bin/env python3
"""docs/media/perception_conveyor.png and perception_pallet.png: the two perception debug
images exactly as published on /perception/debug_image and /pallet_perception/debug_image
(saved by sim_snapshot at the hero moment, no window chrome).
Usage: perception_images.py [run name, default sku_380x280x220]"""

import os
import sys

from PIL import Image

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402

RUN = sys.argv[1] if len(sys.argv) > 1 else 'sku_380x280x220'


def main():
    for src, dst in (('conveyor_debug.png', 'perception_conveyor.png'),
                     ('pallet_debug.png', 'perception_pallet.png')):
        p = common.work(os.path.join('runs', RUN, src))
        if not os.path.exists(p):
            sys.exit(f'{p} missing: run tools/make_media.sh --sim first')
        # native camera resolution (640 x 480, 800 x 600): never upscaled
        common.save_png(Image.open(p), common.out(dst), common.GRID_W)


if __name__ == '__main__':
    main()
