#!/usr/bin/env python3
"""docs/media/hero_cell.png: the cell mid-run (a stack in progress, boxes on the conveyor, the
arm carrying a box), from a snapshot run (`make_media.sh --sim`: sku_runs --snapshot).
Usage: hero_cell.py [run name, default sku_380x280x220]"""

import os
import sys

from PIL import Image

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402

RUN = sys.argv[1] if len(sys.argv) > 1 else 'sku_380x280x220'


def main():
    src = common.work(os.path.join('runs', RUN, 'hero.png'))
    if not os.path.exists(src):
        sys.exit(f'{src} missing: run tools/make_media.sh --sim first')
    img = Image.open(src)
    w, h = img.size                     # 1920 x 1080; trim the empty floor at the bottom
    img = img.crop((0, 0, w, int(h * 0.94)))
    common.save_png(img, common.out('hero_cell.png'), common.FULL_W, colors=0)  # lossless render


if __name__ == '__main__':
    main()
