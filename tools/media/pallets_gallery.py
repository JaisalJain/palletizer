#!/usr/bin/env python3
"""docs/media/pallets_gallery.png: finished pallets from real runs, captured by sim_snapshot
when /pallet/status turned FULL (just before the exchange). Captions from the run's CSVs.
Usage: pallets_gallery.py [run names ...]"""

import csv
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402

RUNS = sys.argv[1:] or ['sku_600x400x300', 'sku_520x320x250', 'sku_450x350x300',
                        'sku_380x280x220']
CELL_W = 800
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_B = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def caption(run):
    rows = list(csv.DictReader(open(common.work(os.path.join('runs', 'results',
                                                               f'{run}_pallets.csv')))))
    r = [x for x in rows if x['pallet'] == '0'][0]
    sku = run.replace('sku_', '').replace('x', '×')
    return (f'{sku} mm',
            f'{r["boxes"]} boxes · {r["layers"]} layers · fill {float(r["fill_percent"]):.0f} % · '
            f'{float(r["picks_per_min"]):.1f} picks/min · {r["end_reason"]}')


def main():
    font = ImageFont.truetype(FONT, 19)
    font_b = ImageFont.truetype(FONT_B, 24)
    cells = []
    for run in RUNS:
        p = common.work(os.path.join('runs', run, 'full_pallet.png'))
        if not os.path.exists(p):
            sys.exit(f'{p} missing: run tools/make_media.sh --sim first')
        img = Image.open(p).convert('RGB')
        w, h = img.size
        img = img.crop((int(w * 0.08), 0, int(w * 0.92), int(h * 0.96)))   # centre the stack
        img = common.resize(img, CELL_W)
        title, sub = caption(run)
        cell = Image.new('RGB', (CELL_W, img.height + 78), common.BG)
        cell.paste(img, (0, 0))
        d = ImageDraw.Draw(cell)
        d.text((16, img.height + 10), title, font=font_b, fill=common.TEXT)
        d.text((16, img.height + 44), sub, font=font, fill=common.TEXT2)
        cells.append(cell)
    cols = 2
    rows = (len(cells) + cols - 1) // cols
    ch = max(c.height for c in cells)
    gap = 6
    sheet = Image.new('RGB', (cols * CELL_W + (cols - 1) * gap, rows * ch + (rows - 1) * gap),
                      common.BG)
    for i, c in enumerate(cells):
        sheet.paste(c, ((i % cols) * (CELL_W + gap), (i // cols) * (ch + gap)))
    common.save_png(sheet, common.out('pallets_gallery.png'), common.FULL_W)


if __name__ == '__main__':
    main()
