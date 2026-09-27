#!/usr/bin/env python3
"""docs/media/exchange_sequence.png: four frames of the pallet exchange from a snapshot run
(lane camera, sim_snapshot exchange/frames.json): the full pallet at the station, leaving,
the new pallet arriving, the new pallet in place. Frames are picked by the ground-truth pallet
x in frames.json (media only).
Usage: exchange_sequence.py [run name, default sku_600x400x300]"""

import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402

RUN = sys.argv[1] if len(sys.argv) > 1 else 'sku_600x400x300'
CELL_W = 800
FONT_B = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def pick(frames):
    x0 = frames[0]['pallet_x']
    full = min(x0, key=lambda n: abs(x0[n]))                  # the pallet at the station

    def new_x(f):
        others = [x for n, x in f['pallet_x'].items() if n != full]
        return others[0] if others else None
    leaving = next((f for f in frames if f['pallet_x'].get(full, 99) > 1.3), frames[len(frames) // 3])
    arriving = next((f for f in frames if new_x(f) is not None and -1.0 < new_x(f) < -0.6),
                    frames[2 * len(frames) // 3])
    return [(frames[0], '1  pallet FULL: arm parked, exchange starts'),
            (leaving, '2  full pallet leaves on the pallet conveyor'),
            (arriving, '3  a new pallet comes in'),
            (frames[-1], '4  new pallet delivered → measured by the camera')]


def main():
    d = common.work(os.path.join('runs', RUN, 'exchange'))
    fj = os.path.join(d, 'frames.json')
    if not os.path.exists(fj):
        sys.exit(f'{fj} missing: run tools/make_media.sh --sim first')
    frames = json.load(open(fj))
    font = ImageFont.truetype(FONT_B, 22)
    cells = []
    for f, label in pick(frames):
        img = Image.open(os.path.join(common.work(os.path.join('runs', RUN)), f['file']))
        w, h = img.size
        img = img.convert('RGB').crop((int(w * 0.2), int(h * 0.08), w, int(h * 0.93)))  # lane
        img = common.resize(img, CELL_W)
        dr = ImageDraw.Draw(img, 'RGBA')
        tw = dr.textlength(label, font=font)
        dr.rounded_rectangle((12, 12, 12 + tw + 24, 54), radius=8, fill=(26, 26, 25, 215))
        dr.text((24, 20), label, font=font, fill=common.TEXT)
        cells.append(img)
    gap = 6
    ch = cells[0].height
    sheet = Image.new('RGB', (2 * CELL_W + gap, 2 * ch + gap), common.BG)
    for i, c in enumerate(cells):
        sheet.paste(c, ((i % 2) * (CELL_W + gap), (i // 2) * (ch + gap)))
    common.save_png(sheet, common.out('exchange_sequence.png'), common.FULL_W, colors=0)


if __name__ == '__main__':
    main()
