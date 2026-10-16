#!/usr/bin/env python3
"""docs/media/warehouse_demo.gif: the real Gazebo simulation in the warehouse world, at 1x speed.

Record: a headless single-SKU run (world:=warehouse, sku 400x300x300, one pallet) with a gz
camera sensor spawned only for the recording (no GUI). Frames are saved with their sim-time
stamps in two windows: one full cycle (from a placement report to the next one) and the pallet
exchange (FULL until the new pallet is READY). Ground-truth pallet positions are logged per
frame to place the cuts; the robot is never touched.
Encode: each part is resampled on the sim clock at 12.5 fps (GIF delay 8 cs, exactly 1x), so
motion plays at real speed whatever the RTF was. Cuts only skip time (the sim clock is shown in
the corner): the cycle plays whole, the exchange shows the full pallet leaving and the new one
arriving. 960 px, ffmpeg palettegen / paletteuse, loops.

  record_gif.py                 record (~15 min wall) + encode
  record_gif.py --encode-only   re-encode the last recording (tools/media/_work/gif)
Options: --cycle-after 10 (placed boxes before the recorded cycle: one on layer 2)
         --seed 9 (the first pallet stops 9 mm farther from the robot than nominal: the
         full 8-per-layer pattern; a pallet that stops closer loses the slot in front of
         the pedestal, e.g. seed 1: 7 per layer)
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402

VIEW = ((-5.0, 3.6, 4.6), (0.3, -0.6, 0.8), 0.95)    # eye, target, hfov (world frame)
REC_W, REC_H, REC_HZ = 1280, 720, 15.0                 # camera (sim time)
FPS = 12.5                                             # GIF: 8 cs per frame = 1x
OUT_W = 960
EXIT_SHOW, ENTRY_SHOW = 4.0, 5.0                       # s of the pallet leaving / arriving
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_B = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
PICK = {'move to pre-grasp': 'pick: approach', 'transit via hub': 'pick: approach',
        'descend': 'pick: down to the top face', 'suction on': 'suction on',
        'lift': 'lift', 'weigh': 'weigh (F/T sensor)'}
PLACE = {'transit above target': 'carry level to the planned slot',
         'descend': 'place: down onto the stack', 'release': 'release', 'retreat': 'retreat'}


# ------------------------------------------------------------------------------ record
class Recorder:
    def __init__(self, sim, frames_dir):
        from cv_bridge import CvBridge
        from palletizer_interfaces.action import PickBox, PlaceBox
        from palletizer_interfaces.msg import CycleReport, PalletStatus
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image as ImageMsg
        self.sim, self.dir = sim, frames_dir
        self.cv = CvBridge()
        self.seg = None              # segment being recorded (None: not recording)
        self.frames = []             # {seg, t, file, pallets: {name: x}}
        self.events = []             # {t, text}: motion phases (captions)
        self.reports = []            # sim time of each placement report
        self.status = None
        self.last_t = 0.0
        self.lock = threading.Lock()
        sim.create_subscription(ImageMsg, '/media/gif/image', self._on_frame,
                                QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE))
        sim.create_subscription(CycleReport, '/orchestrator/cycle_report', self._on_cycle, 10)
        sim.create_subscription(PalletStatus, '/pallet/status',
                                lambda m: setattr(self, 'status', m),
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for action, names in ((PickBox, PICK), (PlaceBox, PLACE)):
            topic = '/motion/' + ('pick' if action is PickBox else 'place') + '/_action/feedback'
            sim.create_subscription(action.Impl.FeedbackMessage, topic,
                                    lambda m, n=names: self._on_phase(n, m.feedback.phase), 10)

    def _on_frame(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_t = t
        if self.seg is None:
            return
        import cv2
        name = f'{len(self.frames):05d}.jpg'
        cv2.imwrite(os.path.join(self.dir, name), self.cv.imgmsg_to_cv2(msg, 'bgr8'),
                    [cv2.IMWRITE_JPEG_QUALITY, 95])
        xs = {n: float(p[0][0]) for n, p in self.sim.pallet_poses.items()}
        with self.lock:
            self.frames.append({'seg': self.seg, 't': t, 'file': name, 'pallets': xs})

    def _on_cycle(self, msg):
        if msg.success and msg.outcome == 'placed':
            self.reports.append(msg.stamp.sec + msg.stamp.nanosec * 1e-9)

    def _on_phase(self, names, phase):
        if phase in names:
            self.events.append({'t': self.last_t, 'text': names[phase]})

    def wait(self, cond, timeout, what):
        if not self.sim.wait_for(cond, timeout):
            raise RuntimeError(f'timed out waiting for {what}')


def record(args, wdir):
    import rclpy
    from palletizer_interfaces.msg import PalletStatus
    from palletizer_orchestrator import sim_snapshot
    from palletizer_tools.sim_client import SimClient

    frames_dir = os.path.join(wdir, 'frames')
    os.makedirs(frames_dir, exist_ok=True)
    for f in os.listdir(frames_dir):
        os.remove(os.path.join(frames_dir, f))
    kill = os.path.join(common.ROOT, 'src', 'palletizer_tools', 'test', 'kill_sim.sh')
    subprocess.run([kill], check=False)
    log = open(os.path.join(wdir, 'run.log'), 'w')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'palletizer_bringup', 'palletizer.launch.py', 'mode:=single_sku',
         'sku:=400x300x300', 'world:=warehouse', 'gui:=false', 'rviz:=false', 'max_pallets:=1',
         'finish_with_next_box:=true', f'seed:={args.seed}',
         f'results_dir:={os.path.join(wdir, "results")}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    sim = SimClient('record_gif')
    rec = Recorder(sim, frames_dir)
    sim_snapshot.VIEWS.clear()
    sim_snapshot.VIEWS['gif'] = VIEW
    sim_snapshot.WIDTH, sim_snapshot.HEIGHT = REC_W, REC_H
    snap = sim_snapshot.Snapshot(sim, wdir, ['gif'], REC_HZ)
    try:
        sim.wait_ready(timeout=300.0)
        snap.spawn()
        rec.wait(lambda: rec.last_t > 0, 120.0, 'camera frames')
        print('record_gif: camera up', flush=True)
        # one full cycle: from a placement report to the next one (+1 s)
        k = args.cycle_after
        rec.wait(lambda: len(rec.reports) >= k, args.timeout, f'{k} placements')
        rec.seg = 'cycle'
        rec.wait(lambda: len(rec.reports) >= k + 1, 300.0, 'the recorded cycle')
        rec.wait(lambda: rec.last_t >= rec.reports[k] + 1.0, 60.0, 'the cycle tail')
        rec.seg = None
        print(f'record_gif: cycle recorded ({len(rec.frames)} frames)', flush=True)
        # the exchange: FULL ... new pallet READY (+1.5 s)
        rec.wait(lambda: rec.status is not None and rec.status.state == PalletStatus.FULL,
                 args.timeout, 'a full pallet')
        rec.seg = 'exchange'
        rec.wait(lambda: rec.status.state == PalletStatus.EXCHANGING, 300.0, 'EXCHANGING')
        rec.wait(lambda: rec.status.state in (PalletStatus.READY, PalletStatus.EMPTY), 900.0,
                 'the new pallet')
        t_ready = rec.last_t
        rec.wait(lambda: rec.last_t >= t_ready + 1.5, 60.0, 'the exchange tail')
        rec.seg = None
        print(f'record_gif: exchange recorded ({len(rec.frames)} frames)', flush=True)
        with rec.lock:
            meta = {'view': VIEW, 'fps': FPS, 'frames': rec.frames, 'events': rec.events,
                    'reports': rec.reports[:k + 1]}
        with open(os.path.join(wdir, 'meta.json'), 'w') as fh:
            json.dump(meta, fh, indent=0)
    finally:
        snap.remove()
        sim.shutdown()
        rclpy.try_shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        subprocess.run([kill], check=False)
        log.close()


# ------------------------------------------------------------------------------ encode
def moving(frames, name, eps=0.002):
    """Sim times at which pallet `name` moved since the previous frame."""
    out, prev = [], None
    for f in frames:
        x = f['pallets'].get(name)
        if x is not None and prev is not None and abs(x - prev) > eps:
            out.append(f['t'])
        prev = x if x is not None else prev
    return out


def parts(meta):
    """[(caption, t0, t1, frames)]: the cycle whole, the exchange cut to the pallet motions."""
    cyc = [f for f in meta['frames'] if f['seg'] == 'cycle']
    exc = [f for f in meta['frames'] if f['seg'] == 'exchange']
    out = [('cycle', cyc[0]['t'], cyc[-1]['t'], cyc)]
    first = exc[0]['pallets']
    old = next(n for n in first if moving(exc, n))                # the full pallet
    new = next(n for f in exc for n in f['pallets'] if n not in first)
    t_out = moving(exc, old)[0]
    t_in = moving(exc, new)
    out.append(('Full pallet out', t_out - 0.5, t_out + EXIT_SHOW, exc))
    out.append(('New pallet in', t_in[-1] - ENTRY_SHOW, min(t_in[-1] + 1.0, exc[-1]['t']), exc))
    return out


def caption_at(events, t, t0):
    text = 'carton waiting at the pick eye'
    for e in events:
        if t0 - 20.0 <= e['t'] <= t:
            text = e['text']
    return text


def overlay(img, title, caption, t):
    d = ImageDraw.Draw(img, 'RGBA')
    s = img.width / 1280
    f_t, f_c = ImageFont.truetype(FONT, round(17 * s)), ImageFont.truetype(FONT_B, round(26 * s))
    pad = round(12 * s)
    for xy, text, font in (((pad, pad), title, f_t),
                           ((pad, img.height - pad - round(36 * s)), caption, f_c),
                           (None, f'sim t {t:6.1f} s', f_t)):
        w, h = d.textbbox((0, 0), text, font=font)[2:]
        if xy is None:
            xy = (img.width - w - pad - round(8 * s), pad)
        x, y = xy
        d.rounded_rectangle((x - round(8 * s), y - round(6 * s), x + w + round(8 * s),
                             y + h + round(8 * s)), radius=round(6 * s), fill=(20, 20, 19, 190))
        d.text((x, y), text, font=font, fill=(255, 255, 255, 255))
    return img


def encode(wdir):
    meta = json.load(open(os.path.join(wdir, 'meta.json')))
    out_dir = os.path.join(wdir, 'out')
    os.makedirs(out_dir, exist_ok=True)
    for f in os.listdir(out_dir):
        os.remove(os.path.join(out_dir, f))
    title = 'Gazebo · world:=warehouse · real time (1×)'
    n = 0
    for name, t0, t1, frames in parts(meta):
        ts = np.array([f['t'] for f in frames])
        for t in np.arange(t0, t1, 1.0 / FPS):
            i = max(int(np.searchsorted(ts, t, side='right')) - 1, 0)   # last frame <= t
            img = Image.open(os.path.join(wdir, 'frames', frames[i]['file'])).convert('RGB')
            img = common.resize(img, OUT_W)
            cap = caption_at(meta['events'], t, t0) if name == 'cycle' else name
            overlay(img, title, cap, t).save(os.path.join(out_dir, f'{n:05d}.png'))
            n += 1
    gif = common.out('warehouse_demo.gif')
    pal = os.path.join(wdir, 'palette.png')
    src = ['-framerate', str(FPS), '-i', os.path.join(out_dir, '%05d.png')]
    subprocess.run(['ffmpeg', '-v', 'error', '-y'] + src +
                   ['-vf', 'palettegen=max_colors=256:stats_mode=full', pal], check=True)
    subprocess.run(['ffmpeg', '-v', 'error', '-y'] + src + ['-i', pal, '-lavfi',
                    'paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle',
                    '-loop', '0', gif], check=True)
    mb = os.path.getsize(gif) / 1e6
    print(f'saved {os.path.relpath(gif, common.ROOT)}: {n} frames, {n / FPS:.1f} s at '
          f'{FPS} fps, {mb:.1f} MB', flush=True)
    if mb > 8.0:
        print('WARNING: larger than 8 MB', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--encode-only', action='store_true')
    ap.add_argument('--cycle-after', type=int, default=10)
    ap.add_argument('--seed', type=int, default=9)
    ap.add_argument('--timeout', type=float, default=3600.0, help='wall [s] per wait')
    args = ap.parse_args()
    wdir = common.work('gif')
    if not args.encode_only:
        record(args, wdir)
    encode(wdir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
