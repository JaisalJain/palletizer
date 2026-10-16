"""Seamless textures for the warehouse (FFT-filtered noise is periodic, hence tileable).
    python make_textures.py <out_dir>        (numpy + pillow)"""
import os, sys, numpy as np
from PIL import Image, ImageDraw, ImageFont
out = sys.argv[1] if len(sys.argv) > 1 else "textures"
os.makedirs(out, exist_ok=True)
def pnoise(n, beta, seed):
    r = np.random.default_rng(seed)
    f = np.fft.fftfreq(n); fx, fy = np.meshgrid(f, f); k = np.sqrt(fx**2 + fy**2); k[0, 0] = 1
    spec = (r.normal(size=(n, n)) + 1j * r.normal(size=(n, n))) / k**beta; spec[0, 0] = 0
    x = np.real(np.fft.ifft2(spec)); return (x - x.mean()) / x.std()
N = 2048
big, mid, fine = pnoise(N, 1.6, 1), pnoise(N, 1.1, 2), pnoise(N, 0.5, 3)
spots = (pnoise(N, 0.9, 4) > 2.6).astype(float) * 0.5
v = 0.035 * big + 0.02 * mid + 0.012 * fine - 0.03 * spots
alb = np.clip(np.array([0.55, 0.55, 0.52])[None, None, :] * (1 + v[..., None]), 0, 1)  # mean = plain ground colour
Image.fromarray((alb ** (1 / 2.2) * 255).astype(np.uint8)).save(os.path.join(out, "concrete_albedo.png"))
Image.fromarray((np.clip(0.42 + 0.10 * big + 0.06 * mid + 0.2 * spots, 0.15, 0.9) * 255).astype(np.uint8)).save(os.path.join(out, "concrete_rough.png"))
fib, streak = pnoise(1024, 0.3, 5), pnoise(1024, 1.3, 6)
cb = np.array([0.62, 0.45, 0.28]) * (1 + 0.05 * fib[..., None] + 0.06 * streak[..., None])
Image.fromarray((np.clip(cb, 0, 1) ** (1 / 2.2) * 255).astype(np.uint8)).save(os.path.join(out, "cardboard.png"))
S = 512; y, x = np.mgrid[0:S, 0:S]; band = ((x + y) // (S // 4)) % 2
hz = np.where(band[..., None] == 0, np.array([0.95, 0.75, 0.05]), np.array([0.03, 0.03, 0.03]))
Image.fromarray((hz ** (1 / 2.2) * 255).astype(np.uint8)).save(os.path.join(out, "hazard.png"))
def font(s, bold=False):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if bold else ""), "arial%s.ttf" % ("bd" if bold else "")):
        try: return ImageFont.truetype(p, s)
        except OSError: pass
    return ImageFont.load_default()
W, H = 600, 400; im = Image.new("RGB", (W, H), (245, 245, 240)); d = ImageDraw.Draw(im)
d.text((24, 18), "PALLETIZER  ·  CELL 01", fill=(20, 20, 20), font=font(34, True))
d.text((24, 64), "SKU 400x300x300   LOT 26-09", fill=(40, 40, 40), font=font(22))
d.line((20, 100, W - 20, 100), fill=(20, 20, 20), width=3)
xx, r = 30, np.random.default_rng(11)
while xx < W - 30:
    w = int(r.integers(2, 7)); d.rectangle((xx, 120, xx + w, 300), fill=(10, 10, 10)); xx += w + int(r.integers(2, 6))
d.text((24, 315), "( 00 ) 3 4012345 000123456 7", fill=(20, 20, 20), font=font(22))
im.save(os.path.join(out, "label.png"))
W, H = 960, 640; im = Image.new("RGB", (W, H), (14, 22, 34)); d = ImageDraw.Draw(im)
d.rectangle((0, 0, W, 64), fill=(22, 40, 70)); d.text((24, 14), "PALLETIZER  ·  CELL 01", font=font(30, True), fill=(230, 238, 248))
d.rounded_rectangle((W - 230, 12, W - 20, 52), 10, fill=(20, 150, 70)); d.text((W - 205, 17), "● RUNNING", font=font(26, True), fill=(255, 255, 255))
for i, (t, val, u, c) in enumerate((("THROUGHPUT", "4.6", "picks/min", (90, 190, 255)), ("PALLET", "3", "in shift", (240, 240, 240)),
                                   ("PLACE ERROR", "0.6", "mm (cam)", (80, 230, 140)))):
    x0 = 24 + i * 313; d.rounded_rectangle((x0, 90, x0 + 285, 220), 12, fill=(26, 38, 56))
    d.text((x0 + 18, 104), t, font=font(20), fill=(150, 170, 195)); d.text((x0 + 18, 138), val, font=font(52, True), fill=c)
    d.text((x0 + 130, 164), u, font=font(20), fill=(150, 170, 195))
d.rounded_rectangle((24, 240, 936, 610), 12, fill=(26, 38, 56))
for k, s in enumerate(["Detect carton", "Plan placement", "Pick (suction)", "Weigh", "Place + verify", "Monitor stack"]):
    yy = 262 + k * 55; col = (80, 230, 140) if k < 3 else ((255, 200, 90) if k == 3 else (110, 125, 145))
    d.ellipse((50, yy + 4, 72, yy + 26), fill=col); d.text((88, yy), s, font=font(24, k == 3), fill=(230, 238, 248))
im.save(os.path.join(out, "hmi.png"))
print("textures ->", out)
