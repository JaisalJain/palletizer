"""Shared helpers for the README media scripts (paths, dark theme, PNG optimization)."""

import os
import shutil
import subprocess

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
MEDIA = os.path.join(ROOT, 'docs', 'media')
RESULTS = os.path.join(ROOT, 'results')
WORK = os.path.join(os.path.dirname(__file__), '_work')   # scratch (gitignored)

FULL_W = 1600          # full-width images
GRID_W = 800           # images in the README's 2 x 2 grid

# Dark theme (validated reference palette, dark steps; see the dataviz notes in make_media.sh)
BG = '#1a1a19'
PANEL = '#232322'
GRID = '#383835'
TEXT = '#ffffff'
TEXT2 = '#c3c2b7'
MUTED = '#8a8980'
BLUE = '#3987e5'
ORANGE = '#d95926'
AQUA = '#199e70'


def out(name):
    os.makedirs(MEDIA, exist_ok=True)
    return os.path.join(MEDIA, name)


def work(name=''):
    os.makedirs(WORK, exist_ok=True)
    return os.path.join(WORK, name)


def mpl_dark():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'figure.facecolor': BG, 'axes.facecolor': BG, 'savefig.facecolor': BG,
        'axes.edgecolor': GRID, 'axes.labelcolor': TEXT2, 'axes.titlecolor': TEXT,
        'xtick.color': TEXT2, 'ytick.color': TEXT2, 'text.color': TEXT,
        'grid.color': GRID, 'grid.linewidth': 0.8,
        'font.family': 'DejaVu Sans', 'font.size': 12,
        'axes.titlesize': 14, 'axes.titleweight': 'bold',
        'axes.spines.top': False, 'axes.spines.right': False,
    })
    return plt


def resize(img, width):
    """PIL image scaled to `width` (never enlarged)."""
    from PIL import Image
    if img.width <= width:
        return img
    h = round(img.height * width / img.width)
    return img.resize((width, h), Image.LANCZOS)


def save_png(img, path, width=None, colors=256):
    """Save a PIL image as an optimized PNG: resize, then pngquant/optipng if installed,
    else a PIL palette quantization (dithered) when it does not hurt (<= `colors`)."""
    from PIL import Image
    img = img.convert('RGB')
    if width:
        img = resize(img, width)
    if shutil.which('pngquant'):
        img.save(path, optimize=True)
        subprocess.run(['pngquant', '--force', '--skip-if-larger', '--quality', '70-95',
                        '--output', path, path], check=False)
    elif colors:
        q = img.quantize(colors=colors, method=Image.Quantize.MEDIANCUT,
                         dither=Image.Dither.FLOYDSTEINBERG)
        q.save(path, optimize=True)
    else:
        img.save(path, optimize=True)
    if shutil.which('optipng'):
        subprocess.run(['optipng', '-quiet', '-o2', path], check=False)
    print(f'saved {os.path.relpath(path, ROOT)} ({Image.open(path).size[0]} x '
          f'{Image.open(path).size[1]}, {os.path.getsize(path) / 1024:.0f} kB)')
    return path


def save_fig(fig, path, width=FULL_W, dpi=100):
    """Matplotlib figure -> optimized PNG of `width` px."""
    import io
    from PIL import Image
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=dpi)
    buf.seek(0)
    return save_png(Image.open(buf), path, width)
