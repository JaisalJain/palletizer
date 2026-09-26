"""Drawing helpers for the perception debug images (no ROS). Images are RGB uint8."""

import cv2
import numpy as np


def tint(img, mask, color, alpha=0.35):
    if mask is None or not mask.any():
        return img
    c = np.array(color, dtype=np.float32)
    img[mask] = (img[mask].astype(np.float32) * (1 - alpha) + c * alpha).astype(np.uint8)
    return img


def poly(img, pts, color, thick=2, closed=True):
    p = np.round(np.asarray(pts)).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(img, [p], closed, color, thick, cv2.LINE_AA)


def fill_poly(img, pts, color, alpha=0.3):
    over = img.copy()
    p = np.round(np.asarray(pts)).astype(np.int32).reshape(-1, 1, 2)
    cv2.fillPoly(over, [p], color, cv2.LINE_AA)
    cv2.addWeighted(over, alpha, img, 1 - alpha, 0, img)


def crosshair(img, pt, color, size=10, thick=2):
    x, y = int(round(pt[0])), int(round(pt[1]))
    cv2.line(img, (x - size, y), (x + size, y), color, thick, cv2.LINE_AA)
    cv2.line(img, (x, y - size), (x, y + size), color, thick, cv2.LINE_AA)
    cv2.circle(img, (x, y), size // 2, color, 1, cv2.LINE_AA)


def arrow(img, p0, p1, color, thick=2):
    cv2.arrowedLine(img, tuple(int(round(v)) for v in p0), tuple(int(round(v)) for v in p1),
                    color, thick, cv2.LINE_AA, tipLength=0.2)


def contour(img, mask, color, thick=1):
    if mask is None or not mask.any():
        return
    cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img, cs, -1, color, thick, cv2.LINE_AA)


def text_block(img, lines, org=(8, 8), color=(255, 255, 255), scale=0.5, bg=(0, 0, 0),
               alpha=0.6):
    font = cv2.FONT_HERSHEY_SIMPLEX
    sizes = [cv2.getTextSize(t, font, scale, 1)[0] for t in lines]
    if not sizes:
        return
    h = sizes[0][1] + 8
    w = max(s[0] for s in sizes) + 12
    x0, y0 = org
    over = img.copy()
    cv2.rectangle(over, (x0, y0), (x0 + w, y0 + h * len(lines) + 4), bg, -1)
    cv2.addWeighted(over, alpha, img, 1 - alpha, 0, img)
    for i, t in enumerate(lines):
        c = color[i] if isinstance(color, list) else color
        cv2.putText(img, t, (x0 + 6, y0 + (i + 1) * h - 2), font, scale, c, 1, cv2.LINE_AA)


def heightmap_inset(H, max_h, cell_px=2):
    """Coloured height map (rows = v, cols = u), NaN = grey; turbo colormap 0..max_h.
    Row 0 (v = 0, the robot-side edge) at the top: the same orientation as the pallet camera
    image (robot at the top, +u to the right)."""
    Hn = np.clip(np.nan_to_num(H, nan=0.0) / max(max_h, 1e-6), 0, 1)
    col = cv2.applyColorMap((Hn * 255).astype(np.uint8), cv2.COLORMAP_TURBO)[..., ::-1].copy()
    col[np.isnan(H)] = (90, 90, 90)
    col[np.nan_to_num(H, nan=1.0) <= 0.003] = (35, 35, 35)
    return cv2.resize(col, (H.shape[1] * cell_px, H.shape[0] * cell_px),
                      interpolation=cv2.INTER_NEAREST)


def paste(img, inset, org, label=None):
    x, y = org
    h, w = inset.shape[:2]
    h = min(h, img.shape[0] - y)
    w = min(w, img.shape[1] - x)
    img[y:y + h, x:x + w] = inset[:h, :w]
    cv2.rectangle(img, (x - 1, y - 1), (x + w, y + h), (255, 255, 255), 1)
    if label:
        cv2.putText(img, label, (x, y + h + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
