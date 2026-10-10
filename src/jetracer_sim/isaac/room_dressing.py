"""Furnish the JetRacer room so visual SLAM can track in it.

The scene's room is four bare plaster walls under a strong sun-like light:
from the car's camera (5 cm above the floor) most views are a flat grey wall
over a grey floor, and ORB finds nothing stable to track. dress() adds, at
start-up and only in memory (the .usd file is never changed):

  - textured overlays on the four walls (posters, whiteboard, stencilled zone
    letters, skirting, outlets, a door): each wall looks different, so places
    are recognisable for relocalisation and loop closure
  - a textured floor overlay (concrete with stains, tape and scuffs) over the
    repeating checker
  - shelves with books and boxes, cabinets, crates and box stacks along the
    walls (static colliders, so the car bumps into them and depth sees them)
  - a ceiling with panel lights; the scene's sun light is switched off

Textures are drawn here with PIL from fixed seeds (the room is the same every
run) and cached in ~/.cache/jetracer_sim/textures_v1.
"""
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

ROOT = '/World/RoomDressing'
TEX_DIR = os.path.expanduser('~/.cache/jetracer_sim/textures_v1')
HALF = 2.975            # inner faces of the walls (m)
HEIGHT = 2.0            # wall height = ceiling (m)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


# ---------------------------------------------------------------- textures

def _font(size, bold=False):
    try:
        return ImageFont.truetype(FONT_BOLD if bold else FONT, size)
    except OSError:
        return ImageFont.load_default()


def _color(rng, lo=0, hi=255):
    return tuple(int(v) for v in rng.integers(lo, hi, 3))


def _mottle(rng, w, h, cell, amp):
    """Smooth random brightness variation (stains, uneven paint): a few
    random values per `cell` pixels, scaled up with bicubic smoothing."""
    small = rng.normal(0, amp, (max(2, h // cell), max(2, w // cell))).astype(np.float32)
    return np.asarray(Image.fromarray(small).resize((w, h), Image.BICUBIC))


def _surface(rng, w, h, base, mottle=18, grain=4):
    a = np.empty((h, w, 3), np.float32)
    a[:] = base
    a += _mottle(rng, w, h, 160, mottle)[..., None]
    a += _mottle(rng, w, h, 30, mottle * 0.5)[..., None]
    a += rng.normal(0, grain, (h, w, 1))
    return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))


def _words(rng, n):
    syll = ['ka', 'ro', 'mi', 'te', 'lu', 'san', 'or', 'bix', 'del', 'ta', 'ven', 'po', 'gra', 'ne', 'ul']
    return ' '.join(''.join(rng.choice(syll, rng.integers(1, 4))) for _ in range(n))


def _poster(rng, w, h):
    """One poster: a random design in a white frame."""
    kind = rng.choice(['blocks', 'circles', 'text', 'chart', 'photo', 'map', 'stripes'])
    img = Image.new('RGB', (w, h), _color(rng, 180, 255))
    d = ImageDraw.Draw(img)
    if kind == 'blocks':
        for _ in range(rng.integers(6, 14)):
            x0, y0 = rng.integers(0, w), rng.integers(0, h)
            d.rectangle([x0, y0, x0 + rng.integers(w // 8, w // 2), y0 + rng.integers(h // 8, h // 2)],
                        fill=_color(rng), outline=(0, 0, 0), width=max(2, w // 60))
    elif kind == 'circles':
        for _ in range(rng.integers(8, 20)):
            r = rng.integers(w // 20, w // 4)
            x0, y0 = rng.integers(0, w), rng.integers(0, h)
            d.ellipse([x0 - r, y0 - r, x0 + r, y0 + r], fill=_color(rng), outline=_color(rng, 0, 80),
                      width=max(1, w // 80))
    elif kind == 'text':
        y = h // 20
        d.text((w // 20, y), _words(rng, 2).upper(), font=_font(max(10, h // 7), True), fill=_color(rng, 0, 120))
        y += h // 5
        while y < h - h // 12:
            d.text((w // 20, y), _words(rng, rng.integers(3, 7)), font=_font(max(8, h // 16)), fill=(20, 20, 20))
            y += h // 11
    elif kind == 'chart':
        m = w // 10
        d.line([m, h - m, w - m, h - m], fill=(0, 0, 0), width=max(2, w // 100))
        d.line([m, m, m, h - m], fill=(0, 0, 0), width=max(2, w // 100))
        n = rng.integers(4, 9)
        bw = (w - 2 * m) // (n * 2)
        for i in range(n):
            top = rng.integers(m, h - 2 * m)
            d.rectangle([m + bw * (2 * i + 1), top, m + bw * (2 * i + 2), h - m], fill=_color(rng, 0, 200))
        d.text((m, m // 4), _words(rng, 2), font=_font(max(8, h // 14), True), fill=(0, 0, 0))
    elif kind == 'photo':
        a = np.zeros((h, w, 3), np.float32)
        for c in range(3):
            a[..., c] = 128 + _mottle(rng, w, h, max(8, w // 6), 70)
        img = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))
        d = ImageDraw.Draw(img)
        for _ in range(rng.integers(3, 8)):
            pts = [(int(rng.integers(0, w)), int(rng.integers(h // 3, h))) for _ in range(3)]
            d.polygon(pts, fill=_color(rng, 0, 160))
    elif kind == 'map':
        for _ in range(rng.integers(5, 10)):
            pts = [(int(rng.integers(0, w)), int(rng.integers(0, h))) for _ in range(rng.integers(3, 6))]
            d.polygon(pts, fill=_color(rng, 120, 230))
        for _ in range(rng.integers(6, 14)):
            pts = [(int(rng.integers(0, w)), int(rng.integers(0, h))) for _ in range(rng.integers(2, 5))]
            d.line(pts, fill=_color(rng, 0, 90), width=max(2, w // 70))
    else:  # stripes, of uneven widths so they do not repeat
        x = -h
        while x < w:
            sw = int(rng.integers(w // 30, w // 8))
            d.polygon([(x, h), (x + sw, h), (x + sw + h, 0), (x + h, 0)], fill=_color(rng))
            x += sw + int(rng.integers(0, w // 12))
    framed = Image.new('RGB', (w + 2 * max(4, w // 25), h + 2 * max(4, w // 25)), (245, 245, 240))
    framed.paste(img, (max(4, w // 25), max(4, w // 25)))
    return framed


def _whiteboard(rng, w, h):
    img = Image.new('RGB', (w, h), (238, 240, 242))
    d = ImageDraw.Draw(img)
    for _ in range(rng.integers(10, 18)):
        col = [(20, 40, 160), (170, 20, 20), (20, 120, 40), (20, 20, 20)][rng.integers(0, 4)]
        if rng.random() < 0.5:
            d.text((int(rng.integers(0, w * 0.8)), int(rng.integers(0, h * 0.85))), _words(rng, rng.integers(1, 4)),
                   font=_font(int(rng.integers(h // 16, h // 7))), fill=col)
        else:
            x, y = rng.integers(0, w), rng.integers(0, h)
            pts = [(int(x), int(y))]
            for _ in range(rng.integers(3, 9)):
                x, y = x + rng.integers(-w // 6, w // 6), y + rng.integers(-h // 5, h // 5)
                pts.append((int(x), int(y)))
            d.line(pts, fill=col, width=max(2, w // 200))
    d.rectangle([0, 0, w - 1, h - 1], outline=(150, 150, 155), width=max(4, w // 80))
    return img


def wall_texture(seed, ppm=320):
    """A 6 m x 2 m wall at `ppm` pixels per metre; image top = ceiling."""
    rng = np.random.default_rng(seed)
    W, H = int(6 * ppm), int(2 * ppm)
    base = [(205, 200, 185), (190, 205, 210), (215, 195, 190), (200, 210, 190)][seed % 4]
    img = _surface(rng, W, H, base, mottle=10, grain=3)
    d = ImageDraw.Draw(img)

    def y_of(z):   # height above the floor (m) -> image row
        return int((HEIGHT - z) * ppm)

    # Lower band (what the 5 cm high camera sees most): skirting, a painted
    # dado band with stencilled zone letters, outlets.
    d.rectangle([0, y_of(0.10), W, H], fill=(60, 45, 35) if seed % 2 else (40, 40, 45))
    for _ in range(int(rng.integers(30, 60))):     # scuffs on the skirting
        x = int(rng.integers(0, W))
        d.line([x, y_of(0.09), x + int(rng.integers(5, 40)), y_of(0.02)], fill=_color(rng, 90, 160), width=2)
    band = [(70, 110, 150), (150, 80, 60), (90, 130, 80), (120, 100, 150)][seed % 4]
    d.rectangle([0, y_of(0.75), W, y_of(0.10)], fill=band)
    d.line([0, y_of(0.75), W, y_of(0.75)], fill=(30, 30, 30), width=max(3, ppm // 60))
    x = int(rng.integers(ppm // 4, ppm))
    big = _font(int(0.45 * ppm), True)
    while x < W - ppm // 2:
        label = f'{"ABCD"[seed % 4]}{int(rng.integers(1, 99))}'
        d.text((x, y_of(0.68)), label, font=big, fill=(245, 245, 235))
        x += int(rng.uniform(1.0, 1.8) * ppm)
    for _ in range(int(rng.integers(3, 6))):         # outlets
        x = int(rng.integers(0, W - ppm // 5))
        d.rectangle([x, y_of(0.36), x + ppm // 8, y_of(0.20)], fill=(235, 235, 230), outline=(80, 80, 80), width=2)
        d.ellipse([x + ppm // 40, y_of(0.31), x + ppm // 40 + ppm // 16, y_of(0.25)], fill=(30, 30, 30))
    for _ in range(int(rng.integers(2, 4))):         # hazard tape patches
        x0 = int(rng.integers(0, W - ppm))
        L = int(rng.uniform(0.3, 0.8) * ppm)
        y0, y1 = y_of(0.30 + rng.uniform(0, 0.15)), y_of(0.18)
        d.rectangle([x0, y0, x0 + L, y1], fill=(240, 200, 20))
        sx = x0
        while sx < x0 + L:
            sw = int(rng.integers(ppm // 25, ppm // 12))
            d.polygon([(sx, y1), (sx + sw, y1), (sx + sw + (y1 - y0), y0), (sx + (y1 - y0), y0)], fill=(20, 20, 20))
            sx += 2 * sw + int(rng.integers(0, ppm // 20))
        d.rectangle([x0, y0, x0 + L, y1], outline=(20, 20, 20), width=2)

    # Upper part: posters, a whiteboard or notice board, and on wall 0 a door.
    taken = []

    def free(x0, x1):
        return all(x1 < a - ppm // 10 or x0 > b + ppm // 10 for a, b in taken)

    if seed % 4 == 0:
        dx = int(rng.uniform(1.0, 4.5) * ppm)
        d.rectangle([dx, y_of(1.95), dx + int(0.9 * ppm), y_of(0.0)], fill=(110, 70, 40), outline=(40, 30, 20), width=8)
        d.rectangle([dx + 20, y_of(1.85), dx + int(0.9 * ppm) - 20, y_of(1.2)], outline=(60, 40, 25), width=6)
        d.rectangle([dx + 20, y_of(1.0), dx + int(0.9 * ppm) - 20, y_of(0.15)], outline=(60, 40, 25), width=6)
        d.ellipse([dx + int(0.75 * ppm), y_of(1.05), dx + int(0.82 * ppm), y_of(0.98)], fill=(200, 190, 120))
        d.rectangle([dx + int(0.2 * ppm), y_of(2.0) + 4, dx + int(0.7 * ppm), y_of(1.96)], fill=(20, 140, 60))
        taken.append((dx, dx + int(0.9 * ppm)))
    bw = int(rng.uniform(1.0, 1.6) * ppm)
    for _ in range(50):
        bx = int(rng.integers(0, W - bw))
        if free(bx, bx + bw):
            board = _whiteboard(rng, bw, int(0.8 * ppm))
            img.paste(board, (bx, y_of(1.75)))
            taken.append((bx, bx + bw))
            break
    for _ in range(200):
        pw, ph = int(rng.uniform(0.35, 0.9) * ppm), int(rng.uniform(0.35, 0.9) * ppm)
        px = int(rng.integers(0, W - pw))
        if not free(px, px + pw):
            continue
        p = _poster(rng, pw, ph)
        top = rng.uniform(0.8 + ph / ppm, 1.9)
        img.paste(p, (px, y_of(top)))
        taken.append((px, px + p.width))
    return img


def floor_texture(seed=7, ppm=512):
    """6 m x 6 m of concrete: stains, cracks, tape lines, tyre marks."""
    rng = np.random.default_rng(seed)
    N = int(6 * ppm)
    img = _surface(rng, N, N, (150, 146, 138), mottle=16, grain=5)
    d = ImageDraw.Draw(img)
    for _ in range(80):                                   # dark and light stains
        r = int(rng.integers(ppm // 40, ppm // 6))
        x, y = int(rng.integers(0, N)), int(rng.integers(0, N))
        c = int(rng.choice([95, 110, 175, 185]))
        d.ellipse([x - r, y - int(r * rng.uniform(0.4, 1.0)), x + r, y + r], fill=(c, c - 4, c - 10))
    img = img.filter(ImageFilter.GaussianBlur(2))
    d = ImageDraw.Draw(img)
    for _ in range(N * N // 300):                         # grit and pebbles, 5 mm to 3 cm:
        r = int(rng.integers(2, 6)) if rng.random() < 0.85 else int(rng.integers(6, 12))   # what the low
        x, y = int(rng.integers(0, N)), int(rng.integers(0, N))                             # camera sees
        c = int(rng.choice([55, 75, 95, 190, 215]))                                          # close up
        d.ellipse([x - r, y - int(r * rng.uniform(0.5, 1.0)), x + r, y + r], fill=(c, c - 3, c - 8))
    for _ in range(40):                                   # cracks
        x, y = int(rng.integers(0, N)), int(rng.integers(0, N))
        pts = [(x, y)]
        for _ in range(int(rng.integers(3, 10))):
            x, y = x + int(rng.integers(-ppm // 5, ppm // 5)), y + int(rng.integers(-ppm // 5, ppm // 5))
            pts.append((x, y))
        d.line(pts, fill=(60, 58, 55), width=int(rng.integers(2, 5)))
    for _ in range(14):                                   # tape strips, arrows, markings
        col = [(230, 190, 30), (230, 230, 225), (40, 110, 200), (200, 50, 40)][int(rng.integers(0, 4))]
        x, y = int(rng.integers(0, N)), int(rng.integers(0, N))
        L, wd = int(rng.uniform(0.3, 1.2) * ppm), int(0.05 * ppm)
        if rng.random() < 0.5:
            d.rectangle([x, y, x + L, y + wd], fill=col)
        else:
            d.rectangle([x, y, x + wd, y + L], fill=col)
    for _ in range(10):
        x, y, s = int(rng.integers(0, N)), int(rng.integers(0, N)), int(rng.uniform(0.15, 0.3) * ppm)
        d.polygon([(x, y), (x + s, y + s // 2), (x, y + s), (x + s // 3, y + s // 2)], fill=(235, 235, 230))
    for _ in range(12):
        d.text((int(rng.integers(0, N)), int(rng.integers(0, N))), f'{"ABCDEF"[int(rng.integers(0, 6))]}{int(rng.integers(1, 40))}',
               font=_font(int(0.2 * ppm), True), fill=(225, 220, 210))
    for _ in range(10):                                   # tyre marks
        cx, cy, r = int(rng.integers(0, N)), int(rng.integers(0, N)), int(rng.uniform(0.5, 2) * ppm)
        a0 = float(rng.uniform(0, 360))
        d.arc([cx - r, cy - r, cx + r, cy + r], a0, a0 + float(rng.uniform(30, 90)), fill=(70, 68, 66), width=int(0.04 * ppm))
    return img


def ceiling_texture(seed=11, ppm=200):
    """Suspended-ceiling tiles (0.6 m) with irregular stains, vents and sprinklers."""
    rng = np.random.default_rng(seed)
    N = int(6.1 * ppm)
    img = _surface(rng, N, N, (225, 225, 220), mottle=8, grain=3)
    d = ImageDraw.Draw(img)
    t = int(0.6 * ppm)
    for i in range(0, N, t):
        d.line([i, 0, i, N], fill=(150, 150, 150), width=4)
        d.line([0, i, N, i], fill=(150, 150, 150), width=4)
    for _ in range(25):
        x, y, r = int(rng.integers(0, N)), int(rng.integers(0, N)), int(rng.integers(ppm // 20, ppm // 5))
        d.ellipse([x - r, y - r, x + r, y + r], fill=(195, 185, 160))
    for _ in range(8):
        i, j = int(rng.integers(0, N // t)) * t, int(rng.integers(0, N // t)) * t
        for k in range(8, t - 8, 12):
            d.line([i + 10, j + k, i + t - 10, j + k], fill=(90, 90, 95), width=4)
    for _ in range(12):
        x, y = int(rng.integers(0, N)), int(rng.integers(0, N))
        d.ellipse([x - 8, y - 8, x + 8, y + 8], fill=(160, 30, 30))
    return img


def item_texture(kind, seed, px=512):
    rng = np.random.default_rng(seed)
    if kind == 'cardboard':
        img = _surface(rng, px, px, (185, 145, 95), mottle=12, grain=6)
        d = ImageDraw.Draw(img)
        d.rectangle([0, px * 0.45, px, px * 0.55], fill=(205, 175, 120))          # tape
        lx, ly = int(rng.integers(20, px // 3)), int(rng.integers(px * 0.6, px * 0.7))
        d.rectangle([lx, ly, lx + px // 2, ly + px // 4], fill=(245, 245, 240))    # label
        x = lx + 10
        while x < lx + px // 2 - 10:                                               # barcode
            bw = int(rng.integers(2, 7))
            d.rectangle([x, ly + 10, x + bw, ly + px // 8], fill=(10, 10, 10))
            x += bw + int(rng.integers(2, 6))
        d.text((lx + 10, ly + px // 7), _words(rng, 2).upper(), font=_font(px // 20, True), fill=(10, 10, 10))
        d.text((int(rng.integers(10, px // 2)), 20), _words(rng, 1).upper(), font=_font(px // 9, True),
               fill=_color(rng, 0, 120))
        ax = int(rng.integers(px // 2, px - 80))
        d.polygon([(ax, 120), (ax + 30, 60), (ax + 60, 120)], fill=(20, 20, 20))
    elif kind == 'wood':
        a = np.zeros((px, px, 3), np.float32)
        a[:] = (150, 105, 60)
        a += (np.sin(np.linspace(0, 40, px))[None, :, None] * 12 + _mottle(rng, px, px, 20, 14)[..., None])
        img = Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))
        d = ImageDraw.Draw(img)
        y = 0
        while y < px:                                     # planks with gaps
            y += int(rng.integers(px // 6, px // 3))
            d.rectangle([0, y, px, y + 8], fill=(60, 40, 25))
        for _ in range(4):
            x, yy = int(rng.integers(0, px)), int(rng.integers(0, px))
            d.ellipse([x, yy, x + 18, yy + 12], fill=(80, 50, 30))
        d.text((int(rng.integers(10, px // 2)), int(rng.integers(10, px // 2))), f'LOT {int(rng.integers(100, 999))}',
               font=_font(px // 10, True), fill=(40, 30, 20))
    elif kind == 'metal':
        img = _surface(rng, px, px, _color(rng, 60, 200), mottle=6, grain=4)
        d = ImageDraw.Draw(img)
        for _ in range(30):                               # scratches
            x, y = int(rng.integers(0, px)), int(rng.integers(0, px))
            d.line([x, y, x + int(rng.integers(-60, 60)), y + int(rng.integers(-20, 20))], fill=_color(rng, 150, 230), width=1)
        for _ in range(int(rng.integers(8, 14))):         # stickers and warning labels
            x, y = int(rng.integers(0, px - 90)), int(rng.integers(0, px - 50))
            sw, sh = int(rng.integers(40, 110)), int(rng.integers(25, 70))
            d.rectangle([x, y, x + sw, y + sh], fill=_color(rng, 120, 255), outline=(20, 20, 20), width=2)
            d.text((x + 4, y + 4), _words(rng, 1)[:6].upper(), font=_font(max(10, sh // 3), True), fill=(15, 15, 15))
        for x in range(14, px, int(rng.integers(40, 70))):  # rivets along the edges
            for y in (12, px - 12):
                d.ellipse([x - 5, y - 5, x + 5, y + 5], fill=(40, 40, 40))
        n = int(rng.integers(2, 5))                       # drawers with labels and handles
        for i in range(n):
            y0 = int(px * i / n) + 6
            d.rectangle([8, y0, px - 8, y0 + px // n - 12], outline=(30, 30, 30), width=3)
            d.rectangle([px // 2 - 50, y0 + 20, px // 2 + 50, y0 + 34], fill=(200, 200, 200))
            d.rectangle([px // 2 - 40, y0 + 45, px // 2 + 40, y0 + 75], fill=(250, 250, 245))
            d.text((px // 2 - 36, y0 + 48), f'{int(rng.integers(1, 99)):02d}-{"XYZW"[i % 4]}', font=_font(22, True),
                   fill=(10, 10, 10))
    elif kind == 'spines':
        img = Image.new('RGB', (px, px), (0, 0, 0))
        d = ImageDraw.Draw(img)
        x = 0
        while x < px:
            bw = int(rng.integers(px // 14, px // 6))
            d.rectangle([x, 0, x + bw, px], fill=_color(rng, 20, 230))
            d.rectangle([x, px // 8, x + bw, px // 8 + 8], fill=(220, 190, 60))
            d.text((x + 4, px // 3), _words(rng, 1)[:5], font=_font(bw // 3, True), fill=_color(rng, 200, 255))
            x += bw + 2
    else:  # 'bin': coloured plastic with a label
        img = _surface(rng, px, px, _color(rng, 40, 220), mottle=5, grain=3)
        d = ImageDraw.Draw(img)
        for i in range(0, px, int(rng.integers(px // 10, px // 6))):
            d.line([0, i, px, i], fill=(0, 0, 0), width=3)
        for _ in range(int(rng.integers(5, 10))):         # scuffs and stickers
            x, y = int(rng.integers(0, px - 80)), int(rng.integers(0, px - 40))
            d.rectangle([x, y, x + int(rng.integers(20, 80)), y + int(rng.integers(15, 40))], fill=_color(rng))
        d.rectangle([px // 4, px // 3, 3 * px // 4, px // 2], fill=(250, 250, 245))
        d.text((px // 4 + 10, px // 3 + 10), _words(rng, 1).upper()[:8], font=_font(px // 12, True), fill=(10, 10, 10))
    return img


def _texture(name, make):
    path = os.path.join(TEX_DIR, name + '.png')
    if not os.path.exists(path):
        os.makedirs(TEX_DIR, exist_ok=True)
        make().save(path)
    return path


# ---------------------------------------------------------------- geometry

class _Builder:
    def __init__(self, stage):
        self.stage = stage
        self.mats = {}
        self.n = 0
        UsdGeom.Xform.Define(stage, ROOT)

    def material(self, name, tex=None, color=(0.8, 0.8, 0.8), rough=0.75, emissive=None):
        if name in self.mats:
            return self.mats[name]
        path = f'{ROOT}/Looks/{name}'
        mat = UsdShade.Material.Define(self.stage, path)
        sh = UsdShade.Shader.Define(self.stage, path + '/Surface')
        sh.CreateIdAttr('UsdPreviewSurface')
        sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
        if tex:
            st = UsdShade.Shader.Define(self.stage, path + '/st')
            st.CreateIdAttr('UsdPrimvarReader_float2')
            st.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
            t = UsdShade.Shader.Define(self.stage, path + '/tex')
            t.CreateIdAttr('UsdUVTexture')
            t.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(tex)
            t.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), 'result')
            t.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
            t.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
            t.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
            t.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
            sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(t.ConnectableAPI(), 'rgb')
        else:
            sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        if emissive:
            sh.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
        self.mats[name] = mat
        return mat

    def mesh(self, name, quads, mat, collide=False):
        """quads: (origin, u, v) with the face's normal along u x v; the
        texture is stretched once over each face."""
        self.n += 1
        m = UsdGeom.Mesh.Define(self.stage, f'{ROOT}/{name}_{self.n}')
        pts, st = [], []
        for o, u, v in quads:
            o, u, v = Gf.Vec3f(*o), Gf.Vec3f(*u), Gf.Vec3f(*v)
            pts += [o, o + u, o + u + v, o + v]
            st += [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr([4] * len(quads))
        m.CreateFaceVertexIndicesAttr(list(range(4 * len(quads))))
        m.CreateSubdivisionSchemeAttr('none')
        UsdGeom.PrimvarsAPI(m).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                             UsdGeom.Tokens.faceVarying).Set(Vt.Vec2fArray(st))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        if collide:
            UsdPhysics.CollisionAPI.Apply(m.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
        return m

    def box(self, name, center, size, mat, collide=True):
        cx, cy, cz = center
        hx, hy, hz = (s / 2 for s in size)
        X, Y, Z = (2 * hx, 0, 0), (0, 2 * hy, 0), (0, 0, 2 * hz)
        neg = lambda a: tuple(-c for c in a)  # noqa: E731
        faces = []
        for n, u, v in [(X, Y, Z), (neg(X), neg(Y), Z), (Y, neg(X), Z), (neg(Y), X, Z), (Z, X, Y), (neg(Z), X, neg(Y))]:
            o = tuple(c + n_ / 2 - u_ / 2 - v_ / 2 for c, n_, u_, v_ in zip((cx, cy, cz), n, u, v))
            faces.append((o, u, v))
        return self.mesh(name, faces, mat, collide)


def _shelf(b, rng, x0, y0, along_x, length, depth, height, n_boards, mats):
    """A shelf unit against a wall. (x0, y0) = centre of its footprint;
    along_x: the long side runs along x (north/south walls) or y."""
    def at(s, t, z):        # s along the shelf, t across it -> world
        return (x0 + s, y0 + t, z) if along_x else (x0 + t, y0 + s, z)

    def size(ls, lt, lz):
        return (ls, lt, lz) if along_x else (lt, ls, lz)

    for s in (-length / 2 + 0.02, length / 2 - 0.02):
        for t in (-depth / 2 + 0.02, depth / 2 - 0.02):
            b.box('post', at(s, t, height / 2), size(0.03, 0.03, height), mats['frame'])
    for i in range(n_boards):
        z = 0.04 + i * (height - 0.06) / (n_boards - 1)
        b.box('board', at(0, 0, z), size(length, depth, 0.02), mats['board'])
        if i == n_boards - 1:
            break
        s = -length / 2 + 0.05
        gap = (height - 0.06) / (n_boards - 1) - 0.04
        while s < length / 2 - 0.08:                       # books and boxes
            if rng.random() < 0.65:
                w, h = rng.uniform(0.03, 0.07), rng.uniform(0.5, 0.9) * gap
                b.box('book', at(s + w / 2, 0, z + 0.01 + h / 2), size(w, depth * 0.7, h),
                      mats['spines'][int(rng.integers(0, len(mats['spines'])))], collide=False)
                s += w + 0.004
            else:
                w, h = rng.uniform(0.15, 0.3), rng.uniform(0.4, 0.8) * gap
                b.box('shelf_box', at(s + w / 2, 0, z + 0.01 + h / 2), size(w, depth * 0.8, h),
                      mats['small'][int(rng.integers(0, len(mats['small'])))], collide=False)
                s += w + rng.uniform(0.01, 0.12)


def dress(stage, light=None):
    """Furnish the room; returns lines for the log. `light`: intensity of
    each ceiling panel (None: default)."""
    log = []
    b = _Builder(stage)
    rng = np.random.default_rng(3)
    ppm = 320

    # Walls: one overlay 5 mm in front of each wall's inner face.
    walls = {  # name: (origin, u, v): u runs left to right as seen from inside
        'south': ((HALF, -HALF + 0.005, 0), (-2 * HALF, 0, 0), (0, 0, HEIGHT)),
        'east': ((HALF - 0.005, HALF, 0), (0, -2 * HALF, 0), (0, 0, HEIGHT)),
        'north': ((-HALF, HALF - 0.005, 0), (2 * HALF, 0, 0), (0, 0, HEIGHT)),
        'west': ((-HALF + 0.005, -HALF, 0), (0, 2 * HALF, 0), (0, 0, HEIGHT)),
    }
    for i, (name, quad) in enumerate(walls.items()):
        tex = _texture(f'wall_{i}', lambda i=i: wall_texture(i, ppm))
        b.mesh('wall_' + name, [quad], b.material('wall_' + name, tex, rough=0.9))
    log.append('walls: 4 textured overlays')

    tex = _texture('floor', lambda: floor_texture())
    b.mesh('floor', [((-HALF, -HALF, 0.001), (2 * HALF, 0, 0), (0, 2 * HALF, 0))], b.material('floor', tex, rough=0.8))

    # Ceiling (facing down) closes the room; the sun light is switched off.
    tex = _texture('ceiling', lambda: ceiling_texture())
    C = HALF + 0.05
    b.mesh('ceiling', [((-C, C, HEIGHT), (2 * C, 0, 0), (0, -2 * C, 0))], b.material('ceiling', tex, rough=0.95))
    for p in stage.Traverse():
        if p.GetTypeName() in ('DistantLight', 'DomeLight') and not p.GetPath().HasPrefix(ROOT):
            p.GetAttribute('inputs:intensity').Set(0.0)
            log.append(f'switched off {p.GetPath()}')
    intensity = light or 12000.0
    panel = b.material('light_panel', color=(1, 1, 1), emissive=(1, 1, 1))
    for x in (-1.8, 0.0, 1.8):
        for y in (-1.8, 0.0, 1.8):
            L = UsdLux.RectLight.Define(stage, f'{ROOT}/light_{int((x + 2) * 10)}_{int((y + 2) * 10)}')
            L.CreateWidthAttr(0.6)
            L.CreateHeightAttr(0.6)
            L.CreateIntensityAttr(intensity)
            L.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.92))
            UsdGeom.Xformable(L).AddTranslateOp().Set(Gf.Vec3d(x, y, HEIGHT - 0.02))
            b.mesh('panel', [((x - 0.3, y + 0.3, HEIGHT - 0.012), (0.6, 0, 0), (0, -0.6, 0))], panel)
    log.append(f'ceiling with 9 panel lights ({intensity:g})')

    # Furniture along the walls (clear of the scene's own objects).
    mats = {
        'frame': b.material('frame', color=(0.15, 0.15, 0.17), rough=0.5),
        'board': b.material('board', _texture('wood_0', lambda: item_texture('wood', 0))),
        'spines': [b.material(f'spines_{i}', _texture(f'spines_{i}', lambda i=i: item_texture('spines', 10 + i)))
                   for i in range(4)],
        'small': [b.material(f'small_{i}', _texture(f'small_{i}', lambda i=i: item_texture(['cardboard', 'bin'][i % 2], 20 + i)))
                  for i in range(4)],
    }
    card = [b.material(f'card_{i}', _texture(f'card_{i}', lambda i=i: item_texture('cardboard', 30 + i))) for i in range(5)]
    crate = [b.material(f'crate_{i}', _texture(f'crate_{i}', lambda i=i: item_texture('wood', 40 + i))) for i in range(3)]
    cab = [b.material(f'cabinet_{i}', _texture(f'cabinet_{i}', lambda i=i: item_texture('metal', 50 + i)), rough=0.5)
           for i in range(3)]
    bins = [b.material(f'bin_{i}', _texture(f'bin_{i}', lambda i=i: item_texture('bin', 60 + i)), rough=0.6)
            for i in range(3)]

    def against(wall, s, depth):
        """Centre of a footprint `depth` deep standing against `wall`, at
        position s along it."""
        e = HALF - 0.02 - depth / 2
        return {'north': (s, e), 'south': (s, -e), 'east': (e, s), 'west': (-e, s)}[wall]

    def item(wall, s, along, depth, h, mat, z0=0.0):
        x, y = against(wall, s, depth)
        sz = (along, depth, h) if wall in ('north', 'south') else (depth, along, h)
        b.box('item', (x, y, z0 + h / 2), sz, mat)

    for wall, s, length, depth, height, boards in [('north', -1.6, 1.6, 0.4, 1.5, 4),
                                                   ('south', -0.1, 1.4, 0.35, 1.2, 3),
                                                   ('west', 1.55, 1.5, 0.38, 1.0, 3)]:
        x, y = against(wall, s, depth)
        _shelf(b, rng, x, y, wall in ('north', 'south'), length, depth, height, boards, mats)
    item('north', 0.1, 0.9, 0.45, 0.95, cab[0])
    item('north', 1.0, 0.5, 0.4, 0.4, card[0])
    item('north', 1.0, 0.4, 0.35, 0.3, card[1], z0=0.4)
    item('north', 1.8, 0.6, 0.4, 0.35, crate[0])
    item('north', 2.5, 0.4, 0.3, 0.25, bins[0])
    item('south', -1.5, 0.6, 0.4, 0.4, crate[1])
    item('south', -1.5, 0.5, 0.35, 0.3, card[2], z0=0.4)
    item('south', 1.1, 0.45, 0.4, 0.5, card[3])
    item('west', -1.0, 0.8, 0.45, 1.4, cab[1])
    item('west', 0.2, 0.5, 0.4, 0.45, card[4])
    item('west', 0.2, 0.4, 0.3, 0.2, bins[1], z0=0.45)
    item('east', -1.2, 0.9, 0.45, 0.9, cab[2])
    item('east', 0.0, 0.5, 0.4, 0.35, crate[2])
    item('east', 0.0, 0.4, 0.3, 0.25, bins[2], z0=0.35)
    item('east', 2.1, 0.6, 0.45, 0.5, card[0])
    log.append(f'{b.n} meshes, textures in {TEX_DIR}')
    return log
