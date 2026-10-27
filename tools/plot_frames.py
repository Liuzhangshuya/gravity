"""Draw the joint axes and DH coordinate frames of the arm.

Parameters are parsed from User/gravity/robot_parameters.h, so the picture
always matches what the firmware actually uses. Two poses are drawn:

  docs/frames_q0.png    model reference pose (q = 0)
  docs/frames_home.png  physical "motor zero" pose (encoder = 0)

Frame {0} is the base. frames[i] = A0*A1*...*Ai is frame {i+1}, so J0 is the
first motor and its rotation axis is z of frame {1}.
"""
from pathlib import Path
import math
import re

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / 'User/gravity/robot_parameters.h'
OUT = ROOT / 'docs'

SS = 2                      # supersampling factor
W, H = 1100, 780

DARK = (40, 40, 40)
LINK = (90, 90, 90)
RED = (192, 57, 43)
GREEN = (39, 128, 70)
BLUE = (37, 99, 168)
ORANGE = (214, 132, 26)
GREY = (150, 150, 150)

AZ = math.radians(38)
EL = math.radians(24)


def load_params():
    """Parse all J<n>_<NAME> defines, ignoring matches inside comments."""
    params = {}
    text = HEADER.read_text(encoding='utf-8')
    for line in text.splitlines():
        if not line.lstrip().startswith('#define'):
            continue
        m = re.match(r'\s*#define\s+J(\d)_([A-Z_]+)\s+([-0-9().f]+)', line)
        if not m:
            continue
        value = m.group(3).strip()
        if value.endswith('f'):
            value = value[:-1]
        params[(int(m.group(1)), m.group(2))] = float(eval(value, {}, {}))
    return params


def dh(a, alpha, d, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st, 0.0, a],
                     [st * ca, ct * ca, -sa, -sa * d],
                     [st * sa, ct * sa, ca, ca * d],
                     [0.0, 0.0, 0.0, 1.0]])


def frames(p, q):
    T = np.eye(4)
    out = []
    for i in range(6):
        theta = q[i] + p[(i, 'THETA_OFFSET')]
        T = T @ dh(p[(i, 'A')], p[(i, 'ALPHA')], p[(i, 'D')], theta)
        out.append(T.copy())
    return out


def project(pt):
    """World (m) -> screen (m) in an isometric-ish view. SVG-like y down."""
    x, y, z = pt
    u = -math.sin(AZ) * x + math.cos(AZ) * y
    v = -math.cos(AZ) * math.sin(EL) * x - math.sin(AZ) * math.sin(EL) * y + math.cos(EL) * z
    return np.array([u, -v])


class Canvas:
    def __init__(self):
        self.img = Image.new('RGB', (W * SS, H * SS), 'white')
        self.draw = ImageDraw.Draw(self.img)
        self.font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 15 * SS)
        self.small = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 13 * SS)
        self.title_font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 19 * SS)

    @staticmethod
    def _s(p):
        return (p[0] * SS, p[1] * SS)

    def line(self, a, b, color, width):
        self.draw.line([self._s(a), self._s(b)], fill=color, width=int(width * SS))

    def dot(self, p, color, r):
        x, y = self._s(p)
        self.draw.ellipse([x - r * SS, y - r * SS, x + r * SS, y + r * SS], fill=color)

    def text(self, p, s, color=DARK, font=None, dx=0, dy=0, anchor='la', halo=True):
        x, y = self._s((p[0] + dx, p[1] + dy))
        font = font or self.small
        if halo:
            left, top, right, bottom = self.draw.textbbox((x, y), s, font=font, anchor=anchor)
            self.draw.rectangle([left - 2, top - 1, right + 2, bottom + 1], fill=(255, 255, 255))
        self.draw.text((x, y), s, fill=color, font=font, anchor=anchor)

    def arrow(self, origin, tip, color, width):
        self.line(origin, tip, color, width)
        d = np.array(tip) - np.array(origin)
        n = np.linalg.norm(d)
        if n < 1e-9:
            return
        d = d / n
        head = 7
        left = np.array(tip) - head * d + head * 0.45 * np.array([-d[1], d[0]])
        right = np.array(tip) - head * d - head * 0.45 * np.array([-d[1], d[0]])
        self.draw.polygon([self._s(tip), self._s(left), self._s(right)], fill=color)


def draw(p, q, title, path):
    F = frames(p, q)
    origins = [np.zeros(3)] + [T[:3, 3] for T in F]

    # Fit: gather link points and axis tips.
    pts = list(origins)
    for T in F:
        z = T[:3, 2]
        pts += [T[:3, 3] + 0.055 * z, T[:3, 3] - 0.055 * z]
    proj = [project(pt) for pt in pts]
    lo = np.min(proj, axis=0)
    hi = np.max(proj, axis=0)
    span = np.max(hi - lo)
    pad = 0.16 * span
    scale = min((W - 260) / (hi[0] - lo[0] + 2 * pad),
                (H - 170) / (hi[1] - lo[1] + 2 * pad))
    cx = 0.5 * (lo[0] + hi[0])
    cy = 0.5 * (lo[1] + hi[1])

    def S(pt):
        u, v = project(pt)
        return np.array([W / 2 + (u - cx) * scale,
                         H / 2 + 30 + (v - cy) * scale])

    c = Canvas()
    c.text((36, 26), title, font=c.title_font)

    # Base footprint on the ground plane.
    s = 0.055
    square = [(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)]
    c.draw.polygon([c._s(S(v)) for v in square], outline=(205, 205, 205),
                   fill=(247, 247, 247))

    # Link centre lines.
    for i in range(6):
        c.line(S(origins[i]), S(origins[i + 1]), LINK, 3.2)

    # Base frame {0}.
    O = S(np.zeros(3))
    for axis, color, name in [((1, 0, 0), RED, 'x0'), ((0, 1, 0), GREEN, 'y0'),
                              ((0, 0, 1), BLUE, 'z0')]:
        tip = S(np.array(axis, dtype=float) * 0.05)
        c.arrow(O, tip, color, 2.0)
        c.text(tip, name, color, dx=3, dy=-9)

    # Frames {1}..{6} and joint axes z1..z6.
    for i, T in enumerate(F):
        origin = T[:3, 3]
        s0 = S(origin)
        for axis, color, length, width in [((1, 0, 0), RED, 0.032, 1.6),
                                           ((0, 1, 0), GREEN, 0.032, 1.6)]:
            tip = S(origin + length * np.array(axis, dtype=float))
            c.arrow(s0, tip, color, width)

        z = T[:3, 2]
        lo_z = S(origin - 0.052 * z)
        hi_z = S(origin + 0.052 * z)
        c.line(lo_z, hi_z, BLUE, 1.6)
        c.arrow((lo_z + hi_z) / 2, hi_z, BLUE, 1.6)
        c.arrow((lo_z + hi_z) / 2, lo_z, BLUE, 1.6)
        c.text(hi_z, f'z{i + 1}', BLUE, dx=5, dy=-15 if i % 2 == 0 else 5)
        c.dot(s0, DARK, 3.0)
        dx, dy = [(9, -21), (9, 10), (9, -21), (11, 12), (30, -30), (9, 14)][i]
        c.text(s0, f'{{{i + 1}}}', DARK, font=c.font, dx=dx, dy=dy)

    # Mass points (centre of mass used by the gravity model).
    for i in range(6):
        link = int(p[(i, 'LINK')])
        if link < 0:
            continue
        com = p[(i, 'COM_X')], p[(i, 'COM_Y')], p[(i, 'COM_Z')]
        world = F[link] @ np.array([*com, 1.0])
        sp = S(world[:3])
        c.dot(sp, ORANGE, 2.6)
        c.text(sp, f'm{i}', ORANGE, dx=5, dy=-2)

    # Legend.
    lx, ly = 34, H - 122
    for k, (color, label) in enumerate([(RED, 'x axis'), (GREEN, 'y axis'),
                                        (BLUE, 'z axis = joint rotation axis'),
                                        (ORANGE, 'point mass (COM)')]):
        yy = ly + k * 22
        c.dot((lx + 4, yy + 8), color, 4)
        c.text((lx + 16, yy), label, DARK)
    c.text((34, H - 26), 'frame {0} = base, {i+1} = frames[i], q = ' +
           '[' + ', '.join(f'{v:.3f}' for v in q) + ']  [rad]', DARK)

    OUT.mkdir(exist_ok=True)
    c.img.resize((W, H), Image.LANCZOS).save(path)
    print('wrote', path)


def main():
    p = load_params()
    zeros = [0.0] * 6
    home = [-p[(i, 'POSITION_SIGN')] * p[(i, 'ENCODER_ZERO')] for i in range(6)]
    draw(p, zeros, 'DH frames at q = 0 (model reference pose)', OUT / 'frames_q0.png')
    draw(p, home, 'DH frames at the physical home pose (encoder = 0)', OUT / 'frames_home.png')


if __name__ == '__main__':
    main()
