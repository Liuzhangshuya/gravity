"""Draw one position-Jacobian column, for teaching the z × r rule.

Fig. 1  docs/jacobian_column_j1.png   the J1 column on the whole arm at q = 0
Fig. 2  docs/jacobian_column_j5_tcp.png  why J5's column is zero at the {6}
         origin and non-zero once the TCP is offset off the axis

Everything is read from User/gravity/robot_parameters.h and drawn with the
same helpers as tools/plot_frames.py, so the pictures match the firmware.
"""
from pathlib import Path
import sys

import numpy as np

sys.dont_write_bytecode = True          # keep tools/ free of .pyc files
sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_frames as pf  # noqa: E402

DARK = pf.DARK
GREY = pf.GREY
BLUE = pf.BLUE
RED = pf.RED
GREEN = pf.GREEN
ORANGE = pf.ORANGE


class View:
    """Fit a set of world points into a rectangle of the logical canvas."""

    def __init__(self, pts, rect, margin=26):
        p = np.array([pf.project(pt) for pt in pts])
        lo, hi = p.min(axis=0), p.max(axis=0)
        span = np.maximum(hi - lo, 1e-9)
        self.rect = rect
        self.scale = min((rect[2] - 2 * margin) / span[0],
                         (rect[3] - 2 * margin) / span[1])
        self.c = 0.5 * (lo + hi)

    def __call__(self, pt):
        u, v = pf.project(pt)
        return np.array([self.rect[0] + self.rect[2] / 2 + (u - self.c[0]) * self.scale,
                         self.rect[1] + self.rect[3] / 2 + (v - self.c[1]) * self.scale])


def dashed(c, a, b, color, width=1.3, dash=8, gap=5):
    a, b = np.asarray(a, float), np.asarray(b, float)
    length = np.linalg.norm(b - a)
    if length < 1e-9:
        return
    d = (b - a) / length
    step = dash + gap
    k = 0
    while k * step < length:
        start = a + d * (k * step)
        end = a + d * min(length, k * step + dash)
        c.line(start, end, color, width)
        k += 1


def column(p, q, tcp_local, joint):
    F = pf.frames(p, q)
    p_tcp = (F[5] @ np.array([*tcp_local, 1.0]))[:3]
    z = F[joint][:3, 2]
    o = F[joint][:3, 3]
    r = p_tcp - o
    return F, p_tcp, z, o, r, np.cross(z, r)


def draw_chain(c, p, q, V, faint=1.6):
    F = pf.frames(p, q)
    pts = [np.zeros(3)] + [T[:3, 3] for T in F]
    for i in range(6):
        c.line(V(pts[i]), V(pts[i + 1]), pf.LINK, 3.0)
    for T in F:
        c.dot(V(T[:3, 3]), DARK, 2.6)
    return F


def figure_j1(p):
    q = np.zeros(6)
    tcp_local = np.zeros(3)
    F, p_tcp, z, o, r, v = column(p, q, tcp_local, 1)

    foot = o + r.dot(z) * z              # projection of the TCP onto the J1 axis
    axis_lo = o - 0.065 * z
    axis_hi = o + 0.065 * z
    v_draw = p_tcp + 0.22 * v            # shortened for the picture only

    pts = [np.zeros(3)] + [T[:3, 3] for T in F] + [p_tcp, v_draw, axis_lo, axis_hi, foot]
    V = View(pts, (250, 66, 800, 620))
    c = pf.Canvas()
    c.text((36, 26), 'Position Jacobian column of J1 at q = 0', font=c.title_font)

    s = 0.055
    c.draw.polygon([c._s(V(v_)) for v_ in [(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)]],
                   outline=(208, 208, 208), fill=(248, 248, 248))

    draw_chain(c, p, q, V)

    # All joint axes, faint, so the highlighted one has context.
    for T in F:
        oi, zi = T[:3, 3], T[:3, 2]
        c.line(V(oi - 0.045 * zi), V(oi + 0.045 * zi), (170, 196, 220), 1.4)

    # Highlighted J1 axis.
    dashed(c, V(axis_lo), V(axis_hi), BLUE, 2.0, 9, 6)
    c.arrow(V(o), V(axis_hi), BLUE, 2.0)
    c.text(V(axis_lo), 'J1 axis (z of frames[1])', BLUE, anchor='rm', dx=-6)

    # r and its perpendicular part.
    dashed(c, V(o), V(p_tcp), GREY, 1.6)
    c.text(V(o + 0.35 * r), 'r = p_tcp - o', GREY, dx=8, dy=8)
    dashed(c, V(foot), V(p_tcp), ORANGE, 1.8)
    c.text(V(foot + 0.65 * (p_tcp - foot)), 'r_perp = 0.278 m', ORANGE, dx=8, dy=-16)

    c.dot(V(o), BLUE, 3.4)
    c.text(V(o), 'o (on axis)', BLUE, dx=9, dy=9)
    c.dot(V(p_tcp), RED, 3.4)
    c.text(V(p_tcp), 'p_tcp', RED, dx=7, dy=-18)

    c.arrow(V(p_tcp), V(v_draw), RED, 3.0)
    c.text(V(v_draw), 'v = z x r    |v| = 0.278 m/rad', RED, dx=6, dy=-8)

    c.text((36, pf.H - 118), 'right-hand rule: turn +q1 about the blue arrow,', DARK)
    c.text((36, pf.H - 96), 'the TCP moves along the red arrow.', DARK)
    c.text((36, pf.H - 62), 'the arrow is shortened; |v| equals the', DARK)
    c.text((36, pf.H - 40), 'perpendicular distance from p_tcp to the axis.', DARK)

    path = pf.OUT / 'jacobian_column_j1.png'
    c.img.resize((pf.W, pf.H), pf.Image.LANCZOS).save(path)
    print('wrote', path)

    print('\nJ1 column at q = 0, tcp_local = [0,0,0]')
    print('  o   =', np.round(o, 6))
    print('  z   =', np.round(z, 6))
    print('  r   =', np.round(r, 6))
    print('  r_perp =', round(float(np.linalg.norm(r - r.dot(z) * z)), 6), 'm')
    print('  v   =', np.round(v, 6), 'm/rad,  |v| =', round(float(np.linalg.norm(v)), 6))


def wrist_panel(c, p, q, tcp_local, rect, title):
    F, p_tcp, z, o, r, v = column(p, q, tcp_local, 5)
    n = np.linalg.norm(v)
    v_draw = p_tcp + (v / n * 0.055 if n > 1e-9 else np.zeros(3))

    pts = [F[3][:3, 3], F[4][:3, 3], F[5][:3, 3], p_tcp,
           o - 0.055 * z, o + 0.055 * z, v_draw]
    V = View(pts, rect, margin=34)

    c.text((rect[0], rect[1]), title, font=c.font)

    # Links {4}->{5}->{6} and the TCP.
    c.line(V(F[3][:3, 3]), V(F[4][:3, 3]), pf.LINK, 3.0)
    c.line(V(F[4][:3, 3]), V(F[5][:3, 3]), pf.LINK, 3.0)
    for i in (3, 4, 5):
        c.dot(V(F[i][:3, 3]), DARK, 2.6)
        c.text(V(F[i][:3, 3]), f'{{{i + 1}}}', DARK, dx=5, dy=18 if i == 5 else 4)

    dashed(c, V(o - 0.055 * z), V(o + 0.055 * z), BLUE, 2.0, 8, 5)
    c.text(V(o - 0.055 * z), 'J5 axis', BLUE, anchor='rm', dx=-6)

    dashed(c, V(o), V(p_tcp), ORANGE, 1.6)
    c.dot(V(p_tcp), RED, 3.4)
    c.text(V(p_tcp), 'p_tcp', RED, dx=6, dy=-16)

    if n > 1e-9:
        c.text(V(o + 0.5 * r), 'r_perp = 0.050 m', ORANGE, dx=6, dy=-6)
        c.arrow(V(p_tcp), V(v_draw), RED, 2.8)
        c.text(V(v_draw), 'v = z x r, |v| = 0.050 m/rad', RED,
               anchor='rm', dx=-6, dy=-4)
    else:
        c.text(V(p_tcp), 'r_perp = 0, v = 0', RED, dx=8, dy=34)


def figure_j5(p):
    q = np.zeros(6)
    c = pf.Canvas()
    c.text((36, 26), 'Why J5 contributes no TCP velocity: only the offset off the axis counts',
           font=c.title_font)
    wrist_panel(c, p, q, np.array([0, 0, 0.0]), (40, 74, 500, 430),
                'tcp_local = (0,0,0): on the axis -> column zero')
    wrist_panel(c, p, q, np.array([0.05, 0, 0.0]), (570, 74, 500, 430),
                'tcp_local = (0.05,0,0): 5 cm off -> column 0.05')
    c.text((40, 566), 'An offset along the axis (tcp_local = (0,0,0.05)) also gives zero:',
           DARK)
    c.text((40, 590), 'sliding a point along an axis does not change its distance to that axis.',
           DARK)

    path = pf.OUT / 'jacobian_column_j5_tcp.png'
    (c.img.resize((pf.W, pf.H), pf.Image.LANCZOS).crop((0, 0, pf.W, 620)).save(path))
    print('wrote', path)

    for tcp_local in ([0, 0, 0], [0, 0, 0.05], [0.05, 0, 0]):
        _, p_tcp, z, o, r, v = column(p, q, tcp_local, 5)
        print(f'  tcp_local={tcp_local}  p_tcp={np.round(p_tcp, 4)}  J5 column={np.round(v, 4)}')


def main():
    p = pf.load_params()
    print('Jv at q = 0, TCP = frames[5] origin')
    q = np.zeros(6)
    for tcp_local in ([0, 0, 0], [0.05, 0, 0]):
        F = pf.frames(p, q)
        p_tcp = (F[5] @ np.array([*tcp_local, 1.0]))[:3]
        J = np.column_stack([np.cross(F[i][:3, 2], p_tcp - F[i][:3, 3]) for i in range(6)])
        print(f'  tcp_local={tcp_local}  p_tcp={np.round(p_tcp, 6)}')
        for row in np.round(J, 6):
            print('    ', row)
    figure_j1(p)
    figure_j5(p)


if __name__ == '__main__':
    main()
