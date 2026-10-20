#!/usr/bin/env python3
"""Detailed original OBJ8 models of a SpaceShipTwo-class spaceplane and a
WhiteKnightTwo-class twin-fuselage carrier, plus rocket plume, smoke trail
and probe objects.

Dimensions follow public figures (SpaceShipTwo: 18.3 m long, 8.3 m span,
2.3 m cabin; WhiteKnightTwo: 43 m span, twin fuselages, four turbofans).
The geometry, texturing and markings are original; no manufacturer assets
or logos are used.
"""
from __future__ import annotations

import sys
from math import pi
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont
from scipy.interpolate import PchipInterpolator

sys.path.insert(0, str(Path(__file__).resolve().parent))
from objlib import Part, mirror_x, preview, translate, write_obj  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
ASSETS.mkdir(exist_ok=True)
PREVIEW = ROOT / "art" / "previews"
PREVIEW.mkdir(parents=True, exist_ok=True)
TEX = 2048
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

# ----------------------------------------------------------------- geometry


def body(stations, n_around=72, n_len=120, uv=(0, 0, 0.5, 1.0), center_x=0.0, flatten=None):
    """Body of revolution / lofted ellipse. stations: (z, rx, ry, cy).

    Returns (grid P, UV, theta, z) with theta measured from the top toward +x.
    """
    st = np.array(stations, float)
    zs = st[:, 0]
    fz = np.linspace(zs[0], zs[-1], n_len)
    rx = PchipInterpolator(zs, st[:, 1])(fz)
    ry = PchipInterpolator(zs, st[:, 2])(fz)
    cy = PchipInterpolator(zs, st[:, 3])(fz)
    th = np.linspace(0, 2 * pi, n_around + 1)
    T, Z = np.meshgrid(th, fz)
    RX = np.maximum(rx[:, None], 0.004)
    RY = np.maximum(ry[:, None], 0.004)
    X = center_x + RX * np.sin(T)
    Y = cy[:, None] + RY * np.cos(T)
    if flatten is not None:
        # Slightly flatter belly: blend the lower half toward a superellipse.
        lower = np.clip(-np.cos(T), 0, 1)
        Y = Y + flatten * lower ** 3 * RY
    P = np.stack([X, Y, Z], axis=2)
    u0, v0, u1, v1 = uv
    U = u0 + (u1 - u0) * T / (2 * pi)
    V = v0 + (v1 - v0) * (Z - zs[0]) / (zs[-1] - zs[0])
    return P, np.stack([U, V], axis=2), th, fz


def airfoil(n=28, t=0.10, camber=0.015, f0=0.0, f1=1.0):
    """Closed airfoil loop: TE(bottom) -> LE -> TE(top), x/c in [f0, f1]."""
    beta = np.linspace(0, pi, n)
    xc = f0 + (f1 - f0) * (1 - np.cos(beta)) / 2
    yt = 5 * t * (0.2969 * np.sqrt(xc) - 0.1260 * xc - 0.3516 * xc ** 2 + 0.2843 * xc ** 3 - 0.1036 * xc ** 4)
    yc = np.where(xc < 0.4, camber / 0.16 * (0.8 * xc - xc ** 2), camber / 0.36 * (0.2 + 0.8 * xc - xc ** 2))
    lower = np.stack([xc[::-1], (yc - yt)[::-1]], 1)
    upper = np.stack([xc[1:], (yc + yt)[1:]], 1)
    return np.concatenate([lower, upper])


def wing(sections, uv, n_chord=26, f0=0.0, f1=1.0, close_tip=True, close_root=False, n_span=None):
    """Wing surface from spanwise sections (span_x, le_z, chord, y, t, camber).

    Built with span along +x, thickness along +y, chord along +z (aft).
    """
    sec = np.array(sections, float)
    if n_span:
        s = np.linspace(sec[0, 0], sec[-1, 0], n_span)
        sec = np.stack([s] + [np.interp(s, sec[:, 0], sec[:, k]) for k in range(1, 6)], 1)
    loops = []
    for x, le, chord, y, t, cam in sec:
        af = airfoil(n_chord, t, cam, f0, f1)
        loops.append(np.stack([np.full(len(af), x), y + af[:, 1] * chord, le + af[:, 0] * chord], 1))
    P = np.array(loops)
    xmin, xmax = sec[:, 0].min(), sec[:, 0].max()
    zmin = (sec[:, 1]).min()
    zmax = (sec[:, 1] + sec[:, 2]).max()
    u0, v0, u1, v1 = uv
    U = u0 + (u1 - u0) * (P[..., 0] - xmin) / max(1e-6, xmax - xmin)
    V = v1 - (v1 - v0) * (P[..., 2] - zmin) / max(1e-6, zmax - zmin)
    part = Part("wing")
    part.add_grid(P, np.stack([U, V], 2))
    _orient_outward(part, P)
    for close, k, sgn in ((close_tip, -1, 1), (close_root, 0, -1)):
        if close:
            loop = P[k]
            c = loop.mean(0)
            uvl = np.stack([U[k], V[k]], 1)
            part.add_tri_fan(c, list(loop) + [loop[0]], (sgn, 0, 0), uvl.mean(0), list(uvl) + [uvl[0]])
    if f0 > 0 or f1 < 1:
        # Close the chordwise cut faces (hinge line).
        for idx, nz in ((0, -1),):
            pass
    return part, P


def _orient_outward(part, P):
    """Flip normals of the latest grid if they point into the solid."""
    center = P.reshape(-1, 3).mean(0)
    vs = part.v
    n = len(P.reshape(-1, 3))
    start = len(vs) - n
    score = 0.0
    for k in range(start, len(vs), max(1, n // 200)):
        p = np.array(vs[k][:3])
        nn = np.array(vs[k][3:6])
        # Compare against the local chord line (thickness direction).
        score += np.dot(nn[1:2], (p - center)[1:2])
    if score < 0:
        for k in range(start, len(vs)):
            x, y, z, nx, ny, nz, s, t = vs[k]
            vs[k] = (x, y, z, -nx, -ny, -nz, s, t)
        tris = part.i
        # Re-wind the triangles that belong to this grid.
        m = len(tris)
        for q in range(0, m, 3):
            a, b, c = tris[q:q + 3]
            if a >= start:
                tris[q + 1], tris[q + 2] = c, b


def rot_to_vertical(part: Part, outboard_sign=1.0):
    """Rotate a wing built along +x into a fin along +y (thickness -> x)."""
    def fp(p):
        return np.array([-p[1] * outboard_sign, p[0], p[2]])
    out = part.transformed(fp, fp)
    if outboard_sign < 0:
        tris = np.array(out.i).reshape(-1, 3)
        out.i = list(tris[:, [0, 2, 1]].ravel())
    return out


def grid_part(name, P, UV, flip=False, wrap_j=False, anim=None, attrs=None):
    p = Part(name, anim or [], attrs or [])
    p.add_grid(P, UV, flip=flip, wrap_j=wrap_j)
    return p


def disk(name, center, radius, normal_axis, uv_c, uv_r, n=32, sign=1):
    p = Part(name)
    th = np.linspace(0, 2 * pi, n + 1)
    ring = []
    uvs = []
    for t in th:
        if normal_axis == "z":
            ring.append(np.array(center) + [radius * np.cos(t), radius * np.sin(t), 0])
        elif normal_axis == "x":
            ring.append(np.array(center) + [0, radius * np.cos(t), radius * np.sin(t)])
        uvs.append((uv_c[0] + uv_r * np.cos(t), uv_c[1] + uv_r * np.sin(t)))
    nrm = {"z": (0, 0, sign), "x": (sign, 0, 0)}[normal_axis]
    p.add_tri_fan(center, ring, nrm, uv_c, uvs)
    return p


def merge(name, parts, anim=None, attrs=None):
    out = Part(name, anim or [], attrs or [])
    for p in parts:
        off = len(out.v)
        out.v.extend(p.v)
        out.i.extend(k + off for k in p.i)
    return out


def box(name, x0, x1, y0, y1, z0, z1, uv):
    p = Part(name)
    u0, v0, u1, v1 = uv
    faces = [
        ((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (0, 0, -1)),
        ((x1, y0, z1), (x0, y0, z1), (x0, y1, z1), (x1, y1, z1), (0, 0, 1)),
        ((x0, y0, z1), (x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (-1, 0, 0)),
        ((x1, y0, z0), (x1, y0, z1), (x1, y1, z1), (x1, y1, z0), (1, 0, 0)),
        ((x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1), (0, 1, 0)),
        ((x0, y0, z1), (x1, y0, z1), (x1, y0, z0), (x0, y0, z0), (0, -1, 0)),
    ]
    for a, b, c, d, n in faces:
        p.add_quad(a, b, c, d, n, [(u0, v0), (u1, v0), (u1, v1), (u0, v1)])
    return p


def wheel(name, cx, cy, cz, r, w, uv):
    """Tyre as a short body of revolution around the x axis."""
    th = np.linspace(0, 2 * pi, 25)
    xs = np.array([-w / 2, -w / 2, -w * 0.45, w * 0.45, w / 2, w / 2])
    rs = np.array([0.01, r * 0.62, r, r, r * 0.62, 0.01])
    P = np.zeros((len(xs), len(th), 3))
    for a, (x, rr) in enumerate(zip(xs, rs)):
        P[a, :, 0] = cx + x
        P[a, :, 1] = cy + rr * np.cos(th)
        P[a, :, 2] = cz + rr * np.sin(th)
    u0, v0, u1, v1 = uv
    U, V = np.meshgrid(np.linspace(u0, u1, len(th)), np.linspace(v0, v1, len(xs)))
    p = Part(name)
    p.add_grid(P, np.stack([U, V], 2), flip=True)
    return p


def strut(name, a, b, r, uv, n=10):
    a, b = np.array(a, float), np.array(b, float)
    d = b - a
    L = np.linalg.norm(d)
    d /= L
    ref = np.array([1, 0, 0]) if abs(d[0]) < 0.9 else np.array([0, 0, 1])
    e1 = np.cross(d, ref)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(d, e1)
    th = np.linspace(0, 2 * pi, n + 1)
    P = np.array([[a + s * L * d + r * (np.cos(t) * e1 + np.sin(t) * e2) for t in th] for s in (0, 1)])
    u0, v0, u1, v1 = uv
    U, V = np.meshgrid(np.linspace(u0, u1, n + 1), [v0, v1])
    p = Part(name)
    p.add_grid(P, np.stack([U, V], 2))
    _orient_radial(p, a, d)
    return p


def _orient_radial(p, a, d):
    v = p.v[0]
    pt = np.array(v[:3]) - a
    radial = pt - np.dot(pt, d) * d
    if np.dot(radial, v[3:6]) < 0:
        p.v = [(x, y, z, -nx, -ny, -nz, s, t) for x, y, z, nx, ny, nz, s, t in p.v]
        tr = np.array(p.i).reshape(-1, 3)
        p.i = list(tr[:, [0, 2, 1]].ravel())


def cut_holes(part: Part, keep_point):
    """Drop triangles whose centroid lies inside an opening."""
    tris = np.array(part.i).reshape(-1, 3)
    V = np.array([v[:3] for v in part.v])
    cent = V[tris].mean(1)
    mask = np.array([keep_point(c) for c in cent])
    part.i = list(tris[mask].ravel())


# ------------------------------------------------------------ SpaceShipTwo

SS2_FUSE = [
    (-8.6, 0.0, 0.0, -0.05), (-8.47, 0.24, 0.22, -0.04), (-8.12, 0.50, 0.47, -0.02),
    (-7.6, 0.74, 0.71, 0.0), (-6.9, 0.94, 0.91, 0.0), (-6.05, 1.07, 1.04, 0.0),
    (-5.2, 1.14, 1.11, 0.0), (-3.5, 1.16, 1.13, 0.0), (1.5, 1.16, 1.13, 0.0),
    (2.6, 1.12, 1.08, 0.0), (3.6, 0.99, 0.95, 0.02), (4.5, 0.80, 0.77, 0.04),
    (5.05, 0.62, 0.60, 0.05), (5.2, 0.58, 0.56, 0.05),
]
SS2_FUSE_UV = (0.0, 0.0, 0.5, 1.0)
SS2_WING_UV = (0.5, 0.5, 1.0, 1.0)
SS2_TAIL_UV = (0.5, 0.25, 1.0, 0.5)
UV_INTERIOR = (0.50, 0.000, 0.625, 0.125)
UV_PANEL = (0.625, 0.000, 0.75, 0.125)
UV_NOZZLE = (0.50, 0.125, 0.625, 0.25)
UV_TYRE = (0.625, 0.125, 0.6875, 0.25)
UV_STRUT = (0.6875, 0.125, 0.75, 0.25)
UV_PLUME_CORE = (0.0, 0.5, 1.0, 1.0)
UV_PLUME_OUTER = (0.0, 0.0, 1.0, 0.5)
HINGE = np.array([0.0, -0.25, 2.2])

# Window centres as (theta deg from top toward +x, z, radius m)
SS2_COCKPIT_WINDOWS = [(-34, -7.45, 0.27), (34, -7.45, 0.27), (-66, -6.85, 0.21), (66, -6.85, 0.21),
                       (0, -6.4, 0.19)]
SS2_CABIN_WINDOWS = [(s * 62, z, 0.215) for s in (-1, 1) for z in (-4.4, -3.25, -2.1, -0.95, 0.2, 1.35)]
SS2_CABIN_WINDOWS += [(0, -3.2, 0.2), (0, -0.9, 0.2)]


def ss2_radius_at(z):
    st = np.array(SS2_FUSE)
    return float(PchipInterpolator(st[:, 0], st[:, 1])(z))


def build_ss2():
    parts = []
    P, UV, th, fz = body(SS2_FUSE, 96, 160, SS2_FUSE_UV, flatten=0.05)
    parts.append(grid_part("ss2_fuselage", P, UV))

    # Rocket nozzle: graphite outer skirt and dark interior.
    th2 = np.linspace(0, 2 * pi, 49)
    zs = np.array([5.05, 5.35, 5.75, 6.0])
    rs = np.array([0.56, 0.52, 0.58, 0.64])
    Pn = np.array([[(r * np.sin(t), 0.05 + r * np.cos(t), z) for t in th2] for z, r in zip(zs, rs)])
    u0, v0, u1, v1 = UV_NOZZLE
    Un, Vn = np.meshgrid(np.linspace(u0, u1, 49), np.linspace(v0, v1, len(zs)))
    parts.append(grid_part("ss2_nozzle", Pn, np.stack([Un, Vn], 2)))
    Pi = Pn[::-1].copy()
    Pi[..., 0] *= 0.93
    Pi[..., 1] = 0.05 + (Pi[..., 1] - 0.05) * 0.93
    Pi[0, :, 2] = 6.0
    Pi[-1, :, 2] = 5.3
    Pi[-1, :, :2] *= 0.3
    parts.append(grid_part("ss2_nozzle_inner", Pi, np.stack([Un, Vn], 2), flip=True))

    # Wing: fixed inboard strake + forward wing, feathering aft flaps.
    def le(x):
        return np.interp(x, [0.9, 1.6, 2.4, 3.2, 4.0], [-4.3, -2.1, -0.6, 0.55, 1.5])

    def y(x):
        return np.interp(x, [0.9, 4.0], [-0.38, -0.25])

    te = 4.4
    t_of = lambda x: np.interp(x, [0.9, 1.6, 4.0], [0.05, 0.08, 0.09])  # noqa: E731
    root = [(x, le(x), te - le(x), y(x), t_of(x), 0.01) for x in np.linspace(0.9, 1.25, 4)]
    w_root, _ = wing(root, SS2_WING_UV, close_tip=False)
    xs = np.linspace(1.25, 4.0, 18)
    fwd_secs = [(x, le(x), te - le(x), y(x), t_of(x), 0.015) for x in xs]
    # Forward part (LE -> hinge) and aft flap (hinge -> TE) built per section
    # with chord fractions so the split follows the straight hinge line.
    fixed, aft = [], []
    for x, l, c, yy, t, cam in fwd_secs:
        f = (HINGE[2] - l) / c
        fixed.append((x, l, c, yy, t, cam, f))
        aft.append((x, l, c, yy, t, cam, f))
    w_fixed = loft_partial(fixed, SS2_WING_UV, upto=True)
    w_aft = loft_partial(aft, SS2_WING_UV, upto=False)

    # Tail boom, fins, stabiliser (all feather with the aft flap).
    bz = [1.2, 1.8, 2.6, 6.0, 8.0, 8.7]
    br = [0.02, 0.22, 0.31, 0.31, 0.24, 0.05]
    boom_st = [(z, r, r, 0.0) for z, r in zip(bz, br)]
    Pb, UVb, _, _ = body(boom_st, 32, 40, SS2_TAIL_UV)
    Pb[..., 0] += 4.0
    Pb[..., 1] += -0.25
    boom = grid_part("ss2_boom", Pb, UVb)
    fin_secs = [(0.05, 5.4, 3.0, 0.0, 0.09, 0.0), (1.6, 6.5, 2.0, 0.0, 0.09, 0.0), (3.1, 7.3, 1.35, 0.0, 0.08, 0.0)]
    fin, _ = wing(fin_secs, SS2_TAIL_UV, n_span=10)
    fin = translate(rot_to_vertical(fin), (4.0, -0.25, 0))
    vfin_secs = [(0.2, 6.6, 1.9, 0.0, 0.09, 0.0), (1.1, 7.2, 1.3, 0.0, 0.08, 0.0)]
    vfin, _ = wing(vfin_secs, SS2_TAIL_UV, n_span=4)
    vfin = rot_to_vertical(vfin)
    vfin = vfin.transformed(lambda p: np.array([p[0] + 4.0, -p[1] - 0.25, p[2]]),
                            lambda n: np.array([n[0], -n[1], n[2]]))
    vt = np.array(vfin.i).reshape(-1, 3)
    vfin.i = list(vt[:, [0, 2, 1]].ravel())
    stab_secs = [(4.25, 6.55, 1.75, -0.22, 0.08, 0.0), (5.0, 7.15, 1.05, -0.2, 0.07, 0.0)]
    stab, _ = wing(stab_secs, SS2_TAIL_UV, n_span=5)

    right_feather = [w_aft, boom, fin, vfin, stab]
    left_feather = [mirror_x(p) for p in right_feather]
    feather = merge("ss2_feather", right_feather + left_feather)
    feather = translate(feather, -HINGE)
    feather.anim = [f"ANIM_trans {HINGE[0]} {HINGE[1]} {HINGE[2]} {HINGE[0]} {HINGE[1]} {HINGE[2]} 0 0 none",
                    "ANIM_rotate 1 0 0 0 -60 0 1 iias/ss2/feather"]

    fixed_wing = merge("ss2_wing", [w_root, w_fixed, mirror_x(w_root), mirror_x(w_fixed)])
    parts.append(fixed_wing)
    parts.append(feather)

    # Landing gear (nose skid + mains) slides into the fuselage.
    gear_parts = [
        strut("nose_leg", (0, -0.9, -6.3), (0, -1.45, -6.45), 0.07, UV_STRUT),
        box("nose_skid", -0.12, 0.12, -1.6, -1.45, -7.0, -5.9, UV_TYRE),
    ]
    for sx in (-1, 1):
        gear_parts.append(strut(f"main_leg{sx}", (sx * 0.85, -0.85, 1.0), (sx * 0.95, -1.27, 1.0), 0.08, UV_STRUT))
        gear_parts.append(wheel(f"main_wheel{sx}", sx * 1.0, -1.27, 1.0, 0.33, 0.24, UV_TYRE))
    gear = merge("ss2_gear", gear_parts)
    gear.anim = ["ANIM_hide 0 0.02 iias/ss2/gear", "ANIM_trans 0 0.75 0 0 0 0 0 1 iias/ss2/gear"]
    parts.append(gear)

    # Cockpit interior: inward-facing shell with real window openings, and an
    # instrument panel with three displays.
    sh = [(z, r * 0.94, ry * 0.94, cy) for z, r, ry, cy in SS2_FUSE if -8.2 <= z <= -3.5]
    Ps, UVs, ths, fzs = body([(-8.18, 0.36, 0.34, -0.03)] + sh, 120, 90, UV_INTERIOR)
    shell = grid_part("ss2_cockpit_shell", Ps, UVs, flip=True)
    windows = SS2_COCKPIT_WINDOWS + [w for w in SS2_CABIN_WINDOWS if w[1] < -3.5]

    def keep(c):
        z = c[2]
        t = np.degrees(np.arctan2(c[0], c[1]))
        r = ss2_radius_at(z)
        for wt, wz, wr in windows:
            dth = np.radians((t - wt + 180) % 360 - 180) * r
            if (dth / (wr + 0.035)) ** 2 + ((z - wz) / (wr + 0.035)) ** 2 < 1.0:
                return False
        return True

    cut_holes(shell, keep)
    parts.append(shell)
    parts.append(window_frames(windows))
    pnl = Part("ss2_panel")
    u0, v0, u1, v1 = UV_PANEL
    pnl.add_quad((-0.78, -0.42, -6.40), (0.78, -0.42, -6.40), (0.70, 0.12, -6.75), (-0.70, 0.12, -6.75),
                 (0, 0.55, 0.83), [(u0, v0), (u1, v0), (u1, v1), (u0, v1)])
    pnl.attrs = ["ATTR_light_level 0 1 iias/const/one"]
    parts.append(pnl)

    return parts


def window_frames(windows, width=0.075):
    """Smooth rings around each cockpit/cabin opening, seen from inside."""
    frame = Part("ss2_window_frames")
    u0, v0, u1, v1 = UV_STRUT
    phi = np.linspace(0, 2 * pi, 49)
    for wt, wz, wr in windows:
        rings = []
        for rho in (wr - 0.004, wr + width * 0.5, wr + width):
            ring = []
            for f in phi:
                z = wz + rho * np.sin(f)
                R = ss2_radius_at(z) * (0.935 if rho > wr else 0.95)
                th = np.radians(wt) + rho * np.cos(f) / R
                ring.append((R * np.sin(th), R * np.cos(th), z))
            rings.append(ring)
        P = np.array(rings)
        U, V = np.meshgrid(np.linspace(u0, u1, len(phi)), np.linspace(v0, v1, 3))
        g = Part("frame")
        g.add_grid(P, np.stack([U, V], 2))
        # Normals must face the cockpit (toward the fuselage axis).
        c = P[1, 0]
        n0 = np.array(g.v[len(phi)][3:6])
        if np.dot(n0, [c[0], c[1], 0]) > 0:
            g.v = [(x, y, z, -a, -b, -cc, ss, tt) for x, y, z, a, b, cc, ss, tt in g.v]
            tr = np.array(g.i).reshape(-1, 3)
            g.i = list(tr[:, [0, 2, 1]].ravel())
        off = len(frame.v)
        frame.v.extend(g.v)
        frame.i.extend(k + off for k in g.i)
    frame.attrs = ["ATTR_no_cull"]
    return frame


def loft_partial(secs, uv, upto):
    """Loft wing sections, keeping chord fraction [0, f] (upto) or [f, 1]."""
    loops = []
    for x, l, c, yy, t, cam, f in secs:
        af = airfoil(24, t, cam, 0.0, f) if upto else airfoil(14, t, cam, f, 1.0)
        loops.append(np.stack([np.full(len(af), x), yy + af[:, 1] * c, l + af[:, 0] * c], 1))
    P = np.array(loops)
    xmin, xmax = 0.9, 4.0
    zmin, zmax = -4.3, 4.4
    u0, v0, u1, v1 = uv
    U = u0 + (u1 - u0) * (P[..., 0] - xmin) / (xmax - xmin)
    V = v1 - (v1 - v0) * (P[..., 2] - zmin) / (zmax - zmin)
    part = Part("wing_partial")
    part.add_grid(P, np.stack([U, V], 2))
    _orient_outward(part, P)
    # Close the cut face (hinge) with a strip of quads and the tip with a fan.
    k = 0 if not upto else -1
    for j in range(P.shape[0] - 1):
        a0, b0 = P[j, 0], P[j, -1] if upto else P[j, 0]
        a1, b1 = P[j + 1, 0], P[j + 1, -1]
        if upto:
            # hinge end is at both ends of the loop (TE side of the partial airfoil)
            p0, p1, p2, p3 = P[j, 0], P[j + 1, 0], P[j + 1, -1], P[j, -1]
            n = (0, 0, 1)
        else:
            p0, p1, p2, p3 = P[j, 0], P[j + 1, 0], P[j + 1, -1], P[j, -1]
            n = (0, 0, -1)
        part.add_quad(p0, p1, p2, p3, n, [(u0, v0), (u0, v0), (u0, v0), (u0, v0)])
    del k, a0, b0, a1, b1
    loop = P[-1]
    c = loop.mean(0)
    uvl = np.stack([U[-1], V[-1]], 1)
    part.add_tri_fan(c, list(loop) + [loop[0]], (1, 0, 0), uvl.mean(0), list(uvl) + [uvl[0]])
    return part


def cone_grid(z0, z1, radii, n=40, uv=(0, 0, 1, 1), y0=0.05):
    zs = np.linspace(z0, z1, len(radii))
    th = np.linspace(0, 2 * pi, n + 1)
    P = np.array([[(r * np.sin(t), y0 + r * np.cos(t), z) for t in th] for z, r in zip(zs, radii)])
    u0, v0, u1, v1 = uv
    U, V = np.meshgrid(np.linspace(v0, v1, n + 1), np.linspace(u0, u1, len(radii)))
    # u runs along the plume, v around it.
    return P, np.stack([V * 0 + np.linspace(u0, u1, len(radii))[:, None], U], 2)


def build_plume():
    parts = []
    attrs = ["ATTR_no_cull", "ATTR_no_shadow", "ATTR_light_level 0 1 iias/const/one"]
    # Dense-air plume: tight bright core with shock diamonds, 16 m flame.
    core_r = []
    zc = np.linspace(0, 1, 48)
    for t in zc:
        diamonds = 0.06 * np.abs(np.sin(t * pi * 7)) * (1 - t)
        core_r.append(0.50 * (1 - t) ** 0.7 + 0.05 + diamonds)
    P, UV = cone_grid(5.95, 11.5, core_r, uv=UV_PLUME_CORE)
    core = grid_part("plume_core", P, UV)
    outer_r = [0.58 + 1.9 * t ** 0.8 for t in np.linspace(0, 1, 30)]
    P, UV = cone_grid(5.95, 24.0, outer_r, uv=UV_PLUME_OUTER)
    outer = grid_part("plume_outer", P, UV)
    low = merge("plume_low", [outer, core],
                anim=["ANIM_show 0.5 1.5 iias/ss2/plume_mode",
                      "ANIM_trans 0 0 0 0 0 0.35 0 1 iias/ss2/plume_flicker"], attrs=attrs)
    # Thin-air plume: under-expanded, wide and translucent.
    hi_r = [0.6 + 9.0 * t ** 0.65 for t in np.linspace(0, 1, 40)]
    P, UV = cone_grid(5.95, 60.0, hi_r, uv=UV_PLUME_OUTER)
    wide = grid_part("plume_wide", P, UV)
    core_hi = [0.55 * (1 - t) + 0.08 for t in np.linspace(0, 1, 20)]
    P, UV = cone_grid(5.95, 14.0, core_hi, uv=UV_PLUME_CORE)
    corehi = grid_part("plume_core_hi", P, UV)
    high = merge("plume_high", [wide, corehi],
                 anim=["ANIM_show 1.5 2.5 iias/ss2/plume_mode",
                       "ANIM_trans 0 0 0 0 0 0.6 0 1 iias/ss2/plume_flicker"], attrs=attrs)
    low.anim.insert(0, "ANIM_hide -1 0.5 iias/ss2/plume_mode")
    low.anim.insert(1, "ANIM_hide 1.5 9 iias/ss2/plume_mode")
    high.anim.insert(0, "ANIM_hide -1 1.5 iias/ss2/plume_mode")
    parts.extend([low, high])
    return parts


# ---------------------------------------------------------- WhiteKnightTwo

WK2_SPACING = 6.6
WK2_FUSE = [
    (-11.0, 0.0, 0.0, -0.1), (-10.8, 0.28, 0.26, -0.08), (-10.2, 0.62, 0.58, -0.04),
    (-9.2, 0.92, 0.88, 0.0), (-8.0, 1.10, 1.06, 0.0), (-6.5, 1.16, 1.13, 0.0),
    (3.5, 1.16, 1.13, 0.0), (6.5, 0.95, 0.92, 0.12), (9.5, 0.62, 0.58, 0.32),
    (12.2, 0.32, 0.30, 0.48), (13.2, 0.10, 0.10, 0.52),
]
WK2_FUSE_UV = (0.0, 0.0, 0.5, 1.0)
WK2_WING_UV = (0.5, 0.5, 1.0, 1.0)
WK2_TAIL_UV = (0.5, 0.25, 0.75, 0.5)
WK2_NACELLE_UV = (0.75, 0.25, 1.0, 0.5)
WK2_DARK_UV = (0.5, 0.125, 0.625, 0.25)
WK2_TYRE_UV = (0.625, 0.125, 0.6875, 0.25)
WK2_STRUT_UV = (0.6875, 0.125, 0.75, 0.25)
WK2_COCKPIT_WINDOWS = [(-38, -9.35, 0.24), (38, -9.35, 0.24), (-72, -8.6, 0.2), (72, -8.6, 0.2), (0, -8.3, 0.18)]
WK2_CABIN_WINDOWS = [(s * 64, z, 0.2) for s in (-1, 1) for z in (-6.3, -5.2, -4.1, -3.0)]


def build_wk2():
    parts = []
    fus = []
    for sx in (-1, 1):
        P, UV, _, _ = body(WK2_FUSE, 80, 140, WK2_FUSE_UV, center_x=sx * WK2_SPACING, flatten=0.04)
        fus.append(grid_part(f"wk2_fuse{sx}", P, UV))
    parts.extend(fus)
    # High wing across the top of both fuselages.
    def le(x):
        return np.interp(x, [0, WK2_SPACING, 21.5], [-2.1, -2.1, -0.9])

    def ch(x):
        return np.interp(x, [0, WK2_SPACING, 21.5], [3.7, 3.6, 1.35])

    def yy(x):
        return np.interp(x, [0, WK2_SPACING, 21.5], [1.35, 1.35, 1.75])

    def tt(x):
        return np.interp(x, [0, 21.5], [0.15, 0.12])

    secs = [(x, le(x), ch(x), yy(x), tt(x), 0.025) for x in np.linspace(0.0, 21.5, 30)]
    w, _ = wing(secs, WK2_WING_UV)
    parts.append(merge("wk2_wing", [w, mirror_x(w)]))
    # Release pylon / hook beam above SpaceShipTwo.
    parts.append(box("wk2_pylon", -0.35, 0.35, 0.45, 1.2, -1.2, 0.8, WK2_TAIL_UV))
    # Four turbofan nacelles on pylons under the outer wing.
    for xn in (9.4, 12.1):
        for sx in (-1, 1):
            st = [(-4.0, 0.0, 0.0, 0.0), (-3.85, 0.52, 0.52, 0.0), (-3.4, 0.58, 0.58, 0.0),
                  (-1.2, 0.56, 0.56, 0.0), (-0.3, 0.42, 0.42, 0.0), (0.1, 0.36, 0.36, 0.0)]
            P, UV, _, _ = body(st, 40, 30, WK2_NACELLE_UV)
            P[..., 0] += sx * xn
            P[..., 1] += 0.55
            parts.append(grid_part(f"nacelle{sx}{xn}", P, UV))
            parts.append(disk(f"intake{sx}{xn}", (sx * xn, 0.55, -3.86), 0.47, "z",
                              (0.5625, 0.1875), 0.05, sign=-1))
            parts.append(box(f"engpylon{sx}{xn}", sx * xn - 0.08, sx * xn + 0.08, 0.9, 1.35, -2.6, -0.6,
                             WK2_DARK_UV))
    # Twin tails.
    for sx in (-1, 1):
        fin_secs = [(0.6, 8.8, 4.0, 0.0, 0.10, 0.0), (2.4, 10.4, 2.6, 0.0, 0.09, 0.0),
                    (4.4, 11.9, 1.7, 0.0, 0.08, 0.0)]
        fin, _ = wing(fin_secs, WK2_TAIL_UV, n_span=10)
        parts.append(translate(rot_to_vertical(fin), (sx * WK2_SPACING, 0.0, 0)))
        for side in (-1, 1):
            stab_secs = [(0.25, 10.4, 2.3, 0.0, 0.09, 0.0), (3.8, 11.6, 1.15, 0.0, 0.08, 0.0)]
            stab, _ = wing(stab_secs, WK2_TAIL_UV, n_span=6)
            if side < 0:
                stab = mirror_x(stab)
            parts.append(translate(stab, (sx * WK2_SPACING, 0.62, 0)))
    # Landing gear.
    gear_parts = []
    for sx in (-1, 1):
        cx = sx * WK2_SPACING
        gear_parts.append(strut(f"wk2_nose_leg{sx}", (cx, -0.9, -7.3), (cx, -2.3, -7.3), 0.08, WK2_STRUT_UV))
        gear_parts.append(wheel(f"wk2_nose_wheel{sx}", cx, -2.27, -7.3, 0.33, 0.2, WK2_TYRE_UV))
        for wx in (-0.32, 0.32):
            gear_parts.append(strut(f"wk2_main_leg{sx}{wx}", (cx + wx * 0.6, -0.9, 0.6),
                                    (cx + wx, -2.15, 0.6), 0.1, WK2_STRUT_UV))
            gear_parts.append(wheel(f"wk2_main_wheel{sx}{wx}", cx + wx, -2.15, 0.6, 0.45, 0.3, WK2_TYRE_UV))
    gear = merge("wk2_gear", gear_parts)
    gear.anim = ["ANIM_hide 0 0.02 iias/wk2/gear", "ANIM_trans 0 1.3 0 0 0 0 0 1 iias/wk2/gear"]
    parts.append(gear)
    return parts


# ------------------------------------------------------------ probe, trail


def build_probe():
    parts = []
    st = [(-0.45, 0.0, 0.0, 0.0), (-0.40, 0.07, 0.07, 0.0), (-0.25, 0.11, 0.11, 0.0), (0.35, 0.11, 0.11, 0.0),
          (0.42, 0.09, 0.09, 0.0)]
    P, UV, _, _ = body(st, 24, 16, (0.0, 0.5, 0.5, 1.0))
    parts.append(grid_part("probe_body", P, UV))
    for k in range(4):
        a = k * pi / 2
        fin = Part(f"fin{k}")
        r0, r1 = 0.11, 0.30
        p0 = (r0 * np.sin(a), r0 * np.cos(a), 0.15)
        p1 = (r1 * np.sin(a), r1 * np.cos(a), 0.28)
        p2 = (r1 * np.sin(a), r1 * np.cos(a), 0.42)
        p3 = (r0 * np.sin(a), r0 * np.cos(a), 0.42)
        nrm = (np.cos(a), -np.sin(a), 0)
        fin.add_quad(p0, p1, p2, p3, nrm, [(0.6, 0.1), (0.7, 0.1), (0.7, 0.2), (0.6, 0.2)])
        fin.attrs = ["ATTR_no_cull"]
        parts.append(fin)
    beacon = disk("beacon", (0, 0.115, -0.05), 0.035, "z", (0.875, 0.125), 0.05)
    beacon.anim = ["ANIM_hide 0.5 2 iias/probe/strobe"]
    beacon.attrs = ["ATTR_light_level 0 1 iias/const/one", "ATTR_no_cull"]
    parts.append(beacon)
    # Drogue parachute shown below 6 km.
    th = np.linspace(0, 2 * pi, 25)
    canopy = np.array([[(r * np.sin(t), 0, z) for t in th] for z, r in ((-4.0, 0.05), (-3.4, 1.0), (-3.0, 1.35))])
    canopy[..., 1] = canopy[..., 0] * 0  # flat in y; canopy axis along -z (ahead of motion)
    U, V = np.meshgrid(np.linspace(0.5, 1.0, 25), [0.5, 0.75, 1.0])
    chute = Part("chute", ["ANIM_hide -1 0.5 iias/probe/chute"], ["ATTR_no_cull"])
    chute.add_grid(np.stack([canopy[..., 0], canopy[..., 0] * 0 + np.array([0.05, 1.0, 1.35])[:, None] *
                             np.cos(th)[None, :], canopy[..., 2]], 2), np.stack([U, V], 2))
    parts.append(chute)
    return parts


def build_trail():
    """Smoke segment along -z (25 m) with size variants chosen by age."""
    parts = []
    for k, (r, a0, a1) in enumerate(((2.0, -1, 4), (4.5, 4, 15), (8.0, 15, 45), (13.0, 45, 140))):
        th = np.linspace(0, 2 * pi, 17)
        zs = np.linspace(-30.0, 30.0, 4)
        rr = [r * 0.7, r, r, r * 0.7]
        P = np.array([[(q * np.sin(t), q * np.cos(t), z) for t in th] for z, q in zip(zs, rr)])
        U, V = np.meshgrid(np.linspace(0, 1, 17), np.linspace(0, 1, 4))
        p = grid_part(f"trail{k}", P, np.stack([U, V], 2),
                      anim=[f"ANIM_show {a0} {a1} iias/trail/age", f"ANIM_hide -9 {a0} iias/trail/age",
                            f"ANIM_hide {a1} 99999 iias/trail/age"],
                      attrs=["ATTR_no_cull", "ATTR_no_shadow"])
        parts.append(p)
    return parts


# ----------------------------------------------------------------- textures


def paint_windows(img_rgb, nrm, gloss, lit, windows, radius_fn, uv_rect, zmin, zmax, frame=(150, 155, 162)):
    d = ImageDraw.Draw(img_rgb)
    dn = ImageDraw.Draw(nrm)
    dg = ImageDraw.Draw(gloss)
    u0, v0, u1, v1 = uv_rect
    for th, z, r in windows:
        R = radius_fn(z)
        cu = u0 + (u1 - u0) * ((th % 360) / 360.0)
        cv = v0 + (v1 - v0) * (z - zmin) / (zmax - zmin)
        ru = (u1 - u0) * r / (2 * pi * R)
        rv = (v1 - v0) * r / (zmax - zmin)
        px, py = cu * TEX, (1 - cv) * TEX
        rx, ry = ru * TEX, rv * TEX
        d.ellipse((px - rx * 1.22, py - ry * 1.22, px + rx * 1.22, py + ry * 1.22), fill=frame)
        d.ellipse((px - rx, py - ry, px + rx, py + ry), fill=(16, 22, 32))
        d.ellipse((px - rx * 0.7, py - ry * 0.85, px - rx * 0.1, py - ry * 0.35), fill=(40, 52, 70))
        dn.ellipse((px - rx * 1.22, py - ry * 1.22, px + rx * 1.22, py + ry * 1.22), outline=(150, 128, 0, 255),
                   width=3)
        dg.ellipse((px - rx, py - ry, px + rx, py + ry), fill=250)


def panel_lines(draw_n, rect, spacing_px, vertical=True, horizontal=True):
    x0, y0, x1, y1 = rect
    if vertical:
        for x in range(int(x0), int(x1), spacing_px):
            draw_n.line((x, y0, x, y1), fill=(110, 128, 0), width=2)
    if horizontal:
        for y in range(int(y0), int(y1), spacing_px):
            draw_n.line((x0, y, x1, y), fill=(128, 110, 0), width=2)


def make_textures(prefix, fuse_stations, windows_cockpit, windows_cabin, name_text, wing_uv, tail_uv,
                  stripe=(32, 74, 140), nacelle=False):
    base = Image.new("RGB", (TEX, TEX), (214, 217, 221))
    rng = np.random.default_rng(7)
    noise = (rng.normal(0, 2.2, (TEX, TEX))).astype(np.int16)
    arr = np.asarray(base).astype(np.int16) + noise[..., None]
    base = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    nrm = Image.new("RGB", (TEX, TEX), (128, 128, 0))   # NORMAL_METALNESS: B = metalness
    gloss = Image.new("L", (TEX, TEX), 175)
    lit = Image.new("RGB", (TEX, TEX), (0, 0, 0))
    d = ImageDraw.Draw(base)
    dn = ImageDraw.Draw(nrm)
    st = np.array(fuse_stations)
    zmin, zmax = st[0, 0], st[-1, 0]
    rfn = lambda z: float(PchipInterpolator(st[:, 0], st[:, 1])(z))  # noqa: E731
    fu = (0, 0, 0.5 * TEX, TEX)
    # Fuselage panel lines (frames every ~1.5 m) and a lower stripe.
    for z in np.arange(zmin + 2.0, zmax - 0.5, 1.55):
        y = (1 - (z - zmin) / (zmax - zmin)) * TEX
        dn.line((0, y, 0.5 * TEX, y), fill=(128, 104, 0), width=2)
    for k in range(1, 8):
        x = k * 0.5 * TEX / 8
        dn.line((x, 0.08 * TEX, x, 0.9 * TEX), fill=(104, 128, 0), width=2)
    # Thermal-protection grey on the nose cap and belly.
    for th in range(130, 231):
        x = th / 360 * 0.5 * TEX
        d.line((x, 0, x, TEX), fill=(214, 216, 220))
    d.rectangle((0, TEX * 0.965, 0.5 * TEX, TEX), fill=(70, 74, 80))
    # Cheat line along each side.
    for side in (90, 270):
        x = side / 360 * 0.5 * TEX
        d.rectangle((x - 3, TEX * 0.1, x + 3, TEX * 0.92), fill=stripe)
        d.rectangle((x + 9, TEX * 0.12, x + 11, TEX * 0.9), fill=(150, 160, 175))
    paint_windows(base, nrm, gloss, lit, windows_cockpit + windows_cabin, rfn, (0, 0, 0.5, 1.0), zmin, zmax)
    # Name and registration (rotated to run along the fuselage).
    font = ImageFont.truetype(FONT, 46)
    small = ImageFont.truetype(FONT, 30)
    for side, sgn in ((76, 1), (284, -1)):
        txt = Image.new("RGBA", (900, 70), (0, 0, 0, 0))
        ImageDraw.Draw(txt).text((0, 6), name_text, font=font, fill=(*stripe, 255))
        txt = txt.rotate(90 * sgn, expand=True)
        x = int(side / 360 * 0.5 * TEX) - txt.width // 2
        base.paste(txt, (x, int(TEX * 0.42)), txt)
        reg = Image.new("RGBA", (300, 44), (0, 0, 0, 0))
        ImageDraw.Draw(reg).text((0, 4), "N270NL", font=small, fill=(60, 64, 70, 255))
        reg = reg.rotate(90 * sgn, expand=True)
        base.paste(reg, (int((side + 6 * sgn) / 360 * 0.5 * TEX) - reg.width // 2, int(TEX * 0.82)), reg)
    # Wings and tail: white composite with panel lines; dark leading edges.
    wx0, wy0 = wing_uv[0] * TEX, (1 - wing_uv[3]) * TEX
    wx1, wy1 = wing_uv[2] * TEX, (1 - wing_uv[1]) * TEX
    panel_lines(dn, (wx0, wy0, wx1, wy1), 96)
    d.rectangle((wx0, wy0, wx1, wy0 + 14), fill=(92, 96, 104))
    tx0, ty0 = tail_uv[0] * TEX, (1 - tail_uv[3]) * TEX
    tx1, ty1 = tail_uv[2] * TEX, (1 - tail_uv[1]) * TEX
    panel_lines(dn, (tx0, ty0, tx1, ty1), 80)
    d.rectangle((tx0, ty0 + (ty1 - ty0) * 0.55, tx1, ty0 + (ty1 - ty0) * 0.62), fill=stripe)
    if nacelle:
        nx0, ny0 = 0.75 * TEX, 0.5 * TEX
        d.rectangle((nx0, ny0, TEX, 0.75 * TEX), fill=(200, 204, 210))
        d.rectangle((nx0, ny0, nx0 + 30, 0.75 * TEX), fill=(120, 125, 132))
        ImageDraw.Draw(nrm).rectangle((nx0, ny0, TEX, 0.75 * TEX), fill=(128, 128, 160))
    # Interior fabric, panel, graphite, tyres, struts.
    def rect(uv, colour, metal=0, gl=None):
        x0, y0, x1, y1 = uv[0] * TEX, (1 - uv[3]) * TEX, uv[2] * TEX, (1 - uv[1]) * TEX
        d.rectangle((x0, y0, x1, y1), fill=colour)
        ImageDraw.Draw(nrm).rectangle((x0, y0, x1, y1), fill=(128, 128, metal))
        if gl is not None:
            ImageDraw.Draw(gloss).rectangle((x0, y0, x1, y1), fill=gl)
        return x0, y0, x1, y1

    rect(UV_INTERIOR, (52, 55, 60), 0, 60)
    x0, y0, x1, y1 = rect(UV_PANEL, (28, 30, 34), 0, 90)
    dl = ImageDraw.Draw(lit)
    w3 = (x1 - x0) / 3
    for k in range(3):
        sx0 = x0 + k * w3 + 10
        d.rectangle((sx0, y0 + 30, sx0 + w3 - 20, y1 - 40), fill=(8, 10, 14))
        dl.rectangle((sx0 + 4, y0 + 34, sx0 + w3 - 24, y1 - 44), fill=(10, 30, 55))
        dl.line((sx0 + 8, (y0 + y1) / 2, sx0 + w3 - 28, (y0 + y1) / 2), fill=(40, 160, 255), width=3)
        dl.rectangle((sx0 + 12, y0 + 44, sx0 + 40, y1 - 56), outline=(0, 200, 120), width=2)
    rect(UV_NOZZLE, (38, 38, 42), 200, 120)
    rect(UV_TYRE, (24, 24, 26), 0, 40)
    rect(UV_STRUT, (150, 152, 158), 220, 190)
    alpha = Image.new("L", (TEX, TEX), 255)
    nrm = nrm.filter(ImageFilter.GaussianBlur(1.2))
    rgba = base.convert("RGBA")
    rgba.putalpha(alpha)
    rgba.save(ASSETS / f"{prefix}.png")
    n_rgba = nrm.convert("RGBA")
    n_rgba.putalpha(gloss)
    n_rgba.save(ASSETS / f"{prefix}_NML.png")
    lit.save(ASSETS / f"{prefix}_LIT.png")


def make_plume_texture():
    """Hybrid-motor flame: white-hot core fading to orange; outer plume
    translucent orange-to-transparent. u runs from the nozzle aft."""
    W, H = 512, 128
    img = np.zeros((H, W, 4), np.float32)
    lit = np.zeros((H, W, 3), np.float32)
    u = np.linspace(0, 1, W)
    for band, (hot, cool, a0, rows) in enumerate((((1.0, 0.98, 0.92), (1.0, 0.62, 0.22), 0.95, slice(0, H // 2)),
                                                  ((1.0, 0.78, 0.45), (0.85, 0.32, 0.10), 0.42, slice(H // 2, H)))):
        col = np.array(hot)[None, :] * (1 - u[:, None]) + np.array(cool)[None, :] * u[:, None]
        alpha = a0 * (1 - u) ** (1.1 if band == 0 else 1.6)
        # Shock-diamond brightening in the core.
        if band == 0:
            alpha = np.clip(alpha * (0.85 + 0.25 * np.abs(np.sin(u * pi * 7))), 0, 1)
        img[rows, :, :3] = col[None]
        img[rows, :, 3] = alpha[None]
        lit[rows] = (col * alpha[:, None])[None]
    Image.fromarray((img * 255).astype(np.uint8), "RGBA").save(ASSETS / "plume.png")
    Image.fromarray((lit * 255).clip(0, 255).astype(np.uint8), "RGB").save(ASSETS / "plume_LIT.png")


def make_small_textures():
    # Probe: anodised aluminium, orange band, beacon emissive.
    img = Image.new("RGBA", (256, 256), (200, 204, 210, 255))
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, 128, 40), fill=(230, 110, 30, 255))
    d.rectangle((150, 200, 256, 256), fill=(255, 40, 40, 255))
    d.rectangle((128, 0, 256, 128), fill=(235, 120, 40, 255))
    img.save(ASSETS / "probe.png")
    lit = Image.new("RGB", (256, 256), (0, 0, 0))
    ImageDraw.Draw(lit).rectangle((200, 200, 256, 256), fill=(255, 60, 60))
    lit.save(ASSETS / "probe_LIT.png")
    # Smoke: soft grey with transparent ends and edges.
    W, H = 128, 128
    a = np.zeros((H, W), np.float32)
    for y in range(H):
        for x in range(W):
            edge = np.sin(pi * y / (H - 1))
            a[y, x] = 0.30 * edge ** 1.2
    rng = np.random.default_rng(3)
    a *= 0.75 + 0.25 * rng.random((H, W))
    rgb = np.full((H, W, 3), 222, np.uint8)
    img = Image.fromarray(np.dstack([rgb, (a * 255).astype(np.uint8)]), "RGBA")
    img.filter(ImageFilter.GaussianBlur(2)).save(ASSETS / "trail.png")


def main():
    ss2 = build_ss2()
    v, t = write_obj(ASSETS / "spaceship2.obj", ss2,
                     {"TEXTURE": "ss2.png", "TEXTURE_LIT": "ss2_LIT.png", "TEXTURE_NORMAL": "ss2_NML.png"},
                     header_extra=["NORMAL_METALNESS", "GLOBAL_specular 1.0"], lod=120000)
    print(f"spaceship2.obj: {v} vertices, {t} triangles")
    make_textures("ss2", SS2_FUSE, SS2_COCKPIT_WINDOWS, SS2_CABIN_WINDOWS, "IIAS  NOCTILUCENT", SS2_WING_UV,
                  SS2_TAIL_UV)
    wk2 = build_wk2()
    v, t = write_obj(ASSETS / "whiteknight2.obj", wk2,
                     {"TEXTURE": "wk2.png", "TEXTURE_LIT": "wk2_LIT.png", "TEXTURE_NORMAL": "wk2_NML.png"},
                     header_extra=["NORMAL_METALNESS", "GLOBAL_specular 1.0"], lod=120000)
    print(f"whiteknight2.obj: {v} vertices, {t} triangles")
    make_textures("wk2", WK2_FUSE, WK2_COCKPIT_WINDOWS, WK2_CABIN_WINDOWS, "IIAS  CARRIER", WK2_WING_UV,
                  WK2_TAIL_UV, nacelle=True)
    plume = build_plume()
    for p in plume:
        p.attrs = [a for a in p.attrs if not a.startswith("ATTR_light_level")] + ["ATTR_light_level 0 1 iias/const/one"]
        # Normals straight up: the flame is never self-shadowed.
        p.v = [(x, y, z, 0.0, 1.0, 0.0, s_, t_) for x, y, z, _, _, _, s_, t_ in p.v]
    write_obj(ASSETS / "plume.obj", plume, {"TEXTURE": "plume.png", "TEXTURE_LIT": "plume_LIT.png"},
              header_extra=["GLOBAL_no_shadow", "GLOBAL_luminance 9000"], lod=200000)
    make_plume_texture()
    probe = build_probe()
    write_obj(ASSETS / "probe.obj", probe, {"TEXTURE": "probe.png", "TEXTURE_LIT": "probe_LIT.png"},
              lod=60000)
    write_obj(ASSETS / "trail.obj", build_trail(), {"TEXTURE": "trail.png"},
              header_extra=["GLOBAL_no_shadow"], lod=200000)
    make_small_textures()
    no_plume = [p for p in ss2 if not p.name.startswith("plume") and "shell" not in p.name and p.name != "ss2_panel"]
    preview(PREVIEW / "ss2.png", no_plume)
    preview(PREVIEW / "wk2.png", wk2)
    print("previews in", PREVIEW)


if __name__ == "__main__":
    main()
