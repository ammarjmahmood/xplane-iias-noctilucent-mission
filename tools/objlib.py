"""Small mesh toolkit for writing X-Plane OBJ8 models.

Conventions (X-Plane OBJ8): +x right, +y up, +z aft (the nose points to -z).
Front faces are wound clockwise; texture t = 0 is the bottom row of the image.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class Part:
    name: str
    anim: list[str] = field(default_factory=list)   # ANIM_* lines (inside begin/end)
    attrs: list[str] = field(default_factory=list)  # ATTR_* lines before TRIS
    v: list = field(default_factory=list)
    i: list = field(default_factory=list)

    def add_grid(self, P, UV, flip=False, wrap_j=False, wrap_i=False):
        """Add a structured grid P[n, m, 3] with UV[n, m, 2] and smooth normals.

        Normals follow cross(dP/di, dP/dj); pass flip=True to reverse.
        """
        P = np.asarray(P, float)
        UV = np.asarray(UV, float)
        n, m, _ = P.shape
        if wrap_i:
            di = (np.roll(P, -1, 0) - np.roll(P, 1, 0))
        else:
            di = np.gradient(P, axis=0)
        if wrap_j:
            dj = (np.roll(P, -1, 1) - np.roll(P, 1, 1))
        else:
            dj = np.gradient(P, axis=1)
        N = np.cross(di, dj)
        if flip:
            N = -N
        mag = np.linalg.norm(N, axis=2, keepdims=True)
        N = np.where(mag > 1e-12, N / np.maximum(mag, 1e-12), np.array([0, 1, 0]))
        base = len(self.v)
        for a in range(n):
            for b in range(m):
                self.v.append((*P[a, b], *N[a, b], *UV[a, b]))
        for a in range(n - 1):
            for b in range(m - 1):
                p00 = base + a * m + b
                p10 = p00 + m
                p11 = p10 + 1
                p01 = p00 + 1
                self._tri(p00, p10, p11)
                self._tri(p00, p11, p01)

    def _tri(self, a, b, c):
        va, vb, vc = (np.array(self.v[k][:3]) for k in (a, b, c))
        n = np.cross(vb - va, vc - va)
        avg = np.array(self.v[a][3:6]) + np.array(self.v[b][3:6]) + np.array(self.v[c][3:6])
        if np.linalg.norm(n) < 1e-14:
            return
        # X-Plane front faces are clockwise: the geometric CCW normal must
        # oppose the shading normal.
        if np.dot(n, avg) > 0:
            self.i.extend((a, c, b))
        else:
            self.i.extend((a, b, c))

    def add_tri_fan(self, center, ring, normal, uv_center, uv_ring):
        base = len(self.v)
        self.v.append((*center, *normal, *uv_center))
        for p, uv in zip(ring, uv_ring):
            self.v.append((*p, *normal, *uv))
        k = len(ring)
        for j in range(k - 1):
            self._tri(base, base + 1 + j, base + 2 + j)

    def add_quad(self, a, b, c, d, normal, uvs):
        base = len(self.v)
        for p, uv in zip((a, b, c, d), uvs):
            self.v.append((*p, *normal, *uv))
        self._tri(base, base + 1, base + 2)
        self._tri(base, base + 2, base + 3)

    def transformed(self, fn_point, fn_normal):
        out = Part(self.name, list(self.anim), list(self.attrs))
        for x, y, z, nx, ny, nz, s, t in self.v:
            p = fn_point(np.array([x, y, z]))
            nn = fn_normal(np.array([nx, ny, nz]))
            out.v.append((*p, *nn, s, t))
        out.i = list(self.i)
        return out


def mirror_x(part: Part, name=None) -> Part:
    out = part.transformed(lambda p: p * [-1, 1, 1], lambda n: n * [-1, 1, 1])
    out.name = name or part.name + "_mirror"
    # Mirroring flips the winding; swap to keep front faces outward.
    tris = np.array(out.i).reshape(-1, 3)
    out.i = list(tris[:, [0, 2, 1]].ravel())
    return out


def translate(part: Part, d) -> Part:
    d = np.asarray(d, float)
    return part.transformed(lambda p: p + d, lambda n: n)


def write_obj(path: Path, parts: list[Part], textures: dict, header_extra=(), lod=None):
    verts = []
    idx = []
    ranges = []
    for p in parts:
        off = len(verts)
        verts.extend(p.v)
        start = len(idx)
        idx.extend(k + off for k in p.i)
        ranges.append((p, start, len(idx) - start))
    lines = ["A", "800", "OBJ", ""]
    for key, value in textures.items():
        lines.append(f"{key} {value}")
    lines.extend(header_extra)
    lines.append(f"POINT_COUNTS {len(verts)} 0 0 {len(idx)}")
    for v in verts:
        lines.append("VT " + " ".join(f"{c:.5f}" for c in v))
    full = len(idx) // 10 * 10
    for k in range(0, full, 10):
        lines.append("IDX10 " + " ".join(str(x) for x in idx[k:k + 10]))
    for k in range(full, len(idx)):
        lines.append(f"IDX {idx[k]}")
    lines.append("")
    if lod:
        lines.append(f"ATTR_LOD 0 {lod}")
    for p, start, count in ranges:
        if count == 0:
            continue
        lines.append(f"# {p.name}")
        if p.anim:
            lines.append("ANIM_begin")
            lines.extend(p.anim)
        lines.extend(p.attrs)
        lines.append(f"TRIS {start} {count}")
        if p.attrs:
            lines.extend(_reset(p.attrs))
        if p.anim:
            lines.append("ANIM_end")
    Path(path).write_text("\n".join(lines) + "\n")
    return len(verts), len(idx) // 3


def _reset(attrs):
    out = []
    for a in attrs:
        key = a.split()[0]
        if key == "ATTR_no_cull":
            out.append("ATTR_cull")
        elif key == "ATTR_light_level":
            out.append("ATTR_light_level_reset")
        elif key == "ATTR_no_shadow":
            out.append("ATTR_shadow")
        elif key == "ATTR_no_blend":
            out.append("ATTR_blend")
        elif key == "ATTR_draw_disable":
            out.append("ATTR_draw_enable")
    return out


def preview(path: Path, parts: list[Part], views=((25, -55), (89, 0), (0, 90), (12, -150)), size=520):
    """Painter's-algorithm shaded preview for checking geometry without X-Plane.

    views are (elevation, azimuth) in degrees; azimuth 0 looks from the nose.
    """
    from PIL import Image, ImageDraw
    V, N, T = [], [], []
    for p in parts:
        off = len(V)
        V.extend(v[:3] for v in p.v)
        N.extend(v[3:6] for v in p.v)
        T.extend(np.array(p.i).reshape(-1, 3) + off)
    V, N, T = np.array(V), np.array(N), np.array(T)
    tiles = []
    for elev, azim in views:
        e, a = np.radians(elev), np.radians(azim)
        # Camera direction (from target toward eye) in OBJ coordinates.
        eye = np.array([np.sin(a) * np.cos(e), np.sin(e), -np.cos(a) * np.cos(e)])
        up = np.array([0, 1, 0]) if abs(elev) < 80 else np.array([0, 0, -1])
        right = np.cross(up, eye); right /= np.linalg.norm(right)
        up2 = np.cross(eye, right)
        xs, ys, zs = V @ right, V @ up2, V @ eye
        span = max(xs.max() - xs.min(), ys.max() - ys.min()) * 1.05
        cx, cy = (xs.max() + xs.min()) / 2, (ys.max() + ys.min()) / 2
        sx = (xs - cx) / span * size + size / 2
        sy = size / 2 - (ys - cy) / span * size
        light = 0.6 * eye + np.array([0.3, 0.7, -0.2]); light /= np.linalg.norm(light)
        fn = N[T].mean(1)
        fn /= np.maximum(1e-9, np.linalg.norm(fn, axis=1))[:, None]
        facing = fn @ eye
        depth = zs[T].mean(1)
        order = np.argsort(depth)
        img = Image.new("RGB", (size, size), (30, 34, 44))
        d = ImageDraw.Draw(img)
        for k in order:
            if facing[k] < -0.05:
                continue
            sh = 0.25 + 0.75 * max(0.0, float(fn[k] @ light))
            c = int(40 + 210 * sh)
            tri = T[k]
            d.polygon([(sx[q], sy[q]) for q in tri], fill=(c, c, min(255, c + 8)))
        tiles.append(img)
    out = Image.new("RGB", (size * len(tiles), size))
    for k, t in enumerate(tiles):
        out.paste(t, (k * size, 0))
    out.save(path)
