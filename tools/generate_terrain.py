#!/usr/bin/env python3
"""Eielson AFB / Tanana Valley scenery for the mission plugin.

The X-Plane 12 demo carries no terrain for interior Alaska, so the plugin
draws its own: three nested, Earth-curved terrain meshes centred on the
Eielson runway 32 threshold, textured with satellite imagery, plus a marked
runway 14/32. The same height grids are exported for the flight model's
ground contact.

Data (downloaded once and cached in downloads/tiles):
  * Elevation: AWS Terrain Tiles (Terrarium encoding; SRTM/USGS 3DEP/GMTED
    derived), https://registry.opendata.aws/terrain-tiles/
  * Imagery: Sentinel-2 cloudless by EOX IT Services GmbH (contains
    modified Copernicus Sentinel data), https://s2maps.eu, CC BY-NC-SA 4.0
"""
from __future__ import annotations

import io
import math
import struct
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
from objlib import Part, write_obj  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / "downloads" / "tiles"
OUT = ROOT / "assets" / "scenery"
OUT.mkdir(parents=True, exist_ok=True)

LAT0, LON0 = 64.6471515, -147.0850382          # runway 32 threshold
END_LAT, END_LON = 64.6842042, -147.1179305     # runway 14 threshold
RWY_WIDTH = 45.0
R_EARTH = 6371000.0
M_LAT = 111132.0
M_LON = 111320.0 * math.cos(math.radians(LAT0))
ELEV_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
IMG_URL = "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless_3857/default/g/{z}/{y}/{x}.jpg"

# (name, half-size m, grid spacing m, imagery zoom, elevation zoom, texture px, drop under inner m)
LEVELS = [
    ("eielson_l0", 14000.0, 70.0, 13, 12, 4096, 0.0),
    ("eielson_l1", 80000.0, 500.0, 10, 9, 4096, 6.0),
    ("eielson_l2", 450000.0, 4000.0, 7, 6, 2048, 40.0),
]


def fetch(url: str, path: Path) -> bytes:
    if path.exists() and path.stat().st_size > 0:
        return path.read_bytes()
    path.parent.mkdir(parents=True, exist_ok=True)
    req = urllib.request.Request(url, headers={"User-Agent": "xplaneiias-scenery/1.0 (personal flight sim)"})
    for attempt in range(4):
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                data = r.read()
            path.write_bytes(data)
            time.sleep(0.05)
            return data
        except Exception as exc:  # noqa: BLE001
            if attempt == 3:
                raise RuntimeError(f"download failed {url}: {exc}") from exc
            time.sleep(1.5 * (attempt + 1))
    return b""


def merc(lat, lon, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1 - np.arcsinh(np.tan(np.radians(lat))) / math.pi) / 2 * n
    return x, y


class TileMosaic:
    """Bilinear sampler over a block of web-mercator tiles."""

    def __init__(self, kind, z, lat_min, lat_max, lon_min, lon_max):
        x0, y1 = merc(lat_min, lon_min, z)
        x1, y0 = merc(lat_max, lon_max, z)
        self.z = z
        self.tx0, self.ty0 = int(math.floor(x0)), int(math.floor(y0))
        tx1, ty1 = int(math.floor(x1)), int(math.floor(y1))
        nx, ny = tx1 - self.tx0 + 1, ty1 - self.ty0 + 1
        chans = 3
        self.img = np.zeros((ny * 256, nx * 256, chans), np.float32)
        total = nx * ny
        print(f"  {kind} z{z}: {nx}x{ny} tiles")
        k = 0
        for ty in range(self.ty0, ty1 + 1):
            for tx in range(self.tx0, tx1 + 1):
                k += 1
                if kind == "elev":
                    url = ELEV_URL.format(z=z, x=tx, y=ty)
                    p = CACHE / "elev" / str(z) / f"{tx}_{ty}.png"
                else:
                    url = IMG_URL.format(z=z, x=tx, y=ty)
                    p = CACHE / "img" / str(z) / f"{tx}_{ty}.jpg"
                data = fetch(url, p)
                im = Image.open(io.BytesIO(data)).convert("RGB")
                a = np.asarray(im, np.float32)
                if kind == "elev":
                    h = a[..., 0] * 256 + a[..., 1] + a[..., 2] / 256 - 32768
                    a = np.stack([h, h, h], 2)
                oy, ox = (ty - self.ty0) * 256, (tx - self.tx0) * 256
                self.img[oy:oy + 256, ox:ox + 256] = a
                if k % 50 == 0:
                    print(f"    {k}/{total}")

    def sample(self, lat, lon):
        x, y = merc(lat, lon, self.z)
        px = (x - self.tx0) * 256 - 0.5
        py = (y - self.ty0) * 256 - 0.5
        h, w, _ = self.img.shape
        px = np.clip(px, 0, w - 1.001)
        py = np.clip(py, 0, h - 1.001)
        x0 = np.floor(px).astype(int)
        y0 = np.floor(py).astype(int)
        fx = (px - x0)[..., None]
        fy = (py - y0)[..., None]
        a = self.img[y0, x0]
        b = self.img[y0, x0 + 1]
        c = self.img[y0 + 1, x0]
        d = self.img[y0 + 1, x0 + 1]
        return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def enu_to_ll(e, n):
    return LAT0 + n / M_LAT, LON0 + e / M_LON


# Runway geometry in ENU (east, north) metres from the 32 threshold.
RWY_END = np.array([(END_LON - LON0) * M_LON, (END_LAT - LAT0) * M_LAT])
RWY_LEN = float(np.linalg.norm(RWY_END))
RWY_DIR = RWY_END / RWY_LEN
RWY_RIGHT = np.array([RWY_DIR[1], -RWY_DIR[0]])


def runway_frame(E, N):
    along = E * RWY_DIR[0] + N * RWY_DIR[1]
    cross = E * RWY_RIGHT[0] + N * RWY_RIGHT[1]
    return along, cross


def runway_profile(elev_mosaic):
    s = np.linspace(-400, RWY_LEN + 400, 60)
    e = s * RWY_DIR[0]
    n = s * RWY_DIR[1]
    lat, lon = enu_to_ll(e, n)
    h = elev_mosaic.sample(lat, lon)[..., 0]
    # A runway is graded smooth: least-squares straight line.
    k = np.polyfit(s, h, 1)
    return k


def build_level(name, half, step, zi, ze, tex, drop, rwy_fit, inner_half):
    print(f"{name}: {2 * half / 1000:.0f} km, {step:.0f} m grid")
    margin = 0.02 * half + step * 2
    lat_min, lon_min = enu_to_ll(-half - margin, -half - margin)
    lat_max, lon_max = enu_to_ll(half + margin, half + margin)
    elev = TileMosaic("elev", ze, lat_min, lat_max, lon_min, lon_max)
    img = TileMosaic("img", zi, lat_min, lat_max, lon_min, lon_max)
    # Height grid.
    s = np.arange(-half, half + step * 0.5, step)
    E, N = np.meshgrid(s, s)          # rows: north increasing
    lat, lon = enu_to_ll(E, N)
    H = elev.sample(lat, lon)[..., 0]
    H = np.where(H < -50, 0.0, H)
    # Flatten the runway strip and its shoulders onto the graded profile.
    along, cross = runway_frame(E, N)
    target = rwy_fit[0] * np.clip(along, -400, RWY_LEN + 400) + rwy_fit[1]
    w = np.clip(1 - (np.abs(cross) - 90) / 160, 0, 1) * np.clip(1 - (np.maximum(-along, along - RWY_LEN) - 300) / 400, 0, 1)
    H = H * (1 - w) + target * w
    H = H - drop
    # Texture: imagery resampled onto the ENU grid.
    t = np.linspace(-half, half, tex)
    TE, TN = np.meshgrid(t, t[::-1])  # image row 0 = north edge
    tlat, tlon = enu_to_ll(TE, TN)
    rgb = img.sample(tlat, tlon)
    # Gentle grade: Sentinel mosaics are bright; deepen for X-Plane lighting.
    rgb = np.clip((rgb / 255.0) ** 1.12 * 255 * 0.92, 0, 255).astype(np.uint8)
    Image.fromarray(rgb).save(OUT / f"{name}.png")
    # Mesh in OBJ space: x east, z south, y up (with Earth curvature).
    curv = -(E ** 2 + N ** 2) / (2 * R_EARTH)
    P = np.stack([E, H + curv, -N], 2)
    U = (E + half) / (2 * half)
    V = (N + half) / (2 * half)
    part = Part(name, attrs=[])
    # Rows increase north (-z), columns east (+x): cross(d/drow, d/dcol) = (-z)x(+x) -> -y; flip for up.
    part.add_grid(P, np.stack([U, V], 2), flip=True)
    if inner_half:
        # Cut the cells covered by the finer level (keep one cell of overlap).
        tris = np.array(part.i).reshape(-1, 3)
        V3 = np.array([v[:3] for v in part.v])
        c = V3[tris].mean(1)
        lim = inner_half - step * 1.5
        keep = ~((np.abs(c[:, 0]) < lim) & (np.abs(c[:, 2]) < lim))
        part.i = list(tris[keep].ravel())
    lod = 900000
    write_obj(OUT / f"{name}.obj", [part], {"TEXTURE": f"{name}.png"},
              header_extra=["GLOBAL_no_shadow"], lod=lod)
    # Height grid for ground contact: int16 metres, rows north-increasing.
    with open(OUT / f"{name}.hgt", "wb") as f:
        f.write(struct.pack("<iff", len(s), half, step))
        f.write(np.round(H + drop).astype("<i2").tobytes())
    print(f"  {len(part.v)} vertices, {len(part.i) // 3} triangles")


def runway_texture():
    """Asphalt with FAA markings. Columns span the 45 m width; rows run
    along the runway. Rows 0-1535: threshold zone (360 m); 1536-2047: a
    repeating 120 m centreline section."""
    W, H = 256, 2048
    m_per_px_x = RWY_WIDTH / W
    rng = np.random.default_rng(5)
    base = 58 + rng.normal(0, 6, (H, W))
    base += np.linspace(0, 6, W)[None, :] * 0
    img = Image.fromarray(np.clip(np.dstack([base, base, base * 1.02]), 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    white = (232, 232, 228)
    def mx(m):
        return m / m_per_px_x
    thr_px = 1536 / 360.0      # px per metre along (threshold zone)
    def my(m):
        return 1535 - m * thr_px  # row for distance m from threshold (texture rows go from far to near)
    # Edge stripes.
    for x0 in (mx(1.0), mx(RWY_WIDTH - 1.9)):
        d.rectangle((x0, 0, x0 + mx(0.9), H), fill=white)
    # Threshold piano keys: 8 stripes each side of centre, 45 m long.
    stripe_w, gap = 1.75, 1.15
    for side in (-1, 1):
        for k in range(4):
            c = RWY_WIDTH / 2 + side * (2.0 + k * (stripe_w + gap) + stripe_w / 2)
            d.rectangle((mx(c - stripe_w / 2), my(51), mx(c + stripe_w / 2), my(6)), fill=white)
        for k in range(4, 8):
            c = RWY_WIDTH / 2 + side * (2.0 + k * (stripe_w + gap) + stripe_w / 2)
            if c - stripe_w / 2 > 2.6 and c + stripe_w / 2 < RWY_WIDTH - 2.6:
                d.rectangle((mx(c - stripe_w / 2), my(51), mx(c + stripe_w / 2), my(6)), fill=white)
    # Threshold bar.
    d.rectangle((mx(2.5), my(4), mx(RWY_WIDTH - 2.5), my(1)), fill=white)
    # Designation numerals are drawn per runway end on a separate overlay.
    # Touchdown zone bars and aiming point.
    for dist, pairs in ((150, 3), (300, 2)):
        for side in (-1, 1):
            for k in range(pairs):
                c = RWY_WIDTH / 2 + side * (5.0 + k * 2.6)
                d.rectangle((mx(c - 0.9), my(dist + 22.5), mx(c + 0.9), my(dist)), fill=white)
    for side in (-1, 1):
        c = RWY_WIDTH / 2 + side * 7.5
        d.rectangle((mx(c - 2.5), my(345), mx(c + 2.5), my(300)), fill=white)
    # Centreline dashes in the threshold zone after the numerals.
    for start in (120, 240):
        d.rectangle((mx(RWY_WIDTH / 2 - 0.45), my(start + 36), mx(RWY_WIDTH / 2 + 0.45), my(start)), fill=white)
    # Repeating section: 36 m dash, 84 m gap (rows 1536..2047 = 120 m).
    rep = 512 / 120.0
    d.rectangle((mx(RWY_WIDTH / 2 - 0.45), 1536 + 10, mx(RWY_WIDTH / 2 + 0.45), 1536 + 10 + 36 * rep), fill=white)
    # Tyre marks in the touchdown zones.
    marks = Image.new("L", (W, H), 0)
    dm = ImageDraw.Draw(marks)
    for k in range(260):
        x = RWY_WIDTH / 2 + rng.normal(0, 3.2)
        y0 = rng.uniform(250, 360)
        dm.line((mx(x), my(y0), mx(x + rng.normal(0, 0.3)), my(y0 - rng.uniform(5, 40))), fill=int(rng.uniform(30, 90)), width=3)
    marks = marks.filter(ImageFilter.GaussianBlur(1.5))
    arr = np.asarray(img).astype(np.float32)
    arr *= (1 - np.asarray(marks, np.float32)[..., None] / 255 * 0.55)
    img = Image.fromarray(arr.clip(0, 255).astype(np.uint8))
    # Numerals area: drawn per end into two extra 256x256 cells appended below.
    full = Image.new("RGB", (W * 2, H), (0, 0, 0))
    full.paste(img, (0, 0))
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 200)
    for k, label in enumerate(("32", "14")):
        cell = Image.fromarray(np.clip(np.dstack([base[:1024], base[:1024], base[:1024] * 1.02]), 0, 255).astype(np.uint8))
        cell = cell.resize((W, 1024))
        dc = ImageDraw.Draw(cell)
        for x0 in (mx(1.0), mx(RWY_WIDTH - 1.9)):
            dc.rectangle((x0, 0, x0 + mx(0.9), 1024), fill=white)
        txt = Image.new("L", (300, 260), 0)
        ImageDraw.Draw(txt).text((150, 130), label, font=font, fill=255, anchor="mm")
        # Numerals are 18 m long on the runway: stretch along the runway axis.
        txt = txt.resize((int(mx(20)), int(18 * 1024 / 60.0)))
        cx = (W - txt.width) // 2
        cy = 1024 - int(42 * 1024 / 60.0)  # 6..60 m band, numerals ~24 m from threshold
        cell.paste(Image.new("RGB", txt.size, white), (cx, cy), txt)
        full.paste(cell, (W, k * 1024))
    full.save(OUT / "runway.png")


def runway_obj(rwy_fit):
    """Runway mesh 3 cm above the graded strip, with ATTR_poly_os."""
    parts = []
    p = Part("runway", attrs=["ATTR_poly_os 2"])
    segs = []
    half = RWY_WIDTH / 2

    def pt(s, c):
        e = s * RWY_DIR[0] + c * RWY_RIGHT[0]
        n = s * RWY_DIR[1] + c * RWY_RIGHT[1]
        h = rwy_fit[0] * s + rwy_fit[1] + 0.05 - (e * e + n * n) / (2 * R_EARTH)
        return (e, h, -n)

    def quad(s0, s1, u0, u1, v0, v1, flip=False):
        a, b, c, d = pt(s0, -half), pt(s0, half), pt(s1, half), pt(s1, -half)
        uv = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
        if flip:
            uv = [(u1, v1), (u0, v1), (u0, v0), (u1, v0)]
        p.add_quad(a, b, c, d, (0, 1, 0), uv)

    # Texture atlas: left half markings (u 0..0.5), right half numerals.
    vt = lambda row: 1 - row / 2048.0  # noqa: E731
    # Each end: 0-6 m blank+bar..., 6-60 m numerals cell, then threshold-zone markings to 360 m.
    for end, (s_start, sign) in enumerate(((0.0, 1), (RWY_LEN, -1))):
        def S(m):
            return s_start + sign * m
        # Threshold zone (0..360 m from this threshold): rows 1535 (0 m) .. 0 (360 m)
        rows_per_m = 1536 / 360.0
        # piano keys etc 0..58 m, then numerals 58..80 m drawn from numerals cell, rest 80..360
        for (m0, m1) in ((0, 58), (80, 360)):
            r0, r1 = 1535 - m0 * rows_per_m, 1535 - m1 * rows_per_m
            if sign > 0:
                quad(S(m0), S(m1), 0.0, 0.5, vt(r0), vt(r1))
            else:
                quad(S(m1), S(m0), 0.0, 0.5, vt(r1), vt(r0), flip=False)
        # Numerals cell: rows k*1024 .. (k+1)*1024 map 0..60 m; use 40..60 m -> 58..80 m
        base_row = end * 1024
        r0 = base_row + 1024 - 42 * 1024 / 60.0 - 4 * 1024 / 60.0
        r1 = base_row + 1024 - 42 * 1024 / 60.0 + 22 * 1024 / 60.0
        if sign > 0:
            quad(S(58), S(80), 0.5, 1.0, vt(r1), vt(r0))
        else:
            quad(S(80), S(58), 0.5, 1.0, vt(r0), vt(r1))
    # Middle: repeating 120 m sections.
    s = 360.0
    while s < RWY_LEN - 360.0:
        s1 = min(s + 120.0, RWY_LEN - 360.0)
        frac = (s1 - s) / 120.0
        quad(s, s1, 0.0, 0.5, vt(1536), vt(1536 + 512 * frac))
        s = s1
    parts.append(p)
    write_obj(OUT / "runway.obj", parts, {"TEXTURE": "runway.png"}, header_extra=["GLOBAL_no_shadow"], lod=60000)


def main():
    probe = TileMosaic("elev", 12, LAT0 - 0.08, LAT0 + 0.08, LON0 - 0.15, LON0 + 0.1)
    fit = runway_profile(probe)
    print(f"runway {RWY_LEN:.0f} m, threshold 32 elevation {fit[1]:.1f} m, slope {fit[0] * 1000:.2f} m/km")
    inner = None
    for name, half, step, zi, ze, tex, drop in LEVELS:
        build_level(name, half, step, zi, ze, tex, drop, fit, inner)
        inner = half
    runway_texture()
    runway_obj(fit)
    (OUT / "runway_profile.txt").write_text(f"{fit[0]:.8f} {fit[1]:.3f}\n")
    (OUT / "ATTRIBUTION.txt").write_text(
        "Imagery: Sentinel-2 cloudless - https://s2maps.eu by EOX IT Services GmbH "
        "(Contains modified Copernicus Sentinel data), CC BY-NC-SA 4.0.\n"
        "Elevation: AWS Terrain Tiles (Mapzen/Tilezen; SRTM, USGS 3DEP, GMTED2010, ETOPO1).\n")
    print("scenery written to", OUT)


if __name__ == "__main__":
    main()
