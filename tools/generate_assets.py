#!/usr/bin/env python3
"""Noctilucent cloud layer built from real NASA AIM/CIPS measurements.

NASA's AIM satellite has photographed polar mesospheric (noctilucent)
clouds from above since 2007. Its CIPS Level 2 product gives, per orbit
strip, the measured cloud albedo and ice water content at ~5 km
resolution (public domain, NASA/LASP/Hampton Univ.):
https://cdaweb.gsfc.nasa.gov/pub/data/aim/cips/data/pmc/level2/cloud_prop/

We pick the most structured fully-observed 350 x 350 km window from the
downloaded orbit strips (real NLC are rare and faint at Eielson's 64.7 N,
so the window comes from further north and is relocated over the mission
area), then build:

  * nlc_macro.obj/png - a 350 km curved sheet whose texture is the real
    albedo map, upsampled, with fine filaments only where cloud was
    measured; soft edge fade.
  * nlc_real.bin - albedo and ice water content grids, plus a 12 km tile
    of fine structure, for the probe's density readings.

Run tools/fetch_aim.py first to download the NASA files.
"""
from __future__ import annotations

import glob
import struct
import sys
from math import pi
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.io import netcdf_file
from scipy.ndimage import zoom

sys.path.insert(0, str(Path(__file__).resolve().parent))
from objlib import Part, write_obj  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets"
AIM = ROOT / "downloads" / "aim"
R_EARTH = 6371000.0 + 82300.0
WIN = 70                  # pixels of real data (5 km each) -> 350 km
MACRO_M = WIN * 5000.0
DETAIL_M = 12000.0
rng = np.random.default_rng(20190704)


def periodic_noise(n, k_lo, k_hi, aniso=1.0, angle=0.0):
    kx = np.fft.fftfreq(n) * n
    KX, KY = np.meshgrid(kx, kx)
    c, s = np.cos(angle), np.sin(angle)
    a = KX * c + KY * s
    b = (-KX * s + KY * c) * aniso
    k = np.hypot(a, b)
    filt = np.exp(-((np.log(np.maximum(k, 1e-6)) - np.log((k_lo * k_hi) ** 0.5)) /
                    (0.5 * np.log(k_hi / k_lo) + 1e-6)) ** 2)
    filt[0, 0] = 0
    f = np.fft.ifft2(np.fft.fft2(rng.normal(size=(n, n))) * filt).real
    return (f - f.mean()) / (f.std() + 1e-9)


def pick_window():
    best = None
    for f in sorted(glob.glob(str(AIM / "aim_cips_pmc_l2_cloud_prop_*_north_*.nc"))):
        d = netcdf_file(f, "r", mmap=False)
        A = d.variables["CLD_ALBEDO"].data.astype(float)
        I = d.variables["ICE_WATER_CONTENT"].data.astype(float)
        H, W = A.shape
        valid = np.isfinite(A)
        for i in range(0, H - WIN, 4):
            for j in range(0, W - WIN, 8):
                if valid[i:i + WIN, j:j + WIN].mean() < 0.995:
                    continue
                w = np.clip(np.nan_to_num(A[i:i + WIN, j:j + WIN]), 0, None)
                cov = (w > 3).mean()
                centre = w[WIN // 2 - 10:WIN // 2 + 10, WIN // 2 - 10:WIN // 2 + 10].mean()
                g = np.abs(np.diff(w, axis=0)).mean() + np.abs(np.diff(w, axis=1)).mean()
                score = g * min(centre, 30.0) * (1.0 if 0.45 < cov < 0.82 else 0.15)
                if best is None or score > best[0]:
                    best = (score, f, i, j, w, np.clip(np.nan_to_num(I[i:i + WIN, j:j + WIN]), 0, None))
    if best is None:
        raise SystemExit("no AIM CIPS level 2 files in downloads/aim; run tools/fetch_aim.py")
    _, f, i, j, w, iwc = best
    print(f"real NLC window: {Path(f).name} rows {i}-{i + WIN} cols {j}-{j + WIN}")
    print(f"  albedo mean {w.mean():.1f}, max {w.max():.1f} (1e-6/sr), cover {(w > 3).mean():.0%}")
    return Path(f).name, i, j, w, iwc


def macro_texture(alb):
    n = 4096
    big = np.clip(zoom(alb, n / alb.shape[0], order=3), 0, None)
    # Structure real NLC show below CIPS resolution: billows (~3-6 km) and
    # filaments, modulating only where cloud was measured.
    bill = periodic_noise(n, 70, 140, aniso=2.5, angle=0.6)
    fil = periodic_noise(n, 200, 520, aniso=5.0, angle=0.6 + pi / 2)
    mid = periodic_noise(n, 18, 45, aniso=1.5, angle=0.2)
    mod = np.clip(0.85 + 0.09 * bill + 0.06 * fil + 0.08 * mid, 0.5, 1.2)
    a = np.clip(big / 45.0, 0, 1.2) * mod
    y, x = np.mgrid[0:n, 0:n] / (n - 1) * 2 - 1
    r = np.maximum(np.abs(x), np.abs(y))
    a *= np.clip((1.0 - r) / 0.22, 0, 1) ** 1.5      # soft edge fade
    alpha = np.clip(a ** 1.15 * 0.42, 0, 0.42)
    t = np.clip(a, 0, 1)
    rgb = np.stack([150 + 95 * t ** 1.4, 196 + 55 * t, np.full_like(t, 255)], 2)
    Image.fromarray(np.dstack([rgb, alpha * 255]).clip(0, 255).astype(np.uint8), "RGBA").save(OUT / "nlc_macro.png")
    lit = (rgb * (alpha[..., None] ** 0.9) * 0.9).clip(0, 255).astype(np.uint8)
    Image.fromarray(lit, "RGB").save(OUT / "nlc_macro_LIT.png")
    (ROOT / "art").mkdir(exist_ok=True)
    Image.fromarray((alpha / 0.42 * 255).astype(np.uint8)).resize((768, 768)).save(ROOT / "art" / "nlc_macro_preview.png")


def detail_texture():
    """One periodic 12 km tile of fine NLC structure (probe density)."""
    n = 1024
    bill = periodic_noise(n, 3, 6, aniso=3.0, angle=0.6)
    bill2 = periodic_noise(n, 6, 12, aniso=2.0, angle=0.9)
    fil = periodic_noise(n, 25, 70, aniso=6.0, angle=0.6 + pi / 2)
    crest = (1 - np.abs(np.sin(pi * (0.5 + 0.5 * np.tanh(bill))))) ** 2
    d = np.clip(0.45 * crest + 0.30 * (0.5 + 0.5 * bill2) + 0.35 * np.clip(fil, 0, None), 0, 1.3)
    d = (d - d.min()) / (d.max() - d.min())
    return (d * 255).astype(np.uint8)


def curved_sheet(name, size, n, dy, uv_rect, anim=None, attrs=None):
    s = np.linspace(-size / 2, size / 2, n + 1)
    X, Z = np.meshgrid(s, s)
    P = np.stack([X, -(X ** 2 + Z ** 2) / (2 * R_EARTH) + dy, Z], 2)
    u0, v0, u1, v1 = uv_rect
    U = u0 + (u1 - u0) * (X + size / 2) / size
    V = v1 - (v1 - v0) * (Z + size / 2) / size   # north (-z) maps to the top of the image
    p = Part(name, anim=anim or [], attrs=attrs or [])
    p.add_grid(P, np.stack([U, V], 2))
    return p


def main():
    src, i, j, alb, iwc = pick_window()
    macro_texture(alb[::-1])          # image rows north->south
    density = detail_texture()
    glow = ["ATTR_no_cull", "ATTR_no_shadow", "ATTR_light_level 0 1 iias/nlc/glow"]
    macro = [curved_sheet(f"macro{k}", MACRO_M, 96, dy, (0, 0, 1, 1),
                          anim=["ANIM_hide -1 0.02 iias/nlc/visible"], attrs=list(glow))
             for k, dy in enumerate((-220.0, 220.0))]
    write_obj(OUT / "nlc_macro.obj", macro, {"TEXTURE": "nlc_macro.png", "TEXTURE_LIT": "nlc_macro_LIT.png"},
              header_extra=["GLOBAL_no_shadow", "GLOBAL_luminance 260"], lod=900000)
    # Probe data: header (n, macro size m, detail tile m, centre offset east,
    # north m); albedo n*n f32 and
    # ice water content n*n f32 (rows south->north, cols west->east); detail
    # density 1024*1024 u8 (rows north->south).
    # Offset that puts the brightest real patch over the mission area.
    from scipy.ndimage import gaussian_filter
    sm = gaussian_filter(alb, 6) * np.clip(1 - np.maximum(*np.abs(np.mgrid[-1:1:WIN * 1j, -1:1:WIN * 1j])) / 0.6, 0, 1)
    pi_, pj = np.unravel_index(np.argmax(sm), sm.shape)
    off_n = -((pi_ + 0.5) / WIN - 0.5) * MACRO_M
    off_e = -((pj + 0.5) / WIN - 0.5) * MACRO_M
    print(f"  sheet offset east {off_e / 1000:.0f} km, north {off_n / 1000:.0f} km")
    with open(OUT / "nlc_real.bin", "wb") as f:
        f.write(struct.pack("<iffff", WIN, MACRO_M, DETAIL_M, off_e, off_n))
        f.write(alb.astype("<f4").tobytes())
        f.write(iwc.astype("<f4").tobytes())
        f.write(density.tobytes())
    (OUT / "nlc_source.txt").write_text(
        f"NASA AIM/CIPS Level 2 cloud properties, {src}, rows {i}-{i + WIN}, cols {j}-{j + WIN}\n"
        "(5 km pixels; window relocated and centred on the mission area). Public domain: NASA/LASP/HU.\n")
    for old in ("nlc_patch.obj", "nlc.png", "nlc_LIT.png", "nlc_density.bin", "nlc_detail.obj",
                "nlc_detail.png", "nlc_detail_LIT.png"):
        (OUT / old).unlink(missing_ok=True)
    print("cloud layer written")


if __name__ == "__main__":
    main()
