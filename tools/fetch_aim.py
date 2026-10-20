#!/usr/bin/env python3
"""Download the NASA AIM/CIPS noctilucent-cloud data used for the cloud layer.

The mission's cloud layer is built from one AIM/CIPS Level 2 orbit strip
(4 July 2019 season, northern hemisphere). Pass --more to download a week of
orbits so tools/generate_assets.py can choose among them.

Data: NASA AIM mission (CIPS instrument; LASP / Hampton University), public
domain, served by NASA CDAWeb.
"""
import gzip
import re
import shutil
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "downloads" / "aim"
BASE = "https://cdaweb.gsfc.nasa.gov/pub/data/aim/cips/data/pmc/level2/cloud_prop/2019/"
DEFAULT = ["aim_cips_pmc_l2_cloud_prop_2019-181_orbit-66732_north_v05.20_r08.nc.gz"]


def get(name: str) -> None:
    target = OUT / name[:-3]
    if target.exists():
        return
    print("downloading", name)
    tmp = OUT / name
    with urllib.request.urlopen(BASE + name, timeout=120) as r, open(tmp, "wb") as f:
        shutil.copyfileobj(r, f)
    with gzip.open(tmp, "rb") as src, open(target, "wb") as dst:
        shutil.copyfileobj(src, dst)
    tmp.unlink()


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    names = list(DEFAULT)
    if "--more" in sys.argv:
        listing = urllib.request.urlopen(BASE, timeout=60).read().decode()
        names = sorted(set(re.findall(r"aim_cips_pmc_l2_cloud_prop_2019-18[0-6]_orbit-\d+_north_v05\.20_r08\.nc\.gz",
                                      listing)))[::3]
    for n in names:
        get(n)
    print("AIM data in", OUT)


if __name__ == "__main__":
    main()
