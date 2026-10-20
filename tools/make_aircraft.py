#!/usr/bin/env python3
"""Create the "IIAS SpaceShipTwo" user-aircraft package.

X-Plane always draws a user aircraft (.acf). The mission plugin flies and
draws SpaceShipTwo itself (override_planepath plus an object instance), so
the .acf is only an invisible container that stops a stock aircraft from
appearing. We take the engine-less Laminar ASK 21 glider file, strip all of
its 3D objects and scripts, and rename it.

Usage: make_aircraft.py "<X-Plane 12 folder>"
"""
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
xp = Path(sys.argv[1] if len(sys.argv) > 1 else Path.home() / "X-Plane 12")
src = xp / "Aircraft" / "Laminar Research" / "Schleicher ASK 21"
dst = xp / "Aircraft" / "IIAS Noctilucent" / "SpaceShipTwo"
if dst.exists():
    shutil.rmtree(dst)
(dst / "objects").mkdir(parents=True)
shutil.copytree(src / "Airfoils", dst / "Airfoils")

acf = (src / "ASK21.acf").read_text(errors="ignore").splitlines()
out = []
for line in acf:
    if line.startswith("P _obja/"):
        continue  # the plugin draws SpaceShipTwo itself; the shell has no objects
    if line.startswith("P acf/_name "):
        line = "P acf/_name IIAS SpaceShipTwo (mission plugin)"
    elif line.startswith("P acf/_ICAO "):
        line = "P acf/_ICAO SS2"
    elif line.startswith("P acf/_descrip "):
        line = "P acf/_descrip Plugin-flown SpaceShipTwo-class spaceplane for the IIAS noctilucent mission"
    elif line.startswith("P acf/_author "):
        line = "P acf/_author xplaneiias"
    out.append(line)
(dst / "SpaceShipTwo.acf").write_text("\n".join(out) + "\n")

for icon in ("ASK21_icon11.png", "ASK21_icon11_thumb.png"):
    if (src / icon).exists():
        shutil.copy2(src / icon, dst / icon.replace("ASK21", "SpaceShipTwo"))
print("aircraft written to", dst)
