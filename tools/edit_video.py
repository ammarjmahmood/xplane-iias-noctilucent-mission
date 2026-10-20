#!/usr/bin/env python3
"""Cut a highlight video from a full raw mission recording.

Mission events and camera switches are written to X-Plane's Log.txt with the
plugin's wall clock ("[wall 123.4]", "camera X at wall 123.4"). Given the
video time at which one known wall time occurs, segments are chosen around
the key events and concatenated (some sped up) with ffmpeg.

Usage: edit_video.py raw.mp4 Log.txt out.mp4 --anchor-wall W --anchor-video V
"""
import argparse
import re
import subprocess
import tempfile
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("raw")
ap.add_argument("log")
ap.add_argument("out")
ap.add_argument("--anchor-wall", type=float, required=True)
ap.add_argument("--anchor-video", type=float, required=True)
args = ap.parse_args()

events = {}
for line in Path(args.log).read_text(errors="ignore").splitlines():
    m = re.match(r"NLC: (.*) \[wall ([0-9.]+)\]$", line)
    if m:
        events.setdefault(m.group(1).split(" --")[0].split(" at ")[0].split(",")[0].strip(), float(m.group(2)))
for k, v in events.items():
    print(f"{v:8.1f}  {k}")


def ev(prefix):
    for k, v in events.items():
        if k.startswith(prefix):
            return v - args.anchor_wall + args.anchor_video
    raise SystemExit(f"event not found: {prefix}")


airborne = ev("WhiteKnightTwo airborne")
orbit_done = ev("Climb orbit complete")
release = ev("RELEASE")
probe = ev("PROBE DEPLOYED")
cloud_up = ev("Crossing noctilucent cloud layer")
apogee = ev("APOGEE")
defeather = ev("DEFEATHER")
hac = ev("On heading-alignment circle")
final = ev("Rolling out on final")
touchdown = ev("TOUCHDOWN")
stop = ev("WHEELS STOP")

# (start, end, speed) in raw-video seconds.
segments = [
    (airborne - 22, airborne + 14, 1.0),          # takeoff roll and lift-off
    (airborne + 14, orbit_done - 2, 6.0),         # compressed climb, cameras cycling
    (release - 16, release + 8, 1.0),             # carrier view, release
    (release + 8, probe + 14, 1.0),               # ignition, full boost, shutdown, probe
    (probe + 14, cloud_up - 8, 2.0),              # coast toward the cloud layer
    (cloud_up - 8, apogee + 22, 1.0),             # NLC crossing and apogee
    (apogee + 22, defeather + 8, 2.5),            # feathered reentry, second crossing
    (defeather + 8, hac + 4, 3.0),                # glider turn back
    (hac + 4, final - 4, 5.0),                    # heading-alignment circle
    (final - 4, touchdown - 28, 2.5),             # final approach
    (touchdown - 28, stop + 6, 1.0),              # flare, touchdown, rollout
]

tmp = Path(tempfile.mkdtemp())
parts = []
for i, (a, b, speed) in enumerate(segments):
    if b <= a:
        continue
    out = tmp / f"seg{i:02d}.mp4"
    vf = f"setpts=PTS/{speed},scale=1920:-2:flags=lanczos" if speed != 1.0 else "scale=1920:-2:flags=lanczos"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{a:.2f}", "-t", f"{b - a:.2f}", "-i", args.raw,
                    "-vf", vf, "-an", "-r", "30", "-c:v", "libx264", "-preset", "slow", "-crf", "24", "-maxrate", "3200k", "-bufsize", "6400k",
                    "-pix_fmt", "yuv420p", str(out)], check=True)
    parts.append(out)
    print(f"segment {i}: {a:7.1f}-{b:7.1f}s at x{speed} -> {(b - a) / speed:5.1f}s")
listing = tmp / "list.txt"
listing.write_text("".join(f"file '{p}'\n" for p in parts))
subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(listing),
                "-c", "copy", "-movflags", "+faststart", args.out], check=True)
print("wrote", args.out)
