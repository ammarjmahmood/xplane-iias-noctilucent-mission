#!/bin/bash
# Reinstall the plugin and restart the X-Plane 12 demo on the NVIDIA GPU,
# clicking through the demo dialogs and resuming the last flight.
XP="$HOME/X-Plane 12"
export DISPLAY=:0
pkill -f '^./X-Plane-x86_64'; sleep 5; pkill -9 -f '^./X-Plane-x86_64'; sleep 1
rm -rf "$XP/Resources/plugins/NoctilucentClouds"
cp -a "$(dirname "$0")/../build/NoctilucentClouds" "$XP/Resources/plugins/"
cd "$XP" || exit 1
setsid env VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json ./X-Plane-x86_64 --no_sound \
  --ignore_webkit_error --window=1920x1080 > /dev/null 2>&1 < /dev/null &
for i in $(seq 1 90); do grep -q "NLC: assets loaded" Log.txt 2>/dev/null && break; sleep 1; done
sleep 15
xdotool mousemove 782 692 click 1; sleep 3
xdotool mousemove 1200 692 click 1; sleep 3
xdotool mousemove 410 860 click 1
for i in $(seq 1 150); do grep -q "Loaded: .*Aircraft/" Log.txt 2>/dev/null && break; sleep 1; done
sleep 25
xdotool mousemove 1919 1079
grep "NLC:" Log.txt | tail -3
