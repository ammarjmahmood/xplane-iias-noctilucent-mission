#!/bin/bash
# Cross-compile the Windows plugin (win.xpl) from Linux with MinGW-w64
# (sudo apt install g++-mingw-w64-x86-64-posix). On Windows itself, use the
# CMake / Visual Studio steps in README.md instead.
set -e
cd "$(dirname "$0")/.."
SDK=third_party/sdk/SDK
mkdir -p build/NoctilucentClouds/64
x86_64-w64-mingw32-g++ -std=c++17 -O2 -shared -static -o build/NoctilucentClouds/64/win.xpl \
  src/plugin.cpp src/avionics.cpp src/flight.cpp src/autopilot.cpp -Isrc -I$SDK/CHeaders/XPLM \
  -DIBM=1 -DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM400=1 -DXPLM410=1 \
  $SDK/Libraries/Win/XPLM_64.lib -lopengl32
echo "built build/NoctilucentClouds/64/win.xpl"
