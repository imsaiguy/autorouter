#!/bin/sh
# Rebuild the compiled search core.  Linux x86-64 and (cross-compiled) Windows x64:
#   sudo apt install gcc gcc-mingw-w64-x86-64
# macOS (run on a Mac):  clang -O2 -shared -o libastar_macos.dylib astar.c
set -e
cd "$(dirname "$0")"
gcc -O2 -shared -fPIC -o libastar_linux_x86_64.so astar.c
x86_64-w64-mingw32-gcc -O2 -shared -static-libgcc -o astar_win64.dll astar.c
ls -la
