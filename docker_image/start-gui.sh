#!/bin/bash

set -e

export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1

Xvfb :1 -screen 0 1600x900x24 &
XVFB_PID=$!

echo "Esperando servidor X..."

until xdpyinfo -display :1 >/dev/null 2>&1; do
    sleep 0.5
done

openbox-session &

xterm \
    -geometry 120x35+20+20 \
    -title "G1 Course Simulator" &

x11vnc \
    -display :1 \
    -forever \
    -shared \
    -nopw \
    -rfbport 5900 &

exec websockify \
    --web=/usr/share/novnc \
    6080 \
    localhost:5900