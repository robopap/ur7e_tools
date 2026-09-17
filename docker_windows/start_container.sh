#!/usr/bin/env bash
set -eo pipefail

export DISPLAY="${DISPLAY:-:1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"

mkdir -p "$XDG_RUNTIME_DIR" /root/ros2_ws/src
chmod 700 "$XDG_RUNTIME_DIR"

source /opt/ros/humble/setup.bash

if [[ ! -f /root/ros2_ws/src/ur7e_tools/package.xml ]]; then
  echo "ERROR: /root/ros2_ws/src/ur7e_tools is not mounted correctly." >&2
  exit 2
fi

# Build the current Windows checkout each time the container starts.  We use a
# normal install (not symlink-install), because Windows bind-mount symlink
# semantics are less predictable and because we intentionally patch only the
# installed launch copy below.
cd /root/ros2_ws
rm -rf build/ur7e_tools install/ur7e_tools
colcon build --packages-select ur7e_tools --event-handlers console_direct+
source /root/ros2_ws/install/setup.bash

DUAL_LAUNCH="/root/ros2_ws/install/ur7e_tools/share/ur7e_tools/launch/dual_ur7e.launch.py"
if [[ -f "$DUAL_LAUNCH" ]]; then
  python3 /opt/ur7e_docker/patch_dual_reverse_ip.py "$DUAL_LAUNCH"
fi

# A real Linux X desktop is created inside the container.  The exact PySide6
# application and RViz render there; noVNC merely displays that desktop in the
# Windows browser.
rm -f /tmp/.X1-lock
mkdir -p /tmp/.X11-unix

Xvfb "$DISPLAY" -screen 0 1920x1200x24 -ac +extension GLX +render -noreset \
  >/tmp/xvfb.log 2>&1 &
XVFB_PID=$!

cleanup() {
  kill "$NOVNC_PID" "$VNC_PID" "$FLUXBOX_PID" "$XVFB_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 50); do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

fluxbox >/tmp/fluxbox.log 2>&1 &
FLUXBOX_PID=$!

x11vnc -display "$DISPLAY" -rfbport 5900 -forever -shared -nopw -localhost \
  >/tmp/x11vnc.log 2>&1 &
VNC_PID=$!

websockify --web=/usr/share/novnc 6080 localhost:5900 \
  >/tmp/novnc.log 2>&1 &
NOVNC_PID=$!

source /root/venvs/ur7e_ui/bin/activate
cd /root/ros2_ws/src/ur7e_tools

# This is equivalent to the repository's run_workcell_ui.sh logic: ROS Humble
# and the workspace are sourced, while the GUI runs under the dedicated venv.
exec python /root/ros2_ws/src/ur7e_tools/ur7e_tools/workcell_ui.py
