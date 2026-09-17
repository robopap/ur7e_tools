#!/usr/bin/env bash
set -eo pipefail

export DISPLAY="${DISPLAY:-:0}"
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

# Build the current Windows checkout.
cd /root/ros2_ws
rm -rf build/ur7e_tools install/ur7e_tools
colcon build --packages-select ur7e_tools --event-handlers console_direct+
source /root/ros2_ws/install/setup.bash

# Patch the installed dual-UR launch so both robots connect back to the
# Windows host IP (UR_REVERSE_IP, normally 10.0.0.8).
DUAL_LAUNCH="/root/ros2_ws/install/ur7e_tools/share/ur7e_tools/launch/dual_ur7e.launch.py"
if [[ -f "$DUAL_LAUNCH" ]]; then
  python3 /opt/ur7e_docker/patch_dual_reverse_ip.py "$DUAL_LAUNCH"
fi

# Verify that the WSLg X server is reachable.
for _ in $(seq 1 50); do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

if ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
  echo "ERROR: WSLg display $DISPLAY is not reachable." >&2
  exit 3
fi

cd /root/ros2_ws/src/ur7e_tools

# Run the PySide UI using the dedicated Python environment.
# RViz launched by the UI inherits the same WSLg display.
exec /root/venvs/ur7e_ui/bin/python \
  /root/ros2_ws/src/ur7e_tools/ur7e_tools/workcell_ui.py