#!/usr/bin/env bash

source "$HOME/venvs/ur7e_ui/bin/activate"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

exec "$HOME/venvs/ur7e_ui/bin/python" \
    "$HOME/ros2_ws/src/ur7e_tools/ur7e_tools/workcell_ui.py"
