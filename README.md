# Robot Workcell Control — Installation Guide

This project provides a ROS 2 desktop interface for laboratory robot workcells.

Current supported functionality includes:

- **Single UR5 / UR5e**
- **Dual UR7e**
- **Simulation mode** using fake hardware
- **Real Robot(s) mode** using the Universal Robots ROS 2 driver
- Dual-UR7e workcell visualization in RViz
- OnRobot 2FG7 visualization and control
- Independent HOME commands for Robot 1 and Robot 2
- Internal UR force/torque monitoring
- External Robotiq FT300-S monitoring
- Selectable-rate F/T CSV recording
- NANSENSE UDP connection and live 3D skeleton visualization
- NANSENSE views for:
  - Full Body
  - Upper Body
  - Both Arms
  - Left Arm
  - Right Arm

> Assumption: the PC already runs **Ubuntu 22.04** with **ROS 2 Humble** installed.

---

## 1. Install system dependencies

Open a terminal:

```bash
sudo apt update

sudo apt install -y \
    git \
    python3-pip \
    python3-venv \
    python3-rosdep \
    python3-colcon-common-extensions \
    libxcb-cursor0 \
    ros-humble-ur-robot-driver \
    ros-humble-ros2controlcli
```

If `rosdep` has never been initialized on the PC:

```bash
sudo rosdep init
rosdep update
```

If `sudo rosdep init` reports that it has already been initialized, simply continue.

---

## 2. Create a ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

---

## 3. Clone this project

Clone the `ur7e_tools` repository into the workspace:

```bash
cd ~/ros2_ws/src
git clone <UR7E_TOOLS_REPOSITORY_URL> ur7e_tools
```

The resulting path should be:

```text
~/ros2_ws/src/ur7e_tools
```

Replace `<UR7E_TOOLS_REPOSITORY_URL>` with the actual Git repository URL.

---

## 4. Create the UI Python environment

The **Robot Workcell Control** UI uses one dedicated Python virtual environment:

```text
~/venvs/ur7e_ui
```

This keeps the UI-specific Python dependencies isolated from the Ubuntu / ROS 2 system Python installation.

Create and activate it:

```bash
mkdir -p ~/venvs
python3 -m venv ~/venvs/ur7e_ui
source ~/venvs/ur7e_ui/bin/activate
```

Upgrade `pip`:

```bash
python -m pip install --upgrade pip
```

Install the UI dependencies from the repository:

```bash
cd ~/ros2_ws/src/ur7e_tools
python -m pip install -r requirements.txt
```

The current UI dependency versions are:

```text
numpy==1.26.4
matplotlib==3.8.4
PySide6==6.11.2
pyqtgraph==0.13.7
PyOpenGL==3.1.7
```

Check the installation:

```bash
python - << 'PY'
import numpy
import matplotlib
import PySide6
import pyqtgraph
import OpenGL

print('numpy:', numpy.__version__)
print('matplotlib:', matplotlib.__version__)
print('PySide6:', PySide6.__version__)
print('pyqtgraph:', pyqtgraph.__version__)
print('PyOpenGL:', OpenGL.__version__)
print('All UI imports OK')
PY
```

The virtual environment is stored outside the Git repository and must **not** be committed to Git.

---

## 5. Install the OnRobot 2FG7 description package

The Dual UR7e setup uses the `onrobot_2fg7_description` package for RViz visualization.

```bash
cd ~/ros2_ws/src

git clone \
https://github.com/touchlab-avatarx/onrobot_2fg7_description.git
```

The workspace should now contain at least:

```text
~/ros2_ws/src/
├── ur7e_tools/
└── onrobot_2fg7_description/
```

---

## 6. Install missing ROS dependencies

From the workspace root:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y
```

This installs ROS dependencies declared by the packages in the workspace.

For `ur7e_tools`, this includes runtime dependencies such as `coal`, `pinocchio`, `python3-serial`, ROS messages/services and launch dependencies declared in `package.xml`.

---

## 7. Build the workspace

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install
```

Then source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

---

## 8. Automatically source ROS 2 in new terminals

For normal ROS 2 command-line use, the following lines can be added to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Then reload:

```bash
source ~/.bashrc
```

The `ur7e_ui` virtual environment does not need to be automatically activated in `.bashrc`.

The UI launcher activates the correct Python environment automatically.

---

# Running Robot Workcell Control

The recommended way to start the application is:

```bash
cd ~/ros2_ws/src/ur7e_tools

./run_workcell_ui.sh
```

The launcher automatically:

1. activates:

```text
~/venvs/ur7e_ui
```

2. sources ROS 2 Humble:

```text
/opt/ros/humble/setup.bash
```

3. sources the workspace:

```text
~/ros2_ws/install/setup.bash
```

4. starts **Robot Workcell Control** using the Python interpreter from the `ur7e_ui` environment.

The launcher can therefore be used from a normal terminal without manually activating the virtual environment first.

---

## Why the UI uses `run_workcell_ui.sh`

The ROS 2 console executable generated by `colcon` may resolve to the Ubuntu system Python:

```bash
ros2 run ur7e_tools workcell_ui
```

The GUI intentionally uses the dedicated `~/venvs/ur7e_ui` environment, which contains the pinned Qt, Matplotlib and OpenGL visualization dependencies.

For this reason, start **Robot Workcell Control** with:

```bash
cd ~/ros2_ws/src/ur7e_tools
./run_workcell_ui.sh
```

The launcher activates the UI environment, sources ROS 2 Humble and the workspace, and then starts `workcell_ui.py` with the correct Python interpreter.

Normal ROS 2 nodes and launch files can still be started with `ros2 run` and `ros2 launch`.

---

# Desktop launcher

A desktop shortcut can optionally call:

```text
~/ros2_ws/src/ur7e_tools/run_workcell_ui.sh
```

The desktop shortcut itself is machine-specific and does not need to be stored in the repository.

The application name is:

```text
Robot Workcell Control
```

---

# Main UI

The UI allows selection between:

```text
Single UR5
Dual UR7e
```

and:

```text
Simulation
Real Robot(s)
```

For the Dual UR7e real-robot setup, the lower part of the interface contains:

```text
Left side:
    Recording folder
    Robot 1 internal F/T
    Robot 2 internal F/T
    External Robotiq FT300-S
    ROS 2 output

Right side:
    NANSENSE live skeleton
```

---

# NANSENSE

NANSENSE Studio runs on the Windows acquisition PC.

The current integration uses:

```text
NANSENSE sensors
        ↓
NANSENSE Studio / solver
        ↓
UDP
        ↓
Ubuntu Robot Workcell Control
```

Current Ubuntu NANSENSE receiver:

```text
UDP port: 33333
```

The MATLAB-format NANSENSE stream provides:

```text
PX, PY, PZ
    World joint position [cm]

RLX, RLY, RLZ
    Local joint rotation [deg]

RWX, RWY, RWZ
    World joint rotation [deg]
```

The UI currently excludes:

```text
Finger joints
Eye joints
Jaw joints
```

The available visualization modes are:

```text
Full Body
Upper Body
Both Arms
Left Arm
Right Arm
```

The source stream can run at approximately:

```text
240 Hz
```

while the live 3D viewer is intentionally refreshed at a lower rate.

The visualization is centered on the Hips only for display purposes.

The raw NANSENSE positions and rotations remain unchanged.

---

# Simulation test

## Dual UR7e

In the UI select:

```text
Setup: Dual UR7e
Mode:  Simulation
```

Then:

```text
START SYSTEM

Robot 1:
    MOVE TO HOME
    Gripper control

Robot 2:
    MOVE TO HOME
    Gripper control

STOP SYSTEM
```

RViz should display:

- the complete workcell,
- Robot 1,
- Robot 2,
- one OnRobot 2FG7 attached to each UR7e.

The two robots and two simulated grippers can be controlled independently.

---

## Single UR5

Select:

```text
Setup: Single UR5
Mode:  Simulation
```

Choose either:

```text
ur5
ur5e
```

Then:

```text
START SYSTEM
STOP SYSTEM
```

---

# Real UR robots

The UI contains a **Real Robot(s)** mode.

For real robots, the PC additionally requires:

- Ethernet connectivity to the robot(s)
- correct PC network configuration
- correct robot IP addresses
- Universal Robots **External Control**
- the robot reachable from the PC
- the appropriate UR safety / remote-control configuration

For the Dual UR7e setup, the current robot IPs can be edited directly in the UI.

Before starting real control, use the **TEST** button for each robot and verify that it is reachable.

> `STOP SYSTEM` is not an emergency stop.

> During real-robot testing, the operator should remain able to access the teach pendant / emergency stop immediately.

---

# Current Dual UR7e network configuration

The current laboratory workcell uses one Ethernet switch and a common `10.0.0.X` network.

```text
Robot 1:      10.0.0.1
Robot 2:      10.0.0.2
Ubuntu PC 1:  10.0.0.3
Ubuntu PC 2:  10.0.0.4

NANSENSE Windows PC:
    any unused 10.0.0.X address
```

Use subnet mask:

```text
255.255.255.0
```

Each device must use a unique address. The NANSENSE Windows PC does **not** require one specific fixed IP; the Robot Workcell UI accepts NANSENSE sources on the `10.0.0.X` network.

## Configure the NANSENSE Windows PC

The preferred setup uses the normal Windows IPv4 interface:

```text
Control Panel
→ Network and Internet
→ Network and Sharing Center
→ Change adapter settings
→ Ethernet
→ Properties
→ Internet Protocol Version 4 (TCP/IPv4)
→ Properties
```

Select **Use the following IP address** and enter, for example:

```text
IP address:       10.0.0.<FREE_IP>
Subnet mask:      255.255.255.0
Default gateway:  leave blank
Preferred DNS:    leave blank
```

`<FREE_IP>` must be an unused address and must not duplicate either robot or another PC.

After applying the configuration, verify connectivity from Windows Command Prompt or PowerShell, for example:

```powershell
ping 10.0.0.1
ping 10.0.0.2
ping 10.0.0.3
```

If the NANSENSE PC uses Wi-Fi for Internet access, Wi-Fi can remain enabled while the Ethernet adapter is used for the isolated robot network.

---

# HOME positions

HOME positions are stored separately:

```text
ur7e_tools/config/home_ur5.yaml
ur7e_tools/config/home_robot1.yaml
ur7e_tools/config/home_robot2.yaml
```

The Dual UR7e simulation can use the stored HOME positions.

For a new physical installation, verify safe HOME positions with the actual robots before relying on the HOME buttons.

---

# OnRobot 2FG7

For the Dual UR7e setup, each robot has an OnRobot 2FG7.

The UI provides a normalized command:

```text
0.0 = closed
1.0 = open
```

For the real UR7e setup, the current implementation sends the command through the robot analog-output interface.

Current service:

```text
/robotX/io_and_status_controller/set_analog_output
```

The simulation uses the gripper visualizer to reproduce the gripper state in RViz.

---

# Force / Torque monitoring

The UI currently supports three wrench sources:

```text
Robot 1 internal UR F/T
Robot 2 internal UR F/T
External Robotiq FT300-S
```

Internal wrench topics:

```text
/robot1/force_torque_sensor_broadcaster/wrench
/robot2/force_torque_sensor_broadcaster/wrench
```

External sensor topic:

```text
/external_ft
```

All three verified streams operate at approximately:

```text
100 Hz
```

The UI display is intentionally refreshed more slowly than the underlying sensor streams.

---

# F/T recording

Each F/T source can be recorded independently.

Available CSV recording rates:

```text
100 Hz
50 Hz
30 Hz
20 Hz
10 Hz
```

Selecting a lower recording rate changes only the CSV sampling rate.

The underlying ROS / sensor stream remains at its full source rate.

Default recording directory:

```text
~/Robot_Recordings
```

This directory is intentionally outside the Git repository.

The recording directory can also be changed directly from the UI.

---

# External Robotiq FT300-S

The external sensor currently uses its native serial stream.

Current device:

```text
/dev/ttyUSB0
```

The Linux user must have permission to access serial devices. Add the current user to the `dialout` group once:

```bash
sudo usermod -aG dialout "$USER"
```

Then **log out and log back in** so the new group membership takes effect.

Verify with:

```bash
groups
ls -l /dev/ttyUSB0
```

The user should appear in the `dialout` group. A typical device entry is owned by `root:dialout`.

The sensor stream runs at approximately:

```text
100 Hz
```

Published ROS topic:

```text
/external_ft
```

Software zero service:

```text
/external_ft/zero
```

A persistent udev device name may be configured separately if required.

---

# Useful direct ROS launch commands

The UI normally launches these automatically, but they can also be run manually.

## Dual UR7e simulation

```bash
ros2 launch ur7e_tools dual_ur7e.launch.py
```

## Single UR5 simulation

```bash
ros2 launch ur7e_tools single_ur5.launch.py
```

For UR5e:

```bash
ros2 launch ur7e_tools single_ur5.launch.py ur_type:=ur5e
```

---

# Current Dual UR7e workcell mounting orientation

The workcell visualization uses:

```text
Robot 1:
    roll  = +1.5708
    pitch =  0.0
    yaw   = +1.5708

Robot 2:
    roll  = +1.5708
    pitch =  0.0
    yaw   = -1.5708
```

These values are stored as defaults in `dual_ur7e.launch.py`.

---

# Troubleshooting

## `Package 'ur7e_tools' not found`

Source ROS 2 and the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

If necessary:

```bash
cd ~/ros2_ws

colcon build --symlink-install

source install/setup.bash
```

---

## Robot is reachable but controller manager is missing

First verify the robot Ethernet connection.

Example:

```bash
ping 10.0.0.1
ping 10.0.0.2
```

Then check available controller-manager services:

```bash
ros2 service list | grep controller_manager
```

A disconnected Ethernet adapter can make a previously working robot disappear from the ROS 2 control system.

---

## UI Python dependency error

Do not install the UI dependencies into the Ubuntu system Python.

Activate the dedicated environment:

```bash
source ~/venvs/ur7e_ui/bin/activate
```

Check the pinned UI packages:

```bash
python - << 'PY'
import numpy
import matplotlib
import PySide6
import pyqtgraph
import OpenGL

print('numpy:', numpy.__version__)
print('matplotlib:', matplotlib.__version__)
print('PySide6:', PySide6.__version__)
print('pyqtgraph:', pyqtgraph.__version__)
print('PyOpenGL:', OpenGL.__version__)
PY
```

Expected versions:

```text
numpy      1.26.4
matplotlib 3.8.4
PySide6    6.11.2
pyqtgraph  0.13.7
PyOpenGL   3.1.7
```

If packages are missing or versions differ, reinstall from the repository requirements:

```bash
cd ~/ros2_ws/src/ur7e_tools
python -m pip install -r requirements.txt
```

Start the UI with:

```bash
./run_workcell_ui.sh
```

Do not use `ros2 run ur7e_tools workcell_ui` for the GUI when it resolves to the Ubuntu system Python.

---

## PySide6 / Qt `xcb` error

Install:

```bash
sudo apt install -y libxcb-cursor0
```

Then test inside the UI environment:

```bash
source ~/venvs/ur7e_ui/bin/activate

python -c "from PySide6.QtWidgets import QApplication; print('PySide6 OK')"
```

---

## UR driver package not found

Install:

```bash
sudo apt update
sudo apt install -y ros-humble-ur-robot-driver
```

Then:

```bash
source /opt/ros/humble/setup.bash
```

---

## OnRobot meshes are missing in RViz

Check:

```bash
ros2 pkg prefix onrobot_2fg7_description
```

It should return a path inside the workspace install directory, for example:

```text
/home/<USER>/ros2_ws/install/onrobot_2fg7_description
```

If not:

```bash
cd ~/ros2_ws

colcon build --symlink-install

source install/setup.bash
```

---

# Quick installation summary

For a PC that already has Ubuntu 22.04 + ROS 2 Humble:

```bash
sudo apt update
sudo apt install -y \
    git \
    python3-pip \
    python3-venv \
    python3-rosdep \
    python3-colcon-common-extensions \
    libxcb-cursor0 \
    ros-humble-ur-robot-driver \
    ros-humble-ros2controlcli
```

Initialize `rosdep` if required:

```bash
sudo rosdep init
rosdep update
```

Create the workspace and clone the repositories:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone <UR7E_TOOLS_REPOSITORY_URL> ur7e_tools

git clone \
https://github.com/touchlab-avatarx/onrobot_2fg7_description.git
```

Create the single UI environment:

```bash
mkdir -p ~/venvs
python3 -m venv ~/venvs/ur7e_ui
source ~/venvs/ur7e_ui/bin/activate
python -m pip install --upgrade pip

cd ~/ros2_ws/src/ur7e_tools
python -m pip install -r requirements.txt

deactivate
```

Install all ROS/system dependencies declared by the workspace packages and build:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

For the external FT300-S serial sensor, add the user to `dialout` once:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership.

Optionally add the ROS environment to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Run the UI:

```bash
cd ~/ros2_ws/src/ur7e_tools
./run_workcell_ui.sh
```

---

# Verified workflow

Current verified functionality includes:

```text
Dual UR7e
→ Simulation / Real Robot(s)
→ START SYSTEM
→ Robot connection
→ Robot 1 / Robot 2 HOME
→ Robot 1 / Robot 2 2FG7 control
→ Internal F/T monitoring
→ External FT300-S monitoring
→ Independent CSV recording
→ STOP SYSTEM

NANSENSE
→ Windows NANSENSE Studio
→ UDP 33333
→ Ubuntu Robot Workcell Control
→ CONNECT NANSENSE
→ Live skeleton
→ Full Body / Upper Body / Arms views
```