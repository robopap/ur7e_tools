# Robot Control UI — Installation Guide

This project provides a ROS 2 desktop UI for:

- **Single UR5 / UR5e**
- **Dual UR7e**
- **Simulation mode** using fake hardware
- **Real Robot mode** using the Universal Robots ROS 2 driver
- Dual-UR7e workcell visualization in RViz
- OnRobot 2FG7 visualization and OPEN/CLOSE control in **simulation**
- Independent HOME commands for Robot 1 and Robot 2

> Assumption: the PC already runs **Ubuntu 22.04** with **ROS 2 Humble** installed.

---

## 1. Install system dependencies

Open a terminal:

```bash
sudo apt update

sudo apt install -y \
    git \
    python3-pip \
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

## 2. Install PySide6

The desktop UI is written with Qt/PySide6:

```bash
python3 -m pip install --user PySide6
```

Check that it is available:

```bash
python3 -c "import PySide6; print(PySide6.__version__)"
```

---

## 3. Create a ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

---

## 4. Clone this project

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

## 5. Install the OnRobot 2FG7 description package

The Dual UR7e simulation uses the `onrobot_2fg7_description` package for RViz visualization.

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

rosdep install \
    --from-paths src \
    --ignore-src \
    -r \
    -y
```

This installs ROS dependencies declared by the packages in the workspace.

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

Add the following lines to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Then reload the terminal configuration:

```bash
source ~/.bashrc
```

---

# Running the UI

Start the application with:

```bash
ros2 run ur7e_tools workcell_ui
```

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

---

# Simulation test

## Dual UR7e

In the UI select:

```text
Setup: Dual UR7e
Mode:  Simulation
```

Then run:

```text
START SYSTEM

Robot 1:
    HOME
    OPEN GRIPPER
    CLOSE GRIPPER

Robot 2:
    HOME
    OPEN GRIPPER
    CLOSE GRIPPER

STOP SYSTEM
```

RViz should display:

- the complete workcell,
- Robot 1,
- Robot 2,
- one OnRobot 2FG7 attached to each UR7e.

The two robots and the two simulated grippers can be controlled independently.

## Single UR5

Select:

```text
Setup: Single UR5
Mode:  Simulation
```

Choose either:

```text
UR5
UR5e
```

Then:

```text
START SYSTEM
STOP SYSTEM
```

---

# Real UR robots

The UI also contains a **Real Robot(s)** mode.

For real robots, the PC must additionally have:

- Ethernet connectivity to the robot(s)
- correct PC network configuration
- correct robot IP addresses entered in the UI
- Universal Robots **External Control** configured on the robot controller
- the robot reachable from the PC
- the appropriate UR safety/remote-control configuration

For the Dual UR7e setup, the current laboratory robot IPs are entered directly in the UI and can be edited there.

Before starting real control, use the **TEST** button for each robot and verify that it is reported as reachable.

> Do not treat the UI's `STOP SYSTEM` button as an emergency stop.  
> During initial real-robot tests, the operator should remain next to the teach pendant / emergency stop.

---

# HOME positions

HOME positions are stored separately for each robot:

```text
ur7e_tools/config/home_ur5.yaml
ur7e_tools/config/home_robot1.yaml
ur7e_tools/config/home_robot2.yaml
```

The Dual UR7e simulation can use the current stored HOME positions.

For a new physical installation, verify or save safe HOME positions using the actual robots before relying on the HOME buttons.

---

# OnRobot 2FG7

The current implementation provides:

```text
Simulation:
    Robot 1 OPEN/CLOSE → RViz simulated 2FG7
    Robot 2 OPEN/CLOSE → RViz simulated 2FG7

Real Robot(s):
    real 2FG7 control is handled separately
```

The simulation gripper visualizer is isolated from the real-robot joint-state path and is only launched with fake hardware.

---

# Useful direct launch commands

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

The workcell visualization uses the following final robot mounting orientation:

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

Source the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

If necessary, rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## PySide6 / Qt `xcb` error

Install:

```bash
sudo apt install -y libxcb-cursor0
```

Then test:

```bash
python3 -c "from PySide6.QtWidgets import QApplication; print('PySide6 OK')"
```

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
    python3-rosdep \
    python3-colcon-common-extensions \
    libxcb-cursor0 \
    ros-humble-ur-robot-driver \
    ros-humble-ros2controlcli

python3 -m pip install --user PySide6

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone <UR7E_TOOLS_REPOSITORY_URL> ur7e_tools
git clone https://github.com/touchlab-avatarx/onrobot_2fg7_description.git

cd ~/ros2_ws

source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash

ros2 run ur7e_tools workcell_ui
```

---

## Verified simulation workflow

The following workflow has been tested successfully:

```text
Dual UR7e
→ Simulation
→ START SYSTEM
→ Robot 1 HOME
→ Robot 1 OPEN/CLOSE
→ Robot 2 HOME
→ Robot 2 OPEN/CLOSE
→ STOP SYSTEM

Single UR5
→ Simulation
→ START SYSTEM
→ STOP SYSTEM
```
