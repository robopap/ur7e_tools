# ur7e_tools on Windows with Docker Desktop

This folder provides a self-contained Windows deployment of the existing Linux/ROS 2 UI.
It does **not** require any previously-created local Docker image.

## Host prerequisites

- Windows 10/11 x64
- Git for Windows
- Docker Desktop using the WSL 2 backend

The Docker image starts from the public official ROS image:

```text
ros:humble-ros-base-jammy
```

## Source checkout

```powershell
mkdir C:\dev -ErrorAction SilentlyContinue
cd C:\dev
git clone -b feature/experiments https://github.com/robopap/ur7e_tools.git
cd C:\dev\ur7e_tools
```

After this Docker folder has been committed to the repository, no separate ZIP/copy step is needed.

## Dependencies built inside Docker

The image installs/clones its own dependencies:

- ROS 2 Humble base
- Universal Robots ROS 2 driver and ROS dependencies
- RViz2 / ros2_control tooling
- OnRobot 2FG7 description package
- UI Python environment from `requirements.txt`
- NANSENSE/FT runtime dependencies declared by `package.xml`

The external Robotiq FT300-S node is part of `ur7e_tools` and uses serial/pyserial; it does not require a separate Robotiq repository.

## Start

Open Docker Desktop and wait until it reports that the engine is running.
Then in PowerShell:

```powershell
cd C:\dev\ur7e_tools\docker_windows
.\start_windows.ps1
```

The script builds the image on first use, detects the Windows `10.0.0.X` Ethernet address when possible, starts the container, and opens:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

## Stop

```powershell
cd C:\dev\ur7e_tools\docker_windows
.\stop_windows.ps1
```

## Notes

- Robot 1 reverse ports: 50001-50004/TCP
- Robot 2 reverse ports: 50011-50014/TCP
- NANSENSE: 33333/UDP
- The Windows checkout is bind-mounted into `/root/ros2_ws/src/ur7e_tools`.
- The container builds the current checkout when it starts.
- The current Windows-specific reverse-IP adaptation patches only the installed launch copy inside the container; it does not modify the Git working tree.
- USB passthrough for the FT300-S / RealSense hardware is a separate Windows-Docker step and must be validated independently.
