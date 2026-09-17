# ur7e_tools on Windows with Docker Desktop

This folder provides the Windows deployment of `ur7e_tools`.

The recommended Windows mode runs ROS 2 Humble inside Docker Desktop while the
PySide6 Workcell UI and RViz are displayed as normal Windows desktop windows
through WSLg. A browser/noVNC mode is also kept as a fallback.

## What is included

The Docker image is built from the public ROS 2 image:

```text
ros:humble-ros-base-jammy
```

The image installs or builds the dependencies required by the project,
including:

- ROS 2 Humble
- Universal Robots ROS 2 driver
- RViz2 and ros2_control tooling
- OnRobot 2FG7 description
- Robotiq ROS description packages
- the Python UI environment from `requirements.txt`
- the current `ur7e_tools` checkout mounted from Windows

The repository remains on Windows. Docker mounts it into:

```text
/root/ros2_ws/src/ur7e_tools
```

and builds the current checkout when the container starts.

---

## Tested configuration

The setup has been validated with:

- Windows 11
- WSL 2 / WSLg
- Docker Desktop with the WSL 2 backend
- Docker Desktop 4.91.0
- Docker Engine 29.8.0
- ROS 2 Humble in Ubuntu 22.04 inside Docker
- two real UR7e robots
- NANSENSE Studio broadcasting from the same Windows PC
- PySide6 Workcell UI and RViz as independent Windows desktop windows

A separate Ubuntu WSL distribution is **not required** for this workflow, and
Docker Desktop WSL integration with a user Ubuntu distribution is not required.

---

# 1. Host prerequisites

Install:

1. Git for Windows
2. WSL 2 with WSLg
3. Docker Desktop

Docker Desktop must use the WSL 2 backend.

It is recommended to update WSL before the first setup:

```powershell
wsl --update
```

Check it with:

```powershell
wsl --version
```

The output should include a WSLg version.

Check Docker Desktop from PowerShell:

```powershell
docker version
```

Both a `Client` and a `Server: Docker Desktop` section should be shown.

---

# 2. Clone the repository

Until this Windows branch is merged into the main development branch:

```powershell
mkdir C:\dev -ErrorAction SilentlyContinue
cd C:\dev
git clone -b feature/windows-docker https://github.com/robopap/ur7e_tools.git
cd C:\dev\ur7e_tools
```

If the Windows changes are later merged, clone the corresponding normal project
branch instead.

---

# 3. Robot Ethernet configuration

The validated workcell network is:

| Device | IPv4 address |
|---|---|
| Robot 1 | `10.0.0.1` |
| Robot 2 | `10.0.0.2` |
| Windows PC | `10.0.0.8` |

Configure the Windows Ethernet adapter with:

```text
IP address: 10.0.0.8
Subnet mask: 255.255.255.0
Gateway: leave empty
DNS: leave empty
```

Wi-Fi can remain connected and provide Internet access.

Verify connectivity:

```powershell
ping 10.0.0.1
ping 10.0.0.2
```

The WSLg launcher searches the Windows host for an IPv4 address on
`10.0.0.x` and exports it as `UR_REVERSE_IP`. With the validated setup it
should print:

```text
UR reverse IP: 10.0.0.8 [Ethernet]
```

Do not use a robot IP as `UR_REVERSE_IP`.

---

# 4. Docker ports

The following host ports are published:

## Robot 1

```text
50001/TCP  reverse interface
50002/TCP  script sender
50003/TCP  trajectory interface
50004/TCP  script command interface
```

## Robot 2

```text
50011/TCP  reverse interface
50012/TCP  script sender
50013/TCP  trajectory interface
50014/TCP  script command interface
```

## NANSENSE

```text
33333/UDP
```

If Windows Firewall asks for permission, allow Docker Desktop on the network
used by the robots. If the robots are reachable from Windows but cannot create
their reverse connection, also check that the TCP ports above are not being
blocked.

---

# 5. Recommended start: WSLg desktop mode

Start Docker Desktop first and wait until its engine is running.

From the repository root:

```powershell
.\docker_windows\start_windows_wslg.ps1
```

On the first run Docker builds the image. This can take several minutes.

After startup:

- the Robot Workcell UI appears as a normal Windows window;
- RViz opens as its own Windows window when launched by the UI;
- both windows can be resized, maximized, minimized, moved between monitors and
  used with Alt-Tab;
- no browser or noVNC desktop is used.

Internally this mode uses the Docker Desktop WSLg X11 socket:

```text
/run/desktop/mnt/host/wslg/.X11-unix
```

The current WSLg compose file binds that socket into the container and uses:

```text
DISPLAY=:0
QT_QPA_PLATFORM=xcb
```

---

# 6. Normal shutdown

The normal way to stop the workcell is simply to close the main Workcell UI
with the window `X`.

The application's `closeEvent()` performs the application cleanup, including
the ROS processes, experiment/analysis processes, FT processes, NANSENSE
shutdown and ROS context shutdown. When the main Python process exits, the
Docker container exits cleanly as well.

A clean shutdown normally appears as:

```text
Exited (0)
```

in:

```powershell
docker ps -a --filter "name=ur7e_tools_windows_wslg"
```

It is normal for an exited container record to remain visible. The next WSLg
start removes any previous container with the same name before creating the new
instance.

For a manual/emergency cleanup:

```powershell
.\docker_windows\stop_windows_wslg.ps1
```

---

# 7. Desktop shortcut

The repository includes:

```text
docker_windows\robot_workcell.ico
docker_windows\create_desktop_shortcut.ps1
```

Run once:

```powershell
.\docker_windows\create_desktop_shortcut.ps1
```

This creates a desktop shortcut named:

```text
Robot Workcell
```

After that the normal workflow is:

```text
double-click Robot Workcell
        |
        v
Workcell UI + ROS + RViz
        |
        v
close the Workcell UI with X
        |
        v
clean container exit
```

No VS Code terminal is required for normal operation.

If the desktop shortcut does not open the UI, run the launcher manually from
PowerShell so that any error remains visible:

```powershell
.\docker_windows\start_windows_wslg.ps1
```

---

# 8. NANSENSE on the same Windows PC

For NANSENSE Studio running on the same Windows machine as Docker, the validated
broadcast destination is:

```text
127.0.0.1
UDP port 33333
```

Docker publishes host UDP port `33333` to the container, so the NANSENSE stream
is received by the ROS/UI environment.

---

# 9. Browser/noVNC fallback

The original browser mode is intentionally retained as a fallback.

Start it with:

```powershell
.\docker_windows\start_windows.ps1
```

It starts an Xvfb/Fluxbox desktop inside the container and exposes it through
x11vnc/noVNC. The launcher opens:

```text
http://localhost:6080/vnc.html?autoconnect=true&resize=scale
```

Stop the browser mode with:

```powershell
.\docker_windows\stop_windows.ps1
```

The browser and WSLg modes publish the same UR and NANSENSE ports and therefore
should not be run at the same time.

The WSLg launcher shuts down the browser-mode compose stack before starting the
WSLg stack.

---

# 10. Important implementation details

## Reverse IP patch

The Windows launcher detects the Windows robot-network address and sets:

```text
UR_REVERSE_IP
```

At container startup, `patch_dual_reverse_ip.py` patches only the installed copy
of the dual-UR launch file inside the Docker workspace.

The Windows Git checkout is not modified by this runtime patch.

## Python environment

The image creates:

```text
/root/venvs/ur7e_ui
```

with `--system-site-packages`.

This lets the dedicated UI environment use PySide6/pyqtgraph while still seeing
ROS 2 Python packages such as `rclpy`.

The WSLg container starts the UI explicitly with that Python interpreter.

## Graphics

For reliable Docker/WSLg rendering the current configuration uses software
OpenGL:

```text
LIBGL_ALWAYS_SOFTWARE=1
```

This avoids depending on a particular host GPU configuration.

---

# 11. Useful diagnostics

Check Docker:

```powershell
docker version
```

Check all workcell containers:

```powershell
docker ps -a --filter "name=ur7e_tools_windows"
```

Check the WSLg container:

```powershell
docker ps -a --filter "name=ur7e_tools_windows_wslg"
```

Follow WSLg logs:

```powershell
docker compose `
  --env-file .\docker_windows\.env `
  -f .\docker_windows\docker-compose.wslg.yml `
  logs -f
```

Validate the WSLg compose file without starting it:

```powershell
docker compose `
  --env-file .\docker_windows\.env `
  -f .\docker_windows\docker-compose.wslg.yml `
  config
```

Verify the robot network:

```powershell
Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.IPAddress -like "10.0.0.*" }
```

---

# 12. Files in `docker_windows`

```text
Dockerfile
docker-compose.yml
docker-compose.wslg.yml
.env.example
README_WINDOWS.md
patch_dual_reverse_ip.py
start_container.sh
start_container_wslg.sh
start_windows.ps1
start_windows_wslg.ps1
stop_windows.ps1
stop_windows_wslg.ps1
create_desktop_shortcut.ps1
robot_workcell.ico
```

The `.env` file is local configuration and must remain ignored by Git.

---

# 13. Hardware not covered automatically

USB passthrough is a separate concern from the Docker/WSLg desktop setup.

Hardware that depends on direct USB access, such as an external serial FT sensor
or USB cameras, may require additional Windows/Docker/USB passthrough
configuration. Do not assume that host USB devices are automatically visible
inside the container.

The dual-UR Ethernet communication and NANSENSE UDP workflow described above
have been validated independently of USB passthrough.
