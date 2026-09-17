FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Everything required by the project is installed from public sources here.
# No pre-existing local Docker image is required.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    ca-certificates \
    curl \
    iputils-ping \
    iproute2 \
    procps \
    usbutils \
    python3-pip \
    python3-venv \
    python3-rosdep \
    python3-colcon-common-extensions \
    xvfb \
    x11vnc \
    fluxbox \
    novnc \
    websockify \
    dbus-x11 \
    xauth \
    x11-utils \
    libxcb-cursor0 \
    libxkbcommon-x11-0 \
    libx11-xcb1 \
    libxcb-xinerama0 \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libegl1 \
    mesa-utils \
    ros-humble-rviz2 \
    ros-humble-ros2controlcli \
    ros-humble-ur-robot-driver \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ros2_ws/src /root/venvs /opt/ur7e_docker

# External ROS description package used by the current Dual UR7e RViz model.
RUN git clone --depth 1 \
      https://github.com/touchlab-avatarx/onrobot_2fg7_description.git \
      /root/ros2_ws/src/onrobot_2fg7_description

RUN git clone --recurse-submodules https://github.com/robotiq/ros.git \
    /root/ros2_ws/src/robotiq_ros

# Read dependency declarations from the checked-out ur7e_tools repository.
COPY package.xml /tmp/ur7e_pkg/package.xml
COPY requirements.txt /tmp/ur7e_pkg/requirements.txt

# The official ROS image already initializes rosdep, but keep this robust if
# that changes in a future base image.
RUN apt-get update && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install \
        --from-paths /tmp/ur7e_pkg /root/ros2_ws/src/robotiq_ros \
        --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Keep the same dedicated Python environment used by the native Ubuntu setup.
# --system-site-packages exposes rclpy and ROS message packages to the UI venv.
RUN python3 -m venv --system-site-packages /root/venvs/ur7e_ui && \
    source /root/venvs/ur7e_ui/bin/activate && \
    python -m pip install --upgrade pip && \
    python -m pip install -r /tmp/ur7e_pkg/requirements.txt

# Build the external description package once into the image.
RUN source /opt/ros/humble/setup.bash && \
    cd /root/ros2_ws && \
    colcon build \
        --packages-up-to \
        onrobot_2fg7_description \
        robotiq_description

COPY docker_windows/start_container.sh /opt/ur7e_docker/start_container.sh
COPY docker_windows/patch_dual_reverse_ip.py /opt/ur7e_docker/patch_dual_reverse_ip.py
RUN chmod +x /opt/ur7e_docker/start_container.sh

ENV DISPLAY=:1 \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root

EXPOSE 6080/tcp \
       50001/tcp 50002/tcp 50003/tcp 50004/tcp \
       50011/tcp 50012/tcp 50013/tcp 50014/tcp \
       33333/udp

ENTRYPOINT ["/opt/ur7e_docker/start_container.sh"]
