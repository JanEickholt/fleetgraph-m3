# Base: official course image (project.nes.uni-due.de/gitea/janzon/bachelor_project_docker,
# commit b56c4dc, 2026-10-20), unchanged up to the user switch. FleetGraph M3
# additions follow in their own layer so the course base stays recognizable.
# ROS 2 Jazzy is only released for Ubuntu 24.04 (Noble)
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=jazzy

# Base tools + locale
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales software-properties-common curl gnupg2 lsb-release ca-certificates \
        sudo git nano vim wget build-essential cmake python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 apt repository
RUN add-apt-repository -y universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list

# ROS 2 Jazzy desktop (RViz etc.), Gazebo Harmonic bridge, dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-desktop \
        ros-jazzy-ros-gz \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-rosdep \
        # GUI / OpenGL support for X11 forwarding
        mesa-utils libgl1 libglx-mesa0 libgl1-mesa-dri x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# Non-root user (Ubuntu 24.04 ships "ubuntu" with UID/GID 1000)
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ubuntu/.bashrc \
    && echo "[ -f /home/ubuntu/ws/install/setup.bash ] && source /home/ubuntu/ws/install/setup.bash" >> /home/ubuntu/.bashrc

# Create the workspace owned by ubuntu so a fresh named volume mounted here
# inherits that ownership.
RUN mkdir -p /home/ubuntu/ws && chown ubuntu:ubuntu /home/ubuntu/ws

# FleetGraph M3 additions: Nav2 (navigation stack + bringup launch/params)
# and the test harness dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        python3-pytest \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Build artefacts live in named volumes mounted below ws/; create them owned
# by ubuntu so fresh volumes inherit that ownership.
RUN mkdir -p /home/ubuntu/ws/src /home/ubuntu/ws/build /home/ubuntu/ws/install /home/ubuntu/ws/log \
    && chown -R ubuntu:ubuntu /home/ubuntu/ws

USER ubuntu
RUN rosdep update
WORKDIR /home/ubuntu/ws

CMD ["sleep", "infinity"]
