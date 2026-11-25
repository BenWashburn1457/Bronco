# syntax=docker/dockerfile:1-labs
# ^ Enables newer Dockerfile features (we use `COPY --parents` below).

# =============================================================================
# Bronco ROS 2 image
#
# One Dockerfile, two "stages" (targets):
#   base -> what the ROBOT needs: ROS 2, build tools, our package dependencies.
#           Runs on the Raspberry Pi (arm64).
#   dev  -> base + GUI tools (RViz, rqt, debugger). Runs on your laptop (amd64).
#
# The official `ros` images are multi-architecture, so this same file builds
# natively on both the Pi and your PC. Docker picks the right CPU version.
# =============================================================================

ARG ROS_DISTRO=jazzy

# -----------------------------------------------------------------------------
# Stage 1: base (robot image)
# -----------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base

# ARGs declared before FROM are forgotten after it, so re-declare what we need.
ARG USERNAME=ben
ARG USER_UID=1000
ARG USER_GID=1000

# Stop apt from asking interactive questions (e.g. timezone) during the build.
ENV DEBIAN_FRONTEND=noninteractive

# General tools: compilers, git, serial/USB helpers for talking to the ESP32.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        sudo \
        nano \
        less \
        usbutils \
        iputils-ping \
        python3-pip \
        python3-serial \
    && rm -rf /var/lib/apt/lists/*

# Create a normal (non-root) user whose UID matches yours on the host.
# Why: files the container creates in the mounted ros2_ws folder will be owned
# by YOU, not root, so you can edit/delete them normally outside Docker.
# Ubuntu 24.04 images already ship a user "ubuntu" with UID 1000, so remove it.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    # dialout = serial ports (ESP32, LiDAR), video = cameras
    && usermod -aG dialout,video ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Install our packages' dependencies with rosdep.
# We copy ONLY the package.xml files (not the code). Docker caches each step,
# so editing code won't re-run this slow apt step; only changing dependencies
# in a package.xml will. `--parents` keeps the folder structure.
COPY --parents ros2_ws/src/*/package.xml /tmp/deps/
# bronco_gazebo is simulation-only (it pulls in Gazebo, ~1 GB). The robot never
# runs the simulator, so we skip its dependencies here and install them only in
# the dev stage below.
RUN rm -rf /tmp/deps/ros2_ws/src/bronco_gazebo \
    && apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/deps/ros2_ws/src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/* /tmp/deps

# Entrypoint script: sources ROS before running whatever command you give.
# --chmod=755 = owner can read/write/execute, everyone else can read/execute.
# (Just `chmod +x` isn't enough: if the file was 600 on the host, the container
# user can execute it but not READ it, and bash needs to read a script to run it.)
COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh

# Make interactive shells (docker exec ... bash) source ROS automatically too.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash 2>/dev/null || true" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# -----------------------------------------------------------------------------
# Stage 2: dev (laptop image) = base + visualization and debugging tools
# -----------------------------------------------------------------------------
FROM base AS dev

ARG USERNAME=ben

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        gdb \
        clang-format \
    && rm -rf /var/lib/apt/lists/*

# Simulation dependencies (Gazebo Harmonic + the ROS <-> Gazebo bridge), read
# from bronco_gazebo's package.xml, same trick as in the base stage.
COPY ros2_ws/src/bronco_gazebo/package.xml /tmp/deps/bronco_gazebo/package.xml
RUN apt-get update \
    && rosdep install --from-paths /tmp/deps --ignore-src -y \
        --skip-keys "bronco_description" \
    && rm -rf /var/lib/apt/lists/* /tmp/deps
USER ${USERNAME}
