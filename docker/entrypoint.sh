#!/bin/bash
# Runs every time a container starts, before your command.
set -e

# Load ROS 2 (puts `ros2`, `colcon`, etc. on your PATH and sets env vars).
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# If the workspace has been built, load our own packages too.
if [ -f "${HOME}/ros2_ws/install/setup.bash" ]; then
    source "${HOME}/ros2_ws/install/setup.bash"
fi

# Hand off to the actual command (default: bash).
exec "$@"
