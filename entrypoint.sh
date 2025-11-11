#!/bin/bash
set -e

printf "Entering px4_ros2_humble container\n"
sleep 1

# Runtime shell setup
if [ ! -f /root/.tmux.conf ]; then
    echo "bind e kill-session" > /root/.tmux.conf
fi

# Source ROS and workspace environment
source /opt/ros/humble/setup.bash
if [ -f /app/ros2_ws/install/setup.bash ]; then
    source /app/ros2_ws/install/setup.bash
fi

# If no arguments, start interactive shell
if [ "$#" -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi

