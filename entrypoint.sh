#!/bin/bash
set -e

printf "Entering px4_ros2_humble container\n"
sleep 1

# Basic tmux config
if [ ! -f /root/.tmux.conf ]; then
    echo "bind e kill-session" > /root/.tmux.conf
fi

# Export ROS_DOMAIN_ID
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-3}

# Source ROS 2 Humble and PX4 ROS 2 workspace
source /opt/ros/humble/setup.bash
if [ -f /app/ros2_ws/install/setup.bash ]; then
    source /app/ros2_ws/install/setup.bash
fi

# Export Gazebo models path
export GZ_SIM_RESOURCE_PATH=/root/.gz/models

# If no arguments, start interactive shell
if [ "$#" -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi

