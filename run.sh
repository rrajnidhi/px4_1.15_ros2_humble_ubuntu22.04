#!/bin/bash

IMAGE_NAME=px4_ros2_humble
CURRENT_USER=$(logname)

# Allow GUI apps (Gazebo)
xhost +local:root > /dev/null

# Create rosbags folder
mkdir -p ./px4_ros2_humble_rosbags
sleep 2

# Run container
docker run -it --rm \
    --device=/dev/kfd \
    --device=/dev/dri \
    --group-add render \
    --group-add video \
    --privileged \
    --env="DISPLAY" \
    --workdir="/app" \
    --volume="$(pwd)/px4_ros2_humble_rosbags:/app/rosbags" \
    --volume="/dev:/dev" \
    --network host \
    $IMAGE_NAME

# Revoke GUI permissions
xhost -local:root > /dev/null

