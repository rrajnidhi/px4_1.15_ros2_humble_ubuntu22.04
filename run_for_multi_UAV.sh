#!/bin/bash

# Multi-Drone PX4–ROS2 Launcher

if [ -z "$1" ]; then
    echo "Usage: $0 <drone_id>"
    echo "Example: $0 1"
    exit 1
fi

DRONE_ID=$1

IMAGE_NAME=px4_ros2_humble

# Unique per drone
CONTAINER_NAME="px4_drone_${DRONE_ID}"
ROS_DOMAIN_ID=$((30 + DRONE_ID))       # 31, 32, 33 ...
XRCE_PORT=$((8800 + DRONE_ID))         # 8801, 8802, 8803 ...

# MAVLink identity & port per drone
MAV_SYS_ID=$DRONE_ID                   # 1, 2, 3 ...
GCS_PORT=$((18570 + DRONE_ID))     

BAG_FOLDER="./rosbags/drone_${DRONE_ID}"

echo " Launching Drone $DRONE_ID"
echo " Container:       $CONTAINER_NAME"
echo " ROS_DOMAIN_ID:   $ROS_DOMAIN_ID"
echo " XRCE_DDS_PORT:   $XRCE_PORT"
echo " MAV_SYS_ID:      $MAV_SYS_ID"
echo " MAVLINK_PORT:    $MAVLINK_PORT"
echo " Rosbags Folder:  $BAG_FOLDER"

# Create rosbags directory
mkdir -p "$BAG_FOLDER"
sleep 1

# Allow GUI apps for Gazebo
xhost +local:root > /dev/null

# Start container
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --privileged \
    --device=/dev/kfd \
    --device=/dev/dri \
    --group-add render \
    --group-add video \
    --env DISPLAY=$DISPLAY \
    --env ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    --env PX4_UXRCE_DDS_PORT="$XRCE_PORT" \
    --env PX4_UXRCE_DDS_NS="cam_drone_$MAV_SYS_ID" \
    --env MAV_SYS_ID="$MAV_SYS_ID" \
    --env MAVLINK_PORT="$MAVLINK_PORT" \
    --workdir="/app" \
    --volume="$BAG_FOLDER:/app/rosbags" \
    --volume="/dev:/dev" \
    --network host \
    $IMAGE_NAME

# Revoke GUI permissions
xhost -local:root > /dev/null

