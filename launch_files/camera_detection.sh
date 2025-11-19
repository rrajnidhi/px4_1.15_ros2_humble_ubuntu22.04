#!/bin/bash

# PX4 + ROS2 Humble + uXRCE-DDS tmux launcher

SESSION_NAME="sitl_px4_ros2_humble_camera"

# Environment variables (use external values if provided, otherwise default)
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-3}
PX4_SYS_AUTOSTART=${PX4_SYS_AUTOSTART:-4002}
PX4_UXRCE_DDS_PORT=${PX4_UXRCE_DDS_PORT:-8888}
PX4_UXRCE_DDS_NS=${PX4_UXRCE_DDS_NS:-cam_drone}
PX4_UAV_MODEL=${PX4_UAV_MODEL:-x500_depth}

# Kill any existing session
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start new tmux session (detached)
tmux new-session -d -s $SESSION_NAME

# Pane 0: PX4 SITL + Gazebo
tmux send-keys -t $SESSION_NAME "cd /app/PX4-Autopilot/; ROS_DOMAIN_ID=$ROS_DOMAIN_ID PX4_UXRCE_DDS_PORT=$PX4_UXRCE_DDS_PORT PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" PX4_GZ_MODEL=$PX4_UAV_MODEL ./build/px4_sitl_default/bin/px4" C-m

# Pane 1: Micro XRCE-DDS Agent with wait-for-SITL to start logic
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.1 "
echo 'Waiting for PX4 SITL process to start...'
while ! ps aux | grep -E '[p]x4_sitl' >/dev/null; do
    sleep 1
done
echo 'PX4 SITL detected. Starting Micro XRCE-DDS Agent...'
MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT" C-m

# Pane 2: Object detector
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run ros_gz_image image_bridge /camera" C-m

# Pane 3: for future use
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.2 "echo 'This pane is reserved for later use'" C-m

# Select first pane
tmux select-pane -t $SESSION_NAME:0.0

# Attach to tmux session
tmux attach-session -t $SESSION_NAME
