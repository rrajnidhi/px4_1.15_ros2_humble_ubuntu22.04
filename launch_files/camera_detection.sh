#!/bin/bash

# PX4 + ROS2 Humble + uXRCE-DDS tmux launcher

SESSION_NAME="sitl_px4_ros2_humble_camera"

# Environment variables (use external values if provided, otherwise default)
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-3}
PX4_UXRCE_DDS_PORT=${PX4_UXRCE_DDS_PORT:-8888}
PX4_UXRCE_DDS_NS=${PX4_UXRCE_DDS_NS:-cam_drone}
PX4_UAV_MODEL=${PX4_UAV_MODEL:-gz_x500_depth}
PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7"

# Kill any existing session
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start new tmux session (detached)
tmux new-session -d -s $SESSION_NAME

# Pane 0: PX4 SITL + Gazebo headless
tmux send-keys -t $SESSION_NAME "cd /app/PX4-Autopilot/; ROS_DOMAIN_ID=$ROS_DOMAIN_ID PX4_UXRCE_DDS_PORT=$PX4_UXRCE_DDS_PORT PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS PX4_GZ_MODEL_POSE=\"$PX4_GZ_MODEL_POSE\"  make px4_sitl $PX4_UAV_MODEL HEADLESS=1" C-m

# Pane 1: Micro XRCE-DDS Agent
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.1 "
echo 'Waiting for PX4 SITL process to start...'
while ! ps aux | grep -E '[p]x4_sitl' >/dev/null; do
    sleep 1
done
echo 'PX4 SITL detected. Starting Micro XRCE-DDS Agent...'
MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT" C-m

# Pane 2: ROS2 image bridge
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.2 'bash -c "source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && sleep 6 && ros2 run ros_gz_image image_bridge /camera"' C-m

# Pane 3: Gazebo GUI (optional, can attach to same simulation)
sim_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $sim_pane "sleep 5; gz sim /app/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf" C-m

# Create a new tmux window for ROS2 tools
tmux new-window -t $SESSION_NAME -n "ros2_tools"
tmux send-keys -t $SESSION_NAME:1 "bash -c 'source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 topic list'" C-m

# Select first pane in first window
tmux select-window -t $SESSION_NAME:0
tmux select-pane -t $SESSION_NAME:0.0


# Attach to tmux session
tmux attach-session -t $SESSION_NAME
