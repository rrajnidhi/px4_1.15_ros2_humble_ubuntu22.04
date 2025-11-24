#!/bin/bash

# PX4 + ROS2 Humble + uXRCE-DDS tmux launcher
# Multi-UAV SITL with per-drone MAV_SYS_ID

SESSION_NAME="sitl_px4_ros2_humble_camera"

# per-drone GCS-MAVLink isolation variables
MAV_SYS_ID=${MAV_SYS_ID:-1}
GCS_PORT=${GCS_PORT_PORT:-18570}

# Environment variables (use external values if provided, otherwise default)
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-3}
PX4_UXRCE_DDS_PORT=${PX4_UXRCE_DDS_PORT:-$((8888 + MAV_SYS_ID))}
PX4_UXRCE_DDS_NS=${PX4_UXRCE_DDS_NS:-cam_drone_$MAV_SYS_ID}
PX4_UAV_MODEL=${PX4_UAV_MODEL:-gz_x500_depth}

## Auto position shifting
BASE_POSE_X=268.08
BASE_POSE_Y=-128.22
BASE_POSE_Z=3.86
X_OFFSET=$(echo "($MAV_SYS_ID - 5) * 2" | bc)
PX4_GZ_MODEL_POSE="$(echo "$BASE_POSE_X + $X_OFFSET" | bc),$BASE_POSE_Y,$BASE_POSE_Z,0.00,0,-0.7"

# Step 0: update px4-rc.params with correct MAV_SYS_ID and 
RC_PARAMS_FILE="/app/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
RC_MAVLINK_FILE="$PX4_HOME/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"

if [ -f "$RC_PARAMS_FILE" ]; then
    # Remove any existing line with MAV_SYS_ID
    sed -i '/^MAV_SYS_ID/d' "$RC_PARAMS_FILE"
    # Append the required MAV_SYS_ID
    echo "param set MAV_SYS_ID $MAV_SYS_ID" >> "$RC_PARAMS_FILE"
    sleep 1
fi

if [ -f "$RC_MAVLINK_FILE" ]; then
    # Replace udp_gcs_port_local line
    sed -i "s/^udp_gcs_port_local=.*/udp_gcs_port_local=$GCS_PORT/" "$RC_MAVLINK_FILE"
    echo "[INFO] Set udp_gcs_port_local=$GCS_PORT in $RC_MAVLINK_FILE"
    sleep 1
else
    echo "[WARN] $RC_MAVLINK_FILE not found!"
fi

# Kill any existing session
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Start new tmux session (detached)
tmux new-session -d -s $SESSION_NAME

# Pane 0: PX4 SITL + Gazebo headless
tmux send-keys -t $SESSION_NAME "
cd /app/PX4-Autopilot/;

# Disable multicast, enforce unicast
export PX4_SIM_HOST_ADDR=127.0.0.1
export MAV_BROADCAST=0
export MAV_SYS_ID=$MAV_SYS_ID

ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
PX4_UXRCE_DDS_PORT=$PX4_UXRCE_DDS_PORT \
PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS \
PX4_GZ_MODEL_POSE=\"$PX4_GZ_MODEL_POSE\" \
make px4_sitl $PX4_UAV_MODEL HEADLESS=1
" C-m

# Pane 1: Micro XRCE-DDS Agent
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.1 "
echo 'Waiting for PX4 SITL process to start...'
while ! ps aux | grep -E '[p]x4_sitl' >/dev/null; do
    sleep 1
done
echo 'PX4 SITL detected. Starting Micro XRCE-DDS Agent...'
MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT
" C-m

# Pane 2: ROS2 image bridge
bridge_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
tmux send-keys -t $bridge_pane 'bash -c "source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && sleep 6 && ros2 run ros_gz_image image_bridge /camera"' C-m

# Pane 3: Gazebo GUI (optional) Uncomment if you need to see sim for each UAVs. This might overload your PC.!
sim_pane=$(tmux split-window -v -t $SESSION_NAME:0.0 -P -F "#{pane_id}")
# tmux send-keys -t $sim_pane 'bash -c "sleep 5; gz sim /app/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf"' C-m

# ROS2 tools window
tmux new-window -t $SESSION_NAME -n "ros2_tools"
tmux send-keys -t $SESSION_NAME:1 " bash -c 'source /opt/ros/humble/setup.bash && source /app/ros2_ws/install/setup.bash && ros2 topic list' " C-m

# Select first pane in first window
tmux select-window -t $SESSION_NAME:0
tmux select-pane -t $SESSION_NAME:0.0

# Attach to tmux session
tmux attach-session -t $SESSION_NAME
