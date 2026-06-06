#!/bin/bash

# Terminate existing Gazebo, ROS 2 and tmux processes
echo "Killing existing ROS 2, Gazebo, and tmux sessions..."
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
pkill -9 -f ros2 2>/dev/null || true
pkill -9 -f auto_explore 2>/dev/null || true
pkill -9 -f slam_toolbox 2>/dev/null || true
pkill -9 -f component_container_isolated 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
tmux kill-session -t mapping 2>/dev/null || true

# Setup ROS 2 environment
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle

# Build the C++ package
echo "Building ROS 2 C++ package (robot_mapping)..."
cd /home/anshul/robot_mapping_ws_cpp
colcon build --symlink-install --packages-select robot_mapping

# Create a new detached tmux session named 'mapping'
echo "Starting simulation environment in tmux..."
tmux new-session -d -s mapping

# Get initial Pane ID (Pane 0)
pane0=$(tmux display-message -p -F "#{pane_id}" -t mapping:0)

# Pane 0: Gazebo Simulation
tmux send-keys -t $pane0 \
"source /home/anshul/robot_mapping_ws_cpp/install/setup.bash && export TURTLEBOT3_MODEL=waffle && export GZ_SIM_RESOURCE_PATH=/home/anshul/robot_mapping_ws_cpp/models:/opt/ros/jazzy/share && ros2 launch robot_mapping turtlebot3_world.launch.py" C-m

# Split Pane 0 horizontally to create Pane 1 (for SLAM Toolbox)
pane1=$(tmux split-window -h -d -P -F "#{pane_id}" -t $pane0)
tmux send-keys -t $pane1 \
"sleep 15 && source /home/anshul/robot_mapping_ws_cpp/install/setup.bash && ros2 launch slam_toolbox online_async_launch.py" C-m

# Split Pane 0 vertically to create Pane 2 (for Nav2 Bringup)
pane2=$(tmux split-window -v -d -P -F "#{pane_id}" -t $pane0)
tmux send-keys -t $pane2 \
"sleep 20 && source /home/anshul/robot_mapping_ws_cpp/install/setup.bash && ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true slam:=False use_localization:=False params_file:=/home/anshul/robot_mapping_ws_cpp/config/waffle.yaml" C-m

# Split Pane 1 vertically to create Pane 3 (for Wall Explorer)
pane3=$(tmux split-window -v -d -P -F "#{pane_id}" -t $pane1)
tmux send-keys -t $pane3 \
"sleep 28 && source /home/anshul/robot_mapping_ws_cpp/install/setup.bash && ros2 run robot_mapping wall_explorer --ros-args -p use_sim_time:=true" C-m

echo "Waiting for services to spin up before launching RViz..."
sleep 25

echo "Launching RViz2..."
nohup rviz2 -d /home/anshul/robot_mapping_ws_cpp/mapping.rviz >/dev/null 2>&1 &

# Attach to the tmux session (fallback gracefully if no terminal supports attachment)
tmux attach -t mapping || echo "Tmux session 'mapping' started in background. Use 'tmux attach -t mapping' to attach."
