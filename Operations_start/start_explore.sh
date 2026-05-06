#!/bin/bash

# Configuration
NUC_IP="10.42.0.227"
NUC_USER="leo-rover-12"
NUC_PASS="team12"

echo "🚀 Launching Rover 12: EXPLORE TEST..."

sleep 1

# 3. CONTROLLER (Explore)
terminator -T "Rover 12: CONTROLLER" -e "sshpass -p '$NUC_PASS' ssh -t $NUC_USER@$NUC_IP \"export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; ros2 launch central_controller module02_launch.py; bash\"" &

sleep 1

# 4. RVIZ
terminator -T "Rover 12: RVIZ" -e "sshpass -p '$NUC_PASS' ssh -YC $NUC_USER@$NUC_IP \"export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; cd src/central_controller/rviz && rviz2 -d nav2_default_view.rviz; bash\"" &

echo "✅ All Rover 12 Terminator windows spawned."

