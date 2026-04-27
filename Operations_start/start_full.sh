#!/bin/bash

# Configuration
NUC_IP="10.42.0.227"
NUC_USER="leo-rover-12"
NUC_PASS="team12"

echo "🚀 Launching Rover 12: FULL CONTROL..."

# 1. VISION
terminator -T "Rover 12: VISION" -e "sshpass -p '$NUC_PASS' ssh -YC $NUC_USER@$NUC_IP \"export ROS_DOMAIN_ID=12; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; source ~/robots/bin/activate && cd ~/vision_pkg && source install/setup.bash; ros2 run vision_pkg rover_vision; bash\"" &

sleep 1 

# 2. ARM
terminator -T "Rover 12: ARM" -e "sshpass -p '$NUC_PASS' ssh -t $NUC_USER@$NUC_IP \"sshpass -p 'trunk' ssh -o StrictHostKeyChecking=no elephant@10.0.1.3 'export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws && source install/setup.bash; ros2 launch my_cobot_control mycobot_with_tf2.launch.py; bash'\"" &

sleep 1

# 3. CONTROLLER (Full Control)
terminator -T "Rover 12: CONTROLLER" -e "sshpass -p '$NUC_PASS' ssh -t $NUC_USER@$NUC_IP \"export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; ros2 launch central_controller starter_launch.py; bash\"" &

sleep 1

# 4. RVIZ
terminator -T "Rover 12: RVIZ" -e "sshpass -p '$NUC_PASS' ssh -YC $NUC_USER@$NUC_IP \"export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; cd src/central_controller/rviz && rviz2 -d nav2_default_view.rviz; bash\"" &

echo "✅ All Rover 12 Terminator windows spawned."

