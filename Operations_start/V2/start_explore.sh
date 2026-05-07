#!/bin/bash

# Configuration
NUC_IP="10.42.0.227"
NUC_USER="leo-rover-12"
NUC_PASS="team12"
DOMAIN="12"

# Path to the .rviz file on YOUR laptop
LOCAL_RVIZ_PATH="/home/student04/Design_Project_Git_Repository/Group12_team_space/System_integration/Ver01_ws/src/central_controller/rviz/nav2_default_view.rviz"

echo "🚀 Launching Rover 12: EXPLORE TEST..."

# --- LOCAL NETWORK PREP ---
# Force laptop to reset its discovery cache to see the NUC
export ROS_DOMAIN_ID=$DOMAIN
ros2 daemon stop
ros2 daemon start

sleep 1

# 1. CONTROLLER (Explore) - Running on NUC
# Using -t to ensure the shell stays open and responsive
terminator -T "Rover 12: CONTROLLER" -e "sshpass -p '$NUC_PASS' ssh -t $NUC_USER@$NUC_IP \"export ROS_DOMAIN_ID=$DOMAIN; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; ros2 launch central_controller module02_launch.py; bash\"" &

sleep 2

# 2. RVIZ - Running LOCALLY on your laptop
# Added DISCOVERY_RANGE=SUBNET to ensure the map/lidar data is found over Wi-Fi
terminator -T "Rover 12: LOCAL RVIZ" -e "bash -c \"
    export ROS_DOMAIN_ID=$DOMAIN;
    export ROS_LOCALHOST_ONLY=0;
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET;
    source /opt/ros/jazzy/setup.bash;
    echo 'Waiting for NUC nodes to appear...';
    sleep 2;
    rviz2 -d $LOCAL_RVIZ_PATH;
    exec bash\"" &

echo "✅ All Rover 12 Terminator windows spawned."
echo "Note: If RViz is still blank, check the 'Fixed Frame' in the left panel and change it to 'base_link'."
