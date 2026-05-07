#!/bin/bash

# Configuration
NUC_IP="10.42.0.227"
NUC_USER="leo-rover-12"
NUC_PASS="team12"
DOMAIN="12"
# Path to the .rviz file on YOUR laptop
LOCAL_RVIZ_PATH="/home/student04/Design_Project_Git_Repository/Group12_team_space/System_integration/Ver01_ws/src/central_controller/rviz/nav2_default_view.rviz"

echo "🚀 Launching Rover 12: PICK AND PLACE TEST..."

# --- LOCAL PREP ---
# Refresh the local ROS 2 daemon to ensure it's not holding onto old discovery data
export ROS_DOMAIN_ID=$DOMAIN
ros2 daemon stop
ros2 daemon start

# 1. VISION (NUC)
terminator -T "Rover 12: VISION" -e "sshpass -p '$NUC_PASS' ssh -YC $NUC_USER@$NUC_IP \"export ROS_DOMAIN_ID=$DOMAIN; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; source ~/robots/bin/activate && cd ~/vision_pkg && source install/setup.bash; ros2 run vision_pkg rover_vision; bash\"" &

sleep 1 

# 2. ARM (Double SSH to Elephant)
terminator -T "Rover 12: ARM" -e "sshpass -p '$NUC_PASS' ssh -t $NUC_USER@$NUC_IP \"sshpass -p 'trunk' ssh -o StrictHostKeyChecking=no elephant@10.0.1.3 'export ROS_DOMAIN_ID=$DOMAIN; source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws && source install/setup.bash; ros2 launch my_cobot_control mycobot_with_tf2.launch.py; bash'\"" &

sleep 1

# 3. CONTROLLER (Pick and Place - On NUC)
terminator -T "Rover 12: CONTROLLER" -e "sshpass -p '$NUC_PASS' ssh -t $NUC_USER@$NUC_IP \"export ROS_DOMAIN_ID=$DOMAIN; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; ros2 launch central_controller module_launch.py; bash\"" &

sleep 2

# 4. RVIZ (Running LOCALLY on your Laptop)
# We add LOCALHOST_ONLY=0 and DISCOVERY_RANGE=SUBNET to make sure your laptop looks for the NUC and Arm
terminator -T "Rover 12: LOCAL RVIZ" -e "bash -c \"
    export ROS_DOMAIN_ID=$DOMAIN; 
    export ROS_LOCALHOST_ONLY=0; 
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET;
    source /opt/ros/jazzy/setup.bash; 
    echo 'Launching RViz... If blank, check Fixed Frame in Global Options.';
    rviz2 -d $LOCAL_RVIZ_PATH; 
    exec bash\"" &

echo "✅ All Rover 12 Terminator windows spawned."
