# Arm Quick Start Guide

## Run on NUC
```bash
# From NUC, copy the source to Pi if any changes were made
scp -r ~/Group12_team_space/Manipulator/src/my_cobot_control elephant@10.0.1.3:~/ros2_ws/src/

# On NUC — SSH into Pi
ssh elephant@10.0.1.3
trunk

# Manually sync time from NUC to Pi
TS=$(ssh leo-rover-12@10.0.1.4 "date -u +%Y-%m-%d\ %H:%M:%S.%N") || { echo "SSH failed"; exit 1; }
[ -n "$TS" ] || { echo "Empty time string"; exit 1; }
sudo date -u -s "$TS"

# Start the controller (all nodes under /arm namespace)
cd ~/ros2_ws
source install/setup.bash
ros2 launch my_cobot_control mycobot_with_tf2.launch.py
```

# Camera Quick Start Guide

## Run on the NUC
```bash
source /home/leo-rover-12/robots/bin/activate
cd /home/leo-rover-12/vision_pkg
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run vision_pkg rover_vision
```
## Echo the Topic in new terminal on NUC
```bash
ros2 topic echo /target_pick/green
```