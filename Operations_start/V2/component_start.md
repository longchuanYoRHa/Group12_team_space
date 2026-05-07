# 🚀 Group 12: One-Command Operations

This guide provides single-line commands to launch each system component. By running **RViz locally**, we offload rendering tasks to the laptop, preserving the NUC's battery and CPU for YOLO and Navigation.

---

## 👁️ 1. Vision Node (GUI on Laptop)
Launches the detection node on the NUC. The `-tYC` flags ensure a stable terminal and X11 Forwarding to your laptop.
```bash
sshpass -p 'team12' ssh -tYC leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; source ~/robots/bin/activate && cd ~/vision_pkg && source install/setup.bash; ros2 run vision_pkg rover_vision; bash"
```

---

## 🦾 2. Manipulator Arm (Double-Hop)
Tunnels through the NUC to the Elephant Robotics board. **Note the `-t` on both SSH calls**—this is required to keep the terminal interactive so `bash` stays open.
```bash
sshpass -p 'team12' ssh -t leo-rover-12@10.42.0.227 "sshpass -p 'trunk' ssh -t -o StrictHostKeyChecking=no elephant@10.0.1.3 \"export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws && source install/setup.bash; ros2 launch my_cobot_control mycobot_with_tf2.launch.py; bash\""
```

---

## 🧠 3. Central Controller (NUC Side)
Launch the specific logic module required for your current mission phase.

### A. Full System (Starter)
*Use this for the complete mission start-to-finish.*
```bash
sshpass -p 'team12' ssh -t leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; ros2 launch central_controller starter_launch.py; bash"
```

### B. Pick and Place Test
*Use this for isolated manipulator testing.*
```bash
sshpass -p 'team12' ssh -t leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; ros2 launch central_controller module_launch.py; bash"
```

---

## 🗺️ 4. Rover RViz Display (Native Laptop)
**RUN THIS LOCALLY.** These environment variables force the laptop to look at the Wi-Fi subnet for the NUC's data.
```bash
export ROS_DOMAIN_ID=12; export ROS_LOCALHOST_ONLY=0; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; source /opt/ros/jazzy/setup.bash; rviz2 -d ~/Design_Project_Git_Repository/Group12_team_space/System_integration/Ver01_ws/src/central_controller/rviz/nav2_default_view.rviz
```

---

## 💡 Quick Tips

*   **PTY Allocation:** We use `ssh -t` (and double `-t` for the arm) to ensure that when a command finishes, the window stays open in a `bash` shell for debugging.
*   **Discovery Fix:** If RViz is empty, run `ros2 daemon stop && ros2 daemon start` on your laptop to refresh the network handshake.
*   **Fixed Frame:** If the screen is still blank but topics are active (check with `ros2 topic list`), change the **Fixed Frame** in RViz (left panel) from `map` to `base_link`.
*   **Battery Management:** Running RViz locally saves ~30-40W of power on the NUC, staying well under the 100W safety limit.
*   **Network:** Ensure your laptop is on the same subnet as the NUC (usually `10.42.0.x`).
