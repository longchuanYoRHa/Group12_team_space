# 🚀 Group 12: One-Command Operations

This guide provides single-line commands to launch each system component directly from your **laptop terminal**. All commands assume the NUC is at `10.42.0.227`.

---

## 👁️ 1. Vision Node (GUI on Laptop)
Launches the RealSense detection node on the NUC and displays the camera feed on your laptop screen.
```bash
sshpass -p 'team12' ssh -YC leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; source ~/robots/activate && cd ~/vision_pkg && source install/setup.bash; ros2 run vision_pkg rover_vision"
```

---

## 🦾 2. Manipulator Arm (Double-Hop)
Tunnels through the NUC to launch the Cobot controller on the Manipulator's Pi (`10.0.1.3`).
```bash
sshpass -p 'team12' ssh -t leo-rover-12@10.42.0.227 "sshpass -p 'trunk' ssh -o StrictHostKeyChecking=no elephant@10.0.1.3 \"export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws && source install/setup.bash; ros2 launch my_cobot_control mycobot_with_tf2.launch.py\""
```

---

## 🧠 3. Central Controller (Integration Tests)
Launch the specific logic module required for your current test. 
*Path: `~/Group12_team_space/System_integration/Ver01_ws`*

### A. Pick and Place Test
```bash
sshpass -p 'team12' ssh -t leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; ros2 launch central_controller module_launch.py"
```

### B. Explore Test
```bash
sshpass -p 'team12' ssh -t leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; ros2 launch central_controller module02_launch.py"
```

### C. Full Control (Starter)
```bash
sshpass -p 'team12' ssh -t leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; ros2 launch central_controller starter_launch.py"
```

---

## 🗺️ 4. Rover RViz Display (GUI on Laptop)
Launches RViz2 on the NUC using the custom Nav2 config and displays the 3D window on your laptop.
```bash
sshpass -p 'team12' ssh -YC leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; cd ~/Group12_team_space/System_integration/Ver01_ws && source install/setup.bash; cd src/central_controller/rviz && rviz2 -d nav2_default_view.rviz"
```

---

### 💡 Quick Tips
*   **X11 Forwarding:** Ensure your laptop has an X-Server running (built-in on Linux; use VcXsrv on Windows) for the Vision and RViz GUIs to appear.
*   **Stopping Nodes:** Press `Ctrl+C` in the terminal to kill the remote process.
*   **Compression:** The `-YC` flag is used on GUI commands to compress data and reduce network lag.

