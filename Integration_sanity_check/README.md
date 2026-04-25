# 🛡️ Universal Sanity Check: NUC, Rover & Manipulator Integration

This automated tool verifies connectivity, synchronizes system clocks to the nanosecond, and launches ROS 2 nodes across a distributed system (**Any Linux Laptop**, **Intel NUC**, **Manipulator Arm**, and **Leo Rover Base**).

---

## 🚀 What the Script Does

1.  **Auto-Portability Check**: Detects the local username and prompts for the local password securely.
2.  **Dependency Manager**: Automatically installs `sshpass` and `arp-scan` on the laptop if missing.
3.  **Subnet Safety Gate**: Ensures the laptop is on the `10.42.0.x` network before starting.
4.  **Network Discovery**: Dynamically finds the NUC via MAC address and identifies the active interface.
5.  **4-Way Nano-Sync**: Aligns **Laptop ↔ NUC ↔ Manipulator ↔ Rover Base** clocks to prevent ROS 2 TF (Transform) errors.
6.  **Chassis & Vision Audit**: Verifies that the Rover's IMU/Odometry stack and the NUC’s RealSense vision node are active and publishing.

---

## 💻 How to Use (One-Command Operations)

### 1. Run the Full Sanity Check
This sets up all clocks, installs tools, and starts the remote nodes.
```bash
chmod +x sanity_check.sh
./sanity_check.sh
```

### 2. View Vision GUI on Laptop (One Command)
Tunnels the graphical output from the NUC directly to your laptop screen via X11 Forwarding.
```bash
sshpass -p 'team12' ssh -YC leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; source ~/robots/bin/activate && cd ~/vision_pkg && source install/setup.bash; ros2 run vision_pkg rover_vision"
```

### 3. Live Topic Stream (One Command)
Monitor coordinates or raw data from your laptop terminal without manually logging into the NUC.
```bash
# Example: Stream green block coordinates
sshpass -p 'team12' ssh leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; source /opt/ros/jazzy/setup.bash; ros2 topic echo /target_pick/green"
```

---

## 📊 Understanding the Output

### 1. Chrony NTP Table Breakdown
Look for the **System Peer Lock** in Phase 2 (Arm) and Phase 4 (Rover):
*   **STATE (^\*)**: The `*` indicates a perfect sync with the NUC master clock.
*   **REACH (377)**: Indicates a 100% stable connection history (8/8 successful packets).
*   **OFFSET**: Precision timing. Lower is better (e.g., `+16us` or `0.000016s`).

### 2. Vision Detections & Node Status
*   **[VISION NOTE]**: If "Recent Detections" says `[FAILURE]`, the node is working perfectly as long as the topics are `[MATCHED]`. It simply means no colored block was in the camera's view during the 30-second test.

---

## ⚠️ Troubleshooting (Universal Fixes)

*   **[CRITICAL FAILURE] Subnet Mismatch**: Your laptop is on the wrong network.
    *   **Fix**: Go to Settings -> Network -> IPv4. Change to **Manual**. Set IP: `10.42.0.50` | Netmask: `255.255.255.0`.
*   **Missing /arm/ Topics**: Ensure the Cobot base has a **Green Light** and the USB cable is plugged into a **USB 3.0 (Blue)** port on the NUC.
*   **No GUI Appears**: Ensure your laptop has an **X-Server** running (built-in on Linux; use Xming/VcXsrv on Windows).
*   **Rover Sync Failure**: If Rover Base time shows `00:00:0X`, check the internal connection to `10.0.0.1`.

---
**END OF DOCUMENTATION**

