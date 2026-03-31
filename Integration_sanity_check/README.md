------------------------------
## Sanity Check Script: NUC & Manipulator Integration
This automated bash script verifies connectivity, synchronizes system clocks, and launches ROS 2 nodes across a distributed system consisting of a Laptop, an Intel NUC, and a Cobot Manipulator.
------------------------------
## 🚀 What the Script Does

   1. Network Discovery: Uses arp-scan to find the NUC's IP via its MAC address with real-time attempt tracking.
   2. 3-Way Time Sync: Synchronizes the clocks of the Laptop, NUC, and Manipulator sequentially to ensure ROS 2 message timestamps match.
   3. Hardware & ROS 2 Launch: Remotely logs into the Manipulator via the NUC, sources the environment, and launches the my_cobot_control drivers.
   4. Vision Node Test: Opens a remote RealSense camera feed with X11 Forwarding to your laptop for 15 seconds to verify the vision_pkg and OpenVINO inference.

------------------------------
## 🛠 Prerequisites & Dependencies## Local Laptop (Linux)
Ensure your laptop has the following tools installed:

sudo apt update
sudo apt install arp-scan sshpass x11-xserver-utils

## Remote NUC & Manipulator

* ROS 2: Jazzy Jalisco (installed and sourced).
* Python: pyrealsense2, opencv-python, numpy, ultralytics, openvino.
* SSH: Server enabled on both NUC and Manipulator.

------------------------------
## 💻 How to Use## 1. Download & Prepare
Download sanity_check.sh and grant execution permissions:

chmod +x sanity_check.sh

## 2. Run the Sanity Check
Run with sudo to allow network scanning.
Note: The script will confirm the NUC IP (default: 10.42.0.227). Always verify the IP in the script's output before manual access.

sudo ./sanity_check.sh

------------------------------
## 🎥 Manual Remote Camera Access
To launch the vision node manually, use the following command. Replace 10.42.0.227 if the Sanity Check reports a different IP.

export libgl_always_software=1 && ssh -YC -o Ciphers=chacha20-poly1305@openssh.com -o Compression=yes leo-rover-12@10.42.0.227 "export ROS_DOMAIN_ID=12; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; source ~/robots/bin/activate && cd ~/vision_pkg && source install/setup.bash && ros2 run vision_pkg rover_vision"

------------------------------
## 🔍 Viewing Topics (Live Data via SSH)
To view live topic data (e.g., coordinates for the green cube), you must SSH into the NUC and align the ROS environment:
## 1. SSH into the NUC

ssh leo-rover-12@10.42.0.227

## 2. Align Environment & View Data
Inside the NUC terminal, run the following to see the topics:

# Set discovery variables
export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# List and Echo topics
ros2 topic list
ros2 topic echo /target_pick/green

------------------------------
## 📊 Understanding the Output

* [CONFIRMED]: The step or connection was successful.
* [FAILURE]: A topic or node is missing; check hardware power and USB cables.
* [NOTE]: Time differences of 1-2s are due to SSH latency and do not indicate desync.

------------------------------
## ⚠️ Troubleshooting

* Vision Node Failure: Ensure your laptop has an X server running and the camera is on a USB 3.0 port.
* Topic Missing: If /arm/ topics are missing, ensure the Cobot is powered on and /dev/ttyUSB* permissions are set.

------------------------------
END OF DOCUMENTATION
------------------------------