------------------------------
## Sanity Check Script: NUC & Manipulator Integration
This automated bash script verifies connectivity, synchronizes system clocks, and launches ROS 2 nodes across a distributed system consisting of a Laptop, an Intel NUC, and a Cobot Manipulator.
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
The following packages and environments must be present on the remote systems:

* ROS 2: Jazzy Jalisco (installed and sourced in ~/.bashrc).
* Python Libraries: pyrealsense2, opencv-python, numpy, ultralytics, and openvino.
* SSH: SSH server enabled on both NUC and Manipulator.

------------------------------
## 💻 How to Use## 1. Download the script
Download the sanity_check.sh file from this repository to your local machine.
## 2. Make the script executable
Grant execution permissions:

chmod +x sanity_check.sh

## 3. Run the Sanity Check
Because arp-scan requires raw socket access, the script must be run with sudo:

sudo ./sanity_check.sh

------------------------------
## 🎥 Manual Remote Camera Access
If you need to access the camera feed manually, use the command below.
Note: If the IP address has changed, first run sudo arp-scan --localnet | grep -i 8c:e9:ee:3a:83:0b to find the current NUC IP.
NUC MAC Address: 8c:e9:ee:3a:83:0b

# Enable software rendering and launch the vision node via SSH
export libgl_always_software=1 && ssh -YC -o Ciphers=chacha20-poly1305@openssh.com -o Compression=yes leo-rover-12@<NUC_IP_HERE> "source ~/robots/bin/activate && cd ~/vision_pkg && source install/setup.bash && ros2 run vision_pkg rover_vision"

------------------------------
## 📊 Understanding the Output

* [CONFIRMED]: The step or connection was successful.
* [FAILURE]: A specific topic or node is missing; check hardware power and USB cables.
* [NOTE]: System time differences of 1-2 seconds are attributed to SSH sourcing and printing delays and do not indicate a clock synchronization failure.

------------------------------
## ⚠️ Troubleshooting

* Vision Node Failure: Ensure your laptop is running an X server and the NUC has the camera connected via USB 3.0.
* Topic Missing: If /arm/ topics are missing, ensure the Cobot is powered on and the USB-Serial connection is active.

------------------------------
END OF DOCUMENTATION
------------------------------

