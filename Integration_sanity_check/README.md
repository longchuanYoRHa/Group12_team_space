------------------------------
## Sanity Check Script: NUC & Manipulator Integration
This automated bash script verifies connectivity, synchronizes system clocks, and launches ROS 2 nodes across a distributed system consisting of a Laptop, an Intel NUC, and a Cobot Manipulator.
------------------------------
## 🚀 What the Script Does

   1. Network Discovery: Finds the NUC's IP via its MAC address.
   2. 3-Way Time Sync: Syncs Laptop -> NUC -> Manipulator for matching timestamps.
   3. Hardware & ROS 2 Launch: Triggers the Manipulator drivers and fixes USB permissions.
   4. Vision Node Test: Tests the RealSense camera feed and OpenVINO inference for 15 seconds.

------------------------------
## 💻 How to Use## 1. Run the Sanity Check
Run with sudo to allow network scanning.
NUC IP (Default): 10.42.0.227 (Verify this in the script output).

chmod +x sanity_check.sh
sudo ./sanity_check.sh

------------------------------
## 🔍 Viewing Topics (Live Data via SSH)
To view live coordinates (e.g., for the green cube), SSH into the NUC:
## 1. SSH into the NUC

ssh leo-rover-12@10.42.0.227

## 2. Set Environment & Echo Topic
Inside the NUC terminal, run:

export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
# List and Echo topics
ros2 topic echo /target_pick/green

------------------------------
## 📊 Understanding the Output

* [MATCHED]: The topic is live on the network.
* [VISION NOTE]: If "Recent Detections" says [FAILURE], it simply means no colored block was in front of the camera. As long as the topics (e.g., /target_pick/green) are [MATCHED], the camera system is healthy.
* [NOTE]: Time differences of 1-2s are due to SSH latency and do not indicate desync.

------------------------------
## ⚠️ Troubleshooting

* Topic Missing: If /arm/ topics are missing, ensure the Cobot is powered on.
* Vision Fail: Ensure the camera is connected to a USB 3.0 port.

------------------------------
END OF DOCUMENTATION
------------------------------

