
------------------------------
## 🛡️ Sanity Check Script: NUC & Manipulator Integration## This automated bash script verifies connectivity, synchronizes system clocks to the nanosecond, and launches ROS 2 nodes across a distributed system (Laptop, Intel NUC, and Cobot Manipulator).## 🚀 What the Script Does

   1. Pre-Check Cache Flush: Clears the laptop's ARP table to ensure a fresh, error-free connection.
   2. Network Discovery: Dynamically finds the NUC's IP via its MAC address.
   3. 3-Way Nano-Sync: Precise time synchronization (Laptop -> NUC -> Manipulator) to prevent ROS 2 TF transform "future" errors.
   4. NTP Verification: Displays a beautified Chrony table to confirm the Manipulator is locked to the NUC.
   5. Hardware & ROS 2 Launch: Fixes USB permissions and triggers the Manipulator drivers.
   6. Vision Node Test: Captures 30s of RealSense data and OpenVINO inference to verify object detection.

------------------------------
## 💻 How to Use## 1. Run the Sanity Check
The script handles the sudo scan internally. Ensure you are on the correct network subnet (e.g., 10.42.0.x).

chmod +x sanity_check.sh
./sanity_check.sh

## 2. Manual Debugging: Network Flush
If the NUC is physically connected but "Not Found," manually clear your laptop's network cache:

sudo ip neigh flush all

------------------------------
## 🔍 Viewing Topics (Live Data via SSH)
To view live coordinates (e.g., for a detected green cube) after the script finishes:
1. SSH into the NUC:

ssh leo-rover-12@10.42.0.227

2. Set Environment & Echo Topic:

export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 topic echo /target_pick/green

------------------------------
## 📊 Understanding the Output## 1. Chrony NTP Table Breakdown
In Phase 2, the script confirms if the Manipulator is "listening" to the NUC's clock.

* STATE (^*): The * indicates the "Current Best" peer. This confirms a perfect sync.
* MASTER IP: The NUC’s bridge IP (10.0.1.4) serving as the time master.
* REACH: 377 is a perfect score (successful communication on all recent attempts).
* OFFSET: The time difference. Microsecond values (e.g., -25us) are ideal for ROS 2.

## 2. Vision Detections & Node Status

* [VISION NOTE]: If "Recent Detections" says [FAILURE], it simply means no colored block was physically placed in front of the camera during the test.
* CRITICAL: As long as the topics (e.g., /target_pick/green) are listed as [MATCHED], the Vision Node is working perfectly regardless of whether a block is currently being detected.

## 3. Time Summary

* [NOTE]: You will see slight millisecond differences in the final summary. This is purely due to SSH network latency. The internal system clocks are locked and synced.

------------------------------
## ⚠️ Troubleshooting

* Phase 1 Failure (NUC not found): Ensure your laptop IP is manually set in the 10.42.0.x range and that the Ethernet cable is secure.
* Phase 2 Failure (NTP Lock missing): If Chrony shows ? instead of ^*, the Manipulator cannot see the NUC's NTP server. Check the 10.0.1.4 bridge.
* Topic Missing (/arm/): Ensure the Cobot base is powered on (Green light) and the USB cable is plugged into the NUC.
* Vision Fail: Ensure the camera is in a Blue (USB 3.0) port. If it hangs, physically replug the camera to reset the USB bus.

------------------------------
## END OF DOCUMENTATION


