Since the script is now portable and works on any laptop by checking subnets and auto-installing tools, the documentation needs to reflect these new "Safety Checks".
Here is the updated, comprehensive README.md.
------------------------------
## 🛡️ Universal Sanity Check: NUC & Manipulator Integration## This automated tool verifies connectivity, synchronizes system clocks to the nanosecond, and launches ROS 2 nodes across a distributed system (Any Linux Laptop, Intel NUC, and Cobot Manipulator).## 🚀 What the Script Does

   1. Auto-Portability Check: Detects the local username and prompts for the local password securely.
   2. Dependency Manager: Automatically installs sshpass and arp-scan if they are missing.
   3. Subnet Safety Gate: Checks if the laptop is actually on the 10.42.0.x network before starting.
   4. Network Discovery: Dynamically identifies the NUC's IP and the laptop's active network interface.
   5. 3-Way Nano-Sync: Aligns Laptop -> NUC -> Manipulator clocks to prevent ROS 2 TF errors.
   6. Vision & Hardware Audit: Fixes USB permissions and verifies RealSense detection for 30s.

------------------------------
## 💻 How to Use## 1. Run the Sanity Check
The script is now safe for any teammate. It will prompt for your local laptop password to perform network scans and time syncs.

chmod +x sanity_check.sh
./sanity_check.sh

## 2. Manual Debugging: Network Flush
If the NUC is physically connected but "Not Found," clear your laptop's network cache manually:

sudo ip neigh flush all

------------------------------
## 🔍 Viewing Topics (Live Data via SSH)
To view live coordinates after the script finishes, SSH into the NUC (use the IP found in Phase 1):

# Example if NUC is at .227
ssh leo-rover-12@10.42.0.227
# Inside the NUC:
export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 topic echo /target_pick/green

------------------------------
## 📊 Understanding the Output## 1. Chrony NTP Table Breakdown
In Phase 2, look for the System Peer Lock:

* STATE (^*): The * indicates a perfect sync.
* REACH (377): Indicates a 100% stable connection history.
* OFFSET: Precision timing (e.g., -25us). Lower is better for ROS 2.

## 2. Vision Detections & Node Status

* [VISION NOTE]: If "Recent Detections" says [FAILURE], the node is still working perfectly as long as the topics are [MATCHED]. It simply means no block was in the camera's view during the test.

## 3. Time Summary

* [NOTE]: Millisecond differences in the final summary are due to SSH latency, not system clock desync.

------------------------------
## ⚠️ Troubleshooting (Universal Fixes)

* [CRITICAL FAILURE] Subnet Mismatch: Your laptop is on the wrong network.
* Fix: Go to Settings -> Network -> IPv4. Change to Manual.
   * Set IP: 10.42.0.50 | Netmask: 255.255.255.0 | Gateway: Leave blank.
* Phase 1 Failure (NUC not found): Ensure the Ethernet/WiFi link to the Rover is active and the NUC is powered on.
* Missing /arm/ Topics: Ensure the Cobot base has a Green Light and the USB cable is plugged into a USB 3.0 (Blue) port on the NUC.
* Dependency Errors: Ensure your laptop has an internet connection for the first run so it can download sshpass.

------------------------------
## END OF DOCUMENTATION
Does the "Subnet Mismatch" warning help your teammates get their network settings right on the first try?


