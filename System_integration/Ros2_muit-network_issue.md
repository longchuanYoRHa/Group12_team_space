# ROS 2 Multi-Network Communication: Issue & Resolution

## 🔴 The Symptoms

* **Physical & TCP/IP connections were fine:** Devices (NUC, Arm, Rover) could successfully `ping` and `ssh` into each other over Ethernet.
* **Remote ROS 2 topics were invisible:** Running `ros2 topic list` on the NUC showed no topics from the Arm (and vice versa).
* **Local topics disappeared (during troubleshooting):** After applying a basic Fast DDS configuration, even nodes running on the *same* machine could no longer communicate (e.g., TF trees broke, local launch files failed).

## 🔍 Root Cause Analysis

This is a classic ROS 2 networking issue caused by a combination of three factors:

1. **DDS Multicast Confusion (Multi-NIC environment):** The NUC is connected to multiple networks simultaneously (Wi-Fi for internet, specific Ethernet ports for the Arm, Rover, and Docker bridges). By default, Fast DDS broadcasts UDP multicast discovery packets across all interfaces, which often results in packets getting lost or routed to the wrong subnet.
2. **Firewall (UFW) Blocking:** Ubuntu's default Uncomplicated Firewall (UFW) frequently drops incoming UDP multicast traffic, silently killing ROS 2 discovery.
3. **The Localhost & SHM Trap:** When configuring a custom Fast DDS XML to force traffic through a specific network interface, disabling `useBuiltinTransports` also disables the local loopback (`127.0.0.1` / `lo`) and **Shared Memory (SHM)**. Without SHM and `lo`, ROS 2 nodes running on the *same machine* become completely isolated from one another.

---

## ✅ The Solution

To resolve this on the NUC, the Arm, and the Rover, follow these three steps:

### Step 1: Securely Configure the Firewall (UFW)

Instead of disabling the firewall completely on the main PC (NUC), we explicitly trust the specific Ethernet interfaces connected to the robots.
*Note: Run `ip a` to find the exact interface names (e.g., `enp0s31f6` for the Arm, `eth1` for the Rover).*

**On the NUC:**

```bash
sudo ufw allow ssh
sudo ufw allow in on <arm_interface_name>   # e.g., enp0s31f6
sudo ufw allow in on <rover_interface_name> # When the Rover is reconnected
sudo ufw enable

```

**On the Arm / Rover (Isolated devices):**
Since these are directly connected via Ethernet and not exposed to the public internet, you can safely disable their firewalls to prevent issues:

```bash
sudo ufw disable

```

### Step 2: Create a Robust Fast DDS XML Profile

Create a configuration file to explicitly tell ROS 2 which network interfaces to use, while actively preserving Local Loopback and Shared Memory.

Create `~/fastdds_profile.xml` on **all devices** (NUC, Arm, Rover).

**Template for the NUC (Multi-NIC):**

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUDPTransport</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>lo</address>                      <address>enp0s31f6</address>               </interfaceWhiteList>
        </transport_descriptor>
        
        <transport_descriptor>
            <transport_id>CustomSHMTransport</transport_id>
            <type>SHM</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>CustomUDPTransport</transport_id>
                <transport_id>CustomSHMTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>

```

*(For the Arm and Rover, use the exact same template, but change the interface in the `<interfaceWhiteList>` to match their respective Ethernet interfaces, usually `eth0`).*

### Step 3: Apply Configurations and Restart Daemon

1. Add the environment variable to your `~/.bashrc` on **all machines**:
```bash
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_profile.xml" >> ~/.bashrc
source ~/.bashrc

```


2. Flush the old ROS 2 daemon cache to apply the changes immediately:
```bash
ros2 daemon stop
ros2 daemon start

```


---
