````markdown
# Leo Rover ↔ NUC ROS 2 Networking (CycloneDDS) — Setup & Troubleshooting Notes

This document summarizes the steps we used to make a Leo Rover (vehicle) and a NUC (laptop/host) communicate over ROS 2 across Wi-Fi hotspot, and how we fixed the common “demo works but robot topics don’t show up” issue.

---

## 1. Network Topology

We used **NUC as a Wi-Fi hotspot** and connected the robot to it.

- **NUC (hotspot gateway):** `10.42.0.1/24` (interface: `wlo1`)
- **Robot (client):** `10.42.0.220/24` (interface: `wlan_int`)
- Robot may also have an internal bridge interface:
  - **Robot internal bridge (not used for NUC comms):** `br0 = 10.0.0.1/24`

> Important: ROS 2 discovery can break when a robot has **multiple interfaces / IPs** (e.g., `br0` and `wlan_int`). We must force DDS to bind to the correct interface.

---

## 2. Basic Network Verification

Check IP addresses:

```bash
ip a
````

Ping test:

* Robot → NUC:

```bash
ping -c 4 10.42.0.1
```

* NUC → Robot:

```bash
ping -c 4 10.42.0.220
```

If ping fails, fix Wi-Fi/hotspot connectivity before touching ROS 2.

---

## 3. ROS 2 Communication Requirements

To ensure both machines can see each other:

* Use the **same ROS_DOMAIN_ID** on both sides (default is `0`)
* Ensure `ROS_LOCALHOST_ONLY=0`
* Use the **same RMW implementation** on both sides
* For hotspot / multi-interface systems, **force DDS network interface binding**

---

## 4. Install CycloneDDS RMW (Both Sides)

On **NUC and robot** (ROS 2 Jazzy):

```bash
sudo apt update
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

---

## 5. Create CycloneDDS Config (Bind to Correct Network Interface)

### 5.1 NUC: bind to `wlo1`

Create `~/cyclonedds.xml`:

```bash
cat > ~/cyclonedds.xml << 'EOF'
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>wlo1</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
EOF
```

### 5.2 Robot: bind to `wlan_int`

Create `~/cyclonedds.xml`:

```bash
cat > ~/cyclonedds.xml << 'EOF'
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>wlan_int</NetworkInterfaceAddress>
    </General>
  </Domain>
</CycloneDDS>
EOF
```

> This prevents CycloneDDS from selecting `br0` (10.0.0.1) and breaking discovery over the hotspot subnet.

---

## 6. Set Environment Variables (Per Terminal)

In **every terminal** where you run ROS 2 nodes:

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

Restart the ROS 2 daemon:

```bash
ros2 daemon stop
ros2 daemon start
```

---

## 7. Minimal End-to-End Test (Recommended)

### Robot (talker)

```bash
ros2 run demo_nodes_cpp talker
```

### NUC (listener)

```bash
ros2 topic list | grep chatter
ros2 topic echo /chatter
```

If this works, the network + DDS layer is functional.

---

## 8. “Demo Works but Robot Topics Don’t Show Up” — Root Cause & Fix

### Symptom

* `/chatter` is visible across machines
* Robot’s original topics (e.g., `/tf`, `/scan`, `/wheel_odom`) are **NOT** visible on NUC

### Root Cause (Typical)

The robot’s “real” nodes are often started by **systemd user services** (LeoOS), which source `/etc/ros/setup.bash`.
If `/etc/ros/setup.bash` does not include the same DDS/Domain settings, those auto-start nodes run in a different ROS graph.

### Fix

Edit `/etc/ros/setup.bash` on the robot and add (or uncomment) these lines:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/<robot_user>/cyclonedds.xml
```

> Use an absolute path for `CYCLONEDDS_URI` so systemd can always find it.
> Replace `<robot_user>` with the actual username (e.g., `pi`, `leo`, etc.).

Restart LeoOS ROS services:

```bash
systemctl --user daemon-reload
systemctl --user restart ros.target
# or individually:
systemctl --user restart ros-nodes.service
systemctl --user restart uros-agent.service
```

Check logs:

```bash
journalctl --user -u ros-nodes.service -n 200 --no-pager
journalctl --user -u ros-nodes.service -f
```

After restart, NUC should see the robot topics.

---

## 9. Optional: Manual Peer Discovery (If Multicast Discovery Is Blocked)

Some hotspot/router setups block multicast discovery. If needed, disable multicast and configure explicit peers.

### NUC config (peer = robot IP)

```xml
<AllowMulticast>false</AllowMulticast>
<Discovery>
  <Peers>
    <Peer address="10.42.0.220"/>
  </Peers>
</Discovery>
```

### Robot config (peer = NUC IP)

```xml
<AllowMulticast>false</AllowMulticast>
<Discovery>
  <Peers>
    <Peer address="10.42.0.1"/>
  </Peers>
</Discovery>
```

Then restart `ros2 daemon` and re-test `/chatter`.

---

## 10. Quick Checklist

* [ ] Both sides can ping each other (10.42.0.x)
* [ ] Same `ROS_DOMAIN_ID`
* [ ] `ROS_LOCALHOST_ONLY=0`
* [ ] Same `RMW_IMPLEMENTATION`
* [ ] `CYCLONEDDS_URI` points to the correct XML
* [ ] XML binds the correct interface (`wlo1` on NUC, `wlan_int` on robot)
* [ ] If robot uses systemd startup: `/etc/ros/setup.bash` contains the DDS env vars
* [ ] Restart systemd `ros.target` after editing `/etc/ros/setup.bash`

---

## Appendix: Useful Commands

Show active ROS environment variables:

```bash
echo "DOMAIN=$ROS_DOMAIN_ID LOCALHOST=$ROS_LOCALHOST_ONLY RMW=$RMW_IMPLEMENTATION URI=$CYCLONEDDS_URI"
```

List nodes/topics:

```bash
ros2 node list
ros2 topic list
```

Inspect systemd units:

```bash
systemctl --user status ros.target
systemctl --user cat ros-nodes.service
systemctl --user cat uros-agent.service
```

```
::contentReference[oaicite:0]{index=0}
```
