# Team Documentation: NTP Time Synchronization for Rover Node

## 1. Overview
In a multi-robot system (ROS 2), precise time synchronization is critical for TF2 transforms, sensor fusion (LiDAR/Camera), and distributed control. This document guides the configuration of the **Rover** (Client) to synchronize its clock with the **NUC** (Master Time Server).

- **Master (NUC) IP:** `10.0.0.2` (Rover-facing interface); `10.0.1.4` (Arm-facing interface)
- **Client (Rover) IP:** `10.0.0.1`
- **Client (Arm) IP:** `10.0.1.3`
- **Protocol:** NTP (via Chrony)

---

## 2. Prerequisites
- The Rover must be able to ping the NUC: `ping 10.0.0.2`
- Operating System: Ubuntu 22.04 / 24.04 (ROS 2 Jazzy/Humble)

---

## 3. Configuration Steps

### Step 1: Install Chrony
Ensure `chrony` is installed on the Rover:
```bash
sudo apt update
sudo apt install chrony -y
```

### Step 2: Disable Conflicting Services
To prevent interference, disable the default `systemd-timesyncd` and the old `ntp` service:
```bash
# Stop and disable systemd-timesyncd
sudo systemctl stop systemd-timesyncd
sudo systemctl disable systemd-timesyncd

# Mask the old ntp service (prevents it from being started by other processes)
sudo systemctl stop ntp || true
sudo systemctl disable ntp || true
sudo systemctl mask ntp
```

### Step 3: Configure Chrony Client
Edit the configuration file:
```bash
sudo nano /etc/chrony/chrony.conf
```

**Perform the following modifications:**
1.  **Comment out default pools:** Add a `#` before any lines starting with `pool` or `server` (e.g., `# pool ntp.ubuntu.com...`).
2.  **Add NUC as the Master Server:** Add the following lines at the end of the file:
    ```text
    # Sync with NUC Master Server
    server 10.0.0.2 iburst minpoll 4 maxpoll 6

    # Allow immediate clock step if error > 1s (Critical for battery-powered robots)
    makestep 1 -1
    ```
3.  **Disable DHCP NTP sources (Optional but Recommended):**
    Comment out the line: `# sourcedir /run/chrony-dhcp`

Save and exit (`Ctrl+O`, `Enter`, `Ctrl+X`).

### Step 4: Apply Changes
Restart the Chrony service using the primary service name:
```bash
sudo systemctl restart chrony
sudo systemctl enable chrony
```

---

## 4. Verification

### Check Synchronization Status
Run the following command to see if the Rover is tracking the NUC:
```bash
chronyc sources -v
```
**Success Criteria:**
- You should see `10.0.0.2` in the list.
- Look for the **`^*`** symbol before the IP.
    - `^*` means "Current best synced".
    - `^?` means "Unreachable" (Check NUC firewall or network).

### Check Time Offset
To see the precision of the sync:
```bash
chronyc tracking
```
Look at **"Last offset"**. In a local network, this should typically be under `0.001s` (1 millisecond).

---

## 5. Troubleshooting
- **Firewall:** If the status is `^?`, ensure the NUC allows UDP traffic on port 123: 
  `sudo ufw allow 123/udp` (on NUC).
- **Service Name Error:** If `systemctl` complains about "alias name", always use `chrony` instead of `chronyd`.
- **Manual Step:** To force an immediate sync, run: `sudo chronyc makestep`.
"""

with open("Rover_NTP_Configuration_Guide.md", "w") as f:
    f.write(content)