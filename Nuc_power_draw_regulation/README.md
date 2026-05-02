# 🚀 NUC Power Management: Permanent Deployment

This configuration permanently locks the NUC's CPU power consumption. This is a critical safety requirement to ensure that the combined draw of the YOLO model, Lidar SLAM, and the Manipulator Arm does not exceed the Rover battery's BMS trip point.

## 📊 Power Profile Summary
*   **Mode:** Turbo Boost Disabled (Prevents transient current spikes).
*   **PL1 (Sustained):** 35 Watts (Target for long-duration mission stability).
*   **PL2 (Peak/Burst):** 45 Watts (Max allowance for short-term processing spikes).

## 🛠️ Installation Instructions

### 1. Create the Systemd Service
Run the following command to create a new hardware management service:
```bash
sudo nano /etc/systemd/system/nuc-power-limits.service
```

### 2. Add the Configuration
Copy and paste the code block below into the editor:

```ini
[Unit]
Description=Permanent NUC Power Limits for Rover Mission
After=multi-user.target
# Ensures we override default Ubuntu power management
Conflicts=power-profiles-daemon.service

[Service]
Type=oneshot
# Disable Intel Turbo Boost
ExecStart=/bin/bash -c 'echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo'
# Set Sustained Power Limit (PL1) to 35W
ExecStart=/bin/bash -c 'echo 35000000 > /sys/class/powercap/intel-rapl:0/constraint_0_power_limit_uw'
# Set Burst Power Limit (PL2) to 45W
ExecStart=/bin/bash -c 'echo 45000000 > /sys/class/powercap/intel-rapl:0/constraint_1_power_limit_uw'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```
*Save and exit: `Ctrl+O`, `Enter`, `Ctrl+X`.*

### 3. Activate the Profile
Execute these commands to enable the service so it runs automatically at every boot:
```bash
sudo systemctl daemon-reload
sudo systemctl enable nuc-power-limits.service
sudo systemctl start nuc-power-limits.service
```

---

## 🔍 Verification Protocol
After a reboot, run these checks to confirm the hardware has accepted the limits. **These values should never change during a mission.**

| Feature | Verification Command | Expected Value |
| :--- | :--- | :--- |
| **Service Status** | `systemctl status nuc-power-limits` | `active (exited)` |
| **Turbo Boost** | `cat /sys/devices/system/cpu/intel_pstate/no_turbo` | `1` |
| **Sustained (35W)** | `cat /sys/class/powercap/intel-rapl:0/constraint_0_power_limit_uw` | `35000000` |
| **Peak (45W)** | `cat /sys/class/powercap/intel-rapl:0/constraint_1_power_limit_uw` | `45000000` |

---

## ⚠️ Operational Notes
*   **Performance Stability:** By setting a 35W sustained floor, the i7-12700H provides a predictable frame rate for YOLO and smooth physics for PyBullet without overheating.
*   **RViz Offloading:** Always run RViz on your local laptop. Even with these limits, the NUC should not waste its 35W budget on 3D rendering; keep that power available for the Lidar and Vision stacks.
*   **Emergency Revert:** To restore full factory power for heavy compilation tasks, run:  
    `sudo systemctl stop nuc-power-limits.service`
