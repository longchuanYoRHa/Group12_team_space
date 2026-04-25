# 🤖 Rover 12: System Operations

This folder contains the automation and documentation required to launch the distributed ROS 2 system for **Rover 12**.

---

## 🚀 Quick Start (Automated)

To launch all four system components (**Vision, Arm, Controller, and RViz**) simultaneously in a grouped Terminator environment, run:

```bash
chmod +x start.sh
./start.sh
```
*Note: This script will spawn 4 separate windows/tabs labeled with "Rover 12". Logs will remain visible even if a node crashes.*

---

## 🛠️ Folder Contents

### 1. `start.sh` (Main Launcher)
The primary automation script. It handles:
*   Secure NUC login via `sshpass`.
*   X11 Forwarding for Vision and RViz GUIs.
*   Double-hop SSH for the Manipulator Arm.
*   Persistent terminal windows for real-time log monitoring.

### 2. `component_start.md` (Manual Commands)
Refer to this document if you need to:
*   Launch only a **single component** (e.g., just the Vision node).
*   Debug specific network or sourcing issues.
*   Copy-paste individual "One-Command" strings for terminal testing.

---

## 💡 System Requirements

*   **Subnet**: Your laptop must be on the `10.42.0.x` network.
*   **Terminator**: Ensure Terminator is installed (`sudo apt install terminator`).
*   **X-Server**: A running X-Server is required to display the Vision and RViz GUIs on your laptop screen.
*   **Credentials**:
    *   **NUC**: `leo-rover-12` | `team12`
    *   **Arm**: `elephant` | `trunk`
    *   **Rover**: `pi` | `raspberry`

---

## ⚠️ Troubleshooting

1.  **"Connection Refused"**: Ensure the NUC is powered on and reachable at `10.42.0.227`.
2.  **Empty GUI Windows**: Check if you have an active X-Server. On Linux, run `xhost +` to allow connections.
3.  **Missing Topics**: Ensure the Sanity Checker was run recently to synchronize system clocks across all nodes.

---
**Rover 12 | Group 12 Integration**

