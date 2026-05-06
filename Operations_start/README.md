# 🤖 Rover 12: System Operations

This folder contains the automation and documentation required to launch the distributed ROS 2 system for **Rover 12**.

---

## 🚀 Quick Start (Automated Launchers)

Choose the script that matches your current testing phase. These scripts will spawn 4 separate **Terminator** windows for Vision, the Arm, the Central Controller, and RViz.

### 1. Pick and Place Test
```bash
chmod +x start_pick_place.sh
./start_pick_place.sh
```

### 2. Autonomous Explore Test
```bash
chmod +x start_explore.sh
./start_explore.sh
```

### 3. Full System Control (Starter)
```bash
chmod +x start_full_control.sh
./start_full_control.sh
```
*Note: Terminal windows will stay open even if a node crashes, allowing you to debug the logs.*

---

## 🛠️ Folder Contents

### 1. Automation Scripts (`.sh`)
The primary launchers for the system. They handle:
*   **Secure Logins**: Automated authentication via `sshpass`.
*   **X11 Forwarding**: Tunnels Vision and RViz GUIs directly to your laptop.
*   **Double-Hop SSH**: Reaches the Manipulator Pi through the NUC.
*   **Environment Sourcing**: Automatically sources ROS 2 Jazzy and your local workspaces.

### 2. `component_start.md` (Manual & Individual Commands)
Refer to this document if you need to:
*   Launch only a **single component** (e.g., just the Arm).
*   Use individual "One-Command" strings for debugging in a single terminal.

---

## 💡 System Requirements

*   **Network**: Laptop must be on the **10.42.0.x** subnet.
*   **Terminator**: Required for multi-window spawning (`sudo apt install terminator`).
*   **X-Server**: Must be active for GUIs to appear. (Linux: `xhost +` | Windows: Xming/VcXsrv).
*   **Credentials**:
    *   **NUC**: `leo-rover-12` | `team12`
    *   **Arm**: `elephant` | `trunk`
    *   **Rover**: `pi` | `raspberry`

---

## ⚠️ Troubleshooting

1.  **Node Crashes on Startup**: Ensure you have run the **Sanity Checker** first to sync all system clocks.
2.  **"sshpass: command not found"**: Install it on your laptop using `sudo apt install sshpass`.
3.  **Controller Fails to Source**: Ensure the workspace path `~/Group12_team_space/System_integration/Ver01_ws` exists on the NUC.

---
**Rover 12 | Group 12 Integration**

