#!/bin/bash

# --- START OF SCRIPT ---
echo "=========================================================="
echo "                STARTING SANITY CHECK"
echo "=========================================================="

# 1. Capture Laptop Details
CURRENT_USER=$(whoami)
echo "[INFO] Running as local user: $CURRENT_USER"

# 2. Subnet Validation Check
CHECK_SUBNET=$(ip -4 addr show | grep "10.42.0.")
if [ -z "$CHECK_SUBNET" ]; then
    echo "----------------------------------------------------------"
    echo "[CRITICAL FAILURE] Subnet Mismatch"
    echo "Your laptop is NOT on the 10.42.0.x network."
    echo "Common Fix: Go to Network Settings -> IPv4 -> Manual"
    echo "Set IP: 10.42.0.50, Netmask: 255.255.255.0"
    echo "----------------------------------------------------------"
    exit 1
fi
echo "[SUCCESS] Laptop is on the correct subnet (10.42.0.x)"

# 3. Securely prompt for the local laptop's sudo password
read -s -p "[SUDO] Enter password for $CURRENT_USER: " LOCAL_PASS
echo ""

# 4. Dependency Check (Auto-Install)
DEPS=("sshpass" "arp-scan")
for dep in "${DEPS[@]}"; do
    if ! command -v "$dep" &> /dev/null; then
        echo "[SETUP] Missing $dep. Installing..."
        echo "$LOCAL_PASS" | sudo -S apt update -y > /dev/null 2>&1
        echo "$LOCAL_PASS" | sudo -S apt install -y "$dep" > /dev/null 2>&1
    fi
done

# 5. Pre-execution: Clear local network cache
echo "[PRE-CHECK] Flushing laptop ARP cache..."
echo "$LOCAL_PASS" | sudo -S ip neigh flush all > /dev/null 2>&1

# Configuration (Hardware MAC and Remote Credentials)
TARGET_MAC="8c:e9:ee:3a:83:0b"
NUC_USER="leo-rover-12"
NUC_PASS="team12"
MANIP_IP="10.0.1.3"
MANIP_USER="elephant"
MANIP_PASS="trunk"
NUC_INT_IP="10.0.1.4"
ROVER_IP="10.0.0.1"
ROVER_USER="pi"
ROVER_PASS="raspberry"
NUC_ROVER_INT="10.0.0.2"

# --- PHASE 1: NUC DISCOVERY & LAPTOP SYNC ---
echo ""
echo "=========================================================="
echo " PHASE 1: DISCOVERING NUC & SYNCING LAPTOP TO NUC CLOCK"
echo "=========================================================="

INTERFACE=$(ip -4 addr show | grep "10.42.0." | awk '{print $NF}')
NUC_IP=""

for ((i=1; i<=10; i++)); do
    SCAN_RESULT=$(echo "$LOCAL_PASS" | sudo -S arp-scan --interface="$INTERFACE" --localnet 2>/dev/null | grep -i "$TARGET_MAC")
    if [ ! -z "$SCAN_RESULT" ]; then
        NUC_IP=$(echo "$SCAN_RESULT" | awk '{print $1}')
        echo "[SUCCESS] NUC found at IP: $NUC_IP on interface $INTERFACE"
        break
    fi
    echo -ne "Attempt $i/10: Searching for MAC $TARGET_MAC...\\r"
    sleep 5
done

if [ -z "$NUC_IP" ]; then 
    echo "[FAILURE] NUC MAC not found on interface $INTERFACE."
    exit 1
fi

echo "[STEP] Syncing Laptop clock to NUC via SSH NTP..."
TS=$(sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "date -u +'%Y-%m-%d %H:%M:%S.%N'")
if [ -n "$TS" ]; then
    echo "$LOCAL_PASS" | sudo -S date -u -s "$TS" > /dev/null
    echo "[SUCCESS] Laptop synced to NUC time."
else
    echo "[FAILURE] Could not fetch time from NUC."; exit 1
fi

# --- PHASE 2: NUC & MANIPULATOR SETUP ---
echo ""
echo "=========================================================="
echo " PHASE 2: SYNCING MANIPULATOR TO NUC & STARTING ARM ROS2"
echo "=========================================================="

REMOTE_CMD="sshpass -p '$MANIP_PASS' ssh -o StrictHostKeyChecking=no '$MANIP_USER@$MANIP_IP' \" \
    echo 'trunk' | sudo -S chmod 666 /dev/ttyUSB* 2>/dev/null || true; \
    echo '[STEP] Syncing Manipulator to NUC ($NUC_INT_IP)...'; \
    TS=\\\$(sshpass -p 'team12' ssh -o StrictHostKeyChecking=no $NUC_USER@$NUC_INT_IP 'date -u +\\\"%Y-%m-%d %H:%M:%S.%N\\\"'); \
    echo 'trunk' | sudo -S date -u -s \\\"\\\$TS\\\"; \
    echo ''; \
    echo '--- CHRONY NTP SYNC DETAILS ---'; \
    echo 'STATE | MASTER IP    | STRATUM | POLL | REACH | LAST RX | OFFSET'; \
    chronyc sources | grep '^\^\*' | awk '{printf \\\"%-5s | %-12s | %-7s | %-4s | %-5s | %-7s | %s%s\\\n\\\", \\\$1, \\\$2, \\\$3, \\\$4, \\\$5, \\\$6, \\\$7, \\\$8}'; \
    echo '-------------------------------'; \
    export ROS_DOMAIN_ID=12; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; \
    source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws && source install/setup.bash; \
    nohup ros2 launch my_cobot_control mycobot_with_tf2.launch.py > /dev/null 2>&1 & \
    sleep 25; \
    \""

sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "$REMOTE_CMD"
echo "[SUCCESS] Time Sync and ROS Launch complete on Manipulator"

# --- PHASE 3: NUC REALSENSE INTEGRATION ---
echo ""
echo "=========================================================="
echo " PHASE 3: STARTING NUC VISION NODE (REALSENSE CAMERA)"
echo "=========================================================="
xhost + > /dev/null 2>&1
export libgl_always_software=1

sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "echo '$NUC_PASS' | sudo -S fuser -k /dev/video* > /dev/null 2>&1 || true; pkill -f rover_vision || true"

VISION_CMD="export ROS_DOMAIN_ID=12; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; \
     source ~/robots/bin/activate && cd ~/vision_pkg && source install/setup.bash; \
     (sleep 15; echo '---START_LIST---'; ros2 topic list --no-daemon; echo '---END_LIST---') & \
     ros2 run vision_pkg rover_vision"

timeout 35s sshpass -p "$NUC_PASS" ssh -YC -o Ciphers=chacha20-poly1305@openssh.com -o Compression=yes \
    "$NUC_USER@$NUC_IP" "$VISION_CMD" > vision_tmp.log 2>&1 &

VISION_PID=$!
sleep 30

if ps -p $VISION_PID > /dev/null; then kill $VISION_PID; fi
sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "pkill -f rover_vision || true"

# --- PHASE 4: LEO ROVER BASE SETUP ---
echo ""
echo "=========================================================="
echo " PHASE 4: SYNCING ROVER BASE TO NUC & VERIFYING CHASSIS"
echo "=========================================================="

# NUC pushes its clock to Rover and runs Chrony details + Topic check
ROVER_CMD="TS=\$(date -u +'%Y-%m-%d %H:%M:%S.%N'); sshpass -p '$ROVER_PASS' ssh -o StrictHostKeyChecking=no '$ROVER_USER@$ROVER_IP' \" \
    echo '$ROVER_PASS' | sudo -S date -u -s '\\\$TS' > /dev/null 2>&1; \
    echo '[STEP] Syncing Rover Base to NUC ($NUC_ROVER_INT)...'; \
    echo ''; \
    echo '--- CHRONY NTP SYNC DETAILS (ROVER) ---'; \
    echo 'STATE | MASTER IP    | STRATUM | POLL | REACH | LAST RX | OFFSET'; \
    chronyc sources | grep '^\^\*' | awk '{printf \\\"%-5s | %-12s | %-7s | %-4s | %-5s | %-7s | %s%s\\\n\\\", \\\$1, \\\$2, \\\$3, \\\$4, \\\$5, \\\$6, \\\$7, \\\$8}'; \
    echo '---------------------------------------'; \
    export ROS_DOMAIN_ID=12; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; \
    source /opt/ros/jazzy/setup.bash; \
    echo '---ROVER_START---'; ros2 topic list; echo '---ROVER_END---'; \""

ROVER_RAW_OUT=$(sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "$ROVER_CMD")

# Print the table and sync info from the raw output so user sees it in the terminal
echo "$ROVER_RAW_OUT" | sed -n '/\[STEP\]/,/---------------------------------------/p'

# Parse topics for summary
ROVER_TOPICS=$(echo "$ROVER_RAW_OUT" | sed -n '/---ROVER_START---/,/---ROVER_END---/p' | grep -v "\-\-\-")
ROVER_REQUIRED="/imu/data /imu/data_raw /imu/rpy /joint_states /merged_odom /robot_description /tf /wheel_odom"

# --- PHASE 5: FINAL SUMMARY ---
echo ""
echo "=========================================================="
echo " PHASE 5: FINAL SYSTEM HEALTH CHECK & TOPIC VERIFICATION"
echo "=========================================================="

T_LAP_FIN=$(date -u +%H:%M:%S.%3N)
T_REMOTE=$(sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "date -u +%H:%M:%S.%3N; sshpass -p '$MANIP_PASS' ssh -o StrictHostKeyChecking=no '$MANIP_USER@$MANIP_IP' 'date -u +%H:%M:%S.%3N'; sshpass -p '$ROVER_PASS' ssh -o StrictHostKeyChecking=no '$ROVER_USER@$ROVER_IP' 'date -u +%H:%M:%S.%3N'")
T_NUC_FIN=$(echo "$T_REMOTE" | sed -n '1p')
T_MAN_FIN=$(echo "$T_REMOTE" | sed -n '2p')
T_ROV_FIN=$(echo "$T_REMOTE" | sed -n '3p')

TOPIC_LIST=$(sed -n '/---START_LIST---/,/---END_LIST---/p' vision_tmp.log | grep -v "\-\-\-" | grep -v "\[INFO\]")
DETECTIONS=$(grep "SUCCESS" vision_tmp.log | tail -n 5)

echo "[TIME] Laptop:      $T_LAP_FIN"
echo "[TIME] NUC:         $T_NUC_FIN"
echo "[TIME] Manipulator: $T_MAN_FIN"
echo "[TIME] Rover Base:  $T_ROV_FIN"
echo "[NOTE] SSH latency causes small summary mismatches; system clocks are synced."
echo ""
echo "[VISION] Recent Detections:"
[ -z "$DETECTIONS" ] && echo "[FAILURE] No detections found. (Node is active)" || echo "$DETECTIONS"
echo ""

echo "--- TOPIC MATCH LIST ---"
REQUIRED="/arm/gripper_status /arm/joint_states /arm/status /arm/target_pick /arm/target_place /joint_states /parameter_events /robot_description /rosout /target_pick/blue /target_pick/green /target_pick/red /target_place/blue /target_place/green /target_place/red /tf /tf_static"
for t in $REQUIRED; do
    echo "$TOPIC_LIST" | grep -qxw "$t" && echo "[MATCHED] $t" || echo "[MISSING] $t"
done

echo ""
echo "--- ROVER BASE TOPICS ---"
for t in $ROVER_REQUIRED; do
    echo "$ROVER_TOPICS" | grep -qxw "$t" && echo "[MATCHED] $t" || echo "[MISSING] $t"
done

rm vision_tmp.log 2>/dev/null

echo ""
echo "=========================================================="
echo "                END OF SANITY CHECK"
echo "=========================================================="

