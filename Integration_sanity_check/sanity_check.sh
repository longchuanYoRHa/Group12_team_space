#!/bin/bash

# Configuration
TARGET_MAC="8c:e9:ee:3a:83:0b"
S_PASS="Manchester1824104"
NUC_USER="leo-rover-12"
NUC_PASS="team12"
MANIP_IP="10.0.1.3"
MANIP_USER="elephant"
MANIP_PASS="trunk"
NUC_INT_IP="10.0.1.4"

# --- PHASE 1: NUC DISCOVERY ---
echo ""
echo "=============================="
echo "      NUC DISCOVERY"
echo "=============================="
NUC_IP=""
for ((i=1; i<=10; i++)); do
    SCAN_RESULT=$(echo "$S_PASS" | sudo -S arp-scan --localnet 2>/dev/null | grep -i "$TARGET_MAC")
    if [ ! -z "$SCAN_RESULT" ]; then
        NUC_IP=$(echo "$SCAN_RESULT" | awk '{print $1}')
        echo "[SUCCESS] NUC found at IP: $NUC_IP"
        break
    fi
    echo -ne "Attempt $i/10: Searching for MAC $TARGET_MAC...\\r"
    sleep 5
done

if [ -z "$NUC_IP" ]; then echo "[FAILURE] NUC MAC not found."; exit 1; fi

echo "[STEP] Pinging NUC at $NUC_IP..."
ping -c 3 "$NUC_IP" > /dev/null 2>&1 && echo "[SUCCESS] Ping to NUC $NUC_IP successful" || { echo "[FAILURE] NUC unreachable"; exit 1; }

# --- PHASE 2: NUC & MANIPULATOR SETUP ---
echo ""
echo "=============================="
echo " NUC & MANIPULATOR INTEGRATION"
echo "=============================="
echo "[STEP] Verifying Manipulator connectivity from NUC..."
if sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "ping -c 3 $MANIP_IP" > /dev/null 2>&1; then
    echo "[SUCCESS] NUC can reach Manipulator at $MANIP_IP"
else
    echo "[FAILURE] Manipulator $MANIP_IP unreachable from NUC"; exit 1
fi

echo "[STEP] Syncing Master Clock & Fixing USB Permissions..."
T_LAP_START=$(date -u +%H:%M:%S)
sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "echo '$NUC_PASS' | sudo -S date -s '$T_LAP_START' > /dev/null 2>&1"

REMOTE_CMD="sshpass -p '$MANIP_PASS' ssh -o StrictHostKeyChecking=no '$MANIP_USER@$MANIP_IP' \" \
    echo 'trunk' | sudo -S chmod 666 /dev/ttyUSB* 2>/dev/null || true; \
    T_NUC_INT=\\\$(sshpass -p 'team12' ssh -o StrictHostKeyChecking=no leo-rover-12@$NUC_INT_IP 'date -u +%H:%M:%S'); \
    echo 'trunk' | sudo -S date -s \\\"\\\$T_NUC_INT\\\" > /dev/null 2>&1; \
    export ROS_DOMAIN_ID=12; export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; \
    source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws && source install/setup.bash; \
    nohup ros2 launch my_cobot_control mycobot_with_tf2.launch.py > /dev/null 2>&1 & \
    sleep 25; \
    \""

sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "$REMOTE_CMD"
echo "[SUCCESS] Time Sync and ROS Launch complete on Manipulator"

# --- PHASE 3: NUC REALSENSE INTEGRATION ---
echo ""
echo "=============================="
echo "  NUC REALSENSE INTEGRATION"
echo "=============================="
echo "[STEP] Cleaning hardware locks and launching Vision Node..."
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

# --- PHASE 4: FINAL SUMMARY (LIVE CAPTURE) ---
echo ""
echo "=============================="
echo "      FINAL SANITY CHECK"
echo "=============================="

# Capture LIVE times now
T_LAP_FIN=$(date -u +%H:%M:%S)
T_REMOTE=$(sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "date -u +%H:%M:%S; sshpass -p '$MANIP_PASS' ssh -o StrictHostKeyChecking=no '$MANIP_USER@$MANIP_IP' 'date -u +%H:%M:%S'")
T_NUC_FIN=$(echo "$T_REMOTE" | sed -n '1p')
T_MAN_FIN=$(echo "$T_REMOTE" | sed -n '2p')

TOPIC_LIST=$(sed -n '/---START_LIST---/,/---END_LIST---/p' vision_tmp.log | grep -v "\-\-\-" | grep -v "\[INFO\]")
DETECTIONS=$(grep "SUCCESS" vision_tmp.log | tail -n 5)

echo "[TIME] Laptop:      $T_LAP_FIN"
echo "[TIME] NUC:         $T_NUC_FIN"
echo "[TIME] Manipulator: $T_MAN_FIN"
echo "[NOTE] A 1-2s difference is attributed to network delays."
echo ""
echo "[VISION] Recent Detections:"
[ -z "$DETECTIONS" ] && echo "[FAILURE] No detections found." || echo "$DETECTIONS"
echo ""

echo "--- TOPIC MATCH LIST ---"
REQUIRED="/arm/gripper_status /arm/joint_states /arm/status /arm/target_pick /arm/target_place /joint_states /parameter_events /robot_description /rosout /target_pick/blue /target_pick/green /target_pick/red /target_place/blue /target_place/green /target_place/red /tf /tf_static"
for t in $REQUIRED; do
    echo "$TOPIC_LIST" | grep -qxw "$t" && echo "[MATCHED] $t" || echo "[MISSING] $t"
done

echo ""
echo "--- FULL ACTIVE TOPIC LIST ---"
echo "$TOPIC_LIST"

rm vision_tmp.log
echo ""
echo "END OF SANITY CHECK"

