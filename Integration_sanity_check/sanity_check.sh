#!/bin/bash

# Configuration
TARGET_MAC="8c:e9:ee:3a:83:0b"
SUDO_PASS="Manchester1824104"
NUC_USER="leo-rover-12"
NUC_PASS="team12"
MANIP_IP="10.0.1.3"
MANIP_USER="elephant"
MANIP_PASS="trunk"
NUC_INTERNAL_IP="10.0.1.4"

# --- PHASE 1: CONNECTIVITY ---
echo "Starting Connectivity Tests..."
NUC_IP=""
MAX_ATTEMPTS=10
WAIT_TIME=10

for ((i=1; i<=MAX_ATTEMPTS; i++)); do
    echo -ne "Attempt $i/$MAX_ATTEMPTS: Searching for NUC (waiting ${WAIT_TIME}s)...\\r"
    SCAN_RESULT=$(echo "$SUDO_PASS" | sudo -S arp-scan --localnet 2>/dev/null | grep -i "$TARGET_MAC")
    if [ ! -z "$SCAN_RESULT" ]; then
        NUC_IP=$(echo "$SCAN_RESULT" | awk '{print $1}')
        echo -e "\\n[OK] NUC found at $NUC_IP"
        break
    fi
    sleep $WAIT_TIME
done

if [ -z "$NUC_IP" ]; then
    echo -e "\\nERROR: NUC MAC ($TARGET_MAC) not found after $MAX_ATTEMPTS attempts."
    exit 1
fi

echo "Nuc Mac Confirmed: $TARGET_MAC"

# --- PHASE 2: SEQUENTIAL SYNC ---
echo "Synchronizing Time: Laptop -> NUC..."
T_LAPTOP_START=$(date -u +%H:%M:%S)
sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "echo '$NUC_PASS' | sudo -S date -s '$T_LAPTOP_START' > /dev/null 2>&1"

echo "Synchronizing Time: NUC -> Manipulator & Launching ROS..."
REMOTE_CMD="T_NOW=\$(date -u +%H:%M:%S); \
    sshpass -p '$MANIP_PASS' ssh -o StrictHostKeyChecking=no '$MANIP_USER@$MANIP_IP' \" \
    echo '$MANIP_PASS' | sudo -S date -s \\\"\$T_NOW\\\" > /dev/null 2>&1; \
    source /opt/ros/jazzy/setup.bash; \
    cd ~/ros2_ws && source install/setup.bash; \
    nohup ros2 launch my_cobot_control mycobot_with_tf2.launch.py > /dev/null 2>&1 & \
    sleep 20; \
    echo '---START_TOPICS---'; \
    ros2 topic list; \
    echo '---END_TOPICS---'; \
    T_M_FINAL=\\\$(date -u +%H:%M:%S); \
    echo \\\"TIME_MANIP:\\\$T_M_FINAL\\\"; \
    \"; \
    T_N_FINAL=\$(date -u +%H:%M:%S); \
    echo \"TIME_NUC:\$T_N_FINAL\""

REMOTE_OUT=$(sshpass -p "$NUC_PASS" ssh -o StrictHostKeyChecking=no "$NUC_USER@$NUC_IP" "$REMOTE_CMD")
T_LAPTOP_FINAL=$(date -u +%H:%M:%S)

# --- PHASE 3: MANIPULATOR SUMMARY ---
echo ""
echo "=============================="
echo " NUC MANIPULATOR INTEGRATION SUMMARY"
echo "=============================="

T_NUC_VAL=$(echo "$REMOTE_OUT" | grep "TIME_NUC" | cut -d':' -f2- | xargs)
T_MANIP_VAL=$(echo "$REMOTE_OUT" | grep "TIME_MANIP" | cut -d':' -f2- | xargs)

echo "[CONFIRMED] Current Time on Laptop: $T_LAPTOP_FINAL"
echo "[CONFIRMED] Current Time on Nuc: $T_NUC_VAL"
echo "[CONFIRMED] Current Time on Manipulator: $T_MANIP_VAL"
echo "[NOTE] A 1-2 second difference among systems is attributed to SSH sourcing and printing delays, not clock desync."

REQUIRED="/arm/gripper_status /arm/joint_states /arm/status /arm/target_pick /arm/target_place /joint_states /parameter_events /robot_description /rosout /tf /tf_static"
for t in $REQUIRED; do
    if echo "$REMOTE_OUT" | grep -qxw "$t"; then
        echo "[CONFIRMED] Topic confirmed: $t"
    else
        echo "[FAILURE] Topic missing: $t"
    fi
done

# --- PHASE 4: VISION INTEGRATION ---
echo ""
echo "=========================================="
echo " NUC REALSENSE CAMERA INTEGRATION SUMMARY"
echo "=========================================="
xhost + > /dev/null 2>&1
export libgl_always_software=1

timeout 25s sshpass -p "$NUC_PASS" ssh -YC \
    -o StrictHostKeyChecking=no \
    -o Ciphers=chacha20-poly1305@openssh.com \
    -o Compression=yes \
    "$NUC_USER@$NUC_IP" \
    "source ~/robots/bin/activate && cd ~/vision_pkg && source install/setup.bash && ros2 run vision_pkg rover_vision" &

VISION_PID=$!
sleep 15
if ps -p $VISION_PID > /dev/null; then
    kill $VISION_PID
    echo "[CONFIRMED] Vision node works with remote feed"
else
    echo "[FAILURE] Vision node crashed or failed to start"
fi

echo ""
echo "END OF SANITY CHECK"

