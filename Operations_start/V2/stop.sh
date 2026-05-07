#!/bin/bash

# Configuration (Matching your launch scripts)
NUC_IP="10.42.0.227"
NUC_USER="leo-rover-12"
NUC_PASS="team12"
ELEPHANT_IP="10.0.1.3"
ELEPHANT_PASS="trunk"

echo "🛑 SHUTDOWN SEQUENCE INITIATED..."

# 1. KILL LOCAL RVIZ AND TERMINATOR PROCESSES
echo "Closing local ROS 2 processes..."
pkill -INT -f "rviz2"

# 2. REMOTE KILL ON NUC (Vision and Controller)
echo "Sending Remote SIGINT to NUC (@$NUC_IP)..."
sshpass -p "$NUC_PASS" ssh -o ConnectTimeout=2 $NUC_USER@$NUC_IP "pkill -INT -u $NUC_USER ros2; pkill -INT -u $NUC_USER python3" 2>/dev/null &

# 3. REMOTE KILL ON ELEPHANT BOARD (The nested SSH hop)
# We log into the NUC to tell it to tell the Elephant board to stop.
echo "Sending Remote SIGINT to Elephant Board (@$ELEPHANT_IP)..."
sshpass -p "$NUC_PASS" ssh -o ConnectTimeout=2 $NUC_USER@$NUC_IP "sshpass -p '$ELEPHANT_PASS' ssh -o StrictHostKeyChecking=no $ELEPHANT_IP 'pkill -INT ros2'" 2>/dev/null &

# 4. GRACE PERIOD
echo "⏳ Waiting 10 seconds for all nodes to release DDS participants..."
sleep 10

# 5. FINAL CLEANUP: CLOSE TERMINALS
echo "🧹 Closing all Rover 12 Terminator windows..."
pkill -f "terminator -T Rover 12"

echo "✅ System clean. Ready for next run."
