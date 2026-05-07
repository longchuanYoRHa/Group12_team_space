#!/bin/bash

# Configuration
NUC_IP="10.42.0.227"
NUC_USER="leo-rover-12"
NUC_PASS="team12"
ELEPHANT_IP="10.0.1.3"
ELEPHANT_PASS="trunk"

echo "🛑 SHUTDOWN SEQUENCE INITIATED..."

# 1. SEND SIGINT (Ctrl+C) TO ALL NODES
# We target the NUC and Elephant Board simultaneously
echo "📡 Sending SIGINT to NUC and Elephant Board..."
sshpass -p "$NUC_PASS" ssh -o ConnectTimeout=2 $NUC_USER@$NUC_IP "
    pkill -INT -u $NUC_USER ros2; 
    sshpass -p '$ELEPHANT_PASS' ssh -o StrictHostKeyChecking=no elephant@$ELEPHANT_IP 'pkill -INT ros2'
" 2>/dev/null &

# 2. LOCAL CLEANUP
pkill -INT rviz2

# 3. THE 10-SECOND GRACE PERIOD
# Essential for large Vision and Nav nodes to clear their queues
echo "⏳ Waiting 10 seconds for graceful node destruction..."
sleep 10

# 4. FINAL HARD CLEANUP & DAEMON RESET
# If anything is still hanging, this forces it closed and clears the discovery cache
echo "🧹 Performing final cleanup and flushing DDS cache..."
sshpass -p "$NUC_PASS" ssh -o ConnectTimeout=2 $NUC_USER@$NUC_IP "
    pkill -9 -u $NUC_USER ros2; 
    pkill -9 -u $NUC_USER python3;
    ros2 daemon stop;
    sshpass -p '$ELEPHANT_PASS' ssh -o StrictHostKeyChecking=no elephant@$ELEPHANT_IP 'pkill -9 ros2'
" 2>/dev/null

# Reset local daemon too
ros2 daemon stop && ros2 daemon start

# 5. CLOSE TERMINALS
echo "🪟 Closing Terminator windows..."
pkill -f "terminator -T Rover 12"

echo "✅ System clean. Ready for the next run."
