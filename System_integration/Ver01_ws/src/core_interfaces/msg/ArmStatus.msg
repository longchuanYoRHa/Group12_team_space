# ArmStatus.msg  
# Robotic arm status feedback (published by arm_client)  

string state           # Robotic arm state machine status  
bool is_holding        # Whether a block is currently being held  
bool is_idle           # Whether the arm is idle  
bool has_error         # Whether there is an error  
string gripper_status  # Gripper status  
float64[] joint_angles # Current joint angles (6 joints, units: radians)  
