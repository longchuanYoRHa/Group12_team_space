# MissionStatus.msg  
# TODO: Determine overall mission status, published by mission_controller  

string state              # Current mission phase (see constants below)  
string current_color      # Current target color  
int32 blocks_picked       # Number of blocks picked  
int32 blocks_placed       # Number of blocks placed  
int32 total_targets       # Total number of targets  
string arm_state          # Arm state  
string rover_state        # Rover state  
string vision_state       # Vision state  
float64 elapsed_time      # Mission elapsed time (seconds)  
string message            # Additional information  

# Mission phase constants  
string IDLE=idle  
string INITIALIZING=initializing  
string SEARCHING_BLOCK=searching_block  
string NAVIGATING_TO_BLOCK=navigating_to_block  
string ALIGNING_TO_BLOCK=aligning_to_block  
string PICKING_BLOCK=picking_block  
string SEARCHING_BIN=searching_bin  
string NAVIGATING_TO_BIN=navigating_to_bin  
string ALIGNING_TO_BIN=aligning_to_bin  
string PLACING_BLOCK=placing_block  
string MISSION_COMPLETE=mission_complete  
string ERROR=error  
string PAUSED=paused  
