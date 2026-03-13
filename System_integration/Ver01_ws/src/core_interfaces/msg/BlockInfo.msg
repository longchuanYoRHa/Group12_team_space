# BlockInfo.msg
# TODO: Define block information, including color, position, and confidence

string color          # Block color: "red", "green", "blue", "yellow"
geometry_msgs/Point position  # 3D position of the block in camera_link frame (mm)
float64 confidence    # Detection confidence [0.0, 1.0]
float64 distance      # Distance from the rover (m)
string frame_id       # Coordinate frame

