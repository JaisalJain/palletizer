# Placement measured by the pallet camera (change detection before / after the place).
std_msgs/Header header
uint32 pallet_id
uint32 index              # placement number on this pallet (0-based)
bool detected
Placement commanded
geometry_msgs/Pose measured_top   # measured top-face centre (world), orientation = yaw
float64 measured_yaw      # world yaw of the measured length axis [rad]
float64 measured_length   # measured top-face dims [m]
float64 measured_width
float64 err_xy            # horizontal error vs the commanded top-face centre [m]
float64 err_z             # vertical error [m]
float64 err_yaw           # [rad], modulo the box symmetry
string sim_model_name     # SIM ONLY: join key for the ground-truth evaluation
string message
