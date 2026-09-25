# Where a box goes on the pallet (world frame).
geometry_msgs/Pose top_center   # top-face center of the box when resting at the target
float64 yaw               # world yaw of the box LENGTH axis at the target
float64 size_x            # footprint along world x / y at the target (after rotation) [m]
float64 size_y
float64 height
float64 rest_z            # z of the supporting surface (bottom of the box) [m]
bool rotated              # true = length axis along pallet y
float64 score
string planner
float64 planning_time     # [s]
uint32 candidates_checked
uint32 physics_checks
