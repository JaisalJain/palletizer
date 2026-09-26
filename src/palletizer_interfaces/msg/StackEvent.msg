# Stack monitoring (pallet_perception): a placed box is no longer where it was placed.
std_msgs/Header header
uint32 pallet_id
uint32 index              # placement index of the box
string kind               # SHIFT (moved > threshold) | COLLAPSE (dropped / missing)
float64 shift             # estimated horizontal displacement [m]
float64 dz                # estimated vertical displacement [m]
geometry_msgs/Point position  # top-face centre where the box was placed (world)
string sim_model_name     # SIM ONLY: evaluation join key
string message
