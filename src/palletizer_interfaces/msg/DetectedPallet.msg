# A pallet measured by the pallet camera (deck top surface), world frame.
# Pallet frame (packing): origin = deck corner nearest the robot on the -u side, v = pallet
# axis pointing away from the robot, u = v rotated by +90 deg about z, z = height above the deck.
std_msgs/Header header
bool valid                # false: no pallet at the station
float64 size_u            # deck extent along u [m]
float64 size_v            # deck extent along v (away from the robot) [m]
float64 deck_z            # world z of the deck top surface [m]
geometry_msgs/Pose center # deck-top centre, orientation = yaw of the u axis
geometry_msgs/Pose origin # pallet-frame origin, orientation = yaw of the u axis
float64 yaw               # world yaw of the u axis [rad]
float64 confidence        # 0..1
