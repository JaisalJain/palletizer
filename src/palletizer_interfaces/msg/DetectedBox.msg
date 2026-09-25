# A box measured by the overhead camera.
std_msgs/Header header    # frame_id = world
uint32 id
float64 length            # top-face long side [m] (length >= width)
float64 width             # top-face short side [m]
float64 height            # top surface - belt top [m]
geometry_msgs/Pose top_center   # grasp point: top-face center, orientation = yaw about z
float64 yaw               # direction of the length axis in world, in (-pi/2, pi/2]
float64 confidence        # 0..1
