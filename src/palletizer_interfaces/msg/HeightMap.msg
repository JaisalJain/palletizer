# Perceived height map of the stack in the pallet frame (same grid as the packing core).
std_msgs/Header header
DetectedPallet pallet
float64 resolution        # cell size [m]
uint32 cols               # along u
uint32 rows               # along v
float32[] data            # row-major [row = v][col = u]: top height above the deck [m], NaN = not observed
uint32 placements         # placements verified on this pallet when the map was taken
