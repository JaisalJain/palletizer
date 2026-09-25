builtin_interfaces/Time stamp
uint32 pallet_index
string planner
uint32 boxes
float64 box_volume        # sum of placed box volumes [m^3]
float64 fill_percent      # box_volume / (pallet area * usable stack height) * 100
float64 max_height        # above pallet top [m]
float64 duration          # [s]
string end_reason         # full | stopped | box_limit
