builtin_interfaces/Time stamp
uint32 pallet_index
string planner
uint32 boxes
float64 box_volume        # sum of placed box volumes [m^3]
float64 fill_percent      # box_volume / (pallet area * usable stack height) * 100
float64 max_height        # above pallet top [m]
float64 duration          # [s]
string end_reason         # PATTERN_COMPLETE | HEIGHT_LIMIT | NO_FEASIBLE_PLACEMENT | SKU_CHANGE | OPERATOR
string mode               # mixed | single_sku
float64 pallet_length     # measured deck [m]
float64 pallet_width
float64 usable_height     # [m] above the deck
float64 sku_length        # measured SKU [m] (single_sku)
float64 sku_width
float64 sku_height
uint32 boxes_in_pattern
uint32 layers
uint32 shift_events       # detected by pallet_perception
