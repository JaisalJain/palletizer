# Pallet state (orchestrator), /pallet/status, transient local.
uint8 EMPTY=0
uint8 IN_PROGRESS=1
uint8 FULL=2
uint8 EXCHANGING=3
uint8 READY=4
builtin_interfaces/Time stamp
uint32 pallet_id
uint8 state
string state_name
string mode               # mixed | single_sku
float64 pallet_length     # measured deck dims [m] (u, v)
float64 pallet_width
float64 sku_length        # measured SKU dims [m] (0: none / mixed)
float64 sku_width
float64 sku_height
uint32 boxes_placed
uint32 boxes_in_pattern   # 0 in mixed mode
uint32 layer
uint32 layers
float64 fill_percent
string reason             # PATTERN_COMPLETE | HEIGHT_LIMIT | NO_FEASIBLE_PLACEMENT | SKU_CHANGE | OPERATOR | ''
uint32 shift_events       # stack events detected by pallet_perception on this pallet
