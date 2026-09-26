# Packing model summary (packing_node), returned by plan / commit / set_pallet.
string mode               # mixed | single_sku
bool has_pallet
float64 pallet_u          # measured deck size [m]
float64 pallet_v
float64 usable_height     # max box-top height above the deck (reachability map, this pallet)
bool has_sku
float64 sku_length        # measured SKU dims [m]
float64 sku_width
float64 sku_height
uint32 per_layer_a        # boxes in layer pattern A / B
uint32 per_layer_b
uint32 optimum_per_layer  # best count for the deck ignoring reachability
uint32 layers
uint32 pattern_count      # boxes in the whole pattern (0 in mixed mode)
string pattern_note       # interlock / column stack, dropped slots, ...
uint32 placed
uint32 layer              # 1-based layer of the next box
float64 fill_percent      # placed box volume / (deck area x usable height)
float64 max_height        # [m] above the deck
uint32 disagreements      # perception vs placement record, this pallet
