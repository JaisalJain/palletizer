# Ground-truth spec of a spawned box (published on /sim/box_specs).
# EVALUATION ONLY: never an input to perception, planning or control.
builtin_interfaces/Time stamp
string name
float64 size_x            # model box size along its own x/y/z axes [m]
float64 size_y
float64 size_z
float64 mass              # [kg]
float64 spawn_yaw         # [rad]
string sku                # SKU key from the spawner catalog / order ("" = mixed mode)
