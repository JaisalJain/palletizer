# One pick-and-place cycle, published by the orchestrator for the metrics logger.
builtin_interfaces/Time stamp
uint32 pallet_index
uint32 box_index
DetectedBox detected
float64 measured_mass     # from the F/T sensor [kg]
Placement placement
bool success
string outcome            # placed | rejected_grasp | rejected_no_fit | detect_failed | ...
uint32 detect_attempts
uint32 grasp_attempts
float64 cycle_time        # box ready at pick eye -> placed [s]
string sim_model_name     # SIM ONLY (suction plugin), for evaluation joins
