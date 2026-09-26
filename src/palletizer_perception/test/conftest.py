# Integration scripts that need a running simulation (see README); keep plain pytest /
# colcon test from collecting them. test_pallet_detector.py is a plain unit test.
collect_ignore = ["test_perception_touching.py"]
