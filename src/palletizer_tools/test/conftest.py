# These are integration scripts that need a running simulation (see README);
# keep plain pytest / colcon test from collecting them.
collect_ignore_glob = ["test_*.py"]
