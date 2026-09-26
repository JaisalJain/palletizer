"""Access to cell_config.yaml, the single source of truth for the cell layout."""

import os

import yaml

# Sections only the simulated plant may read (box and pallet sizes). The robot's
# decision-making nodes (perception, packing, motion, orchestrator) load the config with
# role='robot', which removes them: box and pallet dimensions must be MEASURED.
PLANT_ONLY = ('spawner', 'exchanger')


def default_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('palletizer_bringup'), 'config', 'cell_config.yaml')


def load(path=None, role='plant'):
    """role='plant' (simulated equipment, evaluation, tests): everything.
    role='robot' (decision-making): without the PLANT_ONLY sections."""
    with open(path or default_path()) as f:
        cell = yaml.safe_load(f)
    if role == 'robot':
        for k in PLANT_ONLY:
            cell.pop(k, None)
    elif role != 'plant':
        raise ValueError(f'unknown role {role!r}')
    return cell
