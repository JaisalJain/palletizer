"""Access to cell_config.yaml, the single source of truth for the cell layout."""

import os

import yaml


def default_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('palletizer_bringup'), 'config', 'cell_config.yaml')


def load(path=None):
    with open(path or default_path()) as f:
        return yaml.safe_load(f)
