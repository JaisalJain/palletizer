"""Location of the colcon workspace (results/, docs/, src/) without machine-specific paths."""

import os


def root():
    """$PALLETIZER_WS if set, else the workspace this overlay was built from (the directory
    above install/ that holds src/palletizer_bringup), else the current directory."""
    env = os.environ.get('PALLETIZER_WS')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.realpath(get_package_share_directory('palletizer_bringup'))
    except Exception:
        d = None
    while d and d != os.path.dirname(d):
        if os.path.isdir(os.path.join(d, 'src', 'palletizer_bringup')):
            return d
        d = os.path.dirname(d)
    return os.getcwd()


def path(*parts):
    return os.path.join(root(), *parts)
