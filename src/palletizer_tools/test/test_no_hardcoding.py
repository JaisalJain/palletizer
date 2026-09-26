#!/usr/bin/env python3
"""No-hardcoding check (v2): the robot's decision-making never reads box or pallet dimensions
or the pallet pose from the config. Runs without ROS:  python -m pytest <this file>

1. runtime: cell_config.load(role='robot') has no box / pallet sizes (the SKU catalog and the
   pallet types live in plant-only sections), and the config has no pallet size/pose at all
2. static: every decision-making source file loads the config with role='robot' only, never
   names a plant-only section, and the C++ orchestrator reads no geometry parameters
The runtime counterpart (a SKU and a pallet type that appear nowhere in the config) is the
`sku_runs --name unknown` run, see docs/RESULTS.md.
"""

import os
import re
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
CONFIG = os.path.join(SRC, 'palletizer_bringup', 'config', 'cell_config.yaml')
sys.path.insert(0, os.path.join(SRC, 'palletizer_tools'))

from palletizer_tools import cell_config  # noqa: E402

# Perception, planning, motion, orchestration (evaluation tools, tests and the simulated plant
# -- spawner, conveyor, pallet exchanger, sim_ops -- are not decision-making).
DECISION = [
    'palletizer_perception/palletizer_perception/box_perception.py',
    'palletizer_perception/palletizer_perception/detector.py',
    'palletizer_perception/palletizer_perception/camera_model.py',
    'palletizer_perception/palletizer_perception/pallet_perception.py',
    'palletizer_perception/palletizer_perception/pallet_detector.py',
    'palletizer_packing/palletizer_packing/packing_node.py',
    'palletizer_packing/palletizer_packing/core/pallet.py',
    'palletizer_packing/palletizer_packing/core/planners.py',
    'palletizer_packing/palletizer_packing/core/pattern.py',
    'palletizer_packing/palletizer_packing/core/perceived.py',
    'palletizer_packing/palletizer_packing/core/reach.py',
    'palletizer_packing/palletizer_packing/core/rules.py',
    'palletizer_packing/palletizer_packing/core/physics.py',
    'palletizer_motion/palletizer_motion/motion_server.py',
    'palletizer_motion/palletizer_motion/moveit_client.py',
    'palletizer_motion/palletizer_motion/scene.py',
    'palletizer_orchestrator/palletizer_orchestrator/status_overlay.py',
    'palletizer_tools/palletizer_tools/pallet_frame.py',
    'palletizer_tools/palletizer_tools/ur_ik.py',
]
CPP = ['palletizer_orchestrator/src/orchestrator.cpp']
FORBIDDEN = [r"[\w\]\)]\[\s*'(spawner|exchanger|pallet)'\s*\]",
             r"'catalog'", r"'pallet_types'", r"size_min", r"size_max", r"'mixed'\]"]


@pytest.fixture(scope='module')
def robot_cfg():
    return cell_config.load(CONFIG, role='robot')


def test_robot_view_has_no_sizes(robot_cfg):
    for k in cell_config.PLANT_ONLY:
        assert k not in robot_cfg
    assert 'pallet' not in robot_cfg, 'v1 pallet section (size / centre) must be gone'

    def walk(d, path=''):
        if isinstance(d, dict):
            for k, v in d.items():
                yield from walk(v, f'{path}.{k}')
        else:
            yield path, d
    keys = [p for p, _ in walk(robot_cfg)]
    for bad in ('catalog', 'pallet_types', 'size_min', 'size_max', 'center_xy.size'):
        assert not any(bad in k for k in keys), bad
    # the station is an AREA plus limits, no pallet dims
    assert set(robot_cfg['pallet_station']) <= {'center_xy', 'area', 'max_stack_height',
                                                'max_deck_z'}


def test_plant_view_keeps_the_catalog():
    plant = cell_config.load(CONFIG)
    assert plant['spawner']['catalog'] and plant['exchanger']['pallet_types']


@pytest.mark.parametrize('rel', DECISION)
def test_decision_code_reads_no_plant_config(rel):
    src = open(os.path.join(SRC, rel)).read()
    code = '\n'.join(line.split('#')[0] for line in src.splitlines())   # ignore comments
    for pat in FORBIDDEN:
        assert not re.search(pat, code), f'{rel}: {pat}'
    for m in re.finditer(r'cell_config\.load\(([^)]*)\)', code):
        assert "role='robot'" in m.group(1), f'{rel}: cell_config.load({m.group(1)})'


@pytest.mark.parametrize('rel', CPP)
def test_orchestrator_has_no_geometry(rel):
    code = open(os.path.join(SRC, rel)).read()
    assert 'cell_config' not in code
    for p in ('pallet_area', 'pallet_top', '"usable_height"', 'size_x"', '0.144', '1.05'):
        assert p not in code, f'{rel}: {p}'


def test_launch_passes_no_sizes_to_decision_nodes():
    src = open(os.path.join(SRC, 'palletizer_bringup', 'launch', 'palletizer.launch.py')).read()
    assert "cell['pallet']" not in src
    assert re.findall(r"cell\['(\w+)'\]", src) == ['spawner']      # only the spawn rate
