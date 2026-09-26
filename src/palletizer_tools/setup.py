from glob import glob

from setuptools import find_packages, setup

package_name = 'palletizer_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/test', glob('test/*.py') + glob('test/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaisaljain',
    maintainer_email='jaisaljain345@gmail.com',
    description='Box spawner, conveyor controller and tests for the palletizing cell.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'box_spawner = palletizer_tools.box_spawner:main',
            'conveyor_controller = palletizer_tools.conveyor_controller:main',
            'cloud_relay = palletizer_tools.cloud_relay:main',
            'sim_ops = palletizer_tools.sim_ops:main',
            'pallet_exchanger = palletizer_tools.pallet_exchanger:main',
        ],
    },
)
