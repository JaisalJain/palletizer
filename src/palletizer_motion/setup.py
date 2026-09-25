from glob import glob

from setuptools import find_packages, setup

package_name = 'palletizer_motion'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/test', glob('test/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaisaljain',
    maintainer_email='jaisaljain345@gmail.com',
    description='MoveIt 2 pick and place for the palletizing cell.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motion_server = palletizer_motion.motion_server:main',
            'build_reachability_map = palletizer_motion.reachability:main',
        ],
    },
)
