from setuptools import find_packages, setup

package_name = 'palletizer_packing'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jaisal K Jain',
    maintainer_email='jaisaljain345@gmail.com',
    description='Pallet packing core + ROS wrapper.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'packing_node = palletizer_packing.packing_node:main',
            'pattern_report = palletizer_packing.pattern_report:main',
        ],
    },
)
