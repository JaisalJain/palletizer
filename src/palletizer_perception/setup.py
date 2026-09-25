from glob import glob

from setuptools import find_packages, setup

package_name = 'palletizer_perception'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/test', glob('test/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaisaljain',
    maintainer_email='jaisaljain345@gmail.com',
    description='Overhead RGB-D box perception.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'box_perception = palletizer_perception.box_perception:main',
            'eval_perception = palletizer_perception.eval_perception:main',
        ],
    },
)
