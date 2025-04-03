import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'planning_bolide'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS Course Project Group',
    maintainer_email='baptiste.braun.delvoye@gmail.com',
    description='Planning Bolide package contains everything about moving the car.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = planning_bolide.teleop_keyboard:main',
            'speed_direction_controller = planning_bolide.speed_direction_controller:main',
        ],
    },
)
