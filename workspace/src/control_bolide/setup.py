import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control_bolide'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtle',
    maintainer_email='turtle@todo.todo',
    description='Controle bolide',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_controller = control_bolide.ackermann_controller:main',
            'calibration_direction_copy = control_bolide.calibration_direction_copy:main',
            'calibration_direction = control_bolide.calibration_direction:main',
            'cmd_vel_vizu = control_bolide.cmd_vel_vizu:main',
            'obstacle_detection = control_bolide.obstacle_detection:main',
            'speed_direction_controller = control_bolide.speed_direction_controller:main',
            'stanley_controller = control_bolide.stanley_controller:main',
        ],
    },
)
