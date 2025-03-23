from setuptools import find_packages, setup

package_name = 'perception_bolide'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='voiture',
    maintainer_email='voiture@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_publisher = perception_bolide.stm32_publisher:main',
            'rear_ranges_publisher = perception_bolide.rear_ranges_publisher:main',
            'lidar_publisher = perception_bolide.lidar_publisher:main',
            'imu_publisher = perception_bolide.imu_publisher:main',
            'fork_publisher = perception_bolide.fork_publisher:main',
            'camera_publisher = perception_bolide.camera_publisher:main',
            'lidar_vizu = perception_bolide.lidar_vizu:main',
            'odom_fork_imu = perception_bolide.odom_fork_imu:main',
        ],
    },
)
