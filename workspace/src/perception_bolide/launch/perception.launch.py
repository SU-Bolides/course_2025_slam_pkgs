import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os

def generate_launch_description():
    # Get the package share directory
    # perception_bolide_share_dir = os.path.join(
    #     os.getenv('COLCON_PREFIX_PATH', '/opt/ros/jazzy'), 'share', 'perception_bolide', 'launch'
    # )

    return LaunchDescription([
        # Include main_publisher.launch.py
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(perception_bolide_share_dir, 'main_publisher.launch.py')
        #     )
        # ),

        # # Include process.launch.py
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(perception_bolide_share_dir, 'process.launch.py')
        #     )
        # ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'serial_baudrate': 256000},
                {'frame_id': 'laser_frame'},
                {'inverted': False},
                {'angle_compensate': True},
                {'scan_mode': 'Express'}
            ],
            respawn=True
        ),

        # Node for stm32_publisher.py
        Node(
            package='perception_bolide',
            executable='stm32_publisher',
            name='stm32_publisher',
            output='screen',
            respawn=True
        ),
        Node(
            package='perception_bolide',
            executable='lidar_process',
            name='lidar_process',
            output='screen',
            respawn=True
        ),
    ])