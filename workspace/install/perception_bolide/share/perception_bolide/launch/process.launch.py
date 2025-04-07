import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node for lidar_process.py
        Node(
            package='perception_bolide',
            executable='lidar_process.py',
            name='lidar_process',
            output='screen',
            respawn=True
        ),

        # Uncomment the following node if needed
        # Node(
        #     package='perception_bolide',
        #     executable='camera_info.py',
        #     name='camera_info',
        #     output='screen'
        # ),
    ])