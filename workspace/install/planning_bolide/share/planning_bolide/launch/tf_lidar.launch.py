from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_broadcaster',
            arguments=["0.165", "0", "0", "0", "0", "0", "base_link", "lidar_frame", "100"],
            output='screen',
            respawn=True
        ),
    ])
