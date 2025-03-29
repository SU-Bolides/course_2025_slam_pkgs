import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_bolide',
            executable='stm32_publisher',
            name='stm3_publisher',
            output='screen',
            respawn=True,
        )
    ])

if __name__ == '__main__':
    generate_launch_description()