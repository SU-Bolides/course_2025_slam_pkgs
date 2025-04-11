from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planning_bolide',
            executable='teleop_keyboard.py',
            name='teleop',
            output='screen',
            prefix='sudo -E'  # Remplace `launch-prefix`
        ),
    ])
