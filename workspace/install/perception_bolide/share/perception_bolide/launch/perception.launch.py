import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get the package share directory
    perception_bolide_share_dir = os.path.join(
        os.getenv('COLCON_PREFIX_PATH', '/opt/ros/jazzy'), 'share', 'perception_bolide', 'launch'
    )

    return LaunchDescription([
        # Include main_publisher.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_bolide_share_dir, 'main_publisher.launch.py')
            )
        ),

        # Include process.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perception_bolide_share_dir, 'process.launch.py')
            )
        ),
    ])