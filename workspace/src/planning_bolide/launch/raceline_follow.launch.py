from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Récupérer les chemins des fichiers launch inclus
    perception_launch = os.path.join(
        get_package_share_directory('perception_bolide'),
        'launch',
        'perception.launch.py'
    )

    localize_launch = os.path.join(
        get_package_share_directory('particle_filter'),
        'launch',
        'localize.launch.py'
    )

    stanley_launch = os.path.join(
        get_package_share_directory('control_bolide'),
        'launch',
        'stanley_launch.py'
    )

    return LaunchDescription([
        # Inclure d'autres fichiers de lancement
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localize_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stanley_launch)
        ),

        # Lancer le nœud speed_direction_control
        Node(
            package='control_bolide',
            executable='speed_direction_controller.py',
            name='speed_direction_controller',
            output='screen'
        ),

        # Lancer le nœud odom_fork_imu
        Node(
            package='perception_bolide',
            executable='odom_fork_imu.py',
            name='odom_fork_imu'
        ),

        # Lancer le static_transform_publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gmapping_broadcaster',
            arguments=["0.165", "0", "0", "0", "0", "0", "base_link", "lidar_frame", "100"]
        )
    ])
