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

    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a2m12.launch.py'
    )

    slam_karto_launch = os.path.join(
        get_package_share_directory('slam_karto_g2o'),
        'launch',
        'karto_slam.launch.py'
    )

    ekf_params = os.path.join(
        get_package_share_directory('planning_bolide'),
        'config',
        'ekf_params.yaml'
    )

    return LaunchDescription([
        # Inclure d'autres fichiers de lancement
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_karto_launch)
        ),

        # Lancer le contrôleur Ackermann
        Node(
            package='control_bolide',
            executable='ackermann_controller.py',
            name='ackermann_controller',
            output='screen',
            parameters=[{'u2d2_topic': '/dev/ttyUSB1'}],
            respawn=True
        ),

        # Lancer le filtre EKF
        Node(
            package='robot_localization',
            executable='ekf_localization_node',
            name='ekf_odom',
            output='screen',
            parameters=[ekf_params],
            remappings=[('odometry/filtered', 'odom_filtered')],
            respawn=True
        ),

        # Lancer le static_transform_publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_broadcaster',
            arguments=["0.165", "0", "0", "0", "0", "0", "base_link", "lidar_frame", "100"],
            respawn=True
        ),
    ])
