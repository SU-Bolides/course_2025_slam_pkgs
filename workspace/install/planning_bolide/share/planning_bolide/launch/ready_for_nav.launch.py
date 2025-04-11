from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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

    sllidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a2m12_launch .py'
    )

    localize_launch = os.path.join(
        get_package_share_directory('particle_filter'),
        'launch',
        'localize_launch.py'
    )

    ekf_params = os.path.join(
        get_package_share_directory('planning_bolide'),
        'config',
        'ekf_params.yaml'
    )

    map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('perception_bolide'),
            'maps',
            'esclangon_loop_2.yaml'
        ),
        description='Carte utilisée pour la localisation'
    )

    return LaunchDescription([
        # Déclaration de la carte
        map_file,

        # Inclure d'autres fichiers de lancement
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localize_launch)
        ),

        # Lancer le contrôleur Ackermann
        Node(
            package='control_bolide',
            executable='ackermann_controller.py',
            name='ackermann_controller',
            output='screen',
            parameters=[{'u2d2_topic': '/dev/ttyUSB0'}],
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

        # Lancer le serveur de carte
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            arguments=[os.path.join(
                get_package_share_directory('perception_bolide'),
                'maps',
                'esclangon_loop_2.yaml'
            )],
            output='screen'
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
