#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Définir les chemins et arguments
    pkg_planning_bolide = FindPackageShare('planning_bolide')
    ekf_params_file = [pkg_planning_bolide, '/config/ekf_params.yaml']

    return LaunchDescription([
        # Nœud ekf_localization_node
        Node(
            package='robot_localization',
            executable='ekf_localization_node',
            name='ekf_odom',
            output='screen',  # Ajouté pour voir les logs, équivalent à ROS 1 par défaut
            parameters=[ekf_params_file],
            remappings=[
                ('odometry/filtered', 'odom_filtered'),
            ],
            arguments=['--ros-args', '--params-file', ekf_params_file]  # Alternative pour charger les paramètres
        ),
    ])