import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Déclare un argument pour le topic scan (permet de spécifier à la commande de lancement)
        DeclareLaunchArgument('scan_topic', default_value='lidar_data', description='LIDAR scan topic'),

        # Noeud gmapping pour SLAM
        Node(
            package='gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[{
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_update_interval': 1.0,
                'maxUrange': 6.0,
                'maxRange': 8.0,
                'sigma': 0.05,
                'kernelSize': 1,
                'lstep': 0.05,
                'astep': 0.05,
                'iterations': 5,
                'lsigma': 0.075,
                'ogain': 3.0,
                'lskip': 0,
                'minimumScore': 100,
                'srr': 0.01,
                'srt': 0.02,
                'str': 0.01,
                'stt': 0.02,
                'linearUpdate': 0.5,
                'angularUpdate': 0.436,
                'temporalUpdate': -1.0,
                'resampleThreshold': 0.5,
                'particles': 80,
                'xmin': -1.0,
                'ymin': -1.0,
                'xmax': 1.0,
                'ymax': 1.0,
                'delta': 0.02,
                'llsamplerange': 0.01,
                'llsamplestep': 0.01,
                'lasamplerange': 0.005,
                'lasamplestep': 0.005
            }],
            remappings=[('/scan', LaunchConfiguration('scan_topic'))]  # Utilisation de l'argument 'scan_topic'
        ),
        
        # Node to publish static transform between base_link and lidar_frame (needed for mapping)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='gmapping_broadcaster',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_frame', '100']
        # ),
    ])
