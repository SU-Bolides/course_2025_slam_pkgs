from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Déclaration des arguments
        DeclareLaunchArgument('tf_map_scanmatch_transform_frame_name', default_value='scanmatcher_frame', description='Frame name for the scan matcher transform'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame name'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame name'),
        DeclareLaunchArgument('pub_map_odom_transform', default_value='true', description='Publish map to odom transform'),
        DeclareLaunchArgument('scan_subscriber_queue_size', default_value='10', description='Queue size for the scan topic'),
        DeclareLaunchArgument('scan_topic', default_value='lidar_data', description='The scan topic name'),
        DeclareLaunchArgument('map_size', default_value='2048', description='Size of the map'),
        
        # Node Hector Mapping
        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[
                {'map_frame': 'map'},
                {'base_frame': LaunchConfiguration('base_frame')},
                {'odom_frame': LaunchConfiguration('odom_frame')},
                {'tf_map_scanmatch_transform_frame_name': 'base_link'},
                {'use_tf_scan_transformation': False},
                {'use_tf_pose_start_estimate': False},
                {'pub_map_odom_transform': LaunchConfiguration('pub_map_odom_transform')},
                {'map_resolution': 0.080},
                {'map_size': LaunchConfiguration('map_size')},
                {'map_start_x': 0.5},
                {'map_start_y': 0.5},
                {'map_multi_res_levels': 2},
                {'update_factor_free': 0.4},
                {'update_factor_occupied': 0.9},
                {'map_update_distance_thresh': 0.4},
                {'map_update_angle_thresh': 0.06},
                {'laser_z_min_value': -1.0},
                {'laser_z_max_value': 1.0},
                {'advertise_map_service': True},
                {'scan_subscriber_queue_size': LaunchConfiguration('scan_subscriber_queue_size')},
                {'scan_topic': LaunchConfiguration('scan_topic')},
                {'tf_map_scanmatch_transform_frame_name': LaunchConfiguration('tf_map_scanmatch_transform_frame_name')}
            ]
        ),
        
        # Publisher pour static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_nav_broadcaster',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom', '100']
        )
    ])
