import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node for lidar data publication
        Node(
            package='rplidar_ros',
            executable='rplidarNode',
            name='rplidarNode',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'serial_baudrate': 256000},
                {'frame_id': 'lidar_frame'},
                {'inverted': False},
                {'angle_compensate': True},
                {'scan_mode': 'Express'}
            ],
            respawn=True
        ),

        # Node for stm32_publisher.py
        Node(
            package='perception_bolide',
            executable='stm32_publisher.py',
            name='stm32_publisher',
            output='screen',
            respawn=True
        ),

        # Uncomment the following nodes if needed
        # Node(
        #     package='perception_bolide',
        #     executable='camera_publisher.py',
        #     name='camera_publisher',
        #     output='screen'
        # ),
        # Node(
        #     package='perception_bolide',
        #     executable='imu_publisher.py',
        #     name='imu_publisher',
        #     output='screen'
        # ),
        # Node(
        #     package='perception_bolide',
        #     executable='rear_ranges_publisher.py',
        #     name='rear_ranges_publisher',
        #     output='screen'
        # ),
        # Node(
        #     package='perception_bolide',
        #     executable='fork_publisher.py',
        #     name='optical_fork_publisher',
        #     output='screen'
        # ),
    ])