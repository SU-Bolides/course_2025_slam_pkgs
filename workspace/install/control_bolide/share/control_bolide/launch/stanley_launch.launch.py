import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_bolide',
            executable='stanley_controller',
            name='stanley_controller_node',
            output='screen',
            respawn=True,
            parameters=[
                {'odom_topic': '/pf/pos/odom'},
                {'waypoints_path': '/home/course_2025_slam_pkgs/workspace/course_2024_pkgs/control_bolide/racelines/esclangon_couloir_reverse.csv'},
                {'cmd_topic': '/cmd_vel'},
                {'K_E': 0.4},
                {'K_H': 0.4},
                {'K_V': 0.5},
                {'K_p_obstacle': 0.4},
                {'K_dh': 0.0},
                {'velocity_percentage': 1.0},
                {'steering_limit_deg': 15.3},
                {'brake_threshold_ms': 5.},
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()