import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_bolide',
            executable='speed_direction_controller',
            name='speed_direction_controller',
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\" bash -c"],
            shell = True
        )
    ])

if __name__ == '__main__':
    generate_launch_description()