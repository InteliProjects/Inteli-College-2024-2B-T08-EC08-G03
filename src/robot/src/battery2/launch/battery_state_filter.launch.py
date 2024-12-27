from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='battery2',
            executable='battery_state_filter',
            name='battery_state_filter',
            output='screen'
        )
    ])