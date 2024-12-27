from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talker_listener',
            executable='talker_node',
            name='talker'
        ),
        Node(
            package='talker_listener',
            executable='listener_node',
            name='listener'
        ),
    ])
