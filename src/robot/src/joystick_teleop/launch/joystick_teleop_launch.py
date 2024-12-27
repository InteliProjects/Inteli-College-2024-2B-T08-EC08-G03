from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("joystick_teleop"),
        "config",
        "joystick_teleop_params.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="joystick_teleop",
                executable="joystick_teleop_node",
                name="teleop_joystick",
                parameters=[config_file],
            ),
            Node(package="joy", executable="joy_node", name="joy_node"),
        ]
    )
