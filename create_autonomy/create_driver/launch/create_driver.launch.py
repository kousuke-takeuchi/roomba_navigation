from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    return LaunchDescription([
        Node(package='create_driver', executable='create_driver', output='screen'),
    ])
