from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node', output='screen'),
        Node(package='joy_teleop', executable='joy_teleop', output='screen'),
        Node(package='create_driver', executable='create_driver', output='screen'),
    ])