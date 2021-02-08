import os
from glob import glob

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import ExecuteProcess


# Launch an emulator for testing


def generate_launch_description():
    xacro_path = os.path.join('share', 'create_description', 'urdf/create_2.urdf.xacro')
    print(xacro_path)
    roomba_description_params = [
        {'robot_description': Command(['xacro',' ', xacro_path])}
    ]

    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             node_name='robot_state_publisher',
             parameters=roomba_description_params,
             output='screen'
        ),
    ])