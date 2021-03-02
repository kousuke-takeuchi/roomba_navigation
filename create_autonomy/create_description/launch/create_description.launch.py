import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


# https://qiita.com/hakuturu583/items/7e3a278630422e17f0ba
share_dir_path = os.path.join(get_package_share_directory('create_description'))
xacro_path = os.path.join(share_dir_path, 'urdf', 'create_2.urdf.xacro')
urdf_path = os.path.join(share_dir_path, 'urdf', 'create_2.urdf')


def generate_launch_description():
    # xacroをロード
    doc = xacro.process_file(xacro_path)
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    # urdf_pathに対してurdfを書き出し
    with open(urdf_path, 'w+') as f:
      f.write(robot_desc)
      f.close()

    return LaunchDescription([
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             arguments=[urdf_path],
             output='screen'
        ),
    ])