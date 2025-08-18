import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('robot_logging')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        description='Full path to params file'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    simple_nav = Node(
        package='robot_logging',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen',
    )

    return LaunchDescription([
        params_arg,
        map_server,
        amcl,
        simple_nav
    ])