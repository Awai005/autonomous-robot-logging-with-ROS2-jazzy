import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_logging')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params = LaunchConfiguration(
        'slam_params',
        default=os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan')]  # your bridge already publishes /scan
    )

    rviz_cfg = os.path.join(pkg_share, 'config', 'mapping.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_params', default_value=os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')),
        slam, rviz
    ])
