import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('robot_logging')
    params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')

    # Use the exact ROS topic you bridged from Gazebo; adjust if yours differs
    gz_scan_topic = '/world/empty/model/skid_steer_bot/link/lidar_link/sensor/lidar/scan'

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params],
        remappings=[('/scan', gz_scan_topic)]
    )

    return LaunchDescription([slam])
