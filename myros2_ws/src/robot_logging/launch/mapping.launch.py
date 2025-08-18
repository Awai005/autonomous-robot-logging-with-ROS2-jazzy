import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Point to your SOURCE yaml directly (avoids stale installed copy).
    slam_params_default = os.path.join(
        os.getcwd(),
        "src", "robot_logging", "config", "slam_toolbox.yaml"
    )
    rviz_mapping = os.path.join(
        os.getcwd(),
        "src", "robot_logging", "rviz", "mapping.rviz"
    )
    slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=slam_params_default,
        description="Full path to slam_toolbox param file"
    )

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz", default_value="true",
        description="Whether to start RViz2"
    )

    # SLAM node (lifecycle)
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[LaunchConfiguration("slam_params_file")],
    )

    # Map saver (lifecycle)
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Lifecycle manager controls BOTH nodes
    lifecycle_mgr = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapping",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "bond_timeout": 0.0,
            "node_names": ["slam_toolbox", "map_saver_server"],
        }],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[rviz_mapping],
    )

    return LaunchDescription([
        slam_params_arg,
        start_rviz_arg,
        slam_node,
        map_saver,
        lifecycle_mgr,
        rviz,
    ])
