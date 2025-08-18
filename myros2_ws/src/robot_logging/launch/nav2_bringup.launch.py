import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory("robot_logging")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg, "config", "nav2_params.yaml"),
        description="Full path to Nav2 params file"
    )
    rviz_mapping = os.path.join(
        os.getcwd(),
        "src", "robot_logging", "rviz", "mapping.rviz"
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz", default_value="true",
        description="Start RViz2 with Nav2 panels"
    )

    # Core Nav2 servers
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[("cmd_vel", "/nav2/cmd_vel")],
    )

    behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    smoother = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    # Lifecycle manager to automatically configure/activate everything
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "bond_timeout": 0.0,
            "node_names": [
                "map_server",
                "amcl",
                "planner_server",
                "controller_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "smoother_server",
            ],
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
        params_arg, start_rviz_arg,
        map_server, amcl, planner, controller, behavior,
        bt_navigator, waypoint_follower, smoother,
        lifecycle_manager, rviz
    ])
