import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory("robot_logging")

    world_default = os.path.join(pkg, "worlds", "teranova.sdf")
    model_default = os.path.join(pkg, "urdf", "skid_steer.urdf.xacro")
    slam_yaml = os.path.join(pkg, "config", "slam.yaml")
    rviz_cfg = os.path.join(pkg, "rviz", "slam_view.rviz")
    controllers_yaml = os.path.join(pkg, "config", "controllers.yaml")

    world_arg = DeclareLaunchArgument("world", default_value=world_default)
    model_arg = DeclareLaunchArgument("model", default_value=model_default)

    # Let Gazebo find local resources (worlds, meshes, etc.)
    gz_res = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(pkg).parent.resolve())
    )

    # Robot description from xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Robot State Publisher (TF base_link -> sensors)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # Gazebo Harmonic
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": LaunchConfiguration("world")
        }.items()
    )

    # Spawn the robot from /robot_description
    spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "skid_steer_bot"],
        output="screen",
    )

    # Bridges: /clock, /scan (Gazebo -> ROS2)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[{'use_sim_time': True}],
        output="screen",
    )

    # ros2_control controllers
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager',
                   '--param-file', controllers_yaml],
        output='screen'
    )

    # SLAM Toolbox (online mapping)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_yaml
        }.items()
    )

    # RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        world_arg, model_arg, gz_res,
        rsp, gz, spawner, bridge, jsb, diff,
        slam, rviz
    ])
