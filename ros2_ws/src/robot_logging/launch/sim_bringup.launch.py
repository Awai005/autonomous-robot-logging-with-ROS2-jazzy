import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_logging')
    urdf_xacro = os.path.join(pkg, 'urdf', 'skid_steer.urdf.xacro')
    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')

    # ✅ Make robot_description a proper string (fixes your YAML error)
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_xacro]),
        value_type=str
    )

    scan_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
        # Bidirectional bridge; Gazebo <-> ROS 2
        '/world/empty/model/skid_steer_bot/link/lidar_link/sensor/lidar/scan'
        '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    # 1) Start Gazebo (Harmonic) empty world
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # 2) Publish /robot_description (and TF) with sim time
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}],
        output='screen',
    )

    # 3) Spawn the robot from /robot_description
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'skid_steer_bot', '-topic', 'robot_description'],
        output='screen'
    )

    # 4) Start controllers (JSB first, then diff drive)
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml
        ],
        output='screen'
    )

    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
    )

    return LaunchDescription([
        set_plugin_path, gz, rsp, spawn, jsb_spawner, diff_spawner, clock_bridge, scan_bridge])

