import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_logging = get_package_share_directory("robot_logging")

    controllers_yaml = os.path.join(robot_logging, 'config', 'controllers.yaml')

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        robot_logging, "urdf", "skid_steer.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(robot_logging).parent.resolve())
            ]
        )
    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model")
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments=[
                    ("gz_args", [" -v 4", " -r", " /home/tersoo/autonomous-robot-logging-with-ROS2-jazzy/myros2_ws/src/robot_logging/worlds/teranova.sdf"]
                    )
                ]
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "skid_steer_bot"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Gazebo -> ROS
            "lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
        parameters=[{'use_sim_time': True}],
        remappings=[
            # rename ROS-side topics so everything expects /scan and /points
            ('/lidar', '/scan'),
            ('/lidar/points', '/points'),
        ],
    )
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
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_tf_pub",
        arguments=["0", "0", "0", "0", "0", "0",
                   "base_link", "skid_steer_bot/base_link/gpu_lidar"],
        output="screen"
    )

    

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        jsb_spawner,
        diff_spawner,
        static_tf
    ])