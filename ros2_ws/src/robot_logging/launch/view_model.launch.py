from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('robot_logging')
    urdf_xacro = os.path.join(pkg, 'urdf', 'skid_steer.urdf.xacro')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_xacro]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    jsp = Node(  # use GUI for easy motion; swap to joint_state_publisher if you prefer
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([rsp, jsp, rviz])
