"""Show the arm in RViz2 with a joint slider GUI (no simulation)."""
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('arm_description')
    xacro_file = PathJoinSubstitution([share, 'urdf', 'arm.urdf.xacro'])
    rviz_config = PathJoinSubstitution([share, 'rviz', 'arm.rviz'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file, ' sim:=false']),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
